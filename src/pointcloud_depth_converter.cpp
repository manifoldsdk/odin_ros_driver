#include "pointcloud_depth_converter.hpp"
#include <cmath>
#include <iostream>

PointCloudToDepthConverter::PointCloudToDepthConverter(const CameraParams &params)
    : params_(params)
{
    initializeInternalParams();
    createDistortionMaps();
}

void PointCloudToDepthConverter::initializeInternalParams()
{
    scaled_width_ = static_cast<int>(params_.image_width / params_.scale);
    scaled_height_ = static_cast<int>(params_.image_height / params_.scale);

    K_ = Eigen::Matrix3d::Identity();
    K_(0, 0) = params_.A11;
    K_(0, 1) = params_.A12;
    K_(0, 2) = params_.u0;
    K_(1, 1) = params_.A22;
    K_(1, 2) = params_.v0;

    Kl_ = Eigen::Matrix3d::Identity();
    Kl_(0, 0) = params_.A11 / params_.scale;
    Kl_(0, 1) = params_.A12 / params_.scale;
    Kl_(0, 2) = params_.u0 / params_.scale;
    Kl_(1, 1) = params_.A22 / params_.scale;
    Kl_(1, 2) = params_.v0 / params_.scale;

    K_4x4_ = Eigen::Matrix4d::Identity();
    K_4x4_.block<3, 3>(0, 0) = Kl_;

    Kcl_ = K_4x4_ * params_.Tcl;
}

void PointCloudToDepthConverter::createDistortionMaps()
{
    map_x_ = cv::Mat::zeros(params_.image_height, params_.image_width, CV_32FC1);
    map_y_ = cv::Mat::zeros(params_.image_height, params_.image_width, CV_32FC1);


    for (int u = 0; u < params_.image_width; ++u)
    {
        for (int v = 0; v < params_.image_height; ++v)
        {
            double y = (v - params_.v0) / params_.A22;
            double x = (u - params_.u0 - params_.A12 * y) / params_.A11;
            
            double r = sqrt(x * x + y * y);
            double theta = atan(r);

            double theta_d = theta + params_.k2 * pow(theta, 2) + params_.k3 * pow(theta, 3) +
                                params_.k4 * pow(theta, 4) + params_.k5 * pow(theta, 5) +
                                params_.k6 * pow(theta, 6) + params_.k7 * pow(theta, 7);

            double x_distorted = x * (r / theta_d);
            double y_distorted = y * (r / theta_d);

            map_x_.at<float>(v, u) = static_cast<float>(x_distorted * params_.A11 + params_.A12 * y_distorted + params_.u0);
            map_y_.at<float>(v, u) = static_cast<float>(y_distorted * params_.A22 + params_.v0);
        }
    }

    inv_map_x_ = cv::Mat::zeros(params_.image_height, params_.image_width, CV_32FC1);
    inv_map_y_ = cv::Mat::zeros(params_.image_height, params_.image_width, CV_32FC1);
    for (int u = 0; u < params_.image_width; ++u)
    {
        for (int v = 0; v < params_.image_height; ++v)
        {
            double y = (v - params_.v0) / params_.A22;
            double x = (u - params_.u0 - params_.A12 * y) / params_.A11;
            
            double r = sqrt(x * x + y * y);
            double theta = atan(r);

            double theta_d = theta + params_.k2 * pow(theta, 2) + params_.k3 * pow(theta, 3) +
                                params_.k4 * pow(theta, 4) + params_.k5 * pow(theta, 5) +
                                params_.k6 * pow(theta, 6) + params_.k7 * pow(theta, 7);

            double x_distorted = x * (theta_d / r);
            double y_distorted = y * (theta_d / r);

            inv_map_x_.at<float>(v, u) = static_cast<float>(x_distorted * params_.A11 + params_.A12 * y_distorted + params_.u0);
            inv_map_y_.at<float>(v, u) = static_cast<float>(y_distorted * params_.A22 + params_.v0);
        }
    }
}

PointCloudToDepthConverter::ProcessResult PointCloudToDepthConverter::processCloudAndImage(
    const pcl::PointCloud<pcl::PointXYZ> &cloud,
    const cv::Mat &image)
{
    ProcessResult result;
    result.success = false;

    auto validation_result = validateInputs(cloud, image);
    if (!validation_result.first)
    {
        result.error_message = validation_result.second;
        return result;
    }

    try
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_in_cam;
        pcl::transformPointCloud(cloud, cloud_in_cam, Kcl_);

        cv::Mat depth_img = projectCloudToDepth(cloud_in_cam);

        cv::Mat processed_depth = postProcessDepthImage(depth_img);

        pcl::PointCloud<pcl::PointXYZRGB> colored_cloud = generateColoredCloud(processed_depth, image);

        result.depth_image = processed_depth;
        result.colored_cloud = colored_cloud;
        result.success = true;
    }
    catch (const std::exception &e)
    {
        result.error_message = std::string("Processing error: ") + e.what();
    }

    return result;
}

cv::Mat PointCloudToDepthConverter::projectCloudToDepth(const pcl::PointCloud<pcl::PointXYZ> &cloud_in_cam)
{
    cv::Mat depth_img = cv::Mat::zeros(scaled_height_, scaled_width_, CV_32FC1);

    for (const auto &camera_point : cloud_in_cam)
    {
        if (camera_point.z <= 0)
            continue; 

        int u = static_cast<int>(std::round(camera_point.x / camera_point.z));
        int v = static_cast<int>(std::round(camera_point.y / camera_point.z));

        if (u >= 0 && u < scaled_width_ && v >= 0 && v < scaled_height_)
        {
            depth_img.at<float>(v, u) = static_cast<float>(camera_point.z);

            for (int du = -1; du <= 1; ++du)
            {
                for (int dv = -1; dv <= 1; ++dv)
                {
                    int nu = u + du;
                    int nv = v + dv;
                    if (nu >= 0 && nu < scaled_width_ && nv >= 0 && nv < scaled_height_)
                    {
                        if (depth_img.at<float>(nv, nu) == 0.0f)
                        {
                            depth_img.at<float>(nv, nu) = static_cast<float>(camera_point.z);
                        }
                    }
                }
            }
        }
    }

    return depth_img;
}

cv::Mat PointCloudToDepthConverter::postProcessDepthImage(const cv::Mat &depth_img)
{
    cv::Mat depth_img_upsampled;
    cv::resize(depth_img, depth_img_upsampled,
               cv::Size(params_.image_width, params_.image_height), 0, 0, cv::INTER_LINEAR);// 双线性插值上采样

    cv::Mat grad_x, grad_y, grad_magnitude;
    cv::Sobel(depth_img_upsampled, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(depth_img_upsampled, grad_y, CV_32F, 0, 1, 3);
    cv::magnitude(grad_x, grad_y, grad_magnitude);
    depth_img_upsampled.setTo(0, grad_magnitude > 0.75);

    cv::Mat morphology_mask;
    cv::threshold(depth_img_upsampled, morphology_mask, 0.1f, 255, cv::THRESH_BINARY);
    morphology_mask.convertTo(morphology_mask, CV_8UC1);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::Mat opened_mask;
    cv::morphologyEx(morphology_mask, opened_mask, cv::MORPH_OPEN, kernel);
    depth_img_upsampled.setTo(0, opened_mask == 0);
    depth_undistorted_ = depth_img_upsampled.clone();
    cv::Mat depth_img_distorted;
    cv::remap(depth_img_upsampled, depth_img_distorted, map_x_, map_y_, cv::INTER_LINEAR);
    return depth_img_distorted;
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloudToDepthConverter::generateColoredCloud(
    const cv::Mat &depth_img, const cv::Mat &color_img)
{
    cv::Mat depth_undistorted, color_undistorted;
    depth_undistorted = depth_undistorted_.clone();
    cv::remap(color_img, color_undistorted, inv_map_x_, inv_map_y_, cv::INTER_LINEAR);
    

    pcl::PointCloud<pcl::PointXYZRGB> cloud_colored;

    Eigen::Matrix4d Tlc = params_.Tcl.inverse();

    for (int v = 0; v < depth_undistorted.rows; v += params_.point_sampling_rate)
    {
        for (int u = 0; u < depth_undistorted.cols; u += params_.point_sampling_rate)
        {
            float depth = depth_undistorted.at<float>(v, u);
            if (depth > 0.1f && depth < 100.0f) 
            {
                double y_cam = (v - params_.v0) * depth / params_.A22;
                double x_cam = ((u - params_.u0) * depth  - params_.A12 * y_cam)/ params_.A11;
                
                double z_cam = depth;

                Eigen::Vector4d point_cam(x_cam, y_cam, z_cam, 1.0);

                Eigen::Vector4d point_lidar = Tlc * point_cam;

                pcl::PointXYZRGB point;
                point.x = static_cast<float>(point_lidar[0]);
                point.y = static_cast<float>(point_lidar[1]);
                point.z = static_cast<float>(point_lidar[2]);

                if (u < color_undistorted.cols && v < color_undistorted.rows)
                {
                    cv::Vec3b color = color_undistorted.at<cv::Vec3b>(v, u);
                    point.b = color[0]; 
                    point.g = color[1];
                    point.r = color[2];
                }
                else
                {
                    point.r = point.g = point.b = 255;
                }

                cloud_colored.points.push_back(point);
            }
        }
    }

    cloud_colored.width = cloud_colored.points.size();
    cloud_colored.height = 1;
    cloud_colored.is_dense = false;

    return cloud_colored;
}

std::pair<bool, std::string> PointCloudToDepthConverter::validateInputs(
    const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &image)
{
    if (cloud.empty())
    {
        return {false, "Empty point cloud"};
    }

    if (image.empty())
    {
        return {false, "Empty image"};
    }

    if (params_.A11 < 1e-6 || params_.A22 < 1e-6)
    {
        return {false, "Invalid camera intrinsics"};
    }

    return {true, ""};
}

void PointCloudToDepthConverter::updateCameraParams(const CameraParams &params)
{
    params_ = params;
    initializeInternalParams();
    createDistortionMaps();
}
