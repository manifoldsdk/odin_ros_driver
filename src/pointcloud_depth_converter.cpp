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

cv::Mat PointCloudToDepthConverter::postProcessDepthImage(const cv::Mat &depth_img) {
    if (depth_img.empty()) {
        std::cerr << "ERROR: Input depth image is empty!" << std::endl;
        return cv::Mat();
    }
    
    if (depth_img.data == nullptr) {
        std::cerr << "ERROR: Input depth image has null data pointer!" << std::endl;
        return cv::Mat();
    }
    
    if (depth_img.rows <= 0 || depth_img.cols <= 0) {
        std::cerr << "ERROR: Invalid input dimensions: " 
                  << depth_img.rows << "x" << depth_img.cols << std::endl;
        return cv::Mat();
    }
    
    if (params_.image_width <= 0 || params_.image_height <= 0) {
        std::cerr << "ERROR: Invalid target size: " 
                  << params_.image_width << "x" << params_.image_height << std::endl;
        return cv::Mat();
    }

    cv::Mat safe_input = depth_img.clone();
    if (safe_input.empty()) {
        std::cerr << "ERROR: Failed to create safe copy of input image!" << std::endl;
        return cv::Mat();
    }
    

    cv::Mat depth_img_upsampled;
    try {
        depth_img_upsampled = customResize(safe_input, cv::Size(1600, 1296));
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Custom resize failed: " << e.what() << std::endl;
        return cv::Mat();
    }
    
    if (depth_img_upsampled.empty()) {
        std::cerr << "ERROR: Resized image is empty!" << std::endl;
        return cv::Mat();
    }
    
    if (depth_img_upsampled.rows != 1296 || depth_img_upsampled.cols != 1600) {
        std::cerr << "ERROR: Resized image has wrong dimensions: " 
                  << depth_img_upsampled.cols << "x" << depth_img_upsampled.rows
                  << " (expected " << 1600 << "x" << 1296 << ")" << std::endl;
        return cv::Mat();
    }
    

    cv::Mat grad_x, grad_y, grad_magnitude;
    try {
        cv::Sobel(depth_img_upsampled, grad_x, CV_32F, 1, 0, 3);
        cv::Sobel(depth_img_upsampled, grad_y, CV_32F, 0, 1, 3);
        cv::magnitude(grad_x, grad_y, grad_magnitude);
    } catch (const cv::Exception& e) {
        std::cerr << "ERROR: Sobel/magnitude failed: " << e.what() << std::endl;
        return cv::Mat();
    }
    
    if (grad_magnitude.type() != CV_32F) {
        std::cerr << "ERROR: grad_magnitude has wrong type: " 
                  << grad_magnitude.type() << " (expected CV_32F)" << std::endl;
        return cv::Mat();
    }
    

    cv::Mat threshold_mask;
    try {
        cv::threshold(grad_magnitude, threshold_mask, 0.75, 1, cv::THRESH_BINARY);
        threshold_mask.convertTo(threshold_mask, CV_8U);
        

        depth_img_upsampled.setTo(0, threshold_mask);
    } catch (const cv::Exception& e) {
        std::cerr << "ERROR: Threshold mask failed: " << e.what() << std::endl;
        return cv::Mat();
    }

    cv::Mat depth_img_distorted;
    try {
        depth_undistorted_ = depth_img_upsampled.clone();
        cv::remap(depth_img_upsampled, depth_img_distorted, map_x_, map_y_, cv::INTER_LINEAR);
    } catch (const cv::Exception& e) {
        std::cerr << "ERROR: Remap failed: " << e.what() << std::endl;
        return cv::Mat();
    }
    
    return depth_img_distorted;
}

cv::Mat PointCloudToDepthConverter::customResize(const cv::Mat& src, const cv::Size& size) {
    if (src.empty()) {
        throw std::runtime_error("Source image is empty");
    }
    
    if (size.width <= 0 || size.height <= 0) {
        throw std::runtime_error("Invalid target size");
    }
    

    cv::Mat dst(size.height, size.width, src.type());
    
    float scale_x = src.cols / static_cast<float>(size.width);
    float scale_y = src.rows / static_cast<float>(size.height);
    
    if (src.channels() != 1 || src.type() != CV_32F) {
        throw std::runtime_error("Unsupported image type - expected single channel float");
    }
    

    for (int y = 0; y < dst.rows; y++) {

        int src_y = static_cast<int>(y * scale_y);
        src_y = std::min(src_y, src.rows - 1);
        
        for (int x = 0; x < dst.cols; x++) {
            int src_x = static_cast<int>(x * scale_x);
            src_x = std::min(src_x, src.cols - 1);
            dst.at<float>(y, x) = src.at<float>(src_y, src_x);
        }
    }
    
    return dst;
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
