#include "depth_image_ros_node.hpp"
#include <boost/bind.hpp>

DepthImageRosNode::DepthImageRosNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh_)
{
    PointCloudToDepthConverter::CameraParams camera_params = loadCameraParams();

    depth_converter_ = std::make_unique<PointCloudToDepthConverter>(camera_params);
    pnh_.param<std::string>("cloud_raw_topic", cloud_raw_topic_, std::string("/odin1/cloud_raw"));
    pnh_.param<std::string>("color_compressed_topic_", color_compressed_topic_, std::string("/odin1/image/compressed"));
    pnh_.param<std::string>("depth_image_topic", depth_image_topic_, std::string("/odin1/depth_img_competetion"));
    pnh_.param<std::string>("depth_cloud_topic", depth_cloud_topic_, std::string("/odin1/depth_img_competetion_cloud"));

    ROS_INFO_STREAM("\n  cloud_raw_topic: " << cloud_raw_topic_
                                            << "\n  color_compressed_topic: " << color_compressed_topic_
                                            << "\n  depth_image_topic: " << depth_image_topic_
                                            << "\n  depth_cloud_topic: " << depth_cloud_topic_);

    cloud_sub_.subscribe(nh_, cloud_raw_topic_, 1);
    color_compressed_sub_.subscribe(nh_, color_compressed_topic_, 1);

    sync_ = std::make_shared<Sync>(MySyncPolicy(10), cloud_sub_, color_compressed_sub_);
    sync_->registerCallback(boost::bind(&DepthImageRosNode::syncCallback, this, _1, _2));

    depth_image_pub_ = it_.advertise(depth_image_topic_, 1);
    depth_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(depth_cloud_topic_, 1);

    ROS_INFO("DepthImageRosNode initialized successfully");
}

PointCloudToDepthConverter::CameraParams DepthImageRosNode::loadCameraParams()
{
    PointCloudToDepthConverter::CameraParams params;

    pnh_.param<int>("cam_0/image_width", params.image_width, 1600);
    pnh_.param<int>("cam_0/image_height", params.image_height, 1296);
    pnh_.param<double>("cam_0/A11", params.A11, 0.0);
    pnh_.param<double>("cam_0/A12", params.A12, 0.0);
    pnh_.param<double>("cam_0/A22", params.A22, 0.0);
    pnh_.param<double>("cam_0/u0", params.u0, 0.0);
    pnh_.param<double>("cam_0/v0", params.v0, 0.0);

    pnh_.param<double>("cam_0/k2", params.k2, 0.0);
    pnh_.param<double>("cam_0/k3", params.k3, 0.0);
    pnh_.param<double>("cam_0/k4", params.k4, 0.0);
    pnh_.param<double>("cam_0/k5", params.k5, 0.0);
    pnh_.param<double>("cam_0/k6", params.k6, 0.0);
    pnh_.param<double>("cam_0/k7", params.k7, 0.0);

    pnh_.param<double>("scale", params.scale, 7.0);
    pnh_.param<int>("point_sampling_rate", params.point_sampling_rate, 5);

    std::vector<double> Tcl_vec_param;
    if (pnh_.getParam("Tcl_0", Tcl_vec_param) && Tcl_vec_param.size() == 16)
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                params.Tcl(i, j) = Tcl_vec_param[i * 4 + j];
            }
        }
    }
    else
    {
        ROS_ERROR("Tcl_0 param missing or invalid, colored reproject cloud disabled.");
        ros::shutdown();
    }

    if (params.A11 < 1e-6 || params.A22 < 1e-6 || params.u0 < 1e-6 || params.v0 < 1e-6)
    {
        ROS_ERROR("Invalid camera intrinsics A11 or A22");
        ros::shutdown();
    }

    ROS_INFO("Camera intrinsics:");
    ROS_INFO("Image size: %dx%d", params.image_width, params.image_height);
    ROS_INFO("Intrinsics: A11=%f A12=%f A22=%f u0=%f v0=%f",
             params.A11, params.A12, params.A22, params.u0, params.v0);
    ROS_INFO("Distortions: k2=%f k3=%f k4=%f k5=%f k6=%f k7=%f",
             params.k2, params.k3, params.k4, params.k5, params.k6, params.k7);
    ROS_INFO("Scale: %f, Point sampling rate: %d", params.scale, params.point_sampling_rate);
    ROS_INFO_STREAM("Extrinsics (Tcl):\n"
                    << params.Tcl);

    return params;
}

void DepthImageRosNode::syncCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                     const sensor_msgs::CompressedImageConstPtr &image_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    if (cloud.empty())
    {
        ROS_WARN("Empty point cloud received");
        return;
    }

    cv::Mat img_raw;
    try
    {
        img_raw = cv::imdecode(cv::Mat(image_msg->data), cv::IMREAD_COLOR);
        if (img_raw.empty())
        {
            ROS_WARN("Failed to decode compressed image");
            return;
        }
    }
    catch (const cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge: %s", e.what());
        return;
    }

    auto result = depth_converter_->processCloudAndImage(cloud, img_raw);

    if (!result.success)
    {
        ROS_WARN("Data processing failed: %s", result.error_message.c_str());
        return;
    }

    publishDepthImage(result.depth_image, cloud_msg->header);
    publishDepthCloud(result.colored_cloud, cloud_msg->header);
}

void DepthImageRosNode::publishDepthImage(const cv::Mat &img,
                                          const std_msgs::Header &header,
                                          const std::string &encoding)
{
    sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
    depth_image_pub_.publish(depth_msg);
}

void DepthImageRosNode::publishDepthCloud(const pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud,
                                          const std_msgs::Header &header)
{
    if (!colored_cloud.points.empty())
    {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(colored_cloud, cloud_msg);
        cloud_msg.header = header;
        depth_cloud_pub_.publish(cloud_msg);
    }
}
