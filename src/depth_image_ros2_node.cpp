#include "depth_image_ros2_node.hpp"
#include <functional>

DepthImageRos2Node::DepthImageRos2Node(const rclcpp::NodeOptions & options)
    : Node("depth_image_ros2_node", options)
{
    PointCloudToDepthConverter::CameraParams camera_params = loadCameraParams();

    depth_converter_ = std::make_unique<PointCloudToDepthConverter>(camera_params);
    
    cloud_raw_topic_ = this->declare_parameter<std::string>("cloud_raw_topic", "/odin1/cloud_raw");
    color_compressed_topic_ = this->declare_parameter<std::string>("color_compressed_topic", "/odin1/image/compressed");
    color_raw_topic_ = this->declare_parameter<std::string>("color_raw_topic", "/odin1/image");
    depth_image_topic_ = this->declare_parameter<std::string>("depth_image_topic", "/odin1/depth_img_competetion");
    depth_cloud_topic_ = this->declare_parameter<std::string>("depth_cloud_topic", "/odin1/depth_img_competetion_cloud");

    RCLCPP_INFO_STREAM(this->get_logger(), 
                       "\n  cloud_raw_topic: " << cloud_raw_topic_
                       << "\n  color_compressed_topic: " << color_compressed_topic_
                       << "\n  color_raw_topic: " << color_raw_topic_
                       << "\n  depth_image_topic: " << depth_image_topic_
                       << "\n  depth_cloud_topic: " << depth_cloud_topic_);
}

void DepthImageRos2Node::initialize()
{
    cloud_sub_.subscribe(this, cloud_raw_topic_);
    color_compressed_sub_.subscribe(this, color_compressed_topic_);
    color_sub_.subscribe(this, color_raw_topic_);

    sync_ = std::make_shared<Sync>(MySyncPolicy(10), cloud_sub_, color_sub_);
    sync_->registerCallback(std::bind(&DepthImageRos2Node::syncCallback, this, 
                                     std::placeholders::_1, std::placeholders::_2));

    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    depth_image_pub_ = it_->advertise(depth_image_topic_, 1);
    depth_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(depth_cloud_topic_, 1);

    RCLCPP_INFO(this->get_logger(), "DepthImageRos2Node initialized successfully");
}

PointCloudToDepthConverter::CameraParams DepthImageRos2Node::loadCameraParams()
{
    PointCloudToDepthConverter::CameraParams params;

    params.image_width = this->declare_parameter<int>("cam_0.image_width", 1600);
    params.image_height = this->declare_parameter<int>("cam_0.image_height", 1296);
    params.A11 = this->declare_parameter<double>("cam_0.A11", 0.0);
    params.A12 = this->declare_parameter<double>("cam_0.A12", 0.0);
    params.A22 = this->declare_parameter<double>("cam_0.A22", 0.0);
    params.u0 = this->declare_parameter<double>("cam_0.u0", 0.0);
    params.v0 = this->declare_parameter<double>("cam_0.v0", 0.0);

    params.k2 = this->declare_parameter<double>("cam_0.k2", 0.0);
    params.k3 = this->declare_parameter<double>("cam_0.k3", 0.0);
    params.k4 = this->declare_parameter<double>("cam_0.k4", 0.0);
    params.k5 = this->declare_parameter<double>("cam_0.k5", 0.0);
    params.k6 = this->declare_parameter<double>("cam_0.k6", 0.0);
    params.k7 = this->declare_parameter<double>("cam_0.k7", 0.0);

    params.scale = this->declare_parameter<double>("scale", 7.0);
    params.point_sampling_rate = this->declare_parameter<int>("point_sampling_rate", 5);

    std::vector<double> Tcl_vec_param = this->declare_parameter<std::vector<double>>("Tcl_0", std::vector<double>(16, 0.0));
    if (Tcl_vec_param.size() == 16)
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
        RCLCPP_ERROR(this->get_logger(), "Tcl_0 param missing or invalid, colored reproject cloud disabled.");
        rclcpp::shutdown();
    }

    if (params.A11 < 1e-6 || params.A22 < 1e-6 || params.u0 < 1e-6 || params.v0 < 1e-6)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid camera intrinsics A11 or A22");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Camera intrinsics:");
    RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", params.image_width, params.image_height);
    RCLCPP_INFO(this->get_logger(), "Intrinsics: A11=%f A12=%f A22=%f u0=%f v0=%f",
             params.A11, params.A12, params.A22, params.u0, params.v0);
    RCLCPP_INFO(this->get_logger(), "Distortions: k2=%f k3=%f k4=%f k5=%f k6=%f k7=%f",
             params.k2, params.k3, params.k4, params.k5, params.k6, params.k7);
    RCLCPP_INFO(this->get_logger(), "Scale: %f, Point sampling rate: %d", params.scale, params.point_sampling_rate);
    RCLCPP_INFO_STREAM(this->get_logger(), "Extrinsics (Tcl):\n" << params.Tcl);

    return params;
}

void DepthImageRos2Node::syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
                                     // const sensor_msgs::msg::CompressedImage::ConstSharedPtr image_msg,
                                     const sensor_msgs::msg::Image::ConstSharedPtr color_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    if (cloud.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Empty point cloud received");
        return;
    }

    cv::Mat img_raw;
    try
    {
        // img_raw = cv::imdecode(cv::Mat(image_msg->data), cv::IMREAD_COLOR);
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(color_msg, "bgr8");
        img_raw = cv_ptr->image;
        if (img_raw.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to decode compressed image");
            return;
        }
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
        return;
    }

    auto result = depth_converter_->processCloudAndImage(cloud, img_raw);

    if (!result.success)
    {
        RCLCPP_WARN(this->get_logger(), "Data processing failed: %s", result.error_message.c_str());
        return;
    }

    publishDepthImage(result.depth_image, cloud_msg->header);
    publishDepthCloud(result.colored_cloud, cloud_msg->header);
}

void DepthImageRos2Node::publishDepthImage(const cv::Mat &img,
                                          const std_msgs::msg::Header &header,
                                          const std::string &encoding)
{
    sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
    depth_image_pub_.publish(*depth_msg);
}

void DepthImageRos2Node::publishDepthCloud(const pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud,
                                          const std_msgs::msg::Header &header)
{
    if (!colored_cloud.points.empty())
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(colored_cloud, cloud_msg);
        cloud_msg.header = header;
        depth_cloud_pub_->publish(cloud_msg);
    }
}
