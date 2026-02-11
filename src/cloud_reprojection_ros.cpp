/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "cloud_reprojection_ros_node.hpp"

#include <fstream>
#include <sys/stat.h>
#include <thread>
#include <chrono>

#ifdef ROS2
    #include <functional>
    #include <yaml-cpp/yaml.h>
    #include <rcpputils/filesystem_helper.hpp>
#else
    #include <boost/bind.hpp>
#endif

// Fixed Til (T_imu_lidar): lidar position in imu frame, transforms from lidar to imu
// TODO: Fill in the actual Til values for your sensor setup
static Eigen::Matrix4d getFixedTil()
{
    Eigen::Matrix4d Til = Eigen::Matrix4d::Identity();
    Til(0, 3) = -0.02663;
    Til(1, 3) = 0.03447;
    Til(2, 3) = 0.02174;
    return Til;
}

static bool fileExists(const std::string& filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

#ifdef ROS2
// Helper function to get package source directory for ROS2
static std::string get_package_source_directory() {
    std::string current_file = __FILE__;
    size_t pos = current_file.find("/src/cloud_reprojection_ros.cpp");
    if (pos != std::string::npos) {
        return current_file.substr(0, pos);
    }
    return "";
}

// ==================== ROS2 Implementation ====================

CloudReprojectionRosNode::CloudReprojectionRosNode(const rclcpp::NodeOptions& options)
    : Node("cloud_reprojection_node", options)
{
    loadParameters();

    RCLCPP_INFO_STREAM(this->get_logger(), 
        "\n  cloud_slam_topic: " << cloud_slam_topic_
        << "\n  odometry_topic: " << odometry_topic_
        << "\n  reprojected_image_topic: " << reprojected_image_topic_);

    cloud_sub_.subscribe(this, cloud_slam_topic_);
    odom_sub_.subscribe(this, odometry_topic_);

    sync_ = std::make_shared<Sync>(MySyncPolicy(10), cloud_sub_, odom_sub_);
    sync_->registerCallback(std::bind(&CloudReprojectionRosNode::syncCallback, this, 
                                       std::placeholders::_1, std::placeholders::_2));

    reprojected_image_pub_ = image_transport::create_publisher(this, reprojected_image_topic_);

    RCLCPP_INFO(this->get_logger(), "CloudReprojectionRosNode initialized successfully");
}

void CloudReprojectionRosNode::loadParameters()
{
    // Declare and get parameters
    this->declare_parameter<std::string>("cloud_slam_topic", "/odin1/cloud_slam");
    this->declare_parameter<std::string>("odometry_topic", "/odin1/odometry");
    this->declare_parameter<std::string>("reprojected_image_topic", "/odin1/reprojected_image");

    cloud_slam_topic_ = this->get_parameter("cloud_slam_topic").as_string();
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    reprojected_image_topic_ = this->get_parameter("reprojected_image_topic").as_string();

    // Load camera parameters from calib.yaml file directly
    std::string package_path = get_package_source_directory();
    std::string calib_file = package_path + "/config/calib.yaml";
    
    YAML::Node calib_config;
    try {
        calib_config = YAML::LoadFile(calib_file);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load calib.yaml: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    CloudReprojector::CameraParams cam_params;
    try {
        cam_params.image_width = calib_config["cam_0"]["image_width"].as<int>();
        cam_params.image_height = calib_config["cam_0"]["image_height"].as<int>();
        cam_params.A11 = calib_config["cam_0"]["A11"].as<double>();
        cam_params.A12 = calib_config["cam_0"]["A12"].as<double>();
        cam_params.A22 = calib_config["cam_0"]["A22"].as<double>();
        cam_params.u0 = calib_config["cam_0"]["u0"].as<double>();
        cam_params.v0 = calib_config["cam_0"]["v0"].as<double>();
        cam_params.k2 = calib_config["cam_0"]["k2"].as<double>();
        cam_params.k3 = calib_config["cam_0"]["k3"].as<double>();
        cam_params.k4 = calib_config["cam_0"]["k4"].as<double>();
        cam_params.k5 = calib_config["cam_0"]["k5"].as<double>();
        cam_params.k6 = calib_config["cam_0"]["k6"].as<double>();
        cam_params.k7 = calib_config["cam_0"]["k7"].as<double>();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse camera parameters: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    // Load extrinsic parameters
    CloudReprojector::ExtrinsicParams ext_params;
    try {
        auto Tcl_vec = calib_config["Tcl_0"].as<std::vector<double>>();
        
        if (Tcl_vec.size() == 16)
        {
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    ext_params.Tcl(i, j) = Tcl_vec[i * 4 + j];
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Tcl_0 has invalid size: %zu (expected 16)", Tcl_vec.size());
            rclcpp::shutdown();
            return;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse Tcl_0: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    ext_params.Til = getFixedTil();
    ext_params.Tic = CloudReprojector::calculateTic(ext_params.Tcl, ext_params.Til);

    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded Tcl (camera to lidar):\n" << ext_params.Tcl);
    RCLCPP_INFO_STREAM(this->get_logger(), "Fixed Til (lidar to imu):\n" << ext_params.Til);
    RCLCPP_INFO_STREAM(this->get_logger(), "Calculated Tic:\n" << ext_params.Tic);

    RCLCPP_INFO(this->get_logger(), "Camera intrinsics:");
    RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", cam_params.image_width, cam_params.image_height);
    RCLCPP_INFO(this->get_logger(), "Intrinsics: A11=%f A12=%f A22=%f u0=%f v0=%f",
                cam_params.A11, cam_params.A12, cam_params.A22, cam_params.u0, cam_params.v0);

    reprojector_ = std::make_unique<CloudReprojector>();
    if (!reprojector_->initialize(cam_params, ext_params))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize CloudReprojector");
        rclcpp::shutdown();
    }
}

void CloudReprojectionRosNode::syncCallback(
    const PointCloud2::ConstSharedPtr& cloud_msg,
    const Odometry::ConstSharedPtr& odom_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud_odom;
    pcl::fromROSMsg(*cloud_msg, cloud_odom);

    if (cloud_odom.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Empty cloud_slam received");
        return;
    }

    CloudReprojector::OdomPose odom_pose;
    odom_pose.orientation = Eigen::Quaterniond(
        odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z
    );
    odom_pose.position = Eigen::Vector3d(
        odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z
    );

    cv::Mat reprojected_img = reprojector_->reprojectCloud(cloud_odom, odom_pose);

    auto img_msg = cv_bridge::CvImage(cloud_msg->header, "bgr8", reprojected_img).toImageMsg();
    reprojected_image_pub_.publish(*img_msg);
}

// ==================== ROS2 Main ====================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto temp_node = std::make_shared<rclcpp::Node>("cloud_reprojection_check");
    
    // Check if reprojection is enabled from control_command.yaml
    std::string package_path = get_package_source_directory();
    std::string config_file = package_path + "/config/control_command.yaml";
    
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        std::cout << "config: " << config_file << std::endl;
        if (!config["register_keys"] || !config["register_keys"]["sendreprojection"]) {
            RCLCPP_INFO(temp_node->get_logger(), "sendreprojection parameter not found, cloud reprojection disabled.");
            rclcpp::shutdown();
            return 0;
        }
        
        int sendreprojection = config["register_keys"]["sendreprojection"].as<int>();
        if (sendreprojection == 0) {
            RCLCPP_INFO(temp_node->get_logger(), "Cloud reprojection will not be published.");
            rclcpp::shutdown();
            return 0;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(temp_node->get_logger(), "Failed to read config: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    // Wait for calib.yaml file to be generated by host_sdk_sample
    std::string calib_file = package_path + "/config/calib.yaml";
    RCLCPP_INFO(temp_node->get_logger(), "Waiting for calib.yaml file at: %s", calib_file.c_str());
    
    int wait_count = 0;
    while (rclcpp::ok() && !fileExists(calib_file)) {
        if (wait_count % 10 == 0) {
            RCLCPP_INFO(temp_node->get_logger(), "Still waiting for calib.yaml file...");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        wait_count++;
        
        // Timeout after 5 seconds
        if (wait_count > 10) {
            RCLCPP_ERROR(temp_node->get_logger(), "Timeout waiting for calib.yaml file");
            rclcpp::shutdown();
            return 1;
        }
    }
    
    if (!rclcpp::ok()) {
        RCLCPP_INFO(temp_node->get_logger(), "Node shutdown before calib.yaml file was found.");
        return 0;
    }
    
    RCLCPP_INFO(temp_node->get_logger(), "Found calib.yaml file! Starting cloud reprojection node...");

    auto node = std::make_shared<CloudReprojectionRosNode>();

    RCLCPP_INFO(node->get_logger(), "CloudReprojectionRosNode started");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

#else
// ==================== ROS1 Implementation ====================

CloudReprojectionRosNode::CloudReprojectionRosNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    loadParameters();

    ROS_INFO_STREAM("\n  cloud_slam_topic: " << cloud_slam_topic_
                << "\n  odometry_topic: " << odometry_topic_
                << "\n  reprojected_image_topic: " << reprojected_image_topic_);

    cloud_sub_.subscribe(nh_, cloud_slam_topic_, 1);
    odom_sub_.subscribe(nh_, odometry_topic_, 1);

    sync_ = std::make_shared<Sync>(MySyncPolicy(10), cloud_sub_, odom_sub_);
    sync_->registerCallback(boost::bind(&CloudReprojectionRosNode::syncCallback, this, _1, _2));

    reprojected_image_pub_ = nh_.advertise<sensor_msgs::Image>(reprojected_image_topic_, 1);

    ROS_INFO("CloudReprojectionRosNode initialized successfully");
}

void CloudReprojectionRosNode::loadParameters()
{
    pnh_.param<std::string>("cloud_slam_topic", cloud_slam_topic_, std::string("/odin1/cloud_slam"));
    pnh_.param<std::string>("odometry_topic", odometry_topic_, std::string("/odin1/odometry"));
    pnh_.param<std::string>("reprojected_image_topic", reprojected_image_topic_, std::string("/odin1/reprojected_image"));

    // Load camera parameters
    CloudReprojector::CameraParams cam_params;
    pnh_.param<int>("cam_0/image_width", cam_params.image_width, 1600);
    pnh_.param<int>("cam_0/image_height", cam_params.image_height, 1296);
    pnh_.param<double>("cam_0/A11", cam_params.A11, 0.0);
    pnh_.param<double>("cam_0/A12", cam_params.A12, 0.0);
    pnh_.param<double>("cam_0/A22", cam_params.A22, 0.0);
    pnh_.param<double>("cam_0/u0", cam_params.u0, 0.0);
    pnh_.param<double>("cam_0/v0", cam_params.v0, 0.0);
    pnh_.param<double>("cam_0/k2", cam_params.k2, 0.0);
    pnh_.param<double>("cam_0/k3", cam_params.k3, 0.0);
    pnh_.param<double>("cam_0/k4", cam_params.k4, 0.0);
    pnh_.param<double>("cam_0/k5", cam_params.k5, 0.0);
    pnh_.param<double>("cam_0/k6", cam_params.k6, 0.0);
    pnh_.param<double>("cam_0/k7", cam_params.k7, 0.0);

    // Load extrinsic parameters
    CloudReprojector::ExtrinsicParams ext_params;
    std::vector<double> Tcl_vec_param;
    if (pnh_.getParam("Tcl_0", Tcl_vec_param) && Tcl_vec_param.size() == 16)
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                ext_params.Tcl(i, j) = Tcl_vec_param[i * 4 + j];
    }
    else
    {
        ROS_ERROR("Tcl_0 param missing or invalid.");
        ros::shutdown();
        return;
    }

    ext_params.Til = getFixedTil();
    ext_params.Tic = CloudReprojector::calculateTic(ext_params.Tcl, ext_params.Til);

    ROS_INFO_STREAM("Loaded Tcl (camera to lidar):\n" << ext_params.Tcl);
    ROS_INFO_STREAM("Fixed Til (lidar to imu):\n" << ext_params.Til);
    ROS_INFO_STREAM("Calculated Tic:\n" << ext_params.Tic);

    ROS_INFO("Camera intrinsics:");
    ROS_INFO("Image size: %dx%d", cam_params.image_width, cam_params.image_height);
    ROS_INFO("Intrinsics: A11=%f A12=%f A22=%f u0=%f v0=%f",
             cam_params.A11, cam_params.A12, cam_params.A22, cam_params.u0, cam_params.v0);

    reprojector_ = std::make_unique<CloudReprojector>();
    if (!reprojector_->initialize(cam_params, ext_params))
    {
        ROS_ERROR("Failed to initialize CloudReprojector");
        ros::shutdown();
    }
}

void CloudReprojectionRosNode::syncCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
    const nav_msgs::OdometryConstPtr& odom_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud_odom;
    pcl::fromROSMsg(*cloud_msg, cloud_odom);

    if (cloud_odom.empty())
    {
        ROS_WARN("Empty cloud_slam received");
        return;
    }

    CloudReprojector::OdomPose odom_pose;
    odom_pose.orientation = Eigen::Quaterniond(
        odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z
    );
    odom_pose.position = Eigen::Vector3d(
        odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z
    );

    cv::Mat reprojected_img = reprojector_->reprojectCloud(cloud_odom, odom_pose);

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(cloud_msg->header, "bgr8", reprojected_img).toImageMsg();
    reprojected_image_pub_.publish(img_msg);
}

// ==================== ROS1 Main ====================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_reprojection");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    // Check if reprojection is enabled
    int sendreprojection = 0;
    pnh.param("register_keys/sendreprojection", sendreprojection, 0);
    if(sendreprojection == 0)
    {
        ROS_INFO("Cloud reprojection will not be published.");
        return 0;
    }
    
    std::string calib_file_path;
    pnh.param<std::string>("calib_file_path", calib_file_path, "");
    
    ROS_INFO("Waiting for calib.yaml file at: %s", calib_file_path.c_str());
    while(ros::ok() && !fileExists(calib_file_path))
    {
        ROS_INFO_THROTTLE(5, "Still waiting for calib.yaml file...");
        ros::Duration(0.5).sleep(); 
        ros::spinOnce();
    }
    
    if(!ros::ok())
    {
        ROS_INFO("Node shutdown before calib.yaml file was found.");
        return 0;
    }
    
    ROS_INFO("Found calib.yaml file! Loading parameters...");
    
    std::string node_name = ros::this_node::getName();
    std::string rosparam_command = "rosparam load " + calib_file_path + " " + node_name;
    int result = system(rosparam_command.c_str());
    
    if(result == 0)
    {
        ROS_INFO("Successfully loaded parameters from calib.yaml to namespace: %s", node_name.c_str());
    }
    else
    {
        ROS_ERROR("Failed to load parameters from calib.yaml");
        return 1;
    }
    
    CloudReprojectionRosNode reprojection_node(nh, pnh);
    ros::spin();
    return 0;
}
#endif
