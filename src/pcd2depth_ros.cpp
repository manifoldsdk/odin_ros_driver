#include <ros/ros.h>
#include <fstream>
#include <sys/stat.h>

#include "depth_image_ros_node.hpp"

bool fileExists(const std::string& filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_projector");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    int senddepth=0;
    pnh.param("register_keys/senddepth", senddepth, 0);
    if(senddepth==0)
    {
        ROS_INFO("Depth image will not be published.");
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
    
    DepthImageRosNode projector(nh, pnh);
    ros::spin();
    return 0;
}
