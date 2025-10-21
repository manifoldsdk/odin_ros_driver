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

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <string>
#include "depth_image_ros2_node.hpp"
#include <rcpputils/filesystem_helper.hpp>
bool fileExists(const std::string& filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

bool loadCalibParameters(std::shared_ptr<rclcpp::Node> node, const std::string& calib_file_path) {
    try {
        RCLCPP_INFO(node->get_logger(), "Loading parameters from calib.yaml file: %s", calib_file_path.c_str());
        
        YAML::Node config = YAML::LoadFile(calib_file_path);
        
        if (config["cam_num"]) {
            int cam_num = config["cam_num"].as<int>();
            node->declare_parameter("cam_num", cam_num);
            RCLCPP_INFO(node->get_logger(), "Loaded parameter: cam_num = %d", cam_num);
        }
        
        if (config["img_topic_0"]) {
            std::string img_topic = config["img_topic_0"].as<std::string>();
            node->declare_parameter("img_topic_0", img_topic);
            RCLCPP_INFO(node->get_logger(), "Loaded parameter: img_topic_0 = %s", img_topic.c_str());
        }
        
        if (config["Tcl_0"]) {
            std::vector<double> tcl_matrix = config["Tcl_0"].as<std::vector<double>>();
            node->declare_parameter("Tcl_0", tcl_matrix);
            RCLCPP_INFO(node->get_logger(), "Loaded parameter: Tcl_0 (transform matrix with %zu elements)", tcl_matrix.size());
        }
        
        if (config["cam_0"]) {
            YAML::Node cam_0 = config["cam_0"];
            
            if (cam_0["cam_model"]) {
                std::string cam_model = cam_0["cam_model"].as<std::string>();
                node->declare_parameter("cam_0.cam_model", cam_model);
                RCLCPP_INFO(node->get_logger(), "Loaded parameter: cam_0.cam_model = %s", cam_model.c_str());
            }
            
            if (cam_0["image_width"]) {
                int image_width = cam_0["image_width"].as<int>();
                node->declare_parameter("cam_0.image_width", image_width);
                RCLCPP_INFO(node->get_logger(), "Loaded parameter: cam_0.image_width = %d", image_width);
            }
            
            if (cam_0["image_height"]) {
                int image_height = cam_0["image_height"].as<int>();
                node->declare_parameter("cam_0.image_height", image_height);
                RCLCPP_INFO(node->get_logger(), "Loaded parameter: cam_0.image_height = %d", image_height);
            }
            
            std::vector<std::string> distortion_params = {"k2", "k3", "k4", "k5", "k6", "k7", "p1", "p2"};
            for (const auto& param : distortion_params) {
                if (cam_0[param]) {
                    double value = cam_0[param].as<double>();
                    node->declare_parameter("cam_0." + param, value);
                    RCLCPP_INFO(node->get_logger(), "Loaded parameter: cam_0.%s = %f", param.c_str(), value);
                }
            }
            
            std::vector<std::string> intrinsic_params = {"A11", "A12", "A22", "u0", "v0"};
            for (const auto& param : intrinsic_params) {
                if (cam_0[param]) {
                    double value = cam_0[param].as<double>();
                    node->declare_parameter("cam_0." + param, value);
                    RCLCPP_INFO(node->get_logger(), "Loaded parameter: cam_0.%s = %f", param.c_str(), value);
                }
            }
            
            if (cam_0["isFast"]) {
                int is_fast = cam_0["isFast"].as<int>();
                node->declare_parameter("cam_0.isFast", is_fast);
                RCLCPP_INFO(node->get_logger(), "Loaded parameter: cam_0.isFast = %d", is_fast);
            }
            
            if (cam_0["numDiff"]) {
                int num_diff = cam_0["numDiff"].as<int>();
                node->declare_parameter("cam_0.numDiff", num_diff);
                RCLCPP_INFO(node->get_logger(), "Loaded parameter: cam_0.numDiff = %d", num_diff);
            }
            
            if (cam_0["maxIncidentAngle"]) {
                int max_incident_angle = cam_0["maxIncidentAngle"].as<int>();
                node->declare_parameter("cam_0.maxIncidentAngle", max_incident_angle);
                RCLCPP_INFO(node->get_logger(), "Loaded parameter: cam_0.maxIncidentAngle = %d", max_incident_angle);
            }
        }
        
        RCLCPP_INFO(node->get_logger(), "Successfully loaded all parameters from calib.yaml");
        return true;
        
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(node->get_logger(), "YAML parsing error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error loading calib parameters: %s", e.what());
        return false;
    }
}
std::string get_package_source_directory() {
    // 使用 rcpputils::fs::path 替代 std::filesystem::path
    rcpputils::fs::path current_file(__FILE__);
    
    // 回溯到包根目录
    auto path = current_file.parent_path();
    
    // 使用 rcpputils::fs::exists 替代 std::filesystem::exists
    while (!path.empty() && !rcpputils::fs::exists(path / "package.xml")) {
        path = path.parent_path();
    }
    
    if (path.empty()) {
        throw std::runtime_error("Failed to locate package root directory");
    }
    
    return path.string();
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    

    auto node = std::make_shared<rclcpp::Node>("pcd2depth_node");
    RCLCPP_INFO(node->get_logger(), "Node created");
    std::string package_path = get_package_source_directory();
    RCLCPP_INFO(node->get_logger(), "Package path: %s", package_path.c_str());
    
    std::string config_file = package_path + "/config/control_command.yaml";
    RCLCPP_INFO(node->get_logger(), "Loading config from: %s", config_file.c_str());

    YAML::Node config = YAML::LoadFile(config_file);
        
    if (!config["register_keys"]) {
            throw std::runtime_error("Missing 'register_keys' section");
        }
        
    if (!config["register_keys"]["senddepth"]) {
            throw std::runtime_error("Missing 'senddepth' parameter");
        }  
    int senddepth = config["register_keys"]["senddepth"].as<int>();
    std::cout << "senddepth: " <<    senddepth << std::endl;
    if(senddepth == 0)
    {
        RCLCPP_INFO(node->get_logger(), "Depth image will not be published.");
        return 0;
    }
    
    std::string calib_file_path = node->declare_parameter<std::string>("calib_file_path", "");
    
    RCLCPP_INFO(node->get_logger(), "Waiting for calib.yaml file at: %s", calib_file_path.c_str());
    while(rclcpp::ok() && !fileExists(calib_file_path))
    {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "Still waiting for calib.yaml file...");
        rclcpp::sleep_for(std::chrono::milliseconds(500)); // 等待0.5秒后再检查
        rclcpp::spin_some(node);
    }
    
    if(!rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "Node shutdown before calib.yaml file was found.");
        return 0;
    }
    
    RCLCPP_INFO(node->get_logger(), "Found calib.yaml file! Loading parameters...");
    
    if (!loadCalibParameters(node, calib_file_path)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load parameters from calib.yaml file");
        return 1;
    }
    
    rclcpp::NodeOptions depth_node_options;
    
    std::vector<rclcpp::Parameter> params_override;
    
    try {
        params_override.push_back(rclcpp::Parameter("cam_0.image_width", node->get_parameter("cam_0.image_width").as_int()));
        params_override.push_back(rclcpp::Parameter("cam_0.image_height", node->get_parameter("cam_0.image_height").as_int()));
        params_override.push_back(rclcpp::Parameter("cam_0.A11", node->get_parameter("cam_0.A11").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.A12", node->get_parameter("cam_0.A12").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.A22", node->get_parameter("cam_0.A22").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.u0", node->get_parameter("cam_0.u0").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.v0", node->get_parameter("cam_0.v0").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k2", node->get_parameter("cam_0.k2").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k3", node->get_parameter("cam_0.k3").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k4", node->get_parameter("cam_0.k4").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k5", node->get_parameter("cam_0.k5").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k6", node->get_parameter("cam_0.k6").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k7", node->get_parameter("cam_0.k7").as_double()));
        params_override.push_back(rclcpp::Parameter("Tcl_0", node->get_parameter("Tcl_0").as_double_array()));
        
        depth_node_options.parameter_overrides(params_override);
        
        RCLCPP_INFO(node->get_logger(), "Parameters successfully prepared for DepthImageRos2Node");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error preparing parameters for DepthImageRos2Node: %s", e.what());
        return 1;
    }
    try 
    {
        auto depth_node = std::make_shared<DepthImageRos2Node>(depth_node_options);
        
        depth_node->initialize();
        
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(depth_node);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error creating depth node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
