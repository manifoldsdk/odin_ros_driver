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

#include "rawCloudRender.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <algorithm> 
#include <iostream> 
#include <cmath>
#include <array>
#include <algorithm>

struct ValidPointInfo {
    float x;
    float y;
    float z;
    int u;
    int v;
};


namespace GlobalCameraParams {
    float g_fx = 0.0f;
    float g_fy = 0.0f;
    float g_cx = 0.0f;
    float g_cy = 0.0f;
    float g_skew = 0.0f;
    float g_k2 = 0.0f;
    float g_k3 = 0.0f;
    float g_k4 = 0.0f;
    float g_k5 = 0.0f;
    float g_k6 = 0.0f;
    float g_k7 = 0.0f;
    Eigen::Matrix4f g_T_camera_lidar = Eigen::Matrix4f::Identity();
}
bool raw_debug=0;
bool rawCloudRender::init(const std::string& yamlFilePath) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(yamlFilePath);
    } catch (const YAML::BadFile& e) {
        std::cerr << "Error: Could not open file '" << yamlFilePath << "' - " << e.what() << std::endl;
        return false;
    } catch (const YAML::ParserException& e) {
        std::cerr << "Error: YAML parsing failed - " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Unexpected error: " << e.what() << std::endl;
        return false;
    }

    // Intrinsic parameters node (cam_0)
    const std::string cam_node_name = "cam_0";
    if (!config[cam_node_name]) {
        std::cerr << "Error: Missing camera node '" << cam_node_name << "'" << std::endl;
        return false;
    }
    YAML::Node cam_node = config[cam_node_name];

    // Directly access Tcl_0 node
    const std::string tcl_node_name = "Tcl_0";
    if (!config[tcl_node_name]) {
        std::cerr << "Error: Missing transformation matrix node '" << tcl_node_name << "'" << std::endl;
        return false;
    }
    YAML::Node tclNode = config[tcl_node_name];
    if (tclNode.size() != 16) {
        std::cerr << "Error: Transformation matrix must be 4x4 (16 elements)" << std::endl;
        return false;
    }

    // === Read intrinsic parameters ===
    GlobalCameraParams::g_k2 = cam_node["k2"].as<float>();
    GlobalCameraParams::g_k3 = cam_node["k3"].as<float>();
    GlobalCameraParams::g_k4 = cam_node["k4"].as<float>();
    GlobalCameraParams::g_k5 = cam_node["k5"].as<float>();
    GlobalCameraParams::g_k6 = cam_node["k6"].as<float>();
    GlobalCameraParams::g_k7 = cam_node["k7"].as<float>();
    
    GlobalCameraParams::g_fx = cam_node["A11"].as<float>();
    GlobalCameraParams::g_skew = cam_node["A12"].as<float>();
    GlobalCameraParams::g_fy = cam_node["A22"].as<float>();
    
    GlobalCameraParams::g_cx = cam_node["u0"].as<float>();
    GlobalCameraParams::g_cy = cam_node["v0"].as<float>();

    // === Read extrinsic transformation matrix ===
    GlobalCameraParams::g_T_camera_lidar << 
        tclNode[0].as<float>(), tclNode[1].as<float>(), tclNode[2].as<float>(), tclNode[3].as<float>(),
        tclNode[4].as<float>(), tclNode[5].as<float>(), tclNode[6].as<float>(), tclNode[7].as<float>(),
        tclNode[8].as<float>(), tclNode[9].as<float>(), tclNode[10].as<float>(), tclNode[11].as<float>(),
        tclNode[12].as<float>(), tclNode[13].as<float>(), tclNode[14].as<float>(), tclNode[15].as<float>();

    // === Display key parameters concisely ===
    std::cout << "=== Camera Calibration Parameters ===" << std::endl;
    std::cout << "Intrinsics:" << std::endl;
    std::cout << "  fx: " << GlobalCameraParams::g_fx 
              << ", fy: " << GlobalCameraParams::g_fy
              << ", cx: " << GlobalCameraParams::g_cx 
              << ", cy: " << GlobalCameraParams::g_cy << std::endl;
    std::cout << "Distortion: k2=" << GlobalCameraParams::g_k2 
              << ", k3=" << GlobalCameraParams::g_k3 << std::endl;
    
    Eigen::Vector3f translation = GlobalCameraParams::g_T_camera_lidar.block<3,1>(0,3);
    Eigen::Matrix3f rotation = GlobalCameraParams::g_T_camera_lidar.block<3,3>(0,0);
    std::cout << "Extrinsics:" << std::endl;
    std::cout << "  Translation: [" << translation.x() << ", " 
              << translation.y() << ", " << translation.z() << "]" << std::endl;
    std::cout << "  Rotation (euler angles): " 
              << rotation.eulerAngles(0,1,2).transpose() * 180/M_PI << "°" << std::endl;
    
    return true;
}
void rawCloudRender::render(std::vector<std::vector<float>>& rgb_image, 
                           capture_Image_List_t* pcd_stream, 
                           int pcdIdx, 
                           std::vector<float>& rgbCloud_flat) 
{
    // Initialize constants
    constexpr float inv_1000 = 0.001f;
    const float fx = GlobalCameraParams::g_fx;
    const float fy = GlobalCameraParams::g_fy;
    const float cx = GlobalCameraParams::g_cx;
    const float cy = GlobalCameraParams::g_cy;
    const float skew = GlobalCameraParams::g_skew;
    const float k2 = GlobalCameraParams::g_k2;
    const float k3 = GlobalCameraParams::g_k3;
    const float k4 = GlobalCameraParams::g_k4;
    const float k5 = GlobalCameraParams::g_k5;
    const float k6 = GlobalCameraParams::g_k6;
    const float k7 = GlobalCameraParams::g_k7;
    const Eigen::Matrix4f& T = GlobalCameraParams::g_T_camera_lidar;
    
    // Precompute matrix elements
    const float T00 = T(0,0), T01 = T(0,1), T02 = T(0,2), T03 = T(0,3);
    const float T10 = T(1,0), T11 = T(1,1), T12 = T(1,2), T13 = T(1,3);
    const float T20 = T(2,0), T21 = T(2,1), T22 = T(2,2), T23 = T(2,3);
    
    // Initialize lookup table
    static std::array<float, 10000> dist_table;
    static bool table_init = [&](){
        for (size_t i=0; i<dist_table.size(); ++i) {
            float theta = i * (M_PI/2) / dist_table.size();
            dist_table[i] = theta*(1 + theta*(k2 + theta*(k3 + theta*(k4 + theta*(k5 + theta*(k6 + theta*k7))))));
        }
        return true;
    }();
    
    // Get point cloud data (direct access)
    if (!pcd_stream || pcdIdx < 0 || pcdIdx >= 10) {
        std::cerr << "ERROR: Invalid pcd_stream or index in render function" << std::endl;
        return;
    }
    
    buffer_List_t& pcd_buffer = pcd_stream->imageList[pcdIdx];
    const int total_points = pcd_buffer.height * pcd_buffer.width;
    
    if (!pcd_buffer.pAddr) {
        std::cerr << "ERROR: Null point cloud data pointer in render function" << std::endl;
        return;
    }
    
    float* data = static_cast<float*>(pcd_buffer.pAddr);
    
    // Prepare output
    rgbCloud_flat.clear();
    rgbCloud_flat.resize(total_points * 4); // Preallocate maximum space
    float* output_ptr = rgbCloud_flat.data();
    
    // Get image dimensions
    const int img_height = 1296;
    const int img_width = 1600;
    
    // Process point cloud
    int valid_count = 0;
    for (int idx = 0; idx < total_points; ++idx) 
    {
        float* pf = data + idx*4;
        
        // Quick check for invalid points
        if (std::abs(pf[0]) < 1e-5f && std::abs(pf[1]) < 1e-5f && std::abs(pf[2]) < 1e-5f) {
            continue;
        }
        
        // Coordinate transformation
        const float x = pf[2] * inv_1000;
        const float y = -pf[0] * inv_1000;
        const float z = pf[1] * inv_1000;
        
        // Manual matrix transformation
        const float x1 = T00*x + T01*y + T02*z + T03;
        const float y1 = T10*x + T11*y + T12*z + T13;
        const float z1 = T20*x + T21*y + T22*z + T23;
        
        // Check for points behind camera
        if (z1 <= 0.0f) continue;
        
        // Calculate projection
        const float x1_sq = x1*x1;
        const float y1_sq = y1*y1;
        const float z1_sq = z1*z1;
        
        const float norm = std::sqrt(x1_sq + y1_sq + z1_sq);
        if (norm < 1e-7f) continue;
        
        const float r = std::sqrt(x1_sq + y1_sq);
        if (r < 1e-7f) continue;
        
        const float cost = z1 / norm;
        const float theta = std::acos(cost);
        
        // Safe table lookup
        const size_t table_idx = static_cast<size_t>(theta * (2.0f/M_PI) * dist_table.size());
        const size_t safe_idx = std::min(table_idx, dist_table.size()-1);
        const float thetad = dist_table[safe_idx];
        
        const float scaling = thetad / r;
        const float xd = x1 * scaling;
        const float yd = y1 * scaling;
        const float pd_2d_x = xd * fx + yd * skew + cx;
        const float pd_2d_y = yd * fy + cy;
        
        // Quick boundary check
        const int u = static_cast<int>(pd_2d_x);
        const int v = static_cast<int>(pd_2d_y);
        
        // Strict boundary check
        if (u < 0 || u >= img_width || v < 0 || v >= img_height) {
            continue;
        }
        
        // Safe image data access
        if (v < static_cast<int>(rgb_image.size()) && u < static_cast<int>(rgb_image[v].size())) {
            *output_ptr++ = x;
            *output_ptr++ = y;
            *output_ptr++ = z;
            *output_ptr++ = rgb_image[v][u];
            valid_count++;
        } else {
            // Handle invalid coordinates
            static bool warned = false;
            if (!warned) {
            	if(raw_debug)
            	{
            	          std::cerr << "WARNING: Invalid image coordinates: u=" << u << ", v=" << v 
                          << " (image size: " << rgb_image.size() << "x" 
                          << (rgb_image.empty() ? 0 : rgb_image[0].size()) << ")" << std::endl;
            	}

                warned = true;
            }
        }
    }
       // Resize output
    	rgbCloud_flat.resize(output_ptr - rgbCloud_flat.data());
        if(raw_debug)
         {
		std::cout << "Render completed: " << total_points << " points processed, " 
			  << valid_count << " valid points (" 
			  << (100.0 * valid_count / total_points) << "%)" << std::endl;
         }
}
void rawCloudRender::print_camera_calib() {
    std::cout << model_type_ << std::endl;
    std::cout << image_width_ << std::endl;
    std::cout << image_height_ << std::endl;

    std::cout << "T_camera_lidar" << std::endl;
    std::cout << T_camera_lidar_(0,0) << " " << T_camera_lidar_(0,1) << " " << T_camera_lidar_(0,2) << " " << T_camera_lidar_(0,3) << std::endl;
    std::cout << T_camera_lidar_(1,0) << " " << T_camera_lidar_(1,1) << " " << T_camera_lidar_(1,2) << " " << T_camera_lidar_(1,3) << std::endl;
    std::cout << T_camera_lidar_(2,0) << " " << T_camera_lidar_(2,1) << " " << T_camera_lidar_(2,2) << " " << T_camera_lidar_(2,3) << std::endl;
    std::cout << T_camera_lidar_(3,0) << " " << T_camera_lidar_(3,1) << " " << T_camera_lidar_(3,2) << " " << T_camera_lidar_(3,3) << std::endl;

    std::cout << "cam" << std::endl;
    std::cout << k2_ << std::endl;
    std::cout << k3_ << std::endl;
    std::cout << k4_ << std::endl;
    std::cout << k5_ << std::endl;
    std::cout << k6_ << std::endl;
    std::cout << k7_ << std::endl;

    std::cout << A11_fx_ << std::endl;
    std::cout << A12_skew_ << std::endl;
    std::cout << A22_fy_ << std::endl;
    std::cout << u0_cx_ << std::endl;
    std::cout << v0_cy_ << std::endl;    
}