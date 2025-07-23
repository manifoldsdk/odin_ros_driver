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
#ifndef RGBCLOUD_H
#define RGBCLOUD_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include "lidar_api.h"

namespace GlobalCameraParams {
    extern float g_fx;
    extern float g_fy;
    extern float g_cx;
    extern float g_cy;
    extern float g_skew;
    extern float g_k2;
    extern float g_k3;
    extern float g_k4;
    extern float g_k5;
    extern float g_k6;
    extern float g_k7;
    extern Eigen::Matrix4f g_T_camera_lidar;
}

class rawCloudRender {
public:
  

    bool init(const std::string& yamlFilePath);
    
    void nv12buffer_2_rgb(buffer_List_t &image, std::vector<std::vector<float>>& rgb_image);
    void render(std::vector<std::vector<float>>& rgb_image, capture_Image_List_t* pcdStream, int pcdIdx, std::vector<float>& rgbCloud_flat);
    
    void print_camera_calib();
    
    int getImageWidth() const { return image_width_; }
    int getImageHeight() const { return image_height_; }
    
   

private:
    std::string model_type_;
    std::string camera_name_;
    int image_width_;
    int image_height_;
    int frame_size_;
    bool opencv_available_;
    
    // 4x4 transformation matrix (T_camera_lidar)
    Eigen::Matrix4f T_camera_lidar_;

    float k2_;
    float k3_;
    float k4_;
    float k5_;
    float k6_;
    float k7_;
    float p1_;
    float p2_;
    float A11_fx_;
    float A12_skew_;
    float A22_fy_;
    float u0_cx_;
    float v0_cy_;
    bool isFast_;
    int numDiff_;
    float maxIncidentAngle_; 
    

};

#endif  // RGBCLOUD_H
