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

/**
 * @file polynomial_camera.hpp
 * @brief 多项式畸变相机模型
 *
 * 实现基于多项式的鱼眼/广角相机模型，支持：
 * - 像素坐标 → 归一化三维方向向量（cam2world）
 * - 三维点 → 像素坐标（world2cam）
 * 畸变模型使用 7 阶多项式（k2~k7）。
 */

#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace mini_vikit {

using namespace Eigen;

/**
 * @brief 多项式畸变相机模型类
 *
 * 支持有畸变和无畸变两种模式，构造时自动检测。
 */
class PolynomialCamera {
private:
    const double fx_, fy_;   ///< 焦距
    const double cx_, cy_;   ///< 主点坐标
    const double skew_;      ///< 倾斜系数
    bool distortion_;        ///< 是否启用畸变矫正
    double k2_, k3_, k4_, k5_, k6_, k7_; ///< 多项式畸变系数

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 构造函数
     * @param width  图像宽度（像素）
     * @param height 图像高度（像素）
     * @param fx, fy 焦距
     * @param cx, cy 主点
     * @param skew   倾斜系数
     * @param k2~k7  多项式畸变系数（默认 0.0 表示无畸变）
     */
    PolynomialCamera(double width, double height,
                    double fx, double fy, double cx, double cy, double skew,
                    double k2=0.0, double k3=0.0, double k4=0.0,
                    double k5=0.0, double k6=0.0, double k7=0.0)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), skew_(skew),
          distortion_(std::abs(k2) > 1e-7) {
        k2_ = k2; k3_ = k3; k4_ = k4; k5_ = k5; k6_ = k6; k7_ = k7;
    }

    /**
     * @brief 像素坐标转归一化三维方向向量（考虑畸变，迭代法求解）
     * @param u, v 像素坐标
     * @return 归一化的三维方向向量
     */
    Vector3d cam2world(const double& u, const double& v) const {
        Vector3d xyz;
        if (!distortion_) {
            double y = (v - cy_) / fy_;
            double x = (u - cx_ - y * skew_) / fx_;
            xyz << x, y, 1.0;
        } else {
            double y = (v - cy_) / fy_;
            double x = (u - cx_ - y * skew_) / fx_;

            const double thetad = std::sqrt(x * x + y * y);
            double theta = thetad;

            // 牛顿迭代法反解 theta（7 次迭代）
            for (int i = 0; i < 7; ++i) {
                const double theta2 = theta * theta;
                const double theta3 = theta2 * theta;
                const double theta4 = theta3 * theta;
                const double theta5 = theta4 * theta;
                const double theta6 = theta5 * theta;
                theta = thetad / (1.0 + k2_ * theta + k3_ * theta2 + k4_ * theta3 +
                                 k5_ * theta4 + k6_ * theta5 + k7_ * theta6);
            }

            const double scaling = std::tan(theta) / thetad;
            x *= scaling;
            y *= scaling;
            xyz << x, y, 1.0;
        }
        return xyz.normalized();
    }

    /// 像素坐标（Vector2d）转归一化三维方向向量
    Vector3d cam2world(const Vector2d& px) const {
        return cam2world(px[0], px[1]);
    }

    /**
     * @brief 三维点转像素坐标（正向投影，考虑多项式畸变）
     * @param xyz 三维点（相机坐标系）
     * @return 像素坐标
     */
    Vector2d world2cam(const Vector3d& xyz) const {
        Vector2d px;
        if (!distortion_) {
            px[0] = fx_ * xyz[0] + cx_;
            px[1] = fy_ * xyz[1] + cy_;
        } else {
            double xd, yd;
            const double r = std::sqrt(xyz(1) * xyz(1) + xyz(0) * xyz(0));
            const double theta = std::acos(xyz(2) / xyz.norm());
            const double thetad = thetad_from_theta(theta);
            const double scaling = thetad / r;
            xd = xyz[0] * scaling;
            yd = xyz[1] * scaling;
            px[0] = xd * fx_ + yd * skew_ + cx_;
            px[1] = yd * fy_ + cy_;
        }
        return px;
    }

    /**
     * @brief 归一化平面坐标（uv）转像素坐标
     * @param uv 归一化平面坐标（2D）
     * @return 像素坐标
     */
    Vector2d world2cam(const Vector2d& uv) const {
        Vector2d px;
        if (!distortion_) {
            px[0] = fx_ * uv[0] + cx_;
            px[1] = fy_ * uv[1] + cy_;
        } else {
            double xd, yd;
            const double r = uv.norm();
            if (r < 1e-8) {
                return uv;
            }
            const double theta = std::atan(r);
            const double thetad = thetad_from_theta(theta);
            const double scaling = thetad / r;
            xd = uv[0] * scaling;
            yd = uv[1] * scaling;
            px[0] = xd * fx_ + yd * skew_ + cx_;
            px[1] = yd * fy_ + cy_;
        }
        return px;
    }

    /// 根据入射角 theta 计算畸变后角度 thetad（多项式正向映射）
    inline double thetad_from_theta(const double theta) const {
        const double theta2 = theta * theta;
        const double theta3 = theta2 * theta;
        const double theta4 = theta3 * theta;
        const double theta5 = theta4 * theta;
        const double theta6 = theta5 * theta;
        const double theta7 = theta6 * theta;
        const double thetad = theta + k2_ * theta2 + k3_ * theta3 +
                             k4_ * theta4 + k5_ * theta5 + k6_ * theta6 + k7_ * theta7;
        return thetad;
    }

    double fx() const { return fx_; }    ///< 返回 fx
    double fy() const { return fy_; }    ///< 返回 fy
    double cx() const { return cx_; }    ///< 返回 cx
    double cy() const { return cy_; }    ///< 返回 cy
    double skew() const { return skew_; } ///< 返回倾斜系数
    bool has_distortion() const { return distortion_; } ///< 是否有畸变

    double k2() const { return k2_; } ///< 畸变系数 k2
    double k3() const { return k3_; } ///< 畸变系数 k3
    double k4() const { return k4_; } ///< 畸变系数 k4
    double k5() const { return k5_; } ///< 畸变系数 k5
    double k6() const { return k6_; } ///< 畸变系数 k6
    double k7() const { return k7_; } ///< 畸变系数 k7
};

} // namespace mini_vikit
