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
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace mini_vikit {

using namespace Eigen;

class PolynomialCamera {
private:
    const double fx_, fy_;
    const double cx_, cy_;
    const double skew_; 
    bool distortion_; 
    double k2_, k3_, k4_, k5_, k6_, k7_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PolynomialCamera(double width, double height,
                    double fx, double fy, double cx, double cy, double skew,
                    double k2=0.0, double k3=0.0, double k4=0.0, 
                    double k5=0.0, double k6=0.0, double k7=0.0)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), skew_(skew),
          distortion_(std::abs(k2) > 1e-7) {
        k2_ = k2; k3_ = k3; k4_ = k4; k5_ = k5; k6_ = k6; k7_ = k7;
    }

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

    Vector3d cam2world(const Vector2d& px) const {
        return cam2world(px[0], px[1]);
    }

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

    double fx() const { return fx_; }
    double fy() const { return fy_; }
    double cx() const { return cx_; }
    double cy() const { return cy_; }
    double skew() const { return skew_; }
    bool has_distortion() const { return distortion_; }
    
    double k2() const { return k2_; }
    double k3() const { return k3_; }
    double k4() const { return k4_; }
    double k5() const { return k5_; }
    double k6() const { return k6_; }
    double k7() const { return k7_; }
};
} // namespace mini_vikit
