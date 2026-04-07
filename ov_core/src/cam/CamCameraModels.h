/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_CORE_CAM_CAMERA_MODELS_H
#define OV_CORE_CAM_CAMERA_MODELS_H

#include <stdexcept>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>

#include "CamBase.h"
#include "camera_models/camodocal/camera_models/CameraFactory.h"
#include "camera_models/camodocal/camera_models/CataCamera.h"
#include "camera_models/camodocal/camera_models/EquidistantCamera.h"
#include "camera_models/camodocal/camera_models/PinholeCamera.h"

namespace ov_core {

class CamCameraModels : public CamBase {

public:
  explicit CamCameraModels(const camodocal::CameraPtr &camera)
      : CamBase(camera ? camera->imageWidth() : 0, camera ? camera->imageHeight() : 0), camera_(camera) {
    if (!camera_) {
      throw std::runtime_error("CamCameraModels requires a valid camera_models camera instance");
    }
    initialize_placeholder_values();
  }

  static std::shared_ptr<CamCameraModels> CreateFromYaml(const std::string &yaml_path) {
    camodocal::CameraPtr camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(yaml_path);
    if (!camera) {
      throw std::runtime_error("Failed to load camera_models calibration from: " + yaml_path);
    }
    return std::make_shared<CamCameraModels>(camera);
  }

  void set_value(const Eigen::MatrixXd &calib) override {
    if (calib.rows() == camera_values.rows()) {
      camera_values = calib;
    } else if (calib.rows() == 8) {
      camera_values = calib;
    } else {
      throw std::runtime_error("CamCameraModels::set_value received unsupported calibration dimension");
    }
  }

  Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) override {
    Eigen::Vector3d ray;
    camera_->liftProjective(uv_dist.cast<double>(), ray);
    Eigen::Vector2f pt_out;
    pt_out << static_cast<float>(ray(0) / ray(2)), static_cast<float>(ray(1) / ray(2));
    return pt_out;
  }

  Eigen::Vector2f distort_f(const Eigen::Vector2f &uv_norm) override {
    Eigen::Vector2d uv_dist;
    camera_->spaceToPlane(Eigen::Vector3d(uv_norm(0), uv_norm(1), 1.0), uv_dist);
    return uv_dist.cast<float>();
  }

  void compute_distort_jacobian(const Eigen::Vector2d &uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta) override {
    constexpr double eps = 1e-6;
    Eigen::Vector2d uv_x_plus = uv_norm;
    Eigen::Vector2d uv_x_minus = uv_norm;
    Eigen::Vector2d uv_y_plus = uv_norm;
    Eigen::Vector2d uv_y_minus = uv_norm;
    uv_x_plus(0) += eps;
    uv_x_minus(0) -= eps;
    uv_y_plus(1) += eps;
    uv_y_minus(1) -= eps;

    Eigen::Vector2d fx_plus = distort_d(uv_x_plus);
    Eigen::Vector2d fx_minus = distort_d(uv_x_minus);
    Eigen::Vector2d fy_plus = distort_d(uv_y_plus);
    Eigen::Vector2d fy_minus = distort_d(uv_y_minus);

    H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
    H_dz_dzn.col(0) = (fx_plus - fx_minus) / (2.0 * eps);
    H_dz_dzn.col(1) = (fy_plus - fy_minus) / (2.0 * eps);

    // Runtime-only camera_models support keeps intrinsics fixed.
    H_dz_dzeta = Eigen::MatrixXd::Zero(2, 8);
  }

  camodocal::Camera::ModelType model_type() const { return camera_->modelType(); }

private:
  void initialize_placeholder_values() {
    std::vector<double> params;
    camera_->writeParameters(params);
    camera_values = Eigen::VectorXd::Zero(8);

    switch (camera_->modelType()) {
    case camodocal::Camera::MEI: {
      // [xi, k1, k2, p1, p2, gamma1, gamma2, u0, v0] -> [fx, fy, cx, cy, k1, k2, p1, p2]
      if (params.size() >= 9) {
        camera_values << params[5], params[6], params[7], params[8], params[1], params[2], params[3], params[4];
      }
      break;
    }
    case camodocal::Camera::KANNALA_BRANDT: {
      // [k2, k3, k4, k5, mu, mv, u0, v0] -> [fx, fy, cx, cy, k1, k2, k3, k4]
      if (params.size() >= 8) {
        camera_values << params[4], params[5], params[6], params[7], params[0], params[1], params[2], params[3];
      }
      break;
    }
    case camodocal::Camera::PINHOLE:
    case camodocal::Camera::PINHOLE_FULL: {
      // [k1, k2, p1, p2, fx, fy, cx, cy] -> [fx, fy, cx, cy, k1, k2, p1, p2]
      if (params.size() >= 8) {
        camera_values << params[4], params[5], params[6], params[7], params[0], params[1], params[2], params[3];
      }
      break;
    }
    default:
      break;
    }

    camera_k_OPENCV = cv::Matx33d::eye();
    camera_k_OPENCV(0, 0) = camera_values(0);
    camera_k_OPENCV(1, 1) = camera_values(1);
    camera_k_OPENCV(0, 2) = camera_values(2);
    camera_k_OPENCV(1, 2) = camera_values(3);

    camera_d_OPENCV = cv::Vec4d(camera_values(4), camera_values(5), camera_values(6), camera_values(7));
  }

  camodocal::CameraPtr camera_;
};

} // namespace ov_core

#endif /* OV_CORE_CAM_CAMERA_MODELS_H */