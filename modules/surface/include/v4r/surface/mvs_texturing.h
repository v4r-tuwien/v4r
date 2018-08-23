/****************************************************************************
**
** Copyright (C) 2018 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

#pragma once

#include <boost/program_options.hpp>

#include <Eigen/Core>

#include <pcl/PolygonMesh.h>

#include <v4r/common/intrinsics.h>
#include <v4r/core/macros.h>

namespace v4r {
namespace surface {

class V4R_EXPORTS MVSTexturing {
 public:
  /// MVS-Texturing parameters.
  ///
  /// This is an adapter for Settings struct in MVS-Texturing.
  /// Note that the following parameters are not exposed and will stay at their default values:
  ///  * DataTerm (DATA_TERM_GMI)
  ///  * SmoothnessTerm (SMOOTHNESS_TERM_POTTS)
  ///  * OutlierRemoval (OUTLIER_REMOVAL_NONE)
  ///  * ToneMapping (TONE_MAPPING_NONE)
  /// There is no specific reason why they are not exposed, feel free to add them if needed.
  struct Parameters {
    bool geometric_visibility_test = true;
    bool global_seam_leveling = true;
    bool local_seam_leveling = true;
    bool hole_filling = true;
    bool keep_unseen_faces = false;

    void init(boost::program_options::options_description& desc, const std::string& section_name = "mvs_texturing");
  };

  MVSTexturing() = default;

  MVSTexturing(const Parameters& params) : params_(params) {}

  using PoseVector = std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>;

  /// Create a textured model using MVS-Texturing as a backend.
  ///
  /// \param[in] mesh model to be textured
  /// \param[in] camera_intrinsics camera intrinsics
  /// \param[in] camera_images a vector with BGR images of the model
  /// \param[in] camera_poses a vector of poses of the cameras
  /// \param[in] output_path where to save the textured model, without extension
  void mapTextureToMesh(const pcl::PolygonMesh& mesh, const v4r::Intrinsics& camera_intrinsics,
                        const std::vector<cv::Mat>& camera_images, const PoseVector& camera_poses,
                        const std::string& output_path);

  Parameters& getParameters() {
    return params_;
  }

  void setParameters(const Parameters& params) {
    params_ = params;
  }

 private:
  Parameters params_;
};

}  // namespace surface
}  // namespace v4r
