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

#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>

#include <v4r/core/macros.h>

namespace v4r {

struct V4R_EXPORTS Intrinsics {
  float fx, fy;       ///< focal length
  float cx, cy;       ///< center
  unsigned int w, h;  ///< image size in pixels

  /// Get a 3x3 camera matrix.
  cv::Mat getCameraMatrix() const;

  /**
   * @brief adjust intrinsic parameters to a new image size. If aspect ratio differs to current aspect ratio, it will
   * assume that only vertical field of view has changed. The center of the FOV and the horizontal FOV are assumed to
   * stay constant.
   * @param new_width new image size width
   * @param new_height new image size height
   */
  void adjustToSize(unsigned int new_width, unsigned int new_height);

  /// Construct intrinsics from a 3x3 camera matrix and resolution.
  static Intrinsics fromCameraMatrixAndResolution(cv::InputArray camera_matrix, unsigned int w, unsigned int h);

  /// Load camera intrinsics from a file.
  /// Three file formats are supported:
  ///  - native V4R format (FX FY CX CY W H)
  ///  - COLMAP format with a single OpenCV camera (ID OPENCV W H FX FY CX CY)
  ///  - OpenCV calibration (yaml or xml)
  static Intrinsics load(const std::string& filename);

  /// Get default intrinsics for a PrimeSense camera.
  static Intrinsics PrimeSense();
};

V4R_EXPORTS std::ostream& operator<<(std::ostream& os, const Intrinsics& obj);

}  // namespace v4r
