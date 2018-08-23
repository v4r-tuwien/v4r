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

#include <cstdio>
#include <fstream>
#include <iostream>

#include <boost/algorithm/string/predicate.hpp>

#include <glog/logging.h>

#include <opencv2/opencv.hpp>

#include <v4r/common/intrinsics.h>

namespace v4r {

void Intrinsics::adjustToSize(unsigned int new_width, unsigned int new_height) {
  if (new_width == w && new_height == h)
    return;

  const float downsampling_ratio = static_cast<float>(new_width) / w;  // assume that always
  // all pixel horizontally are used for creating image
  fx *= downsampling_ratio;
  fy *= downsampling_ratio;
  cx *= downsampling_ratio;
  cy *= downsampling_ratio;

  // check if aspect ratio is kept the same
  if (w * new_height != new_width * h) {
    LOG(WARNING) << "New aspect ratio (" << new_width << "x" << new_height
                 << ") is different than the current aspect ratio (" << w << "x" << h
                 << ")in the RGB calibration file. Will assume only vertical FOV has changed. The center of the FOV "
                    "and the horizontal FOV are assumed to stay constant.";

    // assume that only top and bottom pixel are turned off.
    int new_height_according_to_old_aspect_ratio = new_width * h / static_cast<float>(w);
    int difference = new_height_according_to_old_aspect_ratio - new_height;
    cy -= difference / 2.f;
  }
  w = new_width;
  h = new_height;
}

cv::Mat Intrinsics::getCameraMatrix() const {
  return cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1;
}

Intrinsics Intrinsics::fromCameraMatrixAndResolution(cv::InputArray _camera_matrix, unsigned int w, unsigned int h) {
  if (_camera_matrix.size() != cv::Size(3, 3))
    throw std::runtime_error("Camera matrix should have 3x3 size");
  if (_camera_matrix.type() != CV_32FC1 && _camera_matrix.type() != CV_64FC1)
    throw std::runtime_error("Camera matrix should be single- or double-precision floating point");

  cv::Mat_<float> m;
  _camera_matrix.getMat().convertTo(m, CV_32FC1);

  return {m(0, 0), m(1, 1), m(0, 2), m(1, 2), w, h};
}

Intrinsics Intrinsics::load(const std::string& filename) {
  Intrinsics intr;
  if (boost::algorithm::ends_with(filename, ".yaml") || boost::algorithm::ends_with(filename, ".yml") ||
      boost::algorithm::ends_with(filename, ".xml")) {
    // OpenCV storage format
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    cv::Mat m;
    for (const auto& name : {"cameraMatrix", "camera_matrix", "matrix"}) {
      if (!fs[name].empty()) {
        fs[name] >> m;
        break;
      }
    }
    if (m.empty())
      throw std::runtime_error("intrinsics file does not contain camera matrix");
    intr.fx = m.at<double>(0, 0);
    intr.fy = m.at<double>(1, 1);
    intr.cx = m.at<double>(0, 2);
    intr.cy = m.at<double>(1, 2);
    int width = 0, height = 0;
    if (!fs["image_width"].empty())
      fs["image_width"] >> width;
    if (!fs["image_height"].empty())
      fs["image_height"] >> height;
    if (!fs["resolution"].empty()) {
      std::cout << "got resolution" << std::endl;
      auto node = fs["resolution"].begin();
      (*node++)["width"] >> width;
      (*node)["height"] >> height;
    }
    if (width == 0 || height == 0)
      throw std::runtime_error("intrinsics file does not contain image width/height");
    else
      intr.w = width, intr.h = height;
    return intr;
  } else {
    // V4R or COLMAP format
    std::ifstream file(filename);
    if (file.is_open()) {
      std::string line;
      int camera_id;
      while (std::getline(file, line)) {
        if (line.size() == 0 || line[0] == '#')
          continue;
        if (sscanf(line.c_str(), "%f %f %f %f %d %d", &intr.fx, &intr.fy, &intr.cx, &intr.cy, &intr.w, &intr.h) == 6)
          return intr;
        if (sscanf(line.c_str(), "%d OPENCV %d %d %f %f %f %f", &camera_id, &intr.w, &intr.h, &intr.fx, &intr.fy,
                   &intr.cx, &intr.cy) == 7)
          return intr;
        throw std::runtime_error("invalid intrinsics file format");
      }
    }
  }
  throw std::runtime_error("unable to read intrinsics file");
  return intr;
}

Intrinsics Intrinsics::PrimeSense() {
  return {525.0f, 525.0f, 319.5f, 239.0f, 640, 480};
}

std::ostream& operator<<(std::ostream& os, const Intrinsics& obj) {
  os << obj.fx << ", " << obj.fy << "  " << obj.cx << ", " << obj.cy << " " << obj.w << "x" << obj.h;
  return os;
}
}  // namespace v4r
