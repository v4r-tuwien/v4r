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
** For licensing terms and conditions please contact
** office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file img_loader.cpp
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */

#include <glog/logging.h>
#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION < 3
#include <opencv2/highgui/highgui.hpp>
#else
#include <opencv2/imgcodecs/imgcodecs.hpp>
#endif

#include <v4r/common/img_loader.h>

namespace v4r {
ImgContainer::ImgContainer() {
  path_ = "";
  ext_ = "";
}

ImgContainer::ImgContainer(std::string path, std::string ext) {
  setPathToImages(path, ext);
  loadImages();
}

void ImgContainer::setPathToImages(std::string path, std::string ext) {
  path_ = path;
  ext_ = ext;
}

bool ImgContainer::loadImages() {
  imgs_.clear();
  filenames_.clear();

  cv::String pattern = path_ + "/*";
  if (!ext_.empty()) {
    pattern = pattern + ext_;
  }

  cv::glob(pattern, filenames_);

  for (size_t i = 0; i < filenames_.size(); i++) {
    LOG(INFO) << "Loading rgb image " << filenames_[i] << "\r";
    cv::Mat img_temp;
#if CV_MAJOR_VERSION < 3
    img_temp = cv::imread(filenames_[i].c_str(), CV_LOAD_IMAGE_UNCHANGED);  // CV_LOAD_IMAGE_ANYDEPTH
#else
    img_temp = cv::imread(filenames_[i].c_str(), cv::IMREAD_UNCHANGED);
#endif

    if (!img_temp.empty()) {
      imgs_.push_back(img_temp);
    }
  }
  return !imgs_.empty();
}

const std::vector<cv::Mat>& ImgContainer::getImages() const {
  return imgs_;
}

const std::vector<cv::String>& ImgContainer::getFileNames() const {
  return filenames_;
}

size_t ImgContainer::getNumImgs() const {
  return imgs_.size();
}
}  // namespace v4r
