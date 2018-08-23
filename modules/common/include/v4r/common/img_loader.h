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
 * @file img_loader.h
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */

#ifndef IMG_LOADER_H
#define IMG_LOADER_H

#include <opencv2/core/core.hpp>

#include <v4r/core/macros.h>

namespace v4r {

class V4R_EXPORTS ImgContainer {
 public:
  ImgContainer();
  ImgContainer(std::string path, std::string ext = "");

  void setPathToImages(std::string path, std::string ext = "");

  bool loadImages();
  const std::vector<cv::Mat>& getImages() const;
  const std::vector<cv::String>& getFileNames() const;
  size_t getNumImgs() const;

 protected:
  std::vector<cv::Mat> imgs_;
  std::vector<cv::String> filenames_;

 private:
  std::string path_;
  std::string ext_;
};
}  // namespace v4r

#endif
