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
 * @file calibration_params.cpp
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "glog/logging.h"

#include <v4r/calibration/calibration_params.h>

calibrationParams::calibrationParams(int _lattice_height, int _lattice_width)
: lattice_height_(_lattice_height), lattice_width_(_lattice_width) {}

void calibrationParams::setLatticeSize(int height, int width) {
  lattice_height_ = height;
  lattice_width_ = width;
}

void calibrationParams::setRGBImgSize(int height, int width) {
  rgb_height_ = height;
  rgb_width_ = width;
}

void calibrationParams::setDepthImgSize(int height, int width) {
  depth_height_ = height;
  depth_width_ = width;
}

void calibrationParams::setDepthParams(double* param) {
  std::copy(param, param + 8, depth_camera_.begin());
}

void calibrationParams::setRGBParams(double* param) {
  std::copy(param, param + 8, rgb_camera_.begin());
}

void calibrationParams::setAParams(double* param) {
  std::copy(param, param + 2, a_.begin());
}

void calibrationParams::setLatticeParams(double* param) {
  lattice_.clear();
  for (size_t i = 0; i < (size_t)(lattice_height_ * lattice_width_); ++i) {
    lattice_.push_back(param[i]);
  }
}

void calibrationParams::setTrelParams(double* param) {
  std::copy(param, param + 6, Trel_.begin());
}

void calibrationParams::setScaleParams(double* param) {
  s_ = *param;
}

void calibrationParams::getRGBImgSize(int& height, int& width) const {
  height = rgb_height_;
  width = rgb_width_;
}

void calibrationParams::getLatticeSize(int& height, int& width) const {
  height = lattice_height_;
  width = lattice_width_;
}

void calibrationParams::getDepthImgSize(int& height, int& width) const {
  height = depth_height_;
  width = depth_width_;
}

void calibrationParams::getDepthParams(double* param) const {
  std::copy(depth_camera_.begin(), depth_camera_.end(), param);
}

void calibrationParams::getRGBParams(double* param) const {
  std::copy(rgb_camera_.begin(), rgb_camera_.end(), param);
}

void calibrationParams::getAParams(double* param) const {
  std::copy(a_.begin(), a_.end(), param);
}

const std::vector<double>& calibrationParams::getLatticeParams() const {
  return lattice_;
}

void calibrationParams::getTrelParams(double* param) const {
  std::copy(Trel_.begin(), Trel_.end(), param);
}

void calibrationParams::getScaleParams(double* param) const {
  *param = s_;
}

void calibrationParams::printParams(void) const {
  LOG(INFO) << std::endl
            << "OPTIMIZATION PARAMETERS" << std::endl
            << std::endl
            << "RGB CAMERA:" << std::endl
            << "[fx,fy]:\t\t" << rgb_camera_[4] << " , " << rgb_camera_[5] << std::endl
            << "[px,py]:\t\t" << rgb_camera_[6] << " , " << rgb_camera_[7] << std::endl
            << "[k1,k2,k3,k4]:\t\t" << rgb_camera_[0] << " , " << rgb_camera_[1] << " ," << rgb_camera_[2] << " ,"
            << rgb_camera_[3] << std::endl
            << "[height,width]: \t \t" << rgb_height_ << " , " << rgb_width_ << std::endl

            << std::endl
            << std::endl
            << "DEPTH CAMERA:" << std::endl
            << "[fx,fy]:\t\t" << depth_camera_[4] << " , " << depth_camera_[5] << std::endl
            << "[px,py]:\t\t" << depth_camera_[6] << " , " << depth_camera_[7] << std::endl
            << "[k1,k2,k3,k4]:\t\t" << depth_camera_[0] << " , " << depth_camera_[1] << " ," << depth_camera_[2] << " ,"
            << depth_camera_[3] << std::endl
            << "[height,width]: \t \t" << depth_height_ << " , " << depth_width_ << std::endl

            << std::endl
            << "T_REL:" << std::endl
            << "[tx,ty,tz]:\t\t" << Trel_[3] << " , " << Trel_[4] << " , " << Trel_[5] << std::endl
            << "[rx,ry,rz]:\t\t" << Trel_[0] << " , " << Trel_[1] << " , " << Trel_[2] << std::endl

            << std::endl
            << "a:" << std::endl
            << "[a0,a1]:\t\t" << a_[0] << " , " << a_[1] << std::endl

            << std::endl
            << "Scale:" << std::endl
            << "[s]:\t\t\t" << s_ << std::endl
            << std::endl

            << "Lattice" << std::endl
            << "[height]:\t\t" << lattice_height_ << std::endl
            << "[width]:\t\t" << lattice_width_ << std::endl
            << std::endl

            << std::endl

            << std::endl

            << "***************************************************************"
               "*******"
            << std::endl
            << std::endl;
}

void calibrationParams::saveParams(std::string path) const {
  std::ofstream file;
  try {
    file.open(path, std::ios::out);

    std::ostringstream strs;
    strs << std::endl
         << "******************************************************************"
            "****"
         << std::endl
         << std::endl
         << "RGB CAMERA:" << std::endl
         << "[fx,fy]:\t\t" << rgb_camera_[4] << " , " << rgb_camera_[5] << std::endl
         << "[px,py]:\t\t" << rgb_camera_[6] << " , " << rgb_camera_[7] << std::endl
         << "[k1,k2,k3,k4]:\t" << rgb_camera_[0] << " , " << rgb_camera_[1] << " , " << rgb_camera_[2] << " , "
         << rgb_camera_[3] << std::endl
         << "[height,width]: \t \t" << rgb_height_ << " , " << rgb_width_ << std::endl

         << std::endl
         << std::endl
         << "DEPTH CAMERA:" << std::endl
         << "[fx,fy]:\t\t" << depth_camera_[4] << " , " << depth_camera_[5] << std::endl
         << "[px,py]:\t\t" << depth_camera_[6] << " , " << depth_camera_[7] << std::endl
         << "[k1,k2,k3,k4]:\t" << depth_camera_[0] << " , " << depth_camera_[1] << " , " << depth_camera_[2] << " , "
         << depth_camera_[3] << std::endl
         << "[height,width]: \t \t" << depth_height_ << " , " << depth_width_ << std::endl

         << std::endl
         << "T_REL:" << std::endl
         << "[tx,ty,tz]:\t\t" << Trel_[3] << " , " << Trel_[4] << " , " << Trel_[5] << std::endl
         << "[rx,ry,rz]:\t\t" << Trel_[0] << " , " << Trel_[1] << " , " << Trel_[2] << std::endl

         << std::endl
         << "a:" << std::endl
         << "[a0,a1]:\t\t" << a_[0] << " , " << a_[1] << std::endl

         << std::endl
         << "Scale:" << std::endl
         << "[s]:\t\t" << s_ << std::endl

         << "******************************************************************"
            "****"
         << std::endl;

    file << strs.str();
    file.close();

  } catch (std::exception const& e) {
    LOG(WARNING) << "There was an error while saving the optimization parameters "
                    "to a file: "
                 << e.what() << std::endl;
  }

  try {
    std::ofstream ofs;
    ofs.open(path + ".ser", std::ios::out);
    boost::archive::text_oarchive oa(ofs);
    oa << *this;
    ofs.close();
  } catch (std::exception const& e) {
    LOG(WARNING) << "There was an error during the serialization of the "
                    "optimization parameters: "
                 << e.what() << std::endl;
  }
}

//	camera_params[0]= 0.0001;//k1
//	camera_params[1] =0.0001;//k2
//	camera_params[2] =0.0001;//k3
//	camera_params[3] =0.0001;//k4
//	camera_params[4] =400;//fx
//	camera_params[5] =-400;//fy
//	camera_params[6] =0;//px
//	camera_params[7]  =0;//py
