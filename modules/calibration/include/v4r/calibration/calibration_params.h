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
 * @file calibration_params.h
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */

#ifndef OPTIMIZATION_PARAMS_H
#define OPTIMIZATION_PARAMS_H

#include <v4r/core/macros.h>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

class V4R_EXPORTS calibrationParams {
 private:
  std::vector<double> lattice_;

  std::array<double, 8> rgb_camera_;
  std::array<double, 8> depth_camera_;
  std::array<double, 2> a_;

  std::array<double, 6> Trel_;
  double s_;

  int lattice_height_;
  int lattice_width_;

  int rgb_height_;
  int rgb_width_;

  int depth_height_;
  int depth_width_;

 public:
  calibrationParams(int _lattice_height, int _lattice_width);

  void setLatticeSize(int height, int width);
  void setRGBImgSize(int height, int width);
  void setDepthImgSize(int height, int width);
  void setDepthParams(double* param);
  void setRGBParams(double* param);
  void setAParams(double* param);
  void setLatticeParams(double* param);
  void setScaleParams(double* param);
  void setTrelParams(double* param);

  void getLatticeSize(int& height, int& width) const;
  void getRGBImgSize(int& height, int& width) const;
  void getDepthImgSize(int& height, int& width) const;
  void getDepthParams(double* param) const;
  void getRGBParams(double* param) const;
  void getAParams(double* param) const;
  const std::vector<double>& getLatticeParams() const;
  void getScaleParams(double* param) const;
  void getTrelParams(double* param) const;

  void printParams(void) const;
  void saveParams(std::string path) const;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    //        ar & version;
    ar& rgb_height_;
    ar& rgb_width_;

    ar& depth_height_;
    ar& depth_width_;

    ar& lattice_height_;
    ar& lattice_width_;
    ar& s_;

    if (lattice_.size() == 0) {
      lattice_.clear();
      lattice_.resize(lattice_height_ * lattice_width_);
    }

    for (int i = 0; i < lattice_height_ * lattice_width_; i++) {
      ar& lattice_[i];
    }
    for (unsigned int i = 0; i < 8; i++) {
      ar& depth_camera_[i];
    }
    for (unsigned int i = 0; i < 8; i++) {
      ar& rgb_camera_[i];
    }
    for (unsigned int i = 0; i < 6; i++) {
      ar& Trel_[i];
    }
    for (unsigned int i = 0; i < 2; i++) {
      ar& a_[i];
    }
  }
};

#endif
