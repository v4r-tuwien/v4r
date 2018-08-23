/****************************************************************************
**
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
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

/**
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at), Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#include <v4r/features/FeatureDetector.h>
#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace po = boost::program_options;

namespace v4r {

class V4R_EXPORTS FeatureDetector_KD_ORB : public FeatureDetector {
 public:
  class V4R_EXPORTS Parameter {
   public:
    int nfeatures;      ///< The maximum number of features to retain.
    float scaleFactor;  ///< Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where
                        ///< each next level has 4x less pixels than the previous, but such a big scale factor will
                        ///< degrade feature matching scores dramatically. On the other hand, too close to 1 scale
                        ///< factor will mean that to cover certain scale range you will need more pyramid levels and so
                        ///< the speed will suffer.
    int nlevels;        ///< The number of pyramid levels. The smallest level will have linear size equal to
                        ///< input_image_linear_size/pow(scaleFactor, nlevels).
    int patchSize;      ///< 	size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid
                        ///< layers the perceived image area covered by a feature will be larger.
    Parameter(int _nfeatures = 1000, float _scaleFactor = 1.44, int _nlevels = 2, int _patchSize = 17)
    : nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels), patchSize(_patchSize) {}

    /**
     * @brief init parameters
     * @param command_line_arguments (according to Boost program options library)
     * @param section_name section name of program options
     */
    void init(boost::program_options::options_description &desc, const std::string &section_name = "orb");
  };

 private:
  using FeatureDetector::descr_name_;
  Parameter param;

  cv::Ptr<cv::ORB> orb;
  cv::Mat_<unsigned char> im_gray_;

 public:
  FeatureDetector_KD_ORB(const Parameter &_p = Parameter());
  ~FeatureDetector_KD_ORB();

  void detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors,
                        const cv::Mat &object_mask = cv::Mat()) override final;
  void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
              const cv::Mat &object_mask = cv::Mat()) override final;
  void compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) override final;

  typedef std::shared_ptr<FeatureDetector_KD_ORB> Ptr;
  typedef std::shared_ptr<FeatureDetector_KD_ORB const> ConstPtr;
};

}  // namespace v4r
