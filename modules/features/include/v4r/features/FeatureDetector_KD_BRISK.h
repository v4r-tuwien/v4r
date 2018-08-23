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
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2018
 * @brief OpenCV BRISK feature detector implementation
 *
 */

#pragma once

#include <v4r/config.h>
#include <v4r/features/FeatureDetector.h>
#include <boost/program_options.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace po = boost::program_options;

namespace v4r {

class V4R_EXPORTS FeatureDetector_KD_BRISK : public FeatureDetector {
 public:
  struct V4R_EXPORTS Parameter {
    int thresh_ = 30;            //< AGAST detection threshold score.
    int octaves_ = 3;            //< detection octaves. Use 0 to do single scale.
    float patternScale_ = 1.0f;  //< apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
    Parameter() {}

    /**
     * @brief init parameters
     * @param command_line_arguments (according to Boost program options library)
     * @param section_name section name of program options
     */
    void init(boost::program_options::options_description &desc, const std::string &section_name = "brisk");
  };

 private:
  using FeatureDetector::descr_name_;
  Parameter param_;
  cv::Ptr<cv::BRISK> brisk_;
  cv::Mat_<unsigned char> im_gray_;

 public:
  FeatureDetector_KD_BRISK(const Parameter &p = Parameter());
  ~FeatureDetector_KD_BRISK() {}

  void detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors,
                        const cv::Mat &object_mask = cv::Mat()) override final;
  void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
              const cv::Mat &object_mask = cv::Mat()) override final;
  void compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) override final;

  typedef std::shared_ptr<FeatureDetector_KD_BRISK> Ptr;
  typedef std::shared_ptr<FeatureDetector_KD_BRISK const> ConstPtr;
};
}  // namespace v4r
