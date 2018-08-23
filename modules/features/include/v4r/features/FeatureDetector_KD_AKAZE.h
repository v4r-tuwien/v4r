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
 * @file akaze_local_estimator.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */
#pragma once

#include <v4r/config.h>
#include <v4r/features/FeatureDetector.h>
#include <v4r/features/types.h>
#include <boost/program_options.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

namespace po = boost::program_options;

namespace v4r {

class V4R_EXPORTS FeatureDetector_KD_AKAZE : public FeatureDetector {
 public:
  struct V4R_EXPORTS Parameter {
    int descriptor_type_ = cv::AKAZE::DESCRIPTOR_MLDB;  ///< Type of the extracted descriptor: DESCRIPTOR_KAZE,
    ///< DESCRIPTOR_KAZE_UPRIGHT, DESCRIPTOR_MLDB or
    ///< DESCRIPTOR_MLDB_UPRIGHT.
    int descriptor_size_ = 0;            ///< Size of the descriptor in bits. 0 -> Full size
    int descriptor_channels_ = 3;        ///< Number of channels in the descriptor (1, 2, 3)
    float detector_threshold_ = 0.001f;  ///< Detector response threshold to accept point
    int nOctaves_ = 4;                   ///< 	Maximum octave evolution of the image
    int nOctaveLayers_ = 4;              ///< Default number of sublevels per scale level
    int diffusivity_ =
        cv::KAZE::DIFF_PM_G2;  ///< Diffusivity type. DIFF_PM_G1, DIFF_PM_G2, DIFF_WEICKERT or DIFF_CHARBONNIER

    Parameter() {}

    /**
     * @brief init parameters
     * @param command_line_arguments (according to Boost program options library)
     * @param section_name section name of program options
     */
    void init(boost::program_options::options_description &desc, const std::string &section_name = "akaze");
  };

 private:
  using FeatureDetector::descr_name_;
  Parameter param_;
  cv::Mat_<unsigned char> im_gray_;
  cv::Ptr<cv::AKAZE> akaze_;
  void printParams(std::ostream &s = std::cout);

 public:
  FeatureDetector_KD_AKAZE(const Parameter &p = Parameter());

  void detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors,
                        const cv::Mat &object_mask = cv::Mat()) override final;
  void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
              const cv::Mat &object_mask = cv::Mat()) override final;
  void compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) override final;

  typedef std::shared_ptr<FeatureDetector_KD_AKAZE> Ptr;
  typedef std::shared_ptr<FeatureDetector_KD_AKAZE const> ConstPtr;
};
}  // namespace v4r
