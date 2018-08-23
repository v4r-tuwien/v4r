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
 * @file surf_local_estimator.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */
#pragma once

#include <v4r/config.h>
#include <v4r/features/FeatureDetector.h>
#include <v4r/features/types.h>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>

#if CV_VERSION_MAJOR < 3
#include <opencv2/nonfree/features2d.hpp>  // requires OpenCV non-free module
#else
#include <opencv2/xfeatures2d.hpp>
#endif

namespace po = boost::program_options;

namespace v4r {

class V4R_EXPORTS FeatureDetector_KD_SURF : public FeatureDetector {
  using FeatureDetector::descr_name_;

 public:
  struct V4R_EXPORTS Parameter {
    double hessianThreshold_ =
        400;  ///< Threshold for the keypoint detector. Only features, whose hessian is larger than
    ///< hessianThreshold are retained by the detector. Therefore, the larger the value,
    ///< the less keypoints you will get. A good default value could be from 300 to 500,
    ///< depending from the image contrast.
    int nOctaves_ = 4;  ///< 	Number of pyramid octaves the keypoint detector will use.  If you want to get very large
    ///< features, use the larger value. If you want just small features, decrease it.
    int nOctaveLayers_ = 3;  ///< Number of octave layers within each octave.
    bool extended_ = false;  ///< 0 means that the basic descriptors (64 elements each) shall be computed. 1 means that
    ///< the extended descriptors (128 elements each) shall be computed
    bool upright_ =
        false;  ///< 0  means that detector computes orientation of each feature. 1 means that the orientation
    ///< is not computed (which is much, much faster). For example, if you match images from a
    ///< stereo pair, or do image stitching, the matched features likely have very similar angles,
    ///< and you can speed up feature extraction by setting upright=1.

    Parameter() {}

    /**
     * @brief initialization function for the parameter
     * @param desc boost program options
     */
    void init(po::options_description &desc, const std::string &section_name = "surf") {
      desc.add_options()((section_name + ".hessianThreshold").c_str(),
                         po::value<double>(&hessianThreshold_)->default_value(hessianThreshold_),
                         "Threshold for the keypoint detector. Only features, whose hessian is larger than "
                         "hessianThreshold are retained by the detector. Therefore, the larger the value, the less "
                         "keypoints you will get. A good default value could be from 300 to 500, depending from the "
                         "image contrast.");
      desc.add_options()((section_name + ".nOctaves").c_str(), po::value<int>(&nOctaves_)->default_value(nOctaves_),
                         "Number of pyramid octaves the keypoint detector will use");
      desc.add_options()((section_name + ".nOctaveLayers").c_str(),
                         po::value<int>(&nOctaveLayers_)->default_value(nOctaveLayers_),
                         "Default number of sublevels per scale level");
      desc.add_options()((section_name + ".extended").c_str(), po::value<bool>(&extended_)->default_value(extended_),
                         "0 means that the basic descriptors (64 elements each) shall be computed. 1 means that the "
                         "extended descriptors (128 elements each) shall be computed");
      desc.add_options()(
          (section_name + ".upright").c_str(), po::value<bool>(&upright_)->default_value(upright_),
          "0  means that detector computes orientation of each feature. 1 means that the orientation is "
          "not computed (which is much, much faster). For example, if you match images from a stereo "
          "pair, or do image stitching, the matched features likely have very similar angles, and you can "
          "speed up feature extraction by setting upright=1.");
    }
  };

 private:
  Parameter param_;

#if CV_VERSION_MAJOR < 3
  cv::Ptr<cv::SURF> surf_;
#else
  cv::Ptr<cv::Feature2D> surf_;
#endif
  cv::Mat_<unsigned char> im_gray_;

 public:
  FeatureDetector_KD_SURF(const Parameter &p = Parameter());

  void detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors,
                        const cv::Mat &object_mask = cv::Mat()) override final;
  void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
              const cv::Mat &object_mask = cv::Mat()) override final;
  void compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) override final;

  typedef std::shared_ptr<FeatureDetector_KD_SURF> Ptr;
  typedef std::shared_ptr<FeatureDetector_KD_SURF const> ConstPtr;
};
}  // namespace v4r
