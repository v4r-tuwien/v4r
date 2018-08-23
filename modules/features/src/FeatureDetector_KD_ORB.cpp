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
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#include <v4r/features/FeatureDetector_KD_ORB.h>

namespace v4r {

void FeatureDetector_KD_ORB::Parameter::init(boost::program_options::options_description &desc,
                                             const std::string &section_name) {
  desc.add_options()((section_name + ".nfeatures").c_str(), po::value<int>(&nfeatures)->default_value(nfeatures),
                     "The maximum number of features to retain.");
  desc.add_options()(
      (section_name + ".scaleFactor").c_str(), po::value<float>(&scaleFactor)->default_value(scaleFactor),
      "Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has "
      "4x less pixels than the previous, but such a big scale factor will degrade feature matching scores "
      "dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you "
      "will need more pyramid levels and so the speed will suffer.");
  desc.add_options()((section_name + ".nlevels").c_str(), po::value<int>(&nlevels)->default_value(nlevels),
                     "The number of pyramid levels. The smallest level will have linear size equal to "
                     "input_image_linear_size/pow(scaleFactor, nlevels).");
  desc.add_options()((section_name + ".patchSize").c_str(), po::value<int>(&patchSize)->default_value(patchSize),
                     "\tsize of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers "
                     "the perceived image area covered by a feature will be larger.");
}

FeatureDetector_KD_ORB::FeatureDetector_KD_ORB(const Parameter &_p)
: FeatureDetector(FeatureDetector::Type::KD_ORB), param(_p) {
  descr_name_ = "orb";
  // orb = new cv::ORB(10000, 1.2, 6, 13, 0, 2, cv::ORB::HARRIS_SCORE, 13); //31
  // orb = new cv::ORB(1000, 1.44, 2, 17, 0, 2, cv::ORB::HARRIS_SCORE, 17);

#if CV_MAJOR_VERSION < 3
  orb = new cv::ORB(param.nfeatures, param.scaleFactor, param.nlevels, param.patchSize, 0, 2, cv::ORB::HARRIS_SCORE,
                    param.patchSize);
#else
  orb = cv::ORB::create(param.nfeatures, param.scaleFactor, param.nlevels, 31, 0, 2, cv::ORB::HARRIS_SCORE,
                        param.patchSize);
#endif
}

FeatureDetector_KD_ORB::~FeatureDetector_KD_ORB() {}

void FeatureDetector_KD_ORB::detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
                                              cv::Mat &descriptors, const cv::Mat &object_mask) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

#if CV_MAJOR_VERSION < 3
  (*orb)(im_gray_, object_mask, keys, descriptors);
#else
  orb->detectAndCompute(im_gray_, object_mask, keys, descriptors);
#endif
  computeKeypointIndices(im_gray_, keys);
}

void FeatureDetector_KD_ORB::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, const cv::Mat &object_mask) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  orb->detect(im_gray_, keys, object_mask);
  computeKeypointIndices(im_gray_, keys);
}

void FeatureDetector_KD_ORB::compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

#if CV_MAJOR_VERSION < 3
  (*orb)(im_gray_, cv::Mat(), keys, descriptors, true);
#else
  orb->detectAndCompute(im_gray_, cv::Mat(), keys, descriptors, true);
#endif
}
}  // namespace v4r
