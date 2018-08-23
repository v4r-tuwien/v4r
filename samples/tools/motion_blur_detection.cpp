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
 * @file motion_blur_detection.cpp
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */

#include <fstream>
#include <iostream>
#include <string>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <v4r/common/img_loader.h>
#include <v4r/common/motion_blur_detection.h>
namespace po = boost::program_options;

bool parseArgs(int argc, char** argv, std::string& input_folder, std::string& result,
               v4r::MotionBlurDetectionType& method);

int main(int argc, char** argv) {
  std::string img_folder;
  std::string result;
  v4r::MotionBlurDetectionType method;

  if (!parseArgs(argc, argv, img_folder, result, method)) {
    return 0;
  }

  if (std::system("clear") == -1) {
  }
  v4r::ImgContainer img_load(img_folder);
  std::vector<cv::Mat> imgs;
  imgs = img_load.getImages();

  std::vector<cv::String> filenames;
  filenames = img_load.getFileNames();

  if (!(imgs.size() > 0 && filenames.size() > 0)) {
    std::cout << "Wasn't able to load images from " + img_folder << std::endl;
    std::cout << "Aborting ..." << std::endl;
    return -1;
  }

  auto img_ptr = std::make_shared<std::vector<cv::Mat>>(imgs);
  v4r::MotionBlurDetection mb(img_ptr, method);
  mb.calculateMotionBlurForImgVector();
  std::vector<double> bluriness;
  bluriness = mb.getMotionBlurMeasures();

  std::stringstream output;
  for (size_t i = 0; i < bluriness.size(); i++) {
    output << bluriness[i] << "," << filenames[i] << "\n";
  }

  std::cout << "Saving file..." << result << std::endl;

  std::ofstream stddevs;
  stddevs.open(result);
  stddevs << output.str();
  stddevs.close();
}

bool parseArgs(int argc, char** argv, std::string& input_folder, std::string& result,
               v4r::MotionBlurDetectionType& method) {
  std::string method_str;
  po::options_description arguments(
      "Detect Motion Blur. Estimates the bluriness of an image, using either the Variance of Laplacian (VL) or the "
      "Variance of a Modified Laplacian (VML). The higher the variance, the sharper the image.");
  arguments.add_options()("help,h", "");
  arguments.add_options()("img_path,i", po::value<std::string>(&input_folder)->default_value("./"),
                          "Path to image folder");
  arguments.add_options()("result,r", po::value<std::string>(&result)->default_value("./blur.csv"),
                          "Path to result file");
  arguments.add_options()("method,r", po::value<std::string>(&method_str)->default_value("VL"),
                          "Available Methods: Variation of Modified Laplacian (VML), Variation of "
                          "Laplacian (VL) [Default]");

  po::options_description all("");
  all.add(arguments);
  po::options_description visible("");
  visible.add(arguments);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(all).run(), vm);
  po::notify(vm);

  std::string usage = "General usage: [-options] []";

  if (vm.count("help")) {
    std::cout << usage << std::endl;
    std::cout << visible;
    return false;
  }

  boost::to_upper(method_str);
  if (method_str == "VL") {
    method = v4r::MotionBlurDetectionType::VL;
  } else if (method_str == "VML") {
    method = v4r::MotionBlurDetectionType::VML;
  } else {
    std::cout << "Unknown method, user provided " << method_str << " . Using VL." << std::endl;
    method = v4r::MotionBlurDetectionType::VL;
  }

  return true;
}
