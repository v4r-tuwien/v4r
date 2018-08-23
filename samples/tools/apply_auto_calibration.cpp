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
 * @file apply_auto_calibration.cpp
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */

#include <memory.h>
#include <sys/stat.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION < 3
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#else
#include <opencv2/imgcodecs/imgcodecs.hpp>
#endif
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <v4r/calibration/apply_calibration.h>
#include <v4r/common/img_loader.h>

namespace po = boost::program_options;
namespace ac = v4r::auto_calibration;

bool parseArgs(int argc, char** argv, std::string& param_file, std::string& folder, std::string& result, int& num_cams,
               bool& generate_cloud, bool& generate_depth, bool& generate_pseudo_depth, bool& tum_data);
void generatePseudoDepth(std::string result, int im_height, int im_width, std::shared_ptr<ac::apply_calibration> calib);

int main(int argc, char** argv) {
  std::string param_file, folder, result;
  bool generate_cloud, generate_depth, generate_pseudo_depth, tum_data;
  int num_cams = 0;
  int max_cams = 0;

  // get input arguments

  if (!parseArgs(argc, argv, param_file, folder, result, max_cams, generate_cloud, generate_depth,
                 generate_pseudo_depth, tum_data)) {
    return 0;
  }

  // Load RGB Images
  v4r::ImgContainer rgb_img_container;
  rgb_img_container.setPathToImages(folder + "/rgb/", "*.png");
  const std::vector<cv::Mat>& rgb_imgs = rgb_img_container.getImages();

  // Load Depth Images
  v4r::ImgContainer depth_img_container;
  depth_img_container.setPathToImages(folder + "/depth/", "*.png");
  const std::vector<cv::String>& filenames_depth = depth_img_container.getFileNames();
  const std::vector<cv::Mat>& depth_imgs = depth_img_container.getImages();

  // generate mutable depth images (will be converted to float an meter)
  std::vector<cv::Mat> mutable_depth(depth_imgs);
  num_cams = depth_imgs.size();

  if (num_cams > max_cams && max_cams > 0)
    num_cams = max_cams;

  int im_width = depth_imgs[0].cols;
  int im_height = depth_imgs[0].rows;

  int im_width_rgb = rgb_imgs[0].cols;
  int im_height_rgb = rgb_imgs[0].rows;

  //
  ac::apply_calibration calib(param_file);

  // Create Result Directory if it does not exist

  struct stat st;

  if (stat((result).c_str(), &st) == -1) {
    mkdir((result).c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
    std::cout << std::endl << "Creating directory for results: " << result << std::endl;
  } else {
    std::cout << std::endl << "Evaluation directory exists already: " << result << std::endl;
  }

  // If the user asked for pseud depth images (most likely to view the offset compensation for given distances):
  if (generate_pseudo_depth) {
    generatePseudoDepth(result, im_height, im_width, std::make_shared<ac::apply_calibration>(calib));
  }

  // For n (every) image, do
  for (int i = 0; i < num_cams; i++) {
    std::string filename = boost::filesystem::path(filenames_depth[i]).filename().string();
    std::string filename_noext = boost::filesystem::path(filenames_depth[i]).stem().string();

    // Convert depth to mm
    if (tum_data) {
      mutable_depth[i] = mutable_depth[i] * (0.001 / 5.);
    } else {
      mutable_depth[i] = mutable_depth[i] * 0.001;
    }

    // generate the point cloud
    if (generate_cloud) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(im_width, im_height));
      {
        pcl::ScopeTime t1("Generating PointCloud");
        calib.GeneratePointCloud(&depth_imgs[i], &rgb_imgs[i], *cloud);
      }
      // save cloud
      pcl::io::savePCDFileBinaryCompressed(result + "/" + filename_noext + ".pcd", *cloud);
    }

    // generate the corrected and projected depth map (project depth map to rgb image)
    if (generate_depth) {
      cv::Mat projected_depth(im_height_rgb, im_width_rgb, CV_16U, cv::Scalar(0));
      {
        pcl::ScopeTime t1("Generating DepthMap");
        calib.GenerateDepthMap(&depth_imgs[i], projected_depth);
      }

      std::string base_filename = result + "/" + filename;
      // save image
      cv::imwrite(base_filename, projected_depth);
    }
  }
}

bool parseArgs(int argc, char** argv, std::string& param_file, std::string& folder, std::string& result, int& num_cams,
               bool& generate_cloud, bool& generate_depth, bool& generate_pseudo_depth, bool& tum_data) {
  po::options_description arguments(
      "Applies the results of a structure based auto calibration to a depth/rgb image/set. ");
  arguments.add_options()("help,h", "");
  arguments.add_options()("parameter_file,p", po::value<std::string>(&param_file)->default_value("./"),
                          "Path to parameter file");
  arguments.add_options()("img_folder,i ", po::value<std::string>(&folder)->default_value("./"),
                          "Path to image folders; structure: (/path/rgb/n.png /path/depth/n.png)");
  arguments.add_options()("result_folder,r", po::value<std::string>(&result)->default_value("./"),
                          "Path to result folder");
  arguments.add_options()("num_cams,n ", po::value<int>(&num_cams)->default_value(0), "Max Number of images");
  arguments.add_options()("cloud,c ", po::value<bool>(&generate_cloud)->default_value(false), "Generate cloud");
  arguments.add_options()("depth,d ", po::value<bool>(&generate_depth)->default_value(true), "Generate depth map");
  arguments.add_options()("pseudoDepth,g", po::value<bool>(&generate_pseudo_depth)->default_value(false),
                          "Generate Pseudo Depth maps for 0.5-5.5m");
  arguments.add_options()("TUMData,t", po::value<bool>(&tum_data)->default_value(false), "TUM Data (1m=5000)");

  po::options_description config("Other options");
  std::string config_file;

  config.add_options()("configfile,s", po::value<std::string>(&config_file), "Configuration file");

  po::options_description all("");
  all.add(arguments).add(config);
  po::options_description visible("");
  visible.add(arguments).add(config);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(all).run(), vm);
  po::notify(vm);

  if (vm.count("configfile")) {
    std::ifstream ifs(config_file);
    if (ifs) {
      po::store(po::parse_config_file(ifs, all, true), vm);
      ifs.close();
    }

    else {
      std::cout << "Configuration file " << config_file << " cannot be opened!" << std::endl;
      return false;
    }
  }

  po::notify(vm);

  std::string usage = "General usage: program [-options] []";

  if (vm.count("help")) {
    std::cout << usage << std::endl;
    std::cout << visible;
    return false;
  }

  return true;
}

void generatePseudoDepth(std::string result, int im_height, int im_width,
                         std::shared_ptr<ac::apply_calibration> calib) {
  // generate result subdirectory
  std::string result_lattice;
  result_lattice = result + "/lattice/";
  struct stat st;

  if (stat((result_lattice).c_str(), &st) == -1) {
    mkdir((result_lattice).c_str(), S_IRUSR | S_IWUSR | S_IXUSR);
    std::cout << std::endl << "Creating directory for lattice: " << result << std::endl;
  } else {
    std::cout << std::endl << "Directory for lattice exists already: " << result << std::endl;
  }

  // generate 8 artificcal depth image with delta=0.5 starting from 0.5
  cv::Mat pseudo_depth(im_height, im_width, CV_32FC1, cv::Scalar(.5));
  std::vector<cv::Mat> depths;
  std::string filename = "pseudo_depth";
  for (int i = 0; i < 8; ++i) {
    pseudo_depth += 0.5;
    cv::Mat projected_depth(im_height, im_width, CV_16U, cv::Scalar(0));
    cv::Mat conv_depth;
    // Generate the depth map without projecting it to the rgb image = compensate depth error.
    calib->GenerateDepthMapNoProjection(&pseudo_depth, projected_depth);

    projected_depth.convertTo(conv_depth, CV_32FC1, 0.001);  ///(max-min));
    conv_depth -= pseudo_depth;
    depths.push_back(conv_depth);
  }

  // Colorize Image
  // First, find min and max over all images.
  double min = 0;
  double max = 0;
  cv::minMaxIdx(depths[0], &min, &max);
  for (size_t i = 1; i < depths.size(); ++i) {
    double temp_min, temp_max;
    cv::minMaxIdx(depths[i], &temp_min, &temp_max);

    if (temp_max > max) {
      max = temp_max;
    }
    if (temp_min < min) {
      min = temp_min;
    }
  }

  // alpha to convert image from float (m) to 0...255.
  double alpha;
  if (max == min) {
    alpha = 0;  // if min==max set image to 0.
  } else {
    alpha = 255.0 / (max - min);
  }

  for (size_t i = 0; i < depths.size(); ++i) {
    // Color
    cv::Mat conv;
    cv::Mat colored;

    // min, max of image for filename
    double min_img = 0;
    double max_img = 0;

    cv::minMaxIdx(depths[i], &min_img, &max_img);

    // Convert image and apply Colormap
    depths[i].convertTo(conv, CV_8UC3, alpha, -min * alpha);  ///(max-min));
    cv::applyColorMap(conv, colored, cv::COLORMAP_JET);

    // Save image
    cv::imwrite(result_lattice + "/" + filename + "_" + std::to_string(float(i) / 2. + 0.5) + "m_min_" +
                    std::to_string(min_img) + "_max_" + std::to_string(max_img) + ".png",
                colored);
  }
}
