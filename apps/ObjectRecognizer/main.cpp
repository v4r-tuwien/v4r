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
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/serialization/vector.hpp>

#include <v4r/apps/ObjectRecognizer.h>
#include <v4r/common/pcl_serialization.h>

namespace po = boost::program_options;

int main(int argc, char **argv) {
  typedef pcl::PointXYZRGB PT;

  bf::path test_dir;
  bf::path out_dir = "/tmp/object_recognition_results/";
  bf::path recognizer_config_dir = "cfg";
  std::vector<std::string> obj_models_to_search = {};  //< object identities to be detected. If empty, all object models
  // of the object model database will be searched.
  int verbosity = -1;

  po::options_description desc("Object Instance Recognizer\n======================================\n**Allowed options");
  desc.add_options()("help,h", "produce help message");
  desc.add_options()(
      "test_dir,t", po::value<bf::path>(&test_dir)->required(),
      "Directory with test scenes stored as point clouds (.pcd). The camera pose is taken directly from the pcd header "
      "fields \"sensor_orientation_\" and \"sensor_origin_\" (if the test directory contains subdirectories, each "
      "subdirectory is considered as separate sequence for multiview recognition)");
  desc.add_options()("out_dir,o", po::value<bf::path>(&out_dir)->default_value(out_dir),
                     "Output directory where recognition results will be stored.");
  desc.add_options()(
      "cfg", po::value<bf::path>(&recognizer_config_dir)->default_value(recognizer_config_dir),
      "Path to config directory containing the xml config files for the various recognition pipelines and parameters.");
  desc.add_options()("verbosity", po::value<int>(&verbosity)->default_value(verbosity),
                     "set verbosity level for output (<0 minimal output)");
  desc.add_options()("object_models_to_search",
                     po::value<std::vector<std::string>>(&obj_models_to_search)->multitoken(),
                     "object identities to be detected. If empty, all object models "
                     "of the object model database will be searched.");
  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
  std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
  po::store(parsed, vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    to_pass_further.push_back("-h");
  }
  try {
    po::notify(vm);
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
  }

  if (verbosity >= 0) {
    FLAGS_logtostderr = 1;
    FLAGS_v = verbosity;
    std::cout << "Enabling verbose logging." << std::endl;
  }
  google::InitGoogleLogging(argv[0]);

  v4r::apps::ObjectRecognizer<PT> recognizer;
  recognizer.initialize(to_pass_further, recognizer_config_dir);

  std::vector<std::string> sub_folder_names;
  if (!bf::is_regular_file(test_dir)) {
    sub_folder_names = v4r::io::getFoldersInDirectory(test_dir);
    if (sub_folder_names.empty())
      sub_folder_names.push_back("");
  } else {
    sub_folder_names.push_back("");
  }

  for (const std::string &sub_folder_name : sub_folder_names) {
    recognizer.resetMultiView();
    std::vector<std::string> views;

    if (bf::is_regular(test_dir))
      views.push_back("");
    else
      views = v4r::io::getFilesInDirectory(test_dir / sub_folder_name, ".*.pcd", false);

    for (size_t v_id = 0; v_id < views.size(); v_id++) {
      bf::path test_path = test_dir / sub_folder_name / views[v_id];

      std::stringstream info_ss;
      info_ss << "Recognizing file " << test_path.string();
      LOG(INFO) << info_ss.str();

      if (!FLAGS_logtostderr)
        std::cout << info_ss.str() << std::endl;

      pcl::PointCloud<PT>::Ptr cloud(new pcl::PointCloud<PT>());
      pcl::io::loadPCDFile(test_path.string(), *cloud);

      // reset view point - otherwise this messes up PCL's visualization (this does not affect recognition results)
      //            cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
      //            cloud->sensor_origin_ = Eigen::Vector4f::Zero(4);

      std::vector<v4r::ObjectHypothesesGroup> generated_object_hypotheses =
          recognizer.recognize(cloud, obj_models_to_search);
      std::vector<std::pair<std::string, float>> elapsed_time = recognizer.getElapsedTimes();

      if (!out_dir.empty()) {
        // write results to disk (for each verified hypothesis add a row in the text file with
        // object name, dummy confidence value and object pose in row-major order)
        std::string out_basename = test_path.filename().string();
        boost::replace_last(out_basename, ".pcd", ".anno");
        bf::path out_path = out_dir / sub_folder_name / out_basename;

        std::string out_path_generated_hypotheses = out_path.string();
        boost::replace_last(out_path_generated_hypotheses, ".anno", ".generated_hyps");

        std::string out_path_generated_hypotheses_serialized = out_path.string();
        boost::replace_last(out_path_generated_hypotheses_serialized, ".anno", ".generated_hyps_serialized");

        v4r::io::createDirForFileIfNotExist(out_path);

        // save hypotheses
        std::ofstream f_generated(out_path_generated_hypotheses.c_str());
        std::ofstream f_verified(out_path.string().c_str());
        std::ofstream f_generated_serialized(out_path_generated_hypotheses_serialized.c_str());
        boost::archive::text_oarchive oa(f_generated_serialized);
        oa << generated_object_hypotheses;
        f_generated_serialized.close();
        for (size_t ohg_id = 0; ohg_id < generated_object_hypotheses.size(); ohg_id++) {
          for (const v4r::ObjectHypothesis::Ptr &oh : generated_object_hypotheses[ohg_id].ohs_) {
            f_generated << oh->model_id_ << " (" << oh->confidence_ << "): ";
            const Eigen::Matrix4f tf = oh->pose_refinement_ * oh->transform_;

            for (size_t row = 0; row < 4; row++)
              for (size_t col = 0; col < 4; col++)
                f_generated << tf(row, col) << " ";
            f_generated << std::endl;

            if (oh->is_verified_) {
              f_verified << oh->model_id_ << " (" << oh->confidence_ << "): ";
              for (size_t row = 0; row < 4; row++)
                for (size_t col = 0; col < 4; col++)
                  f_verified << tf(row, col) << " ";
              f_verified << std::endl;
            }
          }
        }
        f_generated.close();
        f_verified.close();

        // save elapsed time(s)
        std::string out_path_times = out_path.string();
        boost::replace_last(out_path_times, ".anno", ".times");
        f_verified.open(out_path_times.c_str());
        for (const std::pair<std::string, float> &t : elapsed_time)
          f_verified << t.second << " " << t.first << std::endl;
        f_verified.close();
      }
    }
  }
}
