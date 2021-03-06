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
 * @file nearestNeighbor.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */
#pragma once

#include <v4r/common/metrics.h>
#include <v4r/core/macros.h>
#include <v4r/ml/classifier.h>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <opencv2/flann/flann.hpp>

namespace po = boost::program_options;

namespace v4r {
struct V4R_EXPORTS NearestNeighborClassifierParameter {
  int kdtree_num_trees_ = 4;
  int checks_ = 32;  ///< how many leafs to visit when searching for neighbours (-1 for unlimited)
  size_t knn_ = 1;   ///< nearest neighbors to search for when checking feature descriptions of the scene
  DistanceMetric distance_metric_ = DistanceMetric::L2;  ///< defines the norm used for feature ma

  /**
   * @brief init parameters
   * @param command_line_arguments (according to Boost program options library)
   * @return unused parameters (given parameters that were not used in this initialization call)
   */
  std::vector<std::string> init(int argc, char **argv) {
    std::vector<std::string> arguments(argv + 1, argv + argc);
    return init(arguments);
  }

  /**
   * @brief init parameters
   * @param command_line_arguments (according to Boost program options library)
   * @return unused parameters (given parameters that were not used in this initialization call)
   */
  std::vector<std::string> init(const std::vector<std::string> &command_line_arguments) {
    po::options_description desc("Nearest Neighbor Classifier Parameter\n=====================\n");
    desc.add_options()("help,h", "produce help message");
    desc.add_options()("nn_kdtree_num_trees", po::value<int>(&kdtree_num_trees_)->default_value(kdtree_num_trees_),
                       "Number of kd-trees");
    desc.add_options()("nn_knn", po::value<size_t>(&knn_)->default_value(knn_),
                       "nearest neighbors to search for when checking feature descriptions of the scene");
    desc.add_options()("nn_checks", po::value<int>(&checks_)->default_value(checks_),
                       "How many leafs to visit when searching for neighbours (-1 for unlimited)");
    desc.add_options()("nn_distance_metric",
                       po::value<DistanceMetric>(&distance_metric_)->default_value(distance_metric_),
                       "defines the norm used for feature matching (1... L1 norm, 2... L2 norm)");
    po::variables_map vm;
    po::parsed_options parsed =
        po::command_line_parser(command_line_arguments).options(desc).allow_unregistered().run();
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
    return to_pass_further;
  }
};

class V4R_EXPORTS NearestNeighborClassifier : public Classifier {
 private:
  std::shared_ptr<cv::flann::Index> flann_index;
  mutable cv::Mat knn_indices_;
  mutable cv::Mat knn_distances_;
  cv::Mat training_label_;
  NearestNeighborClassifierParameter param_;
  cv::Mat all_training_data;  ///< all signatures from all objects in the database

 public:
  NearestNeighborClassifier(const NearestNeighborClassifierParameter &p = NearestNeighborClassifierParameter())
  : param_(p) {}

  void predict(const Eigen::MatrixXf &query_data, Eigen::MatrixXi &predicted_label) const override;

  void train(const Eigen::MatrixXf &training_data, const Eigen::VectorXi &training_label) override;

  /**
   * @brief getTrainingSampleIDSforPredictions
   * @param predicted_training_sample_indices
   * @param distances of the training sample to the corresponding query data
   */
  void getTrainingSampleIDSforPredictions(Eigen::MatrixXi &predicted_training_sample_indices,
                                          Eigen::MatrixXf &distances) const override;

  ClassifierType getType() const override {
    return ClassifierType::KNN;
  }

  typedef std::shared_ptr<NearestNeighborClassifier> Ptr;
  typedef std::shared_ptr<NearestNeighborClassifier const> ConstPtr;
};
}  // namespace v4r
