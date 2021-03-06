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
 * @file svmWrapper.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2015
 * @brief
 *
 */
#pragma once

#include <libsvm/svm.h>
#include <v4r/core/macros.h>
#include <v4r/ml/classifier.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <vector>

namespace po = boost::program_options;

namespace v4r {

struct V4R_EXPORTS SVMParameter {
  int do_cross_validation_ = 0;  /// if greater 1, performs k-fold cross validation with k equal set by this variable
  int knn_ = 3;                  ///< return the knn most probably classes when parameter probability is set to true
  bool do_scaling_ = false;      ///< scale each attribute to [0 1]
  ::svm_parameter svm_;

  std::vector<double> cross_validation_range_C_ =
      {exp2(-5), exp2(15), 2};  ///< cross validation range for parameter C (first element minimum,
                                /// second element maximum, third element step size as a multiplier)
  std::vector<double> cross_validation_range_gamma_ =
      {exp2(-15), exp2(3), 4};  ///< cross validation range for parameter gamma (first element
                                /// minimum, second element maximum, third element step size as a
  /// multiplier)

  std::string filename_ =
      "";  ///< filename from where to load svm file (if path exists, will skip training and use this
           /// model instead)

  SVMParameter(int svm_type = ::C_SVC,
               int kernel_type = ::LINEAR,  //::RBF,
               //                    int degree = 2,	/* for poly */
               double gamma = -1, /* for poly/rbf/sigmoid */
               //                    double coef0 = 1,	/* for poly/sigmoid */

               /* these are for training only */
               double cache_size = 100,     /* in MB */
               double eps = 0.001,          /* stopping criteria */
               double C = 10,               /* for C_SVC, EPSILON_SVR and NU_SVR */
               int nr_weight = 0,           /* for C_SVC */
               int *weight_label = nullptr, /* for C_SVC */
               double *weight = nullptr,    /* for C_SVC */
               //                    double nu = 0.5,	/* for NU_SVC, ONE_CLASS, and NU_SVR */
               //                    double p = 1,	/* for EPSILON_SVR */
               int shrinking = 1,  /* use the shrinking heuristics */
               int probability = 0 /* do probability estimates */
               )
  : do_cross_validation_(0), knn_(3), do_scaling_(false), cross_validation_range_C_({exp2(-5), exp2(15), 2}),
    cross_validation_range_gamma_({exp2(-15), exp2(3), 4}), filename_("") {
    svm_.svm_type = svm_type;
    svm_.kernel_type = kernel_type;
    //                svm_.degree = degree;
    svm_.gamma = gamma;  // default 1/ (num_features);
    //                svm_.coef0 = coef0;

    svm_.cache_size = cache_size;
    svm_.eps = eps;
    svm_.C = C;
    svm_.nr_weight = nr_weight;
    svm_.weight_label = weight_label;
    svm_.weight = weight;
    //                svm_.nu = nu;
    //                svm_.p = p;
    svm_.shrinking = shrinking;
    svm_.probability = probability;
  }

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
    po::options_description desc("SVM Classification Parameter\n=====================\n");
    desc.add_options()("help,h", "produce help message");
    desc.add_options()("svm_do_cross_validation",
                       po::value<int>(&do_cross_validation_)->default_value(do_cross_validation_),
                       "if greater 1, performs k-fold cross validation with k equal set by this variable");
    desc.add_options()("svm_knn", po::value<int>(&knn_)->default_value(knn_),
                       "return the knn most probably classes when parameter probability is set to true");
    desc.add_options()("svm_do_scaling", po::value<bool>(&do_scaling_)->default_value(do_scaling_),
                       "scale each attribute to [0 1]");
    desc.add_options()("svm_type", po::value<int>(&svm_.svm_type)->default_value(svm_.svm_type), "according to LIBSVM");
    desc.add_options()("svm_kernel_type", po::value<int>(&svm_.kernel_type)->default_value(svm_.kernel_type),
                       "according to LIBSVM");
    desc.add_options()("svm_gamma", po::value<double>(&svm_.gamma)->default_value(svm_.gamma), "for poly/rbf/sigmoid");
    desc.add_options()("svm_cache_size", po::value<double>(&svm_.cache_size)->default_value(svm_.cache_size), "in MB");
    desc.add_options()("svm_eps", po::value<double>(&svm_.eps)->default_value(svm_.eps), "stopping criteria");
    desc.add_options()("svm_C", po::value<double>(&svm_.C)->default_value(svm_.C), "for C_SVC, EPSILON_SVR and NU_SVR");
    desc.add_options()("svm_nr_weight", po::value<int>(&svm_.nr_weight)->default_value(svm_.nr_weight), "");
    desc.add_options()("svm_shrinking", po::value<int>(&svm_.shrinking)->default_value(svm_.shrinking),
                       "use the shrinking heuristics");
    desc.add_options()("svm_probability", po::value<int>(&svm_.probability)->default_value(svm_.probability),
                       "do probability estimates");
    desc.add_options()(
        "svm_cross_validation_range_C", po::value<std::vector<double>>(&cross_validation_range_C_)->multitoken(),
        "cross validation range for parameter C (first element minimum, second element maximum, third element step "
        "size as a multiplier)");
    desc.add_options()("svm_cross_validation_range_gamma",
                       po::value<std::vector<double>>(&cross_validation_range_gamma_)->multitoken(),
                       "cross validation range for parameter gamma (first element minimum, second element "
                       "maximum, third element step size as a multiplier)");
    desc.add_options()(
        "svm_filename", po::value<std::string>(&filename_)->default_value(filename_),
        "filename from where to load svm file (if path exists, will skip training and use this model instead)");
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

class V4R_EXPORTS svmClassifier : public Classifier {
 private:
  SVMParameter param_;
  ::svm_model *svm_mod_;

  Eigen::VectorXf scale_;  ///< scale for each attribute (only if scaling is enabled)
  //    Eigen::VectorXf offset_; ///< scale offset for each attribute (only if scaling is enabled)
 public:
  svmClassifier(const SVMParameter &p = SVMParameter()) : param_(p) {}

  void predict(const Eigen::MatrixXf &query_data, Eigen::MatrixXi &predicted_label) const override;

  /**
   * @brief saveModel save current svm model
   * @param filename filename to save trained model
   */
  void saveModel(const boost::filesystem::path &filename) const;

  /**
   * @brief loadModel load an SVM model from file
   * @param filename filename to read svm model
   */
  void loadModel(const boost::filesystem::path &filename);

  void train(const Eigen::MatrixXf &training_data, const Eigen::VectorXi &training_label) override;

  ClassifierType getType() const override {
    return ClassifierType::SVM;
  }

  typedef std::shared_ptr<svmClassifier> Ptr;
  typedef std::shared_ptr<svmClassifier const> ConstPtr;
};
}  // namespace v4r
