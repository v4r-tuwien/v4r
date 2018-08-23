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

/*
 * fast_icp_with_gc.h
 *
 *  Created on: Sep 8, 2013
 *      Author: aitor
 */

#pragma once

#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <v4r/common/intrinsics.h>
#include <v4r/core/macros.h>
#include <v4r/registration/uniform_sampling.h>

namespace v4r {
template <typename PointT>
class V4R_EXPORTS ICPNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ICPNode(int l, bool root = false) {
    converged_ = false;
    is_root_ = root;
    level_ = l;
    color_weight_ = 0.f;
    reg_error_ = 0.f;
    osv_fraction_ = 0.f;
    fsv_fraction_ = 0.f;
    overlap_ = 0;
    incr_transform_.setIdentity();
    accum_transform_.setIdentity();
    childs_.clear();
  }

  void addChild(typename std::shared_ptr<ICPNode<PointT>>& c) {
    childs_.push_back(c);
  }

  std::shared_ptr<ICPNode> parent_;
  bool is_root_;
  int level_;                       // equivalent to the ICP iteration
  Eigen::Matrix4f incr_transform_;  // transform from parent to current node
  Eigen::Matrix4f accum_transform_;
  bool converged_;  // whether the alignment path converged or not...
  typename std::vector<std::shared_ptr<ICPNode<PointT>>> childs_;
  float reg_error_;
  float color_weight_;
  int overlap_;
  float osv_fraction_;
  float fsv_fraction_;
  pcl::CorrespondencesPtr after_rej_correspondences_;
  typename pcl::PointCloud<PointT>::Ptr src_keypoints_;
  pcl::PointCloud<pcl::Normal>::Ptr normal_src_keypoints_;
};

template <typename PointT>
class V4R_EXPORTS FastIterativeClosestPointWithGC {
  typedef typename pcl::PointCloud<PointT>::Ptr PointTPtr;
  typedef typename pcl::PointCloud<PointT>::ConstPtr ConstPointTPtr;

  void visualizeICPNodes(typename std::vector<std::shared_ptr<ICPNode<PointT>>>& nodes,
                         pcl::visualization::PCLVisualizer& vis);

  bool filterHypothesesByPose(typename std::shared_ptr<ICPNode<PointT>>& current,
                              typename std::vector<std::shared_ptr<ICPNode<PointT>>>& nodes, float trans_threshold);

  std::vector<float> evaluateHypotheses(typename pcl::PointCloud<PointT>::ConstPtr im1,
                                        typename pcl::PointCloud<PointT>::ConstPtr im_2,
                                        pcl::PointCloud<pcl::Normal>::ConstPtr normals1,
                                        pcl::PointCloud<pcl::Normal>::ConstPtr normals_2,
                                        const Eigen::Matrix4f& pose_2_to_1 = Eigen::Matrix4f::Identity());

  struct Result {
    float registration_error_;
    Eigen::Matrix4f accum_tf_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Result() : registration_error_(-1.), accum_tf_(Eigen::Matrix4f::Identity()) {}

    Result(float registration_error, const Eigen::Matrix4f& accum_tf)
    : registration_error_(registration_error), accum_tf_(accum_tf) {}
  };

  // input_ and target_ need to be organized!
  typename pcl::PointCloud<PointT>::ConstPtr input_, target_;
  pcl::PointCloud<pcl::Normal>::ConstPtr input_normals_, target_normals_;
  pcl::PointIndices input_indices_, target_indices_;

  size_t max_iterations_;
  float corr_dist_threshold_;
  float ransac_threshold_;
  size_t min_number_correspondences_;
  float gc_size_;
  Intrinsics cam_;
  size_t max_keep_;
  float uniform_sampling_radius_;
  float ov_percentage_;
  std::vector<Result, Eigen::aligned_allocator<Result>> result_;
  bool use_normals_;
  bool standard_cg_;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> initial_poses_;
  bool no_cg_;

  void getKeypointsWithMask(typename pcl::PointCloud<PointT>::ConstPtr& cloud, const std::vector<int>& indices,
                            const std::vector<int>& roi_indices, std::vector<int>& indices_out) {
    std::vector<bool> mask;
    mask.resize(cloud->points.size(), false);
    for (int idx : roi_indices)
      mask[idx] = true;

    for (int idx : indices) {
      if (mask[idx]) {
        indices_out.push_back(idx);
      }
    }
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FastIterativeClosestPointWithGC() {
    max_iterations_ = 5;
    corr_dist_threshold_ = 0.05f;
    gc_size_ = ransac_threshold_ = 0.01f;
    min_number_correspondences_ = 5;
    cam_ = Intrinsics::PrimeSense();
    max_keep_ = 7;
    uniform_sampling_radius_ = 0.01f;
    ov_percentage_ = 0.5f;
    standard_cg_ = true;
    no_cg_ = false;
  }

  void setNoCG(bool b) {
    no_cg_ = b;
  }

  void setGCSize(float t) {
    gc_size_ = t;
  }

  void setRansacThreshold(float t) {
    ransac_threshold_ = t;
  }

  void setInitialPoses(const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poses) {
    initial_poses_ = poses;
  }

  void setInputAndTargetIndices(const pcl::PointIndices& input, const pcl::PointIndices& target) {
    input_indices_ = input;
    target_indices_ = target;
  }

  void useStandardCG(bool b) {
    standard_cg_ = b;
  }

  float getFinalTransformation(Eigen::Matrix4f& matrix) const {
    if (result_.empty()) {
      PCL_WARN("There are no result to be returned\n");
      matrix = Eigen::Matrix4f::Identity();
      return -1.f;
    }

    matrix = result_[0].accum_tf_;
    return result_[0].registration_error_;
  }

  void setMaxCorrespondenceDistance(float f) {
    corr_dist_threshold_ = f;
  }

  void setKeepMaxHypotheses(size_t k) {
    max_keep_ = k;
  }

  void setOverlapPercentage(float f) {
    ov_percentage_ = f;
  }

  void setMaximumIterations(size_t it) {
    max_iterations_ = it;
  }

  void setInputSource(const typename pcl::PointCloud<PointT>::ConstPtr cloud) {
    input_ = cloud;
  }

  void setInputTarget(const typename pcl::PointCloud<PointT>::ConstPtr cloud) {
    target_ = cloud;
  }

  void setSourceNormals(const typename pcl::PointCloud<pcl::Normal>::ConstPtr normals) {
    input_normals_ = normals;
  }

  void setTargetNormals(const typename pcl::PointCloud<pcl::Normal>::ConstPtr normals) {
    target_normals_ = normals;
  }

  void align(const Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity());

  void setCameraIntrinsics(const Intrinsics& cam) {
    cam_ = cam;
  }
};
}  // namespace v4r
