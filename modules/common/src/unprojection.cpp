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
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

#include <pcl/pcl_config.h>
#include <pcl/point_types.h>

#include <v4r/common/unprojection.h>
#include <v4r/core/macros.h>

namespace v4r {

namespace {

template <typename PointT>
typename std::enable_if<!pcl::traits::has_color<PointT>::value, void>::type setColor(const cv::Vec3b& /* src */,
                                                                                     PointT& /* tgt */) {}

template <typename PointT>
typename std::enable_if<pcl::traits::has_color<PointT>::value, void>::type setColor(const cv::Vec3b& src, PointT& tgt) {
  tgt.r = src[2];
  tgt.g = src[1];
  tgt.b = src[0];
  tgt.a = 255;
}

}  // anonymous namespace

template <typename PointT>
void unproject(cv::InputArray _color, cv::InputArray _depth, const Intrinsics& intr, pcl::PointCloud<PointT>& cloud) {
  const auto fx_inv = 1.0f / intr.fx;
  const auto fy_inv = 1.0f / intr.fy;
  const auto w = _depth.size().width;
  const auto h = _depth.size().height;

  assert(intr.w == w);
  assert(intr.h == h);
  if (!_color.empty()) {
    assert(_color.size().width == w);
    assert(_color.size().height == h);
  }

  cloud.height = h;
  cloud.width = w;
  cloud.is_dense = false;
  cloud.points.resize(h * w);

  auto depth = _depth.getMat();
  auto color = _color.getMat();

  size_t point_idx = 0;
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u, ++point_idx) {
      auto& pt = cloud.points[point_idx];
      const auto& d = depth.at<float>(v, u);
      if (d == 0 || d == bad_point) {
        pt.x = pt.y = pt.z = bad_point;
      } else {
        pt.z = d;
        pt.x = (static_cast<float>(u) - intr.cx) * pt.z * fx_inv;
        pt.y = (static_cast<float>(v) - intr.cy) * pt.z * fy_inv;
      }
      if (!_color.empty())
        setColor<PointT>(color.at<cv::Vec3b>(v, u), pt);
    }
  }
}

template V4R_EXPORTS void unproject(cv::InputArray, cv::InputArray, const Intrinsics&,
                                    pcl::PointCloud<pcl::PointXYZ>& cloud);
template V4R_EXPORTS void unproject(cv::InputArray, cv::InputArray, const Intrinsics&,
                                    pcl::PointCloud<pcl::PointXYZRGB>& cloud);
template V4R_EXPORTS void unproject(cv::InputArray, cv::InputArray, const Intrinsics&,
                                    pcl::PointCloud<pcl::PointXYZRGBA>& cloud);

}  // namespace v4r
