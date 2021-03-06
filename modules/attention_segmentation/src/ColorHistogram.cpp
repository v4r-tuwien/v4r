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
 * @file ColorHistogram.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Color histogram class.
 */

#include <glog/logging.h>
#include <string.h>

#include "v4r/attention_segmentation/ColorHistogram.h"

namespace v4r {

ColorHistogram::ColorHistogram(int _nrBins, double _UVthreshold) {
  nrBins = _nrBins;
  computed = false;
  have_input_cloud = false;
  have_indices = false;
  colorModel = YUV_MODEL;
  histogramType = HIST_3D;
  maxVal = 255.;
  UVthreshold = _UVthreshold;
  useUVthreshold = false;
  hist = cv::Mat_<double>::zeros(1, 1);
}

void ColorHistogram::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
  if ((_cloud->height <= 1) || (_cloud->width <= 1) || (!_cloud->isOrganized()))
    throw std::runtime_error("[ColorHistogram::setInputCloud] Invalid point cloud (height must be > 1)");

  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;

  have_input_cloud = true;
  computed = false;

  indices.reset(new pcl::PointIndices);
  for (unsigned i = 0; i < cloud->points.size(); i++) {
    indices->indices.push_back(i);
  }
}

void ColorHistogram::setIndices(pcl::PointIndices::Ptr _indices) {
  if (!have_input_cloud) {
    throw std::invalid_argument("[ColorHistogram::setIndices] no input cloud set.");
  }

  indices = _indices;
  have_indices = true;
}

void ColorHistogram::setIndices(std::vector<int> &_indices) {
  indices.reset(new pcl::PointIndices);
  indices->indices = _indices;
}

void ColorHistogram::setIndices(cv::Rect _rect) {
  if (!have_input_cloud) {
    throw std::invalid_argument("[ColorHistogram::setIndices]: Error: No input cloud available.\n");
  }

  if (_rect.y >= height) {
    _rect.y = height - 1;
  }

  if ((_rect.y + _rect.height) >= height) {
    _rect.height = height - _rect.y;
  }

  if (_rect.x >= width) {
    _rect.x = width - 1;
  }

  if ((_rect.x + _rect.width) >= width) {
    _rect.width = width - _rect.x;
  }

  VLOG(1) << "_rect = " << _rect.x << ", " << _rect.y << ", " << _rect.x + _rect.width << ", "
          << _rect.y + _rect.height;

  indices.reset(new pcl::PointIndices);
  for (int r = _rect.y; r < (_rect.y + _rect.height); r++) {
    for (int c = _rect.x; c < (_rect.x + _rect.width); c++) {
      indices->indices.push_back(r * width + c);
    }
  }
}

bool ColorHistogram::init() {
  hist = cv::Mat_<double>::zeros(1, 1);

  switch (histogramType) {
    case HIST_AVERAGE_HIST:
      hist = cv::Mat_<double>::zeros(nrBins, 3);
      return (true);
    case HIST_2D:
      hist = cv::Mat_<double>::zeros(nrBins, nrBins);
      return (true);
    case HIST_3D:
      hist = cv::Mat_<double>::zeros(nrBins, nrBins * nrBins);
      return (true);
    default:
      return (false);
  }
}

void ColorHistogram::getYUV(v4r::RGBValue &color, Color3C &convColor) {
  //@ep: Wikipedia says that transformation matrix is different
  //@ep: why +128???
  //@ep: where exactly those numbers are coming from?
  //@ep: why yuv and not hsl or hsv???
  //@ep: where is the colors ordering stored???
  convColor.ch1 = (0.257 * color.r) + (0.504 * color.g) + (0.098 * color.b) + 16;
  convColor.ch2 = -(0.148 * color.r) - (0.291 * color.g) + (0.439 * color.b) + 128;
  convColor.ch3 = (0.439 * color.r) - (0.368 * color.g) - (0.071 * color.b) + 128;
  /*
   int Y =  (0.257 * pt.r) + (0.504 * pt.g) + (0.098 * pt.b) + 16;
   i nt U = -(0.148 * pt.r) - (0.291 * pt.g) + (0.439 * pt.b) + 128;*
   int V =  (0.439 * pt.r) - (0.368 * pt.g) - (0.071 * pt.b) + 128;
   */
}

void ColorHistogram::getRGB(v4r::RGBValue &color, Color3C &convColor) {
  convColor.ch1 = color.r;
  convColor.ch2 = color.g;
  convColor.ch3 = color.b;
}

bool ColorHistogram::getColor(v4r::RGBValue color, Color3C &convColor) {
  switch (colorModel) {
    case YUV_MODEL:
      getYUV(color, convColor);
      return (true);
    case RGB_MODEL:
      getRGB(color, convColor);
      return (true);
    case BGR_MODEL:
      return (false);
    default:
      return (false);
  }
}

bool ColorHistogram::buildHistogram3D() {
  if (colorModel != YUV_MODEL) {
    LOG(ERROR) << "only yuv model is supported for 3D histogram at the moment!";
    return (false);
  }

  int noCol = 0;
  for (size_t i = 0; i < indices->indices.size(); i++) {
    //@ep: is PointXYZRGB::r correct?
    // pcl::RGBValue color = ....   // out of date! instead use PointXYZRGB::r directly
    v4r::RGBValue color;
    color.float_value = cloud->points.at(indices->indices.at(i)).rgb;

    Color3C convColor;
    if (!getColor(color, convColor))
      return (false);

    int U2 = convColor.ch2 - 128;  // shifted to the middle
    int V2 = convColor.ch3 - 128;  // shifted to the middle
    if ((U2 * U2 + V2 * V2) < UVthreshold) {
      noCol++;
    } else {
      double yBin = convColor.ch1 * (double)nrBins / maxVal;
      double uBin = convColor.ch2 * (double)nrBins / maxVal;
      double vBin = convColor.ch3 * (double)nrBins / maxVal;

      hist.at<double>((int)yBin, ((int)uBin) * nrBins + (int)vBin) += 1;
    }
  }

  double normalization = indices->indices.size() - noCol;
  if (normalization != 0) {
    for (int i = 0; i < nrBins; i++) {
      for (int j = 0; j < nrBins; j++) {
        for (int k = 0; k < nrBins; k++) {
          hist.at<double>((int)i, ((int)j) * nrBins + (int)k) /= normalization;
        }
      }
    }
  } else {
    for (int i = 0; i < nrBins; i++) {
      for (int j = 0; j < nrBins; j++) {
        for (int k = 0; k < nrBins; k++) {
          hist.at<double>((int)i, ((int)j) * nrBins + (int)k) = 0;
        }
      }
    }
  }

  computed = true;
  return (true);
}

bool ColorHistogram::buildHistogram2D() {
  if (colorModel != YUV_MODEL) {
    LOG(ERROR) << "only yuv model is supported for 3D histogram at the moment!";
    return (false);
  }

  int noCol = 0;
  for (unsigned i = 0; i < indices->indices.size(); i++) {
    v4r::RGBValue color;
    color.float_value = cloud->points.at(indices->indices.at(i)).rgb;

    Color3C convColor;
    if (!getColor(color, convColor))
      return (false);

    int U2 = convColor.ch2 - 128;  /// shifted to the middle
    int V2 = convColor.ch3 - 128;  /// shifted to the middle

    if ((U2 * U2 + V2 * V2) < UVthreshold) {
      noCol++;
    } else {
      double uBin = convColor.ch2 * (double)nrBins / maxVal;
      double vBin = convColor.ch3 * (double)nrBins / maxVal;
      hist.at<double>((int)uBin, (int)vBin) += 1;
    }
  }

  double normalization = indices->indices.size() - noCol;
  if (normalization != 0) {
    for (int i = 0; i < nrBins; i++) {
      for (int j = 0; j < nrBins; j++) {
        hist.at<double>(i, j) /= normalization;
      }
    }
  } else {
    for (int i = 0; i < nrBins; i++) {
      for (int j = 0; j < nrBins; j++) {
        hist.at<double>(i, j) = 0;
      }
    }
  }

  return (true);
}

bool ColorHistogram::buildHistogramAverage() {
  for (unsigned i = 0; i < indices->indices.size(); i++) {
    v4r::RGBValue color;
    color.float_value = cloud->points.at(indices->indices.at(i)).rgb;

    Color3C convColor;
    if (!getColor(color, convColor))
      return (false);

    if (colorModel != YUV_MODEL) {
      double bin1 = convColor.ch1 * (double)nrBins / maxVal;
      hist.at<double>((int)bin1, 0) += 1;
    }
    double bin2 = convColor.ch2 * (double)nrBins / maxVal;
    hist.at<double>((int)bin2, 1) += 1;
    double bin3 = convColor.ch3 * (double)nrBins / maxVal;
    hist.at<double>((int)bin3, 2) += 1;
  }

  for (int i = 0; i < nrBins; i++) {
    hist.at<double>(i, 0) /= indices->indices.size();
    hist.at<double>(i, 1) /= indices->indices.size();
    hist.at<double>(i, 2) /= indices->indices.size();
  }

  return (true);
}

bool ColorHistogram::buildHistogram() {
  switch (histogramType) {
    case HIST_AVERAGE_HIST:
      return (buildHistogramAverage());
    case HIST_2D:
      return (buildHistogram2D());
    case HIST_3D:
      return (buildHistogram3D());
    default:
      return (false);
  }
}

void ColorHistogram::compute() {
  if (!init()) {
    throw std::runtime_error("[ColorHistogram::compute::init]: Error: Cannot init histograms. Check histogram type.");
  }

  if (!buildHistogram()) {
    throw std::runtime_error(
        "[ColorHistogram::compute::buildHistogram]: Error: Cannot build histograms. Check histogram type.");
  }

  computed = true;
}

double ColorHistogram::compareAverage(ColorHistogram::Ptr ch) {
  //@ep: why fidelity metric???
  // YUV: Fidelity d=(SUM(sqrt(Pi*Qi)))
  double overall_sum = 0;
  for (int i = 0; i < nrBins; i++) {
    if (colorModel != YUV_MODEL) {
      overall_sum += sqrt(hist.at<double>(i, 0) * (ch->getHist()).at<double>(i, 0));
    }
    overall_sum += sqrt(hist.at<double>(i, 1) * (ch->getHist()).at<double>(i, 1));
    overall_sum += sqrt(hist.at<double>(i, 2) * (ch->getHist()).at<double>(i, 2));
  }

  switch (colorModel) {
    case YUV_MODEL:
      return (overall_sum / 2);
    case RGB_MODEL:
      return (overall_sum / 3);
    default:
      return (0.0);
  }

  return (0.0);
}

double ColorHistogram::compare2D(ColorHistogram::Ptr ch) {
  // YUV: Fidelity d=(SUM(sqrt(Pi*Qi)))
  double overall_sum = 0;
  for (int i = 0; i < nrBins; i++) {
    for (int j = 0; j < nrBins; j++) {
      overall_sum += sqrt(hist.at<double>(i, j) * (ch->hist).at<double>(i, j));
    }
  }

  if (overall_sum > 1.) {
    LOG(WARNING) << "Value larger than 1!";
    printHistogram();
  }

  double fidelity = overall_sum;
  return fidelity;
}

double ColorHistogram::compare3D(ColorHistogram::Ptr ch) {
  // YUV: Fidelity d=(SUM(sqrt(Pi*Qi)))
  double overall_sum = 0;
  for (int i = 0; i < nrBins; i++) {
    for (int j = 0; j < nrBins; j++) {
      for (int k = 0; k < nrBins; k++) {
        overall_sum += sqrt((hist.at<double>((int)i, ((int)j) * nrBins + (int)k)) *
                            (ch->hist).at<double>((int)i, ((int)j) * nrBins + (int)k));
      }
    }
  }

  if (overall_sum > 1.) {
    LOG(WARNING) << "Value larger than 1!";
    printHistogram();
  }

  double fidelity = overall_sum;

  return fidelity;
}

double ColorHistogram::compare(ColorHistogram::Ptr ch) {
  if (nrBins != ch->getBinNumber()) {
    LOG(ERROR) << "Cannot compare histograms with different bin sizes.";
    return 0.;
  }
  if (!computed || !ch->getComputed()) {
    LOG(ERROR) << "Color histogram not computed.";
    return 0.;
  }

  if (colorModel != ch->getColorModel()) {
    LOG(ERROR) << "Different color models.";
    return 0.;
  }

  if (histogramType != ch->getHistogramType()) {
    LOG(ERROR) << "Different histogram types.";
    return 0.;
  }

  switch (histogramType) {
    case HIST_AVERAGE_HIST:
      return (compareAverage(ch));
    case HIST_2D:
      return (compare2D(ch));
    case HIST_3D:
      return (compare3D(ch));
    default:
      return 0.;
  }
}

void ColorHistogram::printHistogramAverage() {
  LOG(INFO) << "[Histogram Average]:";

  switch (colorModel) {
    case YUV_MODEL:
      for (int i = 0; i < nrBins; i++)
        LOG(INFO) << " y[" << i << "]: " << hist.at<double>(i, 0);
      for (int i = 0; i < nrBins; i++)
        LOG(INFO) << " u[" << i << "]: " << hist.at<double>(i, 1);
      for (int i = 0; i < nrBins; i++)
        LOG(INFO) << " v[" << i << "]: " << hist.at<double>(i, 2);
      return;
    case RGB_MODEL:
      for (int i = 0; i < nrBins; i++)
        LOG(INFO) << " red[" << i << "]: " << hist.at<double>(i, 0);
      for (int i = 0; i < nrBins; i++)
        LOG(INFO) << " gre[" << i << "]: " << hist.at<double>(i, 1);
      for (int i = 0; i < nrBins; i++)
        LOG(INFO) << " blu[" << i << "]: " << hist.at<double>(i, 2);
      return;
    default:
      return;
  }

  return;
}

void ColorHistogram::printHistogram2D() {
  LOG(INFO) << "[Histogram2D]:";

  switch (colorModel) {
    case YUV_MODEL:
      for (int i = 0; i < nrBins; i++)
        for (int j = 0; j < nrBins; j++)
          LOG(INFO) << " yuv[" << i << "][" << j << "]: " << hist.at<double>(i, j);
      return;
    default:
      return;
  }

  return;
}

void ColorHistogram::printHistogram3D() {
  LOG(INFO) << "[Histogram3D]:";

  for (int i = 0; i < nrBins; i++)
    for (int j = 0; j < nrBins; j++)
      for (int k = 0; k < nrBins; k++)
        LOG(INFO) << " yuv[" << i << "][" << j << "][" << k << "]: " << hist.at<double>(i, j * nrBins + k);

  return;
}

void ColorHistogram::printHistogram() {
  switch (histogramType) {
    case HIST_AVERAGE_HIST:
      printHistogramAverage();
      return;
    case HIST_2D:
      printHistogram2D();
      return;
    case HIST_3D:
      printHistogram3D();
      return;
    default:
      return;
  }
}
}  // namespace v4r
