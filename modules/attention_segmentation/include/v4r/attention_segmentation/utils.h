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

#ifndef EPUTILS_H
#define EPUTILS_H

#include <v4r/core/macros.h>
#include "v4r/attention_segmentation/eputils_headers.h"

namespace v4r {

/**
 * reads files from a given directory
 * */
V4R_EXPORTS int readFiles(const std::string &directoryName, std::vector<std::string> &names);
/**
 * reads coordinated of the polygons from the given file
 * */
// ep:begin revision 18-07-2014
V4R_EXPORTS void readPolygons(std::vector<std::vector<cv::Point>> &polygons, std::string filename);
// ep:end revision 18-07-2014

/**
 * write coordinated of the polygons from the given file
 * */
V4R_EXPORTS void writePolygons(std::vector<std::vector<cv::Point>> &polygons, std::string &str);

/**
 * reads rectangles form fole
 * */
V4R_EXPORTS void readRectangles(std::vector<cv::Rect> &rectangles, std::string &str);
/**
 * reads attention points from file
 * */
V4R_EXPORTS void readAttentionPoints(std::vector<std::vector<cv::Point>> &attentionPoints, std::string &str);

// revision
V4R_EXPORTS void readAttentionPoints(std::vector<cv::Point> &attentionPoints, std::string &str);
/**
 * writes attention points to file
 * */
V4R_EXPORTS void writeAttentionPoints(std::vector<cv::Point> attentionPoints, std::string &str);
/**
 * reads attention points and contours from file
 * */
// end revision
V4R_EXPORTS void readAttentionPointsAndContours(std::vector<cv::Point> &attentionPoints,
                                                std::vector<std::vector<cv::Point>> &contours, std::string &str);
/**
 * reads ground truth segmentation from the text file
 * */
// ep:begin revision 18-07-2014
V4R_EXPORTS void readAnnotationsFromFile(std::vector<std::vector<cv::Point>> &polygons, std::string filename);
// ep:end revision 18-07-2014
/**
 * saturates image
 * */
V4R_EXPORTS void saturation(cv::Mat &map, float max_value = 1.0);
/**
 * calculates centers of the polygons
 * */
V4R_EXPORTS void calculatePolygonsCenters(std::vector<std::vector<cv::Point>> &polygons,
                                          std::vector<cv::Point> &attentionPoints,
                                          std::vector<int> &attentionPointsBelongTo);

double PolygonArea(std::vector<cv::Point> &polygon);
V4R_EXPORTS void Centroid(std::vector<cv::Point> &polygon, cv::Point &center);
V4R_EXPORTS void makeGaborFilter(cv::Mat &filter0, cv::Mat &filter90, float angle, float stddev = 2.33,
                                 float elongation = 1, int filterSize = 9, int filterPeriod = 7);

V4R_EXPORTS void readCenters(std::vector<cv::Point> &centers, std::string &str);
V4R_EXPORTS void makeGaborKernel2D(cv::Mat &kernel, float &max_sum, float theta = 0, float bandwidth = 4,
                                   float lambda = 10, float sigma = 0, float phi = 0, float gamma = 0.5);
V4R_EXPORTS void saveDistribution(std::vector<float> dist, std::string filename);
V4R_EXPORTS void readDistribution(std::vector<float> &dist, std::string filename);

}  // namespace v4r

#endif  // EPUTILS_H
