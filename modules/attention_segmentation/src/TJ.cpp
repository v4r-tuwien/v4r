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

#include "v4r/attention_segmentation/TJ.h"

namespace v4r {

/*
int dy8[8] = {-1,-1,-1,0,1,1,1,0};
int dx8[8] = {-1,0,1,1,1,0,-1,-1};

int dx4[4] = {-1,1,0,0};
int dy4[4] = {0,0,-1,1};
*/

SaliencyLine::SaliencyLine() {
  points.clear();
  saliency = 0;
  points_num = 0;
}

void SaliencyLine::clear() {
  points.clear();
  saliency = 0;
  points_num = 0;
}

float SaliencyLine::getSaliency() {
  return (saliency);
}

std::vector<JunctionNode> SaliencyLine::getPoints() {
  return (points);
}

int SaliencyLine::getPointsNumber() {
  return (points_num);
}

void SaliencyLine::addPoint(JunctionNode node) {
  points.push_back(node);
  saliency = ((saliency * points_num) + node.saliency) / (points_num + 1);
  points_num += 1;
}

bool calculateSaliencyLine(cv::Mat mask, const cv::Mat symmetry, SaliencyLine &saliencyLine, unsigned int th) {
  assert(mask.size() == symmetry.size());

  int width = mask.cols;
  int height = mask.rows;

  saliencyLine.clear();

  // we assume that each mask contains only one connected component
  int count = 0;
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      if (mask.at<uchar>(i, j)) {
        count++;
        mask.at<uchar>(i, j) = count;
      }
    }
  }

  // create saliency line
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      if (mask.at<uchar>(i, j)) {
        JunctionNode currentNode;

        currentNode.x = j;
        currentNode.y = i;
        currentNode.num = mask.at<uchar>(i, j);
        currentNode.type = UNKNOWN;
        currentNode.saliency = symmetry.at<float>(i, j);

        for (int k = 0; k < 8; ++k) {
          int nx = j + dx8[k];
          int ny = i + dy8[k];

          if ((ny < 0) || (nx < 0) || (ny >= height) || (nx >= width))
            continue;

          if (mask.at<uchar>(ny, nx) > 0) {
            currentNode.edges_num += 1;
            currentNode.edges.push_back(mask.at<uchar>(ny, nx));
            currentNode.type += 1;
          }
        }

        saliencyLine.addPoint(currentNode);
      }
    }
  }

  if (saliencyLine.getPoints().size() < th) {
    return (false);
  }

  return (true);
}

bool findTJunctions(SaliencyLine saliencyLine, std::vector<int> &tjunctionPointsIdx) {
  tjunctionPointsIdx.clear();
  std::vector<JunctionNode> nodes = saliencyLine.getPoints();
  for (unsigned int i = 0; i < nodes.size(); ++i) {
    if (nodes.at(i).type >= T_JUNCTION) {
      tjunctionPointsIdx.push_back(i);
    }
  }

  if (tjunctionPointsIdx.size())
    return (true);

  return (false);
}

std::vector<int> findEndPoints(SaliencyLine saliencyLine, std::vector<int> segment) {
  std::vector<JunctionNode> points = saliencyLine.getPoints();

  std::vector<int> endPoints;
  for (unsigned int i = 0; i < segment.size(); ++i) {
    if (points.at(segment.at(i)).type == END_POINT) {
      endPoints.push_back(segment.at(i));
    }
  }
  return (endPoints);
}

std::vector<int> findEndPoints(SaliencyLine saliencyLine) {
  std::vector<JunctionNode> points = saliencyLine.getPoints();

  std::vector<int> endPoints;
  for (unsigned int i = 0; i < points.size(); ++i) {
    if (points.at(i).type == END_POINT) {
      endPoints.push_back(i);
    }
  }

  if (!(endPoints.size()))
    endPoints.push_back(0);

  return (endPoints);
}

std::vector<int> getEdges(std::vector<JunctionNode> nodes, int nodeIdx) {
  std::vector<int> edgesIdx;
  for (unsigned int i = 0; i < nodes.at(nodeIdx).edges.size(); ++i) {
    int edgeIdxCurrent = nodes.at(nodeIdx).edges.at(i);

    for (unsigned int j = 0; j < nodes.size(); ++j) {
      if (nodes.at(j).num == edgeIdxCurrent) {
        edgesIdx.push_back(j);
        break;
      }
    }
  }

  return (edgesIdx);
}

void breakIntoSegments(SaliencyLine saliencyLine, std::vector<std::vector<int>> &segments) {
  int pointsNumber = saliencyLine.getPointsNumber();
  std::vector<JunctionNode> nodes = saliencyLine.getPoints();

  std::vector<int> tjunctionPointsIdx;
  if (!findTJunctions(saliencyLine, tjunctionPointsIdx)) {
    std::vector<int> segmentCurrent;
    segmentCurrent.resize(pointsNumber);
    for (int i = 0; i < pointsNumber; ++i) {
      segmentCurrent.at(i) = i;
    }
    segments.push_back(segmentCurrent);
    return;
  }

  std::vector<bool> usedPoints(pointsNumber, false);

  // go over all t-junctions
  for (unsigned int i = 0; i < tjunctionPointsIdx.size(); ++i) {
    int tjunctionPointIdxCurrent = tjunctionPointsIdx.at(i);
    usedPoints.at(tjunctionPointIdxCurrent) = true;

    // find endges
    std::vector<int> currentEdges = getEdges(nodes, tjunctionPointIdxCurrent);

    // go over all segments
    for (unsigned int j = 0; j < currentEdges.size(); ++j) {
      std::vector<int> segmentCurrent;
      int currentNodeIdx = currentEdges.at(j);
      while (!usedPoints.at(currentNodeIdx)) {
        usedPoints.at(currentNodeIdx) = true;
        segmentCurrent.push_back(currentNodeIdx);
        std::vector<int> currentEdgesTemp = getEdges(nodes, currentNodeIdx);
        for (unsigned int k = 0; k < currentEdgesTemp.size(); ++k) {
          if ((!usedPoints.at(currentEdgesTemp.at(k))) && (nodes.at(currentEdgesTemp.at(k)).type < T_JUNCTION)) {
            currentNodeIdx = currentEdgesTemp.at(k);
            break;
          }
        }
      }
      segments.push_back(segmentCurrent);
    }
  }
}

void modifySymmetryLine(SaliencyLine saliencyLine, std::vector<bool> &usedPoints, float th) {
  std::vector<std::vector<int>> segments;
  breakIntoSegments(saliencyLine, segments);
  std::vector<JunctionNode> nodes = saliencyLine.getPoints();

  usedPoints.resize(saliencyLine.getPointsNumber(), true);

  for (unsigned int i = 0; i < segments.size(); ++i) {
    std::vector<int> segmentCurrent = segments.at(i);
    std::vector<int> endPoints = findEndPoints(saliencyLine, segmentCurrent);

    if (endPoints.size() <= 0)
      continue;

    unsigned int nodesToDeleteNum = (unsigned int)(segmentCurrent.size() - (int)(th * (segmentCurrent.size())));
    unsigned int j = 0;
    while (j < nodesToDeleteNum) {
      for (unsigned int k = 0; k < endPoints.size(); ++k) {
        int endPointIdx = endPoints.at(k);
        usedPoints.at(endPointIdx) = false;
        std::vector<int> currentEdges = getEdges(nodes, endPointIdx);
        for (unsigned int l = 0; l < currentEdges.size(); ++l) {
          if (usedPoints.at(currentEdges.at(l))) {
            usedPoints.at(currentEdges.at(l)) = false;
            endPoints.at(k) = currentEdges.at(l);
            j++;
            break;
          } else if ((l + 1) == currentEdges.size()) {
            j++;
          }
        }
      }
    }
  }
}

void selectSaliencyCenterPoint(SaliencyLine saliencyLine, PointSaliency &center) {
  std::vector<JunctionNode> nodes = saliencyLine.getPoints();

  std::vector<int> tjunctionPointsIdx;
  bool isTJunction = findTJunctions(saliencyLine, tjunctionPointsIdx);
  int centerIdx = 0;

  if (isTJunction) {
    centerIdx = tjunctionPointsIdx.at(0);
    for (unsigned int i = 1; i < tjunctionPointsIdx.size(); ++i) {
      if (nodes.at(centerIdx).type < nodes.at(tjunctionPointsIdx.at(i)).type) {
        centerIdx = tjunctionPointsIdx.at(i);
      }
    }
  } else {
    std::vector<int> endPoints = findEndPoints(saliencyLine);
    centerIdx = endPoints.at(0);
    unsigned int nodesToDeleteNum = nodes.size() / 2;
    std::vector<bool> usedPoints(nodes.size(), true);

    unsigned int j = 0;
    while (j < nodesToDeleteNum) {
      usedPoints.at(centerIdx) = false;
      std::vector<int> currentEdges = getEdges(nodes, centerIdx);
      for (unsigned int k = 0; k < currentEdges.size(); ++k) {
        if (usedPoints.at(currentEdges.at(k))) {
          centerIdx = currentEdges.at(k);
          usedPoints.at(centerIdx) = false;
          j++;
          break;
        } else if ((k + 1) == currentEdges.size()) {
          j++;
        }
      }
    }
  }

  center.point = cv::Point(nodes.at(centerIdx).x, nodes.at(centerIdx).y);
  center.saliency = saliencyLine.getSaliency();
}

void createSimpleLine(SaliencyLine saliencyLine, std::vector<cv::Point> &points) {
  std::vector<JunctionNode> nodes = saliencyLine.getPoints();
  points.resize(nodes.size());
  for (unsigned int i = 0; i < nodes.size(); ++i) {
    int x = nodes.at(i).x;
    int y = nodes.at(i).y;

    points.at(i) = cv::Point(x, y);
  }
}

bool extractSaliencyLine(cv::Mat mask, cv::Mat map, SaliencyLine &saliencyLine, unsigned int th) {
  cv::Mat skeleton;
  Skeleton(mask, skeleton);

  // cv::imshow("skeleton",255*skeleton);
  // cv::waitKey();

  if (!calculateSaliencyLine(skeleton, map, saliencyLine, th)) {
    return false;
  }

  return (true);
}

void createAttentionPoints(std::vector<PointSaliency> saliencyPoints, std::vector<cv::Point> &attentionPoints) {
  attentionPoints.clear();
  for (unsigned int i = 0; i < saliencyPoints.size(); ++i) {
    attentionPoints.push_back(saliencyPoints.at(i).point);
  }
}
}  // namespace v4r
