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

#include "v4r/attention_segmentation/SurfaceModel.h"

namespace v4r {

bool operator==(const borderIdentification& left, const borderIdentification& right) {
  return ((left.p1 == right.p1) && (left.p2 == right.p2));
}

bool operator<(const borderIdentification& left, const borderIdentification& right) {
  if (left.p1 < right.p1)
    return (true);
  else if (left.p1 == right.p1) {
    return (left.p2 < right.p2);
  } else
    return (false);
}

void View::calculateBorders(pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pcl_cloud) {
  if (!havePatchImage)
    return;

  double max3DDistancePerMeter = 0.007;  // 7 mm

  for (int i = 1; i < height; i++) {
    for (int j = 1; j < width; j++) {
      neighboringPair borderPixel[4];
      int p1[4], p2[4];
      bool used[4];
      double border_distance[4];
      //@ep: this value should be settable
      double b_distance = 10.;

      //  priorities
      //  4 1 3
      //  2 x .
      //  . . .

      int dx[4] = {-1, 1, -1, 0};
      int dy[4] = {-1, -1, 0, -1};
      int dir[4] = {1, 3, 0, 2};

      bool found = false;

      for (int k = 0; k < 4; k++) {
        used[k] = false;

        borderPixel[k].idx1 = 0;
        borderPixel[k].idx2 = 0;
        borderPixel[k].direction = 0;

        p1[k] = -1;
        p2[k] = -1;

        int i1 = i;
        int i2 = i + dy[k];
        int j1 = j;
        int j2 = j + dx[k];

        if ((patchImage.at<int>(i1, j1) == -1) || (patchImage.at<int>(i2, j2) == -1))
          continue;

        // its a border if patches are different
        if (patchImage.at<int>(i1, j1) != patchImage.at<int>(i2, j2)) {
          int p1_ = (patchImage.at<int>(i1, j1) < patchImage.at<int>(i2, j2) ? patchImage.at<int>(i1, j1)
                                                                             : patchImage.at<int>(i2, j2));
          int p2_ = (patchImage.at<int>(i1, j1) > patchImage.at<int>(i2, j2) ? patchImage.at<int>(i1, j1)
                                                                             : patchImage.at<int>(i2, j2));

          int idx1 = i1 * width + j1;
          int idx2 = i2 * width + j2;

          // distance between
          double distance = fabs(current_pcl_cloud->points.at(idx1).z - current_pcl_cloud->points.at(idx2).z);

          if (distance < b_distance) {
            p1[k] = p1_;
            p2[k] = p2_;

            //@ep: BUG ordering should be consistent with patch numbers!!!
            borderPixel[k].idx1 = idx1;
            borderPixel[k].idx2 = idx2;
            border_distance[k] = distance;
            borderPixel[k].direction = dir[k];
            used[k] = true;
            found = true;
          }
        }
      }

      if (found) {
        for (int k = 0; k < 4; ++k) {
          // if k-th direction is used, then try to find the best one
          if (used[k]) {
            int selected_k = k;
            for (int l = k + 1; l < 4; ++l)  // k+1
            {
              if (!used[l])
                continue;

              if ((p1[selected_k] == p1[l]) && (p2[selected_k] == p2[l])) {
                if (border_distance[l] < border_distance[selected_k]) {
                  used[selected_k] = false;
                  //@ep:: somethins is wrong with direction here, since it doesn't change
                  selected_k = l;
                } else {
                  used[l] = false;
                }

                //@ep: BUG this is a bug, the direction should be consistent with selected points
                if (borderPixel[selected_k].direction != borderPixel[l].direction) {
                  borderPixel[selected_k].direction = borderPixel[l].direction;
                }
              }
            }
          }
        }

        for (int k = 0; k < 4; ++k) {
          if (!used[k])
            continue;

          borderIdentification borderId;
          // p1 < p2 ALWAYS!!!
          borderId.p1 = (p1[k] < p2[k] ? p1[k] : p2[k]);
          borderId.p2 = (p1[k] < p2[k] ? p2[k] : p1[k]);

          std::map<borderIdentification, std::vector<neighboringPair>>::iterator it2D;
          it2D = ngbr2D_map.find(borderId);
          if (it2D != ngbr2D_map.end()) {
            (it2D->second).push_back(borderPixel[k]);
          } else {
            std::pair<borderIdentification, std::vector<neighboringPair>> new_border;
            new_border.first = borderId;
            (new_border.second).push_back(borderPixel[k]);
            ngbr2D_map.insert(new_border);
          }
          // z coordinate of the point
          double z_dist = current_pcl_cloud->points.at(i * width + j).z;
          double adaptive_distance = max3DDistancePerMeter * z_dist;
          if (border_distance[k] < adaptive_distance) {
            std::map<borderIdentification, std::vector<neighboringPair>>::iterator it3D;
            it3D = ngbr3D_map.find(borderId);
            if (it3D != ngbr3D_map.end()) {
              (it3D->second).push_back(borderPixel[k]);
            } else {
              std::pair<borderIdentification, std::vector<neighboringPair>> new_border;
              new_border.first = borderId;
              (new_border.second).push_back(borderPixel[k]);
              ngbr3D_map.insert(new_border);
            }
          }
        }
      }
    }
  }
}

bool View::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
  cloud = _cloud;

  height = cloud->height;
  width = cloud->width;
  patchImage = cv::Mat_<int>(height, width);
  patchImage.setTo(-1);

  havePatchImage = false;

  return (true);
}

bool View::setSurfaces(std::vector<SurfaceModel::Ptr> _surfaces) {
  surfaces = _surfaces;
  for (unsigned int i = 0; i < surfaces.size(); i++) {
    surfaces.at(i)->neighbors2D.clear();
    surfaces.at(i)->neighbors3D.clear();
    //@ep: we do not use it
    surfaces.at(i)->neighbors2DNrPixel.clear();
  }

  ngbr2D_map.clear();
  ngbr3D_map.clear();

  return (true);
}

bool View::createPatchImage() {
  patchImage.setTo(-1);

  for (unsigned int i = 0; i < surfaces.size(); ++i) {
    for (unsigned int j = 0; j < surfaces.at(i)->indices.size(); j++) {
      int x = (surfaces.at(i)->indices.at(j)) % width;
      int y = (surfaces.at(i)->indices.at(j)) / width;
      patchImage.at<int>(y, x) = i;
    }
  }

  havePatchImage = true;

  return havePatchImage;
}

void View::updatePatchImage(std::vector<int> addedTo) {
  if (!havePatchImage)
    return;

  for (unsigned int i = 0; i < surfaces.size(); i++) {
    if (addedTo.at(i) >= 0) {
      int new_i = addedTo.at(i);
      while (addedTo.at(new_i) != -1) {
        new_i = addedTo.at(new_i);
      }

      for (unsigned int j = 0; j < surfaces.at(i)->indices.size(); j++) {
        int x = (surfaces.at(i)->indices.at(j)) % width;
        int y = (surfaces.at(i)->indices.at(j)) / width;
        patchImage.at<int>(y, x) = new_i;
      }
    }
  }
}

void View::printPatchImage(std::string file_name) {
  if (!havePatchImage)
    return;

  FILE* f = fopen(file_name.c_str(), "w");

  for (int i = 0; i < patchImage.rows; ++i) {
    for (int j = 0; j < patchImage.cols; ++j) {
      fprintf(f, "%d ", patchImage.at<int>(i, j));
    }
    fprintf(f, "\n");
  }

  fclose(f);
}

void View::get2DNeighborsCurrent(cv::Mat& neighbors2D, cv::Mat& neighbors3D) {
  neighbors2D = cv::Mat_<bool>(surfaces.size(), surfaces.size());
  neighbors2D.setTo(false);
  neighbors3D = cv::Mat_<bool>(surfaces.size(), surfaces.size());
  neighbors3D.setTo(false);

  double z_max_dist = 0.01;

  //@ep TODO: uncomment?
  //   int dr[4] = {-1,-1, 0, 1};
  //   int dc[4] = { 0,-1,-1,-1};

  int dr[4] = {-1, 0, -1};
  int dc[4] = {0, -1, -1};

  for (unsigned int i = 0; i < surfaces.size(); ++i) {
    for (unsigned int j = 0; j < surfaces.at(i)->indices.size(); j++) {
      int c = (surfaces.at(i)->indices.at(j)) % width;
      int r = (surfaces.at(i)->indices.at(j)) / width;
      int patchIdx = patchImage.at<int>(r, c);

      //@ep: why we did not use 1,-1 shift???
      for (int i = 0; i < 3; ++i)  //@ep: TODO 3->4
      {
        int nr = r + dr[i];
        int nc = c + dc[i];

        if ((nr >= height) || (nc >= width) || (nr < 0) || (nc < 0))
          continue;

        int currentPatchIdx = patchImage.at<int>(nr, nc);
        if (currentPatchIdx == -1)
          continue;

        if (patchIdx != currentPatchIdx) {
          neighbors2D.at<bool>(currentPatchIdx, patchIdx) = true;
          neighbors2D.at<bool>(patchIdx, currentPatchIdx) = true;

          int idx0 = r * width + c;
          int idx1 = nr * width + nc;
          // @ep:: are we wure that this should be distance only in z direction, and not Euclidean distance???
          double dis = fabs(cloud->points.at(idx0).z - cloud->points.at(idx1).z);
          if (dis < z_max_dist) {
            neighbors3D.at<bool>(currentPatchIdx, patchIdx) = true;
            neighbors3D.at<bool>(patchIdx, currentPatchIdx) = true;
          }
        }
      }
    }
  }
}

/**
 * computeNeighbors
 */
void View::computeNeighbors() {
  if (!havePatchImage)
    return;

  //@ep: Why in the original version we've been using only 3 directions out of 4 (no use of +1,-1 shift)
  cv::Mat neighbors2D, neighbors3D;
  get2DNeighborsCurrent(neighbors2D, neighbors3D);

  for (unsigned int i = 0; i < surfaces.size(); i++) {
    surfaces.at(i)->neighbors2D.clear();
    surfaces.at(i)->neighbors3D.clear();
  }

  // because neigbors are symmetrcal it is enough to go only through the half
  for (unsigned int i = 0; i < surfaces.size(); i++) {
    for (unsigned int j = 0; j < i; j++) {
      if (neighbors2D.at<bool>(i, j)) {
        surfaces.at(i)->neighbors2D.insert(j);
        surfaces.at(j)->neighbors2D.insert(i);
      }

      if (neighbors3D.at<bool>(i, j)) {
        surfaces.at(i)->neighbors3D.insert(j);
        surfaces.at(j)->neighbors3D.insert(i);
      }
    }
  }
}

void View::setSaliencyMap(cv::Mat& _saliencyMap) {
  _saliencyMap.copyTo(saliencyMap);
  haveSaliencyMap = true;
}

bool View::compareSaliency(const SurfaceModelPair& sm1, const SurfaceModelPair& sm2) {
  return ((sm1.surface->saliency > sm2.surface->saliency) ? true : false);
}

void View::sortPatches() {
  sortedSurfaces.resize(surfaces.size());
  for (unsigned int i = 0; i < surfaces.size(); ++i) {
    surfaces.at(i)->saliency = 1;
    sortedSurfaces.at(i) = i;
  }

  if (!haveSaliencyMap) {
    VLOG(1) << "Saliency map not set! Using default ordering.";
  }

  height = cloud->height;
  width = cloud->width;

  for (unsigned int i = 0; i < surfaces.size(); ++i) {
    if ((!(surfaces.at(i)->valid)) && (surfaces.at(i)->segmented_number == -1)) {
      surfaces.at(i)->saliency = 0;
      continue;
    }

    if ((surfaces.at(i)->indices.size()) < 50) {
      surfaces.at(i)->saliency = 0;
      continue;
    }

    float saliency = 0;
    for (unsigned int j = 0; j < surfaces.at(i)->indices.size(); ++j) {
      int idx = surfaces.at(i)->indices.at(j);
      int x = idx % width;
      int y = idx / width;
      saliency += saliencyMap.at<float>(y, x);
    }
    saliency /= surfaces.at(i)->indices.size();
    surfaces.at(i)->saliency = saliency;
  }

  std::vector<SurfaceModelPair> surfacePairs;
  surfacePairs.resize(surfaces.size());

  for (unsigned int i = 0; i < surfacePairs.size(); ++i) {
    SurfaceModelPair surfaceModelPair;
    surfacePairs.at(i).surface = surfaces.at(i);
    surfacePairs.at(i).index = i;
  }

  sort(surfacePairs.begin(), surfacePairs.end(),
       [this](SurfaceModelPair sm1, SurfaceModelPair sm2) { return compareSaliency(sm1, sm2); });

  for (unsigned int i = 0; i < surfacePairs.size(); ++i) {
    sortedSurfaces.at(i) = surfacePairs.at(i).index;
  }
}

}  // namespace v4r
