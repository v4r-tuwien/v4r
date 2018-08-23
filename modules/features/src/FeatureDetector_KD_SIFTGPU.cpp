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
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#include <GL/glut.h>
#include <v4r/features/FeatureDetector_KD_SIFTGPU.h>
#include <Eigen/Dense>

namespace v4r {

using namespace std;

FeatureDetector_KD_SIFTGPU::FeatureDetector_KD_SIFTGPU(const Parameter &_p, const cv::Ptr<SiftGPU> &_sift)
: FeatureDetector(FeatureDetector::Type::KD_SIFTGPU), param(_p) {
  descr_name_ = "sift_gpu";
  // init sift
  if (_sift.empty()) {
    const char *argv[] = {"-m", "-fo", "-1", "-s", "-v", "1", "-pack"};
    int argc = sizeof(argv) / sizeof(char *);
    sift = new SiftGPU();
    sift->ParseParam(argc, (char **)argv);
    // create an OpenGL context for computation
    if (sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
      throw runtime_error("[FeatureDetector_KD_SIFTGPU::FeatureDetector_KD_SIFTGPU] No GL support!");
  } else
    sift = _sift;
}

FeatureDetector_KD_SIFTGPU::~FeatureDetector_KD_SIFTGPU() {}

void FeatureDetector_KD_SIFTGPU::transformToRootSIFT(cv::Mat &descriptors) const {
  float norm;

  for (int i = 0; i < descriptors.rows; i++) {
    Eigen::Map<Eigen::VectorXf> desc(&descriptors.at<float>(i, 0), descriptors.cols);
    norm = desc.lpNorm<1>();
    desc.array() /= norm;
    desc.array() = desc.array().sqrt();
  }
}

void FeatureDetector_KD_SIFTGPU::detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,
                                                  cv::Mat &descriptors, const cv::Mat &object_mask) {
  cv::Mat_<unsigned char> im_gray;
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray, cv::COLOR_RGB2GRAY);
  else
    im_gray = image;

  descriptors = cv::Mat();
  keypoints.clear();

  SiftGPU *pSift = (SiftGPU *)&(*sift);

  if (pSift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    throw std::runtime_error("SiftGPU: No GL support!");

  pSift->VerifyContextGL();

  if (pSift->RunSIFT(im_gray.cols, im_gray.rows, im_gray.ptr<uchar>(0), GL_LUMINANCE, GL_UNSIGNED_BYTE)) {
    int rows = pSift->GetFeatureNum();
    if (rows > 0) {
      vector<SiftGPU::SiftKeypoint> ks(rows);
      descriptors = cv::Mat(rows, 128, CV_32F);

      pSift->GetFeatureVector(&ks[0], descriptors.ptr<float>(0));

      keypoints.resize(rows);
      keypoint_indices_.resize(rows);

      size_t kept = 0;
      for (size_t i = 0; i < keypoints.size(); i++) {
        int u = std::min<int>(im_gray.cols - 1, ks[i].x + 0.5f);
        int v = std::min<int>(im_gray.rows - 1, ks[i].y + 0.5f);
        int idx = v * im_gray.cols + u;

        if (object_mask.empty() || object_mask.at<uchar>(v, u)) {  // keypoint belongs to given object mask
          descriptors.row(i).copyTo(descriptors.row(kept));
          keypoints[kept] = cv::KeyPoint(ks[i].x, ks[i].y, ks[i].s * 3, -ks[i].o * 180 / M_PI, 1, 0, i);
          keypoint_indices_[kept] = idx;
          kept++;
        }
      }
      descriptors.resize(kept);
      keypoints.resize(kept);
      keypoint_indices_.resize(kept);

      if (param.computeRootSIFT)
        transformToRootSIFT(descriptors);

    } else
      cout << "No SIFT found" << endl;
  } else
    throw runtime_error("PSiftGPU::Detect: SiftGPU Error!");
}

void FeatureDetector_KD_SIFTGPU::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,
                                        const cv::Mat &object_mask) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  keypoints.clear();

  SiftGPU *pSift = (SiftGPU *)&(*sift);

  if (pSift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    throw std::runtime_error("SiftGPU: No GL support!");

  pSift->VerifyContextGL();

  if (pSift->RunSIFT(im_gray_.cols, im_gray_.rows, im_gray_.ptr<uchar>(0), GL_LUMINANCE, GL_UNSIGNED_BYTE)) {
    int num = pSift->GetFeatureNum();
    if (num > 0) {
      vector<SiftGPU::SiftKeypoint> ks(num);

      pSift->GetFeatureVector(&ks[0], NULL);

      // copy sift
      keypoints.resize(num);
      keypoint_indices_.resize(num);
      size_t kept = 0;
      for (int i = 0; i < num; i++) {
        int u = std::min<int>(im_gray_.cols - 1, ks[i].x + 0.5f);
        int v = std::min<int>(im_gray_.rows - 1, ks[i].y + 0.5f);
        int idx = v * im_gray_.cols + u;

        if (object_mask.empty() || object_mask.at<uchar>(v, u)) {  // keypoint belongs to given object mask
          keypoints[kept] = cv::KeyPoint(ks[i].x, ks[i].y, ks[i].s * 3, -ks[i].o * 180 / M_PI, 1, 0, i);
          keypoint_indices_[kept] = idx;
          kept++;
        }
      }
      keypoints.resize(kept);
      keypoint_indices_.resize(kept);
    } else
      cout << "No SIFT found" << endl;
  } else
    throw runtime_error("PSiftGPU::Detect: SiftGPU Error!");
}

void FeatureDetector_KD_SIFTGPU::compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,
                                         cv::Mat &descriptors) {
  if (keypoints.empty())
    return;

  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  vector<SiftGPU::SiftKeypoint> ks(keypoints.size());
  for (size_t i = 0; i < ks.size(); i++) {
    ks[i].x = keypoints[i].pt.x;
    ks[i].y = keypoints[i].pt.y;
    ks[i].s = keypoints[i].size / 3.;
    ks[i].o = -keypoints[i].angle * M_PI / 180.;
  }

  SiftGPU *pSift = (SiftGPU *)&(*sift);

  if (pSift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    throw std::runtime_error("SiftGPU: No GL support!");

  pSift->VerifyContextGL();

  pSift->SetKeypointList(ks.size(), &ks[0], 0);

  if (pSift->RunSIFT(im_gray_.cols, im_gray_.rows, im_gray_.ptr<uchar>(0), GL_LUMINANCE, GL_UNSIGNED_BYTE)) {
    int num = pSift->GetFeatureNum();
    if (num == (int)keypoints.size()) {
      descriptors = cv::Mat(num, 128, CV_32F);
      pSift->GetFeatureVector(NULL, descriptors.ptr<float>(0));
      if (param.computeRootSIFT)
        transformToRootSIFT(descriptors);
    } else
      cerr << "Keypoints returned by SiftGPU is different than the number of keypoints given to this function." << endl;
  } else
    throw runtime_error("PSiftGPU::computeImpl: SiftGPU Error!");
}

std::vector<std::pair<int, int>> FeatureDetector_KD_SIFTGPU::matchSIFT(const cv::Mat &desc1, const cv::Mat &desc2) {
  SiftMatchGPU matcher(4096 * 4);
  matcher.VerifyContextGL();

  matcher.SetDescriptors(0, desc1.rows, desc1.data);  // image 1
  matcher.SetDescriptors(1, desc2.rows, desc2.data);  // image 2

  // match and get result.
  int(*match_buf)[2] = new int[desc1.rows][2];

  //        int num_match = matcher->GetSiftMatch(desc1.rows, match_buf,0.75f, 0.8f, 1);
  int num_match = matcher.GetSiftMatch(desc1.rows, match_buf, 0.5f, 0.95f, 1);

  std::vector<std::pair<int, int>> matches(num_match);
  for (int j = 0; j < num_match; j++)
    matches[j] = std::pair<int, int>(match_buf[j][0], match_buf[j][1]);

  delete[] match_buf;
  return matches;
}

}  // namespace v4r
