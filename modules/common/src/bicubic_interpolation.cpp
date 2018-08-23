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
** For licensing terms and conditions please contact
** office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file bicubic_interpolation.cpp
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */

#include <iostream>

#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <v4r/common/bicubic_interpolation.h>

// BICUBIC INTERPOLATION
// of a Matrix or at a given point of a new image.
//
// implemented in coefficient formulation: c(u)=g(u)*AX*beta
// u....... [x,y]
// AX..... [16x16] matrix
// beta...16 coefficients needed for Interpolation ->  4 surrounding points
// [f(0,0), f(1,0), f(0,1), f(1,1)],
//	12
// other points needed for: df/dx,df/dy, df/dxy           ---> y
//                                                       |
// b_1		b_5		b_9		b_13                         x
//
// b_2		b_6 	b_10	b_14		b_6= f(0,0)
//										b_7= f(1,0)
// b_3      b_7		b_11	b_15		b_10= f(0,1)
//										b_11= f(1,1)
// b_4      b_8		b_12	b_16

namespace v4r {

void BicubicInterpolation::interpolateMatrix(const cv::Mat& orig, cv::Mat& bicubic, int border_treatment) {
  double x, y;
  for (int u = 0; u < bicubic.rows; u++) {
    for (int v = 0; v < bicubic.cols; v++) {
      x = double(u) * double(orig.rows - 1) / double(bicubic.rows);
      y = double(v) * double(orig.cols - 1) / double(bicubic.cols);

      Eigen::MatrixXf AX(16, 16);

      AX << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 1, -2.5, 2, -0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.5, 1.5, -1.5, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          -0.5, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0.25, 0, -0.25, 0, 0, 0, 0, 0, -0.25, 0, 0.25, 0, 0, 0, 0,
          0, -0.5, 1.25, -1, 0.25, 0, 0, 0, 0, 0.5, -1.25, 1, -0.25, 0, 0, 0, 0, 0.25, -0.75, 0.75, -0.25, 0, 0, 0, 0,
          -0.25, 0.75, -0.75, 0.25, 0, 0, 0, 0, 0, 1, 0, 0, 0, -2.5, 0, 0, 0, 2, 0, 0, 0, -0.5, 0, 0, -0.5, 0, 0.5, 0,
          1.25, 0, -1.25, 0, -1, 0, 1, 0, 0.25, 0, -0.25, 0, 1, -2.5, 2, -0.5, -2.5, 6.25, -5, 1.25, 2, -5, 4, -1, -0.5,
          1.25, -1, 0.25, -0.5, 1.5, -1.5, 0.5, 1.25, -3.75, 3.75, -1.25, -1, 3, -3, 1, 0.25, -0.75, 0.75, -0.25, 0,
          -0.5, 0, 0, 0, 1.5, 0, 0, 0, -1.5, 0, 0, 0, 0.5, 0, 0, 0.25, 0, -0.25, 0, -0.75, 0, 0.75, 0, 0.75, 0, -0.75,
          0, -0.25, 0, 0.25, 0, -0.5, 1.25, -1, 0.25, 1.5, -3.75, 3, -0.75, -1.5, 3.75, -3, 0.75, 0.5, -1.25, 1, -0.25,
          0.25, -0.75, 0.75, -0.25, -0.75, 2.25, -2.25, 0.75, 0.75, -2.25, 2.25, -0.75, -0.25, 0.75, -0.75, 0.25;

      Eigen::VectorXf betas(16);
      int rowi, coli;
      rowi = int(x) + 2;
      coli = int(y) + 2;

      cv::Mat orig_border = orig;
      cv::copyMakeBorder(orig, orig_border, 2, 2, 2, 2, border_treatment);

      double beta_1 = orig_border.at<double>(rowi - 1, coli - 1);
      double beta_2 = orig_border.at<double>(rowi, coli - 1);
      double beta_3 = orig_border.at<double>(rowi + 1, coli - 1);
      double beta_4 = orig_border.at<double>(rowi + 2, coli - 1);

      double beta_5 = orig_border.at<double>(rowi - 1, coli);
      double beta_6 = orig_border.at<double>(rowi, coli);
      double beta_7 = orig_border.at<double>(rowi + 1, coli);
      double beta_8 = orig_border.at<double>(rowi + 2, coli);

      double beta_9 = orig_border.at<double>(rowi - 1, coli + 1);
      double beta_10 = orig_border.at<double>(rowi, coli + 1);
      double beta_11 = orig_border.at<double>(rowi + 1, coli + 1);
      double beta_12 = orig_border.at<double>(rowi + 2, coli + 1);

      double beta_13 = orig_border.at<double>(rowi - 1, coli + 2);
      double beta_14 = orig_border.at<double>(rowi, coli + 2);
      double beta_15 = orig_border.at<double>(rowi + 1, coli + 2);
      double beta_16 = orig_border.at<double>(rowi + 2, coli + 2);

      betas << beta_1, beta_2, beta_3, beta_4, beta_5, beta_6, beta_7, beta_8, beta_9, beta_10, beta_11, beta_12,
          beta_13, beta_14, beta_15, beta_16;

      Eigen::VectorXf alphas(16);

      alphas = AX * betas;

      x = x - int(x);
      y = y - int(y);

      double x_2 = x * x;
      double x_3 = x * x * x;
      double y_2 = y * y;
      double y_3 = y * y * y;

      Eigen::RowVectorXf g(16);

      g << 1, x, x_2, x_3, y, y * x, y * x_2, y * x_3, y_2, y_2 * x, y_2 * x_2, y_2 * x_3, y_3, y_3 * x, y_3 * x_2,
          y_3 * x_3;

      double interpol = g * alphas;

      bicubic.at<double>(u, v) = interpol;
    }
  }

  return;
}

double BicubicInterpolation::interpolatePoint(double ux, double uy, const double* lattice, cv::Size src_dim,
                                              cv::Size dst_dim, std::vector<int>& indices, Eigen::VectorXd& beta) {
  double row, col;

  row = double(uy) * double(src_dim.height - 1) / double(dst_dim.height);
  col = double(ux) * double(src_dim.width - 1) / double(dst_dim.width);

  int rowi, coli;
  rowi = int(row) + 2;
  coli = int(col) + 2;

  cv::Mat index_mask(src_dim.height, src_dim.width, CV_16U);

  for (int row_index = 0; row_index < index_mask.rows; row_index++) {
    for (int col_index = 0; col_index < index_mask.cols; col_index++) {
      index_mask.at<u_int16_t>(row_index, col_index) = (col_index) + (row_index) * (index_mask.cols);
    }
  }

  cv::Mat index_mask_border;
  cv::copyMakeBorder(index_mask, index_mask_border, 2, 2, 2, 2, cv::BORDER_REFLECT_101);

  unsigned m = 0;
  for (int k = -1; k <= 2; k++) {
    for (int l = -1; l <= 2; l++) {
      int rowl = rowi + l;
      int colk = coli + k;

      indices[m] = int(index_mask_border.at<u_int16_t>(rowl, colk));
      m++;
    }
  }

  for (size_t i = 0; i < 16; i++) {
    beta[i] = lattice[indices[i]];
  }

  double r = row - int(row);
  double c = col - int(col);

  double t2 = 0.2500000000e0 * r * c;
  double t3 = r * r;
  double t4 = t3 * c;
  double t5 = 0.5000000000e0 * t4;
  double t6 = t3 * r;
  double t7 = t6 * c;
  double t8 = 0.2500000000e0 * t7;
  double t9 = pow(c, 0.2e1);
  double t10 = r * t9;
  double t11 = 0.5000000000e0 * t10;
  double t12 = t3 * t9;
  double t13 = 0.1e1 * t12;
  double t14 = t6 * t9;
  double t15 = 0.5000000000e0 * t14;
  double t16 = t9 * c;
  double t17 = r * t16;
  double t18 = 0.2500000000e0 * t17;
  double t19 = t3 * t16;
  double t20 = 0.5000000000e0 * t19;
  double t21 = t6 * t16;
  double t22 = 0.2500000000e0 * t21;
  double t26 = 0.5000000000e0 * c;
  double t27 = 0.1250000000e1 * t4;
  double t28 = 0.7500000000e0 * t7;
  double t30 = 0.2500000000e1 * t12;
  double t31 = 0.1500000000e1 * t14;
  double t32 = 0.5000000000e0 * t16;
  double t33 = 0.1250000000e1 * t19;
  double t34 = 0.7500000000e0 * t21;
  double t38 = 0.1e1 * t4;
  double t39 = 0.2e1 * t12;
  double t40 = 0.1e1 * t19;
  double t44 = 0.2500000000e0 * t4;
  double t45 = 0.5000000000e0 * t12;
  double t46 = 0.2500000000e0 * t19;
  double t50 = 0.5000000000e0 * r;
  double t52 = 0.5000000000e0 * t6;
  double t53 = 0.1250000000e1 * t10;
  double t54 = 0.1250000000e1 * t14;
  double t55 = 0.7500000000e0 * t17;
  double t56 = 0.1500000000e1 * t19;
  double t61 = 0.1500000000e1 * t6;
  double t64 = 0.3750000000e1 * t14;
  double t65 = 0.1500000000e1 * t16;
  double t66 = 0.3750000000e1 * t19;
  double t67 = 0.2250000000e1 * t21;
  double t72 = 0.5e1 * t12;
  double t73 = 0.3e1 * t19;
  double t78 = 0.1250000000e1 * t12;
  double t79 = 0.7500000000e0 * t19;
  double t83 = 0.1e1 * t10;
  double t84 = 0.1e1 * t14;
  double t89 = 0.3e1 * t14;
  double t100 = 0.2500000000e0 * t10;
  double t101 = 0.2500000000e0 * t14;
  double t106 = 0.7500000000e0 * t14;
  double interpol =
      (t2 - t5 + t8 - t11 + t13 - t15 + t18 - t20 + t22) * beta[0] +
      (-t26 + t27 - t28 + 0.1e1 * t9 - t30 + t31 - t32 + t33 - t34) * beta[1] +
      (-t2 - t38 + t28 + t11 + t39 - t31 - t18 - t40 + t34) * beta[2] + (t44 - t8 - t45 + t15 + t46 - t22) * beta[3] +
      (-t50 + 0.1e1 * t3 - t52 + t53 - t30 + t54 - t55 + t56 - t34) * beta[4] +
      (0.1e1 - 0.2500000000e1 * t3 + t61 - 0.2500000000e1 * t9 + 0.6250000000e1 * t12 - t64 + t65 - t66 + t67) *
          beta[5] +
      (t50 + 0.2e1 * t3 - t61 - t53 - t72 + t64 + t55 + t73 - t67) * beta[6] +
      (-0.5000000000e0 * t3 + t52 + t78 - t54 - t79 + t34) * beta[7] +
      (-t2 + t5 - t8 - t83 + t39 - t84 + t55 - t56 + t34) * beta[8] +
      (t26 - t27 + t28 + 0.2e1 * t9 - t72 + t89 - t65 + t66 - t67) * beta[9] +
      (t2 + t38 - t28 + t83 + 0.4e1 * t12 - t89 - t55 - t73 + t67) * beta[10] +
      (-t44 + t8 - t13 + t84 + t79 - t34) * beta[11] + (t100 - t45 + t101 - t18 + t20 - t22) * beta[12] +
      (-0.5000000000e0 * t9 + t78 - t106 + t32 - t33 + t34) * beta[13] +
      (-t100 - t13 + t106 + t18 + t40 - t34) * beta[14] + (0.2500000000e0 * t12 - t101 - t46 + t22) * beta[15];

  return interpol;
}

void BicubicInterpolation::getGamma(const double ux, const double uy, const cv::Size src_dim, const cv::Size dst_dim,
                                    Eigen::RowVectorXd& gamma) {
  double r, c;

  r = double(uy) * double(src_dim.height - 1) / double(dst_dim.height);
  c = double(ux) * double(src_dim.width - 1) / double(dst_dim.width);

  r = r - int(r);
  c = c - int(c);

  gamma[0] = 0.2500000000e0 * r * c - 0.5000000000e0 * r * r * c + 0.2500000000e0 * pow(r, 0.3e1) * c -
             0.5000000000e0 * r * pow(c, 0.2e1) + 0.1e1 * r * r * pow(c, 0.2e1) -
             0.5000000000e0 * pow(r, 0.3e1) * pow(c, 0.2e1) + 0.2500000000e0 * r * pow(c, 0.3e1) -
             0.5000000000e0 * r * r * pow(c, 0.3e1) + 0.2500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[1] = -0.5000000000e0 * c + 0.1250000000e1 * r * r * c - 0.7500000000e0 * pow(r, 0.3e1) * c +
             0.1e1 * pow(c, 0.2e1) - 0.2500000000e1 * r * r * pow(c, 0.2e1) +
             0.1500000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1) - 0.5000000000e0 * pow(c, 0.3e1) +
             0.1250000000e1 * r * r * pow(c, 0.3e1) - 0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[2] = -0.2500000000e0 * r * c - 0.1e1 * r * r * c + 0.7500000000e0 * pow(r, 0.3e1) * c +
             0.5000000000e0 * r * pow(c, 0.2e1) + 0.2e1 * r * r * pow(c, 0.2e1) -
             0.1500000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1) - 0.2500000000e0 * r * pow(c, 0.3e1) -
             0.1e1 * r * r * pow(c, 0.3e1) + 0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[3] = 0.2500000000e0 * r * r * c - 0.2500000000e0 * pow(r, 0.3e1) * c - 0.5000000000e0 * r * r * pow(c, 0.2e1) +
             0.5000000000e0 * pow(r, 0.3e1) * pow(c, 0.2e1) + 0.2500000000e0 * r * r * pow(c, 0.3e1) -
             0.2500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[4] = -0.5000000000e0 * r + 0.1e1 * r * r - 0.5000000000e0 * pow(r, 0.3e1) + 0.1250000000e1 * r * pow(c, 0.2e1) -
             0.2500000000e1 * r * r * pow(c, 0.2e1) + 0.1250000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1) -
             0.7500000000e0 * r * pow(c, 0.3e1) + 0.1500000000e1 * r * r * pow(c, 0.3e1) -
             0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[5] = 0.1e1 - 0.2500000000e1 * r * r + 0.1500000000e1 * pow(r, 0.3e1) - 0.2500000000e1 * pow(c, 0.2e1) +
             0.6250000000e1 * r * r * pow(c, 0.2e1) - 0.3750000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1) +
             0.1500000000e1 * pow(c, 0.3e1) - 0.3750000000e1 * r * r * pow(c, 0.3e1) +
             0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[6] = 0.5000000000e0 * r + 0.2e1 * r * r - 0.1500000000e1 * pow(r, 0.3e1) - 0.1250000000e1 * r * pow(c, 0.2e1) -
             0.5e1 * r * r * pow(c, 0.2e1) + 0.3750000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1) +
             0.7500000000e0 * r * pow(c, 0.3e1) + 0.3e1 * r * r * pow(c, 0.3e1) -
             0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[7] = -0.5000000000e0 * r * r + 0.5000000000e0 * pow(r, 0.3e1) + 0.1250000000e1 * r * r * pow(c, 0.2e1) -
             0.1250000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1) - 0.7500000000e0 * r * r * pow(c, 0.3e1) +
             0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[8] = -0.2500000000e0 * r * c + 0.5000000000e0 * r * r * c - 0.2500000000e0 * pow(r, 0.3e1) * c -
             0.1e1 * r * pow(c, 0.2e1) + 0.2e1 * r * r * pow(c, 0.2e1) - 0.1e1 * pow(r, 0.3e1) * pow(c, 0.2e1) +
             0.7500000000e0 * r * pow(c, 0.3e1) - 0.1500000000e1 * r * r * pow(c, 0.3e1) +
             0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[9] = 0.5000000000e0 * c - 0.1250000000e1 * r * r * c + 0.7500000000e0 * pow(r, 0.3e1) * c +
             0.2e1 * pow(c, 0.2e1) - 0.5e1 * r * r * pow(c, 0.2e1) + 0.3e1 * pow(r, 0.3e1) * pow(c, 0.2e1) -
             0.1500000000e1 * pow(c, 0.3e1) + 0.3750000000e1 * r * r * pow(c, 0.3e1) -
             0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[10] = 0.2500000000e0 * r * c + 0.1e1 * r * r * c - 0.7500000000e0 * pow(r, 0.3e1) * c +
              0.1e1 * r * pow(c, 0.2e1) + 0.4e1 * r * r * pow(c, 0.2e1) - 0.3e1 * pow(r, 0.3e1) * pow(c, 0.2e1) -
              0.7500000000e0 * r * pow(c, 0.3e1) - 0.3e1 * r * r * pow(c, 0.3e1) +
              0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[11] = -0.2500000000e0 * r * r * c + 0.2500000000e0 * pow(r, 0.3e1) * c - 0.1e1 * r * r * pow(c, 0.2e1) +
              0.1e1 * pow(r, 0.3e1) * pow(c, 0.2e1) + 0.7500000000e0 * r * r * pow(c, 0.3e1) -
              0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[12] = 0.2500000000e0 * r * pow(c, 0.2e1) - 0.5000000000e0 * r * r * pow(c, 0.2e1) +
              0.2500000000e0 * pow(r, 0.3e1) * pow(c, 0.2e1) - 0.2500000000e0 * r * pow(c, 0.3e1) +
              0.5000000000e0 * r * r * pow(c, 0.3e1) - 0.2500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[13] = -0.5000000000e0 * pow(c, 0.2e1) + 0.1250000000e1 * r * r * pow(c, 0.2e1) -
              0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.2e1) + 0.5000000000e0 * pow(c, 0.3e1) -
              0.1250000000e1 * r * r * pow(c, 0.3e1) + 0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[14] = -0.2500000000e0 * r * pow(c, 0.2e1) - 0.1e1 * r * r * pow(c, 0.2e1) +
              0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.2e1) + 0.2500000000e0 * r * pow(c, 0.3e1) +
              0.1e1 * r * r * pow(c, 0.3e1) - 0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
  gamma[15] = 0.2500000000e0 * r * r * pow(c, 0.2e1) - 0.2500000000e0 * pow(r, 0.3e1) * pow(c, 0.2e1) -
              0.2500000000e0 * r * r * pow(c, 0.3e1) + 0.2500000000e0 * pow(r, 0.3e1) * pow(c, 0.3e1);
}
void BicubicInterpolation::calculateGradient(const cv::Mat& src, const cv::Size dst_dim, const cv::Point dst_point,
                                             Eigen::RowVector2d& dcdu, const int border_treatment) {
  double x, y;

  x = double(dst_point.y) * double(src.rows - 1) / double(dst_dim.height);
  y = double(dst_point.x) * double(src.cols - 1) / double(dst_dim.width);

  Eigen::VectorXd betas(16);
  int rowi, coli;
  rowi = int(x) + 2;
  coli = int(y) + 2;

  cv::Mat orig_border;
  cv::copyMakeBorder(src, orig_border, 2, 2, 2, 2, border_treatment);

  double beta_1 = orig_border.at<double>(rowi - 1, coli - 1);
  double beta_2 = orig_border.at<double>(rowi, coli - 1);
  double beta_3 = orig_border.at<double>(rowi + 1, coli - 1);
  double beta_4 = orig_border.at<double>(rowi + 2, coli - 1);

  double beta_5 = orig_border.at<double>(rowi - 1, coli);
  double beta_6 = orig_border.at<double>(rowi, coli);
  double beta_7 = orig_border.at<double>(rowi + 1, coli);
  double beta_8 = orig_border.at<double>(rowi + 2, coli);

  double beta_9 = orig_border.at<double>(rowi - 1, coli + 1);
  double beta_10 = orig_border.at<double>(rowi, coli + 1);
  double beta_11 = orig_border.at<double>(rowi + 1, coli + 1);
  double beta_12 = orig_border.at<double>(rowi + 2, coli + 1);

  double beta_13 = orig_border.at<double>(rowi - 1, coli + 2);
  double beta_14 = orig_border.at<double>(rowi, coli + 2);
  double beta_15 = orig_border.at<double>(rowi + 1, coli + 2);
  double beta_16 = orig_border.at<double>(rowi + 2, coli + 2);

  betas << beta_1, beta_2, beta_3, beta_4, beta_5, beta_6, beta_7, beta_8, beta_9, beta_10, beta_11, beta_12, beta_13,
      beta_14, beta_15, beta_16;

  calculateGradient(x, y, src.size(), dst_dim, betas, dcdu);

  return;
}
void BicubicInterpolation::calculateGradient(const double ux, const double uy, const cv::Size lattice_dim,
                                             const cv::Size dst_dim, const Eigen::VectorXd& beta,
                                             Eigen::RowVector2d& dcdu) {
  double fact[2];
  fact[0] = (double(lattice_dim.width - 1)) / double(dst_dim.width);
  fact[1] = (double(lattice_dim.height - 1)) / double(dst_dim.height);

  double row = double(uy) * fact[1];
  double col = double(ux) * fact[0];

  // Alphas are interpolation coefficients
  double r = row - int(row);
  double c = col - int(col);

  // Compute the derivative of the bicubic interpolation.
  // c(u)=g(u)^T*A*X*beta;
  // c(u)=gamma(u)*beta;
  // dc/du=beta^T*dgamma/du

  dcdu[0] = (0.2500000000e0 * r - 0.5000000000e0 * r * r + 0.2500000000e0 * pow(r, 0.3e1) - 0.1000000000e1 * r * c +
             0.2e1 * r * r * c - 0.1000000000e1 * pow(r, 0.3e1) * c + 0.7500000000e0 * r * pow(c, 0.2e1) -
             0.1500000000e1 * r * r * pow(c, 0.2e1) + 0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[0] +
            (-0.5000000000e0 + 0.1250000000e1 * r * r - 0.7500000000e0 * pow(r, 0.3e1) + 0.2e1 * c -
             0.5000000000e1 * r * r * c + 0.3000000000e1 * pow(r, 0.3e1) * c - 0.1500000000e1 * pow(c, 0.2e1) +
             0.3750000000e1 * r * r * pow(c, 0.2e1) - 0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[1] +
            (-0.2500000000e0 * r - 0.1e1 * r * r + 0.7500000000e0 * pow(r, 0.3e1) + 0.1000000000e1 * r * c +
             0.4e1 * r * r * c - 0.3000000000e1 * pow(r, 0.3e1) * c - 0.7500000000e0 * r * pow(c, 0.2e1) -
             0.3e1 * r * r * pow(c, 0.2e1) + 0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[2] +
            (0.2500000000e0 * r * r - 0.2500000000e0 * pow(r, 0.3e1) - 0.1000000000e1 * r * r * c +
             0.1000000000e1 * pow(r, 0.3e1) * c + 0.7500000000e0 * r * r * pow(c, 0.2e1) -
             0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[3] +
            (0.2500000000e1 * r * c - 0.5000000000e1 * r * r * c + 0.2500000000e1 * pow(r, 0.3e1) * c -
             0.2250000000e1 * r * pow(c, 0.2e1) + 0.4500000000e1 * r * r * pow(c, 0.2e1) -
             0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[4] +
            (-0.5000000000e1 * c + 0.1250000000e2 * r * r * c - 0.7500000000e1 * pow(r, 0.3e1) * c +
             0.4500000000e1 * pow(c, 0.2e1) - 0.1125000000e2 * r * r * pow(c, 0.2e1) +
             0.6750000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[5] +
            (-0.2500000000e1 * r * c - 0.10e2 * r * r * c + 0.7500000000e1 * pow(r, 0.3e1) * c +
             0.2250000000e1 * r * pow(c, 0.2e1) + 0.9e1 * r * r * pow(c, 0.2e1) -
             0.6750000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[6] +
            (0.2500000000e1 * r * r * c - 0.2500000000e1 * pow(r, 0.3e1) * c - 0.2250000000e1 * r * r * pow(c, 0.2e1) +
             0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[7] +
            (-0.2500000000e0 * r + 0.5000000000e0 * r * r - 0.2500000000e0 * pow(r, 0.3e1) - 0.2e1 * r * c +
             0.4e1 * r * r * c - 0.2e1 * pow(r, 0.3e1) * c + 0.2250000000e1 * r * pow(c, 0.2e1) -
             0.4500000000e1 * r * r * pow(c, 0.2e1) + 0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[8] +
            (0.5000000000e0 - 0.1250000000e1 * r * r + 0.7500000000e0 * pow(r, 0.3e1) + 0.4e1 * c - 0.10e2 * r * r * c +
             0.6e1 * pow(r, 0.3e1) * c - 0.4500000000e1 * pow(c, 0.2e1) + 0.1125000000e2 * r * r * pow(c, 0.2e1) -
             0.6750000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[9] +
            (0.2500000000e0 * r + 0.1e1 * r * r - 0.7500000000e0 * pow(r, 0.3e1) + 0.2e1 * r * c + 0.8e1 * r * r * c -
             0.6e1 * pow(r, 0.3e1) * c - 0.2250000000e1 * r * pow(c, 0.2e1) - 0.9e1 * r * r * pow(c, 0.2e1) +
             0.6750000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[10] +
            (-0.2500000000e0 * r * r + 0.2500000000e0 * pow(r, 0.3e1) - 0.2e1 * r * r * c + 0.2e1 * pow(r, 0.3e1) * c +
             0.2250000000e1 * r * r * pow(c, 0.2e1) - 0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[11] +
            (0.5000000000e0 * r * c - 0.1000000000e1 * r * r * c + 0.5000000000e0 * pow(r, 0.3e1) * c -
             0.7500000000e0 * r * pow(c, 0.2e1) + 0.1500000000e1 * r * r * pow(c, 0.2e1) -
             0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[12] +
            (-0.1000000000e1 * c + 0.2500000000e1 * r * r * c - 0.1500000000e1 * pow(r, 0.3e1) * c +
             0.1500000000e1 * pow(c, 0.2e1) - 0.3750000000e1 * r * r * pow(c, 0.2e1) +
             0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[13] +
            (-0.5000000000e0 * r * c - 0.2e1 * r * r * c + 0.1500000000e1 * pow(r, 0.3e1) * c +
             0.7500000000e0 * r * pow(c, 0.2e1) + 0.3e1 * r * r * pow(c, 0.2e1) -
             0.2250000000e1 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[14] +
            (0.5000000000e0 * r * r * c - 0.5000000000e0 * pow(r, 0.3e1) * c - 0.7500000000e0 * r * r * pow(c, 0.2e1) +
             0.7500000000e0 * pow(r, 0.3e1) * pow(c, 0.2e1)) *
                beta[15];

  dcdu[1] =
      (0.2500000000e0 * c - 0.1000000000e1 * r * c + 0.7500000000e0 * r * r * c - 0.5000000000e0 * pow(c, 0.2e1) +
       0.2e1 * r * pow(c, 0.2e1) - 0.1500000000e1 * r * r * pow(c, 0.2e1) + 0.2500000000e0 * pow(c, 0.3e1) -
       0.1000000000e1 * r * pow(c, 0.3e1) + 0.7500000000e0 * r * r * pow(c, 0.3e1)) *
          beta[0] +
      (0.2500000000e1 * r * c - 0.2250000000e1 * r * r * c - 0.5000000000e1 * r * pow(c, 0.2e1) +
       0.4500000000e1 * r * r * pow(c, 0.2e1) + 0.2500000000e1 * r * pow(c, 0.3e1) -
       0.2250000000e1 * r * r * pow(c, 0.3e1)) *
          beta[1] +
      (-0.2500000000e0 * c - 0.2e1 * r * c + 0.2250000000e1 * r * r * c + 0.5000000000e0 * pow(c, 0.2e1) +
       0.4e1 * r * pow(c, 0.2e1) - 0.4500000000e1 * r * r * pow(c, 0.2e1) - 0.2500000000e0 * pow(c, 0.3e1) -
       0.2e1 * r * pow(c, 0.3e1) + 0.2250000000e1 * r * r * pow(c, 0.3e1)) *
          beta[2] +
      (0.5000000000e0 * r * c - 0.7500000000e0 * r * r * c - 0.1000000000e1 * r * pow(c, 0.2e1) +
       0.1500000000e1 * r * r * pow(c, 0.2e1) + 0.5000000000e0 * r * pow(c, 0.3e1) -
       0.7500000000e0 * r * r * pow(c, 0.3e1)) *
          beta[3] +
      (-0.5000000000e0 + 0.2e1 * r - 0.1500000000e1 * r * r + 0.1250000000e1 * pow(c, 0.2e1) -
       0.5000000000e1 * r * pow(c, 0.2e1) + 0.3750000000e1 * r * r * pow(c, 0.2e1) - 0.7500000000e0 * pow(c, 0.3e1) +
       0.3000000000e1 * r * pow(c, 0.3e1) - 0.2250000000e1 * r * r * pow(c, 0.3e1)) *
          beta[4] +
      (-0.5000000000e1 * r + 0.4500000000e1 * r * r + 0.1250000000e2 * r * pow(c, 0.2e1) -
       0.1125000000e2 * r * r * pow(c, 0.2e1) - 0.7500000000e1 * r * pow(c, 0.3e1) +
       0.6750000000e1 * r * r * pow(c, 0.3e1)) *
          beta[5] +
      (0.5000000000e0 + 0.4e1 * r - 0.4500000000e1 * r * r - 0.1250000000e1 * pow(c, 0.2e1) -
       0.10e2 * r * pow(c, 0.2e1) + 0.1125000000e2 * r * r * pow(c, 0.2e1) + 0.7500000000e0 * pow(c, 0.3e1) +
       0.6e1 * r * pow(c, 0.3e1) - 0.6750000000e1 * r * r * pow(c, 0.3e1)) *
          beta[6] +
      (-0.1000000000e1 * r + 0.1500000000e1 * r * r + 0.2500000000e1 * r * pow(c, 0.2e1) -
       0.3750000000e1 * r * r * pow(c, 0.2e1) - 0.1500000000e1 * r * pow(c, 0.3e1) +
       0.2250000000e1 * r * r * pow(c, 0.3e1)) *
          beta[7] +
      (-0.2500000000e0 * c + 0.1000000000e1 * r * c - 0.7500000000e0 * r * r * c - 0.1e1 * pow(c, 0.2e1) +
       0.4e1 * r * pow(c, 0.2e1) - 0.3e1 * r * r * pow(c, 0.2e1) + 0.7500000000e0 * pow(c, 0.3e1) -
       0.3000000000e1 * r * pow(c, 0.3e1) + 0.2250000000e1 * r * r * pow(c, 0.3e1)) *
          beta[8] +
      (-0.2500000000e1 * r * c + 0.2250000000e1 * r * r * c - 0.10e2 * r * pow(c, 0.2e1) +
       0.9e1 * r * r * pow(c, 0.2e1) + 0.7500000000e1 * r * pow(c, 0.3e1) - 0.6750000000e1 * r * r * pow(c, 0.3e1)) *
          beta[9] +
      (0.2500000000e0 * c + 0.2e1 * r * c - 0.2250000000e1 * r * r * c + 0.1e1 * pow(c, 0.2e1) +
       0.8e1 * r * pow(c, 0.2e1) - 0.9e1 * r * r * pow(c, 0.2e1) - 0.7500000000e0 * pow(c, 0.3e1) -
       0.6e1 * r * pow(c, 0.3e1) + 0.6750000000e1 * r * r * pow(c, 0.3e1)) *
          beta[10] +
      (-0.5000000000e0 * r * c + 0.7500000000e0 * r * r * c - 0.2e1 * r * pow(c, 0.2e1) +
       0.3e1 * r * r * pow(c, 0.2e1) + 0.1500000000e1 * r * pow(c, 0.3e1) - 0.2250000000e1 * r * r * pow(c, 0.3e1)) *
          beta[11] +
      (0.2500000000e0 * pow(c, 0.2e1) - 0.1000000000e1 * r * pow(c, 0.2e1) + 0.7500000000e0 * r * r * pow(c, 0.2e1) -
       0.2500000000e0 * pow(c, 0.3e1) + 0.1000000000e1 * r * pow(c, 0.3e1) - 0.7500000000e0 * r * r * pow(c, 0.3e1)) *
          beta[12] +
      (0.2500000000e1 * r * pow(c, 0.2e1) - 0.2250000000e1 * r * r * pow(c, 0.2e1) -
       0.2500000000e1 * r * pow(c, 0.3e1) + 0.2250000000e1 * r * r * pow(c, 0.3e1)) *
          beta[13] +
      (-0.2500000000e0 * pow(c, 0.2e1) - 0.2e1 * r * pow(c, 0.2e1) + 0.2250000000e1 * r * r * pow(c, 0.2e1) +
       0.2500000000e0 * pow(c, 0.3e1) + 0.2e1 * r * pow(c, 0.3e1) - 0.2250000000e1 * r * r * pow(c, 0.3e1)) *
          beta[14] +
      (0.5000000000e0 * r * pow(c, 0.2e1) - 0.7500000000e0 * r * r * pow(c, 0.2e1) -
       0.5000000000e0 * r * pow(c, 0.3e1) + 0.7500000000e0 * r * r * pow(c, 0.3e1)) *
          beta[15];

  dcdu[0] *= fact[0];
  dcdu[1] *= fact[1];
  return;
}
}  // namespace v4r
