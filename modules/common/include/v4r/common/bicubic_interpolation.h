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
 * @file bicubic_interpolation.h
 * @author Georg Halmetschlager-Funek (gh@acin.tuwien.ac.at)
 * @date 2018
 * @brief
 *
 */

#ifndef BICUBIC_INTERPOLATION_HPP
#define BICUBIC_INTERPOLATION_HPP

#include <v4r/core/macros.h>

#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>

namespace v4r {
/** @brief BicubicInterpolation.
 *         Performs a Bicubic Interpolation and provides its (partial) derivatives respective to the control points and
 * the image gradient.
 *
 */
namespace BicubicInterpolation {

/**
 * @brief interpolates a matrix using a bicubic interpolation
 * @param[in] original image
 * @param[in] result
 * @param[in] [optional] border treatment (OpenCV) Default: cv::BORDER_REFLECT
 *
 */
V4R_EXPORTS void interpolateMatrix(const cv::Mat& orig, cv::Mat& bicubic,
                                   const int border_treatment = cv::BORDER_REFLECT);

/**
 * @brief Analytic gradient of a bicubic interpolation (dI/dx, dI/dy) for a given point
 * @param[in] original image
 * @param[in] new size
 * @param[in] location
 * @param[in] result [dx,dy]
 * @param[in] [optional] border treatment (OpenCV) Default: cv::BORDER_REFLECT
 */
V4R_EXPORTS void calculateGradient(const cv::Mat& src, const cv::Size dst_dim, const cv::Point dst_point,
                                   Eigen::RowVector2d& dcdu, const int border_treatment = cv::BORDER_REFLECT);

/**
 * @brief Analytic gradient of a bicubic interpolation (dI/dx, dI/dy) for a given point
 * @param[in] x location
 * @param[in] y location
 * @param[in] src dim
 * @param[in] dst dim
 * @param[in] coefficients
 * @param[in] result [dx,dy]
 */
V4R_EXPORTS void calculateGradient(const double ux, const double uy, const cv::Size lattice_dim, const cv::Size dst_dim,
                                   const Eigen::VectorXd& beta, Eigen::RowVector2d& dcdu);

/**
 * @brief Performs a bicubic interpolation for a given point based on a lower dimensional lattice
 * @param[in] x location
 * @param[in] y location
 * @param[in] coefficients/lattice
 * @param[in] src dim
 * @param[in] dst dim
 * @param[in] indices of 16 points used for interpolation
 * @param[in] vector holding the 16 coefficients
 */
V4R_EXPORTS double interpolatePoint(const double ux, const double uy, const double* lattice, const cv::Size src_dim,
                                    const cv::Size dst_dim, std::vector<int>& indices, Eigen::VectorXd& beta);

// Gamma contains the partial derivative of the interpolation lattice respective to the control points beta
/**
 * @brief Get the partial derivative for the interpolation lattice respective to the control points beta
 * @param[in] x location
 * @param[in] y location
 * @param[in] src dim
 * @param[in] dst dim
 * @param[in] gamma: result (depends only on the location)
 */
V4R_EXPORTS void getGamma(const double ux, const double uy, const cv::Size src_dim, const cv::Size dst_dim,
                          Eigen::RowVectorXd& gamma);

}  // namespace BicubicInterpolation
}  // namespace v4r

#endif  // BICUBIC_INTERPOLATION_HPP
