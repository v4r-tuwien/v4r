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
 * @file ViewRenderer.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Simon Schreiberhuber (schreiberhuber@acin.tuwien.ac.at)
 * @date October 2017
 * @brief Renders Views from a textured mesh model and saves it into a colored point cloud.
 *
 */

#pragma once

#include <v4r/common/intrinsics.h>
#include <v4r/config.h>
#include <v4r/core/macros.h>
#include <v4r/io/filesystem.h>

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;

namespace v4r {

namespace apps {

struct V4R_EXPORTS ViewRendererParameter {
  bool upperHemisphere;  //< if true, samples views from the upper hemisphere only (z>0)
  bool autoscale;  //< if true, scales the coordinate system such that the object model is enclosed by the unit sphere
  size_t subdivisions;
  float radius;
  bool overwrite;

  ViewRendererParameter() : upperHemisphere(false), autoscale(false), subdivisions(0), radius(3.f), overwrite(false) {}

  /**
   * @brief init parameters
   * @param command_line_arguments (according to Boost program options library)
   * @param section_name section name of program options
   * @return unused parameters (given parameters that were not used in this initialization call)
   */
  void init(boost::program_options::options_description &desc, const std::string &section_name = "rendering") {
    desc.add_options()((section_name + ".subdivisions").c_str(),
                       po::value<size_t>(&subdivisions)->default_value(subdivisions),
                       "defines the number of subdivsions used for rendering");
    desc.add_options()((section_name + ".autoscale").c_str(), po::bool_switch(&autoscale),
                       "scales the model into the unit sphere and translates it to the origin");
    desc.add_options()((section_name + ".northHemisphere").c_str(), po::bool_switch(&upperHemisphere),
                       "only renders the objects from views of the upper hemisphere");
    desc.add_options()((section_name + ".radius").c_str(),
                       po::value<float>(&radius)->default_value(radius, boost::str(boost::format("%.2e") % radius)),
                       "defines the radius used for rendering");
    desc.add_options()((section_name + ".overwrite").c_str(), po::bool_switch(&overwrite),
                       "overwrites existing files in output path");
  }
};

/**
 * @brief class to render point cloud from a textured mesh
 */
class V4R_EXPORTS ViewRenderer {
 private:
  const Intrinsics cam_;
  ViewRendererParameter param_;

 public:
  ViewRenderer(const Intrinsics &cam, const ViewRendererParameter &p) : cam_(cam), param_(p) {}

  void render(const bf::path &input_path, const bf::path &out_path = "rendered_views");
};
}  // namespace apps
}  // namespace v4r
