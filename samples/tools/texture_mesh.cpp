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

#include <iostream>

#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <v4r/io/eigen.h>
#include <v4r/io/pcd_grabber.h>
#include <v4r/surface/mvs_texturing.h>

#include "program_options.h"

struct ProgramOptions : public ProgramOptionsBase {
  std::string model_path = "";
  std::string views_path = "";
  std::string output_path = "textured_mesh";
  v4r::surface::MVSTexturing::Parameters mvs_params;
  bool visualize = false;

  void printDescription() override {
    std::cout << "Texture a mesh using a set of camera images with known poses." << std::endl;
    std::cout << std::endl;
    std::cout << "Output path for the model should have no extension. Three files will be created:" << std::endl;
    std::cout << "  <output-path>.obj - with mesh and texture coordinates " << std::endl;
    std::cout << "  <output-path>.mtl - with materials" << std::endl;
    std::cout << "  <output-path>_material.png - with texture atlas" << std::endl;
  }

  void addGeneral(po::options_description& desc) override {
    desc.add_options()("visualize,v", po::bool_switch(&visualize), "Visualize textured model");
  }

  void addOther(std::vector<po::options_description>& descs) override {
    descs.push_back(po::options_description("MVS Texturing"));
    mvs_params.init(descs.back());
  }

  void addPositional(po::options_description& desc, po::positional_options_description& positional) override {
    desc.add_options()("model-path", po::value<std::string>(&model_path), "Path to model PLY file");
    desc.add_options()("views-path", po::value<std::string>(&views_path), "Path to views directory");
    desc.add_options()("output-path", po::value<std::string>(&output_path),
                       "Path to output textured model (without extension)");
    positional.add("model-path", 1);
    positional.add("views-path", 1);
    positional.add("output-path", 1);
  }
};

int main(int argc, const char** argv) {
  ProgramOptions options;
  if (!options.parse(argc, argv))
    return 1;

  pcl::PolygonMesh mesh;
  std::vector<cv::Mat> images;
  v4r::surface::MVSTexturing::PoseVector poses;

  LOG(INFO) << "Loading mesh from file: " << options.model_path;
  pcl::io::loadPolygonFile(options.model_path, mesh);

  LOG(INFO) << "Loading camera images: " << options.views_path;
  v4r::io::PCDGrabber grabber(options.views_path);
  while (grabber.hasMoreFrames()) {
    cv::Mat color, depth;
    auto timestamp = grabber.grabFrame(color, depth);
    LOG(INFO) << "  timestamp: " << timestamp << " size: " << color.size().width << " x " << color.size().height;
    boost::filesystem::path p = options.views_path;
    p /= boost::str(boost::format("pose_%08d.txt") % timestamp);
    Eigen::Matrix4f pose = v4r::io::readMatrixFromFile(p.string()).inverse();
    images.push_back(color);
    poses.push_back(pose);
  }

  v4r::surface::MVSTexturing tex;
  tex.mapTextureToMesh(mesh, grabber.getCameraIntrinsics(), images, poses, options.output_path);

  if (options.visualize) {
    // We have to use both loaders and merge the results to work around this PCL issue:
    // https://github.com/PointCloudLibrary/pcl/issues/2252
    pcl::TextureMesh textured_mesh;
    pcl::io::loadPolygonFileOBJ(options.output_path + ".obj", textured_mesh);
    pcl::TextureMesh mesh2;
    pcl::io::loadOBJFile(options.output_path + ".obj", mesh2);
    textured_mesh.tex_materials = mesh2.tex_materials;

    pcl::visualization::PCLVisualizer visualizer;
    visualizer.addTextureMesh(textured_mesh);
    visualizer.spin();
  }

  return 0;
}
