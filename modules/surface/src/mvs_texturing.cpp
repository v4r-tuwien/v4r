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

#include <glog/logging.h>

#include <boost/filesystem.hpp>

#include <mve/mesh.h>
#include <mvstexturing/texturing.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <v4r/surface/mvs_texturing.h>

namespace {

/// Convert PCL mesh into MVE mesh.
mve::TriangleMesh::Ptr convertMesh(const pcl::PolygonMesh& in) {
  auto out = mve::TriangleMesh::create();
  auto& vertices = out->get_vertices();
  auto num_vertices = in.cloud.width * in.cloud.height;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(in.cloud, cloud);
  vertices.reserve(num_vertices);
  for (size_t i = 0; i < num_vertices; ++i)
    vertices.emplace_back(cloud[i].x, cloud[i].y, cloud[i].z);
  auto& faces = out->get_faces();
  faces.reserve(in.polygons.size() * 3);
  for (const auto& polygon : in.polygons)
    std::copy(polygon.vertices.begin(), polygon.vertices.end(), std::back_inserter(faces));
  return out;
}

/// Convert OpenCV image (BGR) into MVE image.
mve::ByteImage::Ptr convertImage(cv::InputArray in) {
  if (in.kind() != cv::_InputArray::MAT || in.type() != CV_8UC3)
    throw std::runtime_error("Invalid image");
  cv::Mat rgb;
  cv::cvtColor(in, rgb, cv::COLOR_BGR2RGB);
  auto out = mve::ByteImage::create(in.size().width, in.size().height, in.channels());
  std::memcpy(out->get_data_pointer(), rgb.data, in.size().area() * 3);
  return out;
}

/// Convert adapter parameters struct into MVS-Texturing settings.
tex::Settings convertParameters(const v4r::surface::MVSTexturing::Parameters& params) {
  tex::Settings settings;
  settings.geometric_visibility_test = params.geometric_visibility_test;
  settings.global_seam_leveling = params.global_seam_leveling;
  settings.local_seam_leveling = params.local_seam_leveling;
  settings.hole_filling = params.hole_filling;
  settings.keep_unseen_faces = params.keep_unseen_faces;
  return settings;
}

}  // anonymous namespace

namespace v4r {
namespace surface {

namespace po = boost::program_options;

void MVSTexturing::Parameters::init(boost::program_options::options_description& desc,
                                    const std::string& section_name) {
  desc.add_options()((section_name + ".gv").c_str(),
                     po::value<bool>(&geometric_visibility_test)->default_value(geometric_visibility_test),
                     "perform geometric visibility test  based on ray intersection");
  desc.add_options()((section_name + ".gsl").c_str(),
                     po::value<bool>(&global_seam_leveling)->default_value(global_seam_leveling),
                     "perform global seam leveling");
  desc.add_options()((section_name + ".lsl").c_str(),
                     po::value<bool>(&local_seam_leveling)->default_value(local_seam_leveling),
                     "perform local seam leveling (Poisson editing)");
  desc.add_options()((section_name + ".hf").c_str(), po::value<bool>(&hole_filling)->default_value(hole_filling),
                     "perform hole filling");
  desc.add_options()((section_name + ".kuf").c_str(),
                     po::value<bool>(&keep_unseen_faces)->default_value(keep_unseen_faces), "keep unseen faces");
}

void MVSTexturing::mapTextureToMesh(const pcl::PolygonMesh& mesh, const v4r::Intrinsics& camera_intrinsics,
                                    const std::vector<cv::Mat>& camera_images, const PoseVector& camera_poses,
                                    const std::string& output_path) {
  auto m = convertMesh(mesh);
  mve::MeshInfo mesh_info(m);
  tex::prepare_mesh(&mesh_info, m);
  std::size_t const num_faces = m->get_faces().size() / 3;

  LOG(INFO) << "Input mesh: " << mesh_info.size() << " vertices, " << num_faces << " faces";

  mve::CameraInfo cam;
  cam.dist[0] = 0;
  cam.dist[1] = 0;
  cam.flen = camera_intrinsics.fx / camera_intrinsics.w;
  cam.paspect = camera_intrinsics.fx / camera_intrinsics.fy;
  cam.ppoint[0] = camera_intrinsics.cx / camera_intrinsics.w;
  cam.ppoint[1] = camera_intrinsics.cy / camera_intrinsics.h;

  tex::TextureViews texture_views;
  for (size_t i = 0; i < camera_images.size(); ++i) {
    // mve::CameraInfo accepts transformation as a pointer to row-major matrix, but Eigen is column-major by default.
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> p = camera_poses[i];
    cam.set_transformation(p.data());
    auto image = convertImage(camera_images[i]);
    texture_views.emplace_back(i, cam, image);
  }

  LOG(INFO) << "Input camera views: " << texture_views.size();

  tex::Graph graph(num_faces);
  tex::build_adjacency_graph(m, mesh_info, &graph);

  auto settings = convertParameters(params_);
  tex::DataCosts data_costs(num_faces, texture_views.size());
  tex::calculate_data_costs(m, &texture_views, settings, &data_costs);

  tex::view_selection(data_costs, &graph, settings);

  tex::TextureAtlases texture_atlases;
  tex::TexturePatches texture_patches;
  tex::VertexProjectionInfos vertex_projection_infos;

  tex::generate_texture_patches(graph, m, mesh_info, &texture_views, settings, &vertex_projection_infos,
                                &texture_patches);

  CHECK(!texture_patches.empty()) << "No texture patches generated";

  if (settings.global_seam_leveling) {
    tex::global_seam_leveling(graph, m, mesh_info, vertex_projection_infos, &texture_patches);
  } else {
#pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < texture_patches.size(); ++i) {
      std::vector<math::Vec3f> patch_adjust_values(texture_patches[i]->get_faces().size() * 3, math::Vec3f(0.0f));
      texture_patches[i]->adjust_colors(patch_adjust_values);
    }
  }

  if (settings.local_seam_leveling) {
    tex::local_seam_leveling(graph, m, vertex_projection_infos, &texture_patches);
  }

  tex::generate_texture_atlases(&texture_patches, settings, &texture_atlases);

  tex::Model model;
  tex::build_model(m, texture_atlases, &model);

  namespace bf = boost::filesystem;
  auto path = bf::absolute(output_path);
  if (!bf::exists(path.parent_path())) {
    LOG(INFO) << "Creating directory " << path.parent_path();
    bf::create_directories(path.parent_path());
  }

  LOG(INFO) << "Saving textured model to " << path << ".obj";

  tex::Model::save(model, path.c_str());
}

}  // namespace surface
}  // namespace v4r
