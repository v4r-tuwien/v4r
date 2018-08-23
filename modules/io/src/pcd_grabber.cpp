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

#include <cinttypes>

#include <boost/filesystem.hpp>
#include <boost/throw_exception.hpp>

#include <pcl/io/pcd_io.h>

#include <v4r/io/pcd_grabber.h>

namespace fs = boost::filesystem;

namespace v4r {
namespace io {

struct PCDGrabber::Impl {
  cv::Size image_resolution = {0, 0};
  size_t image_size = 0;
  bool has_color = false;
  Intrinsics intrinsics;
  fs::path path;
  std::list<std::pair<double, fs::path>> frames;
  std::list<std::pair<double, fs::path>>::iterator current_frame;
  bool repeat = false;

  pcl::PCDReader reader;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  Impl(const std::string& directory) : path(directory) {
    if (!fs::exists(path) || !fs::is_directory(path))
      BOOST_THROW_EXCEPTION(GrabberException("Path does not exist or is not a directory")
                            << GrabberException::Filename(path.string()));

    bool intrinsics_loaded = false;
    for (const auto& extension : {"txt", "yml", "yaml", "xml"}) {
      auto intrinsics_filename = path / "intrinsics";
      intrinsics_filename.replace_extension(extension);
      if (fs::exists(intrinsics_filename)) {
        intrinsics = Intrinsics::load(intrinsics_filename.string());
        intrinsics_loaded = true;
        break;
      }
    }
    if (!intrinsics_loaded)
      intrinsics = Intrinsics::PrimeSense();

    loadFrames();
    if (frames.empty())
      BOOST_THROW_EXCEPTION(GrabberException("No PCD files found"));

    current_frame = frames.begin();

    loadMetadata(current_frame->second);
  }

  void loadFrames() {
    std::vector<fs::path> files;
    std::copy(fs::directory_iterator(fs::path(path)), fs::directory_iterator(), std::back_inserter(files));
    std::sort(files.begin(), files.end());
    for (const auto& f : files) {
      auto extension = f.extension().string();
      if (extension == ".pcd") {
        Timestamp timestamp;
        if (sscanf(f.stem().string().c_str(), "cloud_%" SCNu64, &timestamp) == 1)
          frames.emplace_back(timestamp, f);
      }
    }
  }

  void loadMetadata(fs::path& p) {
    if (reader.readHeader(p.string(), pcl_cloud))
      BOOST_THROW_EXCEPTION(GrabberException("Failed to read from PCD file") << GrabberException::Filename(p.string()));

    image_resolution = {pcl_cloud.width, pcl_cloud.height};
    has_color = false;
    for (const auto& field : pcl_cloud.fields)
      if (field.name == "rgb" || field.name == "rgba")
        has_color = true;
  }

  void loadPCD(const fs::path& p, cv::Mat& color, cv::Mat& depth) {
    if (reader.read(p.string(), pcl_cloud))
      BOOST_THROW_EXCEPTION(GrabberException("Failed to read from PCD file") << GrabberException::Filename(p.string()));

    if (static_cast<int>(pcl_cloud.width) != image_resolution.width ||
        static_cast<int>(pcl_cloud.height) != image_resolution.height)
      BOOST_THROW_EXCEPTION(GrabberException("PCD cloud resolution mismatch")
                            << GrabberException::Filename(p.string()));

    std::set<std::string> available_fields;
    for (const auto& field : pcl_cloud.fields)
      available_fields.insert(field.name);

    if (has_color && !(available_fields.count("rgb") || available_fields.count("rgba")))
      BOOST_THROW_EXCEPTION(GrabberException("PCD file does not have color field")
                            << GrabberException::Filename(p.string()));

    pcl::fromPCLPointCloud2(pcl_cloud, cloud);

    for (size_t i = 0; i < cloud.height; ++i)
      for (size_t j = 0; j < cloud.width; ++j) {
        depth.at<float>(i, j) = cloud(j, i).z;
        if (has_color)
          color.at<cv::Vec3b>(i, j) = {cloud(j, i).b, cloud(j, i).g, cloud(j, i).r};
      }
  }

  Timestamp grabFrame(cv::Mat& color, cv::Mat& depth) {
    Timestamp ts = 0;
    if (current_frame != frames.end()) {
      loadPCD(current_frame->second, color, depth);
      ts = current_frame->first;
      if (++current_frame == frames.end() && repeat)
        current_frame = frames.begin();
    }
    return ts;
  }
};

PCDGrabber::PCDGrabber(const std::string& directory) : p(new Impl(directory)) {}

PCDGrabber::~PCDGrabber() = default;

Grabber::Timestamp PCDGrabber::grabFrame(cv::OutputArray _color, cv::OutputArray _depth) {
  if (p->has_color)
    _color.create(p->image_resolution.height, p->image_resolution.width, CV_8UC3);
  else
    _color.clear();
  _depth.create(p->image_resolution.height, p->image_resolution.width, CV_32FC1);
  cv::Mat color = _color.getMat();
  cv::Mat depth = _depth.getMat();
  return p->grabFrame(color, depth);
}

inline bool PCDGrabber::hasMoreFrames() const {
  return p->repeat || p->current_frame != p->frames.end();
}

int PCDGrabber::getNumberOfFrames() const {
  return static_cast<int>(p->frames.size());
}

int PCDGrabber::getCurrentFrameIndex() const {
  return static_cast<int>(std::distance(p->frames.begin(), p->current_frame)) - 1;
}

Intrinsics PCDGrabber::getCameraIntrinsics() const {
  return p->intrinsics;
}

void PCDGrabber::printInfo(std::ostream& os) const {
  os << "PCDGrabber :: " << p->path.string() << std::endl;
  os << " Number of frames: " << p->frames.size() << std::endl;
  Grabber::printInfo(os);
}

Grabber::StreamMode PCDGrabber::getActiveColorStreamMode() const {
  return {p->image_resolution, 0};
}

Grabber::StreamMode PCDGrabber::getActiveDepthStreamMode() const {
  return {p->image_resolution, 0};
}

bool PCDGrabber::isFeatureSupported(Feature feature) const {
  switch (feature) {
    case Feature::SEEK:
    case Feature::REPEAT:
      return true;
    default:
      return false;
  }
}

void PCDGrabber::seek(unsigned int index) {
  if (index > p->frames.size())
    BOOST_THROW_EXCEPTION(GrabberException("Seeking to a non-existent index") << GrabberException::Index(index));
  p->current_frame = p->frames.begin();
  std::advance(p->current_frame, index);
}

void PCDGrabber::setRepeatEnabled(bool state) {
  throwIfNotSupported(Feature::REPEAT);
  p->repeat = state;
}

bool PCDGrabber::getRepeatEnabled() const {
  return p->repeat;
}

}  // namespace io
}  // namespace v4r
