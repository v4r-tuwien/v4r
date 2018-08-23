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

#pragma once

#include <string>

#include <v4r/core/macros.h>
#include <v4r/io/grabber.h>

namespace v4r {
namespace io {

/// Grab PCD files from a given directory.
///
/// All files matching format "cloud_%u.pcd" are loaded in alphabetical order. The second part of the file name is
/// considered to be a timestamp.
///
/// The width and height, as well as availability of RGB data, are determined from the very first PCD file. If any of
/// the following files differs in width/height or available fields, an exception is thrown.
///
/// If the directory contains "intrinsics.{txt,yaml,yml,xml}", Intrinsics will be loaded from it and made available
/// through getCameraIntrinsics(). If the file is not present, default Kinect intrinsics are assumed.
class V4R_EXPORTS PCDGrabber : public Grabber {
 public:
  using Ptr = std::shared_ptr<PCDGrabber>;

  /// Construct a grabber for a given directory.
  PCDGrabber(const std::string& directory);

  ~PCDGrabber() override;

  Timestamp grabFrame(cv::OutputArray color, cv::OutputArray depth) override;

  bool hasMoreFrames() const override;

  int getNumberOfFrames() const override;

  int getCurrentFrameIndex() const override;

  Intrinsics getCameraIntrinsics() const override;

  using Grabber::printInfo;

  void printInfo(std::ostream& os) const override;

  StreamMode getActiveColorStreamMode() const override;

  StreamMode getActiveDepthStreamMode() const override;

  bool isFeatureSupported(Feature feature) const override;

  void seek(unsigned int index) override;

  void setRepeatEnabled(bool state) override;

  bool getRepeatEnabled() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> p;
};

}  // namespace io
}  // namespace v4r
