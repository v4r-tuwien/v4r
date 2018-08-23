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

#include <boost/format.hpp>

#include <v4r/io/filesystem.h>
#include <v4r/io/grabber.h>

namespace v4r {
namespace io {

template <Grabber::Feature F>
void printFeatureState(std::ostream& /* os */, const Grabber* /* g */) {}

template <>
void printFeatureState<Grabber::Feature::AUTO_WHITE_BALANCE>(std::ostream& os, const Grabber* g) {
  os << (g->getAutoWhiteBalanceEnabled() ? "ON" : "OFF");
}

template <>
void printFeatureState<Grabber::Feature::AUTO_EXPOSURE>(std::ostream& os, const Grabber* g) {
  os << (g->getAutoExposureEnabled() ? "ON" : "OFF");
}

template <>
void printFeatureState<Grabber::Feature::EXPOSURE>(std::ostream& os, const Grabber* g) {
  auto range = g->getExposureRange();
  os << g->getExposure() << " [" << range.first << " .. " << range.second << "]";
}

template <>
void printFeatureState<Grabber::Feature::GAIN>(std::ostream& os, const Grabber* g) {
  os << g->getGain();
}

template <>
void printFeatureState<Grabber::Feature::WHITE_BALANCE>(std::ostream& os, const Grabber* g) {
  os << g->getWhiteBalance();
}

template <>
void printFeatureState<Grabber::Feature::SERIAL_NUMBER>(std::ostream& os, const Grabber* g) {
  os << g->getSerialNumber();
}

template <>
void printFeatureState<Grabber::Feature::CAMERA_MODEL_NAME>(std::ostream& os, const Grabber* g) {
  os << g->getCameraModelName();
}

template <Grabber::Feature F>
void printFeature(std::ostream& os, const Grabber* g) {
  static std::string FEATURE_NAMES[] = {"SEEK", "REPEAT",        "AUTO_WHITE_BALANCE", "AUTO_EXPOSURE",    "EXPOSURE",
                                        "GAIN", "WHITE_BALANCE", "SERIAL_NUMBER",      "CAMERA_MODEL_NAME"};
  const auto& name = FEATURE_NAMES[static_cast<unsigned int>(F)];
  auto supported = g->isFeatureSupported(F);
  os << "  " << (supported ? "[+]" : " - ") << " " << std::setw(23) << std::left << std::setfill(' ') << name
     << std::right;
  if (supported)
    printFeatureState<F>(os, g);
  os << std::endl;
}

void Grabber::printInfo(std::ostream& os) const {
  os << " Features:" << std::endl;
  printFeature<Feature::SEEK>(os, this);
  printFeature<Feature::REPEAT>(os, this);
  printFeature<Feature::AUTO_WHITE_BALANCE>(os, this);
  printFeature<Feature::AUTO_EXPOSURE>(os, this);
  printFeature<Feature::EXPOSURE>(os, this);
  printFeature<Feature::GAIN>(os, this);
  printFeature<Feature::WHITE_BALANCE>(os, this);
  printFeature<Feature::SERIAL_NUMBER>(os, this);
  printFeature<Feature::CAMERA_MODEL_NAME>(os, this);

  // Stream modes
  auto print_mode = [&os](const StreamMode& mode, const StreamMode& current, int i) {
    const char* br = mode == current ? "[]" : "  ";
    os << " " << (i < 9 ? " " : "") << br[0] << i + 1 << br[1] << " ";
    os << std::setw(4) << mode.resolution.width << " x " << std::setw(4) << mode.resolution.height;
    if (mode.fps > 0)
      os << " @ " << std::setw(3) << mode.fps << " fps";
    os << std::endl;
  };
  os << " Color stream modes:" << std::endl;
  auto color_modes = getSupportedColorStreamModes();
  for (size_t i = 0; i < color_modes.size(); ++i)
    print_mode(color_modes[i], getActiveColorStreamMode(), i);
  os << " Depth stream modes:" << std::endl;
  auto depth_modes = getSupportedDepthStreamModes();
  for (size_t i = 0; i < depth_modes.size(); ++i)
    print_mode(depth_modes[i], getActiveDepthStreamMode(), i);
}

void Grabber::printInfo() const {
  printInfo(std::cout);
}

bool Grabber::isFile() const {
  return getNumberOfFrames() != -1;
}

bool Grabber::isDevice() const {
  return getNumberOfFrames() == -1;
}

Grabber::StreamModes Grabber::getSupportedColorStreamModes() const {
  // Default implementation, useful for file-based grabbers where only a single streaming mode is available
  if (hasColorStream())
    return {getActiveColorStreamMode()};
  return {};
}

Grabber::StreamModes Grabber::getSupportedDepthStreamModes() const {
  // Default implementation, useful for file-based grabbers where only a single streaming mode is available
  if (hasDepthStream())
    return {getActiveDepthStreamMode()};
  return {};
}

bool Grabber::hasColorStream() const {
  return getActiveColorStreamMode().resolution.area() != 0;
}

bool Grabber::hasDepthStream() const {
  return getActiveDepthStreamMode().resolution.area() != 0;
}

bool Grabber::isFeatureSupported(Feature /*feature*/) const {
  return false;
}

void Grabber::seek(unsigned int /*index*/) {
  throwIfNotSupported(Feature::SEEK);
}

void Grabber::setRepeatEnabled(bool /*state*/) {
  throwIfNotSupported(Feature::REPEAT);
}

bool Grabber::getRepeatEnabled() const {
  throwIfNotSupported(Feature::REPEAT);
  return false;
}

void Grabber::setAutoWhiteBalanceEnabled(bool /*state*/) {
  throwIfNotSupported(Feature::AUTO_WHITE_BALANCE);
}

bool Grabber::getAutoWhiteBalanceEnabled() const {
  throwIfNotSupported(Feature::AUTO_WHITE_BALANCE);
  return false;
}

void Grabber::setAutoExposureEnabled(bool /*state*/) {
  throwIfNotSupported(Feature::AUTO_EXPOSURE);
}

bool Grabber::getAutoExposureEnabled() const {
  throwIfNotSupported(Feature::AUTO_EXPOSURE);
  return false;
}

void Grabber::setExposure(unsigned int /*exposure*/) {
  throwIfNotSupported(Feature::EXPOSURE);
}

unsigned int Grabber::getExposure() const {
  throwIfNotSupported(Feature::EXPOSURE);
  return 0;
}

std::pair<unsigned int, unsigned int> Grabber::getExposureRange() const {
  throwIfNotSupported(Feature::EXPOSURE);
  return {0, 0};
}

void Grabber::setGain(int /*gain*/) {
  throwIfNotSupported(Feature::GAIN);
}

int Grabber::getGain() const {
  throwIfNotSupported(Feature::GAIN);
  return 0;
}

void Grabber::setWhiteBalance(int /*white_balance*/) {
  throwIfNotSupported(Feature::WHITE_BALANCE);
}

int Grabber::getWhiteBalance() const {
  throwIfNotSupported(Feature::WHITE_BALANCE);
  return 0;
}

std::string Grabber::getSerialNumber() const {
  throwIfNotSupported(Feature::SERIAL_NUMBER);
  return "";
}

std::string Grabber::getCameraModelName() const {
  throwIfNotSupported(Feature::CAMERA_MODEL_NAME);
  return "";
}

std::string Grabber::getCameraUID() const {
  throwIfNotSupported(Feature::CAMERA_MODEL_NAME);
  throwIfNotSupported(Feature::SERIAL_NUMBER);
  return getCameraModelName() + "." + getSerialNumber();
}

void Grabber::throwIfNotSupported(Feature feature) const {
  if (!isFeatureSupported(feature))
    BOOST_THROW_EXCEPTION(GrabberFeatureNotSupportedException()
                          << GrabberFeatureNotSupportedException::Feature(feature));
}

bool Grabber::tryLoadIntrinsics(Intrinsics& intr) const {
  VLOG(1) << "Trying to load camera intrinsics";

  if (!isFeatureSupported(Feature::CAMERA_MODEL_NAME) || !isFeatureSupported(Feature::SERIAL_NUMBER)) {
    VLOG(1) << "Unable to construct intrinsics filename because camera model name or serial number is not known";
    return false;
  }

  auto uid = getCameraUID();
  auto dir = getDataDir() / "calibration";
  VLOG(1) << "Camera UID: " << uid;
  VLOG(1) << "Calibration data directory: " << dir;

  for (const auto& ext : {"txt", "intr", "yaml", "yml", "xml"})
    try {
      auto fn = dir / (uid + "." + ext);
      VLOG(1) << "Loading intrinsics from file: " << fn << "";
      intr = Intrinsics::load(fn.string());
      VLOG(1) << "Loaded intrinsics: " << intr;
      return true;
    } catch (std::runtime_error& e) {
      VLOG(1) << "Failed to load: " << e.what();
    }

  VLOG(1) << "Unable to load camera intrinsics";
  return false;
}

bool Grabber::StreamMode::operator==(const StreamMode& m) const {
  return this->resolution == m.resolution && this->fps == m.fps;
}

Grabber::StreamMode::StreamMode(cv::Size _resolution, unsigned int _fps) : resolution(_resolution), fps(_fps) {}

int selectStreamMode(const Grabber::StreamModes& modes, const Grabber::StreamMode& target_mode, bool exact) {
  if (modes.empty())
    return -1;

  // How good is the match between two modes, the smaller the better
  auto compute_score = [](const Grabber::StreamMode& m1, const Grabber::StreamMode& m2) {
    return std::abs(m1.resolution.width - m2.resolution.width) * 1e9 +
           std::abs(m1.resolution.height - m2.resolution.height) * 1e4 +
           std::abs(static_cast<int>(m1.fps) - static_cast<int>(m2.fps));
  };

  int best_mode_index = 0;
  for (size_t m = 1; m < modes.size(); ++m)
    if (compute_score(target_mode, modes[m]) < compute_score(target_mode, modes[best_mode_index]))
      best_mode_index = m;

  if (exact && compute_score(target_mode, modes[best_mode_index]) > 0)
    return -1;

  return best_mode_index;
}

}  // namespace io
}  // namespace v4r
