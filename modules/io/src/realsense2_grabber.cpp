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

#include <chrono>
#include <thread>

#include <librealsense2/rs.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/throw_exception.hpp>

#include <v4r/io/realsense2_grabber.h>

namespace v4r {
namespace io {

namespace {

// Test if a given device is an Intel RealSense device.
bool isRealSenseDevice(const rs2::device& device) {
  return boost::algorithm::starts_with(device.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense");
}

// Get an Intel RealSense device with a given serial number.
// Throws if no such device is connected.
rs2::device getDevice(const rs2::context& context, const std::string& serial_number) {
  for (const auto& device : context.query_devices())
    if (std::string(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) == serial_number)
      return device;
  BOOST_THROW_EXCEPTION(GrabberException("No Intel RealSense2-compatible device with a given serial number connected")
                        << GrabberException::SerialNumber(serial_number));
}

// Get any connected Intel RealSense device.
// Throws if no devices are connected.
rs2::device getDevice(const rs2::context& context) {
  for (const auto& device : context.query_devices())
    if (isRealSenseDevice(device))
      return device;
  BOOST_THROW_EXCEPTION(GrabberException("No Intel RealSense2-compatible devices connected"));
}

// Get the sensor providing a given stream on a given device.
// Returns empty sensor if not found.
rs2::sensor getSensor(rs2::device device, rs2_stream stream) {
  for (const auto& sensor : device.query_sensors())
    for (const auto& profile : sensor.get_stream_profiles())
      if (profile.stream_type() == stream)
        return sensor;
  return {};
}

// Get a list of stream modes for a given stream and pixel format on a given device.
Grabber::StreamModes getSupportedModes(rs2::device device, rs2_stream stream, rs2_format format) {
  Grabber::StreamModes modes;
  for (const auto& sensor : device.query_sensors())
    for (const auto& profile : sensor.get_stream_profiles())
      if (profile.stream_type() == stream && profile.format() == format)
        if (auto video = profile.as<rs2::video_stream_profile>())
          modes.emplace_back(cv::Size{video.width(), video.height()}, profile.fps());
  return modes;
}

}  // anonymous namespace

struct RealSense2Grabber::Impl {
  Mode grabber_mode;
  StreamMode color_stream_mode;
  StreamMode depth_stream_mode;

  rs2::context ctx;
  rs2::pipeline pipeline;
  rs2::config config;
  rs2::device device;
  std::unique_ptr<rs2::align> align;
  rs2::sensor color_sensor;
  std::string serial_number;

  int next_frame_index = 0;
  Timestamp timestamp_zero_offset = 0;

  Intrinsics intrinsics;

  const rs2_format COLOR_FORMAT{RS2_FORMAT_BGR8};
  const rs2_format DEPTH_FORMAT{RS2_FORMAT_Z16};

  Impl() : pipeline(ctx) {}

  void open(const std::string& device_name, Mode mode) {
    if (mode.color == Mode::DISABLE && mode.depth == Mode::DISABLE)
      BOOST_THROW_EXCEPTION(GrabberException("At least one of the streams should be enabled"));

    // Step 1
    // Acquire an appropriate device

    // If device name is not one of the special strings, then it is assumed to be a serial number
    if (device_name != "" && device_name != "intel" && device_name != "realsense2" && device_name != "rs2") {
      device = getDevice(ctx, device_name);
      serial_number = device_name;
    } else {
      device = getDevice(ctx);
      serial_number = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    }

    config.disable_all_streams();
    config.enable_device(serial_number);

    // Step 2
    // Enable depth stream

    if (mode.depth != Mode::DISABLE) {
      if (mode.depth == Mode::DEFAULT) {
        config.enable_stream(RS2_STREAM_DEPTH, DEPTH_FORMAT);
      } else {
        auto modes = getSupportedModes(device, RS2_STREAM_DEPTH, DEPTH_FORMAT);
        if (mode.depth > static_cast<uint8_t>(modes.size()))
          BOOST_THROW_EXCEPTION(GrabberException("Invalid depth mode"));
        const auto& m = modes[mode.depth - 1];
        config.enable_stream(RS2_STREAM_DEPTH, -1, m.resolution.width, m.resolution.height, DEPTH_FORMAT, m.fps);
      }
    }

    // Step 3
    // Enable color stream

    if (mode.color != Mode::DISABLE) {
      if (mode.color == Mode::DEFAULT) {
        config.enable_stream(RS2_STREAM_COLOR, COLOR_FORMAT);
      } else {
        auto modes = getSupportedModes(device, RS2_STREAM_COLOR, COLOR_FORMAT);
        if (mode.color > static_cast<uint8_t>(modes.size()))
          BOOST_THROW_EXCEPTION(GrabberException("Invalid color mode"));
        const auto& m = modes[mode.color - 1];
        config.enable_stream(RS2_STREAM_COLOR, -1, m.resolution.width, m.resolution.height, COLOR_FORMAT, m.fps);
      }
      color_sensor = getSensor(device, RS2_STREAM_COLOR);
    }

    // Step 4
    // Start the pipeline and query stream resolutions and framerates

    auto profile = pipeline.start(config);

    if (auto depth = profile.get_stream(RS2_STREAM_DEPTH)) {
      auto s = depth.as<rs2::video_stream_profile>();
      depth_stream_mode.resolution = {s.width(), s.height()};
      depth_stream_mode.fps = depth.fps();
    }

    if (auto color = profile.get_stream(RS2_STREAM_COLOR)) {
      auto s = color.as<rs2::video_stream_profile>();
      color_stream_mode.resolution = {s.width(), s.height()};
      color_stream_mode.fps = color.fps();
    }

    // Step 5
    // Setup alignment processing block

    // Force "No alignment" mode if any of the streams is disabled
    if (mode.color == Mode::DISABLE || mode.depth == Mode::DISABLE)
      mode.alignment = Mode::NO_ALIGNMENT;

    // Configure alignment processing block and identify which stream is the "main" one
    // Main stream is the stream onto which alignment is happening
    rs2_stream main_stream;
    if (mode.alignment == Mode::COLOR_TO_DEPTH) {
      align.reset(new rs2::align(RS2_STREAM_DEPTH));
      main_stream = RS2_STREAM_DEPTH;
      color_stream_mode.resolution = depth_stream_mode.resolution;
    } else if (mode.alignment == Mode::DEPTH_TO_COLOR) {
      align.reset(new rs2::align(RS2_STREAM_COLOR));
      main_stream = RS2_STREAM_COLOR;
      depth_stream_mode.resolution = color_stream_mode.resolution;
    } else {
      main_stream = mode.depth != Mode::DISABLE ? RS2_STREAM_DEPTH : RS2_STREAM_COLOR;
    }

    // Step 6
    // Query intrinsics

    auto intr = profile.get_stream(main_stream).as<rs2::video_stream_profile>().get_intrinsics();
    intrinsics = {intr.fx, intr.fy, intr.ppx, intr.ppy, intr.width, intr.height};

    grabber_mode = mode;
  }

  Timestamp grabFrame(cv::Mat& color, cv::Mat& depth) {
    auto frameset = pipeline.wait_for_frames();

    if (align)
      frameset = align->process(frameset);

    if (grabber_mode.depth != Mode::DISABLE) {
      auto depth_data = frameset.get_depth_frame();
      memcpy(depth.data, depth_data.get_data(), depth.total() * depth.elemSize());
    }

    if (grabber_mode.color != Mode::DISABLE) {
      auto color_data = frameset.get_color_frame();
      memcpy(color.data, color_data.get_data(), color.total() * color.elemSize());
    }

    ++next_frame_index;
    return computeTimestamp(frameset.get_timestamp());
  }

  Timestamp computeTimestamp(double device_timestamp) {
    if (timestamp_zero_offset == 0) {
      auto now = std::chrono::high_resolution_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
      timestamp_zero_offset = ms - device_timestamp;
    }
    return (timestamp_zero_offset + device_timestamp);
  }
};

std::vector<std::string> RealSense2Grabber::enumerateConnectedDevices() {
  rs2::context ctx;
  std::vector<std::string> devices;
  for (const auto& device : ctx.query_devices())
    if (isRealSenseDevice(device))
      devices.emplace_back(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
  return devices;
}

RealSense2Grabber::RealSense2Grabber(const std::string& device_uri) : p(new Impl) {
  auto hash = device_uri.find_first_of("#");
  if (hash != std::string::npos)
    p->open(device_uri.substr(0, hash), Mode::parse(device_uri.substr(hash + 1)));
  else
    p->open(device_uri, Mode());
}

RealSense2Grabber::RealSense2Grabber(Mode mode, const std::string& device_uri) : p(new Impl) {
  auto hash = device_uri.find_first_of("#");
  if (hash != std::string::npos)
    p->open(device_uri.substr(0, hash), mode);
  else
    p->open(device_uri, mode);
}

RealSense2Grabber::~RealSense2Grabber() = default;

Grabber::Timestamp RealSense2Grabber::grabFrame(cv::OutputArray _color, cv::OutputArray _depth) {
  _color.create(p->color_stream_mode.resolution, CV_8UC3);
  _depth.create(p->depth_stream_mode.resolution, CV_16UC1);
  cv::Mat color = _color.getMat();
  cv::Mat depth = _depth.getMat();
  return p->grabFrame(color, depth);
}

inline bool RealSense2Grabber::hasMoreFrames() const {
  return true;
}

inline int RealSense2Grabber::getNumberOfFrames() const {
  return -1;
}

inline int RealSense2Grabber::getCurrentFrameIndex() const {
  return p->next_frame_index - 1;
}

inline Intrinsics RealSense2Grabber::getCameraIntrinsics() const {
  return p->intrinsics;
}

void RealSense2Grabber::printInfo(std::ostream& os) const {
  static std::string ALIGNMENT[] = {"color → depth", "depth → color", "no alignment"};
  os << "RealSense2Grabber :: " << p->serial_number << std::endl;
  os << " Firmware: " << p->device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
  os << " Alignment: " << ALIGNMENT[p->grabber_mode.alignment] << std::endl;
  Grabber::printInfo(os);
}

Grabber::StreamModes RealSense2Grabber::getSupportedColorStreamModes() const {
  return getSupportedModes(p->device, RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
}

Grabber::StreamModes RealSense2Grabber::getSupportedDepthStreamModes() const {
  return getSupportedModes(p->device, RS2_STREAM_DEPTH, RS2_FORMAT_Z16);
}

Grabber::StreamMode RealSense2Grabber::getActiveColorStreamMode() const {
  return p->color_stream_mode;
}

Grabber::StreamMode RealSense2Grabber::getActiveDepthStreamMode() const {
  return p->depth_stream_mode;
}

bool RealSense2Grabber::isFeatureSupported(Feature feature) const {
  switch (feature) {
    case Feature::AUTO_WHITE_BALANCE:
      return p->color_sensor && p->color_sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
    case Feature::AUTO_EXPOSURE:
      return p->color_sensor && p->color_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE);
    case Feature::EXPOSURE:
      return p->color_sensor && p->color_sensor.supports(RS2_OPTION_EXPOSURE);
    case Feature::GAIN:
      return p->color_sensor && p->color_sensor.supports(RS2_OPTION_GAIN);
    case Feature::WHITE_BALANCE:
      return p->color_sensor && p->color_sensor.supports(RS2_OPTION_WHITE_BALANCE);
    case Feature::SERIAL_NUMBER:
      return true;
    case Feature::CAMERA_MODEL_NAME:
      return p->device.supports(RS2_CAMERA_INFO_NAME);
    default:
      return false;
  }
}

void RealSense2Grabber::setAutoWhiteBalanceEnabled(bool state) {
  throwIfNotSupported(Feature::AUTO_WHITE_BALANCE);
  p->color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, state ? 1.0 : 0.0);
}

bool RealSense2Grabber::getAutoWhiteBalanceEnabled() const {
  throwIfNotSupported(Feature::AUTO_WHITE_BALANCE);
  return p->color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) == 1.0;
}

void RealSense2Grabber::setAutoExposureEnabled(bool state) {
  throwIfNotSupported(Feature::AUTO_EXPOSURE);
  p->color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, state ? 1.0 : 0.0);
}

bool RealSense2Grabber::getAutoExposureEnabled() const {
  throwIfNotSupported(Feature::AUTO_EXPOSURE);
  return p->color_sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE) == 1.0;
}

void RealSense2Grabber::setExposure(unsigned int exposure) {
  throwIfNotSupported(Feature::EXPOSURE);
  p->color_sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
}

unsigned int RealSense2Grabber::getExposure() const {
  throwIfNotSupported(Feature::EXPOSURE);
  return p->color_sensor.get_option(RS2_OPTION_EXPOSURE);
}

std::pair<unsigned int, unsigned int> RealSense2Grabber::getExposureRange() const {
  throwIfNotSupported(Feature::EXPOSURE);
  auto range = p->color_sensor.get_option_range(RS2_OPTION_EXPOSURE);
  return {range.min, range.max};
}

void RealSense2Grabber::setGain(int gain) {
  throwIfNotSupported(Feature::GAIN);
  p->color_sensor.set_option(RS2_OPTION_GAIN, gain);
}

int RealSense2Grabber::getGain() const {
  throwIfNotSupported(Feature::GAIN);
  return p->color_sensor.get_option(RS2_OPTION_GAIN);
}

void RealSense2Grabber::setWhiteBalance(int white_balance) {
  throwIfNotSupported(Feature::WHITE_BALANCE);
  p->color_sensor.set_option(RS2_OPTION_WHITE_BALANCE, white_balance);
}

int RealSense2Grabber::getWhiteBalance() const {
  throwIfNotSupported(Feature::WHITE_BALANCE);
  return p->color_sensor.get_option(RS2_OPTION_WHITE_BALANCE);
}

std::string RealSense2Grabber::getSerialNumber() const {
  return p->serial_number;
}

std::string RealSense2Grabber::getCameraModelName() const {
  std::string expected_prefix = "Intel RealSense ";
  std::string name = p->device.get_info(RS2_CAMERA_INFO_NAME);
  auto pos = name.find(expected_prefix);
  if (pos == std::string::npos)
    BOOST_THROW_EXCEPTION(GrabberException("Unable to extract camera model name from: " + name));
  name = "d" + name.substr(pos + expected_prefix.size());
  boost::algorithm::to_lower(name);
  return name;
}

RealSense2Grabber::Mode RealSense2Grabber::Mode::parse(const std::string& mode_string) {
  Mode mode;
  char r = 0;
  if (sscanf(mode_string.c_str(), "%hhu/%hhu/%c", &mode.color, &mode.depth, &r) != 3 &&
      sscanf(mode_string.c_str(), "%hhu/%hhu", &mode.color, &r) != 2)
    BOOST_THROW_EXCEPTION(GrabberException("Invalid mode specification"));
  if (r == 'd' || r == 'D')
    mode.alignment = COLOR_TO_DEPTH;
  else if (r == 'c' || r == 'C')
    mode.alignment = DEPTH_TO_COLOR;
  else
    mode.alignment = NO_ALIGNMENT;
  return mode;
}

}  // namespace io
}  // namespace v4r
