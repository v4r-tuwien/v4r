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

#include <OpenNI.h>

#include <boost/algorithm/string.hpp>
#include <boost/throw_exception.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <v4r/io/openni2_grabber.h>

namespace v4r {
namespace io {

struct OpenNI2Grabber::Impl {
  StreamMode color_stream_mode;
  StreamMode depth_stream_mode;
  bool depth_available = false;
  bool color_available = false;
  Intrinsics intrinsics;
  mutable openni::Device device;
  openni::VideoStream color_stream;
  openni::VideoStream depth_stream;
  openni::VideoFrameRef color_frame;
  openni::VideoFrameRef depth_frame;
  std::vector<openni::VideoStream*> streams;
  uint64_t timestamp_zero_offset = 0;
  int num_frames = -1;
  int next_frame_index = 0;
  bool repeat = false;
  bool is_file = false;

  const openni::PixelFormat COLOR_FORMAT{openni::PIXEL_FORMAT_RGB888};
  const openni::PixelFormat DEPTH_FORMAT{openni::PIXEL_FORMAT_DEPTH_1_MM};
  const StreamMode DEFAULT_STREAM_MODE{cv::Size{640, 480}, 30};

  Impl() {
    openni::OpenNI::initialize();
  }

  void open(const std::string& device_name, Mode mode) {
    const char* uri;
    if (device_name.empty() || device_name == "asus" || device_name == "kinect" || device_name == "openni2" ||
        device_name == "oni2")
      uri = openni::ANY_DEVICE;
    else
      uri = device_name.c_str();

    if (mode.color == Mode::DISABLE && mode.depth == Mode::DISABLE)
      BOOST_THROW_EXCEPTION(GrabberException("At least one of streams should be enabled"));

    if (device.open(uri) != openni::STATUS_OK)
      BOOST_THROW_EXCEPTION(GrabberException("Failed to open device")
                            << GrabberException::ErrorInfo(openni::OpenNI::getExtendedError()));

    if (mode.depth != Mode::DISABLE) {
      auto depth_modes = getSupportedModes(openni::SENSOR_DEPTH);

      if (mode.depth > static_cast<uint8_t>(depth_modes.size()) && mode.depth != Mode::DEFAULT)
        BOOST_THROW_EXCEPTION(GrabberException("Invalid depth mode"));

      if (depth_stream.create(device, openni::SENSOR_DEPTH) != openni::STATUS_OK)
        BOOST_THROW_EXCEPTION(GrabberException("Failed to create depth stream")
                              << GrabberException::ErrorInfo(openni::OpenNI::getExtendedError()));

      if (device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) != openni::STATUS_OK)
        BOOST_THROW_EXCEPTION(GrabberException("Failed to enable depth to color registration")
                              << GrabberException::ErrorInfo(openni::OpenNI::getExtendedError()));

      device.setDepthColorSyncEnabled(true);

      auto default_index = selectStreamMode(toStreamModes(depth_modes), DEFAULT_STREAM_MODE, false);
      auto index = mode.depth == Mode::DEFAULT ? default_index : mode.depth - 1;

      openni::VideoMode video_mode = depth_modes[index];
      depth_stream.setVideoMode(video_mode);
      depth_stream_mode.resolution.width = depth_stream.getVideoMode().getResolutionX();
      depth_stream_mode.resolution.height = depth_stream.getVideoMode().getResolutionY();
      depth_stream_mode.fps = depth_stream.getVideoMode().getFps();
      depth_stream.setMirroringEnabled(false);

      if (depth_stream.start() != openni::STATUS_OK) {
        depth_stream.destroy();
        BOOST_THROW_EXCEPTION(GrabberException("Failed to start depth stream")
                              << GrabberException::ErrorInfo(openni::OpenNI::getExtendedError()));
      }

      streams.push_back(&depth_stream);
      depth_available = true;
    }

    if (mode.color != Mode::DISABLE) {
      auto color_modes = getSupportedModes(openni::SENSOR_COLOR);

      if (mode.color > static_cast<uint8_t>(color_modes.size()) && mode.color != Mode::DEFAULT)
        BOOST_THROW_EXCEPTION(GrabberException("Invalid color mode"));

      if (color_stream.create(device, openni::SENSOR_COLOR) != openni::STATUS_OK)
        BOOST_THROW_EXCEPTION(GrabberException("Failed to create color stream")
                              << GrabberException::ErrorInfo(openni::OpenNI::getExtendedError()));

      auto default_index = selectStreamMode(toStreamModes(color_modes), DEFAULT_STREAM_MODE, false);
      auto index = mode.color == Mode::DEFAULT ? default_index : mode.color - 1;

      openni::VideoMode video_mode = color_modes[index];
      color_stream.setVideoMode(video_mode);
      color_stream_mode.resolution.width = color_stream.getVideoMode().getResolutionX();
      color_stream_mode.resolution.height = color_stream.getVideoMode().getResolutionY();
      color_stream_mode.fps = color_stream.getVideoMode().getFps();
      color_stream.setMirroringEnabled(false);

      if (color_stream.start() != openni::STATUS_OK) {
        color_stream.destroy();
        BOOST_THROW_EXCEPTION(GrabberException("Failed to start color stream")
                              << GrabberException::ErrorInfo(openni::OpenNI::getExtendedError()));
      }
      streams.push_back(&color_stream);
      color_available = true;
    }

    auto control = device.getPlaybackControl();
    if (control != nullptr) {
      // This is a file, make sure we get every frame
      control->setSpeed(-1.0f);
      control->setRepeatEnabled(repeat);
      num_frames = control->getNumberOfFrames(depth_stream);
      is_file = true;
      if (num_frames == -1)
        BOOST_THROW_EXCEPTION(GrabberException("Unable to determine number of frames in ONI file"));
    }

    if (color_stream_mode.resolution.width == 320 && color_stream_mode.resolution.height == 240)
      intrinsics.adjustToSize(320, 240);
  }

  ~Impl() {
    if (depth_available) {
      depth_stream.stop();
      depth_stream.destroy();
    }
    if (color_available) {
      color_stream.stop();
      color_stream.destroy();
    }
    device.close();
    openni::OpenNI::shutdown();
  }

  Timestamp grabFrame(cv::Mat& color, cv::Mat& depth) {
    int changed_index;
    auto num_streams = static_cast<int>(streams.size());
    auto status = openni::OpenNI::waitForAnyStream(streams.data(), num_streams, &changed_index);
    if (status != openni::STATUS_OK)
      return 0;

    if (depth_available)
      depth_stream.readFrame(&depth_frame);
    if (color_available)
      color_stream.readFrame(&color_frame);

    if (depth_available && !depth_frame.isValid())
      return 0;
    if (color_available && !color_frame.isValid())
      return 0;

    auto ts = depth_available ? depth_frame.getTimestamp() : color_frame.getTimestamp();
    auto timestamp = computeTimestamp(ts);

    if (color_available) {
      cv::Mat m(color_stream_mode.resolution, CV_8UC3, const_cast<void*>(color_frame.getData()), cv::Mat::AUTO_STEP);
      cv::cvtColor(m, color, cv::COLOR_RGB2BGR);
    }

    if (depth_available) {
      cv::Mat m(depth_stream_mode.resolution, CV_16UC1, const_cast<void*>(depth_frame.getData()), cv::Mat::AUTO_STEP);
      m.convertTo(depth, CV_32FC1, 0.001f);
    }

    ++next_frame_index;
    return timestamp;
  }

  Timestamp computeTimestamp(uint64_t device_timestamp) {
    if (timestamp_zero_offset == 0 && !is_file) {
      auto now = std::chrono::high_resolution_clock::now();
      auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
      timestamp_zero_offset = us - device_timestamp;
    }
    return timestamp_zero_offset + device_timestamp;
  }

  std::string getCameraModelName() const {
    std::string name = device.getDeviceInfo().getName();
    boost::algorithm::to_lower(name);
    return name;
  }

  std::string getSerialNumber() const {
    char serial[1024];
    device.getProperty(ONI_DEVICE_PROPERTY_SERIAL_NUMBER, &serial);
    return std::string(serial);
  }

  // Get the list of supported modes of a given sensor.
  std::vector<openni::VideoMode> getSupportedModes(openni::SensorType type) {
    const openni::PixelFormat PIXEL_FORMAT = (type == openni::SENSOR_COLOR ? COLOR_FORMAT : DEPTH_FORMAT);
    std::vector<openni::VideoMode> modes;
    auto& available_modes = device.getSensorInfo(type)->getSupportedVideoModes();
    for (int i = 0; i < available_modes.getSize(); ++i)
      if (available_modes[i].getPixelFormat() == PIXEL_FORMAT)
        modes.push_back(available_modes[i]);
    return modes;
  }

  static StreamModes toStreamModes(const std::vector<openni::VideoMode>& modes) {
    StreamModes output;
    output.reserve(modes.size());
    for (const auto& m : modes)
      output.emplace_back(cv::Size(m.getResolutionX(), m.getResolutionY()), m.getFps());
    return output;
  }
};

std::vector<std::string> OpenNI2Grabber::enumerateConnectedDevices() {
  openni::Array<openni::DeviceInfo> d;
  openni::OpenNI::initialize();
  openni::OpenNI::enumerateDevices(&d);
  std::vector<std::string> devices;
  for (int i = 0; i < d.getSize(); ++i)
    devices.emplace_back(d[i].getUri());
  return devices;
}

OpenNI2Grabber::OpenNI2Grabber(const std::string& device_uri) : p(new Impl) {
  auto hash = device_uri.find_first_of("#");
  if (hash != std::string::npos)
    p->open(device_uri.substr(0, hash), Mode::parse(device_uri.substr(hash + 1)));
  else
    p->open(device_uri, Mode());
  if (!tryLoadIntrinsics(p->intrinsics))
    p->intrinsics = Intrinsics::PrimeSense();
}

OpenNI2Grabber::OpenNI2Grabber(Mode mode, const std::string& device_uri) : p(new Impl) {
  auto hash = device_uri.find_first_of("#");
  if (hash != std::string::npos)
    p->open(device_uri.substr(0, hash), mode);
  else
    p->open(device_uri, mode);
  if (!tryLoadIntrinsics(p->intrinsics))
    p->intrinsics = Intrinsics::PrimeSense();
}

OpenNI2Grabber::~OpenNI2Grabber() = default;

Grabber::Timestamp OpenNI2Grabber::grabFrame(cv::OutputArray _color, cv::OutputArray _depth) {
  _color.create(p->color_stream_mode.resolution, CV_8UC3);
  _depth.create(p->depth_stream_mode.resolution, CV_32FC1);
  cv::Mat color = _color.getMat();
  cv::Mat depth = _depth.getMat();

  return p->grabFrame(color, depth);
}

inline bool OpenNI2Grabber::hasMoreFrames() const {
  return p->num_frames == -1 || p->repeat || p->next_frame_index < p->num_frames - 1;
}

int OpenNI2Grabber::getNumberOfFrames() const {
  return p->num_frames;
}

int OpenNI2Grabber::getCurrentFrameIndex() const {
  return p->next_frame_index - 1;
}

Intrinsics OpenNI2Grabber::getCameraIntrinsics() const {
  return p->intrinsics;
}

void OpenNI2Grabber::printInfo(std::ostream& os) const {
  os << "OpenNI2Grabber :: " << p->device.getDeviceInfo().getUri() << std::endl;
  os << " Synchronization: " << (p->device.getDepthColorSyncEnabled() ? "yes" : "no") << std::endl;
  Grabber::printInfo(os);
}

Grabber::StreamModes OpenNI2Grabber::getSupportedColorStreamModes() const {
  return p->toStreamModes(p->getSupportedModes(openni::SENSOR_COLOR));
}

Grabber::StreamModes OpenNI2Grabber::getSupportedDepthStreamModes() const {
  return p->toStreamModes(p->getSupportedModes(openni::SENSOR_DEPTH));
}

Grabber::StreamMode OpenNI2Grabber::getActiveColorStreamMode() const {
  return p->color_stream_mode;
}

Grabber::StreamMode OpenNI2Grabber::getActiveDepthStreamMode() const {
  return p->depth_stream_mode;
}

bool OpenNI2Grabber::isFeatureSupported(Feature feature) const {
  switch (feature) {
    case Feature::SEEK:
      return p->device.isCommandSupported(openni::DEVICE_COMMAND_SEEK);
    case Feature::REPEAT:
      return p->is_file;
    case Feature::AUTO_WHITE_BALANCE:
      return p->color_stream.isPropertySupported(openni::STREAM_PROPERTY_AUTO_WHITE_BALANCE);
    case Feature::AUTO_EXPOSURE:
      return p->color_stream.isPropertySupported(openni::STREAM_PROPERTY_AUTO_EXPOSURE);
    case Feature::EXPOSURE:
      return p->color_stream.isPropertySupported(openni::STREAM_PROPERTY_EXPOSURE);
    case Feature::GAIN:
      return p->color_stream.isPropertySupported(openni::STREAM_PROPERTY_GAIN);
    case Feature::SERIAL_NUMBER:
      return p->device.isPropertySupported(ONI_DEVICE_PROPERTY_SERIAL_NUMBER);
    case Feature::CAMERA_MODEL_NAME:
      return true;
    default:
      return false;
  }
}

void OpenNI2Grabber::seek(unsigned int index) {
  throwIfNotSupported(Feature::SEEK);
  if (static_cast<int>(index) >= p->num_frames)
    BOOST_THROW_EXCEPTION(GrabberException("Attempted to seek beyond the valid frame index"));
  p->device.getPlaybackControl()->seek(p->depth_stream, index);
  p->next_frame_index = index + 1;
}

void OpenNI2Grabber::setRepeatEnabled(bool state) {
  throwIfNotSupported(Feature::REPEAT);
  p->device.getPlaybackControl()->setRepeatEnabled(state);
  p->repeat = state;
}

bool OpenNI2Grabber::getRepeatEnabled() const {
  throwIfNotSupported(Feature::REPEAT);
  return p->repeat;
}

void OpenNI2Grabber::setAutoWhiteBalanceEnabled(bool state) {
  throwIfNotSupported(Feature::AUTO_WHITE_BALANCE);
  p->color_stream.getCameraSettings()->setAutoWhiteBalanceEnabled(state);
}

bool OpenNI2Grabber::getAutoWhiteBalanceEnabled() const {
  throwIfNotSupported(Feature::AUTO_WHITE_BALANCE);
  return p->color_stream.getCameraSettings()->getAutoWhiteBalanceEnabled();
}

void OpenNI2Grabber::setAutoExposureEnabled(bool state) {
  throwIfNotSupported(Feature::AUTO_EXPOSURE);
  p->color_stream.getCameraSettings()->setAutoExposureEnabled(state);
}

bool OpenNI2Grabber::getAutoExposureEnabled() const {
  throwIfNotSupported(Feature::AUTO_EXPOSURE);
  return p->color_stream.getCameraSettings()->getAutoExposureEnabled();
}

void OpenNI2Grabber::setExposure(unsigned int exposure_ms) {
  throwIfNotSupported(Feature::EXPOSURE);
  p->color_stream.getCameraSettings()->setExposure(exposure_ms);
}

unsigned int OpenNI2Grabber::getExposure() const {
  throwIfNotSupported(Feature::EXPOSURE);
  return p->color_stream.getCameraSettings()->getExposure();
}

std::pair<unsigned int, unsigned int> OpenNI2Grabber::getExposureRange() const {
  throwIfNotSupported(Feature::EXPOSURE);
  return {1, 1000};
}

void OpenNI2Grabber::setGain(int gain) {
  throwIfNotSupported(Feature::GAIN);
  p->color_stream.getCameraSettings()->setGain(gain);
}

int OpenNI2Grabber::getGain() const {
  throwIfNotSupported(Feature::GAIN);
  return p->color_stream.getCameraSettings()->getGain();
}

std::string OpenNI2Grabber::getSerialNumber() const {
  throwIfNotSupported(Feature::SERIAL_NUMBER);
  return p->getSerialNumber();
}

std::string OpenNI2Grabber::getCameraModelName() const {
  throwIfNotSupported(Feature::CAMERA_MODEL_NAME);
  return p->getCameraModelName();
}

OpenNI2Grabber::Mode OpenNI2Grabber::Mode::parse(const std::string& mode_string) {
  Mode mode;
  if (mode_string != "" && sscanf(mode_string.c_str(), "%hhu/%hhu", &mode.color, &mode.depth) != 2)
    BOOST_THROW_EXCEPTION(GrabberException("Invalid mode specification"));
  return mode;
}

}  // namespace io
}  // namespace v4r
