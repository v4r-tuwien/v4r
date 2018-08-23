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
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include <StructureCore.h>
#include <StructureCoreUtils.h>

#include <boost/algorithm/string.hpp>
#include <boost/throw_exception.hpp>

#include <v4r/io/structure_core_grabber.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace v4r {
namespace io {

struct StructureCoreGrabber::Impl {
  const float DISPARITY_THRESHOLD = 20;
  const unsigned int CONFIDENCE_THRESHOLD = 8;

  /// (Hard-coded) list of supported modes.
  StreamModes DEPTH_STREAM_MODES;
  StreamModes COLOR_STREAM_MODES;

  StreamMode color_stream_mode;
  StreamMode depth_stream_mode;
  bool depth_available = false;
  bool color_available = false;
  Intrinsics depth_intrinsics;
  Intrinsics color_intrinsics;
  std::vector<float> distortion_coeff;
  // Rectification mapping
  cv::Mat map1, map2;

  // Extrinsics (color -> depth)
  std::vector<float> translation;
  std::vector<float> rotation;

  int next_frame_index = 0;
  SCConnectionState device_state = SCConnectionState_Disconnected;

  cv::Mat color_frame;
  bool have_new_color_frame = false;
  cv::Mat depth_frame;
  bool have_new_depth_frame = false;
  Timestamp timestamp;

  SCDeviceInfo device_info;

  std::mutex frame_mutex;

  Impl() {
    COLOR_STREAM_MODES.emplace_back(cv::Size(640, 480), 30);
    DEPTH_STREAM_MODES.emplace_back(cv::Size(608, 456), 30);
    DEPTH_STREAM_MODES.emplace_back(cv::Size(1216, 912), 30);
  }

  void open(const std::string& device_name, Mode mode) {
    SCInitParams params;
    std::memset(&params, 0, sizeof(params));
    if (!device_name.empty() && device_name != "structure")
      params.serial = device_name.c_str();

    StructureCore_Init(deviceStateCallback, this, &params);

    if (mode.color == Mode::DISABLE && mode.depth == Mode::DISABLE)
      BOOST_THROW_EXCEPTION(GrabberException("At least one of the streams should be enabled"));

    SCStreamingConfig config;
    std::memset(&config, 0, sizeof(config));

    if (mode.depth != Mode::DISABLE) {
      if (mode.depth == Mode::DEFAULT)
        mode.depth = 2;
      if (mode.depth > DEPTH_STREAM_MODES.size())
        BOOST_THROW_EXCEPTION(GrabberException("Invalid depth mode"));
      config.depth.resolution = mode.depth == 2 ? SCResolution_Full : SCResolution_VGA;
      config.depth.fps = 30;
      config.depthOptimization = true;
      config.depth.cb = depthFrameCallback;
      config.depth.ctx = this;

      // These are somewhat random values, feel free to change
      config.camFeatures[config.numCamFeatures++] = {SCCameraSource_IRBoth, SCCameraFeature_AnalogGain, 1, {4.5, 0.}};
      config.camFeatures[config.numCamFeatures++] = {SCCameraSource_IRBoth, SCCameraFeature_Exposure, 1, {0.01, 0.}};

      depth_stream_mode = DEPTH_STREAM_MODES[mode.depth - 1];
      depth_available = true;
    }

    if (mode.color != Mode::DISABLE) {
      config.visible.fps = 30;
      config.visible.resolution = SCResolution_Full;
      config.visible.cb = visibleFrameCallback;
      config.visible.ctx = this;
      color_stream_mode = COLOR_STREAM_MODES[0];
      color_available = true;
    }

    while (device_state != SCConnectionState_Ready) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto status = StructureCore_StartStreamingFrames(&config);
    if (status != SCStatus_Success)
      BOOST_THROW_EXCEPTION(GrabberException("Failed to start streaming")
                            << GrabberException::ErrorInfo(getStatusString(status)));

    StructureCore_SetDeviceLogParams(SCLogLevel_Error, false);

    // The values stored in depth_intrirsics are always for full resolution.
    // If the depth stream is VGA, then cut intrinsics in half.
    // This may change in future when we have more streams.
    if (mode.depth == 1)
      depth_intrinsics = depth_intrinsics(1);
  }

  ~Impl() {
    StructureCore_StopStreamingFrames();
    StructureCore_Deinit();
  }

  static void depthFrameCallback(const SCFrameData* frame_data, const SCCalibrationData* calibration, void* context) {
    reinterpret_cast<StructureCoreGrabber::Impl*>(context)->depthFrameCallback(frame_data, calibration);
  }

  void depthFrameCallback(const SCFrameData* frame_data, const SCCalibrationData* calibration) {
    VLOG(2) << "Depth frame received (" << frame_data->width << "x" << frame_data->height << ")";
    VLOG(2) << "Min disparity: " << frame_data->meta.depth.minDisparity
            << ", max disparity: " << frame_data->meta.depth.maxDisparity;

    float fx = depth_intrinsics.fx;
    float fx_inv = 1.0f / fx;
    float baseline = calibration->rectifiedIR.baselineMM;
    std::lock_guard<std::mutex> guard(frame_mutex);
    // Depth frame has the same size as color image because we are projecting onto color image plane
    depth_frame.create(color_intrinsics.h, color_intrinsics.w, CV_32FC1);
    depth_frame.setTo(0);
    auto ptr = frame_data->contents;
    for (size_t y = 0; y < frame_data->height; ++y) {
      for (size_t x = 0; x < frame_data->width; ++x, ++ptr) {
        auto disparity = *ptr;
        uint16_t confidence = (disparity & 0b1111);
        uint16_t fractional_disparity = (disparity >> 4) & 0b1111;
        float disparity_value = (disparity >> 8) + (fractional_disparity / 16.0f);
        if (disparity_value > DISPARITY_THRESHOLD && confidence > CONFIDENCE_THRESHOLD) {
          disparity_value += frame_data->meta.depth.minDisparity;
          float depth_value = fx * baseline / disparity_value * 0.001f;
          auto x3d = (static_cast<float>(x) - depth_intrinsics.cx) * depth_value * fx_inv;
          auto y3d = (static_cast<float>(y) - depth_intrinsics.cy) * depth_value * fx_inv;
          auto z = depth_value + translation[2];
          auto u = (x3d + translation[0]) * color_intrinsics.fx / z + color_intrinsics.cx;
          auto v = (y3d + translation[1]) * color_intrinsics.fy / z + color_intrinsics.cy;
          if (u >= 0 && v >= 0 && u < color_intrinsics.w && v < color_intrinsics.h)
            depth_frame.at<float>(v, u) = z;
        }
      }
    }

    have_new_depth_frame = true;
    timestamp = getTimestamp();
  }

  static void visibleFrameCallback(const SCFrameData* frame_data, const SCCalibrationData* calibration, void* context) {
    reinterpret_cast<StructureCoreGrabber::Impl*>(context)->visibleFrameCallback(frame_data, calibration);
  }

  void visibleFrameCallback(const SCFrameData* frame_data, const SCCalibrationData* /* calibration */) {
    VLOG(2) << "Visible frame received (" << frame_data->width << "x" << frame_data->height << ")";
    VLOG(2) << "Exposure: " << frame_data->meta.visible.exposure
            << ", analog gain: " << frame_data->meta.visible.analogGain
            << ", digital gain: " << frame_data->meta.visible.digitalGain;
    std::lock_guard<std::mutex> guard(frame_mutex);
    color_frame.create(frame_data->height, frame_data->width, CV_8UC3);
    auto ptr = frame_data->contents;
    for (size_t y = 0; y < frame_data->height; ++y) {
      for (size_t x = 0; x < frame_data->width; ++x, ++ptr) {
        uint8_t b = *ptr >> 2;
        color_frame.at<cv::Vec3b>(y, x) = {b, b, b};
      }
    }
    cv::Mat out;
    cv::remap(color_frame, out, map1, map2, cv::INTER_AREA);
    std::swap(out, color_frame);

    have_new_color_frame = true;
    timestamp = getTimestamp();
  }

  static void deviceStateCallback(SCConnectionState state, void* context) {
    reinterpret_cast<StructureCoreGrabber::Impl*>(context)->deviceStateCallback(state);
  }

  void deviceStateCallback(SCConnectionState state) {
    VLOG(1) << "Device state: " << StructureCore_SCConnectionStateStr(state);
    device_state = state;
    if (state == SCConnectionState_Ready) {
      auto status = StructureCore_GetDeviceInfo(&device_info);
      if (status != SCStatus_Success)
        BOOST_THROW_EXCEPTION(GrabberException("Failed to get device info")
                              << GrabberException::ErrorInfo(getStatusString(status)));
      SCCalibrationData calibration;
      std::memset(&calibration, 0, sizeof(calibration));
      status = StructureCore_GetCalibrationData(&calibration);
      if (status != SCStatus_Success)
        BOOST_THROW_EXCEPTION(GrabberException("Failed to get device calibration data")
                              << GrabberException::ErrorInfo(getStatusString(status)));

      color_intrinsics.fx = calibration.visible.fc_x;
      color_intrinsics.fy = calibration.visible.fc_y;
      color_intrinsics.cx = calibration.visible.cc_x;
      color_intrinsics.cy = calibration.visible.cc_y;
      color_intrinsics.w = color_stream_mode.resolution.width;
      color_intrinsics.h = color_stream_mode.resolution.height;
      VLOG(1) << "Color intrinsics: " << color_intrinsics;

      depth_intrinsics.fx = calibration.rectifiedIR.fc_right;
      depth_intrinsics.fy = calibration.rectifiedIR.fc_right;
      depth_intrinsics.cx = calibration.rectifiedIR.cc_right_x;
      depth_intrinsics.cy = calibration.rectifiedIR.cc_right_y;
      depth_intrinsics.w = DEPTH_STREAM_MODES[1].resolution.width;
      depth_intrinsics.h = DEPTH_STREAM_MODES[1].resolution.height;
      VLOG(1) << "Depth intrinsics: " << depth_intrinsics;

      VLOG(1) << "Extrinsics: T " << calibration.visible.transl_x << " " << calibration.visible.transl_y << " "
              << calibration.visible.transl_z << ", R " << calibration.visible.rotat_x << " "
              << calibration.visible.rotat_y << " " << calibration.visible.rotat_z;
      VLOG(1) << "K " << calibration.visible.k1 << " " << calibration.visible.k2 << " " << calibration.visible.k3 << " "
              << calibration.visible.k4 << " " << calibration.visible.k5;
      VLOG(1) << "P " << calibration.visible.p1 << " " << calibration.visible.p2 << "  D " << calibration.visible.du
              << " " << calibration.visible.dv << " N " << calibration.visible.nx << " " << calibration.visible.ny;

      translation = {calibration.visible.transl_x, calibration.visible.transl_y, calibration.visible.transl_z};
      rotation = {calibration.visible.rotat_x, calibration.visible.rotat_y, calibration.visible.rotat_z};
      distortion_coeff = {calibration.visible.k1, calibration.visible.k2, calibration.visible.p1,
                          calibration.visible.p2, calibration.visible.k3};

      // Create rotation matrix from Rodriguez vector of inversed transform
      cv::Mat R;
      cv::Rodrigues(rotation, R);
      cv::invert(R, R);

      // Preserve color camera matrix
      cv::Mat camera_matrix = color_intrinsics.getCameraMatrix();

      // Change focal length such that we "zoom in" color image a bit
      color_intrinsics.fx += 160;
      color_intrinsics.fy += 160;

      // Create undistortion mappings
      cv::initUndistortRectifyMap(camera_matrix, distortion_coeff, R, color_intrinsics.getCameraMatrix(),
                                  {color_intrinsics.w, color_intrinsics.h}, CV_16SC2, map1, map2);
    }
  }

  Timestamp grabFrame(cv::OutputArray color, cv::OutputArray depth) {
    std::lock_guard<std::mutex> guard(frame_mutex);

    if (color_available == have_new_color_frame && depth_available == have_new_depth_frame)
      ++next_frame_index;

    if (color_available) {
      color_frame.copyTo(color);
      have_new_color_frame = false;
    }

    if (depth_available) {
      depth_frame.copyTo(depth);
      have_new_depth_frame = false;
    }

    return timestamp;
  }

  bool hasMoreFrames() {
    return color_available == have_new_color_frame && depth_available == have_new_depth_frame;
  }

  static Timestamp getTimestamp() {
    auto now = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
    return us;
  }

  static std::string getStatusString(SCStatus status) {
    static const std::vector<std::string> TEXT = {
        "Success",
        "Error",
        "Timeout",
        "InvalidParameter",
        "ChecksumInvalid",
        "DeviceNotReady",
        "InvalidConfiguration",
        "NotSupported",
    };
    return TEXT.at(status);
  }
};

std::vector<std::string> StructureCoreGrabber::enumerateConnectedDevices() {
  unsigned int num_devices = 0;
  std::vector<SCUQueryResults> results(10);
  StructureCoreUtils_QueryAttachedDevices(results.data(), results.size(), &num_devices);
  std::vector<std::string> devices;
  for (unsigned int i = 0; i < num_devices; ++i)
    devices.emplace_back(results[i].serial);
  return devices;
}

StructureCoreGrabber::StructureCoreGrabber(const std::string& device_uri) : p(new Impl) {
  auto hash = device_uri.find_first_of("#");
  if (hash != std::string::npos)
    p->open(device_uri.substr(0, hash), Mode::parse(device_uri.substr(hash + 1)));
  else
    p->open(device_uri, Mode());
}

StructureCoreGrabber::StructureCoreGrabber(Mode mode, const std::string& device_uri) : p(new Impl) {
  auto hash = device_uri.find_first_of("#");
  if (hash != std::string::npos)
    p->open(device_uri.substr(0, hash), mode);
  else
    p->open(device_uri, mode);
}

StructureCoreGrabber::~StructureCoreGrabber() = default;

Grabber::Timestamp StructureCoreGrabber::grabFrame(cv::OutputArray _color, cv::OutputArray _depth) {
  return p->grabFrame(_color, _depth);
}

inline bool StructureCoreGrabber::hasMoreFrames() const {
  return p->hasMoreFrames();
  // return p->num_frames == -1 || p->repeat || p->next_frame_index < p->num_frames - 1;
}

int StructureCoreGrabber::getNumberOfFrames() const {
  return -1;
}

int StructureCoreGrabber::getCurrentFrameIndex() const {
  return p->next_frame_index - 1;
}

Intrinsics StructureCoreGrabber::getCameraIntrinsics() const {
  return p->color_intrinsics;
}

void StructureCoreGrabber::printInfo(std::ostream& os) const {
  os << "StructureCoreGrabber" << std::endl;
  os << " Version: " << p->device_info.version.major << "." << p->device_info.version.minor << "."
     << p->device_info.version.revision << std::endl;
  os << " Hardware ID: " << p->device_info.hwId << std::endl;
  Grabber::printInfo(os);
}

Grabber::StreamModes StructureCoreGrabber::getSupportedColorStreamModes() const {
  return p->COLOR_STREAM_MODES;
}

Grabber::StreamModes StructureCoreGrabber::getSupportedDepthStreamModes() const {
  return p->DEPTH_STREAM_MODES;
}

Grabber::StreamMode StructureCoreGrabber::getActiveColorStreamMode() const {
  return p->color_stream_mode;
}

Grabber::StreamMode StructureCoreGrabber::getActiveDepthStreamMode() const {
  return p->depth_stream_mode;
}

bool StructureCoreGrabber::isFeatureSupported(Feature feature) const {
  switch (feature) {
    case Feature::SERIAL_NUMBER:
      return true;
    case Feature::CAMERA_MODEL_NAME:
      return true;
    default:
      return false;
  }
}

std::string StructureCoreGrabber::getSerialNumber() const {
  return p->device_info.serial;
}

std::string StructureCoreGrabber::getCameraModelName() const {
  std::string name = p->device_info.model;
  boost::algorithm::to_lower(name);
  return name;
}

StructureCoreGrabber::Mode StructureCoreGrabber::Mode::parse(const std::string& mode_string) {
  Mode mode;
  if (mode_string != "" && sscanf(mode_string.c_str(), "%hhu/%hhu", &mode.color, &mode.depth) != 2)
    BOOST_THROW_EXCEPTION(GrabberException("Invalid mode specification"));
  return mode;
}

}  // namespace io
}  // namespace v4r
