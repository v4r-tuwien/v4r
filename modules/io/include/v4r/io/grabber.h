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

#include <cstdint>
#include <memory>
#include <vector>

#include <opencv2/core/core.hpp>

#include <v4r/common/intrinsics.h>
#include <v4r/core/macros.h>
#include <v4r/io/exceptions.h>

namespace v4r {
namespace io {

/** Abstract base class for RGB-D grabbers. */
class V4R_EXPORTS Grabber {
 public:
  using Timestamp = uint64_t;
  using Ptr = std::shared_ptr<Grabber>;

  virtual ~Grabber() = default;

  /// Grab an RGB-D frame.
  /// If the provided arrays are already allocated and have appropriate size/type, then this space will be reused.
  /// Color image will have 8UC3 format with BGR channel order.
  /// Depth image will have 32FC1 format with metric depth values.
  virtual Timestamp grabFrame(cv::OutputArray color, cv::OutputArray depth) = 0;

  /// Check if there are more frames to be grabbed.
  virtual bool hasMoreFrames() const = 0;

  /// Get the number of RGB-D frames that the grabber can emit.
  /// When grabbing from a device this number is unknown, so -1 should be returned.
  virtual int getNumberOfFrames() const = 0;

  /// Get zero-based index of the current frame (the last frame obtained using grabFrame()).
  /// \return -1 before grabFrame() was called for the first time.
  virtual int getCurrentFrameIndex() const = 0;

  /// Get camera intrinsics.
  virtual Intrinsics getCameraIntrinsics() const = 0;

  /// Print information about the grabber (e.g. image resolution, supported features) to the given stream.
  /// Base implementation only prints feature list.
  virtual void printInfo(std::ostream& os) const;

  /// Print information about the grabber (e.g. image resolution, supported features) to the console.
  /// Uses the other printInfo() overload.
  virtual void printInfo() const;

  /// Return true if grabbing from a file.
  bool isFile() const;

  /// Return true if grabbing from a device.
  bool isDevice() const;

  /// Streaming mode
  struct StreamMode {
    StreamMode() = default;
    StreamMode(cv::Size resolution, unsigned int fps);
    bool operator==(const StreamMode& m) const;
    cv::Size resolution = {0, 0};  ///< Image resolution
    unsigned int fps = 0;          ///< Zero means that stream has no associated framerate (typical for file grabbers)
  };

  using StreamModes = std::vector<StreamMode>;

  /// Get the list of supported modes for the color stream.
  virtual StreamModes getSupportedColorStreamModes() const;

  /// Get the list of supported modes for the depth stream.
  virtual StreamModes getSupportedDepthStreamModes() const;

  /// Get the currently active color stream mode.
  /// If there is no color stream (or it is not active), then a mode with zero resolution is returned.
  virtual StreamMode getActiveColorStreamMode() const = 0;

  /// Get the currently active depth stream mode.
  /// If there is no depth stream (or it is not active), then a mode with zero resolution is returned.
  virtual StreamMode getActiveDepthStreamMode() const = 0;

  /// Check if color stream is enabled.
  bool hasColorStream() const;

  /// Check if depth stream is enabled.
  bool hasDepthStream() const;

  /// List of features that may be supported by a grabber.
  enum class Feature {
    SEEK,                ///< Seeking to a frame (only for file-based grabbers)
    REPEAT,              ///< Repeating (only for file-based grabbers)
    AUTO_WHITE_BALANCE,  ///< Built-in AWB
    AUTO_EXPOSURE,       ///< Built-in AEC
    EXPOSURE,            ///< Color camera exposure time setting
    GAIN,                ///< Color camera gain setting
    WHITE_BALANCE,       ///< Color camera white balance setting
    SERIAL_NUMBER,       ///< Device serial number
    CAMERA_MODEL_NAME,   ///< Device camera model
  };

  /// Query if a particular feature is supported.
  virtual bool isFeatureSupported(Feature feature) const;

  /// Seek to the frame with a given index.
  /// Throws GrabberFeatureNotSupportedException if Feature::SEEK is not supported.
  virtual void seek(unsigned int index);

  /// Enable/disable repeating (rewind to the beginning after the last frame is grabbed).
  /// Throws GrabberFeatureNotSupportedException if Feature::REPEAT is not supported.
  virtual void setRepeatEnabled(bool state = true);

  /// Get the state of repeating setting.
  /// Throws GrabberFeatureNotSupportedException if Feature::REPEAT is not supported.
  virtual bool getRepeatEnabled() const;

  /// Enable/disable auto white balance.
  /// Throws GrabberFeatureNotSupportedException if Feature::AUTO_WHITE_BALANCE is not supported.
  virtual void setAutoWhiteBalanceEnabled(bool state = true);

  /// Get the state of auto white balance setting.
  /// Throws GrabberFeatureNotSupportedException if Feature::AUTO_WHITE_BALANCE is not supported.
  virtual bool getAutoWhiteBalanceEnabled() const;

  /// Enable/disable auto exposure controller.
  /// Throws GrabberFeatureNotSupportedException if Feature::AUTO_EXPOSURE is not supported.
  virtual void setAutoExposureEnabled(bool state = true);

  /// Get the state of auto exposure controller setting.
  /// Throws GrabberFeatureNotSupportedException if Feature::AUTO_EXPOSURE is not supported.
  virtual bool getAutoExposureEnabled() const;

  /// Set exposure time (milliseconds).
  /// Throws GrabberFeatureNotSupportedException if Feature::EXPOSURE is not supported.
  virtual void setExposure(unsigned int exposure_ms);

  /// Get exposure time (milliseconds).
  /// Throws GrabberFeatureNotSupportedException if Feature::EXPOSURE is not supported.
  virtual unsigned int getExposure() const;

  /// Get the range of supported exposure times (milliseconds).
  /// Throws GrabberFeatureNotSupportedException if Feature::EXPOSURE is not supported.
  virtual std::pair<unsigned int, unsigned int> getExposureRange() const;

  /// Set camera gain.
  /// Throws GrabberFeatureNotSupportedException if Feature::GAIN is not supported.
  virtual void setGain(int gain);

  /// Get camera gain.
  /// Throws GrabberFeatureNotSupportedException if Feature::GAIN is not supported.
  virtual int getGain() const;

  /// Set white balance.
  /// Throws GrabberFeatureNotSupportedException if Feature::WHITE_BALANCE is not supported.
  virtual void setWhiteBalance(int white_balance);

  /// Get white balance.
  /// Throws GrabberFeatureNotSupportedException if Feature::WHITE_BALANCE is not supported.
  virtual int getWhiteBalance() const;

  /// Get the serial number of the camera.
  /// Throws GrabberFeatureNotSupportedException if Feature::SERIAL_NUMBER is not supported.
  virtual std::string getSerialNumber() const;

  /// Get the model name of the camera (e.g. "ps1080", "sr300").
  /// Throws GrabberFeatureNotSupportedException if Feature::CAMERA_MODEL_NAME is not supported.
  virtual std::string getCameraModelName() const;

  /// Get a unique identifier of the camera (concatenation of model name and serial number).
  /// Throws GrabberFeatureNotSupportedException if Feature::SERIAL_NUMBER or Feature::CAMERA_MODEL_NAME is not
  /// supported.
  std::string getCameraUID() const;

 protected:
  void throwIfNotSupported(Feature feature) const;

  /// Try to load camera intrinsics.
  /// Intrinsics file name should match the camera UID and have one of the formats supported by \ref Intrinsics::load().
  /// The file is searched for in the "calibration" subdirectory of the V4R user data directory, i.e. in
  ///   $XDG_DATA_HOME/v4r/calibration
  bool tryLoadIntrinsics(Intrinsics& intr) const;
};

class GrabberFeatureNotSupportedException : public GrabberException {
 public:
  GrabberFeatureNotSupportedException() : GrabberException("Feature not supported") {}
  using Feature = boost::error_info<struct tag_feature, Grabber::Feature>;
};

/// Create a grabber for a given URI.
///
/// This iterates through all available (i.e. included into the V4R build) grabbers and tries to instantiate them for
/// the given URI. The first succeessfuly instantiated grabber is returned. If no grabber is created, a \c nullptr is
/// returned.
///
/// Note: each grabber has its own URI format, refer to the class documentation for details.
V4R_EXPORTS Grabber::Ptr createGrabber(const std::string& uri = "");

/// From a list of streaming modes select the one that most closely matches a given mode.
///
/// \param[in] exact controls whether the match should be exact
/// \return index of the selected mode or -1 of nothing is selected
V4R_EXPORTS int selectStreamMode(const Grabber::StreamModes& modes, const Grabber::StreamMode& target_mode,
                                 bool exact = false);

}  // namespace io
}  // namespace v4r
