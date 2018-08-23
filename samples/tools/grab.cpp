#include <iostream>

#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <v4r/common/unprojection.h>
#include <v4r/io/grabber.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "program_options.h"

struct ProgramOptions : public ProgramOptionsBase {
  std::string source = "";
  bool print_info = false;
  bool three_dee = false;
  bool overlay = false;

  void printDescription() override {
    std::cout << "Grab RGB-D stream from a given source (camera or file) and visualize." << std::endl;
    std::cout << std::endl;
    std::cout << "On startup this tool will try to instantiate each grabber (available in the current V4R built), "
              << "passing it the provided source URI. The first successfully instantiated grabber will be used. "
              << "Refer to the grabber documentation regarding the supported URI formats. Note that an empty string is "
              << "a valid source URI that means \"any camera\"." << std::endl;
    std::cout << std::endl;
    std::cout << "How the RGB-D stream is visualized is controlled by the following options. By default (without any "
              << "options), color and depth streams are shown in separate windows. With the --overlay option, the "
              << "depth image will be overlaid on the color image. With the --3d option, the depth image will be "
              << "unprojected into 3D space and visualized as a point cloud." << std::endl;
    std::cout << std::endl;
    std::cout << "The --3d visualization mode supports saving unprojected point clouds to the disk by pressing \"s\" "
              << "(while the focus is on the visualization window)." << std::endl;
  }

  void addGeneral(po::options_description& desc) override {
    desc.add_options()("print-info,i", po::bool_switch(&print_info), "Print information about created grabber");
    desc.add_options()("3d", po::bool_switch(&three_dee), "Unproject RGB-D frames and visualize point cloud");
    desc.add_options()("overlay", po::bool_switch(&overlay), "Overlay depth image on color image");
  }

  void addPositional(po::options_description& desc, po::positional_options_description& positional) override {
    desc.add_options()("source-uri", po::value<std::string>(&source), "RGB-D stream source URI");
    positional.add("source-uri", -1);
  }
};

int main(int argc, const char** argv) {
  ProgramOptions options;
  if (!options.parse(argc, argv))
    return 1;

  auto grabber = v4r::io::createGrabber(options.source);
  if (!grabber) {
    std::cerr << "Failed to create a grabber for URI: \"" << options.source << "\"" << std::endl;
    return 2;
  }

  if (options.print_info) {
    grabber->printInfo();
    std::cout << "Intrinsics: " << grabber->getCameraIntrinsics() << std::endl;
  }

  if (options.three_dee && !grabber->hasDepthStream()) {
    std::cerr << "The grabber does not have a depth stream, impossible to visualize as a point cloud" << std::endl;
    return 3;
  }

  if (options.three_dee && options.overlay) {
    std::cerr << "Options --3d and --overlay are exclusive, only one of them can be supplied" << std::endl;
    return 4;
  }

  if (options.overlay && !(grabber->hasColorStream() && grabber->hasDepthStream())) {
    std::cerr << "The grabber should have both color and depth streams to visualize with overlaying" << std::endl;
    return 5;
  }

  cv::Mat color, depth;
  v4r::io::Grabber::Timestamp timestamp;

  if (options.three_dee) {
    pcl::visualization::PCLVisualizer visualizer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler(cloud);
    auto keyboardCallback = [&](const pcl::visualization::KeyboardEvent& event) {
      if (event.getKeySym() == "s" && event.keyUp()) {
        std::string p = boost::str(boost::format("cloud_%08d.pcd") % timestamp);
        std::cout << "Saving unprojected point cloud to \"" << p << "\"" << std::endl;
        pcl::io::savePCDFileBinaryCompressed(p, *cloud);
      }
    };
    visualizer.addPointCloud<pcl::PointXYZRGB>(cloud, handler);
    visualizer.registerKeyboardCallback(keyboardCallback);
    while (grabber->hasMoreFrames() && !visualizer.wasStopped()) {
      timestamp = grabber->grabFrame(color, depth);
      v4r::unproject(color, depth, grabber->getCameraIntrinsics(), *cloud);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler(cloud);
      visualizer.updatePointCloud<pcl::PointXYZRGB>(cloud, handler);
      visualizer.spinOnce(30);
    }
  } else {
    const auto min_depth = 0.2f;
    const auto max_depth = 3.4f;
    const float scale = 255.0 / (max_depth - min_depth);
    cv::Mat depth_u8, colormapped, mask;
    int key = 0;
    while (key != 27) {
      if (grabber->hasMoreFrames())
        timestamp = grabber->grabFrame(color, depth);
      else
        continue;
      if (!color.empty())
        if (!options.overlay)
          cv::imshow("Color", color);
      if (!depth.empty()) {
        depth.convertTo(depth_u8, CV_8UC1, scale, -scale * min_depth);
#if CV_MAJOR_VERSION >= 3
        cv::applyColorMap(depth_u8, colormapped, cv::COLORMAP_PARULA);
#else
        colormapped = depth_u8;
#endif
        if (!options.overlay)
          cv::imshow("Depth", colormapped);
      }
      if (options.overlay) {
        cv::threshold(depth, mask, 0, 255, cv::THRESH_BINARY);
        mask.convertTo(mask, CV_8UC1);
        colormapped.copyTo(color, mask);
        cv::imshow("Overlaid", color);
      }
      key = cv::waitKey(30);
    }
  }

  return 0;
}
