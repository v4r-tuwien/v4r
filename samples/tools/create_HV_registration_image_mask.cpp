#include <pcl/io/pcd_io.h>
#include <v4r/io/filesystem.h>
#include <opencv2/opencv.hpp>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char **argv) {
  typedef pcl::PointXYZRGB PointT;
  bf::path input_dir;
  bf::path output_file = "/tmp/depth_mask.png";
  bool search_recursive = false;

  po::options_description desc(
      "Tool to compute the image mask from a set of point clouds. The mask shows which points of a registered point"
      "cloud are visible\n======================================\n**Allowed options");
  desc.add_options()("help,h", "produce help message");
  desc.add_options()("input_dir,i", po::value<bf::path>(&input_dir)->required(),
                     "input directory containing the .pcd files");
  desc.add_options()("output_file,o", po::value<bf::path>(&output_file)->default_value(output_file),
                     "path to where the image mask (.png) will be saved to.");
  desc.add_options()("recursive_file_search, r", po::value<bool>(&search_recursive)->default_value(search_recursive),
                     "include sub-folders for input");

  po::positional_options_description p;
  p.add("input_dir", 1);
  p.add("output_file", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return false;
  }

  try {
    po::notify(vm);
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }

  std::vector<std::string> sub_folder_names = v4r::io::getFilesInDirectory(input_dir, ".*.pcd", search_recursive);

  cv::Mat_<unsigned char> img_mask;

  for (const std::string &basename : sub_folder_names) {
    bf::path input_fn = input_dir / basename;

    std::cout << "Loading image " << input_fn.string() << std::endl;
    pcl::PointCloud<PointT> cloud;
    pcl::io::loadPCDFile(input_fn.string(), cloud);

    if (!img_mask.data) {
      img_mask = cv::Mat_<unsigned char>(cloud.height, cloud.width);
      img_mask.setTo(0);
    }

    for (size_t u = 0; u < cloud.width; u++) {
      for (size_t v = 0; v < cloud.height; v++) {
        if (pcl::isFinite(cloud.at(u, v)))
          img_mask.at<unsigned char>(v, u) = 255;
      }
    }
  }

  cv::imwrite(output_file.string(), img_mask);
}
