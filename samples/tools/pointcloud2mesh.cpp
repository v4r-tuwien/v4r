#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <boost/program_options.hpp>
#include <v4r/camera_tracking_and_mapping/PoissonTriangulation.hh>

namespace bf = boost::filesystem;
namespace po = boost::program_options;

int main(int argc, char **argv) {
  typedef pcl::PointXYZRGBNormal PointT;
  bf::path input_file, out_dir = ".";
  int depth = 7;
  int samplesPerNode = 2;
  bool cropModel = true;

  po::options_description desc(
      "Generate mesh file from point cloud\n======================================\n**Allowed options");
  desc.add_options()("help,h", "produce help message");
  desc.add_options()("input_file,i", po::value<bf::path>(&input_file)->required(), "File path to input point cloud.");
  desc.add_options()("out_dir,o", po::value<bf::path>(&out_dir)->default_value(out_dir), "Output directory.");
  desc.add_options()("depth,d", po::value<int>(&depth)->default_value(depth), "");
  desc.add_options()("crop,c", po::value<bool>(&cropModel)->default_value(depth), "crop model");
  desc.add_options()("samples,s", po::value<bool>(&cropModel)->default_value(depth), "samples per node");
  po::positional_options_description p;
  p.add("input_file", 1);

  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).positional(p).run();
  po::store(parsed, vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
  }
  try {
    po::notify(vm);
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
  }

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::io::loadPCDFile(input_file.string(), *cloud);
  v4r::PoissonTriangulation pt(depth, samplesPerNode, cropModel);
  pcl::PolygonMesh mesh;
  pt.reconstruct(cloud, mesh);
  pcl::io::savePLYFile((out_dir / "mesh.ply").string(), mesh);
}