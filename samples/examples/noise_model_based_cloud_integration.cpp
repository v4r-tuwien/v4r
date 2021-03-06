#include <stdio.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

#include <v4r/common/noise_models.h>
#include <v4r/common/normals.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/io/eigen.h>
#include <v4r/io/filesystem.h>
#include <v4r/registration/noise_model_based_cloud_integration.h>

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <glog/logging.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
namespace bf = boost::filesystem;
using namespace v4r;

int main(int argc, const char *argv[]) {
  typedef pcl::PointXYZRGB PointT;
  bf::path test_dir, out_dir;
  std::string view_prefix = "cloud_", obj_indices_prefix = "object_indices_", pose_prefix = "pose_";
  float chop_z = std::numeric_limits<float>::max();
  bool visualize = false, use_object_mask = false, use_pose_file = false, debug = false;

  google::InitGoogleLogging(argv[0]);

  NMBasedCloudIntegrationParameter nm_int_param;
  nm_int_param.min_points_per_voxel_ = 1;
  nm_int_param.octree_resolution_ = 0.002f;

  NguyenNoiseModelParameter nm_param;

  typename v4r::NormalEstimator<PointT>::Ptr normal_estimator_;

  NormalEstimatorType normal_method = NormalEstimatorType::PCL_INTEGRAL_NORMAL;

  po::options_description desc(
      "Noise model based cloud integration\n======================================\n**Allowed options");
  desc.add_options()("help,h", "produce help message");
  desc.add_options()("input_dir,i", po::value<bf::path>(&test_dir)->required(), "directory containing point clouds");
  desc.add_options()(
      "out_dir,o", po::value<bf::path>(&out_dir),
      "output directory where the registered cloud will be stored. If not set, nothing will be written to disk");
  desc.add_options()("view_prefix", po::value<std::string>(&view_prefix)->default_value(view_prefix),
                     "view filename prefix for each point cloud (used when using object mask)");
  desc.add_options()("obj_indices_prefix",
                     po::value<std::string>(&obj_indices_prefix)->default_value(obj_indices_prefix),
                     "filename prefix for each object mask file(used when using object mask)");
  desc.add_options()("pose_prefix", po::value<std::string>(&pose_prefix)->default_value(pose_prefix),
                     "filename prefix for each camera pose (used when using use_pose_file)");
  desc.add_options()("resolution,r",
                     po::value<float>(&nm_int_param.octree_resolution_)->default_value(nm_int_param.octree_resolution_),
                     "");
  desc.add_options()(
      "min_points_per_voxel",
      po::value<size_t>(&nm_int_param.min_points_per_voxel_)->default_value(nm_int_param.min_points_per_voxel_), "")
      //            ("threshold_explained",
      //            po::value<float>(&nm_int_param.threshold_explained_)->default_value(nm_int_param.threshold_explained_),
      //            "")
      ;
  desc.add_options()("use_depth_edges",
                     po::value<bool>(&nm_param.use_depth_edges_)->default_value(nm_param.use_depth_edges_), "");
  desc.add_options()("focal_length,f", po::value<float>(&nm_param.focal_length_)->default_value(nm_param.focal_length_),
                     "");
  desc.add_options()("normal_method,n", po::value<NormalEstimatorType>(&normal_method)->default_value(normal_method),
                     "method used for normal computation");
  desc.add_options()("chop_z,z", po::value<float>(&chop_z)->default_value(chop_z), "cut of distance in m ");
  desc.add_options()("visualize,v", po::bool_switch(&visualize), "turn visualization on");
  desc.add_options()("use_object_mask,m", po::bool_switch(&use_object_mask),
                     "reads mask file and only extracts those indices (only if file exists)");
  desc.add_options()(
      "use_pose_file,p", po::bool_switch(&use_pose_file),
      "reads pose from separate pose file instead of extracting it directly from .pcd file (only if file exists)");
  desc.add_options()("debug,d", po::bool_switch(&debug),
                     "saves debug information (e.g. point properties) if output dir is set");
  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
  std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
  po::store(parsed, vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    to_pass_further.push_back("-h");
  }
  try {
    po::notify(vm);
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
  }

  normal_estimator_ = v4r::initNormalEstimator<PointT>(normal_method, to_pass_further);

  std::vector<std::string> folder_names = io::getFoldersInDirectory(test_dir);

  if (folder_names.empty())
    folder_names.push_back("");

  int vp1, vp2;
  pcl::visualization::PCLVisualizer::Ptr vis;

  for (const std::string &sub_folder : folder_names) {
    const bf::path test_seq = test_dir / sub_folder;
    std::vector<std::string> views = io::getFilesInDirectory(test_seq, ".*.pcd", false);

    pcl::PointCloud<PointT>::Ptr big_cloud_unfiltered(new pcl::PointCloud<PointT>);
    std::vector<pcl::PointCloud<PointT>::ConstPtr> clouds(views.size());
    std::vector<pcl::PointCloud<pcl::Normal>::ConstPtr> normals(views.size());
    std::vector<std::vector<std::vector<float>>> pt_properties(views.size());
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> camera_transforms(views.size());
    std::vector<std::vector<int>> obj_indices(views.size());

    for (size_t v_id = 0; v_id < views.size(); v_id++) {
      std::stringstream txt;
      txt << "processing view " << v_id;
      pcl::ScopeTime t(txt.str().c_str());
      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
      pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);

      pcl::io::loadPCDFile((test_seq / views[v_id]).string(), *cloud);

      std::string obj_fn(views[v_id]);
      boost::replace_first(obj_fn, view_prefix, obj_indices_prefix);
      boost::replace_last(obj_fn, ".pcd", ".txt");

      if (use_object_mask) {
        bf::path object_indices_fn = test_seq;
        object_indices_fn /= obj_fn;
        if (io::existsFile(object_indices_fn.string())) {
          ifstream f((test_seq / obj_fn).string().c_str());
          int idx;
          while (f >> idx)
            obj_indices[v_id].push_back(idx);
          f.close();
        } else
          std::cerr << "Parameter use_object_mask is set but object indices file " << object_indices_fn.string()
                    << " not found! " << std::endl;
      }

      std::string pose_fn(views[v_id]);
      boost::replace_first(pose_fn, view_prefix, pose_prefix);
      boost::replace_last(pose_fn, ".pcd", ".txt");

      if (io::existsFile(test_seq / pose_fn) && use_pose_file)
        camera_transforms[v_id] = io::readMatrixFromFile(test_seq / pose_fn);
      else
        camera_transforms[v_id] = RotTrans2Mat4f(cloud->sensor_orientation_, cloud->sensor_origin_);

      // reset view point otherwise pcl visualization is potentially messed up
      cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
      cloud->sensor_origin_ = Eigen::Vector4f::Zero();

      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.f, chop_z);
      pass.setKeepOrganized(true);
      pass.filter(*cloud);

      {
        pcl::ScopeTime tt("Computing normals");
        normal_estimator_->setInputCloud(cloud);
        normal = normal_estimator_->compute();
      }

      {
        pcl::ScopeTime tt("Computing noise model parameter for cloud");
        NguyenNoiseModel<PointT> nm(nm_param);
        nm.setInputCloud(cloud);
        nm.setInputNormals(normal);
        nm.compute();
        pt_properties[v_id] = nm.getPointProperties();
      }

      pcl::PointCloud<PointT> object_cloud, object_aligned;
      if (use_object_mask)
        pcl::copyPointCloud(*cloud, obj_indices[v_id], object_cloud);
      else
        pcl::copyPointCloud(*cloud, object_cloud);

      pcl::transformPointCloud(object_cloud, object_aligned, camera_transforms[v_id]);
      *big_cloud_unfiltered += object_aligned;

      clouds[v_id] = cloud;
      normals[v_id] = normal;
    }

    pcl::PointCloud<PointT>::Ptr octree_cloud(new pcl::PointCloud<PointT>);
    NMBasedCloudIntegration<PointT> nmIntegration(nm_int_param);
    nmIntegration.setInputClouds(clouds);
    nmIntegration.setPointProperties(pt_properties);
    nmIntegration.setTransformations(camera_transforms);
    nmIntegration.setInputNormals(normals);
    nmIntegration.setIndices(obj_indices);
    nmIntegration.compute(octree_cloud);
    std::vector<pcl::PointCloud<PointT>::Ptr> clouds_used;
    nmIntegration.getInputCloudsUsed(clouds_used);

    std::cout << "Size cloud unfiltered: " << big_cloud_unfiltered->points.size()
              << ", filtered: " << octree_cloud->points.size() << std::endl;

    if (vm.count("out_dir")) {
      const bf::path out_path = out_dir / sub_folder;
      io::createDirIfNotExist(out_path);
      pcl::io::savePCDFileBinaryCompressed((out_path / "/registered_cloud_filtered.pcd").string(), *octree_cloud);
      pcl::io::savePCDFileBinaryCompressed((out_path / "/registered_cloud_unfiltered.pcd").string(),
                                           *big_cloud_unfiltered);

      for (size_t v_id = 0; v_id < clouds_used.size(); v_id++) {
        if (debug) {
          std::stringstream fn;
          fn << out_path.string() << "/filtered_input_" << setfill('0') << setw(5) << v_id << ".pcd";
          pcl::io::savePCDFileBinaryCompressed(fn.str(), *clouds_used[v_id]);

          fn.str("");
          fn << out_path.string() << "/distance_to_edge_px_" << setfill('0') << setw(5) << v_id << ".txt";
          std::ofstream f(fn.str());
          for (size_t pt_id = 0; pt_id < clouds_used[v_id]->points.size(); pt_id++) {
            f << pt_properties[v_id][pt_id][2] << std::endl;

            PointT &pt_tmp = clouds_used[v_id]->points[pt_id];
            if (!pcl::isFinite(pt_tmp))  // set background color for nan points
              pt_tmp.r = pt_tmp.g = pt_tmp.b = 255.f;
          }
          f.close();

          fn.str("");
          fn << out_path.string() << "/filter_input_image" << setfill('0') << setw(5) << v_id << ".png";
          PCLOpenCVConverter<PointT> pcl_opencv_converter;
          pcl_opencv_converter.setInputCloud(clouds_used[v_id]);
          cv::imwrite(fn.str(), pcl_opencv_converter.getRGBImage());
        }
      }
    }

    if (visualize) {
      if (!vis) {
        vis.reset(new pcl::visualization::PCLVisualizer("registered cloud"));
        vis->createViewPort(0, 0, 0.5, 1, vp1);
        vis->createViewPort(0.5, 0, 1, 1, vp2);
      }
      vis->removeAllPointClouds(vp1);
      vis->removeAllPointClouds(vp2);
      vis->addPointCloud(big_cloud_unfiltered, "unfiltered_cloud", vp1);
      vis->addPointCloud(octree_cloud, "filtered_cloud", vp2);
      vis->spin();
    }
  }
}
