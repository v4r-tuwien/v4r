#include <glog/logging.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <v4r/common/intrinsics.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/common/pcl_utils.h>
#include <v4r/config.h>
#include <v4r/features/FeatureDetector_KD_SIFTGPU.h>
#include <v4r/io/eigen.h>
#include <v4r/io/filesystem.h>
#include <v4r/registration/FeatureBasedRegistration.h>
#include <v4r/registration/fast_icp_with_gc.h>
#include <boost/filesystem.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/program_options.hpp>

namespace bf = boost::filesystem;
namespace po = boost::program_options;

namespace std {
std::ostream &operator<<(std::ostream &, const std::vector<float> &);

std::ostream &operator<<(std::ostream &os, const std::vector<float> &vec) {
  for (auto item : vec) {
    os << item << " ";
  }
  return os;
}
}  // namespace std

struct CamConnect {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix4f transformation_;
  float edge_weight;
  size_t source_id_, target_id_;

  CamConnect() : edge_weight(0.f) {}

  bool operator<(const CamConnect &e) const {
    return edge_weight < e.edge_weight;
  }

  bool operator<=(const CamConnect &e) const {
    return edge_weight <= e.edge_weight;
  }

  bool operator>(const CamConnect &e) const {
    return edge_weight > e.edge_weight;
  }

  bool operator>=(const CamConnect &e) const {
    return edge_weight >= e.edge_weight;
  }
};

struct View {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef pcl::PointXYZRGB PointT;
  cv::Mat descriptors_;
  std::vector<int> keypoint_indices_;
  Eigen::Matrix4f camera_pose_;
  pcl::PointCloud<PointT>::ConstPtr cloud_;
};

class ViewRegistration {
 private:
  typedef pcl::PointXYZRGB PointT;
  std::vector<View, Eigen::aligned_allocator<View>> grph_;

  typedef boost::property<boost::edge_weight_t, CamConnect> EdgeWeightProperty;
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, size_t, EdgeWeightProperty> Graph;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;
  typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
  typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;

  Graph gs_;

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> absolute_poses_;

  v4r::Intrinsics cam_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ViewRegistration() {}

  void calcEdgeWeightAndRefineTf(const typename pcl::PointCloud<PointT>::ConstPtr &cloud_src,
                                 const typename pcl::PointCloud<PointT>::ConstPtr &cloud_dst,
                                 const Eigen::Matrix4f &transform, float &registration_quality,
                                 Eigen::Matrix4f &refined_transform) {
    const float best_overlap_ = 0.75f;

    v4r::FastIterativeClosestPointWithGC<PointT> icp;

    bool use_normals = true;
    // compute normals
    if (use_normals) {
      pcl::PointCloud<pcl::Normal>::Ptr src_normals(new pcl::PointCloud<pcl::Normal>);
      pcl::PointCloud<pcl::Normal>::Ptr dst_normals(new pcl::PointCloud<pcl::Normal>);
      pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> normal_estimation;
      normal_estimation.setNormalEstimationMethod(
          pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::AVERAGE_3D_GRADIENT);
      normal_estimation.setNormalSmoothingSize(10.0);
      normal_estimation.setBorderPolicy(pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::BORDER_POLICY_MIRROR);
      normal_estimation.setInputCloud(cloud_src);
      normal_estimation.compute(*src_normals);
      normal_estimation.setInputCloud(cloud_dst);
      normal_estimation.compute(*dst_normals);
      icp.setSourceNormals(src_normals);
      icp.setTargetNormals(dst_normals);
    }

    icp.setCameraIntrinsics(cam_);
    icp.setMaxCorrespondenceDistance(0.02f);
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_dst);
    icp.useStandardCG(true);
    icp.setNoCG(true);
    icp.setOverlapPercentage(best_overlap_);
    icp.setKeepMaxHypotheses(5);
    icp.setMaximumIterations(10);
    icp.align(transform);
    float w_after_icp_ = icp.getFinalTransformation(refined_transform);

    if (w_after_icp_ < 0 || !pcl_isfinite(w_after_icp_))
      w_after_icp_ = std::numeric_limits<float>::max();
    else
      w_after_icp_ = best_overlap_ - w_after_icp_;

    //    transform = icp_trans; // refined transformation
    registration_quality = w_after_icp_;
  }

  void computeAbsolutePosesRecursive(
      const Graph &grph, const Vertex start, const Eigen::Matrix4f &accum,
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &absolute_poses,
      std::vector<bool> &hop_list) {
    boost::property_map<Graph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, gs_);
    boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::out_edges(start, grph); ei != ei_end; ++ei) {
      Vertex targ = boost::target(*ei, grph);
      size_t target_id = boost::target(*ei, grph);

      if (hop_list[target_id])
        continue;

      hop_list[target_id] = true;
      CamConnect my_e = weightmap[*ei];
      Eigen::Matrix4f intern_accum;
      Eigen::Matrix4f trans = my_e.transformation_;
      if (my_e.target_id_ != target_id) {
        Eigen::Matrix4f trans_inv;
        trans_inv = trans.inverse();
        trans = trans_inv;
      }
      intern_accum = accum * trans;
      absolute_poses[target_id] = intern_accum;
      computeAbsolutePosesRecursive(grph, targ, intern_accum, absolute_poses, hop_list);
    }
  }

  void computeAbsolutePoses(const Graph &grph,
                            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &absolute_poses,
                            const Eigen::Matrix4f &initial_transform = Eigen::Matrix4f::Identity()) {
    size_t num_frames = boost::num_vertices(grph);
    absolute_poses.resize(num_frames);
    std::vector<bool> hop_list(num_frames, false);
    Vertex source_view = 0;
    hop_list[0] = true;
    Eigen::Matrix4f accum = initial_transform;
    absolute_poses[0] = accum;
    computeAbsolutePosesRecursive(grph, source_view, accum, absolute_poses, hop_list);
  }

  void addView(const pcl::PointCloud<PointT>::ConstPtr &cloud) {
    View view;
    view.cloud_ = cloud;

    v4r::PCLOpenCVConverter<PointT> pcl_opencv_converter;
    pcl_opencv_converter.setInputCloud(view.cloud_);
    auto color_image = pcl_opencv_converter.getRGBImage();

    std::vector<cv::KeyPoint> keypoints;
    v4r::FeatureDetector_KD_SIFTGPU sift;
    sift.detectAndCompute(color_image, keypoints, view.descriptors_);
    view.keypoint_indices_ = sift.getKeypointIndices();

    grph_.push_back(view);
  }

  void buildGraph() {
    for (size_t view_a = 0; view_a < grph_.size(); view_a++) {
      for (size_t view_b = 0; view_b < view_a; view_b++) {
        std::vector<CamConnect> transforms;
        CamConnect edge;
        edge.source_id_ = view_b;
        edge.target_id_ = view_a;

        const auto tfs = v4r::Registration::FeatureBasedRegistration<PointT>::estimateViewTransformationBySIFT(
            *grph_[view_a].cloud_, *grph_[view_b].cloud_, grph_[view_a].keypoint_indices_,
            grph_[view_b].keypoint_indices_, grph_[view_a].descriptors_, grph_[view_b].descriptors_);

        for (const auto &tf : tfs) {
          edge.transformation_ = tf;
          transforms.push_back(edge);
        }

        for (CamConnect &c : transforms) {
          Eigen::Matrix4f icp_refined_trans;
          calcEdgeWeightAndRefineTf(grph_[view_a].cloud_, grph_[view_b].cloud_, c.transformation_, c.edge_weight,
                                    icp_refined_trans);
          c.transformation_ = icp_refined_trans;
          LOG(INFO) << "Edge weight is " << c.edge_weight << " for edge connecting vertex " << c.source_id_ << " and "
                    << c.target_id_;
        }

        if (!transforms.empty()) {
          std::sort(transforms.begin(), transforms.end());
          boost::add_edge(transforms[0].source_id_, transforms[0].target_id_, transforms[0], gs_);
        }
      }
    }
  }

  void compute_mst() {
    boost::property_map<Graph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, gs_);
    std::vector<Edge> spanning_tree;
    boost::kruskal_minimum_spanning_tree(gs_, std::back_inserter(spanning_tree));

    Graph grph_mst;
    std::cout << "Print the edges in the MST:" << std::endl;
    for (std::vector<Edge>::iterator ei = spanning_tree.begin(); ei != spanning_tree.end(); ++ei) {
      CamConnect my_e = weightmap[*ei];
      std::cout << "[" << source(*ei, gs_) << "->" << target(*ei, gs_) << "] with weight " << my_e.edge_weight
                << std::endl;
      boost::add_edge(source(*ei, gs_), target(*ei, gs_), weightmap[*ei], grph_mst);
    }

    computeAbsolutePoses(grph_mst, absolute_poses_);

    for (size_t view_id = 0; view_id < absolute_poses_.size(); view_id++) {
      grph_[view_id].camera_pose_ = absolute_poses_[view_id];
    }
  }

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> getAbsolutePoses() const {
    return absolute_poses_;
  }

  void setCameraIntrinsics(const v4r::Intrinsics &cam) {
    cam_ = cam;
  }
};

int main(int argc, char **argv) {
  typedef pcl::PointXYZRGB PointT;
  bf::path input_dir;
  bool save_pose = false;
  double chop_z = std::numeric_limits<float>::max();
  int verbosity = 0;
  bf::path camera_calibration_file;

  po::options_description desc(
      "Feature-based View Registration via Minimum Spanning Tree\n======================================\n**Allowed "
      "options");
  desc.add_options()("help,h", "produce help message");
  desc.add_options()("input_dir,i", po::value<bf::path>(&input_dir)->required(),
                     "Directory with point clouds to be registered");
  desc.add_options()("chop_z,z", po::value<double>(&chop_z)->default_value(chop_z),
                     "Cut-off distance in meter in z direction.");
  desc.add_options()("save_pose,s", po::bool_switch(&save_pose),
                     "save computed camera pose in sensor header fields of input point clouds.");
  desc.add_options()("camera_calibration,c", po::value<bf::path>(&camera_calibration_file),
                     "Camera calibration file with intrinsic parameters");
  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).run();
  po::store(parsed, vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return -1;
  }
  try {
    po::notify(vm);
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
  }

  if (verbosity >= 0) {
    FLAGS_v = verbosity;
    std::cout << "Enabling verbose logging." << std::endl;
  }
  FLAGS_logtostderr = 1;
  google::InitGoogleLogging(argv[0]);

  LOG(INFO) << "Processing all point clouds in folder " << input_dir.string();

  const auto files_intern = v4r::io::getFilesInDirectory(input_dir, ".*.pcd", false);
  if (files_intern.empty()) {
    LOG(ERROR) << "No files in directory: " << input_dir.string();
    return -1;
  }

  v4r::Intrinsics cam = v4r::Intrinsics::PrimeSense();

  if (bf::exists(camera_calibration_file)) {
    try {
      cam = v4r::Intrinsics::load(camera_calibration_file.string());
    } catch (const std::runtime_error &e) {
      LOG(WARNING) << "Failed to load camera calibration file from " << camera_calibration_file.string()
                   << "! Will use Primesense default camera intrinsics parameters!";
    }
  }

  ViewRegistration view_reg;
  view_reg.setCameraIntrinsics(cam);
  for (const auto &fn : files_intern) {
    const bf::path full_path = input_dir / fn;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(full_path.string(), *cloud);

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    pcl::PassThrough<PointT> pass;
    pass.setFilterLimits(0.f, chop_z);
    pass.setFilterFieldName("z");
    pass.setInputCloud(cloud);
    pass.setKeepOrganized(true);
    pass.filter(*cloud_filtered);

    view_reg.addView(cloud_filtered);
  }

  view_reg.buildGraph();
  view_reg.compute_mst();

  const auto abs_poses = view_reg.getAbsolutePoses();

  if (save_pose) {
    for (size_t i = 0; i < files_intern.size(); i++) {
      const bf::path full_path = input_dir / files_intern[i];
      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
      pcl::io::loadPCDFile(full_path.string(), *cloud);
      v4r::setCloudPose(abs_poses.at(i), *cloud);
      pcl::io::savePCDFileBinaryCompressed(full_path.string(), *cloud);
    }
  }
}
