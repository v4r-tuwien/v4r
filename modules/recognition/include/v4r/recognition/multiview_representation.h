#ifndef MULTI_VIEW_REPRESENTATION_H
#define MULTI_VIEW_REPRESENTATION_H
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <v4r/common/plane_model.h>
#include <v4r/recognition/local_rec_object_hypotheses.h>
#include <v4r/recognition/model.h>

namespace v4r {

template <typename PointT>
class V4R_EXPORTS View {
 protected:
  typedef Model<PointT> ModelT;
  typedef std::shared_ptr<ModelT> ModelTPtr;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  View();
  typename pcl::PointCloud<PointT>::Ptr scene_;
  //    typename pcl::PointCloud<PointT>::Ptr scene_f_;
  pcl::PointCloud<pcl::Normal>::Ptr scene_normals_;
  std::vector<int> filtered_scene_indices_;
  Eigen::Matrix4f absolute_pose_;
  typename pcl::PointCloud<PointT>::Ptr scene_kp_;
  pcl::PointCloud<pcl::Normal>::Ptr scene_kp_normals_;
  typename std::map<std::string, LocalObjectHypothesis<PointT>> hypotheses_;
  std::vector<std::vector<float>> sift_signatures_;
  //    std::vector<float> sift_keypoints_scales_;
  std::vector<int> sift_kp_indices_;
  Eigen::Matrix4f transform_to_world_co_system_;
  bool has_been_hopped_;
  double cumulative_weight_to_new_vrtx_;
  pcl::PointIndices kp_indices_;
  size_t id_;

  /** @brief: generated object hypotheses from correspondence grouping (before verification) */
  std::vector<ModelTPtr> models_;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> transforms_;
  std::vector<PlaneModel<PointT>, Eigen::aligned_allocator<PlaneModel<PointT>>> planes_;

  /** @brief boolean vector defining if model or plane is verified (models are first in the vector and its size is equal
   * to models_) */
  std::vector<bool> model_or_plane_is_verified_;

  /** @brief vector defining from which view the object hypothesis comes from */
  std::vector<size_t> origin_view_id_;

  std::vector<std::vector<float>> pt_properties_;  ///< noise properties for each point
};
}  // namespace v4r
#endif
