#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <v4r/segmentation/segmenter_conditional_euclidean.h>
#include <pcl/impl/instantiate.hpp>

namespace v4r {

template <typename PointT>
void ConditionalEuclideanSegmenter<PointT>::segment() {
  // Prepare output (going to use push_back)
  clusters_.clear();

  // Validity checks
  if (scene_->points.empty() || !condition_function_)
    return;

  if (!searcher_) {
    if (scene_->isOrganized())
      searcher_.reset(new pcl::search::OrganizedNeighbor<PointT>());
    else
      searcher_.reset(new pcl::search::KdTree<PointT>());
  }
  searcher_->setInputCloud(scene_);

  // Create a bool vector of processed point indices, and initialize it to false
  // Need to have it contain all possible points because radius search can not return indices into indices
  boost::dynamic_bitset<> processed(scene_->points.size(), 0);
  for (size_t idx = 0; idx < scene_->points.size(); idx++)  // iii = input indices iterator
  {
    // Has this point been processed before?
    if (processed[idx])
      continue;

    // Set up a new growing cluster
    std::vector<int> current_cluster = {static_cast<int>(idx)};  // Add the point to the cluster
    processed.set(idx);

    size_t cii = 0;
    // Process the current cluster (it can be growing in size as it is being processed)
    while (cii < current_cluster.size()) {
      // Search for neighbors around the current seed point of the current cluster
      float cluster_tolerance = param_.cluster_tolerance_;
      if (param_.z_adaptive_) {
        float mult = std::max(scene_->points[current_cluster[cii]].z, 1.f);
        cluster_tolerance = param_.cluster_tolerance_ * mult;
      }

      // Temp variables used by search class
      std::vector<int> nn_indices;
      std::vector<float> nn_distances;

      if (!pcl::isFinite(scene_->points[current_cluster[cii]])) {
        cii++;
        continue;
      }

      if (searcher_->radiusSearch(scene_->points[current_cluster[cii]], cluster_tolerance, nn_indices, nn_distances) <
          1) {
        cii++;
        continue;
      }

      // Process the neighbors
      for (size_t nii = 1; nii < nn_indices.size(); nii++) {
        // Has this point been processed before?
        if (processed[nn_indices[nii]])
          continue;

        // Validate if condition holds
        if (condition_function_(scene_->points[current_cluster[cii]], scene_->points[nn_indices[nii]],
                                nn_distances[nii])) {
          // Add the point to the cluster
          current_cluster.push_back(nn_indices[nii]);
          processed.set(nn_indices[nii]);
        }
      }
      cii++;
    }

    if ((current_cluster.size() >= param_.min_cluster_size_) && (current_cluster.size() <= param_.max_cluster_size_)) {
      clusters_.push_back(current_cluster);
    }
  }
}

#define PCL_INSTANTIATE_ConditionalEuclideanSegmenter(T) template class V4R_EXPORTS ConditionalEuclideanSegmenter<T>;
PCL_INSTANTIATE(ConditionalEuclideanSegmenter, PCL_XYZ_POINT_TYPES)
}  // namespace v4r
