#include <glog/logging.h>
#include <v4r/ml/nearestNeighbor.h>
#include <opencv2/core/eigen.hpp>

namespace v4r {

void NearestNeighborClassifier::train(const Eigen::MatrixXf &data, const Eigen::VectorXi &label) {
  cv::Mat training_data, training_label;
  cv::eigen2cv(data, training_data);
  cv::eigen2cv(label, training_label);
  all_training_data = training_data.clone();
  training_label_ = training_label.clone();

  switch (param_.distance_metric_) {
    case DistanceMetric::L1:
      flann_index.reset(new cv::flann::Index(all_training_data, cv::flann::KDTreeIndexParams(param_.kdtree_num_trees_),
                                             cvflann::FLANN_DIST_L1));
      break;
    case DistanceMetric::L2:
      flann_index.reset(new cv::flann::Index(all_training_data, cv::flann::KDTreeIndexParams(param_.kdtree_num_trees_),
                                             cvflann::FLANN_DIST_L2));
      break;
    case DistanceMetric::CHI_SQUARE:
      flann_index.reset(new cv::flann::Index(all_training_data, cv::flann::KDTreeIndexParams(param_.kdtree_num_trees_),
                                             cvflann::FLANN_DIST_CHI_SQUARE));
      break;
    case DistanceMetric::HELLINGER:
      flann_index.reset(new cv::flann::Index(all_training_data, cv::flann::KDTreeIndexParams(param_.kdtree_num_trees_),
                                             cvflann::FLANN_DIST_HELLINGER));
      break;
    case DistanceMetric::HAMMING:
      flann_index.reset(new cv::flann::Index(all_training_data, cv::flann::KDTreeIndexParams(param_.kdtree_num_trees_),
                                             cvflann::FLANN_DIST_HAMMING));
      break;
    default:
      LOG(ERROR) << "Given distance metric " << param_.distance_metric_ << " is not implemented!";
  }
}

void NearestNeighborClassifier::predict(const Eigen::MatrixXf &query_data, Eigen::MatrixXi &predicted_label) const {
  cv::Mat query_data_cv;
  cv::eigen2cv(query_data, query_data_cv);
  flann_index->knnSearch(query_data_cv, knn_indices_, knn_distances_, param_.knn_,
                         cv::flann::SearchParams(param_.checks_));

  predicted_label.resize(knn_indices_.rows, knn_indices_.cols);
  for (int row_id = 0; row_id < knn_indices_.rows; row_id++) {
    for (int col_id = 0; col_id < knn_indices_.cols; col_id++) {
      int idx = knn_indices_.at<int>(row_id, col_id);
      predicted_label(row_id, col_id) = training_label_.at<int>(idx);
    }
  }
}

void NearestNeighborClassifier::getTrainingSampleIDSforPredictions(Eigen::MatrixXi &predicted_training_sample_indices,
                                                                   Eigen::MatrixXf &distances) const {
  cv::cv2eigen(knn_indices_, predicted_training_sample_indices);
  cv::cv2eigen(knn_distances_, distances);
}

}  // namespace v4r
