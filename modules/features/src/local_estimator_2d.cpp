#include <glog/logging.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/features/local_estimator_2d.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace v4r {

template <typename PointT>
void LocalEstimator2D<PointT>::removeNanKeypoints(cv::Mat &signatures, std::vector<cv::KeyPoint> &keypoints2d) {
  size_t kept = 0;
  for (size_t i = 0; i < keypoint_indices_.size(); i++) {
    int idx = keypoint_indices_[i];

    if (pcl::isFinite(cloud_->points[idx]) && cloud_->points[idx].z < max_distance_) {
      signatures.row(i).copyTo(signatures.row(kept));
      keypoints2d[kept] = keypoints2d[i];
      keypoint_indices_[kept] = idx;
      kept++;
    }
  }
  signatures.resize(kept);
  keypoints2d.resize(kept);
  keypoint_indices_.resize(kept);
  indices_.clear();
}

template <typename PointT>
void LocalEstimator2D<PointT>::compute(cv::Mat &signatures) {
  CHECK(cloud_ && cloud_->isOrganized());

  PCLOpenCVConverter<PointT> pcl_opencv_converter;
  pcl_opencv_converter.setInputCloud(cloud_);
  const cv::Mat_<cv::Vec3b> colorImage = pcl_opencv_converter.getRGBImage();

  cv::Mat object_mask;
  if (!indices_.empty()) {
    object_mask = cv::Mat_<unsigned char>(colorImage.rows, colorImage.cols);
    object_mask.setTo(0);

    for (int idx : indices_) {
      int v = idx / colorImage.cols;
      int u = idx % colorImage.cols;
      object_mask.at<unsigned char>(v, u) = 255;
    }
  }

  std::vector<cv::KeyPoint> keypoints2d;
  compute(colorImage, keypoints2d, signatures, object_mask);
  removeNanKeypoints(signatures, keypoints2d);
}

template <typename PointT>
void LocalEstimator2D<PointT>::compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &signatures,
                                       const cv::Mat &object_mask) {
  if (keypoint_detector_ && keypoint_detector_->getDescriptorName() != feat_->getDescriptorName()) {
    keypoint_detector_->detect(image, keypoints, object_mask);
    feat_->compute(image, keypoints, signatures);
    keypoint_indices_ = keypoint_detector_->getKeypointIndices();
  } else {
    feat_->detectAndCompute(image, keypoints, signatures, object_mask);
    keypoint_indices_ = feat_->getKeypointIndices();
  }
}

template class V4R_EXPORTS LocalEstimator2D<pcl::PointXYZRGB>;

}  // namespace v4r