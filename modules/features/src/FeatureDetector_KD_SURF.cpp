#include <glog/logging.h>
#include <pcl/common/io.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/features/FeatureDetector_KD_SURF.h>

namespace v4r {

FeatureDetector_KD_SURF::FeatureDetector_KD_SURF(const Parameter &p)
: FeatureDetector(FeatureDetector::Type::KD_SURF), param_(p) {
  descr_name_ = "surf";
#if CV_MAJOR_VERSION < 3
  surf_.reset(new cv::SURF(param_.hessianThreshold_, param_.nOctaves_, param_.nOctaveLayers_, param_.extended_,
                           param_.upright_));
#else
  surf_ = cv::xfeatures2d::SURF::create(param_.hessianThreshold_, param_.nOctaves_, param_.nOctaveLayers_,
                                        param_.extended_, param_.upright_);
#endif
}

void FeatureDetector_KD_SURF::detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
                                               cv::Mat &descriptors, const cv::Mat &object_mask) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  surf_->detectAndCompute(im_gray_, object_mask, keys, descriptors);
  computeKeypointIndices(im_gray_, keys);
}

void FeatureDetector_KD_SURF::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
                                     const cv::Mat &object_mask) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  surf_->detect(im_gray_, keys, object_mask);
  computeKeypointIndices(im_gray_, keys);
}

void FeatureDetector_KD_SURF::compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  surf_->compute(im_gray_, keys, descriptors);
}

}  // namespace v4r
