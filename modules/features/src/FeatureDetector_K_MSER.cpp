#include <v4r/features/FeatureDetector_K_MSER.h>

namespace v4r {

FeatureDetector_K_MSER::FeatureDetector_K_MSER(const Parameter &_p)
: FeatureDetector(FeatureDetector::Type::K_MSER), param(_p) {
  descr_name_ = "mser";

#if CV_MAJOR_VERSION < 3
  mser_ = new cv::MSER(param.delta_, param.min_area_, param.max_area_, param.max_variation_, param.min_diversity_,
                       param.max_evolution_, param.area_threshold_, param.min_margin_, param.edge_blur_size_);
#else
  mser_ = cv::MSER::create(param.delta_, param.min_area_, param.max_area_, param.max_variation_, param.min_diversity_,
                           param.max_evolution_, param.area_threshold_, param.min_margin_, param.edge_blur_size_);
#endif
}

void FeatureDetector_K_MSER::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, const cv::Mat &object_mask) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  mser_->detect(im_gray_, keys, object_mask);
  computeKeypointIndices(im_gray_, keys);
}
}  // namespace v4r
