#include <v4r/features/FeatureDetector_KD_BRISK.h>

namespace v4r {

void FeatureDetector_KD_BRISK::Parameter::init(boost::program_options::options_description &desc,
                                               const std::string &section_name) {
  desc.add_options()((section_name + ".thresh").c_str(), po::value<int>(&thresh_)->default_value(thresh_),
                     "AGAST detection threshold score");
  desc.add_options()((section_name + ".octaves").c_str(), po::value<int>(&octaves_)->default_value(octaves_),
                     "detection octaves. Use 0 to do single scale.");
  desc.add_options()((section_name + ".patternScale").c_str(),
                     po::value<float>(&patternScale_)->default_value(patternScale_),
                     "apply this scale to the pattern used for sampling the neighbourhood of a keypoint.");
}

FeatureDetector_KD_BRISK::FeatureDetector_KD_BRISK(const Parameter &p)
: FeatureDetector(FeatureDetector::Type::KD_BRISK), param_(p) {
  descr_name_ = "brisk";

#if CV_MAJOR_VERSION < 3
  brisk_ = new cv::BRISK(param_.thresh_, param_.octaves_, param_.patternScale_);
#else
  brisk_ = cv::BRISK::create(param_.thresh_, param_.octaves_, param_.patternScale_);
#endif
}

void FeatureDetector_KD_BRISK::detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
                                                cv::Mat &descriptors, const cv::Mat &object_mask) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

#if CV_MAJOR_VERSION < 3
  brisk_->detect(im_gray_, keys, object_mask);
  brisk_->compute(im_gray_, keys, descriptors);
#else
  brisk_->detectAndCompute(im_gray_, object_mask, keys, descriptors);
#endif
  computeKeypointIndices(im_gray_, keys);
}

void FeatureDetector_KD_BRISK::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
                                      const cv::Mat &object_mask) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  brisk_->detect(im_gray_, keys, object_mask);
  computeKeypointIndices(im_gray_, keys);
}

void FeatureDetector_KD_BRISK::compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  brisk_->compute(im_gray_, keys, descriptors);
}
}  // namespace v4r
