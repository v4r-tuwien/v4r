#include <v4r/features/FeatureDetector_D_FREAK.h>

namespace v4r {
void FeatureDetector_D_FREAK::Parameter::init(boost::program_options::options_description &desc,
                                              const std::string &section_name) {
  desc.add_options()((section_name + ".orientationNormalized").c_str(),
                     po::value<bool>(&orientationNormalized_)->default_value(orientationNormalized_),
                     "Enable orientation normalization");
  desc.add_options()((section_name + ".scaleNormalized").c_str(),
                     po::value<bool>(&scaleNormalized_)->default_value(scaleNormalized_), "Enable scale normalization");
  desc.add_options()((section_name + ".patternScale").c_str(),
                     po::value<float>(&patternScale_)->default_value(patternScale_),
                     "Scaling of the description pattern.");
  desc.add_options()((section_name + ".nOctaves").c_str(), po::value<int>(&nOctaves_)->default_value(nOctaves_),
                     "Number of octaves covered by the detected keypoints");
}

FeatureDetector_D_FREAK::FeatureDetector_D_FREAK(const Parameter &_p)
: FeatureDetector(FeatureDetector::Type::D_FREAK), param_(_p) {
#if CV_MAJOR_VERSION < 3
  freak_.reset(
      new cv::FREAK(param_.orientationNormalized_, param_.scaleNormalized_, param_.patternScale_, param_.nOctaves_));
#else
  freak_ = cv::xfeatures2d::FREAK::create(param_.orientationNormalized_, param_.scaleNormalized_, param_.patternScale_,
                                          param_.nOctaves_);
#endif
}

void FeatureDetector_D_FREAK::compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  freak_->compute(im_gray_, keys, descriptors);
}

}  // namespace v4r
