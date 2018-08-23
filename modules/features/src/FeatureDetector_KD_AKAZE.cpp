#include <glog/logging.h>
#include <v4r/features/FeatureDetector_KD_AKAZE.h>

namespace v4r {

FeatureDetector_KD_AKAZE::FeatureDetector_KD_AKAZE(const Parameter &p)
: FeatureDetector(FeatureDetector::Type::KD_AKAZE), param_(p) {
  descr_name_ = "akaze";
  akaze_ = cv::AKAZE::create();
  akaze_->setDescriptorSize(param_.descriptor_size_);
  akaze_->setDescriptorType(param_.descriptor_type_);
  akaze_->setDescriptorChannels(param_.descriptor_channels_);
  akaze_->setThreshold(param_.detector_threshold_);
  akaze_->setNOctaves(param_.nOctaves_);
  akaze_->setNOctaveLayers(param_.nOctaveLayers_);
  akaze_->setDiffusivity(param_.diffusivity_);
}

void FeatureDetector_KD_AKAZE::Parameter::init(boost::program_options::options_description &desc,
                                               const std::string &section_name) {
  desc.add_options()((section_name + ".descriptor_type").c_str(),
                     po::value<int>(&descriptor_type_)->default_value(descriptor_type_),
                     "Type of the extracted descriptor: DESCRIPTOR_KAZE, DESCRIPTOR_KAZE_UPRIGHT, DESCRIPTOR_MLDB or "
                     "DESCRIPTOR_MLDB_UPRIGHT.");
  desc.add_options()((section_name + ".descriptor_size").c_str(),
                     po::value<int>(&descriptor_size_)->default_value(descriptor_size_),
                     "Size of the descriptor in bits. 0 -> Full size");
  desc.add_options()((section_name + ".descriptor_channels").c_str(),
                     po::value<int>(&descriptor_channels_)->default_value(descriptor_channels_),
                     "Number of channels in the descriptor (1, 2, 3)");
  desc.add_options()((section_name + ".detector_threshold").c_str(),
                     po::value<float>(&detector_threshold_)->default_value(detector_threshold_),
                     "Detector response threshold to accept point");
  desc.add_options()((section_name + ".nOctaves").c_str(), po::value<int>(&nOctaves_)->default_value(nOctaves_),
                     "Maximum octave evolution of the image");
  desc.add_options()((section_name + ".nOctaveLayers").c_str(),
                     po::value<int>(&nOctaveLayers_)->default_value(nOctaveLayers_),
                     "Default number of sublevels per scale level");
  desc.add_options()((section_name + ".diffusivity").c_str(),
                     po::value<int>(&diffusivity_)->default_value(diffusivity_),
                     "Diffusivity type. DIFF_PM_G1, DIFF_PM_G2, DIFF_WEICKERT or DIFF_CHARBONNIER");
}

void FeatureDetector_KD_AKAZE::printParams(std::ostream &s) {
  s << "descriptor type: " << akaze_->getDescriptorType() << std::endl
    << "descriptor size: " << akaze_->getDescriptorSize() << std::endl
    << "descriptor channels: " << akaze_->getDescriptorChannels() << std::endl
    << "threshold: " << akaze_->getThreshold() << std::endl
    << "nOctaves: " << akaze_->getNOctaves() << std::endl
    << "nOctavesLayers: " << akaze_->getNOctaveLayers() << std::endl
    << "diffusivity: " << akaze_->getDiffusivity() << std::endl;
}

void FeatureDetector_KD_AKAZE::detectAndCompute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
                                                cv::Mat &descriptors, const cv::Mat &object_mask) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  akaze_->detectAndCompute(im_gray_, object_mask, keys, descriptors);
  computeKeypointIndices(im_gray_, keys);
}

void FeatureDetector_KD_AKAZE::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys,
                                      const cv::Mat &object_mask) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  akaze_->detect(im_gray_, keys, object_mask);
  computeKeypointIndices(im_gray_, keys);
}

void FeatureDetector_KD_AKAZE::compute(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray_, cv::COLOR_RGB2GRAY);
  else
    im_gray_ = image;

  akaze_->compute(im_gray_, keys, descriptors);
}
}  // namespace v4r
