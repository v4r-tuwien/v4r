#include <v4r/features/FeatureDetector.h>
#include <boost/algorithm/string.hpp>

namespace v4r {

void FeatureDetector::computeKeypointIndices(const cv::Mat &image, const std::vector<cv::KeyPoint> &keys) {
  keypoint_indices_.resize(keys.size());
  for (size_t i = 0; i < keys.size(); i++) {
    int u = std::min<int>(image.cols - 1, keys[i].pt.x + 0.5f);
    int v = std::min<int>(image.rows - 1, keys[i].pt.y + 0.5f);
    int idx = v * image.cols + u;
    keypoint_indices_[i] = idx;
  }
}

std::istream &operator>>(std::istream &in, FeatureDetector::Type &t) {
  std::string token;
  in >> token;
  boost::to_upper(token);
  if (token == "MSER")
    t = FeatureDetector::Type::K_MSER;
  else if (token == "HARRIS")
    t = FeatureDetector::Type::K_HARRIS;
  else if (token == "CVSIFT")
    t = FeatureDetector::Type::KD_CVSIFT;
  else if (token == "SIFTGPU")
    t = FeatureDetector::Type::KD_SIFTGPU;
  else if (token == "FREAK")
    t = FeatureDetector::Type::D_FREAK;
  else if (token == "BRISK")
    t = FeatureDetector::Type::KD_BRISK;
  else if (token == "ORB")
    t = FeatureDetector::Type::KD_ORB;
  else if (token == "FAST_IMGD")
    t = FeatureDetector::Type::KD_FAST_IMGD;
  else if (token == "AKAZE")
    t = FeatureDetector::Type::KD_AKAZE;
  else if (token == "SURF")
    t = FeatureDetector::Type::KD_SURF;
  else
    in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream &operator<<(std::ostream &out, const FeatureDetector::Type &t) {
  switch (t) {
    case FeatureDetector::Type::K_MSER:
      out << "MSER";
      break;
    case FeatureDetector::Type::K_HARRIS:
      out << "HARRIS";
      break;
    case FeatureDetector::Type::KD_CVSIFT:
      out << "CVSIFT";
      break;
    case FeatureDetector::Type::KD_SIFTGPU:
      out << "SIFTGPU";
      break;
    case FeatureDetector::Type::D_FREAK:
      out << "FREAK";
      break;
    case FeatureDetector::Type::KD_BRISK:
      out << "BRISK";
      break;
    case FeatureDetector::Type::KD_ORB:
      out << "ORB";
      break;
    case FeatureDetector::Type::KD_FAST_IMGD:
      out << "FAST_IMGD";
      break;
    case FeatureDetector::Type::KD_AKAZE:
      out << "AKAZE";
      break;
    case FeatureDetector::Type::KD_SURF:
      out << "SURF";
      break;
    default:
      out.setstate(std::ios_base::failbit);
  }
  return out;
}

}  // namespace v4r