//
// Created by mbodych on 11.05.18.
//

#include <v4r/features/types.h>
#include <boost/algorithm/string.hpp>

namespace v4r {

std::istream &operator>>(std::istream &in, FeatureType &t) {
  std::string token;
  in >> token;
  boost::to_upper(token);
  if (token == "FPFH")
    t = FeatureType::FPFH;
  else if (token == "SHOT")
    t = FeatureType::SHOT;
  else if (token == "AKAZE")
    t = FeatureType::AKAZE;
  else if (token == "ALEXNET")
    t = FeatureType::ALEXNET;
  else if (token == "ESF")
    t = FeatureType::ESF;
  else if (token == "GLOBAL_COLOR")
    t = FeatureType::GLOBAL_COLOR;
  else if (token == "OURCVFH")
    t = FeatureType::OURCVFH;
  else if (token == "ROPS")
    t = FeatureType::ROPS;
  else if (token == "SHOT_COLOR")
    t = FeatureType::SHOT_COLOR;
  else if (token == "SIFT_GPU")
    t = FeatureType::SIFT_GPU;
  else if (token == "SIFT_OPENCV")
    t = FeatureType::SIFT_OPENCV;
  else if (token == "SIMPLE_SHAPE")
    t = FeatureType::SIMPLE_SHAPE;
  else if (token == "SURF")
    t = FeatureType::SURF;
  else
    in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream &operator<<(std::ostream &out, const FeatureType &t) {
  switch (t) {
    case FeatureType::FPFH:
      out << "FPFH";
      break;
    case FeatureType::SHOT:
      out << "SHOT";
      break;
    case FeatureType::AKAZE:
      out << "AKAZE";
      break;
    case FeatureType::ALEXNET:
      out << "ALEXNET";
      break;
    case FeatureType::ESF:
      out << "ESF";
      break;
    case FeatureType::GLOBAL_COLOR:
      out << "GLOBAL_COLOR";
      break;
    case FeatureType::OURCVFH:
      out << "OURCVFH";
      break;
    case FeatureType::ROPS:
      out << "ROPS";
      break;
    case FeatureType::SHOT_COLOR:
      out << "SHOT_COLOR";
      break;
    case FeatureType::SIFT_GPU:
      out << "SIFT_GPU";
      break;
    case FeatureType::SIFT_OPENCV:
      out << "SIFT_OPENCV";
      break;
    case FeatureType::SIMPLE_SHAPE:
      out << "SIMPLE_SHAPE";
      break;
    case FeatureType::SURF:
      out << "SURF";
      break;
    default:
      out.setstate(std::ios_base::failbit);
  }
  return out;
}

}  // namespace v4r