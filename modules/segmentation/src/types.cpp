#include <v4r/segmentation/types.h>
#include <boost/algorithm/string.hpp>

namespace v4r {

std::istream& operator>>(std::istream& in, SegmentationType& st) {
  std::string token;
  in >> token;
  boost::to_upper(token);
  if (token == "CONNECTED_COMPONENTS_2D")
    st = SegmentationType::CONNECTED_COMPONENTS_2D;
  else if (token == "EUCLIDEAN")
    st = SegmentationType::EUCLIDEAN_SEGMENTATION;
  else if (token == "ORGANIZED_CONNECTED_COMPONENTS")
    st = SegmentationType::ORGANIZED_CONNECTED_COMPONENTS;
  else if (token == "SMOOTH_EUCLIDEAN")
    st = SegmentationType::SMOOTH_EUCLIDEAN_CLUSTERING;
  else
    in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream& operator<<(std::ostream& out, const SegmentationType& st) {
  switch (st) {
    case SegmentationType::CONNECTED_COMPONENTS_2D:
      out << "CONNECTED_COMPONENTS_2D";
      break;
    case SegmentationType::EUCLIDEAN_SEGMENTATION:
      out << "EUCLIDEAN";
      break;
    case SegmentationType::ORGANIZED_CONNECTED_COMPONENTS:
      out << "ORGANIZED_CONNECTED_COMPONENTS";
      break;
    case SegmentationType::SMOOTH_EUCLIDEAN_CLUSTERING:
      out << "SMOOTH_EUCLIDEAN";
      break;
    default:
      out.setstate(std::ios_base::failbit);
  }
  return out;
}

std::istream& operator>>(std::istream& in, PlaneExtractionType& pt) {
  std::string token;
  in >> token;
  boost::to_upper(token);
  if (token == "ORGANIZED_MULTIPLANE")
    pt = PlaneExtractionType::ORGANIZED_MULTIPLANE;
  else if (token == "SAC")
    pt = PlaneExtractionType::SAC;
  else if (token == "SAC_NORMALS")
    pt = PlaneExtractionType::SAC_NORMALS;
  else if (token == "TILE")
    pt = PlaneExtractionType::TILE;
  else
    in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream& operator<<(std::ostream& out, const PlaneExtractionType& pt) {
  switch (pt) {
    case PlaneExtractionType::ORGANIZED_MULTIPLANE:
      out << "ORGANIZED_MULTIPLANE";
      break;
    case PlaneExtractionType::SAC:
      out << "SAC";
      break;
    case PlaneExtractionType::SAC_NORMALS:
      out << "SAC_NORMALS";
      break;
    case PlaneExtractionType::TILE:
      out << "TILE";
      break;
    default:
      out.setstate(std::ios_base::failbit);
  }
  return out;
}

}  // namespace v4r