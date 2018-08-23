#include <v4r/common/metrics.h>
#include <boost/algorithm/string.hpp>

namespace v4r {

std::istream &operator>>(std::istream &in, DistanceMetric &dm) {
  std::string token;
  in >> token;
  boost::to_upper(token);
  if (token == "L1")
    dm = DistanceMetric::L1;
  else if (token == "L2")
    dm = DistanceMetric::L2;
  else if (token == "CHI_SQUARE")
    dm = DistanceMetric::CHI_SQUARE;
  else if (token == "HELLINGER")
    dm = DistanceMetric::HELLINGER;
  else if (token == "HAMMING")
    dm = DistanceMetric::HAMMING;
  else
    in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream &operator<<(std::ostream &out, const DistanceMetric &dm) {
  switch (dm) {
    case DistanceMetric::L1:
      out << "L1";
      break;
    case DistanceMetric::L2:
      out << "L2";
      break;
    case DistanceMetric::CHI_SQUARE:
      out << "CHI_SQUARE";
      break;
    case DistanceMetric::HELLINGER:
      out << "HELLINGER";
      break;
    case DistanceMetric::HAMMING:
      out << "HAMMING";
      break;
    default:
      out.setstate(std::ios_base::failbit);
  }
  return out;
}

}  // namespace v4r