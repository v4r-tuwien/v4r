#include <v4r/keypoints/types.h>
#include <boost/algorithm/string.hpp>

namespace v4r {

std::istream &operator>>(std::istream &in, KeypointType &t) {
  std::string token;
  in >> token;
  boost::to_upper(token);
  if (token == "HARRIS3D")
    t = KeypointType::HARRIS3D;
  else if (token == "ISS")
    t = KeypointType::ISS;
  else if (token == "NARF")
    t = KeypointType::NARF;
  else if (token == "UNIFORM_SAMPLING")
    t = KeypointType::UniformSampling;
  else
    in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream &operator<<(std::ostream &out, const KeypointType &t) {
  switch (t) {
    case KeypointType::HARRIS3D:
      out << "HARRIS3D";
      break;
    case KeypointType::ISS:
      out << "ISS";
      break;
    case KeypointType::NARF:
      out << "NARF";
      break;
    case KeypointType::UniformSampling:
      out << "UNIFORM_SAMPLING";
      break;
    default:
      out.setstate(std::ios_base::failbit);
  }
  return out;
}

}  // namespace v4r