#include <v4r/common/normal_estimator.h>
#include <boost/algorithm/string.hpp>

namespace v4r {
std::istream& operator>>(std::istream& in, NormalEstimatorType& nt) {
  std::string token;
  in >> token;
  boost::to_upper(token);
  if (token == "PCL_DEFAULT")
    nt = NormalEstimatorType::PCL_DEFAULT;
  else if (token == "PCL_INTEGRAL_NORMAL")
    nt = NormalEstimatorType::PCL_INTEGRAL_NORMAL;
  else if (token == "Z_ADAPTIVE")
    nt = NormalEstimatorType::Z_ADAPTIVE;
  else
    in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream& operator<<(std::ostream& out, const NormalEstimatorType& nt) {
  switch (nt) {
    case NormalEstimatorType::PCL_DEFAULT:
      out << "PCL_DEFAULT";
      break;
    case NormalEstimatorType::PCL_INTEGRAL_NORMAL:
      out << "PCL_INTEGRAL_NORMAL";
      break;
    case NormalEstimatorType::Z_ADAPTIVE:
      out << "Z_ADAPTIVE";
      break;
    default:
      out.setstate(std::ios_base::failbit);
  }
  return out;
}
}  // namespace v4r