#include <v4r/ml/types.h>
#include <boost/algorithm/string.hpp>
#include <iostream>

namespace v4r {

std::istream& operator>>(std::istream& in, ClassifierType& cm) {
  std::string token;
  in >> token;
  boost::to_upper(token);
  if (token == "KNN")
    cm = ClassifierType::KNN;
  else if (token == "SVM")
    cm = ClassifierType::SVM;
  else if (token == "CNN")
    cm = ClassifierType::CNN;
  else
    in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream& operator<<(std::ostream& out, const ClassifierType& cm) {
  switch (cm) {
    case ClassifierType::KNN:
      out << "KNN";
      break;
    case ClassifierType::SVM:
      out << "SVM";
      break;
    case ClassifierType::CNN:
      out << "CNN";
      break;
    default:
      out.setstate(std::ios_base::failbit);
  }
  return out;
}

}  // namespace v4r