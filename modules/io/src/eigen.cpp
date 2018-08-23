#include <v4r/io/eigen.h>

#include <glog/logging.h>
#include <boost/algorithm/string.hpp>

namespace v4r {
namespace io {

bool writeMatrixToFile(const boost::filesystem::path &path, const Eigen::Matrix4f &matrix) {
  std::ofstream out(path.string().c_str());
  if (!out) {
    std::cout << "Cannot open file.\n";
    return false;
  }

  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      out << matrix(i, j);
      if (!(i == 3 && j == 3))
        out << " ";
    }
  }
  out.close();

  return true;
}

Eigen::Matrix4f readMatrixFromFile(const boost::filesystem::path &path, size_t padding) {
  CHECK(boost::filesystem::exists(path) || boost::filesystem::is_regular_file(path))
      << "Given file path " << path.string() << " to read matrix does not exist!";

  std::ifstream in(path.string().c_str());

  std::vector<float> vals;
  float val;
  while (in >> val)
    vals.push_back(val);
  in.close();

  CHECK(vals.size() == 16 + padding) << "input file " << path << " is not a 4x4 matrix with " << padding << " padding.";

  Eigen::Matrix4f matrix;
  for (int i = 0; i < 16; i++)
    matrix(i / 4, i % 4) = vals[padding + i];
  return matrix;
}
}  // namespace io
}  // namespace v4r
