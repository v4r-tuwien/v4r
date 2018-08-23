#include <v4r/io/cv.h>
#include <fstream>
#include <iostream>

namespace v4r {
namespace io {

bool writeMatBinary(const bf::path &path, const cv::Mat &mat) {
  std::ofstream fs(path.string(), std::fstream::binary);

  // Header
  int type = mat.type();
  int channels = mat.channels();
  fs.write((char *)&mat.rows, sizeof(int));  // rows
  fs.write((char *)&mat.cols, sizeof(int));  // cols
  fs.write((char *)&type, sizeof(int));      // type
  fs.write((char *)&channels, sizeof(int));  // channels

  // Data
  if (mat.isContinuous()) {
    fs.write(mat.ptr<char>(0), (mat.dataend - mat.datastart));
  } else {
    int rowsz = CV_ELEM_SIZE(type) * mat.cols;
    for (int r = 0; r < mat.rows; ++r) {
      fs.write(mat.ptr<char>(r), rowsz);
    }
  }
  fs.close();

  return true;
}

cv::Mat readMatBinary(const bf::path &filename) {
  std::ifstream fs(filename.string(), std::fstream::binary);

  // Header
  int rows, cols, type, channels;
  fs.read((char *)&rows, sizeof(int));      // rows
  fs.read((char *)&cols, sizeof(int));      // cols
  fs.read((char *)&type, sizeof(int));      // type
  fs.read((char *)&channels, sizeof(int));  // channels

  // Data
  cv::Mat mat(rows, cols, type);
  fs.read((char *)mat.data, CV_ELEM_SIZE(type) * rows * cols);
  fs.close();
  return mat;
}

}  // namespace io
}  // namespace v4r
