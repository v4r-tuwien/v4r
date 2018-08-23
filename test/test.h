#pragma once

#include <string>

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}

/// Get an absolute path to a test file given its relative path to the test data directory.
/// Additionally checks file existence. If the file does not exist, warns the user that Git LFS should be used to
/// fetch test data from the server.
inline std::string getTestFilename(const std::string& relative_path) {
  boost::filesystem::path p(std::string(TEST_DATA_DIR));
  auto file = p / relative_path;
  if (!boost::filesystem::exists(file)) {
    std::cerr << "Test data file " << file << " does not exist." << std::endl;
    std::cerr << "Make sure to run `git lfs fetch` to download test data." << std::endl;
  }
  return file.make_preferred().string();
}
