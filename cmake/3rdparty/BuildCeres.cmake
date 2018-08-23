# Glog is a mandatory dependency and should have been found by now.
# Extract the include directory and library location from the "glog" imported target.
get_target_property(_library glog IMPORTED_LOCATION)
get_target_property(_include_dirs glog INTERFACE_INCLUDE_DIRECTORIES)

v4r_build_external_project("Ceres"
  URL "file://${CMAKE_SOURCE_DIR}/ceres-solver-1.14.0.tar.gz"
  URL_HASH SHA256=4744005fc3b902fed886ea418df70690caa8e2ff6b5a90f3dd88a3d291ef8e8e
  CMAKE_ARGS "-DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DGLOG_INCLUDE_DIR=${_include_dirs} -DGLOG_LIBRARY=${_library}"
)
