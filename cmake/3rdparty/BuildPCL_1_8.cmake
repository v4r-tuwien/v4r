v4r_build_external_project("PCL_1_8"
  BUNDLED
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DPCL_DIR=${PCL_CONFIG_PATH}"
)
