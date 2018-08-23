v4r_build_external_project("MVSTexturing"
  GIT_REPOSITORY https://rgit.acin.tuwien.ac.at/v4r/mvs-texturing.git
  GIT_TAG v4r
  CMAKE_ARGS "-DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DBUILD_APPS=OFF"
)
