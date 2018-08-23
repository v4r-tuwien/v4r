set(mvstexturing_DIR ${V4R_3P_MVSTEXTURING_INSTALL_DIR}/lib/cmake/mvstexturing)

find_package(mvstexturing CONFIG)

if(mvstexturing_FOUND)
  set(HAVE_MVSTEXTURING TRUE)
else()
  set(HAVE_MVSTEXTURING FALSE)
endif()
