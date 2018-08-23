find_package(realsense2 2.10.0)

if(realsense2_FOUND)
  set(HAVE_REALSENSE2 TRUE)
  set(REALSENSE2_VERSION "${realsense_VERSION_MAJOR}.${realsense_VERSION_MINOR}.${realsense_VERSION_PATCH}")
else()
  set(HAVE_REALSENSE2 FALSE)
endif()

mark_as_advanced(realsense2_DIR)
