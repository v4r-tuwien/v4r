SET(FeatureBasedViewRegistrationByMST_DEPS v4r_core v4r_registration v4r_io v4r_common v4r_features)
v4r_check_dependencies(${FeatureBasedViewRegistrationByMST_DEPS})

if(NOT V4R_DEPENDENCIES_FOUND)
  message(***FeatureBasedViewRegistrationByMST_DEPS does not meed dependencies*****)
  return()
endif()

find_package(PkgConfig REQUIRED)
pkg_search_module(GL REQUIRED gl)

v4r_include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

if(WIN32)
  link_directories(${CMAKE_CURRENT_BINARY_DIR})
endif()

add_executable(FeatureBasedViewRegistrationByMST main.cpp)
target_link_libraries(FeatureBasedViewRegistrationByMST ${FeatureBasedViewRegistrationByMST_DEPS} ${DEP_LIBS})

INSTALL(TARGETS FeatureBasedViewRegistrationByMST
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

