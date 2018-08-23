SET(DemoTemporalPointCloudFilter_DEPS v4r_core v4r_io v4r_common v4r_tracking v4r_reconstruction v4r_keypoints v4r_features v4r_camera_tracking_and_mapping v4r_recognition v4r_registration)
v4r_check_dependencies(${DemoTemporalPointCloudFilter_DEPS})

if(NOT V4R_DEPENDENCIES_FOUND)
  message(***DemoTemporalPointCloudFilter does not meed dependencies*****)
  return()
endif()

v4r_include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

if(WIN32)
  link_directories(${CMAKE_CURRENT_BINARY_DIR})
endif()

if(NOT HAVE_GLM)
  message(***DemoTemporalPointCloudFilter requires GLM*****)
  return()
endif()

if(NOT HAVE_QT)
  message(***DemoTemporalPointCloudFilter requires Qt*****)
  return()
endif()

if(NOT HAVE_OPENGL)
  message(***DemoTemporalPointCloudFilter requires OpenGL*****)
  return()
endif()

set(SOURCES
        Camera.cpp
        glviewer.cpp
        main.cpp
        mainwindow.cpp
        params.cpp
        sensor.cpp
)

set(HEADERS
        glviewer.h
        mainwindow.h
        params.h
        sensor.h
)

set(FORMS
        mainwindow.ui
        params.ui
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
qt5_wrap_ui(FORMS_MOC ${FORMS})

find_package(PkgConfig REQUIRED)
pkg_search_module(GL REQUIRED gl)

add_definitions(-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
add_definitions(-Dlinux -D__x86_64__)


include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OPENNI2_INCLUDE_DIR})
add_executable(demoTemporalPointCloudFilter ${SOURCES} ${HEADERS_MOC} ${FORMS_MOC})
target_link_libraries(demoTemporalPointCloudFilter ${DemoTemporalPointCloudFilter_DEPS} glm qt opengl ${DEP_LIBS})

INSTALL(TARGETS demoTemporalPointCloudFilter
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

