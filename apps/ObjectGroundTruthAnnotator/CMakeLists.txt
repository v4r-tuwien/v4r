SET(GT6DOF_DEPS v4r_core v4r_common v4r_io v4r_features v4r_keypoints v4r_segmentation v4r_recognition v4r_registration v4r_ml v4r_apps)
v4r_check_dependencies(${GT6DOF_DEPS})

if(NOT V4R_DEPENDENCIES_FOUND)
    message(***ObjectGroundTruthAnnotator does not meet dependencies*****)
    return()
endif()


if(NOT HAVE_QT)
    message(***ObjectGroundTruthAnnotator requires Qt*****)
    return()
endif()


v4r_include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

set(SOURCES
        main_window.cpp
        GT6DOF.cpp
)

set(HEADERS
        main_window.h
)

set(FORMS
        mainwindow.ui
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
qt5_wrap_ui(FORMS_MOC ${FORMS})


add_definitions(-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
add_definitions(-Dlinux -D__x86_64__)


add_executable(6DOF_ObjectGroundTruthAnnotator ${SOURCES} ${HEADERS_MOC} ${FORMS_MOC})
target_link_libraries(6DOF_ObjectGroundTruthAnnotator ${GT6DOF_DEPS} qt glog)

INSTALL(TARGETS 6DOF_ObjectGroundTruthAnnotator
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        )
