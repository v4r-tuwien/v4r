if(BUILD_UTILITY_TOOLS)

  if(CMAKE_COMPILER_IS_GNUCXX AND NOT ENABLE_NOISY_WARNINGS)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
  endif()

  SET(sample_kind tool)
  SET(sample_KIND TOOL)

  SET(V4R_DEPS v4r_common v4r_io)
  V4R_DEFINE_CPP_EXAMPLE(convertOldModelDatabaseToNew)

  SET(V4R_DEPS v4r_recognition)
  V4R_DEFINE_CPP_EXAMPLE(create_annotated_images_from_recognition_gt_data)

  SET(V4R_DEPS v4r_common v4r_io v4r_rendering)
  V4R_DEFINE_CPP_EXAMPLE(create_classification_db_from_ply_files)

  SET(V4R_DEPS v4r_common v4r_io)
  V4R_DEFINE_CPP_EXAMPLE(pcd2png)

  SET(V4R_DEPS v4r_recognition)
  V4R_DEFINE_CPP_EXAMPLE(pcd_ground_truth_labelling)

  SET(V4R_DEPS v4r_camera_tracking_and_mapping)
  V4R_DEFINE_CPP_EXAMPLE(pointcloud2mesh)

  SET(V4R_DEPS v4r_common v4r_io)
  V4R_DEFINE_CPP_EXAMPLE(save_pose_into_pcd)

  SET(V4R_DEPS v4r_common v4r_io)
  V4R_DEFINE_CPP_EXAMPLE(view_all_point_clouds_in_folder)

  SET(V4R_DEPS pcl boost)
  V4R_DEFINE_CPP_EXAMPLE(visualize_hypothesis)

  SET(V4R_DEPS v4r_io pcl opencv)
  V4R_DEFINE_CPP_EXAMPLE(create_HV_registration_image_mask)

  SET(V4R_DEPS v4r_common v4r_io)
  V4R_DEFINE_CPP_EXAMPLE(grab)

  SET(V4R_DEPS v4r_common v4r_io v4r_surface)
  V4R_DEFINE_CPP_EXAMPLE(texture_mesh)

  SET(V4R_DEPS v4r_common glog)
  V4R_DEFINE_CPP_EXAMPLE(motion_blur_detection)

  SET(V4R_DEPS v4r_common v4r_calibration glog)
  V4R_DEFINE_CPP_EXAMPLE(apply_auto_calibration)
endif()

if (INSTALL_C_UTILITY_TOOLS)
  file(GLOB C_SAMPLES *.c *.cpp *.jpg *.png *.data makefile.* build_all.sh *.dsp *.cmd )
  install(FILES ${C_SAMPLES}
          DESTINATION ${V4R_SAMPLES_SRC_INSTALL_PATH}/cpp
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ COMPONENT samples)
endif()
