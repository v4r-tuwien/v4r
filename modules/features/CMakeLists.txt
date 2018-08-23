v4r_add_module(
        DESCRIPTION "Features"
        REQUIRED v4r_core v4r_common v4r_keypoints v4r_io pcl
        OPTIONAL siftgpu
        HEADERS
        ComputeImGDescOrientations.h
        ComputeImGradientDescriptors.h
        FeatureDetector.h
        FeatureDetectorHeaders.h
        FeatureDetector_KD_BRISK.h
        FeatureDetector_KD_FAST_IMGD.h
        FeatureDetector_KD_ORB.h
        FeatureDetector_K_HARRIS.h
        FeatureDetector_K_MSER.h
        FeatureSelection.h
        ImGDescOrientation.h
        ImGradientDescriptor.h
        all_headers.h
        color_ourcvfh_estimator.h
        colorshot_local_estimator.h
        esf_estimator.h
        global_color_estimator.h
        global_concatenated.h
        global_estimator.h
        global_simple_shape_estimator.h
        local_estimator.h
        local_estimator_2d.h
        ourcvfh_estimator.h
        pcl_ourcvfh.h
        rops_local_estimator.h
        shot_local_estimator.h
        types.h
        impl/pcl_ourcvfh.hpp
        SOURCES
        types.cpp
        ComputeImGDescOrientations.cpp
        ComputeImGradientDescriptors.cpp
        FeatureDetector.cpp
        FeatureDetector_KD_BRISK.cpp
        FeatureDetector_KD_FAST_IMGD.cpp
        FeatureDetector_KD_ORB.cpp
        FeatureDetector_K_HARRIS.cpp
        FeatureDetector_K_MSER.cpp
        FeatureSelection.cpp
        ImGDescOrientation.cpp
        ImGradientDescriptor.cpp
        esf_estimator.cpp
        global_color_estimator.cpp
        global_concatenated.cpp
        global_simple_shape_estimator.cpp
        local_estimator_2d.cpp
        ourcvfh_estimator.cpp
        pcl_ourcvfh.cpp
        rops_local_estimator.cpp
        shot_local_estimator.cpp
)

v4r_add_submodule(
        "alexnet_global_estimator"
        REQUIRED caffe
        HEADERS
        global_alexnet_cnn_estimator.h
        SOURCES
        global_alexnet_cnn_estimator.cpp
)

v4r_add_submodule(
        "siftgpu_detector"
        REQUIRED siftgpu
        HEADERS
        FeatureDetector_KD_SIFTGPU.h
        SOURCES
        FeatureDetector_KD_SIFTGPU.cpp
)

v4r_add_submodule(
        "akaze_local_estimator"
        REQUIRED opencv>=3
        HEADERS
        FeatureDetector_KD_AKAZE.h
        SOURCES
        FeatureDetector_KD_AKAZE.cpp
)


v4r_add_submodule(
        "opencv_xfeatures2d"
        REQUIRED opencv
        PRIVATE_REQUIRED opencv_xfeatures2d
        HEADERS
        FeatureDetector_KD_SURF.h
        FeatureDetector_D_FREAK.h
        SOURCES
        FeatureDetector_KD_SURF.cpp
        FeatureDetector_D_FREAK.cpp
)

v4r_add_submodule(
        "sift_local_estimator"
        REQUIRED opencv
        PRIVATE_REQUIRED opencv_xfeatures2d
        HEADERS
        FeatureDetector_KD_CVSIFT.h
        SOURCES
        FeatureDetector_KD_CVSIFT.cpp
)