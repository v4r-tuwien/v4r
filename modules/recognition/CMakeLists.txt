v4r_add_module(
  DESCRIPTION "Recognition"
  REQUIRED v4r_core v4r_common v4r_io v4r_features v4r_registration v4r_segmentation v4r_keypoints v4r_ml pcl pcl_1_8 opencv glog boost
  OPTIONAL v4r_rendering siftgpu v4r_reconstruction
)
