#include <glog/logging.h>
#include <v4r/apps/ObjectRecognizerParameter.h>

namespace v4r {
namespace apps {

void FeatureDetectorParameter::init(boost::program_options::options_description &desc) {
#if HAVE_V4R_FEATURES_AKAZE_LOCAL_ESTIMATOR
  akaze_.init(desc, "akaze");
#endif

#if HAVE_V4R_FEATURES_OPENCV_XFEATURES2D
  surf_.init(desc, "surf");
  freak_.init(desc, "freak");
#endif

  brisk_.init(desc, "brisk");
  orb_.init(desc, "orb");
  // mser_.init(desc, "mser");
}

void ObjectRecognizerParameter::init(boost::program_options::options_description &desc,
                                     const std::string &section_name) {
  desc.add_options()((section_name + ".do_local_2d").c_str(),
                     po::value<bool>(&do_local_2d_)->default_value(do_local_2d_),
                     "Enable local 2D recognition pipeline");
  desc.add_options()((section_name + ".local_2D_feature_estimator").c_str(),
                     po::value<v4r::FeatureDetector::Type>(&local_2D_feat_est_)->default_value(local_2D_feat_est_),
                     "Local 2D feature estimator type");
  desc.add_options()(
      (section_name + ".local_2D_keypoint_detector").c_str(),
      po::value<v4r::FeatureDetector::Type>(&local_2D_feat_detector_)->default_value(local_2D_feat_detector_),
      "Local 2D keypoint detector type");
  desc.add_options()((section_name + ".do_shot").c_str(), po::value<bool>(&do_shot_)->default_value(do_shot_),
                     "Enable local 3D SHOT pipeline.");
  desc.add_options()((section_name + ".global_feature_types").c_str(),
                     po::value<std::vector<int>>(&global_feature_types_)->multitoken(), "");
  desc.add_options()((section_name + ".keypoint_support_radii").c_str(),
                     po::value<std::vector<float>>(&keypoint_support_radii_)->multitoken(),
                     "support radi used for describing SHOT features");
  desc.add_options()((section_name + ".remove_planes").c_str(),
                     po::value<bool>(&remove_planes_)->default_value(remove_planes_),
                     "if enabled, removes the dominant plane in the input cloud (given there are at least N inliers)");
  desc.add_options()(
      (section_name + ".normal_computation_method").c_str(),
      po::value<NormalEstimatorType>(&normal_computation_method_)->default_value(normal_computation_method_),
      "normal computation method (PCL_DEFAULT, PCL_INTEGRAL_NORMAL, Z_ADAPTIVE)");
  desc.add_options()(
      (section_name + ".segmentation_method").c_str(),
      po::value<SegmentationType>(&segmentation_method_)->default_value(segmentation_method_),
      "Segmentation method (CONNECTED_COMPONENTS_2D, EUCLIDEAN, ORGANIZED_CONNECTED_COMPONENTS, SMOOTH_EUCLIDEAN)");
  desc.add_options()((section_name + ".chop_z").c_str(), po::value<double>(&chop_z_)->default_value(chop_z_),
                     "Cut-off distance in z-direction of the camera");
  desc.add_options()((section_name + ".min_plane_inliers").c_str(),
                     po::value<size_t>(&min_plane_inliers_)->default_value(min_plane_inliers_),
                     "required inliers for plane to be removed");
  desc.add_options()((section_name + ".plane_inlier_threshold").c_str(),
                     po::value<float>(&plane_inlier_threshold_)->default_value(plane_inlier_threshold_),
                     "maximum distance for plane inliers");
  desc.add_options()(
      (section_name + ".shot_keypoint_extractor_method").c_str(),
      po::value<KeypointType>(&shot_keypoint_extractor_method_)->default_value(shot_keypoint_extractor_method_),
      "Keypoint extraction method used for SHOT features");
  desc.add_options()((section_name + ".classification_methods").c_str(),
                     po::value<std::vector<ClassifierType>>(&classification_methods_)->multitoken(), "");
  desc.add_options()((section_name + ".use_multiview").c_str(),
                     po::value<bool>(&use_multiview_)->default_value(use_multiview_), "");
  desc.add_options()((section_name + ".use_multiview_hv").c_str(),
                     po::value<bool>(&use_multiview_hv_)->default_value(use_multiview_hv_), "");
  desc.add_options()((section_name + ".use_multiview_with_kp_correspondence_transfer").c_str(),
                     po::value<bool>(&use_multiview_with_kp_correspondence_transfer_)
                         ->default_value(use_multiview_with_kp_correspondence_transfer_),
                     "if true, transfers keypoints instead of full hypotheses (see Faeulhammer et al, ICRA 2015)");
  desc.add_options()((section_name + ".multiview_max_views").c_str(),
                     po::value<size_t>(&multiview_max_views_)->default_value(multiview_max_views_),
                     "Maximum number of views used for multi-view recognition (if more views are available, "
                     "information from oldest views will be ignored)");
  desc.add_options()((section_name + ".use_change_detection").c_str(),
                     po::value<bool>(&use_change_detection_)->default_value(use_change_detection_), "");
  desc.add_options()(
      (section_name + ".remove_non_upright_objects").c_str(),
      po::value<bool>(&remove_non_upright_objects_)->default_value(remove_non_upright_objects_),
      "remove all hypotheses that are not standing upright on a support plane (support plane extraction must be "
      "enabled)");
  desc.add_options()((section_name + ".icp_iterations").c_str(),
                     po::value<size_t>(&icp_iterations_)->default_value(icp_iterations_),
                     "ICP iterations. Only used if hypotheses are not verified. Otherwise ICP is done inside HV");
  desc.add_options()("skip_verification", po::bool_switch(&skip_verification_),
                     "if true, skips verification (only hypotheses generation)");
  desc.add_options()("visualization.hv_vis_cues", po::bool_switch(&visualize_hv_go_cues_),
                     "If set, visualizes cues computed at the"
                     "hypothesis verification stage such as inlier, outlier points. Mainly used for debugging.");
  desc.add_options()("visualization.hv_vis_model_cues", po::bool_switch(&visualize_hv_model_cues_),
                     "If set, visualizes the model cues in the hypotheses verification stage.");
  desc.add_options()("visualization.hv_vis_pairwise_cues", po::bool_switch(&visualize_hv_pairwise_cues_),
                     "If set, visualizes the pairwise cues in the hypotheses verification stage.");
  desc.add_options()("visualization.rec_visualize_keypoints", po::bool_switch(&visualize_keypoints_),
                     "If set, visualizes detected keypoints.");
  desc.add_options()("visualization.rec_visualize_global_pipeline", po::bool_switch(&visualize_global_results_),
                     "If set, visualizes segments and results from global pipeline.");
  desc.add_options()(
      "visualization.layout", po::value<ObjRecoVisLayoutStyle>(&vis_layout_)->default_value(vis_layout_),
      "defines the layout of the visualization (FULL... full, INTERMEDIATE... only show input and verified,"
      " SIMPLE... show input, processed, and verified)");
  plane_filter_.init(desc, "plane_filter");
  local_rec_pipeline_.init(desc, "local_rec");
  gc_.init(desc, "cg");
  global_rec_.init(desc, "global_rec");
  hv_.init(desc, "hv");
  rendering_.init(desc, "rendering");
  feat_params_.init(desc);
  local_2d_pipeline_.init(desc, "local_2d_pipeline");
  shot_pipeline_.init(desc, "shot_pipeline");

  multiview_.init(desc, "mv");
  visualization_->init(desc, "visualization");
  visualization_->bg_color_ = Eigen::Vector3i(255, 255, 255);    // TODO: add this to init
  visualization_->text_color_ = Eigen::Vector3f(0.f, 0.f, 0.f);  // TODO: add this to init
}

void ObjectRecognizerParameter::validate() {
  if (global_feature_types_.size() != classification_methods_.size()) {
    size_t minn = std::min<size_t>(global_feature_types_.size(), classification_methods_.size());

    LOG(ERROR) << "The given parameter for feature types, classification methods "
               << "and configuration files for global recognition are not the same size!";
    if (minn)
      LOG(ERROR) << " Will only use the first " << minn << " global recognizers for which all three elements are set! ";
    else
      LOG(ERROR) << "Global recognition is disabled!";

    global_feature_types_.resize(minn);
    classification_methods_.resize(minn);
  }
}
}  // namespace apps
}  // namespace v4r