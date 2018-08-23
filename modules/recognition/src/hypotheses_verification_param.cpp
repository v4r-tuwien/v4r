#include <v4r/recognition/hypotheses_verification_param.h>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

namespace v4r {

void HV_Parameter::init(boost::program_options::options_description& desc, const std::string& section_name) {
  desc.add_options()((section_name + ".icp_iterations").c_str(),
                     po::value<int>(&icp_iterations_)->default_value(icp_iterations_),
                     "number of icp iterations. If 0, no pose refinement will be done");
  desc.add_options()((section_name + ".icp_max_correspondence").c_str(),
                     po::value<float>(&icp_max_correspondence_)->default_value(icp_max_correspondence_), "");
  desc.add_options()(
      (section_name + ".min_Euclidean_dist_between_centroids").c_str(),
      po::value<float>(&min_Euclidean_dist_between_centroids_)->default_value(min_Euclidean_dist_between_centroids_),
      "minimum Euclidean distances in meter between the centroids of two hypotheses of the same object model to be "
      "treated "
      "separately");
  desc.add_options()(
      (section_name + ".min_angular_degree_dist_between_hypotheses").c_str(),
      po::value<float>(&min_angular_degree_dist_between_hypotheses_)
          ->default_value(min_angular_degree_dist_between_hypotheses_),
      "minimum angular distance in degree between two hypotheses of the same object model to be treated separately");
  desc.add_options()(
      (section_name + ".clutter_regularizer").c_str(),
      po::value<float>(&clutter_regularizer_)
          ->default_value(clutter_regularizer_, boost::str(boost::format("%.2e") % clutter_regularizer_)),
      "The penalty multiplier used to penalize unexplained scene points within the clutter influence radius "
      "<i>radius_neighborhood_clutter_</i> of an explained scene point when they belong to the same smooth "
      "segment.");
  desc.add_options()(
      (section_name + ".inlier_threshold_color").c_str(),
      po::value<float>(&inlier_threshold_color_)
          ->default_value(inlier_threshold_color_, boost::str(boost::format("%.2e") % inlier_threshold_color_)),
      "allowed chrominance (AB channel of LAB color space) variance for a point of an object hypotheses to be "
      "considered explained by a corresponding scene point (between 0 and 1, the higher the fewer objects get "
      "rejected)");
  desc.add_options()((section_name + ".inlier_threshold_normals_dotp").c_str(),
                     po::value<float>(&inlier_threshold_normals_dotp_)
                         ->default_value(inlier_threshold_normals_dotp_,
                                         boost::str(boost::format("%.2e") % inlier_threshold_normals_dotp_)),
                     "");
  desc.add_options()(
      (section_name + ".sigma_xyz").c_str(),
      po::value<float>(&sigma_xyz_)->default_value(sigma_xyz_, boost::str(boost::format("%.2e") % sigma_xyz_)), "");
  desc.add_options()(
      (section_name + ".sigma_color").c_str(),
      po::value<float>(&sigma_color_)->default_value(sigma_color_, boost::str(boost::format("%.2e") % sigma_color_)),
      "allowed illumination (L channel of LAB color space) variance for a point of an object hypotheses to be "
      "considered explained by a corresponding scene point (between 0 and 1, the higher the fewer objects get "
      "rejected)");
  desc.add_options()((section_name + ".sigma_normals").c_str(),
                     po::value<float>(&sigma_normals_)
                         ->default_value(sigma_normals_, boost::str(boost::format("%.2e") % sigma_normals_)),
                     "variance for surface normals");
  desc.add_options()((section_name + ".histogram_specification").c_str(),
                     po::value<bool>(&use_histogram_specification_)->default_value(use_histogram_specification_), " ");
  desc.add_options()((section_name + ".ignore_color").c_str(),
                     po::value<bool>(&ignore_color_even_if_exists_)->default_value(ignore_color_even_if_exists_), " ");
  desc.add_options()(
      (section_name + ".color_comparison_method").c_str(),
      po::value<ColorComparisonMethod>(&color_comparison_method_)->default_value(color_comparison_method_),
      "method used for color comparison (0... CIE76, 1... CIE94, 2... CIEDE2000)");
  desc.add_options()(
      (section_name + ".inlier_threshold").c_str(),
      po::value<float>(&inlier_threshold_xyz_)
          ->default_value(inlier_threshold_xyz_, boost::str(boost::format("%.2e") % inlier_threshold_xyz_)),
      "Represents the maximum distance between model and scene points in order to state that a scene point is "
      "explained by a model point. Valid model points that do not have any corresponding scene point within this "
      "threshold are considered model outliers");
  desc.add_options()(
      (section_name + ".occlusion_threshold").c_str(),
      po::value<float>(&occlusion_thres_)
          ->default_value(occlusion_thres_, boost::str(boost::format("%.2e") % occlusion_thres_)),
      "Threshold for a point to be considered occluded when model points are back-projected to the scene ( "
      "depends e.g. on sensor noise)");
  desc.add_options()((section_name + ".resolution_mm").c_str(),
                     po::value<int>(&resolution_mm_)->default_value(resolution_mm_),
                     "The resolution of models and scene used to verify hypotheses (in milli meters)");
  desc.add_options()(
      (section_name + ".min_visible_ratio").c_str(),
      po::value<float>(&min_visible_ratio_)
          ->default_value(min_visible_ratio_, boost::str(boost::format("%.2e") % min_visible_ratio_)),
      "defines how much of the object has to be visible in order to be included in the verification stage");
  desc.add_options()(
      (section_name + ".min_ratio_smooth_cluster_explained").c_str(),
      po::value<float>(&min_ratio_cluster_explained_)
          ->default_value(min_ratio_cluster_explained_,
                          boost::str(boost::format("%.2e") % min_ratio_cluster_explained_)),
      " defines the minimum ratio a smooth cluster has to be explained by the visible points (given there are at "
      "least 100 points)");
  desc.add_options()((section_name + ".eps_angle_threshold").c_str(),
                     po::value<float>(&eps_angle_threshold_deg_)->default_value(eps_angle_threshold_deg_),
                     "smooth clustering parameter for the angle threshold");
  desc.add_options()((section_name + ".cluster_tolerance").c_str(),
                     po::value<float>(&cluster_tolerance_)->default_value(cluster_tolerance_),
                     "smooth clustering parameter for cluster_tolerance");
  desc.add_options()((section_name + ".curvature_threshold").c_str(),
                     po::value<float>(&curvature_threshold_)->default_value(curvature_threshold_),
                     "smooth clustering parameter for curvate");
  desc.add_options()(
      (section_name + ".check_smooth_clusters").c_str(),
      po::value<bool>(&check_smooth_clusters_)->default_value(check_smooth_clusters_),
      "if true, checks for each hypotheses how well it explains occupied smooth surface patches. Hypotheses are "
      "rejected if they only partially explain smooth clusters.");
  desc.add_options()((section_name + ".do_smoothing").c_str(),
                     po::value<bool>(&do_smoothing_)->default_value(do_smoothing_), "");
  desc.add_options()((section_name + ".do_erosion").c_str(), po::value<bool>(&do_erosion_)->default_value(do_erosion_),
                     "");
  desc.add_options()((section_name + ".erosion_radius").c_str(),
                     po::value<int>(&erosion_radius_)->default_value(erosion_radius_), "");
  desc.add_options()((section_name + ".smoothing_radius").c_str(),
                     po::value<int>(&smoothing_radius_)->default_value(smoothing_radius_), "");
  desc.add_options()((section_name + ".weight_xyz").c_str(), po::value<float>(&w_xyz_)->default_value(w_xyz_), "");
  desc.add_options()((section_name + ".weight_normals").c_str(),
                     po::value<float>(&w_normals_)->default_value(w_normals_), "");
  desc.add_options()((section_name + ".weight_color").c_str(), po::value<float>(&w_color_)->default_value(w_color_),
                     "");
  desc.add_options()((section_name + ".normal_method").c_str(),
                     po::value<NormalEstimatorType>(&normal_method_)->default_value(normal_method_), "");
  desc.add_options()((section_name + ".max_iterations").c_str(),
                     po::value<int>(&max_iterations_)->default_value(max_iterations_), "");
  desc.add_options()((section_name + ".min_points").c_str(),
                     po::value<size_t>(&min_points_)->default_value(min_points_), "");
  desc.add_options()((section_name + ".z_adaptive").c_str(), po::value<bool>(&z_adaptive_)->default_value(z_adaptive_),
                     "");
  desc.add_options()((section_name + ".min_pts_smooth_cluster_to_be_explained").c_str(),
                     po::value<size_t>(&min_pts_smooth_cluster_to_be_epxlained_)
                         ->default_value(min_pts_smooth_cluster_to_be_epxlained_),
                     "minimum number of points a cluster need to be explained by model "
                     "points to be considered for a check (avoids the fact that boundary "
                     " points of a smooth region can be close to an object");
  desc.add_options()((section_name + ".min_fitness").c_str(),
                     po::value<float>(&min_fitness_)->default_value(min_fitness_), "");
}
}  // namespace v4r