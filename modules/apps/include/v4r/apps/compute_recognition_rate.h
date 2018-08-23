#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <v4r/common/intrinsics.h>
#include <v4r/common/pcl_visualization_utils.h>
#include <v4r/core/macros.h>
#include <v4r/io/filesystem.h>

#include <fstream>
#include <iostream>
#include <sstream>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

namespace bf = boost::filesystem;

namespace v4r {
namespace apps {

std::vector<std::vector<int>> PermGenerator(int n, int k);

struct Hypothesis {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix4f pose;
  float occlusion;
};

class V4R_EXPORTS RecognitionEvaluator {
 public:
  struct Parameter {
    float rotation_error_threshold_deg_ = 30.f;
    float translation_error_threshold_m_ = 0.05f;
    float occlusion_threshold_ = 0.95f;
    bf::path out_dir = "recognition_rates";
    bf::path gt_dir;
    bf::path or_dir;
    bf::path models_dir;
    bf::path test_dir;
    bf::path img_out_dir = "recognition_output_images";
    bf::path camera_calibration_file = v4r::io::getConfigDir() / "rgb_calibration.yaml";
    bool visualize_ = false;
    bool visualize_errors_only_ = false;
    bool save_images_to_disk_ = false;
    bool highlight_errors_ = false;
    bool use_generated_hypotheses_ = false;
    v4r::Intrinsics cam = v4r::Intrinsics::PrimeSense();

    Parameter() {}

    /**
     * @brief init set directories and stuff from (boost) console arguments
     * @param params parameters (boost program options)
     */
    void init(boost::program_options::options_description &desc);
  };

 private:
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointXYZRGBNormal ModelT;
  Parameter param_;

  struct Model {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    pcl::PointCloud<ModelT>::Ptr cloud;
    Eigen::Vector4f centroid;
    bool is_rotation_invariant_ = false;  // in-plane (x/y)
    bool is_rotational_symmetric_ = false;
  };

  pcl::visualization::PCLVisualizer::Ptr vis_;
  int vp1_, vp2_, vp3_;

  std::map<std::string, Model> models;

  PCLVisualizationParams::Ptr vis_params_;

  void loadModels();

  std::vector<std::string>
      rotational_symmetric_objects_;  ///< name of objects which are rotational symmetric with respect to xy plane
  std::vector<std::string>
      rotational_invariant_objects_;  ///< name of objects which are rotational invariant with respect to xy plane

 public:
  RecognitionEvaluator(const Parameter &p = Parameter()) : param_(p) {
    rotational_invariant_objects_ = {
        "toilet_paper",     "red_mug_white_spots", "muller_milch_shoko", "muller_milch_banana",
        "coffee_container", "object_11",           "object_19",          "object_29",
        "object_32",        "object_18",           "object_22",          "object_23",
        "object_25",        "object_27",           "object_28"  // last row are debatable objects - not completely
                                                                // invariant but very hard to distinguish
    };

    rotational_symmetric_objects_ = {
        "jasmine_green_tea", "fruchtmolke", "asus_box",  "object_10", "object_26",
        "object_35",         "object_01",   "object_02", "object_03", "object_09",
        "object_08",         "object_30",   "object_31", "object_33"  // last row are debatable objects - not 100%
                                                                      // symmetric but very hard to distinguish

    };
    vis_params_.reset(new PCLVisualizationParams());
    vis_params_->bg_color_ = Eigen::Vector3i(255, 255, 255);
    vis_params_->coordinate_axis_scale_ = 0.04f;
  }

  // =======  DECLARATIONS ===================
  /**
   * @brief computeError compute translation error for a given pose
   * @param[in] pose_a
   * @param[in] pose_b
   * @param[in] centroid of object model in model coordinate system
   * @param[out] trans_error translation error
   * @param[out] rot_error rotation error
   * @param[in] true if model is invariant for rotation around z
   * @param[in] true if model is symmetric for rotation around z (180deg periodic)
   * @return boolean indicitating if error is outside given threshold
   */
  bool computeError(const Eigen::Matrix4f &pose_a, const Eigen::Matrix4f &pose_b, const Eigen::Vector4f &centroid_model,
                    float &trans_error, float &rot_error, bool is_rotation_invariant, bool is_rotational_symmetric);

  /**
   * @brief selectBestMatch computes the best matches for a set of hypotheses. This
   * tackles the problem when multiple object instances are present in the scene and
   * the question is which ground-truth object belongs to which recognized object. By
   * permuating through all possible solutions, it finds the match which gives the
   * best f-score taking into account to be neglected hypotheses due to occlusion.
   * @param rec_hyps recognized hypotheses
   * @param gt_hyps ground-truth hypotheses
   * @param centroid of object model in model coordinate system
   * @param tp true positives for best match
   * @param fp false positives for best match
   * @param fn false negatives for best match
   * @param translation_errors accumulated translation error for best match
   * @param rotational_errors accumulated rotational error for best match
   * @param[in] true if model is invariant for rotation around z
   * @param[in] true if model is symmetric for rotation around z (180deg periodic)
   * @return best match for the given hypotheses. First index corresponds to element in
   * given recognition hypothesis, second index to ground-truth hypothesis
   */
  std::vector<std::pair<int, int>> selectBestMatch(
      const std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>> &rec_hyps,
      const std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>> &gt_hyps,
      const Eigen::Vector4f &model_centroid, size_t &tp, size_t &fp, size_t &fn, std::vector<float> &translation_errors,
      std::vector<float> &rotational_errors, bool is_rotation_invariant, bool is_rotational_symmetric);

  std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> readHypothesesFromFile(
      const bf::path &filename);

  /**
   * @brief setRotationalInvariantObjects
   * @param rot_invariant_objects name of objects which are rotational invariant with respect to xy plane
   */
  void setRotationalInvariantObjects(const std::vector<std::string> &rot_invariant_objects) {
    rotational_invariant_objects_ = rot_invariant_objects;
  }

  /**
   * @brief setRotationalSymmetricObjects
   * @param rotational_symmetric_objects name of objects which are rotational symmetric with respect to xy plane
   */
  void setRotationalSymmetricObjects(const std::vector<std::string> &rotational_symmetric_objects) {
    rotational_symmetric_objects_ = rotational_symmetric_objects;
  }

  void compute_recognition_rate(size_t &total_tp, size_t &total_fp, size_t &total_fn);
  float compute_recognition_rate_over_occlusion();  ///< basically checks for each ground-truth object if there exists a
                                                    /// corresponding recognized object
  void
  checkIndividualHypotheses();  ///< check for each recognized object if there is a corresponding ground-truth object<w

  /**
   * @brief visualize results for an input cloud with ground-truth and recognized object models
   * @param input_cloud cloud of the input scene
   * @param gt_path file path to the ground-truth annotation file
   * @param recognition_results_path file path to the results stored in the format of the annotation file
   */
  void visualizeResults(const typename pcl::PointCloud<PointT>::Ptr &input_cloud, const bf::path &gt_path,
                        const bf::path &recognition_results_path);

  Eigen::MatrixXi compute_confusion_matrix();
};
}  // namespace apps
}  // namespace v4r
