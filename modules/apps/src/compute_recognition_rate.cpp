#include <v4r/apps/compute_recognition_rate.h>
#include <v4r/common/pcl_opencv.h>
#include <v4r/io/filesystem.h>

#include <glog/logging.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <numeric>

#include <glog/logging.h>

namespace po = boost::program_options;

namespace v4r {
namespace apps {

/**
 * @brief readHypothesesFromFile reads annotations from a text file
 * @param filename filename
 * @return stores hypotheses into Hypothesis class for each object model
 */
std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> readHypothesesFromFile(
    const bf::path &filename);
// ==========================================

bool RecognitionEvaluator::computeError(const Eigen::Matrix4f &pose_a, const Eigen::Matrix4f &pose_b,
                                        const Eigen::Vector4f &centroid_model, float &trans_error, float &rot_error,
                                        bool is_rotation_invariant, bool is_rotational_symmetric) {
  const Eigen::Vector4f centroid_a = pose_a * centroid_model;
  const Eigen::Vector4f centroid_b = pose_b * centroid_model;

  const Eigen::Matrix3f rot_a = pose_a.block<3, 3>(0, 0);
  const Eigen::Matrix3f rot_b = pose_b.block<3, 3>(0, 0);

  const Eigen::Vector3f rotX_a = rot_a * Eigen::Vector3f::UnitX();
  const Eigen::Vector3f rotX_b = rot_b * Eigen::Vector3f::UnitX();
  //    const Eigen::Vector3f rotY_a = rot_a * Eigen::Vector3f::UnitY();
  //    const Eigen::Vector3f rotY_b = rot_b * Eigen::Vector3f::UnitY();
  const Eigen::Vector3f rotZ_a = rot_a * Eigen::Vector3f::UnitZ();
  const Eigen::Vector3f rotZ_b = rot_b * Eigen::Vector3f::UnitZ();

  //    float angleX = pcl::rad2deg( acos( rotX_a.dot(rotX_b) ) );
  //    float angleY = pcl::rad2deg( acos( rotY_a.dot(rotY_b) ) );
  float dotpz = rotZ_a.dot(rotZ_b);
  dotpz = std::min(0.9999999f, std::max(-0.999999999f, dotpz));
  float angleZ = pcl::rad2deg(acos(dotpz));

  float angleXY = 0.f;
  if (!is_rotation_invariant) {
    float dotpxy = rotX_a.dot(rotX_b);
    dotpxy = std::min(0.9999999f, std::max(-0.999999999f, dotpxy));
    angleXY = pcl::rad2deg(acos(dotpxy));

    if (is_rotational_symmetric)
      angleXY = std::min<float>(angleXY, fabs(180.f - angleXY));
  }

  //    std::cout << " error_rotxy: " << angleXY << " error_rotx: " << angleX << " error_roty: " << angleY << "
  //    error_rotz: " << angleZ << std::endl;

  trans_error = (centroid_a.head(3) - centroid_b.head(3)).norm();
  rot_error = std::max<float>(angleXY, angleZ);

  VLOG(1) << "translation error: " << trans_error << ", rotational error: " << angleXY << "(xy), " << angleZ
          << "(z), is rotational invariant? " << is_rotation_invariant << ", is rotational symmetric? "
          << is_rotational_symmetric;

  return trans_error > param_.translation_error_threshold_m_ || rot_error > param_.rotation_error_threshold_deg_;
}

std::vector<std::vector<int>> PermGenerator(int n, int k) {
  std::vector<std::vector<int>> possible_permutations;

  std::vector<int> d(n);
  std::iota(d.begin(), d.end(), 0);
  do {
    std::vector<int> p(k);
    for (int i = 0; i < k; i++)
      p[i] = d[i];

    possible_permutations.push_back(p);
    std::reverse(d.begin() + k, d.end());
  } while (next_permutation(d.begin(), d.end()));

  return possible_permutations;
}

std::vector<std::pair<int, int>> RecognitionEvaluator::selectBestMatch(
    const std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>> &rec_hyps,
    const std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>> &gt_hyps, const Eigen::Vector4f &model_centroid,
    size_t &tp, size_t &fp, size_t &fn, std::vector<float> &translation_errors, std::vector<float> &rotational_errors,
    bool is_rotation_invariant, bool is_rotational_symmetric) {
  float best_fscore = -1;
  tp = 0, fp = 0, fn = 0;

  std::vector<std::pair<int, int>> best_match;

  size_t k = std::min(rec_hyps.size(), gt_hyps.size());
  size_t n = std::max(rec_hyps.size(), gt_hyps.size());

  std::vector<std::vector<int>> perms = PermGenerator(n, k);

  for (const std::vector<int> &perm : perms) {
    std::vector<std::pair<int, int>> rec2gt_matches(k);

    size_t tp_tmp = 0, fp_tmp = 0, fn_tmp = 0;

    if (rec_hyps.size() < n) {
      boost::dynamic_bitset<> taken_gts(n, 0);

      for (size_t i = 0; i < k; i++) {
        std::pair<int, int> &p = rec2gt_matches[i];
        p.first = i;
        p.second = perm[i];
        taken_gts.set(perm[i]);
      }

      for (size_t i = 0; i < n; i++) {
        if (!taken_gts[i] && gt_hyps[i].occlusion < param_.occlusion_threshold_)
          fn_tmp++;
      }
    } else {
      boost::dynamic_bitset<> taken_recs(n, 0);
      for (size_t i = 0; i < k; i++) {
        std::pair<int, int> &p = rec2gt_matches[i];
        p.first = perm[i];
        p.second = i;
        taken_recs.set(perm[i]);
      }
      for (size_t i = 0; i < n; i++) {
        if (!taken_recs[i])
          fp_tmp++;
      }
    }

    std::vector<float> translation_errors_tmp(k);
    std::vector<float> rotational_errors_tmp(k);
    for (size_t i = 0; i < k; i++) {
      const std::pair<int, int> &x = rec2gt_matches[i];
      VLOG(1) << x.first << "/" << x.second << " " << std::endl;

      const Hypothesis &rec_hyp = rec_hyps[x.first];
      const Hypothesis &gt_hyp = gt_hyps[x.second];

      if (computeError(rec_hyp.pose, gt_hyp.pose, model_centroid, translation_errors_tmp[i], rotational_errors_tmp[i],
                       is_rotation_invariant, is_rotational_symmetric)) {
        if (gt_hyp.occlusion < param_.occlusion_threshold_) {
          fn_tmp++;
          fp_tmp++;
          VLOG(1) << "Adding false negative and false positive";
        } else {
          if (translation_errors_tmp[i] >
              param_.translation_error_threshold_m_)  // ignore rotation erros for occluded objects
          {
            fp_tmp++;
            VLOG(1) << "Adding false positive but ignoring false negative due to occlusion";
          } else
            VLOG(1) << "Ignoring due to occlusion";
        }
      } else {
        VLOG(1) << "Adding true positive";
        tp_tmp++;
      }
    }

    float recall = 1.f;
    if (tp_tmp + fn_tmp)  // if there are some ground-truth objects
      recall = (float)tp_tmp / (tp_tmp + fn_tmp);

    float precision = 1.f;
    if (tp_tmp + fp_tmp)  // if there are some recognized objects
      precision = (float)tp_tmp / (tp_tmp + fp_tmp);

    float fscore = 0.f;
    if (precision + recall > std::numeric_limits<float>::epsilon())
      fscore = 2.f * precision * recall / (precision + recall);

    if ((fscore > best_fscore))  // || (fscore==best_fscore && translation_errors_tmp/tp_tmp < translation_errors/tp))
    {
      best_fscore = fscore;
      translation_errors = translation_errors_tmp;
      rotational_errors = rotational_errors_tmp;
      tp = tp_tmp;
      fp = fp_tmp;
      fn = fn_tmp;
      best_match = rec2gt_matches;
    }
  }

  VLOG(1) << "BEST MATCH: ";
  for (auto &x : best_match) {
    VLOG(1) << x.first << "/" << x.second << " ";
  }
  VLOG(1) << std::endl;

  return best_match;
}

std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>>
RecognitionEvaluator::readHypothesesFromFile(const bf::path &filename) {
  std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> hypotheses;

  std::ifstream anno_f(filename.string().c_str());
  std::string line;
  while (std::getline(anno_f, line)) {
    std::istringstream iss(line);
    std::string model_name, occlusion_tmp;

    Hypothesis h;
    iss >> model_name >> occlusion_tmp;
    occlusion_tmp = occlusion_tmp.substr(1, occlusion_tmp.length() - 3);
    std::istringstream os(occlusion_tmp);
    float visible;
    os >> visible;
    h.occlusion = 1.f - visible;

    for (size_t i = 0; i < 16; i++)
      iss >> h.pose(i / 4, i % 4);

    auto pose_it = hypotheses.find(model_name);
    if (pose_it != hypotheses.end())
      pose_it->second.push_back(h);
    else
      hypotheses[model_name] = std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>(1, h);
  }

  return hypotheses;
}

void RecognitionEvaluator::visualizeResults(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
                                            const bf::path &gt_path, const bf::path &recognition_results_path) {
  std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> gt_hyps =
      readHypothesesFromFile(gt_path);
  std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> rec_hyps =
      readHypothesesFromFile(recognition_results_path);

  static pcl::visualization::PCLVisualizer::Ptr vis;
  static int vp1, vp2, vp3;
  if (!vis) {
    vis.reset(new pcl::visualization::PCLVisualizer("results"));
    vis->setBackgroundColor(vis_params_->bg_color_(0), vis_params_->bg_color_(1), vis_params_->bg_color_(2));
    vis->createViewPort(0, 0, 1, 0.33, vp1);
    vis->createViewPort(0, 0.33, 1, 0.66, vp2);
    vis->createViewPort(0, 0.66, 1, 1, vp3);
  }

  vis->removeAllPointClouds();
  vis->removeAllShapes();
#if PCL_VERSION >= 100800
  vis->removeAllCoordinateSystems();
#endif

  input_cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
  input_cloud->sensor_origin_ = Eigen::Vector4f::Zero(4);
  vis->addPointCloud(input_cloud, "scene", vp1);
  vis->addText("scene", 10, 10, 14, vis_params_->text_color_(0), vis_params_->text_color_(1),
               vis_params_->text_color_(2), "scene", vp1);
  vis->addText("ground-truth", 10, 10, 14, vis_params_->text_color_(0), vis_params_->text_color_(1),
               vis_params_->text_color_(2), "gt", vp2);
  vis->addText("recognition results", 10, 10, 14, vis_params_->text_color_(0), vis_params_->text_color_(1),
               vis_params_->text_color_(2), "rec", vp3);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> gray(input_cloud, 255, 255, 255);
  vis->addPointCloud(input_cloud, gray, "input_vp2", vp2);
  vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "input_vp2");
  vis->addPointCloud(input_cloud, gray, "input_vp3", vp3);
  vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "input_vp3");

  for (const auto &m : models) {
    auto it = rec_hyps.find(m.first);
    if (it != rec_hyps.end()) {
      typename pcl::PointCloud<ModelT>::ConstPtr model_cloud = m.second.cloud;
      size_t counter = 0;
      for (const Hypothesis &hyp_vis : it->second) {
        typename pcl::PointCloud<ModelT>::Ptr model_aligned(new pcl::PointCloud<ModelT>());
        pcl::transformPointCloud(*model_cloud, *model_aligned, hyp_vis.pose);
        std::stringstream unique_id;
        unique_id << "rec_" << m.first << "_" << counter++;
        vis->addPointCloud<ModelT>(model_aligned, unique_id.str(), vp3);
      }
    }

    it = gt_hyps.find(m.first);
    if (it != gt_hyps.end()) {
      typename pcl::PointCloud<ModelT>::ConstPtr model_cloud = m.second.cloud;
      size_t counter = 0;
      for (const Hypothesis &hyp_vis : it->second) {
        typename pcl::PointCloud<ModelT>::Ptr model_aligned(new pcl::PointCloud<ModelT>());
        pcl::transformPointCloud(*model_cloud, *model_aligned, hyp_vis.pose);
        std::stringstream unique_id;
        unique_id << "gt_" << m.first << "_" << counter++;
        vis->addPointCloud<ModelT>(model_aligned, unique_id.str(), vp2);
      }
    }
  }

  vis->spin();
}

void RecognitionEvaluator::compute_recognition_rate(size_t &total_tp, size_t &total_fp, size_t &total_fn) {
  std::stringstream description;
  description << "Tool to compute object instance recognition rate." << std::endl
              << "==================================================" << std::endl
              << "This will generate a text file containing:" << std::endl
              << "Column 1: annotation file" << std::endl
              << "Column 2: true positives" << std::endl
              << "Column 3: false positives" << std::endl
              << "Column 4: false negatives" << std::endl
              << "Column 5: accumulated translation error of all true positive objects" << std::endl
              << "Column 6: accumulated rotational error of all true positive objects" << std::endl
              << "Column 7: elapsed time in ms" << std::endl
              << "==================================================" << std::endl
              << "** Allowed options";

  loadModels();
  const bf::path out_path = param_.out_dir / "recognition_results.txt";
  v4r::io::createDirForFileIfNotExist(out_path.string());
  std::ofstream of(out_path.string().c_str());

  std::vector<std::string> annotation_files = v4r::io::getFilesInDirectory(param_.gt_dir, ".*.anno", true);

  total_tp = 0;
  total_fp = 0;
  total_fn = 0;

  for (const std::string anno_file : annotation_files) {
    const bf::path gt_path = param_.gt_dir / anno_file;

    std::string rec_file = anno_file;
    if (param_.use_generated_hypotheses_)
      boost::replace_last(rec_file, ".anno", ".generated_hyps");

    const bf::path rec_path = param_.or_dir / rec_file;

    if (!v4r::io::existsFile(rec_path))
      continue;

    auto gt_hyps = readHypothesesFromFile(gt_path);
    auto rec_hyps = readHypothesesFromFile(rec_path);

    size_t tp_view = 0;
    size_t fp_view = 0;
    size_t fn_view = 0;
    double sum_translation_error_view = 0.;
    double sum_rotational_error_view = 0.;

    if (vis_) {
      vis_->removeAllPointClouds();
      vis_->removeAllShapes();
#if PCL_VERSION >= 100800
      vis_->removeAllCoordinateSystems();
#endif
    }

    pcl::PointCloud<PointT>::Ptr all_hypotheses;
    pcl::PointCloud<PointT>::Ptr all_groundtruth_objects;

    if (param_.save_images_to_disk_) {
      all_hypotheses.reset(new pcl::PointCloud<PointT>);
      all_groundtruth_objects.reset(new pcl::PointCloud<PointT>);
    }

    for (const auto &m : models) {
      std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>> rec_hyps_tmp, gt_hyps_tmp;

      auto it = rec_hyps.find(m.first);
      if (it != rec_hyps.end())
        rec_hyps_tmp = it->second;

      it = gt_hyps.find(m.first);
      if (it != gt_hyps.end())
        gt_hyps_tmp = it->second;

      size_t tp_tmp = 0, fp_tmp = 0, fn_tmp = 0;
      std::vector<float> translation_errors_tmp;
      std::vector<float> rotational_errors_tmp;
      std::vector<std::pair<int, int>> matches;

      if (gt_hyps_tmp.empty() && rec_hyps_tmp.empty())
        continue;
      //            else if( gt_hyps_tmp.empty() )
      //            {
      //                fp_tmp = rec_hyps_tmp.size();
      //                matches = std::vector<std::pair<int,int> > (rec_hyps_tmp.size());
      //                for(size_t r_id=0; r_id<rec_hyps_tmp.size(); r_id++)
      //                    matches[r_id] = std::pair<int,int>(r_id, -1);
      //            }
      //            else if( rec_hyps_tmp.empty() )
      //            {
      //                for(const Hypothesis &gt_hyp : gt_hyps_tmp )
      //                {
      //                    if( gt_hyp.occlusion < occlusion_threshold) // only count if the gt object is not occluded
      //                        fn_tmp++;

      //                    matches = std::vector<std::pair<int,int> > (gt_hyps_tmp.size());
      //                    for(size_t gt_id=0; gt_id<gt_hyps_tmp.size(); gt_id++)
      //                        matches[gt_id] = std::pair<int,int>(-1, gt_id);
      //                }
      //            }
      //            else
      {
        const Eigen::Vector4f &centroid = m.second.centroid;
        matches =
            selectBestMatch(rec_hyps_tmp, gt_hyps_tmp, centroid, tp_tmp, fp_tmp, fn_tmp, translation_errors_tmp,
                            rotational_errors_tmp, m.second.is_rotation_invariant_, m.second.is_rotational_symmetric_);
      }

      tp_view += tp_tmp;
      fp_view += fp_tmp;
      fn_view += fn_tmp;

      float sum_translation_error_tmp = 0.f;
      float sum_rotational_error_tmp = 0.f;

      for (size_t t_id = 0; t_id < translation_errors_tmp.size(); t_id++) {
        if (translation_errors_tmp[t_id] > 0.f) {
          sum_translation_error_tmp += translation_errors_tmp[t_id];
          sum_rotational_error_tmp += rotational_errors_tmp[t_id];
        }
      }
      sum_translation_error_view += sum_translation_error_tmp;
      sum_rotational_error_view += sum_rotational_error_tmp;

      if (param_.visualize_ && !vis_) {
        vis_.reset(new pcl::visualization::PCLVisualizer("results"));
        vis_->createViewPort(0, 0, 1, 0.33, vp1_);
        vis_->createViewPort(0, 0.33, 1, 0.66, vp2_);
        vis_->createViewPort(0, 0.66, 1, 1, vp3_);
        vis_->setBackgroundColor(vis_params_->bg_color_(0), vis_params_->bg_color_(1), vis_params_->bg_color_(2));
        vis_->setBackgroundColor(vis_params_->bg_color_(0), vis_params_->bg_color_(1), vis_params_->bg_color_(2), vp1_);
        vis_->setBackgroundColor(vis_params_->bg_color_(0), vis_params_->bg_color_(1), vis_params_->bg_color_(2), vp2_);
        vis_->setBackgroundColor(vis_params_->bg_color_(0), vis_params_->bg_color_(1), vis_params_->bg_color_(2), vp3_);
      }

      if (param_.visualize_ || param_.save_images_to_disk_) {
        size_t counter = 0;
        for (const Hypothesis &rec_hyp : rec_hyps_tmp) {
          pcl::PointCloud<ModelT>::ConstPtr model_cloud = m.second.cloud;
          pcl::PointCloud<ModelT>::Ptr model_aligned(new pcl::PointCloud<ModelT>());
          pcl::transformPointCloudWithNormals(*model_cloud, *model_aligned, rec_hyp.pose);

          if (param_.save_images_to_disk_) {
            pcl::PointCloud<PointT> model_cloud_rgb;
            pcl::copyPointCloud(*model_aligned, model_cloud_rgb);
            *all_hypotheses += model_cloud_rgb;
          }

          if (param_.visualize_) {
            std::stringstream unique_id;
            unique_id << m.first << "_" << counter++;
#if PCL_VERSION >= 100800
            Eigen::Matrix4f tf_tmp = rec_hyp.pose;
            Eigen::Matrix3f rot_tmp = tf_tmp.block<3, 3>(0, 0);
            Eigen::Vector3f trans_tmp = tf_tmp.block<3, 1>(0, 3);
            Eigen::Affine3f affine_trans;
            affine_trans.fromPositionOrientationScale(trans_tmp, rot_tmp, Eigen::Vector3f::Ones());
            std::stringstream co_id;
            co_id << m.first << "_co_" << counter;
            vis_->addCoordinateSystem(0.1f, affine_trans, co_id.str(), vp3_);
#endif
            vis_->addPointCloud<ModelT>(model_aligned, unique_id.str(), vp3_);
          }
        }

        for (const Hypothesis &gt_hyp : gt_hyps_tmp) {
          typename pcl::PointCloud<ModelT>::ConstPtr model_cloud = m.second.cloud;
          typename pcl::PointCloud<ModelT>::Ptr model_aligned(new pcl::PointCloud<ModelT>());
          pcl::transformPointCloud(*model_cloud, *model_aligned, gt_hyp.pose);

          if (param_.save_images_to_disk_) {
            pcl::PointCloud<PointT> model_cloud_rgb;
            pcl::copyPointCloud(*model_aligned, model_cloud_rgb);
            *all_groundtruth_objects += model_cloud_rgb;
          }

          if (param_.visualize_) {
            std::stringstream unique_id;
            unique_id << m.first << "_" << counter++;
#if PCL_VERSION >= 100800
            Eigen::Matrix4f tf_tmp = gt_hyp.pose;
            Eigen::Matrix3f rot_tmp = tf_tmp.block<3, 3>(0, 0);
            Eigen::Vector3f trans_tmp = tf_tmp.block<3, 1>(0, 3);
            Eigen::Affine3f affine_trans;
            affine_trans.fromPositionOrientationScale(trans_tmp, rot_tmp, Eigen::Vector3f::Ones());
            std::stringstream co_id;
            co_id << m.first << "_co_" << counter;
            vis_->addCoordinateSystem(0.1f, affine_trans, co_id.str(), vp2_);
#endif
            vis_->addPointCloud<ModelT>(model_aligned, unique_id.str(), vp2_);
          }
        }
      }
    }

    if (param_.save_images_to_disk_) {
      if (param_.visualize_errors_only_ && fp_view == 0)
        continue;

      v4r::Intrinsics cam = v4r::Intrinsics::PrimeSense();

      std::string scene_name(anno_file);
      boost::replace_last(scene_name, ".anno", ".pcd");
      const bf::path scene_path = param_.test_dir / scene_name;
      pcl::PointCloud<PointT>::Ptr scene_cloud(new pcl::PointCloud<PointT>);
      pcl::io::loadPCDFile(scene_path.string(), *scene_cloud);
      boost::replace_last(scene_name, ".pcd", "");

      v4r::PCLOpenCVConverter<PointT> ocv;
      ocv.setCameraIntrinsics(cam);
      ocv.setInputCloud(all_hypotheses);
      ocv.setBackgroundColor(255, 255, 255);
      ocv.setRemoveBackground(false);
      cv::Mat all_hypotheses_img = ocv.getRGBImage();
      const bf::path img_path = param_.img_out_dir / scene_name / "all_hypotheses.jpg";
      v4r::io::createDirForFileIfNotExist(img_path.string());
      cv::imwrite(img_path.string(), all_hypotheses_img);
      ocv.setInputCloud(all_groundtruth_objects);
      cv::Mat all_groundtruth_objects_img = ocv.getRGBImage();
      cv::imwrite((param_.img_out_dir / scene_name / "all_groundtruth_objects.jpg").string(),
                  all_groundtruth_objects_img);

      ocv.setInputCloud(scene_cloud);
      cv::Mat scene_cloud_img = ocv.getRGBImage();
      cv::imwrite((param_.img_out_dir / scene_name / "scene.jpg").string(), scene_cloud_img);
    }

    // get time measurements
    size_t time_view = 0;
    {
      std::map<std::string, size_t> time_measurements;
      std::string time_file = anno_file;
      boost::replace_last(time_file, ".anno", ".times");
      const bf::path time_path = param_.or_dir / time_file;

      std::ifstream time_f(time_path.string());
      std::string line;
      while (std::getline(time_f, line)) {
        size_t elapsed_time;
        std::istringstream iss(line);
        iss >> elapsed_time;
        std::stringstream elapsed_time_ss;
        elapsed_time_ss << elapsed_time;

        const std::string time_description = line.substr(elapsed_time_ss.str().length() + 1);
        time_measurements[time_description] = elapsed_time;
      }
      time_f.close();

      for (const auto &t_map : time_measurements) {
        VLOG(1) << t_map.first << ": " << t_map.second;
        if ((t_map.first == "Computing normals") || (t_map.first == "Removing planes") ||
            (t_map.first == "Generation of object hypotheses") || (t_map.first == "Computing noise model") ||
            (t_map.first == "Noise model based cloud integration") ||
            (t_map.first == "Verification of object hypotheses")) {
          VLOG(1) << "count!";
          time_view += t_map.second;
        }
      }
    }

    std::cout << anno_file << ": " << tp_view << " " << fp_view << " " << fn_view << " " << sum_translation_error_view
              << " " << sum_rotational_error_view << " " << time_view << std::endl;
    of << anno_file << " " << tp_view << " " << fp_view << " " << fn_view << " " << sum_translation_error_view << " "
       << sum_rotational_error_view << " " << time_view << std::endl;

    total_tp += tp_view;
    total_fp += fp_view;
    total_fn += fn_view;

    if (param_.visualize_) {
      if (param_.visualize_errors_only_ && fp_view == 0)
        continue;

      std::string scene_name(anno_file);
      boost::replace_last(scene_name, ".anno", ".pcd");
      const bf::path scene_path = param_.test_dir / scene_name;
      pcl::PointCloud<PointT>::Ptr scene_cloud(new pcl::PointCloud<PointT>);
      pcl::io::loadPCDFile(scene_path.string(), *scene_cloud);
      // reset view point - otherwise this messes up PCL's visualization (this does not affect recognition results)
      scene_cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
      scene_cloud->sensor_origin_ = Eigen::Vector4f::Zero(4);
      vis_->addPointCloud(scene_cloud, "scene", vp1_);

      //            pcl::visualization::PointCloudColorHandlerCustom<PointT> gray (scene_cloud, 255, 255, 255);
      //            vis_->addPointCloud(scene_cloud, gray, "input_vp2", vp2_);
      //            vis_->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2,
      //            "input_vp2");
      //            vis_->addPointCloud(scene_cloud, gray, "input_vp3", vp3_);
      //            vis_->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2,
      //            "input_vp3");

      vis_->addText(scene_name, 10, 10, 15, vis_params_->text_color_(0), vis_params_->text_color_(1),
                    vis_params_->text_color_(2), "scene_text", vp1_);
      vis_->addText("ground-truth objects (occluded objects in blue, false ones in red, pose errors green)", 10, 10, 15,
                    vis_params_->text_color_(0), vis_params_->text_color_(1), vis_params_->text_color_(2), "gt_text",
                    vp2_);
      std::stringstream rec_text;
      rec_text << "recognized objects (tp: " << tp_view << ", fp: " << fp_view << ", fn: " << fn_view;
      if (tp_view) {
        rec_text << " trans_error: " << sum_translation_error_view / tp_view;
        rec_text << " rot_error: " << sum_rotational_error_view / tp_view;
      }
      rec_text << ")";
      vis_->addText(rec_text.str(), 10, 10, 15, vis_params_->text_color_(0), vis_params_->text_color_(1),
                    vis_params_->text_color_(2), "rec_text", vp3_);
      //            vis->resetCamera();
      vis_->spin();
    }
  }
  of.close();
}

void RecognitionEvaluator::loadModels() {
  std::vector<std::string> model_filenames = v4r::io::getFilesInDirectory(param_.models_dir, "3D_model.pcd", true);
  for (const std::string &model_fn : model_filenames) {
    pcl::PointCloud<ModelT>::Ptr model_cloud(new pcl::PointCloud<ModelT>);
    const bf::path model_full_path = param_.models_dir / model_fn;
    pcl::io::loadPCDFile(model_full_path.string(), *model_cloud);

    const bf::path model_path = model_fn;
    const std::string model_name = model_path.parent_path().string();
    Model m;
    m.cloud = model_cloud;
    m.is_rotational_symmetric_ = std::find(rotational_symmetric_objects_.begin(), rotational_symmetric_objects_.end(),
                                           model_name) != rotational_symmetric_objects_.end();
    m.is_rotation_invariant_ = std::find(rotational_invariant_objects_.begin(), rotational_invariant_objects_.end(),
                                         model_name) != rotational_invariant_objects_.end();
    pcl::compute3DCentroid(*m.cloud, m.centroid);

    // model identity is equal folder name -> remove \"/3D_model.pcd\" from filename
    models[model_name] = m;
  }
}

void RecognitionEvaluator::Parameter::init(boost::program_options::options_description &desc) {
  desc.add_options()("groundtruth_dir,g", po::value<bf::path>(&gt_dir)->required(),
                     "Root directory containing annotation files (i.e. 4x4 "
                     "ground-truth pose of each object with filename "
                     "viewId_ModelId_ModelInstanceCounter.txt");
  desc.add_options()("rec_results_dir,r", po::value<bf::path>(&or_dir)->required(),
                     "Root directory containing the recognition results (same format as annotation files).");
  desc.add_options()("out_dir,o", po::value<bf::path>(&out_dir)->default_value(out_dir)->required(),
                     "Output directory where recognition results will be stored");
  desc.add_options()("models_dir,m", po::value<bf::path>(&models_dir)->required(),
                     "Root directory containing the model files (i.e. filenames 3D_model.pcd).");
  desc.add_options()("camera_calibration_file",
                     po::value<bf::path>(&camera_calibration_file)->default_value(camera_calibration_file),
                     "RGB camera intrinsic calibration file in OpenCV format");
  desc.add_options()("img_out_dir", po::value<bf::path>(&img_out_dir)->default_value(img_out_dir),
                     "Path to where images should be saved (if save_images_to_disk is enabled)");
  desc.add_options()("trans_thresh",
                     po::value<float>(&translation_error_threshold_m_)->default_value(translation_error_threshold_m_),
                     "Maximal allowed translational error in meters");
  desc.add_options()("rot_thresh",
                     po::value<float>(&rotation_error_threshold_deg_)->default_value(rotation_error_threshold_deg_),
                     "Maximal allowed rotational error in degrees");
  desc.add_options()("occlusion_thresh", po::value<float>(&occlusion_threshold_)->default_value(occlusion_threshold_),
                     "Occlusion threshold. Object with higher occlusion will be ignored in the evaluation");
  desc.add_options()("visualize,v", po::bool_switch(&visualize_), "visualize recognition results");
  desc.add_options()("visualize_errors_only", po::bool_switch(&visualize_errors_only_),
                     "visualize only if there are errors (visualization must be on)");
  desc.add_options()("save_images_to_disk", po::bool_switch(&save_images_to_disk_),
                     "if true, saves images to disk (visualization must be on)");
  desc.add_options()("highlight_errors", po::bool_switch(&highlight_errors_),
                     "if true, highlights errors in the visualization");
  desc.add_options()("test_dir,t", po::value<bf::path>(&test_dir),
                     "Only for visualization. Root directory containing the scene files.");
  desc.add_options()("use_generated_hypotheses", po::bool_switch(&use_generated_hypotheses_),
                     "if true, computes recognition rate for all generated hypotheses instead of verified ones.");
}

float RecognitionEvaluator::compute_recognition_rate_over_occlusion() {
  loadModels();
  std::stringstream description;
  description << "Tool to compute object instance recognition rate." << std::endl
              << "==================================================" << std::endl
              << "This will generate a text file containing:" << std::endl
              << "Column 1: occlusion" << std::endl
              << "Column 2: is recognized" << std::endl
              << "==================================================" << std::endl
              << "** Allowed options";

  const bf::path out_path = param_.out_dir / "results_occlusion.txt";

  v4r::io::createDirForFileIfNotExist(out_path.string());
  std::ofstream f(out_path.string());
  std::cout << "Writing results to " << out_path.string() << "..." << std::endl;

  std::vector<std::string> annotation_files = v4r::io::getFilesInDirectory(param_.gt_dir, ".*.anno", true);

  size_t num_recognized = 0;
  size_t num_total = 0;
  for (const std::string &anno_file : annotation_files) {
    const bf::path gt_path = param_.gt_dir / anno_file;

    std::string rec_file = anno_file;
    if (param_.use_generated_hypotheses_)
      boost::replace_last(rec_file, ".anno", ".generated_hyps");

    const bf::path rec_path = param_.or_dir / rec_file;

    if (!v4r::io::existsFile(rec_path.string())) {
      LOG(INFO) << "File " << rec_path.string() << " not found.";
      continue;
    }

    std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> gt_hyps =
        readHypothesesFromFile(gt_path.string());
    std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> rec_hyps =
        readHypothesesFromFile(rec_path.string());

    for (auto const &gt_model_hyps : gt_hyps) {
      const std::string &model_name_gt = gt_model_hyps.first;
      const Model &m = models[model_name_gt];
      const std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>> &hyps = gt_model_hyps.second;

      for (const Hypothesis &h_gt : hyps) {
        bool is_recognized = false;

        const Eigen::Matrix4f &gt_pose = h_gt.pose;

        float occlusion = h_gt.occlusion;

        const auto it = rec_hyps.find(model_name_gt);
        if (it != rec_hyps.end()) {
          const std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>> &rec_model_hyps = it->second;
          for (const Hypothesis &h_rec : rec_model_hyps) {
            const Eigen::Matrix4f &rec_pose = h_rec.pose;
            float trans_error, rot_error;
            if (!computeError(gt_pose, rec_pose, m.centroid, trans_error, rot_error, m.is_rotation_invariant_,
                              m.is_rotational_symmetric_))
              is_recognized = true;
          }
        }
        num_total++;

        if (is_recognized)
          num_recognized++;

        f << occlusion << " " << is_recognized << std::endl;
      }
    }
  }
  f.close();
  std::cout << "Done!" << std::endl;

  return (float)num_recognized / num_total;
}

void RecognitionEvaluator::checkIndividualHypotheses() {
  loadModels();
  const std::vector<std::string> annotation_files = io::getFilesInDirectory(param_.gt_dir, ".*.anno", true);

  pcl::visualization::PCLVisualizer::Ptr vis;
  int vp1, vp2;
  if (param_.visualize_) {
    vis.reset(new pcl::visualization::PCLVisualizer("results"));
    vis->setBackgroundColor(vis_params_->bg_color_(0), vis_params_->bg_color_(1), vis_params_->bg_color_(2));
    vis->createViewPort(0, 0, 1, 0.5, vp1);
    vis->createViewPort(0, 0.5, 1, 1, vp2);
  }

  for (const std::string anno_file : annotation_files) {
    const bf::path gt_path = param_.gt_dir / anno_file;

    std::string rec_file = anno_file;
    if (param_.use_generated_hypotheses_)
      boost::replace_last(rec_file, ".anno", ".generated_hyps");

    const bf::path rec_path = param_.or_dir / rec_file;

    std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> gt_hyps =
        readHypothesesFromFile(gt_path.string());
    std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> rec_hyps =
        readHypothesesFromFile(rec_path.string());

    if (param_.visualize_) {
      vis->removeAllPointClouds();
      vis->removeAllShapes(vp1);
#if PCL_VERSION >= 100800
      vis->removeAllCoordinateSystems(vp1);
#endif
      std::string scene_name(anno_file);
      boost::replace_last(scene_name, ".anno", ".pcd");

      const bf::path scene_path = param_.test_dir / scene_name;
      pcl::PointCloud<PointT>::Ptr scene_cloud(new pcl::PointCloud<PointT>);
      pcl::io::loadPCDFile(scene_path.string(), *scene_cloud);

      // reset view point - otherwise this messes up PCL's visualization (this does not affect recognition results)
      scene_cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
      scene_cloud->sensor_origin_ = Eigen::Vector4f::Zero(4);

      vis->addPointCloud(scene_cloud, "scene", vp1);

      pcl::visualization::PointCloudColorHandlerCustom<PointT> gray(scene_cloud, 255, 255, 255);
      vis->addPointCloud(scene_cloud, gray, "input_vp2", vp2);
      vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "input_vp2");
      vis->addText(scene_name, 10, 10, 15, vis_params_->text_color_(0), vis_params_->text_color_(1),
                   vis_params_->text_color_(2), "scene_text", vp1);
    }

    for (const auto &m : models) {
      std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>> rec_hyps_tmp, gt_hyps_tmp;

      auto it = rec_hyps.find(m.first);
      if (it != rec_hyps.end())
        rec_hyps_tmp = it->second;

      it = gt_hyps.find(m.first);
      if (it != gt_hyps.end())
        gt_hyps_tmp = it->second;

      for (const Hypothesis &h : rec_hyps_tmp) {
        bool is_correct = false;
        float translation_error = std::numeric_limits<double>::max();
        float rotational_error = std::numeric_limits<double>::max();
        int best_matching_gt_id = -1;

        if (!gt_hyps_tmp.empty()) {
          best_matching_gt_id = -1;
          translation_error = std::numeric_limits<float>::max();
          rotational_error = std::numeric_limits<float>::max();

          for (size_t gt_id = 0; gt_id < gt_hyps_tmp.size(); gt_id++) {
            const Hypothesis &gt_hyp = gt_hyps_tmp[gt_id];
            float trans_error_tmp, rot_error_tmp;
            if (!computeError(h.pose, gt_hyp.pose, m.second.centroid, trans_error_tmp, rot_error_tmp,
                              m.second.is_rotation_invariant_, m.second.is_rotational_symmetric_))
              is_correct = true;

            if (trans_error_tmp < translation_error) {
              translation_error = trans_error_tmp;
              rotational_error = rot_error_tmp;
              best_matching_gt_id = gt_id;
            }
          }
        }

        if (param_.visualize_) {
          vis->removePointCloud("model_cloud", vp2);
          vis->removeAllShapes(vp2);
#if PCL_VERSION >= 100800
          vis->removeAllCoordinateSystems(vp2);
#endif
          pcl::PointCloud<ModelT>::Ptr model_cloud = m.second.cloud;
          pcl::PointCloud<ModelT>::Ptr model_aligned(new pcl::PointCloud<ModelT>());
          pcl::transformPointCloudWithNormals(*model_cloud, *model_aligned, h.pose);

          vis->addPointCloud<ModelT>(model_aligned, "model_cloud", vp2);

          if (is_correct)
            vis->setBackgroundColor(0, 255, 0, vp2);
          else
            vis->setBackgroundColor(255, 0, 0, vp2);

#if PCL_VERSION >= 100800
          Eigen::Matrix4f tf_tmp = h.pose;
          Eigen::Matrix3f rot_tmp = tf_tmp.block<3, 3>(0, 0);
          Eigen::Vector3f trans_tmp = tf_tmp.block<3, 1>(0, 3);
          Eigen::Affine3f affine_trans;
          affine_trans.fromPositionOrientationScale(trans_tmp, rot_tmp, Eigen::Vector3f::Ones());
          vis->addCoordinateSystem(0.1f, affine_trans, "model_co", vp2);
#endif
          if (best_matching_gt_id >= 0) {
            const Hypothesis &gt_hyp = gt_hyps_tmp[best_matching_gt_id];
            pcl::PointXYZ center_rec, center_gt;
            center_rec.getVector4fMap() = h.pose * m.second.centroid;
            vis->addSphere(center_rec, 0.01, 0, 0, 255, "center_rec", vp2);
            center_gt.getVector4fMap() = gt_hyp.pose * m.second.centroid;
            vis->addSphere(center_gt, 0.01, 125, 125, 255, "center_gt", vp2);
            vis->addLine(center_rec, center_gt, 0, 0, 255, "distance", vp2);
            std::stringstream model_txt;
            model_txt.precision(2);
            model_txt << "Transl. error: " << translation_error * 100.f << "cm; rotational error: " << rotational_error
                      << "deg; occlusion: " << gt_hyp.occlusion;
            vis->addText(model_txt.str(), 10, 10, 15, vis_params_->text_color_(0), vis_params_->text_color_(1),
                         vis_params_->text_color_(2), "model_text", vp2);
          }
          //                    vis->resetCamera();
          vis->spin();
        }
      }
    }
  }
}

Eigen::MatrixXi RecognitionEvaluator::compute_confusion_matrix() {
  //    std::stringstream description;
  //    description << "Tool to compute object instance recognition rate." << std::endl <<
  //                   "==================================================" << std::endl <<
  //                   "This will generate a text file containing:" << std::endl <<
  //                   "Column 1: occlusion" << std::endl <<
  //                   "Column 2: is recognized" << std::endl <<
  //                   "==================================================" << std::endl <<
  //                   "** Allowed options";

  const bf::path out_path = param_.out_dir / "confusion_matrix.txt";

  v4r::io::createDirForFileIfNotExist(out_path.string());
  std::ofstream f(out_path.string());
  std::cout << "Writing results to " << out_path.string() << "..." << std::endl;

  std::vector<std::string> annotation_files = v4r::io::getFilesInDirectory(param_.gt_dir, ".*.anno", true);

  Eigen::MatrixXi confusion_matrix = Eigen::MatrixXi::Zero(models.size(), models.size());

  std::map<std::string, int> modelname2modelid;
  size_t id = 0;
  for (const auto &m : models) {
    modelname2modelid[m.first] = id;
    //        f << id << ": " << m.first << std::endl;
    id++;
  }

  for (const std::string &anno_file : annotation_files) {
    const bf::path gt_path = param_.gt_dir / anno_file;

    std::string rec_file = anno_file;
    if (param_.use_generated_hypotheses_)
      boost::replace_last(rec_file, ".anno", ".generated_hyps");

    const bf::path rec_path = param_.or_dir / rec_file;

    if (!v4r::io::existsFile(rec_path.string())) {
      LOG(WARNING) << "Recognition path " << rec_path.string() << " does not exist!";
      continue;
    }

    Eigen::MatrixXi tmp_confusion_matrix = Eigen::MatrixXi::Zero(models.size(), models.size());

    std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> gt_hyps_all_models =
        readHypothesesFromFile(gt_path.string());
    std::map<std::string, std::vector<Hypothesis, Eigen::aligned_allocator<Hypothesis>>> rec_hyps =
        readHypothesesFromFile(rec_path.string());

    for (auto const &gt_model_hyps : gt_hyps_all_models) {
      const std::string &model_name_gt = gt_model_hyps.first;
      // const Model &m = models[model_name_gt];

      for (const Hypothesis &h_gt : gt_model_hyps.second) {
        //        float occlusion = h_gt.occlusion;
        float lowest_trans_error = std::numeric_limits<float>::max();
        std::string best_match = "";

        for (auto const &rec_model_hyps : rec_hyps) {
          const std::string &rec_model_name = rec_model_hyps.first;
          for (const Hypothesis &h_rec : rec_model_hyps.second) {
            // const Eigen::Vector4f centroid_a = h_gt.pose * m.centroid;
            // const Eigen::Vector4f centroid_b = h_rec.pose * m.centroid;

            // ignore z

            float trans_error = (h_gt.pose.block<2, 1>(0, 3) - h_rec.pose.block<2, 1>(0, 3)).norm();
            VLOG(1) << h_gt.pose;
            VLOG(1) << h_rec.pose;
            VLOG(1) << trans_error;

            if (trans_error < lowest_trans_error) {
              best_match = rec_model_name;
              lowest_trans_error = trans_error;
            }
          }
        }

        if (!best_match.empty() && lowest_trans_error < param_.translation_error_threshold_m_) {
          tmp_confusion_matrix(modelname2modelid[model_name_gt], modelname2modelid[best_match])++;
        }
      }
    }

    VLOG(1) << tmp_confusion_matrix << std::endl
            << std::endl
            << "view accuracy: " << tmp_confusion_matrix.trace() << " / " << tmp_confusion_matrix.sum() << " ("
            << (float)tmp_confusion_matrix.trace() / tmp_confusion_matrix.sum() << ")" << std::endl;

    if (param_.visualize_) {
      std::string scene_name(anno_file);
      boost::replace_last(scene_name, ".anno", ".pcd");

      const bf::path scene_path = param_.test_dir / scene_name;
      pcl::PointCloud<PointT>::Ptr scene_cloud(new pcl::PointCloud<PointT>);
      pcl::io::loadPCDFile(scene_path.string(), *scene_cloud);

      visualizeResults(scene_cloud, gt_path, rec_path);
    }

    confusion_matrix += tmp_confusion_matrix;
  }

  std::cout << confusion_matrix << std::endl
            << std::endl
            << "view accuracy: " << confusion_matrix.trace() << " / " << confusion_matrix.sum() << " ("
            << (float)confusion_matrix.trace() / confusion_matrix.sum() << ")" << std::endl;

  f << confusion_matrix;
  f.close();
  std::cout << "Done!" << std::endl;

  return confusion_matrix;
}
}  // namespace apps
}  // namespace v4r
