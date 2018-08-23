#include <glog/logging.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/zbuffering.h>
#include <pcl/impl/instantiate.hpp>

namespace v4r {

// NOTE: The following template specialization will cause a compiler warning during instantiation
// template <>
// void ZBuffering<pcl::PointXYZRGBNormal>::depthBuffering(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud,
//                                                        int subsample) {
//  float cx = cam_->getCx();
//  float cy = cam_->getCy();
//  float fx = cam_->getFocalLengthX();
//  float fy = cam_->getFocalLengthY();
//  int width = static_cast<int>(cam_->getWidth());
//  int height = static_cast<int>(cam_->getHeight());
//
//  for (size_t i = 0; i < cloud.points.size(); i += subsample) {
//    const pcl::PointXYZRGBNormal &pt = cloud.points[i];
//    int u = fx * pt.x / pt.z + cx;
//    int v = fy * pt.y / pt.z + cy;
//
//    if (u > width || v > height || u < 0 || v < 0)
//      continue;
//
//    if (param_.use_normals_ &&
//        pt.getNormalVector4fMap().dot(pt.getVector4fMap()) > 0.f)  /// NOTE: We do not need to normalize here
//      continue;
//
//    pcl::PointXYZRGBNormal &r_pt = rendered_view_->at(u, v);
//    if (!pcl_isfinite(r_pt.z) || (pt.z < r_pt.z)) {
//      r_pt = pt;
//      index_map_(v, u) = i;
//    }
//  }
//}

template <typename PointT>
void ZBuffering<PointT>::depthBuffering(const pcl::PointCloud<PointT> &cloud, size_t subsample) {
  if (param_.use_normals_ && (!cloud_normals_ || cloud_normals_->points.size() != cloud.points.size())) {
    if (pcl::traits::has_normal<PointT>::value) {
      LOG(ERROR) << "Surface normals must be set separately. Surface normals stored in point clouds with intrinsic "
                    "surface normals such as pcl::PointXYZNormal or pcl::PointXYZRGBNormal are not supported "
                    "currently! Will ignore normals!!";
    } else {
      LOG(ERROR) << "Parameters set to use normals but normals are not set or do "
                    "not correspond with "
                    "input cloud! Will ignore normals!!";
    }
    param_.use_normals_ = false;
  }

  CHECK(subsample > 0) << "subsampling value must be greater 0!";

  int width = static_cast<int>(cam_.w);
  int height = static_cast<int>(cam_.h);

  for (size_t i = 0; i < cloud.points.size(); i += subsample) {
    const PointT &p = cloud.points[i];
    int u = cam_.fx * p.x / p.z + cam_.cx;
    int v = cam_.fy * p.y / p.z + cam_.cy;

    if (u >= width || v >= height || u < 0 || v < 0)
      continue;

    if (param_.use_normals_) {
      const pcl::Normal &n = cloud_normals_->points[i];
      if (n.getNormalVector4fMap().dot(p.getVector4fMap()) > 0.f)  /// NOTE: We do not need to normalize here
        continue;
    }

    PointT &r_pt = rendered_view_->at(u, v);
    if (!pcl_isfinite(r_pt.z) || (p.z < r_pt.z)) {
      r_pt = p;
      index_map_(v, u) = i;
    }
  }
}

template <typename PointT>
void ZBuffering<PointT>::doSmoothing() {
  pcl::PointCloud<PointT> rendered_view_unsmooth = *rendered_view_;

  for (size_t u = param_.smoothing_radius_; u < (rendered_view_->width - param_.smoothing_radius_); u++) {
    for (size_t v = param_.smoothing_radius_; v < (rendered_view_->height - param_.smoothing_radius_); v++) {
      float min = std::numeric_limits<float>::max();
      int min_uu = u, min_vv = v;
      for (size_t uu = (u - param_.smoothing_radius_); uu <= (u + param_.smoothing_radius_); uu++) {
        for (size_t vv = (v - param_.smoothing_radius_); vv <= (v + param_.smoothing_radius_); vv++) {
          if (uu > rendered_view_->width || vv > rendered_view_->height)  // this shouldn't happen anyway
            continue;

          const PointT &p = rendered_view_unsmooth.at(uu, vv);
          if (!pcl_isfinite(min) || (pcl::isFinite(p) && (p.z < min))) {
            min = p.z;
            min_uu = uu;
            min_vv = vv;
          }
        }
      }

      rendered_view_->at(u, v) = rendered_view_unsmooth.at(min_uu, min_vv);
      index_map_(v, u) = index_map_(min_vv,
                                    min_uu);  /// NOTE: Be careful, this is maybe not what you want to get!
    }
  }
}

template <typename PointT>
void ZBuffering<PointT>::doNoiseFiltering() {
  pcl::PointCloud<PointT> rendered_view_filtered = *rendered_view_;

  for (size_t u = param_.smoothing_radius_; u < ((int)rendered_view_->width - param_.smoothing_radius_); u++) {
    for (size_t v = param_.smoothing_radius_; v < ((int)rendered_view_->height - param_.smoothing_radius_); v++) {
      PointT &p = rendered_view_filtered.at(u, v);
      bool is_noise = true;
      for (size_t uu = (u - param_.smoothing_radius_); uu <= (u + param_.smoothing_radius_) && is_noise; uu++) {
        for (size_t vv = (v - param_.smoothing_radius_); vv <= (v + param_.smoothing_radius_); vv++) {
          if (uu > rendered_view_->width || vv > rendered_view_->height)  // this shouldn't happen anyway
            continue;

          if (uu == u && vv == v)
            continue;

          PointT &p_tmp = rendered_view_->at(uu, vv);
          if (std::abs(p.z - p_tmp.z) < param_.inlier_threshold_) {
            is_noise = false;
            break;
          }
        }
      }
      if (is_noise) {
        p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
        index_map_(v, u) = -1;
      }
    }
  }
  *rendered_view_ = rendered_view_filtered;
}

template <typename PointT>
void ZBuffering<PointT>::renderPointCloud(const pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &rendered_view,
                                          size_t subsample) {
  if (!rendered_view_) {  // keep allocated memory over multiple calls
    rendered_view_.reset(new pcl::PointCloud<PointT>);
    rendered_view_->points.resize(cam_.w * cam_.h);
    rendered_view_->width = cam_.w;
    rendered_view_->height = cam_.h;
    rendered_view_->is_dense = false;
  }

  index_map_ = Eigen::MatrixXi::Constant(cam_.h, cam_.w, -1);

  for (PointT &p : rendered_view_->points)  // initialize points to infinity
    p.z = std::numeric_limits<float>::quiet_NaN();

  depthBuffering(cloud, subsample);

  if (param_.do_smoothing_) {
    doSmoothing();
  }

  if (param_.do_noise_filtering_) {
    doNoiseFiltering();
  }

  if (param_.extract_indices_) {
    boost::dynamic_bitset<> pt_is_kept(cloud.points.size(), 0);

    for (int v = 0; v < index_map_.rows(); v++) {
      for (int u = 0; u < index_map_.cols(); u++) {
        if (index_map_(v, u) >= 0) {
          pt_is_kept.set(index_map_(v, u));
        }
      }
    }

    kept_indices_ = createIndicesFromMask<int>(pt_is_kept);
  } else {
    kept_indices_.clear();
  }

  rendered_view = *rendered_view_;
}

#define PCL_INSTANTIATE_ZBuffering(T) template class V4R_EXPORTS ZBuffering<T>;
PCL_INSTANTIATE(ZBuffering, PCL_XYZ_POINT_TYPES)

}  // namespace v4r