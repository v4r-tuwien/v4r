#include <glog/logging.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <v4r/common/ZAdaptiveNormals.h>
#include <v4r/common/normal_estimator.h>

#include <v4r/common/normal_estimator_integral_image.h>
#include <v4r/common/normal_estimator_pcl.h>
#include <v4r/common/normal_estimator_z_adpative.h>
#include <pcl/impl/instantiate.hpp>

namespace v4r {

template <typename PointT>
typename NormalEstimator<PointT>::Ptr initNormalEstimator(NormalEstimatorType method,
                                                          std::vector<std::string> &params) {
  typename NormalEstimator<PointT>::Ptr ne;

  if (method == NormalEstimatorType::PCL_DEFAULT) {
    NormalEstimatorPCLParameter param;
    params = param.init(params);
    typename NormalEstimatorPCL<PointT>::Ptr ke(new NormalEstimatorPCL<PointT>(param));
    ne = std::dynamic_pointer_cast<NormalEstimator<PointT>>(ke);
  } else if (method == NormalEstimatorType::PCL_INTEGRAL_NORMAL) {
    NormalEstimatorIntegralImageParameter param;
    params = param.init(params);
    typename NormalEstimatorIntegralImage<PointT>::Ptr ne_tmp(new NormalEstimatorIntegralImage<PointT>(param));
    ne = std::dynamic_pointer_cast<NormalEstimator<PointT>>(ne_tmp);
  } else if (method == NormalEstimatorType::Z_ADAPTIVE) {
    ZAdaptiveNormalsParameter param;
    params = param.init(params);
    typename ZAdaptiveNormalsPCL<PointT>::Ptr ne_tmp(new ZAdaptiveNormalsPCL<PointT>(param));
    ne = std::dynamic_pointer_cast<NormalEstimator<PointT>>(ne_tmp);
  } else {
    LOG(ERROR) << "Normal estimation method " << method << " is not implemented! ";
  }

  return ne;
}

#define PCL_INSTANTIATE_initNormalEstimator(T)                                                      \
  template V4R_EXPORTS typename NormalEstimator<T>::Ptr initNormalEstimator<T>(NormalEstimatorType, \
                                                                               std::vector<std::string> &);
PCL_INSTANTIATE(initNormalEstimator, PCL_XYZ_POINT_TYPES)
}  // namespace v4r
