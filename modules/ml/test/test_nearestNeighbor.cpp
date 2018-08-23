#include "test.h"

#include <v4r/ml/nearestNeighbor.h>
#include <numeric>

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) {
  // initialize original index locations
  std::vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

  return idx;
}

/// randomly sample points in high-dimensional space and give them a random label. These labelled data points serve as
/// training data. The test checks if nearest neighbors returned to a random query vector with respect to a given
/// distance metric are correct.
TEST(NearestNeighbor, knn) {
  size_t num_dim = 300;
  size_t num_training_samples = 10000;
  size_t num_query_data = 10;
  float eps = 0.01f;  ///< allowed error bound for distances returned by nearest neighbor search

  const std::vector<v4r::DistanceMetric> dist_metric = {v4r::DistanceMetric::L1, v4r::DistanceMetric::L2};

  v4r::NearestNeighborClassifierParameter param;
  param.knn_ = 3;
  param.kdtree_num_trees_ = 1;
  param.checks_ = -1;  // to get reliable results, this number needs to be high for high-dimensional data.

  for (const auto &dm : dist_metric) {
    param.distance_metric_ = dm;
    v4r::NearestNeighborClassifier nn(param);
    ASSERT_EQ(nn.getType(), v4r::ClassifierType::KNN);

    Eigen::MatrixXf training_data = Eigen::MatrixXf::Random(num_training_samples, num_dim);
    Eigen::VectorXi training_label = Eigen::VectorXi::Random(num_training_samples);
    Eigen::MatrixXf query_data = Eigen::MatrixXf::Random(num_query_data, num_dim);

    nn.train(training_data, training_label);
    Eigen::MatrixXi prediction;
    nn.predict(query_data, prediction);

    Eigen::MatrixXi predicted_training_sample_indices;
    Eigen::MatrixXf dist;
    nn.getTrainingSampleIDSforPredictions(predicted_training_sample_indices, dist);

    for (size_t query_id = 0; query_id < num_query_data; query_id++) {
      std::vector<float> distances(num_training_samples);
      for (size_t i = 0; i < num_training_samples; i++) {
        switch (dm) {
          case v4r::DistanceMetric::L1:
            distances[i] = (training_data.row(i) - query_data.row(query_id)).lpNorm<1>();
            break;
          case v4r::DistanceMetric::L2:
            distances[i] = (training_data.row(i) - query_data.row(query_id)).lpNorm<2>();
            break;
          default:
            std::cout << "Nearest neighbor test for distance metric " << dm << " not implemented!"
                      << std::endl;  /// TODO: Fix this
        }
      }

      std::vector<size_t> sorted_indices = sort_indexes(distances);
      for (size_t k = 0; k < param.knn_; k++) {
        EXPECT_EQ(prediction(query_id, k), training_label(sorted_indices[k]));
        EXPECT_EQ(predicted_training_sample_indices(query_id, k), sorted_indices[k]);
        switch (dm) {
          case v4r::DistanceMetric::L1:
            EXPECT_NEAR(dist(query_id, k), distances[sorted_indices[k]], eps);
            break;
          case v4r::DistanceMetric::L2:
            EXPECT_NEAR(sqrt(dist(query_id, k)), distances[sorted_indices[k]], eps);
            break;
          default:
            std::cout << "Nearest neighbor test for distance metric " << dm << " not implemented!"
                      << std::endl;  /// TODO: Fix this
        }
      }
    }
  }
}
