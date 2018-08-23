#include <glog/logging.h>
#include <v4r/common/binary_algorithms.h>
#include <v4r/common/miscellaneous.h>

namespace v4r {
boost::dynamic_bitset<> binary_operation(const boost::dynamic_bitset<> &mask1, const boost::dynamic_bitset<> &mask2,
                                         BINARY_OPERATOR operation) {
  CHECK(mask1.size() == mask2.size());

  boost::dynamic_bitset<> output_mask(mask1.size());

  for (size_t i = 0; i < mask1.size(); i++) {
    if (operation == BINARY_OPERATOR::AND) {
      output_mask[i] = mask1[i] && mask2[i];
    } else if (operation == BINARY_OPERATOR::AND_N) {
      output_mask[i] = mask1[i] && !mask2[i];
    } else if (operation == BINARY_OPERATOR::OR) {
      output_mask[i] = mask1[i] || mask2[i];
    } else if (operation == BINARY_OPERATOR::OR_N) {
      output_mask[i] = mask1[i] || !mask2[i];
    } else if (operation == BINARY_OPERATOR::XOR) {
      output_mask[i] = (mask1[i] && !mask2[i]) || (!mask1[i] && mask2[i]);
    }
  }
  return output_mask;
}

template <typename IdxT>
boost::dynamic_bitset<> createMaskFromVecIndices(const typename std::vector<std::vector<IdxT>> &v_indices,
                                                 size_t image_size) {
  boost::dynamic_bitset<> mask;

  if (mask.size() != image_size)
    mask.resize(image_size, 0);

  for (size_t i = 0; i < v_indices.size(); i++) {
    boost::dynamic_bitset<> mask_tmp = v4r::createMaskFromIndices(v_indices[i], image_size);

    if (!mask.empty())
      mask = binary_operation(mask, mask_tmp, BINARY_OPERATOR::OR);
    else
      mask = mask_tmp;
  }

  return mask;
}

}  // namespace v4r