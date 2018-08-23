/*
 * Author: Thomas Faeulhammer
 * Date: August 2015
 *
 */

#pragma once

#include <v4r/core/macros.h>
#include <boost/dynamic_bitset.hpp>
#include <vector>

namespace v4r {

enum class BINARY_OPERATOR {
  AND,
  AND_N,  // this will negate the second argument
  OR,
  OR_N,  // this will negate the second argument
  XOR
};

/**
 * @brief performs bit wise logical operations
 * @param[in] bit mask1
 * @param[in] bit mask2
 * @param[in] operation (AND, AND_N, OR, XOR)
 * @return output bit mask
 */
V4R_EXPORTS boost::dynamic_bitset<> binary_operation(const boost::dynamic_bitset<> &mask1,
                                                     const boost::dynamic_bitset<> &mask2, BINARY_OPERATOR operation);

/**
 * @brief given vector of indices of an image or pointcloud,
 * this function creates a boolean mask of the concatenated indices
 * @param vector of objct objectIndices
 * @param image_size
 * @return object bit mask
 */
template <typename IdxT>
V4R_EXPORTS boost::dynamic_bitset<> createMaskFromVecIndices(const typename std::vector<std::vector<IdxT>> &v_indices,
                                                             size_t image_size);

}  // namespace v4r
