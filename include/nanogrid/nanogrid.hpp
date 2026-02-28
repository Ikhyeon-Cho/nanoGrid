// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025 Ikhyeon Cho. All rights reserved.
//
// nanoGrid — Lightweight 2D grid map library for robotics.
// Extracted and modernized from ETH/ANYbotics grid_map_core.

#pragma once

#include "nanogrid/TypeDefs.hpp"
#include "nanogrid/GridMap.hpp"
#include "nanogrid/SubmapGeometry.hpp"
#include "nanogrid/GridMapMath.hpp"
#include "nanogrid/BufferRegion.hpp"
#include "nanogrid/iterators/GridMapIterator.hpp"
#include "nanogrid/eigen_plugins/Functors.hpp"

// nanoGrid additions
#include "nanogrid/IndexHash.hpp"

namespace nanogrid {

/// Returns a mask where finite cells are 1.0 and NaN cells are 0.0.
inline Eigen::MatrixXf isFinite(const GridMap& map, const std::string& layer) {
  const auto& data = map.get(layer).array();
  return (1.0f - (data != data).cast<float>()).matrix();
}

}  // namespace nanogrid
