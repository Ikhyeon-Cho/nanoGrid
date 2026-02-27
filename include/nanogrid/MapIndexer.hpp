// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025 Ikhyeon Cho. All rights reserved.

#pragma once

#include "nanogrid/GridMap.hpp"

#include <cmath>
#include <utility>
#include <vector>

namespace nanogrid {

/// @brief Lightweight helper for iterating over GridMap cells.
///
/// Handles circular buffer index mapping internally so callers can iterate
/// with plain (row, col) loops and access Eigen matrices without knowing
/// about the underlying buffer layout.
struct MapIndexer {
  const int rows, cols;
  const float resolution;

  explicit MapIndexer(const GridMap& map)
      : rows(map.getSize()(0)),
        cols(map.getSize()(1)),
        resolution(static_cast<float>(map.getResolution())),
        sr_(map.getStartIndex()(0)),
        sc_(map.getStartIndex()(1)) {}

  /// Grid (row, col) -> matrix position for Eigen access.
  std::pair<int, int> operator()(int row, int col) const {
    int r = row + sr_;
    if (r >= rows) r -= rows;
    int c = col + sc_;
    if (c >= cols) c -= cols;
    return {r, c};
  }

  /// Check if (row, col) is within [0, rows) x [0, cols).
  bool contains(int row, int col) const {
    return row >= 0 && row < rows && col >= 0 && col < cols;
  }

  /// Relative cell offset within a circular radius.
  struct Neighbor {
    int dr, dc;
    float dist_sq;  ///< Squared distance in meters.
  };

  /// Precompute neighbor offsets within a circular radius.
  /// @note Includes the center cell (dr=0, dc=0, dist_sq=0).
  std::vector<Neighbor> circleNeighbors(float radius) const {
    std::vector<Neighbor> offsets;
    const int r_cells = static_cast<int>(std::ceil(radius / resolution));
    const float radius_sq = radius * radius;
    for (int dr = -r_cells; dr <= r_cells; ++dr) {
      for (int dc = -r_cells; dc <= r_cells; ++dc) {
        const float dsq =
            resolution * resolution * static_cast<float>(dr * dr + dc * dc);
        if (dsq <= radius_sq) offsets.push_back({dr, dc, dsq});
      }
    }
    return offsets;
  }

 private:
  int sr_, sc_;
};

/// Convenience free function to create a MapIndexer.
inline MapIndexer makeIndexer(const GridMap& map) { return MapIndexer(map); }

}  // namespace nanogrid
