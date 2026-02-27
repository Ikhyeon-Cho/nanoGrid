// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025 Ikhyeon Cho. All rights reserved.

#pragma once

#include <cstdint>
#include <functional>
#include <unordered_map>

#include "nanogrid/TypeDefs.hpp"

namespace nanogrid {

/// Hash functor for using nanogrid::Index as unordered_map keys.
struct IndexHash {
  std::size_t operator()(const Index& idx) const {
    const auto key =
        (static_cast<uint64_t>(static_cast<uint32_t>(idx(0))) << 32) |
        static_cast<uint64_t>(static_cast<uint32_t>(idx(1)));
    return std::hash<uint64_t>()(key);
  }
};

/// Equality functor for nanogrid::Index.
struct IndexEqual {
  bool operator()(const Index& a, const Index& b) const {
    return a(0) == b(0) && a(1) == b(1);
  }
};

/// Convenience alias for a cell-indexed unordered map.
template <typename T>
using CellMap = std::unordered_map<Index, T, IndexHash, IndexEqual>;

}  // namespace nanogrid
