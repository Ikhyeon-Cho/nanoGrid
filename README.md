<h1>
  <img src="./assets/nanoGrid_logo.svg" align="left" height="46px" alt="nanoGrid logo"/>
  <span>nanoGrid</span>
</h1>

[![C++17](https://img.shields.io/badge/C++17-00599C?logo=cplusplus&logoColor=white)](https://github.com/Ikhyeon-Cho/nanoGrid) [![License](https://img.shields.io/badge/license-BSD--3--Clause-%2328A745)](https://github.com/Ikhyeon-Cho/nanoGrid/blob/main/LICENSE)

> Multi-layer grid maps for any C++ project. Eigen-only. Plain CMake.

nanoGrid is a C++17 library for 2.5D multi-layer grid maps. Each layer — elevation, surface normals, traversability, or any per-cell data — is stored as a named Eigen float matrix on a fixed-resolution grid. It adds a modernized API and faster iteration on top of the original.

This library is based on [grid_map](https://github.com/ANYbotics/grid_map) by Péter Fankhauser (ETH Zurich / ANYbotics). Following the original license terms, nanoGrid is distributed under the BSD-3-Clause license. It uses the `nanogrid::` namespace and coexists with ROS `grid_map` side by side.

→ [Quick Start](#quick-start)

---

## Why nanoGrid?

nanoGrid takes `grid_map`'s core and makes it standalone, modernized, and faster:

- **Standalone** — Pure CMake. Add it with `FetchContent` in 3 lines.
- **Modern API** — Idiomatic C++17 value-returning patterns.
- **Faster map iteration** — Range-based for loop at raw Eigen speed.

  <details>
  <summary>Benchmark: 5000×5000 grid (25M cells, Release build)</summary>

  | Method | Time | vs Baseline |
  |--------|------|-------------|
  | **Eigen** bulk operation (SIMD-optimized) | 8.2 ms | **1.0x** |
  | **nanoGrid `map.cells()`** | **8.3 ms** | **1.0x** |
  | **nanoGrid `map.cells()`** + grid coordinates | **22.7 ms** | **2.8x** |
  | **grid_map** `GridMapIterator` + `getLinearIndex()` | 83.7 ms | 10.3x |
  | **grid_map** `GridMapIterator` + `operator*()` | 192 ms | 23.5x |
  | **grid_map** `GridMapIterator` + `map.at()` | 614 ms | 75.2x |

  </details>

---

## Quick Start

### Build

```bash
git clone https://github.com/Ikhyeon-Cho/nanoGrid.git
cd nanoGrid
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install  # optional
```

<details>
<summary>To use nanoGrid in your project</summary>
<br>

Link against `nanoGrid::nanoGrid`:

```cmake
find_package(nanoGrid REQUIRED)
target_link_libraries(your_target PUBLIC nanoGrid::nanoGrid)
```

Or fetch it directly:

```cmake
include(FetchContent)
FetchContent_Declare(nanoGrid
  GIT_REPOSITORY https://github.com/Ikhyeon-Cho/nanoGrid.git
  GIT_TAG main
)
FetchContent_MakeAvailable(nanoGrid)
target_link_libraries(your_target PUBLIC nanoGrid::nanoGrid)
```

</details>

### Minimal Example

Create a 20×20 m grid at 0.1 m resolution, add an elevation layer, write cells, and read back by world position:

```cpp
#include <nanogrid/nanogrid.hpp>
using namespace nanogrid;

GridMap map;
map.setGeometry(Length(20.0, 20.0), 0.1);  // 20×20m, 0.1m resolution
map.add("elevation");

// Write — direct Eigen matrix access
auto& elevation = map["elevation"];
for (auto cell : map.cells()) {
    elevation(cell.index) = 42.0f;
}

// Read — query by world position
if (auto val = map.get("elevation", Position(1.0, 2.0))) {
    // *val is 42.0f
}
```

---

## API Compatibility

The full `grid_map` API is retained — just swap `grid_map::` to `nanogrid::`. On top of that, nanoGrid adds modern C++17 alternatives. See [`GridMap.hpp`](include/nanogrid/GridMap.hpp) for the full API.

**Reading a cell by position:**

```cpp
// Classic: out-parameter + bool check
Index idx;
if (map.getIndex(pos, idx)) {
    if (map.isValid(idx, "elevation")) {
        float val = map.at("elevation", idx);
    }
}

// Modern: single call
if (auto val = map.get("elevation", pos)) {
    // use *val
}
```

**Iterating over all cells:**

```cpp
// Classic: verbose iterator
for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const size_t i = it.getLinearIndex();
    elevation(i) = ...;
}

// Modern: range-based for with spatial info
for (auto cell : map.cells()) {
    elevation(cell.index) = 0.0f;
    if (cell.row == 0 || cell.col == 0) {
        elevation(cell.index) = -1.0f;  // mark borders
    }
}
```

---

## Acknowledgments

nanoGrid would not exist without the excellent work on [grid_map](https://github.com/ANYbotics/grid_map) by Péter Fankhauser and the teams at ETH Zurich and ANYbotics AG. Their design of the multi-layer grid map data structure has been foundational for robotics research and deployment worldwide.

---

<div align="center">

BSD-3-Clause License © [Ikhyeon Cho](mailto:ikhyeon.c@gmail.com), [ANYbotics AG](https://www.anybotics.com/)

</div>
