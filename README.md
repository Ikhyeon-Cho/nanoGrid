<h1>
  <img src="./assets/nanoGrid_logo.svg" align="left" height="46px" alt="nanoGrid logo"/>
  <span>nanoGrid</span>
</h1>

[![C++17](https://img.shields.io/badge/C++17-00599C?logo=cplusplus&logoColor=white)](https://github.com/Ikhyeon-Cho/nanoGrid) [![License](https://img.shields.io/badge/license-BSD--3--Clause-%2328A745)](https://github.com/Ikhyeon-Cho/nanoGrid/blob/main/LICENSE)

A standalone C++17 extraction of [grid_map_core](https://github.com/ANYbotics/grid_map) (ETH/ANYbotics), with modern API additions and faster iteration. For the full API documentation, see the [original repository](https://github.com/ANYbotics/grid_map).

---

## What's Different from grid_map

- ***Standalone*** — Pure CMake. No ROS, no catkin, no ament.
- ***Modern C++17 API*** — Cleaner alternatives to the classic out-parameter pattern ([see below](#modern-c17-api)).
- ***Minimal*** — Just the core data structure. No ROS msgs, no RViz plugins, no OpenCV/PCL bridges.
- ***Extra utilities*** — Additional helpers for grid iteration, cell indexing, and more.
- ***Namespace*** — `nanogrid::` instead of `grid_map::`. Coexists with ROS grid_map, and easy to migrate.

---

## Dependencies

- **Eigen3** — Linear algebra

---

## Build

```bash
git clone https://github.com/Ikhyeon-Cho/nanoGrid.git
cd nanoGrid
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install  # optional
```

---

## Usage

### Basic (grid_map compatible)

nanoGrid is fully API-compatible with grid_map — just use `nanogrid::` instead of `grid_map::`.

```cpp
#include <nanogrid/nanogrid.hpp>

// Create a 10x10m grid at 0.1m resolution
nanogrid::GridMap map;
map.setGeometry(nanogrid::Length(10.0, 10.0), 0.1);
map.add("elevation", 0.0f);

// Access layer data
nanogrid::Matrix& layer = map.get("elevation");

// Position / index conversion
nanogrid::Index index;
map.getIndex(nanogrid::Position(1.0, 2.0), index);
map.at("elevation", index) = 1.5f;
```

### Modern C++17 API

nanoGrid also provides `std::optional`-based methods that replace the classic out-parameter pattern.

```cpp
// Before: out-parameter + bool check
Index idx;
if (map.getIndex(pos, idx)) {
    if (map.isValid(idx, "elevation")) {
        float val = map.at("elevation", idx);
        // use val
    }
}
// After: single call, one line
if (auto val = map.value("elevation", pos)) {
    // use *val
}


// Before: verbose iterator
for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const size_t i = it.getLinearIndex();
    data(i) = ...;
}
// After: range-based for (Eigen-equivalent performance)
for (auto i : map.cells()) {
    data(i) = ...;
}
```

| Classic (grid_map compatible) | Modern (C++17) |
|-------------------------------|----------------|
| `bool getIndex(pos, idx)` | `std::optional<Index> index(pos)` |
| `bool getPosition(idx, pos)` | `std::optional<Position> position(idx)` |
| `bool getPosition3(layer, idx, pos3)` | `std::optional<Position3> position3(layer, idx)` |
| `bool getVector(prefix, idx, vec)` | `std::optional<Vector3> vector3(prefix, idx)` |
| `getSubmap(pos, len, success)` | `std::optional<GridMap> submap(pos, len)` |
| `isInside()` + `getIndex()` + `isValid()` + `at()` | `std::optional<float> value(layer, pos)` |
| `for (GridMapIterator it(map); !it.isPastEnd(); ++it)` | `for (auto i : map.cells())` |

### CMake Integration

```cmake
find_package(nanoGrid REQUIRED)
target_link_libraries(your_target PUBLIC nanoGrid::nanoGrid)
```

Or via FetchContent:

```cmake
include(FetchContent)
FetchContent_Declare(nanoGrid
  GIT_REPOSITORY https://github.com/Ikhyeon-Cho/nanoGrid.git
  GIT_TAG main
)
FetchContent_MakeAvailable(nanoGrid)
target_link_libraries(your_target PUBLIC nanoGrid::nanoGrid)
```

---

## Performance

Iteration benchmark on a 5000×5000 grid (25M cells, Release build):

| Method | Time | vs Fastest |
|--------|------|-----------|
| Eigen `cwiseMax()` | 8.1 ms | **1.0x** |
| `cells()` range-based for | 8.1 ms | **1.0x** |
| Raw linear loop | 8.3 ms | 1.0x |
| `GridMapIterator` + `getLinearIndex()` | 69 ms | 8.6x |
| `GridMapIterator` + `operator*()` | 181 ms | 22.5x |
| `GridMapIterator` + `map.at()` | 572 ms | 70.9x |

```cpp
// Recommended: use cells() or Eigen operations
auto& src = map["elevation"];
auto& dst = map["cost"];
for (auto i : map.cells()) {
    dst(i) = src(i) / 45.0f;
}
```

---

## Acknowledgments

Based on [grid_map](https://github.com/ANYbotics/grid_map) by Péter Fankhauser, ETH Zurich / ANYbotics AG.

---

<div align="center">

BSD-3-Clause License © [Ikhyeon Cho](mailto:ikhyeon.c@gmail.com), [ANYbotics AG](https://www.anybotics.com/)

</div>
