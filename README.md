# nanoGrid

[![C++17](https://img.shields.io/badge/C++17-00599C?logo=cplusplus&logoColor=white)](https://github.com/Ikhyeon-Cho/nanoGrid) [![License](https://img.shields.io/badge/license-BSD--3--Clause-%2328A745)](https://github.com/Ikhyeon-Cho/nanoGrid/blob/main/LICENSE)

> Lightweight 2.5D grid map library for robotics — no ROS required.

A standalone C++17 extraction of [grid_map_core](https://github.com/ANYbotics/grid_map) (ETH/ANYbotics), with the same API and added utilities. For the full API documentation, see the [original repository](https://github.com/ANYbotics/grid_map).

---

## What's Different from grid_map

- ***Standalone*** — Pure CMake. No ROS, no catkin, no ament.
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

nanoGrid is fully API-compatible with grid_map_core — just use `nanogrid::` instead of `grid_map::`.

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

## Acknowledgments

Based on [grid_map](https://github.com/ANYbotics/grid_map) by Péter Fankhauser, ETH Zurich / ANYbotics AG.

---

<div align="center">

BSD-3-Clause License © [Ikhyeon Cho](mailto:ikhyeon.c@gmail.com), [ANYbotics AG](https://www.anybotics.com/)

</div>
