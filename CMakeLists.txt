cmake_minimum_required(VERSION 3.14)
project(nanoGrid VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wextra>
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wpedantic>
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# =======================
# Dependencies
# =======================

find_package(Eigen3 REQUIRED NO_MODULE)

# =======================
# Library
# =======================

add_library(${PROJECT_NAME}
  src/GridMap.cpp
  src/GridMapMath.cpp
  src/BufferRegion.cpp
  src/SubmapGeometry.cpp
  src/CubicInterpolation.cpp
  src/iterators/GridMapIterator.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

# Eigen plugins for NaN-aware reductions (sumOfFinites, meanOfFinites, etc.)
target_compile_definitions(${PROJECT_NAME} PUBLIC
  EIGEN_FUNCTORS_PLUGIN="nanogrid/eigen_plugins/FunctorsPlugin.hpp"
  EIGEN_DENSEBASE_PLUGIN="nanogrid/eigen_plugins/DenseBasePlugin.hpp"
)

# =======================
# Tests
# =======================

option(NANOGRID_BUILD_TESTS "Build nanoGrid tests" OFF)
if(NANOGRID_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# =======================
# Benchmarks
# =======================

option(NANOGRID_BUILD_BENCHMARKS "Build nanoGrid benchmarks" OFF)
if(NANOGRID_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

# =======================
# Install
# =======================

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
