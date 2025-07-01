# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Cloudini is a high-performance pointcloud compression library with bindings for ROS, PCL, and WebAssembly. It implements a two-stage compression approach: custom encoding for pointcloud fields followed by general-purpose compression (LZ4/ZSTD).

## Architecture

The project consists of three main components:

- **cloudini_lib/**: Core compression library (C++20) with field encoders/decoders
- **cloudini_ros/**: ROS integration with point_cloud_transport plugins and conversion utilities
- **cloudini_web/**: WebAssembly interface for browser-based compression

## Build Commands

### Core Library (Standalone)
```bash
cmake -B build -S cloudini_lib -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

### Debug Build with Sanitizers
```bash
cmake -B build_debug -S cloudini_lib -DCMAKE_BUILD_TYPE=Debug
cmake --build build_debug --parallel
```

### ROS Build
```bash
colcon build --packages-select cloudini_lib cloudini_ros
```

### WebAssembly Build
```bash
# Requires Emscripten
cmake -B build_wasm -S cloudini_lib -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake
cmake --build build_wasm
```

### Web Interface
```bash
cd cloudini_web
npm install
npm run dev  # Development server
npm run build  # Production build
```

## Testing

```bash
# Core library tests
cd build && ctest

# ROS integration tests (requires ROS environment)
colcon test --packages-select cloudini_lib cloudini_ros

# Run specific benchmark
./build/benchmarks/pcd_benchmark

# ROS bag benchmark (requires test data)
./build_jazzy/test/rosbag_benchmark path/to/rosbag.mcap
```

## Key Dependencies

- **Required**: CMake 3.16+, C++20 compiler
- **Auto-downloaded via CPM**: LZ4, ZSTD, cxxopts, benchmark, googletest
- **Optional**: PCL (for pointcloud utilities), ROS 2 (for integration)
- **Web**: Node.js 18+, Vite

## Development Notes

- Uses CPM for dependency management - set `CPM_SOURCE_CACHE` environment variable to avoid re-downloading
- Supports SSE4.1 optimizations on x86_64
- Address sanitizer enabled automatically in Debug builds on GCC/Clang
- Custom intrinsics for SIMD operations in `include/cloudini_lib/intrinsics.hpp`
- Field encoders support both lossy (quantized floats) and lossless compression modes

## Tools

- **cloudini_rosbag_converter**: Convert MCAP rosbags between compressed/uncompressed pointclouds
- **mcap_cutter**: Extract portions of MCAP files
- **pcd_benchmark**: Benchmark compression on PCD files
- **run_encoder.sh**: Batch processing script for test data
