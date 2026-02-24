# Voliro Custom Point Cloud Format Support

This document describes the changes made to the upstream [Cloudini Foxglove extension](https://github.com/facontidavide/cloudini) to support the Voliro custom point cloud format (`perception_types_ipc.proto.CloudiniCompressedPointCloud`).

## Overview

The upstream extension converts ROS2 `point_cloud_interfaces/msg/CompressedPointCloud2` messages to `sensor_msgs/msg/PointCloud2`. The Voliro fork replaces this with a converter from the Voliro-specific Protobuf schema `perception_types_ipc.proto.CloudiniCompressedPointCloud` to `foxglove.PointCloud`.

### Key Differences from Upstream

| Aspect | Upstream | Voliro Custom |
|---|---|---|
| Input schema | `point_cloud_interfaces/msg/CompressedPointCloud2` | `perception_types_ipc.proto.CloudiniCompressedPointCloud` |
| Output schema | `sensor_msgs/msg/PointCloud2` | `foxglove.PointCloud` |
| Timestamp | ROS `Header.stamp` (`sec`/`nsec`) | `VolirotTimestamp` (`timestamp_ns`) |
| Pose / TF | From ROS TF tree | Inline `VolirotTransform` (translation + quaternion) |
| Point fields | From message fields (width, height, point_step, etc.) | Extracted from Cloudini compressed header via WASM YAML parser |
| Package name | `foxglove-cloudini-converter` | `foxglove-cloudini-converter--voliro-custom` |

## What Was Changed

### 1. Schemas (`src/Schemas.tsx`)

Replaced the ROS2-based `CompressedPointCloud` and `PointCloud` types with Voliro-specific types:

- **`VolirotTimestamp`** — nanoseconds since epoch (`timestamp_ns: number`)
- **`Vector3d`** — 3D vector (`values: number[]`)
- **`Quaterniond`** — quaternion (`values_xyzw: number[]`)
- **`VolirotTransform`** — translation + rotation
- **`CloudiniCompressedPointCloud`** — the top-level input type with `timestamp`, `frame_id`, optional `pose`, and `compressed_data`

### 2. Message Converter Registration (`src/index.ts`)

- `fromSchemaName` changed to `"perception_types_ipc.proto.CloudiniCompressedPointCloud"`
- `toSchemaName` changed to `"foxglove.PointCloud"`

### 3. Point Cloud Converter (`src/PointCloudConverter.tsx`)

- **Timestamp conversion**: Converts `timestamp_ns` (nanoseconds) to Foxglove `{ seconds, nanos }` format
- **Pose conversion**: Maps `VolirotTransform` to Foxglove `Pose` (position + orientation), defaulting to identity pose when absent
- **Header parsing**: Since the Voliro message only carries `compressed_data` (no `width`, `height`, `point_step`, or `fields`), metadata is now extracted from the Cloudini header embedded in the compressed data:
  - Calls `_cldn_GetDecompressedSizeFromData()` to determine output buffer size
  - Calls `_cldn_GetHeaderAsYAML()` to extract field definitions and `point_stride`
  - A YAML parser maps Cloudini field types to Foxglove `NumericType` values

### 4. WASM Bindings (`src/cloudini_wasm_single.d.ts`)

Added TypeScript declarations for new WASM functions:
- `_cldn_GetDecompressedSizeFromData(inputPtr, inputSize)` — get decompressed buffer size from compressed data
- `_cldn_GetHeaderAsYAML(inputPtr, inputSize, outputPtr)` — extract header metadata as YAML string
- `UTF8ToString(ptr, maxBytesToRead?)` — Emscripten helper to read C strings

### 5. Package Metadata (`package.json`)

- Package name: `foxglove-cloudini-converter--voliro-custom`
- Publisher: `Cloudini + Voliro`
- Output file: `release/cloudini.foxglove-cloudini-converter--voliro-custom-<version>.foxe`

## How to Update Schemas

If the Voliro Protobuf schema changes (e.g., new fields added to `CloudiniCompressedPointCloud`), update the following files:

1. **`src/Schemas.tsx`** — Update the TypeScript types to match the new Protobuf definitions. The types here must mirror the wire format of `perception_types_ipc.proto.CloudiniCompressedPointCloud`.

2. **`src/PointCloudConverter.tsx`** — Update the converter functions if:
   - The timestamp format changes → update `convertTimestamp()`
   - The pose/transform format changes → update `convertPose()`
   - New fields are added that need mapping to `foxglove.PointCloud`

3. **`src/index.ts`** — Update `fromSchemaName` if the Protobuf schema is renamed.

## How to Rebuild the Extension

### Prerequisites

- Node.js 18+
- The compiled `cloudini_wasm_single.js` file (see [main README](../README.md) for WASM build instructions)

### Build Steps

```bash
cd cloudini_foxglove

# 1. Ensure the WASM module is up to date
#    Copy cloudini_wasm_single.js from the WASM build output into src/
cp ../build/wasm/cloudini_wasm_single.js src/

# 2. Install dependencies
npm install

# 3. Build and package the extension
npm run package
```

This produces a `.foxe` file in the `release/` directory:
```
release/cloudini.foxglove-cloudini-converter--voliro-custom-<version>.foxe
```

Upload the updated extension to voliro-main/applications/foxglove_studio_assets/ so it gets shipped with the release workflow.

### Install in Foxglove

1. Open Foxglove Studio
2. Go to **Settings → Extensions**
3. Drag and drop the `.foxe` file into the extension manager

### Local Development

To install directly into Foxglove for development/testing:

```bash
npm run local-install
```

### Building the WASM Module

The Foxglove extension requires a compiled WebAssembly module (`cloudini_wasm_single.js`). This section covers both first-time setup and subsequent rebuilds.

#### 1. Install Emscripten SDK

If you don't have Emscripten installed yet:

```bash
# Clone the SDK
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk

# Install and activate the latest version
./emsdk install latest
./emsdk activate latest

# Add to your current shell (add to .bashrc/.zshrc for persistence)
source ./emsdk_env.sh
```

Verify the installation:

```bash
emcc --version
```

#### 2. Build the WASM Module

From the repository root:

```bash
# Configure the build with the Emscripten toolchain
cmake -B build/wasm -S cloudini_lib \
  -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake \
  -DCMAKE_BUILD_TYPE=Release

# Build (this produces cloudini_wasm_single.js in build/wasm/)
cmake --build build/wasm --parallel
```

The CMake configuration automatically:
- Builds with `-s SINGLE_FILE=1` so the WASM binary is embedded in the `.js` file
- Sets `ENVIRONMENT='web'` and `FILESYSTEM=0` for browser use
- Exports the required C functions (`_cldn_DecodeCompressedData`, `_cldn_GetDecompressedSizeFromData`, `_cldn_GetHeaderAsYAML`, etc.)
- Configures 64 MB initial / 128 MB max memory with growth enabled

#### 3. Copy to the Extension

```bash
cp build/wasm/cloudini_wasm_single.js cloudini_foxglove/src/
```

#### Subsequent Rebuilds

After modifying C++ code in `cloudini_lib/`, you only need to repeat steps 2 and 3 (the CMake configure step can be skipped if already done):

```bash
source /path/to/emsdk/emsdk_env.sh  # if not already in your shell
cmake --build build/wasm --parallel
cp build/wasm/cloudini_wasm_single.js cloudini_foxglove/src/
```

If new C functions are exported from `cloudini_lib/src/wasm_functions.cpp`:
1. Add the function name to `CLD_EXPORTED_FUNCTIONS` in [cloudini_lib/CMakeLists.txt](../cloudini_lib/CMakeLists.txt) (around line 140)
2. Add the TypeScript declaration in `src/cloudini_wasm_single.d.ts`

## Version Bumping

Update the version in `package.json` before packaging a new release. The version is embedded in the output `.foxe` filename.
