# Cloudini Python Decoder

Python decoder for Cloudini-compressed point clouds that uses WebAssembly (via wasmtime) and outputs structured NumPy arrays with named fields.

## Features

✅ **Structured NumPy Arrays**: Returns point clouds with named fields (x, y, z, intensity, etc.)
✅ **Efficient Memory Management**: Pre-allocated 32MB output buffer for optimal performance
✅ **Type-Safe**: Automatic mapping from Cloudini types to NumPy dtypes
✅ **Handles Complex Layouts**: Supports non-contiguous field offsets
✅ **Reusable Decoder**: Single instance can decode multiple messages
✅ **MCAP Integration**: Command-line tool for batch processing MCAP files

## File Structure

- **`cloudini_decoder.py`**: Reusable `CloudiniDecoder` class for decoding CompressedPointCloud2 DDS messages
- **`decode_mcap.py`**: Command-line tool for decoding from MCAP files with statistics
- **`requirements.txt`**: Python dependencies (wasmtime, mcap, numpy)
- **`IMPLEMENTATION_NOTES.md`**: Technical documentation and architecture details
- **`REFACTORING_SUMMARY.md`**: Development history and design decisions

## Requirements

- Python 3.8+
- Cloudini WASM module (built from cloudini_lib)
- Emscripten SDK (for building WASM module)

## Setup

1. **Build the WASM module** (if not already built):

   To build the Python-compatible WASM module:

   ```bash
   cd /path/to/cloudini

   # First-time setup: Configure the build
   cmake -B build_wasm -S cloudini_lib \
     -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake

   # Build the cloudini_wasm target (for Python/Wasmtime)
   cmake --build build_wasm --target cloudini_wasm
   ```

   This creates:
   - `build_wasm/cloudini_wasm.wasm` - Pure WASM binary (for Wasmtime)
   - `build_wasm/cloudini_wasm.js` - JavaScript glue code

2. **Create a virtual environment**:
   ```bash
   cd cloudini_py
   python3 -m venv venv
   source venv/bin/activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

### Command-Line Tool

Decode compressed point clouds from MCAP files:

```bash
# Decode all messages from an MCAP file
python decode_mcap.py /path/to/encoded.mcap

# Decode only first 10 messages
python decode_mcap.py /path/to/encoded.mcap --max-messages 10

# Use custom WASM module path
python decode_mcap.py /path/to/encoded.mcap --wasm ./cloudini_wasm.wasm
```

**Output Example:**
```
--- Message 1 ---
Topic: /points
Timestamp: 1715873853.286s
✓ Decoded successfully!
  Data type: [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('ring', '<u2'), ('timestamp', '<f8')]
  Fields: x, y, z, intensity, ring, timestamp
  First point: (0., 0., 0., 0., 0, 1.71460504e+09)
  Compression ratio: 1.70x
```

### As a Python Library

Use the `CloudiniDecoder` class directly in your Python code:

```python
from cloudini_decoder import CloudiniDecoder
import numpy as np

# Initialize the decoder once
decoder = CloudiniDecoder("cloudini_wasm.wasm")

# Decode a raw DDS message (CompressedPointCloud2)
# This could come from MCAP, ROS2, network, file, etc.
point_cloud, header = decoder.decode_message(compressed_dds_message)

# point_cloud is a structured numpy array with named fields
print(f"Number of points: {len(point_cloud)}")
print(f"Field names: {point_cloud.dtype.names}")
print(f"First point: {point_cloud[0]}")

# Access individual fields (returns 1D arrays)
x = point_cloud['x']  # All X coordinates
y = point_cloud['y']  # All Y coordinates
z = point_cloud['z']  # All Z coordinates
intensity = point_cloud['intensity']

# Perform operations on fields
distances = np.sqrt(x**2 + y**2 + z**2)
print(f"Average distance from origin: {distances.mean():.3f}m")

# Filter points (boolean indexing)
nearby_points = point_cloud[distances < 5.0]
print(f"Points within 5m: {len(nearby_points)}")

# Convert to regular arrays for libraries that don't support structured arrays
xyz = np.column_stack([point_cloud['x'],
                       point_cloud['y'],
                       point_cloud['z']])  # Shape: (N, 3)

# Reuse the decoder for multiple messages (efficient)
for msg in message_stream:
    point_cloud, header = decoder.decode_message(msg)
    # Process point_cloud...
```

### Output Format

The decoder returns a tuple of `(point_cloud, header)`:

- **`point_cloud`**: NumPy structured array with named fields
  - Shape: `(num_points,)`
  - Each point is accessed by field name: `point_cloud['x']`, `point_cloud['intensity']`, etc.
  - Supports all NumPy operations: slicing, boolean indexing, arithmetic

- **`header`**: Dictionary with metadata
  - `width`, `height`: Point cloud dimensions
  - `point_step`: Bytes per point
  - `encoding_opt`: Encoding option (LOSSY, LOSSLESS, NONE)
  - `compression_opt`: Compression option (LZ4, ZSTD, NONE)
  - `fields`: List of field definitions (name, offset, type, resolution)

## How It Works

### Architecture Overview

```
MCAP File
    ↓
decode_mcap.py (iterates messages)
    ↓
CloudiniDecoder.decode_message()
    ↓
┌─────────────────────────────────────┐
│  WASM Module (cloudini_wasm)        │
│                                     │
│  1. cldn_GetHeaderAsYAMLFromDDS     │
│     → Extract header as YAML        │
│                                     │
│  2. cldn_DecodeCompressedMessage    │
│     → Decompress point cloud data   │
│     → DDS CompressedPointCloud2     │
│     → DDS PointCloud2              │
└─────────────────────────────────────┘
    ↓
Parse YAML header (get field definitions)
    ↓
Extract point data from PointCloud2 DDS
    ↓
Create structured NumPy array
    ↓
Return (point_cloud, header)
```

### Key Steps

1. **WASM Initialization**:
   - Load WASM module using wasmtime
   - Configure WASI (stdin/stdout/stderr)
   - Create stub functions for C++ runtime imports
   - Pre-allocate 32MB output buffer for efficiency

2. **Header Extraction**:
   - Call `cldn_GetHeaderAsYAMLFromDDS` on compressed message
   - Parse YAML to extract field definitions (name, offset, type, resolution)
   - Build field metadata dictionary

3. **Decompression**:
   - Call `cldn_DecodeCompressedMessage` with pre-allocated buffer
   - Decompress and decode point cloud data
   - Extract raw bytes from PointCloud2 DDS message

4. **NumPy Conversion**:
   - Map Cloudini types (FLOAT32, UINT16, etc.) to NumPy dtypes
   - Create structured dtype with explicit field offsets
   - Handle non-contiguous field layouts (e.g., gaps between fields)
   - Return structured array with named fields

### Supported Field Types

- `FLOAT32` → `np.float32`
- `FLOAT64` → `np.float64`
- `UINT8` → `np.uint8`
- `UINT16` → `np.uint16`
- `UINT32` → `np.uint32`
- `INT8` → `np.int8`
- `INT16` → `np.int16`
- `INT32` → `np.int32`

## Performance

### Benchmark Results

Tested on real-world MCAP files:

| Dataset | Points/msg | Compressed Size | Uncompressed Size | Ratio |
|---------|-----------|----------------|-------------------|-------|
| dexory_encoded.mcap | 131,072 (1024×128) | ~712 KB | 4.19 MB | **5.89x** |
| bount_encoded.mcap | 64,000 (64000×1) | ~979 KB | 1.66 MB | **1.70x** |

### Optimizations

- **Pre-allocated buffer**: 32MB output buffer allocated once at initialization
- **Zero-copy reads**: Direct memory access from WASM to Python
- **Reusable decoder**: Single instance processes unlimited messages
- **Efficient structured arrays**: No data duplication for field access

### Memory Usage

- **WASM heap**: ~32-40 MB (includes pre-allocated output buffer)
- **Per message**: Minimal additional allocation (input buffer only)
- **NumPy array**: Point step × number of points (1-5 MB typical)

## Troubleshooting

### WASM module not found

```
Error: WASM module not found at /path/to/cloudini_wasm.wasm
```

**Solution**: Build the WASM module first:
```bash
cmake -B build_wasm -S cloudini_lib \
  -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake
cmake --build build_wasm --target cloudini_wasm
```

### Import errors

```
ModuleNotFoundError: No module named 'wasmtime'
```

**Solution**: Install dependencies:
```bash
pip install -r requirements.txt
```

### Memory errors with large point clouds

```
RuntimeError: Failed to allocate memory
```

**Solution**: The decoder pre-allocates 32MB. For larger point clouds, edit `cloudini_decoder.py` line 123:
```python
self.output_ptr = self.allocate(64 * 1024 * 1024)  # Increase to 64MB
```

### Decoding failures

```
RuntimeError: Failed to convert compressed message to PointCloud2
```

**Possible causes**:
- Corrupted MCAP file
- Wrong message type (not CompressedPointCloud2)
- Incompatible Cloudini version

**Debug steps**:
1. Verify message type: `ros2 topic info /topic_name` (for ROS2 bags)
2. Test with C++ tool: `./mcap_header_inspector -f file.mcap`
3. Check WASM module version matches compressed data

## Additional Resources

- **Technical details**: See [IMPLEMENTATION_NOTES.md](IMPLEMENTATION_NOTES.md)
- **Development history**: See [REFACTORING_SUMMARY.md](REFACTORING_SUMMARY.md)
- **Cloudini library**: See [../cloudini_lib/](../cloudini_lib/)
- **NumPy structured arrays**: https://numpy.org/doc/stable/user/basics.rec.html
