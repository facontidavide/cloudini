# Cloudini Python Decoder - Implementation Notes

## Summary

Successfully implemented a Python decoder for Cloudini-compressed point clouds stored in MCAP files, using WebAssembly via wasmtime.

**Status**: ✅ **FULLY WORKING**

## Key Achievements

1. **WASM Integration**: Successfully loaded and executed Cloudini WASM module using wasmtime
2. **Modular Architecture**: Separated reusable decoder class from MCAP reading logic
3. **Structured Arrays**: Creates proper numpy structured arrays with named fields
4. **Performance**: Optimized memory allocation with pre-allocated output buffer (32MB)
5. **Compression Ratios**: Achieves 1.7x-5.9x depending on point cloud characteristics
6. **Field Type Mapping**: Automatic conversion from Cloudini types to numpy dtypes

## Technical Challenges & Solutions

### Challenge 1: Dual WASM Build Targets
**Problem**: Emscripten's exception handling differs between Python (wasmtime) and browser (Foxglove) use cases.

**Solution**: Created two separate WASM targets in CMakeLists.txt:
- `cloudini_wasm`: `-s DISABLE_EXCEPTION_CATCHING` for Python/wasmtime
- `cloudini_wasm_single`: `-s SINGLE_FILE=1 -s NO_DISABLE_EXCEPTION_CATCHING` for Foxglove

### Challenge 2: Memory Export Detection
**Problem**: WASM exports were minified ('o', 'p', 'q', 'r') instead of named.

**Solution**: Implemented type-based detection to find memory export by checking `isinstance(obj, MemoryType)`.

### Challenge 3: WASI Configuration
**Problem**: WASM module required WASI functions and C++ runtime imports.

**Solution**:
- Configured WASI with `WasiConfig()` and `linker.define_wasi()`
- Created stub functions for non-WASI imports with proper type handling

### Challenge 4: YAML Header Parsing
**Problem**: Initial implementation only parsed top-level fields, losing nested field definitions.

**Solution**: Implemented stateful YAML parser that tracks `in_fields_section` and properly parses nested structures:
```python
fields:
  - name: x
    offset: 0
    type: FLOAT32
    resolution: 0.001
```

### Challenge 5: Structured Numpy Arrays with Gaps
**Problem**: Point cloud fields aren't always contiguous (e.g., x,y,z at offsets 0,4,8 but intensity at offset 16).

**Solution**: Used numpy's explicit offset specification in dtype:
```python
dtype_spec = {
    'names': ['x', 'y', 'z', 'intensity'],
    'formats': [np.float32, np.float32, np.float32, np.float32],
    'offsets': [0, 4, 8, 16],
    'itemsize': 32  # Full point step size
}
```

### Challenge 6: Function Selection and Error Handling
**Problem**: Initial attempts with `cldn_GetHeaderAsYAML` caused WASM traps due to exceptions.

**Solution**: Switched to `cldn_GetHeaderAsYAMLFromDDS` and `cldn_DecodeCompressedMessage` which don't throw exceptions when built with `-s DISABLE_EXCEPTION_CATCHING`.

## Architecture

```
┌─────────────┐
│  MCAP File  │
└──────┬──────┘
       │
       ▼
┌─────────────────────┐
│  decode_mcap.py     │ ← User-facing script
│  - MCAP iteration   │
│  - Progress display │
└──────┬──────────────┘
       │
       ▼
┌──────────────────────────┐
│  CloudiniDecoder class   │ ← Reusable decoder
│  (cloudini_decoder.py)   │
└──────┬───────────────────┘
       │
       ├─ get_header_info() → Parse YAML header
       │                        Extract field definitions
       │
       └─ decode_message() ───┐
                               │
                               ▼
                    ┌──────────────────┐
                    │  WASM Module     │
                    │  cloudini_wasm   │
                    └──────┬───────────┘
                           │
                           ├─ cldn_GetHeaderAsYAMLFromDDS
                           │    → Extract header as YAML
                           │
                           └─ cldn_DecodeCompressedMessage
                                → DDS → PointCloud2 DDS
                                   (decompression happens here)
       │
       ▼
┌─────────────────────────┐
│  bytes_to_numpy()       │ ← Create structured array
│  - Field type mapping   │
│  - Handle offset gaps   │
└──────┬──────────────────┘
       │
       ▼
┌─────────────────────────┐
│  Structured Numpy Array │
│  point_cloud['x']       │
│  point_cloud['y']       │
│  point_cloud['z']       │
│  point_cloud['intensity']│
└─────────────────────────┘
```

## Files in cloudini_py/

1. **cloudini_decoder.py**: Reusable `CloudiniDecoder` class for WASM interaction
2. **decode_mcap.py**: Command-line tool for decoding MCAP files
3. **requirements.txt**: Python dependencies (wasmtime, mcap, numpy)
4. **README.md**: User documentation with build instructions and examples
5. **IMPLEMENTATION_NOTES.md**: This file (technical documentation)
6. **REFACTORING_SUMMARY.md**: Record of refactoring decisions

## Key Classes/Functions

### CloudiniDecoder Class (cloudini_decoder.py)

**Initialization:**
- `__init__(wasm_path)`:
  - Loads WASM module and compiles with wasmtime
  - Configures WASI (inherit stdin/stdout/stderr)
  - Creates stub functions for non-WASI imports
  - Pre-allocates 32MB output buffer (`self.output_ptr`)
  - Stores references to WASM functions

**Memory Management:**
- `allocate(size)`: Allocates WASM memory using malloc or bump allocator
- `deallocate(ptr)`: Frees WASM memory (if malloc/free available)
- `write_bytes(ptr, data)`: Writes Python bytes to WASM memory
- `read_bytes(ptr, size)`: Reads bytes from WASM memory to Python

**Decoding Pipeline:**
- `decode_message(compressed_msg, verbose=True)`:
  - Main entry point for decoding
  - Allocates input buffer, writes compressed DDS message
  - Calls `cldn_DecodeCompressedMessage` to decompress
  - Extracts header info and creates structured numpy array
  - Returns `(point_cloud, header_info)` tuple

- `get_header_info(compressed_msg)`:
  - Calls `cldn_GetHeaderAsYAMLFromDDS` to extract header as YAML
  - Parses YAML to extract: width, height, point_step, encoding_opt, compression_opt
  - Parses nested fields section with name, offset, type, resolution
  - Returns dict with all header information

- `extract_data_from_pc2_msg(pc2_msg, header_info)`:
  - Extracts raw point cloud data from PointCloud2 DDS message
  - Uses expected size calculation to locate data field
  - Calls `bytes_to_numpy()` for final conversion

- `bytes_to_numpy(data, header_info)`:
  - Maps Cloudini field types to numpy dtypes
  - Creates structured dtype with explicit offsets and itemsize
  - Handles non-contiguous field layouts
  - Falls back to byte array if structured array creation fails

### decode_mcap.py Functions

- `decode_mcap_file(mcap_path, wasm_path, max_messages)`:
  - Opens MCAP file and creates reader
  - Iterates through messages, filtering for CompressedPointCloud2
  - Decodes each message and displays statistics
  - Stops after `max_messages` if specified

### WASM Functions Used

- `cldn_DecodeCompressedMessage(in_ptr, in_size, out_ptr)`:
  - Main decoding function
  - Input: DDS-serialized CompressedPointCloud2 message
  - Output: DDS-serialized PointCloud2 message
  - Returns actual output size

- `cldn_GetHeaderAsYAMLFromDDS(in_ptr, in_size, out_ptr)`:
  - Extracts Cloudini header from CompressedPointCloud2 DDS message
  - Converts to YAML format
  - Returns YAML string size

- `malloc(size)`: Allocates WASM memory (optional, uses bump allocator if not available)
- `free(ptr)`: Frees WASM memory (optional)

## Performance Metrics

### Tested Files

**dexory_encoded.mcap:**
- Input (compressed): ~712 KB per message
- Output (uncompressed): 4.19 MB per message
- Compression ratio: **5.89x**
- Point cloud size: 131,072 points (1024×128)
- Point step: 32 bytes
- Fields: x, y, z, intensity (all FLOAT32)

**bount_encoded.mcap:**
- Input (compressed): ~979 KB per message
- Output (uncompressed): 1.66 MB per message
- Compression ratio: **1.70x**
- Point cloud size: 64,000 points (64000×1)
- Point step: 26 bytes
- Fields: x, y, z, intensity (FLOAT32), ring (UINT16), timestamp (FLOAT64)

### Optimizations Implemented

1. **Pre-allocated output buffer**: 32MB allocated once at initialization (`self.output_ptr`)
2. **Minimal allocations**: Reuses output buffer across all decode calls
3. **Direct memory access**: Zero-copy reads from WASM memory via `memory.read()`
4. **Structured arrays**: Efficient field access without data duplication

## Known Limitations

1. **CDR parsing simplification**: Extracts data by taking last N bytes instead of full CDR deserialization
2. **Error messages**: Limited error reporting from WASM (exceptions disabled for Python target)
3. **Fixed output buffer**: 32MB limit may be insufficient for very large point clouds
4. **YAML parsing**: Simple line-based parser, not a full YAML library

## Future Improvements

1. **Full CDR parser**: Implement proper CDR deserialization for PointCloud2 DDS messages
2. **Dynamic output buffer**: Allocate based on actual decompressed size
3. **Batch processing**: Process multiple messages with single WASM instance
4. **Async support**: Add async/await support for non-blocking decoding
5. **Visualization**: Integration with Open3D or similar for point cloud visualization
6. **Streaming**: Support streaming MCAP files without loading entire file into memory

## Testing

### Test Coverage

**Files tested:**
1. `DATA/dexory_encoded.mcap` - 457 messages, 100% success
2. `DATA/bount_encoded.mcap` - 1000 messages, tested first 2, 100% success

**Verified functionality:**
- YAML header parsing with nested field definitions
- Structured numpy array creation with correct dtypes
- Non-contiguous field offsets (x,y,z at 0,4,8 and intensity at 16)
- Mixed field types (FLOAT32, UINT16, FLOAT64)
- Individual field access: `point_cloud['x']`, `point_cloud['intensity']`, etc.
- Compression ratio calculation
- Max messages limit (`--max-messages` parameter)

**Test tools:**
- `decode_mcap.py`: Command-line decoding with statistics
- `mcap_header_inspector` (C++): Validates YAML header extraction matches C++ implementation

## Usage Examples

### Basic Decoding

```python
from cloudini_decoder import CloudiniDecoder

# Initialize decoder
decoder = CloudiniDecoder("cloudini_wasm.wasm")

# Decode a compressed DDS message
point_cloud, header = decoder.decode_message(compressed_msg_bytes)

# Access fields
x_coords = point_cloud['x']
y_coords = point_cloud['y']
z_coords = point_cloud['z']
intensity = point_cloud['intensity']

print(f"Points: {len(point_cloud)}")
print(f"Fields: {point_cloud.dtype.names}")
print(f"First point: {point_cloud[0]}")
```

### Field Access Patterns

```python
# Individual field access (returns 1D array)
x_values = point_cloud['x']  # shape: (N,)

# Access multiple fields
xyz = np.column_stack([point_cloud['x'],
                       point_cloud['y'],
                       point_cloud['z']])  # shape: (N, 3)

# Boolean indexing/filtering
distances = np.sqrt(point_cloud['x']**2 +
                   point_cloud['y']**2 +
                   point_cloud['z']**2)
nearby = point_cloud[distances < 5.0]

# Access by index
first_point = point_cloud[0]
print(f"x={first_point['x']}, y={first_point['y']}")
```

### Decoding from MCAP

```bash
# Decode all messages
python decode_mcap.py /path/to/file.mcap

# Decode first 10 messages only
python decode_mcap.py /path/to/file.mcap --max-messages 10

# Use custom WASM module path
python decode_mcap.py /path/to/file.mcap --wasm /path/to/cloudini_wasm.wasm
```

## Build Requirements

**For WASM compilation:**
- Emscripten SDK
- CMake 3.16+
- C++20 compiler

**For Python usage:**
- Python 3.8+
- wasmtime (Python package)
- numpy
- mcap (for MCAP file reading)

## Build Instructions

```bash
# 1. Build WASM module (from cloudini_lib directory)
cmake -B build_wasm -S . \
  -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake
cmake --build build_wasm --target cloudini_wasm

# 2. Copy WASM file to cloudini_py directory
cp build_wasm/cloudini_wasm.wasm ../cloudini_py/

# 3. Install Python dependencies
cd ../cloudini_py
pip install -r requirements.txt

# 4. Test decoder
python decode_mcap.py /path/to/test.mcap --max-messages 1
```

## References

- Wasmtime Python API: https://docs.wasmtime.dev/api/wasmtime/
- MCAP format: https://mcap.dev/
- Emscripten: https://emscripten.org/
- Numpy structured arrays: https://numpy.org/doc/stable/user/basics.rec.html
- Cloudini library: ../cloudini_lib/
