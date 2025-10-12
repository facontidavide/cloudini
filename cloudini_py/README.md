# Cloudini Python Decoder

This package demonstrates how to decode Cloudini-compressed point clouds using WebAssembly (via wasmtime) and convert them to NumPy arrays.

**Status**: ✅ Working! Successfully decodes compressed point clouds at ~5.9x compression ratio.

## File Structure

- **`cloudini_decoder.py`**: Reusable `CloudiniDecoder` class that decodes CompressedPointCloud2 DDS messages to numpy arrays
- **`decode_mcap.py`**: Command-line tool for decoding compressed point clouds from MCAP files
- **`example_direct_usage.py`**: Example showing how to use `CloudiniDecoder` directly in your code
- **`requirements.txt`**: Python dependencies
- **`rebuild_wasm_no_exceptions.sh`**: Script to rebuild WASM module with correct flags

## Requirements

- Python 3.8+
- Cloudini WASM module (built from cloudini_lib with exceptions disabled)

## Setup

1. **Build the WASM module** (if not already built):

   **Option A: Use the provided rebuild script (recommended)**:
   ```bash
   cd cloudini_py
   ./rebuild_wasm_no_exceptions.sh
   ```

   This script rebuilds the WASM module with exceptions disabled (`-s DISABLE_EXCEPTION_CATCHING`), which is required for wasmtime compatibility.

   **Option B: Manual build**:
   ```bash
   cd /home/davide/ws_compression/src/clooudini
   cmake -B build_wasm -S cloudini_lib -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake
   cmake --build build_wasm
   ```
   Note: The manual build uses exception handling that may not work properly with standalone WASM runtimes.

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

### As a Command-Line Tool

Decode compressed point clouds from MCAP files:

```bash
# Basic usage (decodes first 3 messages by default)
python decode_mcap.py ../DATA/dexory_encoded.mcap
```

### Decode specific number of messages
```bash
python decode_mcap.py ../DATA/dexory_encoded.mcap --max-messages 10
```

### Decode all messages
```bash
python decode_mcap.py ../DATA/dexory_encoded.mcap --max-messages -1
```

### Specify custom WASM module location
```bash
python decode_mcap.py ../DATA/dexory_encoded.mcap --wasm /path/to/cloudini_wasm.wasm
```

### As a Python Library

Use the `CloudiniDecoder` class directly in your Python code:

```python
from cloudini_decoder import CloudiniDecoder

# Initialize the decoder
decoder = CloudiniDecoder("path/to/cloudini_wasm.wasm")

# Decode a raw DDS message (bytes)
# This could come from MCAP, ROS2, network, file, etc.
point_cloud, header = decoder.decode_message(compressed_msg_bytes)

# point_cloud is a numpy array of shape (num_points, point_step)
print(f"Shape: {point_cloud.shape}")
print(f"Data type: {point_cloud.dtype}")
print(f"Header: {header}")

# Reuse the decoder for multiple messages
for msg in message_stream:
    point_cloud, header = decoder.decode_message(msg)
    # Process point_cloud...
```

See `example_direct_usage.py` for more details.

## How It Works

1. **WASM Module Loading**: Uses wasmtime to load the Cloudini WASM module
2. **MCAP Reading**: Uses the mcap library to read messages from the MCAP file
3. **Message Decoding**: Calls WASM functions to decode compressed point clouds:
   - `cldn_GetDecompressedSize` - Get size needed for output buffer
   - `cldn_DecodeCompressedMessage` - Decode the compressed DDS message
   - `cldn_GetHeaderAsYAML` - Extract header metadata
4. **NumPy Conversion**: Converts raw bytes to NumPy arrays

## API Reference

### CloudiniDecoder Class

Main class for decoding compressed point clouds from DDS messages.

**Constructor:**
```python
CloudiniDecoder(wasm_path: str)
```
- `wasm_path`: Path to the cloudini WASM module (e.g., `"build_wasm/cloudini_wasm.wasm"`)

**Methods:**

#### `decode_message(compressed_msg: bytes, verbose: bool = True) -> tuple[np.ndarray, dict]`
Decodes a CompressedPointCloud2 DDS message to a numpy array.

**Parameters:**
- `compressed_msg`: Raw DDS message bytes containing CompressedPointCloud2
- `verbose`: Whether to print progress messages (default: True)

**Returns:**
- `point_cloud`: Numpy array of shape `(num_points, point_step)` with dtype `uint8`
- `header`: Dictionary with keys `'width'`, `'height'`, `'point_step'`

**Example:**
```python
decoder = CloudiniDecoder("path/to/cloudini_wasm.wasm")
point_cloud, header = decoder.decode_message(compressed_msg_bytes)

print(f"Decoded {header['width']} x {header['height']} point cloud")
print(f"Shape: {point_cloud.shape}")  # e.g., (262144, 16)
```

### decode_mcap_file Function

Convenience function to decode all compressed point clouds from an MCAP file.

**Signature:**
```python
decode_mcap_file(
    mcap_path: str,
    wasm_path: str,
    max_messages: int = None
)
```

**Parameters:**
- `mcap_path`: Path to the MCAP file
- `wasm_path`: Path to the cloudini WASM module
- `max_messages`: Maximum number of messages to decode (None = decode all)

**Example:**
```python
from decode_mcap import decode_mcap_file

decode_mcap_file(
    mcap_path="data/recording.mcap",
    wasm_path="build_wasm/cloudini_wasm.wasm",
    max_messages=10
)
```

## Output

The script prints:
- MCAP file summary (channels, message counts, timestamps)
- Per-message decoding results
- Header information (width, height, point_step, encoding options)
- Point cloud shape and data type
- Compression ratios

Example output:
```
Loading WASM module from /home/davide/ws_compression/src/clooudini/build_wasm/cloudini_wasm.wasm
Available exports with types: {'memory': 'Memory', '__wasm_call_ctors': 'Func', 'cldn_GetHeaderAsYAML': 'Func', ...}
WASM module loaded successfully!

Reading MCAP file: /home/davide/ws_compression/src/clooudini/DATA/dexory_encoded.mcap

MCAP Summary:
  Duration: 1717690291.89s to 1717690314.92s
  Message count: 457
  Channels: 1
    Channel 1: /lidars/os1_cover_filtered (cdr)

--- Message 1 ---
Topic: /lidars/os1_cover_filtered
Timestamp: 1717690291.886s
Input size: 712477 bytes, decompressed: 4194304 bytes
✓ Decoded successfully!
  Point cloud shape: (262144, 16)
  Data type: uint8
  Compression ratio: 5.89x

--- Message 2 ---
Topic: /lidars/os1_cover_filtered
Timestamp: 1717690291.930s
Input size: 712609 bytes, decompressed: 4194304 bytes
✓ Decoded successfully!
  Point cloud shape: (262144, 16)
  Data type: uint8
  Compression ratio: 5.89x

--- Message 3 ---
Topic: /lidars/os1_cover_filtered
Timestamp: 1717690291.971s
Input size: 712609 bytes, decompressed: 4194304 bytes
✓ Decoded successfully!
  Point cloud shape: (262144, 16)
  Data type: uint8
  Compression ratio: 5.89x

Reached max messages limit (3)

=== Summary ===
Total messages: 3
Successfully decoded: 3
```

## Technical Details

### Exception Handling
The WASM module must be built with exceptions disabled (`-s DISABLE_EXCEPTION_CATCHING`) for compatibility with wasmtime. Emscripten's default JavaScript-based exception handling doesn't work with standalone WASM runtimes.

The `rebuild_wasm_no_exceptions.sh` script automatically handles this by temporarily modifying the CMake flags.

### Memory Management
- Uses wasmtime's `malloc`/`free` exports for WASM memory allocation
- Memory is automatically managed within each decode operation
- WASM memory is configured with 64MB initial size and can grow as needed

### Decoder Implementation
- Uses `cldn_ConvertCompressedMsgToPointCloud2Msg` WASM function
- This function:
  1. Deserializes the CompressedPointCloud2 DDS message
  2. Decodes the compressed data using Cloudini's decoder
  3. Serializes the result as a PointCloud2 DDS message
  4. Returns the serialized message which is then converted to numpy

## Notes

- The decoded point cloud is returned as a raw byte array (numpy uint8)
- For structured access (x, y, z, rgb, etc.), you need to parse the field definitions from the PointCloud2 message header
- The WASM module must match the encoding version used to compress the data
- Current implementation uses a simplified header extraction (hardcoded width/height/point_step)

## Troubleshooting

### "WASM module not found"
Make sure you've built the WASM module:
```bash
cd /home/davide/ws_compression/src/clooudini
cmake -B build_wasm -S cloudini_lib -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake
cmake --build build_wasm
```

### "Failed to decode message"
- Check that the MCAP file contains CompressedPointCloud2 messages
- Verify the message encoding matches the Cloudini version
- Try increasing WASM memory limits in `CloudiniDecoder.__init__()`

### Import errors
Make sure the virtual environment is activated:
```bash
source venv/bin/activate
pip install -r requirements.txt
```
