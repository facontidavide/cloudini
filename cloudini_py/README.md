# Cloudini Python Decoder

This package demonstrates how to decode Cloudini-compressed point clouds using WebAssembly (via wasmtime) and convert them to NumPy arrays.

## File Structure

- **`cloudini_decoder.py`**: Reusable `CloudiniDecoder` class that decodes CompressedPointCloud2 DDS messages to numpy arrays
- **`decode_mcap.py`**: Command-line tool for decoding compressed point clouds from MCAP files
- **`requirements.txt`**: Python dependencies

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

Decode compressed point clouds from MCAP files:

```bash
# Basic usage
python decode_mcap.py my_encoded_rosbag.mcap
```

### As a Python Library

Use the `CloudiniDecoder` class directly in your Python code:

```python
from cloudini_decoder import CloudiniDecoder

# Initialize the decoder
decoder = CloudiniDecoder("path/to/cloudini_wasm.wasm")

# Decode a raw DDS message (bytes)
# This could come from MCAP, ROS2, network, file, etc.
point_cloud, header = decoder.decode_message(compressed_dds_message)

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
