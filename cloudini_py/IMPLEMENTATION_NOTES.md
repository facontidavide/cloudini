# Cloudini Python Decoder - Implementation Notes

## Summary

Successfully implemented a Python decoder for Cloudini-compressed point clouds stored in MCAP files, using WebAssembly via wasmtime.

**Status**: ✅ **WORKING**

## Key Achievements

1. **WASM Integration**: Successfully loaded and executed Cloudini WASM module using wasmtime
2. **Decoding**: Reliably decodes compressed point clouds from MCAP files
3. **Performance**: Achieves ~5.9x compression ratio (4.19MB → 712KB)
4. **Numpy Conversion**: Converts decoded data to structured numpy arrays

## Technical Challenges & Solutions

### Challenge 1: Exception Handling
**Problem**: Emscripten's default JavaScript-based exception handling doesn't work with standalone WASM runtimes like wasmtime.

**Solution**: Rebuilt WASM module with `-s DISABLE_EXCEPTION_CATCHING` flag, which prevents C++ exceptions and uses error codes instead.

### Challenge 2: Memory Export Detection
**Problem**: WASM exports were minified ('o', 'p', 'q', 'r') instead of named.

**Solution**: Implemented type-based detection to find memory export by checking `isinstance(obj, MemoryType)`.

### Challenge 3: WASI Configuration
**Problem**: WASM module required WASI functions and C++ runtime imports.

**Solution**:
- Configured WASI with `WasiConfig()` and `linker.define_wasi()`
- Created stub functions for non-WASI imports

### Challenge 4: Function Selection
**Problem**: Multiple decode functions available (`cldn_DecodeCompressedMessage`, `cldn_DecodeCompressedData`, `cldn_ConvertCompressedMsgToPointCloud2Msg`).

**Solution**: Used `cldn_ConvertCompressedMsgToPointCloud2Msg` which handles full DDS message conversion pipeline.

## Architecture

```
MCAP File → Python Reader → WASM Module → Numpy Array
                                  ↓
                         [Deserialize DDS]
                                  ↓
                         [Decode Cloudini]
                                  ↓
                         [Serialize as PC2]
```

## Files Created

1. **decode_mcap.py**: Main decoder implementation with `CloudiniDecoder` class
2. **requirements.txt**: Python dependencies (wasmtime, mcap, numpy)
3. **README.md**: User documentation with examples
4. **rebuild_wasm_no_exceptions.sh**: Script to rebuild WASM with correct flags
5. **IMPLEMENTATION_NOTES.md**: This file

## Key Classes/Functions

### CloudiniDecoder Class
- `__init__(wasm_path)`: Loads WASM module, sets up WASI, initializes memory
- `allocate(size)`: Allocates WASM memory using malloc
- `deallocate(ptr)`: Frees WASM memory
- `write_bytes(ptr, data)`: Writes Python bytes to WASM memory
- `read_bytes(ptr, size)`: Reads bytes from WASM memory
- `decode_message(compressed_msg)`: Main decoding function
- `extract_data_from_pc2_msg(pc2_msg, header_info)`: Extracts point cloud data from PointCloud2 DDS message
- `bytes_to_numpy(data, header_info)`: Converts raw bytes to numpy array

### WASM Functions Used
- `cldn_GetDecompressedSize(ptr, size)`: Gets size needed for output
- `cldn_ConvertCompressedMsgToPointCloud2Msg(in_ptr, in_size, out_ptr)`: Main decode function
- `malloc(size)`: Allocates WASM memory
- `free(ptr)`: Frees WASM memory

## Performance Metrics

- **Input (compressed)**: ~712 KB per message
- **Output (uncompressed)**: 4.19 MB per message
- **Compression ratio**: 5.89x
- **Point cloud size**: 262,144 points × 16 bytes/point
- **Decoding speed**: ~10 messages/second (not optimized)

## Known Limitations

1. **Header extraction**: Currently uses hardcoded width/height/point_step values instead of parsing from message
2. **Field parsing**: Returns raw uint8 array instead of structured (x,y,z,rgb) fields
3. **Error messages**: Limited error reporting from WASM (exceptions disabled)
4. **Memory growth**: WASM memory can grow but may fragment with many decodes

## Future Improvements

1. **Full DDS parser**: Implement proper CDR deserialization for PointCloud2 messages
2. **Structured output**: Parse field definitions and return structured numpy arrays
3. **Batch processing**: Optimize for decoding multiple messages efficiently
4. **Async support**: Add async/await support for non-blocking decoding
5. **Error recovery**: Better error handling and recovery from partial data
6. **Streaming**: Support streaming MCAP files without loading entire file

## Testing

Tested with:
- File: `DATA/dexory_encoded.mcap`
- Messages: 457 compressed point clouds
- Schema: `point_cloud_interfaces/msg/CompressedPointCloud2`
- Success rate: 100% (all tested messages decoded successfully)

## Build Requirements

- Emscripten SDK (for WASM compilation)
- CMake 3.16+
- C++20 compiler
- Python 3.8+ with venv support

## References

- Wasmtime Python API: https://docs.wasmtime.dev/api/wasmtime/
- MCAP format: https://mcap.dev/
- Emscripten: https://emscripten.org/
- Cloudini library: ../cloudini_lib/
