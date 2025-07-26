# Changelog

All notable changes to this project will be documented in this file.

## [0.0.1] - 2025-01-26

### Added
- Initial release of Cloudini Foxglove extension
- WebAssembly-based decompression of Cloudini compressed point clouds
- Support for converting CompressedPointCloud2 to PointCloud2 messages
- Proper memory management for WASM operations
- Error handling and logging

### Fixed
- WASM module loading race condition
- Memory safety issues with decodedData
- Correct WASM function names matching C++ API
