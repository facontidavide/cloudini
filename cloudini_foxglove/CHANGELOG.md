# Changelog

All notable changes to this project will be documented in this file.

## [1.4.0] - 2026-09-28

The extension version now follows the Cloudini release it is built from.

### Added
- Decodes the V6 wire format, the default of Cloudini 1.4.0. Extension 1.2.2 and
  earlier fail on V6 messages with "Unsupported encoding version. Current is:5, got: 6".

### Changed
- Faster decoding of V4, V5 and V6 clouds.
- Messages from a newer Cloudini fail with an error that asks to update the extension.

### Fixed
- Hardened decoding of corrupted input (V5 RLE heap overflow, writes past the output).

## [1.2.2] - 2026-06-04

- Released as an installable `.foxe` again.

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
