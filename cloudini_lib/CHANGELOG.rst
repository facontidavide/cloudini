^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudini_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-02-21)
-------------------
* minor changes
* feat: add bag directory support and metadata.yaml generation to rosbag converter
  The converter now accepts ROS2 bag directories as input (in addition to
  bare .mcap files), auto-detects sibling metadata.yaml, and generates a
  transformed metadata.yaml alongside the output with updated topic types
  and file references. Output defaults to a new sibling directory to
  prevent accidental overwrites of the original bag.
  Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>
* feat: add single-threaded compression mode to PointcloudEncoder
  Add `use_threads` field to EncodingInfo (defaults to true). When false,
  LZ4/ZSTD compression runs inline in the calling thread instead of
  spawning a worker thread. Useful for simpler deployments, debugging,
  and environments where threading overhead isn't worth the pipelining
  benefit.
  No wire format changes — the field is runtime-only and not serialized.
* Fixed cmake issues and added some improvements (`#55 <https://github.com/facontidavide/cloudini/issues/55>`_)
* fix: harden encoder/decoder against UB, data races, and corrupted input (`#57 <https://github.com/facontidavide/cloudini/issues/57>`_)
  * fix: harden encoder/decoder against UB, data races, and corrupted input
  Address issues found during code review:
  - Fix decodeVarint shift UB: check overflow before shift operation
  - Fix decodeVarint underflow: guard against uval==0 on corrupted data
  - Fix worker thread deadlock: detect dead worker on subsequent encode()
  - Fix WASM encode: avoid extra allocation by encoding directly when buffer fits
  - Clean up waitForCompressionComplete: only set worker_failed\_ in error path
  - Optimize encode(BufferView&): use header\_.size() instead of YAML reserialization
  - Improve readability and error messages in DecodeHeader/MaxSerializedFieldSize
  - Add explicit <cstring> include in encoding_utils.hpp
  - Fix pre-existing FloatLossy test bug: use float instead of double for kResolution
  - Re-enable previously commented-out tests
  * fix: additional hardening for decoder bounds checks and encoder thread safety
  - Add per-point bounds check in decodeChunk to replace per-field checks in
  fixed-size decoders (FieldDecoderCopy, FieldDecoderFloat_XOR), eliminating
  12-14% decode overhead while maintaining safety
  - Keep per-field checks for variable-size decoders (Float_Lossy, FloatN_Lossy)
  where overhead is negligible relative to varint decoding
  - Fix encoder deadlock on re-use after worker thread failure by joining dead
  thread and re-spawning; protect shared state resets with mutex
  - Tighten decodeVarint overflow check from shift>=64 to shift>=63
  - Derive point count from actual data size in ROS conversion instead of
  trusting metadata width*height to prevent over-allocation DoS
  * fix: remove non-existent apt package from Jazzy CI workflow
  python3-ament-package does not exist on Ubuntu 24.04. The setup-ros
  action and action-ros-ci already handle installing colcon and ament
  dependencies, matching the working Humble CI workflow.
  * fix: install ros-humble-ament-cmake-test for test runner
  * revert some changes
  ---------
  Co-authored-by: Claude Opus 4.6 <noreply@anthropic.com>
* Need to add these constructors for emscripten build to work on ubuntu 24 (`#54 <https://github.com/facontidavide/cloudini/issues/54>`_)
  * Need to add these constructors for emscripten build to work
  * add default ctors and default values
  ---------
* Contributors: Alireza Moayyedi, Davide Faconti, tomdeblauwe

0.11.1 (2025-12-12)
-------------------
* fix ROS compilation
* Contributors: Davide Faconti

0.11.0 (2025-11-29)
-------------------
* optimize yaml parser
* yaml parser
* fix benchmark
* add better Draco benchmarking
* Contributors: Davide Faconti

0.10.0 (2025-10-13)
-------------------
* working on the python code
* critical bug fix
* update
* try fixing multi-threading code
* cherry picking change from `#38 <https://github.com/facontidavide/cloudini/issues/38>`_ . Better function name
  Thanks @Tanishq30052002 for the suggestions
* tons of ROS examples and utilities
* Contributors: Davide Faconti

0.9.0 (2025-10-11)
------------------
* fix error in wasm module
* fix mcap_converter
* remove redundancy
* Fix redundancy in ros msg utils (`#37 <https://github.com/facontidavide/cloudini/issues/37>`_)
  * remove redundant function
  * remove code redundancy
  * move header decoding to common
* Contributors: Davide Faconti, Tanishq Chaudhary

0.8.0 (2025-10-09)
------------------
* Merge branch 'main' into msadowski/release_foxglove_extension
* Merge pull request `#33 <https://github.com/facontidavide/cloudini/issues/33>`_ from facontidavide/refactor_ros_interface
* updated wasm plugin
* changes API
* Remove 'lz4' compression method support (`#32 <https://github.com/facontidavide/cloudini/issues/32>`_)
* use function argument
* force vendoring?
* Preprare ros release (`#28 <https://github.com/facontidavide/cloudini/issues/28>`_)
* Contributors: Davide Faconti

0.7.0 (2025-09-19)
------------------
* Merge pull request `#27 <https://github.com/facontidavide/cloudini/issues/27>`_ from facontidavide/yaml_encoding
  Yaml encoding
* each chunk should reset the state of encoders/decoders to allow parallel extraction
* fix PCD conversion
* use YAML instead
* add JSON encoding to header and WIP pcl_converter
* version 03: add multi-threading and chunks
* Contributors: Davide Faconti

0.6.1 (2025-08-28)
------------------
* bug fix (memory boundaries) and typo addressed
* better benchmark print
* Contributors: Davide Faconti

0.5.0 (2025-06-30)
------------------
* fix 64 bits types
* don't create an encoder at each loop
* add mcap cutter utility
* speedup in rosbag conversion
* fix compilation (clang++ 20) (`#18 <https://github.com/facontidavide/cloudini/issues/18>`_)
  Co-authored-by: Giuseppe Rizzi <giuseppe.rizzi@ascento.ch>
* Contributors: Davide Faconti, Giuseppe Rizzi

0.4.0 (2025-06-15)
------------------
* downgrade MCAP for compatibility with ROS2 Jazzy
* make MCAP a private dependency and copy metadata (`#17 <https://github.com/facontidavide/cloudini/issues/17>`_)
  * make MCAP a private dependency and copy metadata
  * minor cleanup
* fix buffer size in worst case scenario (`#16 <https://github.com/facontidavide/cloudini/issues/16>`_)
  Co-authored-by: Giuseppe Rizzi <giuseppe.rizzi@ascento.ch>
* Contributors: Davide Faconti, Giuseppe Rizzi

0.3.3 (2025-06-11)
------------------
* Compression profile (MCAP writer) (`#14 <https://github.com/facontidavide/cloudini/issues/14>`_)
  Co-authored-by: Giuseppe Rizzi <giuseppe.rizzi@ascento.ch>
* Null character termination (`#13 <https://github.com/facontidavide/cloudini/issues/13>`_)
* add experimental WASM + web tester
* Contributors: Davide Faconti, Giuseppe Rizzi

0.3.1 (2025-06-10)
------------------
* fix formatting
* Merge branch 'main' of github.com:facontidavide/cloudini
* small speed optimization
* fix bugs in PCL
* fix compilation on arm (`#9 <https://github.com/facontidavide/cloudini/issues/9>`_)
  Co-authored-by: Giuseppe Rizzi <giuseppe.rizzi@ascento.ch>
* Contributors: Davide Faconti, Giuseppe Rizzi

0.3.0 (2025-06-03)
------------------
* PCL conversion fixed and tested
* same speed with varint 64
* small fix
* Contributors: Davide Faconti

0.2.0 (2025-05-31)
------------------
* faster DDS decompression with less copies
* add license
* Contributors: Davide Faconti
