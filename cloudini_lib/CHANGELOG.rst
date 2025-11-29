^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudini_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
