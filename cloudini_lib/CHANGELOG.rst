^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudini_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
