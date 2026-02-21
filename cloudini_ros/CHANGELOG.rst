^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudini_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-02-21)
-------------------
* feat: make topic_converter composable for component container usage
  Build topic_converter as a shared component library with
  rclcpp_components, while keeping the standalone executable via
  EXECUTABLE directive. Intra-process comms moved into constructor
  so it works in both standalone and component container modes.
* feat: add convenience API for compressing PointCloud2 directly (`#58 <https://github.com/facontidavide/cloudini/issues/58>`_)
  Add SerializeCompressedPointCloud2() and ConvertToRosPointCloud2() to allow
  users to compress sensor_msgs::msg::PointCloud2 without the topic_converter
  node. Includes test_direct_publisher example node and CLAUDE.md documentation.
  Also removes redundant PCL_INCLUDE_DIRS from test_cloudini_subscriber target.
* Fixed cmake issues and added some improvements (`#55 <https://github.com/facontidavide/cloudini/issues/55>`_)
* fix resolution profile not applied in ros_topic_converter (`#49 <https://github.com/facontidavide/cloudini/issues/49>`_)
* Contributors: Alireza Moayyedi, Davide Faconti, ペンギンの何か

0.11.1 (2025-12-12)
-------------------
* try fixing build in ROS
* Contributors: Davide Faconti

0.11.0 (2025-11-29)
-------------------
* fix typo (`#42 <https://github.com/facontidavide/cloudini/issues/42>`_)
* add better Draco benchmarking
* Contributors: Bastian Lampe, Davide Faconti

0.10.0 (2025-10-13)
-------------------
* cherry picking change from `#38 <https://github.com/facontidavide/cloudini/issues/38>`_ . Better function name
  Thanks @Tanishq30052002 for the suggestions
* tons of ROS examples and utilities
* files renamed
* Contributors: Davide Faconti

0.9.0 (2025-10-11)
------------------
* fix compilation
* fix mcap_converter
* Contributors: Davide Faconti

0.8.0 (2025-10-09)
------------------
* Merge branch 'main' into msadowski/release_foxglove_extension
* Merge pull request `#33 <https://github.com/facontidavide/cloudini/issues/33>`_ from facontidavide/refactor_ros_interface
  Refactor ros interface
* fixed topic_converter
* changes API
* fixed benchmark
* updated benchmarks
* Preprare ros release (`#28 <https://github.com/facontidavide/cloudini/issues/28>`_)
* Contributors: Davide Faconti

0.7.0 (2025-09-19)
------------------

0.6.1 (2025-08-28)
------------------

0.5.0 (2025-06-30)
------------------

0.4.0 (2025-06-15)
------------------
* updated README
* Contributors: Davide Faconti

0.3.3 (2025-06-11)
------------------

0.3.1 (2025-06-10)
------------------
* Merge branch 'main' of github.com:facontidavide/cloudini
* Update CMakeLists.txt
* Included ROS 2 lint testing (`#3 <https://github.com/facontidavide/cloudini/issues/3>`_)
  Co-authored-by: Davide Faconti <davide.faconti@gmail.com>
* Contributors: Alejandro Hernández Cordero, Davide Faconti

0.3.0 (2025-06-03)
------------------

0.2.0 (2025-05-31)
------------------
* fixes
* add efficient "bridge" between compressed and regular pointclouds
* add license
* Contributors: Davide Faconti
