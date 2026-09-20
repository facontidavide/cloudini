^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudini_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2026-09-20)
------------------
* No changes; released together with cloudini_lib 1.3.1

1.3.0 (2026-09-20)
------------------
* Fix license tags in package.xml
  According to ros_license_toolkit[^1] license tags should be in SPDX
  list of licenses. In case of claudini_ros, the license version was
  missing and in case of claudini_lib, the license name was not "exact".
  This commit changes the license tags to use SPDX license identifiers.
  Note that it doesn't fix the failures reported in claudini_lib in the
  output below:
  [cloudini_ros]
  git hash of (/home/src/github.com/facontidavide/cloudini): d202e5255d12519ff1f3db1dac4df3ca0e550ee7
  SchemaCheck
  SUCCESS Detected package.xml version 3, validation of scheme successful.
  LicenseTagExistsCheck
  SUCCESS Found licenses ['Apache']
  LicenseTagIsInSpdxListCheck
  WARNING Licenses ['Apache'] are not in SPDX list of licenses. Make sure to exactly match one of https://spdx.org/licenses/.
  LicenseTextExistsCheck
  WARNING Since they are not in the SPDX list, we can not check if these tags have the correct license text:
  'Apache': License text file '../LICENSE' is of license Apache-2.0 but tag is Apache.
  LicensesInCodeCheck
  WARNING For the following files, please change the License Tag in the package file to SPDX format:
  'include/cloudini_plugin/cloudini_publisher_plugin.hpp' is of Apache-2.0 but its Tag is Apache.
  'include/cloudini_plugin/cloudini_subscriber_plugin.hpp' is of Apache-2.0 but its Tag is Apache.
  'include/cloudini_ros/cloudini_subscriber_pcl.hpp' is of Apache-2.0 but its Tag is Apache.
  'include/cloudini_ros/conversion_utils.hpp' is of Apache-2.0 but its Tag is Apache.
  'src/cloudini_publisher_plugin.cpp' is of Apache-2.0 but its Tag is Apache.
  'src/cloudini_subscriber_pcl.cpp' is of Apache-2.0 but its Tag is Apache.
  'src/cloudini_subscriber_plugin.cpp' is of Apache-2.0 but its Tag is Apache.
  'src/conversion_utils.cpp' is of Apache-2.0 but its Tag is Apache.
  'src/plugin_manifest.cpp' is of Apache-2.0 but its Tag is Apache.
  'src/topic_converter.cpp' is of Apache-2.0 but its Tag is Apache.
  'test/draco_helper.cpp' is of Apache-2.0 but its Tag is Apache.
  'test/draco_helper.hpp' is of Apache-2.0 but its Tag is Apache.
  'test/rosbag_benchmark.cpp' is of Apache-2.0 but its Tag is Apache.
  'test/test_cloudini_subscriber.cpp' is of Apache-2.0 but its Tag is Apache.
  'test/test_direct_publisher.cpp' is of Apache-2.0 but its Tag is Apache.
  'test/test_plugin_publisher.cpp' is of Apache-2.0 but its Tag is Apache.
  'test/test_plugin_subscriber.cpp' is of Apache-2.0 but its Tag is Apache.
  LicenseFilesReferencedCheck
  SUCCESS All license declaration are referenced by a tag.
  --------------------
  [cloudini_lib]
  git hash of (/home/src/github.com/facontidavide/cloudini): d202e5255d12519ff1f3db1dac4df3ca0e550ee7
  SchemaCheck
  SUCCESS Detected package.xml version 3, validation of scheme successful.
  LicenseTagExistsCheck
  SUCCESS Found licenses ['Apache 2.0']
  LicenseTagIsInSpdxListCheck
  WARNING Licenses ['Apache 2.0'] are not in SPDX list of licenses. Make sure to exactly match one of https://spdx.org/licenses/.
  LicenseTextExistsCheck
  WARNING Since they are not in the SPDX list, we can not check if these tags have the correct license text:
  'Apache 2.0': License text file '../LICENSE' is of license Apache-2.0 but tag is Apache 2.0.
  LicensesInCodeCheck
  FAILURE
  The following files contain licenses that are not covered by any license tag:
  'benchmarks/pcd_benchmark.cpp': ['MIT']
  'cmake/CPM.cmake': ['MIT']
  'include/cloudini_lib/ros_message_definitions.hpp': ['BSD-3-Clause']
  'include/cloudini_lib/contrib/ankerl/stl.h': ['MIT']
  'include/cloudini_lib/contrib/ankerl/unordered_dense.h': ['MIT']
  LicenseFilesReferencedCheck
  SUCCESS All license declaration are referenced by a tag.
  [^1]: https://github.com/boschresearch/ros_license_toolkit
* Contributors: Michal Sojka

1.2.2 (2026-06-04)
------------------
* build: synchronized version bump with cloudini_lib (MSVC support; no functional changes here)

1.2.1 (2026-05-20)
------------------
* ci(rolling): build point_cloud_interfaces from source via vcstool until the apt-sync drop is resolved

1.2.0 (2026-05-05)
------------------

1.1.0 (2026-04-20)
------------------
* feat(gorilla): Gorilla bit-packed XOR for FLOAT64 lossless (backward compatible) (`#93 <https://github.com/facontidavide/cloudini/issues/93>`_)
* Contributors: Davide Faconti

1.0.4 (2026-04-06)
------------------
* fix(ros): use ament_target_dependencies for pcl_conversions
  pcl_conversions 2.8.0 on Kilted switched from
  ament_export_include_directories to ament_export_targets, making
  ${pcl_conversions_INCLUDE_DIRS} empty and breaking the buildfarm.
  Use ament_target_dependencies which handles both old-style
  (INCLUDE_DIRS) and new-style (TARGETS) exports idiomatically.
  Co-Authored-By: Claude Opus 4.6 (1M context) <noreply@anthropic.com>
* Contributors: Davide Faconti

1.0.2 (2026-03-04)
------------------

1.0.1 (2026-03-01)
------------------

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
