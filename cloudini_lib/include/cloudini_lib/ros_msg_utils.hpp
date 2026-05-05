/*
 * Copyright 2025 Davide Faconti
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <map>
#include <utility>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/contrib/nanocdr.hpp"
namespace cloudini_ros {

/**
 * @brief This structure mimics the sensor_msgs/msg/PointCloud2 message fields
 *
 * @details:
 * ---
 * sensor_msgs/msg/PointCloud2
 *     std_msgs/msg/Header header
 *     uint32 height
 *     uint32 width
 *     PointField[] fields
 *     bool    is_bigendian
 *     uint32  point_step
 *     uint32  row_step
 *     uint8[] data
 *     bool is_dense
 * ---
 * sensor_msgs/msg/PointField:
 *     string name
 *     uint32 offset
 *     uint8  datatype
 *     uint32 count
 *
 */
struct RosHeader {
  int32_t stamp_sec = 0;    // seconds
  uint32_t stamp_nsec = 0;  // nanoseconds
  std::string frame_id;     // frame ID
};

// This structure mimics the sensor_msgs/msg/PointCloud2 message fields
struct RosPointCloud2 {
  RosPointCloud2() = default;

  RosPointCloud2(const RosPointCloud2& other)
      : cdr_header(other.cdr_header),
        ros_header(other.ros_header),
        height(other.height),
        width(other.width),
        fields(other.fields),
        point_step(other.point_step),
        row_step(other.row_step),
        is_bigendian(other.is_bigendian),
        data(other.data),
        is_dense(other.is_dense),
        owned_data(other.owned_data) {
    rebindOwnedDataView(other);
  }

  RosPointCloud2(RosPointCloud2&& other) noexcept {
    const bool owned_data_view = other.dataViewsOwnedData();
    cdr_header = std::move(other.cdr_header);
    ros_header = std::move(other.ros_header);
    height = other.height;
    width = other.width;
    fields = std::move(other.fields);
    point_step = other.point_step;
    row_step = other.row_step;
    is_bigendian = other.is_bigendian;
    data = other.data;
    is_dense = other.is_dense;
    owned_data = std::move(other.owned_data);
    if (owned_data_view) {
      data = Cloudini::ConstBufferView(owned_data.data(), owned_data.size());
    }
  }

  RosPointCloud2& operator=(const RosPointCloud2& other) {
    if (this == &other) {
      return *this;
    }
    cdr_header = other.cdr_header;
    ros_header = other.ros_header;
    height = other.height;
    width = other.width;
    fields = other.fields;
    point_step = other.point_step;
    row_step = other.row_step;
    is_bigendian = other.is_bigendian;
    data = other.data;
    is_dense = other.is_dense;
    owned_data = other.owned_data;
    rebindOwnedDataView(other);
    return *this;
  }

  RosPointCloud2& operator=(RosPointCloud2&& other) noexcept {
    if (this == &other) {
      return *this;
    }
    const bool owned_data_view = other.dataViewsOwnedData();
    cdr_header = std::move(other.cdr_header);
    ros_header = std::move(other.ros_header);
    height = other.height;
    width = other.width;
    fields = std::move(other.fields);
    point_step = other.point_step;
    row_step = other.row_step;
    is_bigendian = other.is_bigendian;
    data = other.data;
    is_dense = other.is_dense;
    owned_data = std::move(other.owned_data);
    if (owned_data_view) {
      data = Cloudini::ConstBufferView(owned_data.data(), owned_data.size());
    }
    return *this;
  }

  nanocdr::CdrHeader cdr_header;
  RosHeader ros_header;                      // ROS header
  uint32_t height = 1;                       // default to unorganized point cloud
  uint32_t width = 0;                        // number of points when height == 1
  std::vector<Cloudini::PointField> fields;  // point fields
  uint32_t point_step = 0;                   // size of a single point in bytes
  uint32_t row_step = 0;                     // size of a single row in bytes (not used)
  bool is_bigendian = false;                 // endianness (not used)
  Cloudini::ConstBufferView data;
  bool is_dense = true;  // whether all points are valid

  // Optional owned buffer used by lossy preprocessing helpers (e.g.
  // applyVizLossyPreprocessing) to hold a rewritten point payload. When
  // non-empty, `data` is expected to be a view over `owned_data`. Default
  // empty: `data` views the original DDS buffer.
  std::vector<uint8_t> owned_data;

 private:
  bool dataViewsOwnedData() const {
    return !owned_data.empty() && data.data() == owned_data.data() && data.size() == owned_data.size();
  }

  void rebindOwnedDataView(const RosPointCloud2& source) {
    if (source.dataViewsOwnedData()) {
      data = Cloudini::ConstBufferView(owned_data.data(), owned_data.size());
    }
  }
};

// Resolutions to be applied to the fields in RosPointCloud2 or EncodingInfo.
// Note that a resolution is 0, the entire field will be removed
using ResolutionProfile = std::map<std::string, float>;

/**
 * @brief Apply a resolution profile to the fields in the point cloud.
 * If a field has a resolution of 0, it will be removed.
 *
 * @param profile The resolution profile to apply.
 * @param field The fields to apply the profile to.
 * @param default_resolution Optional default resolution to apply to FLOAT32 fields not in the profile.
 */
void applyResolutionProfile(
    const ResolutionProfile& profile, std::vector<Cloudini::PointField>& field,
    std::optional<float> default_resolution = std::nullopt);

Cloudini::EncodingInfo toEncodingInfo(const RosPointCloud2& pc_info);

//------------------------------------------------------------------------

void writePointCloudHeader(nanocdr::Encoder& encoder, const RosPointCloud2& pc_info);

// Extract information from a raw DDS message (sensor_msgs/msg/PointCloud2) or
// (point_cloud_interfaces/msg/CompressedPointCloud2) into a RosPointCloud2 structure
RosPointCloud2 getDeserializedPointCloudMessage(Cloudini::ConstBufferView pc2_dds_msg);

// Given as input a raw DDS message, containing a sensor_msgs/msg/PointCloud2,
// apply compression and write the result into a point_cloud_interfaces/msg/CompressedPointCloud2
void convertPointCloud2ToCompressedCloud(
    const RosPointCloud2& pc_info, const Cloudini::EncodingInfo& encoding_info,
    std::vector<uint8_t>& compressed_dds_msg);

// Assumining that pc_info contains compressed data, decompress it directly into raw_dds_msg
void convertCompressedCloudToPointCloud2(const RosPointCloud2& pc_info, std::vector<uint8_t>& pc2_dds_msg);

/**
 * @brief Visualization-oriented lossy preprocessing applied in place.
 *
 * Bundles three lossy operations that approximately halve output size on
 * real LIDAR after stage-2 ZSTD, while preserving every declared field:
 *   1. NaN drop. Points whose geometry triple (xyz) contains NaN/inf are
 *      removed.
 *   2. Voxel dedup (1mm resolution by default — uses xyz fields' resolution).
 *      Hash-based, order-preserving: first occurrence of each voxel wins.
 *   3. 1µs FLOAT64 quantization. Every FLOAT64 field whose `resolution` is
 *      unset gets `resolution = 1e-6` so the encoder routes it through
 *      FieldEncoderFloat_Lossy (quantize+varint) instead of Gorilla. Affects
 *      typically per-point `timestamp` / `time` fields stored as
 *      seconds-since-epoch FLOAT64.
 *
 * Modifies pc_info in place: rewrites pc_info.owned_data with the cleaned
 * point bytes, points pc_info.data at it, updates pc_info.width to the new
 * point count, and sets resolution=1µs on FLOAT64 fields without one.
 *
 * No-op if pc_info has no geometry triple. The triple is detected
 * structurally (first 3 FLOAT32 fields with shared resolution and consecutive
 * offsets {b, b+4, b+8}); the field names are never read.
 */
void applyVizLossyPreprocessing(RosPointCloud2& pc_info);

}  // namespace cloudini_ros
