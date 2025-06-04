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

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cloudini_lib/cloudini.hpp>

namespace Cloudini {

EncodingInfo ConvertToEncodingInfo(const pcl::PCLPointCloud2& cloud, double resolution_XYZ);

template <typename PointT>
EncodingInfo ConvertToEncodingInfo(const pcl::PointCloud<PointT>& cloud, double resolution_XYZ);

//-------------------------------------------------------------------
// IMPORTANT: currently we specialized only these point types.
// If you need to add more specializations, look at the implmentation to see how it is done.
template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZ>(  //
    const pcl::PointCloud<pcl::PointXYZ>& cloud, double resolution_XYZ);

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZI>(  //
    const pcl::PointCloud<pcl::PointXYZI>& cloud, double resolution_XYZ);

//-------------------------------------------------------------------

/**
 * @brief Encodes a point cloud into a serialized format.
 *
 * @param cloud The input point cloud to encode.
 * @param serialized_cloud The output vector where the serialized data will be stored.
 * @param resolution_XYZ The resolution for the XYZ coordinates, used for lossy encoding.
 * @return The size of the serialized data in bytes.
 */
size_t PointcloudEncode(
    const pcl::PCLPointCloud2& cloud, std::vector<uint8_t>& serialized_cloud, double resolution_XYZ);

/**
 * @brief Encodes a point cloud into a serialized format.
 * If you use a point type different that PointXYZ or PointXYZI, you need to specialize the
 * ConvertToEncodingInfo function for that point type.
 *
 * @param cloud The input point cloud to encode.
 * @param serialized_cloud The output vector where the serialized data will be stored.
 * @param resolution_XYZ The resolution for the XYZ coordinates, used for lossy encoding.
 * @return The size of the serialized data in bytes.
 */
template <typename PointT>
size_t PointcloudEncode(
    const pcl::PointCloud<PointT>& cloud, std::vector<uint8_t>& serialized_cloud, double resolution_XYZ);

/**
 * @brief Decodes a serialized point cloud into a pcl::PCLPointCloud2 object.
 *
 * @param serialized_data The input serialized data to decode.
 * @param cloud The output pcl::PCLPointCloud2 object where the decoded data will be stored.
 */
void PointcloudDecode(ConstBufferView serialized_data, pcl::PCLPointCloud2& cloud);

/**
 * @brief Decodes a serialized point cloud into a pcl::PointCloud<PointT> object.
 * If you use a point type different that PointXYZ or PointXYZI, you need to specialize the
 * ConvertToEncodingInfo function for that point type.
 *
 * @param serialized_data The input serialized data to decode.
 * @param cloud The output pcl::PointCloud<PointT> object where the decoded data will be stored.
 */
template <typename PointT>
void PointcloudDecode(ConstBufferView serialized_data, pcl::PointCloud<PointT>& cloud);

//-------------------------------------------------------------------
//-------------------------------------------------------------------
// Implementations of the templated functions
//-------------------------------------------------------------------
//-------------------------------------------------------------------
bool isSameEncodingInfo(const EncodingInfo& info1, const EncodingInfo& info2);

template <typename PointT>
inline size_t PointcloudEncode(
    const pcl::PointCloud<PointT>& cloud, std::vector<uint8_t>& serialized_cloud, double resolution_XYZ) {
  // get the encoding info
  EncodingInfo info = ConvertToEncodingInfo(cloud, resolution_XYZ);
  // size in bytes of the data
  serialized_cloud.resize(ComputeHeaderSize(info.fields) + cloud.points.size() * sizeof(PointT));

  ConstBufferView data_view(reinterpret_cast<const uint8_t*>(cloud.points.data()), cloud.points.size());
  PointcloudEncoder encoder(info);
  return encoder.encode(data_view, serialized_cloud);
}

template <typename PointT>
inline void PointcloudDecode(ConstBufferView serialized_data, pcl::PointCloud<PointT>& cloud) {
  // decode the header
  EncodingInfo header_info = DecodeHeader(serialized_data);
  // resize the cloud to the decoded size
  cloud.points.resize(header_info.width * header_info.height);
  cloud.width = header_info.width;
  cloud.height = header_info.height;

  const EncodingInfo cloud_info = ConvertToEncodingInfo<PointT>(cloud, 1.0);  // 1.0 is a placeholder resolution

  // adapt the offset in the fields of "header_info" to match those in  cloud_info
  for (size_t i = 0; i < header_info.fields.size(); ++i) {
    auto& field = header_info.fields[i];
    // find the field with the same name
    auto it = std::find_if(cloud_info.fields.begin(), cloud_info.fields.end(), [&field](const PointField& f) {
      return f.name == field.name;
    });
    // if found, copy, otherwise, set to kDecodeButSkipStore
    if (it != cloud_info.fields.end()) {
      field.offset = it->offset;
      if (field.type != it->type) {
        throw std::runtime_error(
            "Field type mismatch for field: " + field.name + ". Expected: " +
            std::to_string(static_cast<int>(it->type)) + ", got: " + std::to_string(static_cast<int>(field.type)));
      }
    } else {
      // if the field is not found, we can skip it
      field.offset = Cloudini::kDecodeButSkipStore;
    }
  }

  // decode the data
  BufferView output_view(reinterpret_cast<uint8_t*>(cloud.points.data()), cloud.points.size());
  PointcloudDecoder decoder;
  decoder.decode(header_info, serialized_data, output_view);
}

};  // namespace Cloudini
