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

#include <cstddef>
#include <cstdint>
#include <cstring>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define WASM_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define WASM_EXPORT
#endif

extern "C" {

// retrieves a YAML representation of Cloudini::EncodingInfo
uint32_t cldn_GetHeaderAsYAML(uintptr_t encoded_data_ptr, uint32_t encoded_data_size, uintptr_t output_yaml_ptr);

// Performs a full compression of the point cloud data, but return only the size of the
// compressed data, not the data itself. Used mainly for testing purposes.
WASM_EXPORT uint32_t cldn_ComputeCompressedSize(uintptr_t dds_msg_ptr, uint32_t dds_msg_size, float resolution);

// Preview the size of the decompressed point cloud data, needed to allocate memory in advance.
// No actual decompression is performed.
WASM_EXPORT uint32_t cldn_GetDecompressedSize(uintptr_t encoded_msg_ptr, uint32_t encoded_msg_size);

/**
 * @brief Given the a serialized DDS message containing "point_cloud_interfaces/CompressedPointCloud2",
 * perform decoding and write the result into a "sensor_msgs/PointCloud2" serialized message.
 *
 * @param encoded_dds_ptr pointer to the serialized DDS message.
 * @param encoded_dds_size size of the serialized DDS message.
 * @param output_data pointer for the output buffer where DDS message will be written.
 * @return The size of the decompressed point cloud data, or 0 on failure.
 */
WASM_EXPORT uint32_t cldn_ConvertCompressedMsgToPointCloud2Msg(
    uintptr_t compressed_msg_ptr, uint32_t encoded_data_size, uintptr_t output_msg_ptr);

/**
 * @brief Given compressed cloudini buffer, perform decoding.
 * The compressed data will come from the field "point_cloud_interfaces::CompressedPointCloud2::compressed_data",
 * while the deserialized data will likely go into the field "sensor_msgs::PointCloud2::data".
 *
 * @param encoded_data_ptr pointer to the serialized DDS message.
 * @param encoded_data_size size of the serialized DDS message.
 * @param output_data pointer for the output buffer where decompressed data will be written.
 * @return The size of the decompressed point cloud data, or 0 on failure.
 */
WASM_EXPORT uint32_t
cldn_DecodeCompressedData(uintptr_t encoded_data_ptr, uint32_t encoded_data_size, uintptr_t output_data);

// Same as cldn_DecodeCompressedData, but the input is a DDS message containing
// "point_cloud_interfaces/CompressedPointCloud2"
WASM_EXPORT uint32_t
cldn_DecodeCompressedMessage(uintptr_t compressed_msg_ptr, uint32_t msg_size, uintptr_t output_data_ptr);

/**
 * @brief Give a ROS2 DDS message containing "sensor_msgs/PointCloud2", perform compression
 * and serialize the result as a cloudini compressed point cloud.
 * The `output_data` is usually the "data" field of "point_cloud_interfaces::CompressedPointCloud2"
 * or can be saved directly to file.
 */
WASM_EXPORT uint32_t cldn_EncodePointcloudMessage(
    const uintptr_t pointcloud_msg_ptr, uint32_t msg_size, float resolution, uintptr_t output_data_ptr);

/**
 * @brief Given the raw point cloud data, perform compression
 * and serialize the result as a cloudini compressed point cloud.
 * The `output_data` is usually the "data" field of "point_cloud_interfaces::CompressedPointCloud2"
 * or can be saved directly to file.
 */
WASM_EXPORT uint32_t cldn_EncodePointcloudData(
    const char* header_as_yaml, const uintptr_t pc_data_ptr, uint32_t pc_data_size, uintptr_t output_data_ptr);
}
