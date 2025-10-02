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

// retrieves a JSON representation of Cloudini::EncodingInfo
size_t cldn_GetHeaderAsJSON(uintptr_t encoded_data_ptr, size_t encoded_data_size, uintptr_t output_json_ptr);

// Performs a full compression of the point cloud data, but return only the size of the
// compressed data, not the data itself. Used mainly for testing purposes.
WASM_EXPORT size_t cldn_ComputeCompressedSize(uintptr_t dds_msg_ptr, size_t dds_msg_size, float resolution);

// Preview the size of the decompressed point cloud data, needed to allocate memory in advance.
// No actual decompression is performed.
WASM_EXPORT size_t cldn_GetDecompressedSize(uintptr_t encoded_msg_ptr, size_t encoded_msg_size);

/**
 * @brief Given the a whole serialized DDS message containing "point_cloud_interfaces/CompressedPointCloud2",
 * decode the message and return the size of the decompressed point cloud data.
 * The deserialized data will likely go into the field "sensor_msgs::PointCloud2::data".
 *
 * @param encoded_dds_ptr pointer to the serialized DDS message.
 * @param encoded_dds_size size of the serialized DDS message.
 * @param output_data pointer for the output buffer where decompressed data will be written.
 * @return The size of the decompressed point cloud data, or 0 on failure.
 */
WASM_EXPORT size_t
cldn_DecodeCompressedMessage(uintptr_t encoded_dds_ptr, size_t encoded_dds_size, uintptr_t output_data);

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
WASM_EXPORT size_t
cldn_DecodeCompressedData(uintptr_t encoded_data_ptr, size_t encoded_data_size, uintptr_t output_data);
}
