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

// The WebAssembly exports, built natively (test/wasm_shim stands in for emscripten.h): pointers are uintptr_t either
// way.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/wasm_functions.h"

namespace {

// A bare Cloudini blob (what CompressedPointCloud2::compressed_data or a protobuf bytes field holds): 100 points of
// x, y, z FLOAT32 at 1 mm, organized as 25 x 4.
std::vector<uint8_t> encodeBlob() {
  Cloudini::EncodingInfo info;
  for (const char* name : {"x", "y", "z"}) {
    Cloudini::PointField field;
    field.name = name;
    field.offset = static_cast<uint32_t>(info.fields.size() * sizeof(float));
    field.type = Cloudini::FieldType::FLOAT32;
    field.resolution = 0.001F;
    info.fields.push_back(field);
  }
  info.width = 25;
  info.height = 4;
  info.point_step = 3 * sizeof(float);
  std::vector<float> points(100 * 3);
  for (size_t i = 0; i < points.size(); ++i) {
    points[i] = 0.01F * static_cast<float>(i);
  }
  std::vector<uint8_t> blob;
  Cloudini::PointcloudEncoder(info).encode(
      Cloudini::ConstBufferView(points.data(), points.size() * sizeof(float)), blob);
  return blob;
}

uintptr_t address(const void* data) {
  return reinterpret_cast<uintptr_t>(data);
}

}  // namespace

TEST(WasmFunctions, GetDataInfoReadsTheHeaderOfABareBlob) {
  const auto blob{encodeBlob()};
  std::array<uint32_t, 4> info{};
  ASSERT_EQ(cldn_GetDataInfo(address(blob.data()), static_cast<uint32_t>(blob.size()), address(info.data())), 1u);
  EXPECT_EQ(info[0], 25u);  // width
  EXPECT_EQ(info[1], 4u);   // height
  EXPECT_EQ(info[2], 12u);  // point_step
  EXPECT_EQ(info[3], 3u);   // number of fields
}

TEST(WasmFunctions, GetDecodedDataSizeIsTheBufferDecodeCompressedDataFills) {
  const auto blob{encodeBlob()};
  const uint32_t size{cldn_GetDecodedDataSize(address(blob.data()), static_cast<uint32_t>(blob.size()))};
  EXPECT_EQ(size, 100u * 12u);
  std::vector<uint8_t> decoded(size);
  EXPECT_EQ(
      cldn_DecodeCompressedData(address(blob.data()), static_cast<uint32_t>(blob.size()), address(decoded.data())),
      size);
}

TEST(WasmFunctions, AHeaderThatDoesNotParseGivesZero) {
  const auto blob{encodeBlob()};
  std::array<uint32_t, 4> info{7, 7, 7, 7};
  EXPECT_EQ(cldn_GetDataInfo(address(blob.data()), 3, address(info.data())), 0u);
  EXPECT_EQ(info, (std::array<uint32_t, 4>{7, 7, 7, 7}));  // left untouched
  EXPECT_EQ(cldn_GetDecodedDataSize(address(blob.data()), 3), 0u);
}
