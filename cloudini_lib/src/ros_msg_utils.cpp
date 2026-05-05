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

#include "cloudini_lib/ros_msg_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "cloudini_lib/contrib/ankerl/unordered_dense.h"

namespace cloudini_ros {

namespace {

// Pack a quantized voxel coordinate into a 63-bit uint64 key.
//
// Layout: bits[0..20] = qx + bias, bits[21..41] = qy + bias, bits[42..62] = qz + bias.
// Bias = 2^20 makes nonneg, so each axis fits in 21 bits and represents
// signed values in [-2^20, 2^20-1] ≈ [-1.04M, 1.04M] ticks. At 1mm resolution
// that's ±1km per axis; at 1cm it's ±10km. Any realistic LIDAR fits.
//
// Out-of-range coords silently truncate (would cause false dedup positives).
// For our DATA corpus (200m max range × 1mm) this is well within the safe
// region; if encountered, bumping resolution to 1cm gives 10× headroom.
inline uint64_t packVoxelKey21(int32_t qx, int32_t qy, int32_t qz) {
  constexpr int64_t kBias = int64_t{1} << 20;
  constexpr uint64_t kAxisMask = (uint64_t{1} << 21) - 1;
  const uint64_t ux = static_cast<uint64_t>(static_cast<int64_t>(qx) + kBias) & kAxisMask;
  const uint64_t uy = static_cast<uint64_t>(static_cast<int64_t>(qy) + kBias) & kAxisMask;
  const uint64_t uz = static_cast<uint64_t>(static_cast<int64_t>(qz) + kBias) & kAxisMask;
  return ux | (uy << 21) | (uz << 42);
}

}  // namespace


void readPointCloud2MessageCommon(nanocdr::Decoder& cdr, RosPointCloud2& result) {
  //----- read the header -----
  result.cdr_header = cdr.header();
  cdr.decode(result.ros_header.stamp_sec);
  cdr.decode(result.ros_header.stamp_nsec);
  cdr.decode(result.ros_header.frame_id);

  //----- pointcloud info -----
  cdr.decode(result.height);
  cdr.decode(result.width);

  //----- fields -----
  uint32_t num_fields = 0;
  cdr.decode(num_fields);

  for (uint32_t i = 0; i < num_fields; ++i) {
    Cloudini::PointField field;
    cdr.decode(field.name);
    cdr.decode(field.offset);
    uint8_t type = 0;
    cdr.decode(type);
    field.type = static_cast<Cloudini::FieldType>(type);

    uint32_t count = 0;  // not used
    cdr.decode(count);

    result.fields.push_back(std::move(field));
  }

  bool is_bigendian = false;  // not used
  cdr.decode(is_bigendian);

  cdr.decode(result.point_step);
  cdr.decode(result.row_step);
  cdr.decode(result.data);
  cdr.decode(result.is_dense);
}

RosPointCloud2 getDeserializedPointCloudMessage(Cloudini::ConstBufferView raw_dds_msg) {
  RosPointCloud2 result;
  nanocdr::Decoder cdr(raw_dds_msg);
  readPointCloud2MessageCommon(cdr, result);
  return result;
}

void writePointCloudHeader(nanocdr::Encoder& encoder, const RosPointCloud2& pc_info) {
  //----- write the header -----
  encoder.encode(pc_info.ros_header.stamp_sec);   // header_stamp_sec
  encoder.encode(pc_info.ros_header.stamp_nsec);  // header_stamp_nsec
  encoder.encode(pc_info.ros_header.frame_id);    // frame_id

  //----- pointcloud info -----
  encoder.encode(pc_info.height);
  encoder.encode(pc_info.width);

  //----- fields -----
  encoder.encode(static_cast<uint32_t>(pc_info.fields.size()));
  for (const auto& field : pc_info.fields) {
    encoder.encode(field.name);
    encoder.encode(field.offset);
    encoder.encode(static_cast<uint8_t>(field.type));
    encoder.encode(static_cast<uint32_t>(1));  // count, not used
  }

  encoder.encode(false);  // is_bigendian, not used
  encoder.encode(pc_info.point_step);
  encoder.encode(static_cast<uint32_t>(pc_info.point_step * pc_info.width));
}

Cloudini::EncodingInfo toEncodingInfo(const RosPointCloud2& pc_info) {
  Cloudini::EncodingInfo info;
  info.height = pc_info.height;
  info.width = pc_info.width;
  info.point_step = pc_info.point_step;
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;      // default to lossy encoding
  info.compression_opt = Cloudini::CompressionOption::ZSTD;  // default to ZSTD compression
  info.fields = pc_info.fields;
  return info;
}
//-------------------------------------------------------------------

void convertCompressedCloudToPointCloud2(const RosPointCloud2& pc_info, std::vector<uint8_t>& pc2_dds_msg) {
  // We need to reserve enough space in the output buffer
  const size_t cloud_data_size = pc_info.width * pc_info.height * pc_info.point_step;

  pc2_dds_msg.clear();
  nanocdr::Encoder cdr_encoder(pc_info.cdr_header, pc2_dds_msg);
  writePointCloudHeader(cdr_encoder, pc_info);

  // Start writing the field PointCloud2::data
  cdr_encoder.encode(static_cast<uint32_t>(cloud_data_size));

  // special case: empty cloud
  if (cloud_data_size == 0) {
    cdr_encoder.encode(pc_info.is_dense);
    return;
  }

  // Reserve enough memory for the decoded cloud and take a buffer view to that memory
  const auto prev_size = pc2_dds_msg.size();
  pc2_dds_msg.resize(prev_size + cloud_data_size);
  // this buffer view points to a writable memory area after the PointCloud2Header
  Cloudini::BufferView decoded_buffer(pc2_dds_msg.data() + prev_size, cloud_data_size);

  Cloudini::PointcloudDecoder cloud_decoder;
  Cloudini::ConstBufferView compressed_data = pc_info.data;
  auto restored_header = Cloudini::DecodeHeader(compressed_data);
  cloud_decoder.decode(restored_header, compressed_data, decoded_buffer);

  // add last field
  cdr_encoder.encode(pc_info.is_dense);
}

void convertPointCloud2ToCompressedCloud(
    const RosPointCloud2& pc_info, const Cloudini::EncodingInfo& encoding_info,
    std::vector<uint8_t>& compressed_dds_msg) {
  compressed_dds_msg.clear();
  nanocdr::Encoder cdr_encoder(pc_info.cdr_header, compressed_dds_msg);
  writePointCloudHeader(cdr_encoder, pc_info);

  // we will write into this area later
  uint64_t cloud_size_memory_offset = compressed_dds_msg.size();

  // writing size 0 as a placeholder for now
  cdr_encoder.encode(static_cast<uint32_t>(0));
  const size_t prev_size = compressed_dds_msg.size();

  // special case that can happen: empty point cloud
  if (pc_info.data.size() == 0) {
    // nothing to compress
    cdr_encoder.encode(pc_info.is_dense);
    cdr_encoder.encode(std::string("cloudini"));  // format
    return;
  }

  if (encoding_info.point_step == 0) {
    throw std::runtime_error("convertPointCloud2ToCompressedCloud: point_step cannot be 0");
  }
  // Derive point count from actual data size rather than trusting metadata width*height,
  // which could be maliciously large and cause excessive allocation.
  const size_t points_count = pc_info.data.size() / encoding_info.point_step;
  const size_t max_compressed_size = Cloudini::MaxCompressedSize(encoding_info, points_count, true);
  // reserve enough memory for the compressed data. we will resize later to the actual size used
  compressed_dds_msg.resize(prev_size + max_compressed_size);

  Cloudini::BufferView compressed_data_view(
      compressed_dds_msg.data() + prev_size, compressed_dds_msg.size() - prev_size);
  Cloudini::PointcloudEncoder cloud_encoder(encoding_info);
  const size_t compressed_size = cloud_encoder.encode(pc_info.data, compressed_data_view, true);

  // we can finally write the actual size of the compressed data
  memcpy(compressed_dds_msg.data() + cloud_size_memory_offset, &compressed_size, sizeof(uint32_t));

  // Resize the output buffer to the actual size used
  compressed_dds_msg.resize(prev_size + compressed_size);

  // Add the remaining fields
  cdr_encoder.encode(pc_info.is_dense);
  cdr_encoder.encode(std::string("cloudini"));  // format
}

//-------------------------------------------------------------------

void applyResolutionProfile(
    const ResolutionProfile& profile, std::vector<Cloudini::PointField>& fields,
    std::optional<float> default_resolution) {
  // erase-remove idiom to remove fields with profile resolution of 0
  fields.erase(
      std::remove_if(
          fields.begin(), fields.end(),
          [&profile](const auto& field) {
            auto profile_it = profile.find(field.name);
            return (profile_it != profile.end() && profile_it->second == 0);
          }),
      fields.end());

  for (auto& field : fields) {
    auto profile_it = profile.find(field.name);
    if (profile_it != profile.end()) {
      field.resolution = profile_it->second;
    } else if (default_resolution && field.type == Cloudini::FieldType::FLOAT32) {
      field.resolution = *default_resolution;
    }
  }
}

// ---------------------------------------------------------------------------
// applyVizLossyPreprocessing — visualization-oriented lossy preprocessing.
//
// Detects the geometry triple structurally (first 3 FLOAT32 fields with
// shared resolution and consecutive offsets {b, b+4, b+8}); never reads field
// names. Drops NaN points, hash-deduplicates voxels at xyz resolution
// (order-preserving — first occurrence wins), and quantizes FLOAT64 fields
// without a resolution to 1µs. See header for full contract.
// ---------------------------------------------------------------------------
void applyVizLossyPreprocessing(RosPointCloud2& pc_info) {
  if (pc_info.fields.size() < 3 || pc_info.point_step == 0) {
    return;
  }

  const auto& f0 = pc_info.fields[0];
  const auto& f1 = pc_info.fields[1];
  const auto& f2 = pc_info.fields[2];
  const bool has_triple =
      f0.type == Cloudini::FieldType::FLOAT32 && f1.type == Cloudini::FieldType::FLOAT32 &&
      f2.type == Cloudini::FieldType::FLOAT32 && f0.resolution.has_value() &&
      f1.resolution.has_value() && f2.resolution.has_value() &&
      f0.resolution.value() == f1.resolution.value() &&
      f0.resolution.value() == f2.resolution.value() &&
      f1.offset == f0.offset + 4u && f2.offset == f0.offset + 8u;
  if (!has_triple) {
    return;
  }

  const float xyz_res = f0.resolution.value();
  if (!(xyz_res > 0.0f) || !std::isfinite(xyz_res)) {
    return;
  }
  const float inv_res = 1.0f / xyz_res;
  const uint32_t xyz_off[3] = {f0.offset, f1.offset, f2.offset};

  const size_t n_in = (pc_info.data.size() == 0) ? 0 : (pc_info.data.size() / pc_info.point_step);
  if (n_in == 0) {
    return;
  }

  // Pack the quantized voxel coordinate into a 63-bit uint64 (see
  // packVoxelKey21 above) and dedup with ankerl::unordered_dense::set —
  // an open-addressing flat hash set with single-allocation backing storage.
  // Combined with the packed key (identity-mixed; no std::hash + Boost
  // combine cost) this is empirically ~3× faster than the previous
  // std::unordered_set<VoxelKey> approach.
  //
  // Identity hash works because the packed key already spreads coordinate
  // bits across all 63 bits; ankerl applies its own internal mix
  // (mumxor-finisher) on top to handle pathological clustering.
  // Default ankerl hash on uint64 (wyhash); the packed key has axis-major
  // bit layout so lower bits cluster per-scan — letting ankerl's mix do its
  // job avoids pathological probe-chain blowups on real LIDAR data.
  // (An identity hash with `is_avalanching = void` was a 250× regression on
  // bount/dexory/etc. — the lower bits of the packed key collide into the
  // same buckets when ankerl skips its mix.)
  thread_local ankerl::unordered_dense::set<uint64_t> seen;
  seen.clear();
  seen.reserve(n_in);

  std::vector<uint8_t> out;
  out.reserve(pc_info.data.size());

  // Single pass: NaN-drop + quantize xyz + hash-dedup + copy survivor bytes.
  uint64_t kept = 0;
  for (size_t i = 0; i < n_in; ++i) {
    const uint8_t* p = pc_info.data.data() + i * pc_info.point_step;
    float fx = 0.0f, fy = 0.0f, fz = 0.0f;
    std::memcpy(&fx, p + xyz_off[0], 4);
    std::memcpy(&fy, p + xyz_off[1], 4);
    std::memcpy(&fz, p + xyz_off[2], 4);
    if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) {
      continue;  // drop NaN/inf
    }
    const uint64_t key = packVoxelKey21(
        static_cast<int32_t>(std::lround(fx * inv_res)),
        static_cast<int32_t>(std::lround(fy * inv_res)),
        static_cast<int32_t>(std::lround(fz * inv_res)));
    if (!seen.insert(key).second) {
      continue;  // voxel duplicate
    }
    const size_t before = out.size();
    out.resize(before + pc_info.point_step);
    std::memcpy(out.data() + before, p, pc_info.point_step);
    ++kept;
  }

  // Replace pc_info's data view with the new owned buffer.
  pc_info.owned_data = std::move(out);
  pc_info.data =
      Cloudini::ConstBufferView(pc_info.owned_data.data(), pc_info.owned_data.size());
  pc_info.width = static_cast<uint32_t>(kept);
  pc_info.height = 1;
  pc_info.row_step = pc_info.point_step * pc_info.width;

  // Quantize FLOAT64 fields without a resolution to 1µs.
  for (auto& f : pc_info.fields) {
    if (f.type == Cloudini::FieldType::FLOAT64 && !f.resolution.has_value()) {
      f.resolution = 1e-6f;
    }
  }
}

}  // namespace cloudini_ros
