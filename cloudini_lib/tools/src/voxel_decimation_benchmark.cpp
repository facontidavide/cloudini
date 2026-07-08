/*
 * Copyright 2025 Davide Faconti
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// voxel_decimation_benchmark
//
// Measure hash-based vs sort-based voxel decimation on real PointCloud2 data.
//
// Motivation: Cloudini's --viz path deduplicates points into a voxel grid with
// an open-addressing hash set (ankerl::unordered_dense) keyed by packVoxelKey21,
// keeping the first occurrence (order-preserving, single pass). PCL's VoxelGrid
// instead builds an (index, point) vector, radix-sorts it, then scans runs and
// averages each voxel to a centroid. This tool times both strategies on the SAME
// input at the SAME grid so the comparison is apples-to-apples.
//
// To keep the comparison honest for Cloudini's regime, ALL sort variants sort
// the identical 63-bit packVoxelKey21 that the hash uses. PCL's dense row-major
// index (div_b_ / divb_mul_) can't be used at 1mm over LIDAR extents without
// int32 overflow; --pcl-index-check demonstrates that on the real data.
//
// Variants (all at one decimation resolution, independent of storage precision):
//   hash          ankerl set, first-occurrence, order-preserving  (Cloudini today)
//   sort_std      std::sort of (key,idx), scan runs, first-of-voxel
//   sort_radix    LSD byte-radix sort of the key, scan runs        (spreadsort proxy)
//   sort_centroid sort_radix + average XYZ per voxel               (PCL default)
//
// Correctness gate: every variant must select the SAME set of voxels. We verify
// by folding (XOR + count) the surviving voxel keys; all variants must match.
//
// Timing: each variant runs over every preloaded frame, repeated --repeat times;
// we report the BEST (min) wall time to reject scheduler noise, as Mpoints/s.
//
// Preloads frames into RAM so timing excludes MCAP I/O. Use --max-messages to
// bound memory on large bags.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "cloudini_lib/contrib/ankerl/unordered_dense.h"
#include "cloudini_lib/ros_message_definitions.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"
#include "cxxopts.hpp"

#define MCAP_IMPLEMENTATION
#include "mcap/reader.hpp"

namespace {

using Clock = std::chrono::steady_clock;

// Verbatim copy of cloudini_ros::packVoxelKey21 (that one lives in an anonymous
// namespace). Axis-major layout: bits[0..20]=qx, [21..41]=qy, [42..62]=qz, each
// biased by 2^20 so signed coords fit in 21 bits (±1.04M ticks per axis).
inline uint64_t packVoxelKey21(int32_t qx, int32_t qy, int32_t qz) {
  constexpr int64_t kBias = int64_t{1} << 20;
  constexpr uint64_t kAxisMask = (uint64_t{1} << 21) - 1;
  const uint64_t ux = static_cast<uint64_t>(static_cast<int64_t>(qx) + kBias) & kAxisMask;
  const uint64_t uy = static_cast<uint64_t>(static_cast<int64_t>(qy) + kBias) & kAxisMask;
  const uint64_t uz = static_cast<uint64_t>(static_cast<int64_t>(qz) + kBias) & kAxisMask;
  return ux | (uy << 21) | (uz << 42);
}

// One preloaded point cloud: raw interleaved point bytes + XYZ field offsets.
struct Frame {
  std::vector<uint8_t> data;
  uint32_t point_step = 0;
  uint32_t xyz_off[3] = {0, 0, 0};
  size_t points() const { return point_step ? data.size() / point_step : 0; }
};

// Order-independent fingerprint of a set of surviving voxels: XOR of the keys
// plus the count. Two variants that select the same voxels produce the same
// fold regardless of emission order or representative-point choice.
struct KeyFold {
  uint64_t xored = 0;
  uint64_t count = 0;
  bool operator==(const KeyFold& o) const { return xored == o.xored && count == o.count; }
};

inline void readXYZ(const uint8_t* p, const uint32_t off[3], float& x, float& y, float& z) {
  std::memcpy(&x, p + off[0], 4);
  std::memcpy(&y, p + off[1], 4);
  std::memcpy(&z, p + off[2], 4);
}

// ---- Variant 1: hash (Cloudini today) --------------------------------------
// Single pass: NaN-drop + quantize + hash-insert + emit survivor. First
// occurrence wins, output preserves scan order.
KeyFold decimateHash(const Frame& fr, float inv_res, std::vector<uint8_t>& out) {
  thread_local ankerl::unordered_dense::set<uint64_t> seen;
  seen.clear();
  const size_t n = fr.points();
  seen.reserve(n);
  out.clear();
  out.reserve(fr.data.size());
  KeyFold fold;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* p = fr.data.data() + i * fr.point_step;
    float x, y, z;
    readXYZ(p, fr.xyz_off, x, y, z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    const uint64_t key = packVoxelKey21(
        static_cast<int32_t>(std::lround(x * inv_res)), static_cast<int32_t>(std::lround(y * inv_res)),
        static_cast<int32_t>(std::lround(z * inv_res)));
    if (!seen.insert(key).second) continue;
    const size_t before = out.size();
    out.resize(before + fr.point_step);
    std::memcpy(out.data() + before, p, fr.point_step);
    fold.xored ^= key;
    ++fold.count;
  }
  return fold;
}

struct KeyIdx {
  uint64_t key;
  uint32_t idx;
};

// Build the (key, idx) vector once, dropping NaN. Shared by the sort variants.
void buildKeyIdx(const Frame& fr, float inv_res, std::vector<KeyIdx>& kv) {
  const size_t n = fr.points();
  kv.clear();
  kv.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* p = fr.data.data() + i * fr.point_step;
    float x, y, z;
    readXYZ(p, fr.xyz_off, x, y, z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    kv.push_back({packVoxelKey21(
                      static_cast<int32_t>(std::lround(x * inv_res)),
                      static_cast<int32_t>(std::lround(y * inv_res)),
                      static_cast<int32_t>(std::lround(z * inv_res))),
                  static_cast<uint32_t>(i)});
  }
}

// Scan a key-sorted vector, emit one survivor per run (min idx = scan-order
// first, matching the hash's representative so outputs are byte-identical).
KeyFold emitRuns(const Frame& fr, const std::vector<KeyIdx>& kv, std::vector<uint8_t>& out) {
  out.clear();
  out.reserve(fr.data.size());
  KeyFold fold;
  size_t i = 0;
  const size_t m = kv.size();
  while (i < m) {
    const uint64_t key = kv[i].key;
    uint32_t rep = kv[i].idx;
    size_t j = i + 1;
    while (j < m && kv[j].key == key) {
      rep = std::min(rep, kv[j].idx);
      ++j;
    }
    const uint8_t* p = fr.data.data() + static_cast<size_t>(rep) * fr.point_step;
    const size_t before = out.size();
    out.resize(before + fr.point_step);
    std::memcpy(out.data() + before, p, fr.point_step);
    fold.xored ^= key;
    ++fold.count;
    i = j;
  }
  return fold;
}

// ---- Variant 2: sort_std ---------------------------------------------------
KeyFold decimateSortStd(const Frame& fr, float inv_res, std::vector<KeyIdx>& kv, std::vector<uint8_t>& out) {
  buildKeyIdx(fr, inv_res, kv);
  std::sort(kv.begin(), kv.end(), [](const KeyIdx& a, const KeyIdx& b) { return a.key < b.key; });
  return emitRuns(fr, kv, out);
}

// LSD byte-radix sort on the 63-bit key (8 passes, skipping passes whose byte is
// constant across all elements). This is the spirit of boost::spreadsort's
// integer_sort — the sort PCL's VoxelGrid uses.
void radixSort(std::vector<KeyIdx>& kv, std::vector<KeyIdx>& tmp) {
  const size_t n = kv.size();
  if (n < 2) return;
  tmp.resize(n);
  KeyIdx* src = kv.data();
  KeyIdx* dst = tmp.data();
  for (int byte = 0; byte < 8; ++byte) {
    const int shift = byte * 8;
    size_t count[256] = {0};
    for (size_t i = 0; i < n; ++i) ++count[(src[i].key >> shift) & 0xFF];
    if (count[(src[0].key >> shift) & 0xFF] == n) continue;  // byte constant: skip pass
    size_t sum = 0;
    for (int b = 0; b < 256; ++b) {
      const size_t c = count[b];
      count[b] = sum;
      sum += c;
    }
    for (size_t i = 0; i < n; ++i) dst[count[(src[i].key >> shift) & 0xFF]++] = src[i];
    std::swap(src, dst);
  }
  if (src != kv.data()) std::memcpy(kv.data(), src, n * sizeof(KeyIdx));  // ensure result in kv
}

// ---- Variant 3: sort_radix -------------------------------------------------
KeyFold decimateSortRadix(
    const Frame& fr, float inv_res, std::vector<KeyIdx>& kv, std::vector<KeyIdx>& tmp, std::vector<uint8_t>& out) {
  buildKeyIdx(fr, inv_res, kv);
  radixSort(kv, tmp);
  return emitRuns(fr, kv, out);
}

// ---- Variant 4: sort_centroid (PCL default) --------------------------------
// radix sort + average XYZ across each voxel's members; other fields copied
// from the first member. Produces the same voxel SET (same fold) but synthetic
// XYZ positions.
KeyFold decimateSortCentroid(
    const Frame& fr, float inv_res, std::vector<KeyIdx>& kv, std::vector<KeyIdx>& tmp, std::vector<uint8_t>& out) {
  buildKeyIdx(fr, inv_res, kv);
  radixSort(kv, tmp);
  out.clear();
  out.reserve(fr.data.size());
  KeyFold fold;
  size_t i = 0;
  const size_t m = kv.size();
  while (i < m) {
    const uint64_t key = kv[i].key;
    uint32_t rep = kv[i].idx;
    double sx = 0, sy = 0, sz = 0;
    size_t j = i;
    for (; j < m && kv[j].key == key; ++j) {
      const uint8_t* p = fr.data.data() + static_cast<size_t>(kv[j].idx) * fr.point_step;
      float x, y, z;
      readXYZ(p, fr.xyz_off, x, y, z);
      sx += x;
      sy += y;
      sz += z;
      rep = std::min(rep, kv[j].idx);
    }
    const double inv_n = 1.0 / static_cast<double>(j - i);
    const uint8_t* pr = fr.data.data() + static_cast<size_t>(rep) * fr.point_step;
    const size_t before = out.size();
    out.resize(before + fr.point_step);
    std::memcpy(out.data() + before, pr, fr.point_step);  // copy all fields from first member
    const float cx = static_cast<float>(sx * inv_n);
    const float cy = static_cast<float>(sy * inv_n);
    const float cz = static_cast<float>(sz * inv_n);
    std::memcpy(out.data() + before + fr.xyz_off[0], &cx, 4);  // overwrite XYZ with centroid
    std::memcpy(out.data() + before + fr.xyz_off[1], &cy, 4);
    std::memcpy(out.data() + before + fr.xyz_off[2], &cz, 4);
    fold.xored ^= key;
    ++fold.count;
    i = j;
  }
  return fold;
}

// Count how many frames would overflow PCL's int32 dense voxel index at res.
// PCL computes dx*dy*dz over the cloud bbox and warns when it exceeds INT32_MAX.
void pclIndexCheck(const std::vector<Frame>& frames, float res) {
  const double inv = 1.0 / res;
  size_t overflow = 0;
  double worst = 0;
  for (const auto& fr : frames) {
    float mn[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                   std::numeric_limits<float>::max()};
    float mx[3] = {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                   -std::numeric_limits<float>::max()};
    const size_t n = fr.points();
    for (size_t i = 0; i < n; ++i) {
      const uint8_t* p = fr.data.data() + i * fr.point_step;
      float v[3];
      readXYZ(p, fr.xyz_off, v[0], v[1], v[2]);
      for (int a = 0; a < 3; ++a) {
        if (!std::isfinite(v[a])) continue;
        mn[a] = std::min(mn[a], v[a]);
        mx[a] = std::max(mx[a], v[a]);
      }
    }
    const int64_t dx = static_cast<int64_t>((mx[0] - mn[0]) * inv) + 1;
    const int64_t dy = static_cast<int64_t>((mx[1] - mn[1]) * inv) + 1;
    const int64_t dz = static_cast<int64_t>((mx[2] - mn[2]) * inv) + 1;
    const double prod = static_cast<double>(dx) * static_cast<double>(dy) * static_cast<double>(dz);
    worst = std::max(worst, prod);
    if (prod > static_cast<double>(std::numeric_limits<int32_t>::max())) ++overflow;
  }
  std::cout << "\nPCL dense-index int32 overflow check @ res=" << res << " m:\n";
  std::cout << "  frames overflowing INT32_MAX: " << overflow << " / " << frames.size() << "\n";
  std::cout << "  worst-case dx*dy*dz = " << std::scientific << std::setprecision(2) << worst << std::defaultfloat
            << "  (INT32_MAX = 2.15e9)\n";
  std::cout << "  -> where this overflows, PCL VoxelGrid either no-ops or aliases distant voxels.\n";
}

double msOf(uint64_t ns) { return static_cast<double>(ns) / 1e6; }

}  // namespace

int main(int argc, char** argv) {
  cxxopts::Options options(
      "voxel_decimation_benchmark", "Hash-based vs sort-based voxel decimation on real PointCloud2 data.");
  options.add_options()                                                                                    //
      ("h,help", "Print usage")                                                                            //
      ("f,filename", "Input MCAP file (positional also accepted)", cxxopts::value<std::string>())          //
      ("r,res", "Decimation voxel size in meters", cxxopts::value<float>()->default_value("0.05"))         //
      ("max-messages", "Preload at most N messages per topic (0 = all)",                                   //
       cxxopts::value<uint64_t>()->default_value("200"))                                                   //
      ("repeat", "Repeat the timed loop N times, keep the best", cxxopts::value<uint64_t>()->default_value("5"))  //
      ("variant", "Time only one variant: hash|sort_std|sort_radix|sort_centroid (for perf stat)",         //
       cxxopts::value<std::string>()->default_value(""))                                                   //
      ("no-gate", "Skip the correctness cross-check (for clean perf-stat counters)")                        //
      ("pcl-index-check", "Report PCL int32 dense-index overflow on this data and exit");
  options.parse_positional({"filename"});
  options.positional_help("<file.mcap>");

  cxxopts::ParseResult pr;
  try {
    pr = options.parse(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Argument error: " << e.what() << "\n";
    return 1;
  }
  if (pr.count("help") || !pr.count("filename")) {
    std::cout << options.help() << std::endl;
    return pr.count("help") ? 0 : 1;
  }

  const std::filesystem::path input_file = pr["filename"].as<std::string>();
  const float res = pr["res"].as<float>();
  const uint64_t max_per_topic = pr["max-messages"].as<uint64_t>();
  const uint64_t repeat = std::max<uint64_t>(1, pr["repeat"].as<uint64_t>());
  const std::string only = pr["variant"].as<std::string>();
  const bool no_gate = pr.count("no-gate") > 0;
  const bool pcl_check = pr.count("pcl-index-check") > 0;
  if (!(res > 0.0f)) {
    std::cerr << "Error: --res must be > 0\n";
    return 1;
  }
  if (!std::filesystem::exists(input_file)) {
    std::cerr << "Error: file does not exist: " << input_file << "\n";
    return 1;
  }

  // ----- open + preload -----
  std::ifstream input_stream(input_file);
  auto data_source = std::make_shared<mcap::FileStreamReader>(input_stream);
  mcap::McapReader reader;
  if (auto s = reader.open(*data_source); !s.ok()) {
    std::cerr << "Error opening MCAP: " << s.message << "\n";
    return 1;
  }
  if (auto s = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan); !s.ok()) {
    std::cerr << "Error reading summary: " << s.message << "\n";
    return 1;
  }
  std::map<mcap::ChannelId, std::string> pc_channels;
  for (const auto& [channel_id, channel_ptr] : reader.channels()) {
    const auto& schema_ptr = reader.schema(channel_ptr->schemaId);
    if (schema_ptr && schema_ptr->name == pointcloud_schema_name) pc_channels[channel_id] = channel_ptr->topic;
  }
  if (pc_channels.empty()) {
    std::cerr << "No sensor_msgs/msg/PointCloud2 topics found.\n";
    return 1;
  }

  std::vector<Frame> frames;
  std::map<std::string, uint64_t> seen;
  uint64_t total_points = 0;
  mcap::ProblemCallback problem = [](const mcap::Status&) {};
  mcap::ReadMessageOptions ropts;
  for (const auto& msg : reader.readMessages(problem, ropts)) {
    auto it = pc_channels.find(msg.channel->id);
    if (it == pc_channels.end()) continue;
    if (max_per_topic != 0 && ++seen[it->second] > max_per_topic) continue;

    Cloudini::ConstBufferView raw(msg.message.data, msg.message.dataSize);
    cloudini_ros::RosPointCloud2 pc;
    try {
      pc = cloudini_ros::getDeserializedPointCloudMessage(raw);
    } catch (const std::exception&) {
      continue;
    }
    // Require the geometry triple at fields[0..2] as consecutive FLOAT32 (same
    // structural assumption applyVizLossyPreprocessing makes).
    if (pc.fields.size() < 3 || pc.point_step == 0 || pc.data.size() == 0) continue;
    const auto& f0 = pc.fields[0];
    const auto& f1 = pc.fields[1];
    const auto& f2 = pc.fields[2];
    const bool triple = f0.type == Cloudini::FieldType::FLOAT32 && f1.type == Cloudini::FieldType::FLOAT32 &&
                        f2.type == Cloudini::FieldType::FLOAT32 && f1.offset == f0.offset + 4u &&
                        f2.offset == f0.offset + 8u;
    if (!triple) continue;

    Frame fr;
    fr.point_step = pc.point_step;
    fr.xyz_off[0] = f0.offset;
    fr.xyz_off[1] = f1.offset;
    fr.xyz_off[2] = f2.offset;
    fr.data.assign(pc.data.data(), pc.data.data() + pc.data.size());
    total_points += fr.points();
    frames.push_back(std::move(fr));
  }
  if (frames.empty()) {
    std::cerr << "No usable PointCloud2 frames (need FLOAT32 xyz triple at fields[0..2]).\n";
    return 1;
  }

  std::cout << "File: " << input_file.filename().string() << "   frames=" << frames.size()
            << "   points=" << total_points << "   decimation res=" << res << " m\n";

  if (pcl_check) {
    pclIndexCheck(frames, res);
    return 0;
  }

  const float inv_res = 1.0f / res;

  // Reusable scratch buffers (allocated once, outside the timed loop).
  std::vector<uint8_t> out;
  std::vector<KeyIdx> kv, tmp;

  // ----- correctness gate: all variants must select the same voxel set -----
  if (!no_gate) {
    KeyFold agg_hash, agg_std, agg_radix, agg_cent;
    for (const auto& fr : frames) {
      auto a = decimateHash(fr, inv_res, out);
      auto b = decimateSortStd(fr, inv_res, kv, out);
      auto c = decimateSortRadix(fr, inv_res, kv, tmp, out);
      auto d = decimateSortCentroid(fr, inv_res, kv, tmp, out);
      agg_hash.xored ^= a.xored; agg_hash.count += a.count;
      agg_std.xored ^= b.xored; agg_std.count += b.count;
      agg_radix.xored ^= c.xored; agg_radix.count += c.count;
      agg_cent.xored ^= d.xored; agg_cent.count += d.count;
    }
    const bool ok = agg_hash == agg_std && agg_hash == agg_radix && agg_hash == agg_cent;
    std::cout << "Correctness: survivors=" << agg_hash.count << "  (kept "
              << std::fixed << std::setprecision(1) << (100.0 * agg_hash.count / total_points) << "% of points)  "
              << (ok ? "[all variants agree on voxel set]" : "[MISMATCH]") << "\n";
    if (!ok) {
      std::cerr << "  hash  count=" << agg_hash.count << " xor=" << std::hex << agg_hash.xored << std::dec << "\n";
      std::cerr << "  std   count=" << agg_std.count << " xor=" << std::hex << agg_std.xored << std::dec << "\n";
      std::cerr << "  radix count=" << agg_radix.count << " xor=" << std::hex << agg_radix.xored << std::dec << "\n";
      return 2;
    }
  }

  // ----- timing -----
  struct VariantResult {
    const char* name;
    uint64_t best_ns = std::numeric_limits<uint64_t>::max();
  };
  auto timeVariant = [&](const char* name, auto&& fn) -> VariantResult {
    VariantResult vr;
    vr.name = name;
    for (uint64_t r = 0; r < repeat; ++r) {
      const auto t0 = Clock::now();
      for (const auto& fr : frames) fn(fr);
      const auto t1 = Clock::now();
      const uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
      vr.best_ns = std::min(vr.best_ns, ns);
    }
    return vr;
  };

  auto run_hash = [&](const Frame& fr) { decimateHash(fr, inv_res, out); };
  auto run_std = [&](const Frame& fr) { decimateSortStd(fr, inv_res, kv, out); };
  auto run_radix = [&](const Frame& fr) { decimateSortRadix(fr, inv_res, kv, tmp, out); };
  auto run_cent = [&](const Frame& fr) { decimateSortCentroid(fr, inv_res, kv, tmp, out); };

  std::vector<VariantResult> results;
  if (only.empty() || only == "hash") results.push_back(timeVariant("hash", run_hash));
  if (only.empty() || only == "sort_std") results.push_back(timeVariant("sort_std", run_std));
  if (only.empty() || only == "sort_radix") results.push_back(timeVariant("sort_radix", run_radix));
  if (only.empty() || only == "sort_centroid") results.push_back(timeVariant("sort_centroid", run_cent));
  if (results.empty()) {
    std::cerr << "Error: unknown --variant '" << only << "'\n";
    return 1;
  }

  std::cout << "  " << std::string(52, '-') << "\n";
  std::cout << "  " << std::left << std::setw(16) << "Variant" << std::right << std::setw(12) << "best ms"
            << std::setw(14) << "Mpoints/s" << std::setw(10) << "vs hash" << "\n";
  std::cout << "  " << std::string(52, '-') << "\n";
  double hash_mpps = 0;
  for (const auto& vr : results) {
    const double mpps = static_cast<double>(total_points) / (static_cast<double>(vr.best_ns) / 1e3);  // pts/us = Mpts/s
    if (std::string(vr.name) == "hash") hash_mpps = mpps;
  }
  for (const auto& vr : results) {
    const double mpps = static_cast<double>(total_points) / (static_cast<double>(vr.best_ns) / 1e3);
    std::ostringstream rel;
    if (hash_mpps > 0) rel << std::fixed << std::setprecision(2) << (mpps / hash_mpps) << "x";
    else rel << "-";
    std::cout << "  " << std::left << std::setw(16) << vr.name << std::right << std::fixed << std::setprecision(2)
              << std::setw(12) << msOf(vr.best_ns) << std::setw(14) << mpps << std::setw(10) << rel.str() << "\n";
  }
  return 0;
}
