/*
 * Copyright 2025 Davide Faconti
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// mcap_codec_benchmark
//
// Stream a single MCAP file, and per PointCloud2 topic measure compression
// ratio + encode/decode wall time for four variants:
//
//   * V4         — V4 lossy codec, every field preserved (lossless on integer
//                  fields, FieldEncoderFloat_Lossy on FLOAT32-with-resolution,
//                  FieldEncoderFloat_Gorilla on FLOAT64-no-resolution).
//   * V5         — V5 adaptive-integer codec. FLOAT fields use the V4 paths;
//                  integer fields can switch per chunk between V4 delta-varint,
//                  palette indexes, and RLE.
//   * V4/V5 + viz — same codec, with `applyVizLossyPreprocessing` run per
//                  message before encoding: drop NaN points, voxel-dedupe at
//                  the xyz resolution, quantize FLOAT64 fields to 1µs.
//
// For each variant we report bytes out of the codec, ratio vs raw input,
// encode MB/s, and decode MB/s. A `--zstd` flag runs the same variants through
// Cloudini's built-in ZSTD chunk compression path.
//
// Bypasses the DDS layer and calls PointcloudEncoder/Decoder directly on the
// raw point payload, so the timings reflect the codec rather than nanocdr.
// info.compression_opt is forced to NONE unless --zstd is passed, so the
// default numbers stay independent of stage-2 compression.
//
// Streams the file message-by-message; safe on bags larger than RAM.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/ros_message_definitions.hpp"
#include "cloudini_lib/ros_msg_utils.hpp"
#include "cxxopts.hpp"

#define MCAP_IMPLEMENTATION
#include "mcap/reader.hpp"
#include "mcap/types.hpp"

namespace {

enum class Mode { V4 = 0, V5 = 1, V4_VIZ = 2, V5_VIZ = 3 };
constexpr int kModeCount = 4;
const char* kModeNames[kModeCount] = {"V4", "V5", "V4-viz", "V5-viz"};

int modeIndexFromName(const std::string& name) {
  for (int i = 0; i < kModeCount; ++i) {
    if (name == kModeNames[i]) {
      return i;
    }
  }
  return -1;
}

struct ModeStats {
  uint64_t out_bytes = 0;
  uint64_t zstd_bytes = 0;
  uint64_t enc_input_bytes = 0;
  uint64_t dec_input_bytes = 0;
  uint64_t enc_ns = 0;
  uint64_t dec_ns = 0;
};

struct TopicStats {
  uint64_t messages = 0;
  uint64_t points = 0;
  uint64_t in_bytes = 0;
  ModeStats per_mode[kModeCount];
};

using Clock = std::chrono::steady_clock;

struct DecodeSample {
  std::string topic;
  int mode = -1;
  uint64_t raw_size = 0;
  uint64_t out_bytes_needed = 0;
  Cloudini::EncodingInfo header_info;
  std::vector<uint8_t> encoded;
};

inline uint64_t elapsedNs(Clock::time_point t0, Clock::time_point t1) {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
}


void configureMode(Cloudini::EncodingInfo& info, Mode mode, bool with_zstd) {
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
  // When --zstd is set, use cloudini's built-in stage-2 ZSTD on each chunk.
  // The encoder/decoder handle compression internally; encode() returns only
  // after the worker-thread ZSTD pass has caught up, so wall-clock encode
  // time includes both stages. Decoder mirrors. This matches the path
  // cloudini_rosbag_converter takes when MCAP chunk compression is off.
  info.compression_opt =
      with_zstd ? Cloudini::CompressionOption::ZSTD : Cloudini::CompressionOption::NONE;
  info.version = (mode == Mode::V5 || mode == Mode::V5_VIZ) ? 5 : 4;
  // Viz modes differ only in the per-message preprocessing step applied to
  // pc_info before this configureMode runs.
}

std::string fmtMiB(uint64_t bytes) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(2) << (static_cast<double>(bytes) / (1024.0 * 1024.0));
  return s.str();
}

std::string fmtRatio(uint64_t out_bytes, uint64_t in_bytes) {
  if (in_bytes == 0) return "n/a";
  std::ostringstream s;
  s << std::fixed << std::setprecision(1) << (100.0 * static_cast<double>(out_bytes) / static_cast<double>(in_bytes))
    << "%";
  return s.str();
}

std::string fmtMBps(uint64_t bytes, uint64_t ns) {
  if (ns == 0) return "n/a";
  std::ostringstream s;
  s << std::fixed << std::setprecision(0)
    << (static_cast<double>(bytes) * 1000.0 / static_cast<double>(ns));
  return s.str();
}

void printTable(const std::string& title, const TopicStats& st, bool show_zstd) {
  std::cout << "\n" << title;
  if (show_zstd) {
    std::cout << "  [encode + ZSTD / ZSTD-decompress + decode]";
  } else {
    std::cout << "  [codec-only, no stage-2 ZSTD]";
  }
  std::cout << "\n";
  std::cout << "  messages=" << st.messages << "  points=" << st.points << "  raw=" << fmtMiB(st.in_bytes) << " MiB\n";
  std::cout << "  " << std::string(60, '-') << "\n";
  std::cout << "  " << std::left << std::setw(10) << "Mode" << std::right << std::setw(12) << "Out MiB"
            << std::setw(10) << "Ratio" << std::setw(12) << "Enc MB/s" << std::setw(12) << "Dec MB/s" << "\n";
  std::cout << "  " << std::string(60, '-') << "\n";
  for (int m = 0; m < kModeCount; ++m) {
    const auto& ms = st.per_mode[m];
    std::cout << "  " << std::left << std::setw(10) << kModeNames[m] << std::right << std::setw(12)
              << fmtMiB(ms.out_bytes) << std::setw(10) << fmtRatio(ms.out_bytes, st.in_bytes)
              << std::setw(12) << fmtMBps(ms.enc_input_bytes, ms.enc_ns) << std::setw(12)
              << fmtMBps(ms.dec_input_bytes, ms.dec_ns) << "\n";
  }
}

// --explain mode: print the field schema for the first message of each topic,
// the NaN count, and the dedup count after viz preprocessing. Helpful for
// understanding what --viz will do on a given bag without running the full
// throughput sweep.
void explainOneMessage(
    const std::string& topic, const cloudini_ros::RosPointCloud2& base_pc_info,
    const Cloudini::EncodingInfo& base_info) {
  using namespace Cloudini;
  std::cout << "\n========================================================================\n";
  std::cout << "Topic: " << topic << "\n";
  const size_t n_in = base_pc_info.data.size() / base_info.point_step;
  std::cout << "  point_step=" << base_info.point_step << "  n_points=" << n_in
            << "  raw=" << fmtMiB(base_pc_info.data.size()) << " MiB\n";

  std::cout << "\n  Fields:\n";
  for (size_t i = 0; i < base_info.fields.size(); ++i) {
    const auto& f = base_info.fields[i];
    std::cout << "    [" << i << "] " << std::left << std::setw(18) << f.name
              << " type=" << std::setw(8) << ToString(f.type) << " offset=" << std::setw(3)
              << f.offset << " size=" << SizeOf(f.type)
              << " resolution=" << (f.resolution ? std::to_string(f.resolution.value()) : "n/a")
              << "\n";
  }

  // Run viz preprocessing on a copy and report what changed.
  cloudini_ros::RosPointCloud2 pc_copy = base_pc_info;
  cloudini_ros::applyVizLossyPreprocessing(pc_copy);
  const size_t n_after = pc_copy.data.size() / std::max<uint32_t>(1u, pc_copy.point_step);
  std::cout << "\n  After --viz preprocessing:\n";
  std::cout << "    points: " << n_in << " -> " << n_after << "  (removed " << (n_in - n_after)
            << " = ";
  if (n_in > 0) {
    std::cout << std::fixed << std::setprecision(1) << (100.0 * (n_in - n_after) / n_in) << "%";
  } else {
    std::cout << "n/a";
  }
  std::cout << ")\n";
  std::cout << "    FLOAT64 fields quantized to 1us: ";
  bool any = false;
  for (const auto& f : pc_copy.fields) {
    if (f.type == FieldType::FLOAT64 && f.resolution.has_value()) {
      std::cout << (any ? ", " : "") << f.name;
      any = true;
    }
  }
  if (!any) std::cout << "(none)";
  std::cout << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  cxxopts::Options options(
      "mcap_codec_benchmark",
      "Compare V4/V5 and V4-viz/V5-viz lossy PointCloud2 compression in an MCAP file.\n"
      "Streams the file message-by-message; safe on bags larger than RAM.");
  options.add_options()                                                                                    //
      ("h,help", "Print usage")                                                                            //
      ("f,filename", "Input MCAP file (positional also accepted)", cxxopts::value<std::string>())          //
      ("r,resolution", "XYZ tick size in meters (default 0.001)", cxxopts::value<float>()->default_value(  //
                                                                      "0.001"))                            //
      ("max-messages", "Stop after N messages per topic (0 = unlimited)",                                  //
       cxxopts::value<uint64_t>()->default_value("0"))                                                     //
      ("sample-every", "Process only 1 of every N messages per topic (>=1)",                               //
       cxxopts::value<uint64_t>()->default_value("1"))                                                     //
      ("zstd", "Use Cloudini ZSTD chunk compression and report compressed sizes")                    //
      ("mode", "Profile only one mode: V4, V5, V4-viz, or V5-viz", cxxopts::value<std::string>())           //
      ("encode-only", "Skip decode timing; useful with --mode for perf profiling")                          //
      ("decode-replay",
       "Store encoded pointclouds, then time decode after all MCAP messages are read/encoded")              //
      ("decode-repeat", "Repeat decode replay N times (only with --decode-replay)",                         //
       cxxopts::value<uint64_t>()->default_value("1"))                                                     //
      ("profile-sleep-ms", "Sleep after preload and before decode replay so perf can attach",               //
       cxxopts::value<uint64_t>()->default_value("0"))                                                     //
      ("explain", "Print the field schema and viz-preprocessing effect for the first message of each topic and exit");
  options.parse_positional({"filename"});
  options.positional_help("<file.mcap>");

  cxxopts::ParseResult parse_result;
  try {
    parse_result = options.parse(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Argument error: " << e.what() << "\n";
    return 1;
  }
  if (parse_result.count("help") || !parse_result.count("filename")) {
    std::cout << options.help() << std::endl;
    return parse_result.count("help") ? 0 : 1;
  }

  const std::filesystem::path input_file = parse_result["filename"].as<std::string>();
  const float default_resolution = parse_result["resolution"].as<float>();
  const uint64_t max_per_topic = parse_result["max-messages"].as<uint64_t>();
  const uint64_t sample_every = std::max<uint64_t>(1, parse_result["sample-every"].as<uint64_t>());
  const bool show_zstd = parse_result.count("zstd") > 0;
  const bool explain_mode = parse_result.count("explain") > 0;
  const bool encode_only = parse_result.count("encode-only") > 0;
  const bool decode_replay = parse_result.count("decode-replay") > 0;
  const uint64_t decode_repeat = std::max<uint64_t>(1, parse_result["decode-repeat"].as<uint64_t>());
  const uint64_t profile_sleep_ms = parse_result["profile-sleep-ms"].as<uint64_t>();
  int only_mode = -1;
  if (parse_result.count("mode") > 0) {
    only_mode = modeIndexFromName(parse_result["mode"].as<std::string>());
    if (only_mode < 0) {
      std::cerr << "Error: unknown mode '" << parse_result["mode"].as<std::string>()
                << "'. Expected one of: V4, V5, V4-viz, V5-viz\n";
      return 1;
    }
  }
  if (decode_replay && encode_only) {
    std::cerr << "Error: --decode-replay and --encode-only are mutually exclusive\n";
    return 1;
  }
  if (decode_replay && only_mode < 0) {
    std::cerr << "Error: --decode-replay requires --mode so the in-memory replay stays bounded\n";
    return 1;
  }

  if (!std::filesystem::exists(input_file)) {
    std::cerr << "Error: file does not exist: " << input_file << "\n";
    return 1;
  }

  std::cout << "File: " << input_file << "\n";
  std::cout << "Resolution: " << default_resolution << " m   max-messages/topic: ";
  if (max_per_topic == 0)
    std::cout << "unlimited";
  else
    std::cout << max_per_topic;
  std::cout << "   sample-every: " << sample_every;
  if (show_zstd) std::cout << "   +zstd";
  std::cout << "\n";

  std::ifstream input_stream(input_file);
  auto data_source = std::make_shared<mcap::FileStreamReader>(input_stream);
  mcap::McapReader reader;

  auto status = reader.open(*data_source);
  if (!status.ok()) {
    std::cerr << "Error opening MCAP: " << status.message << "\n";
    return 1;
  }
  status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  if (!status.ok()) {
    std::cerr << "Error reading summary: " << status.message << "\n";
    return 1;
  }

  std::map<mcap::ChannelId, std::string> pc_channels;
  for (const auto& [channel_id, channel_ptr] : reader.channels()) {
    const auto& schema_ptr = reader.schema(channel_ptr->schemaId);
    if (schema_ptr && schema_ptr->name == pointcloud_schema_name) {
      pc_channels[channel_id] = channel_ptr->topic;
    }
  }
  if (pc_channels.empty()) {
    std::cerr << "No raw sensor_msgs/msg/PointCloud2 topics found.\n";
    return 1;
  }
  std::cout << "PointCloud2 topics found: " << pc_channels.size() << "\n";
  for (const auto& [_, topic] : pc_channels) std::cout << "  - " << topic << "\n";

  // ----- explain mode -----
  if (explain_mode) {
    std::map<std::string, bool> explained;
    mcap::ReadMessageOptions ropts;
    mcap::ProblemCallback prob = [](const mcap::Status&) {};
    for (const auto& msg : reader.readMessages(prob, ropts)) {
      auto chan_it = pc_channels.find(msg.channel->id);
      if (chan_it == pc_channels.end()) continue;
      const std::string& topic = chan_it->second;
      if (explained[topic]) continue;
      explained[topic] = true;

      Cloudini::ConstBufferView raw_dds_msg(msg.message.data, msg.message.dataSize);
      cloudini_ros::RosPointCloud2 pc_info;
      try {
        pc_info = cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg);
      } catch (const std::exception& e) {
        std::cerr << "  [warn] " << topic << " parse failed: " << e.what() << "\n";
        continue;
      }
      cloudini_ros::applyResolutionProfile({}, pc_info.fields, default_resolution);
      Cloudini::EncodingInfo base = cloudini_ros::toEncodingInfo(pc_info);
      explainOneMessage(topic, pc_info, base);

      bool all_done = true;
      for (const auto& [_, t] : pc_channels) {
        if (!explained[t]) { all_done = false; break; }
      }
      if (all_done) break;
    }
    return 0;
  }

  // ----- throughput mode -----
  std::map<std::string, TopicStats> stats;
  std::map<std::string, uint64_t> seen;
  std::vector<uint8_t> enc_buf[kModeCount];
  std::vector<uint8_t> dec_buf[kModeCount];
  std::vector<DecodeSample> decode_samples;

  mcap::ReadMessageOptions reader_options;
  mcap::ProblemCallback problem = [](const mcap::Status&) {};
  uint64_t total_messages_processed = 0;

  for (const auto& msg : reader.readMessages(problem, reader_options)) {
    auto chan_it = pc_channels.find(msg.channel->id);
    if (chan_it == pc_channels.end()) continue;
    const std::string& topic = chan_it->second;
    const uint64_t this_seen = ++seen[topic];
    if (max_per_topic != 0 && this_seen > max_per_topic) continue;
    if ((this_seen - 1) % sample_every != 0) continue;

    Cloudini::ConstBufferView raw_dds_msg(msg.message.data, msg.message.dataSize);
    cloudini_ros::RosPointCloud2 pc_info_orig;
    try {
      pc_info_orig = cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg);
    } catch (const std::exception& e) {
      std::cerr << "  [warn] " << topic << " msg #" << this_seen << ": parse failed (" << e.what()
                << "), skipping\n";
      continue;
    }
    cloudini_ros::applyResolutionProfile({}, pc_info_orig.fields, default_resolution);
    Cloudini::EncodingInfo base = cloudini_ros::toEncodingInfo(pc_info_orig);

    Cloudini::ConstBufferView raw_points(pc_info_orig.data.data(), pc_info_orig.data.size());
    const uint64_t raw_size = static_cast<uint64_t>(raw_points.size());
    if (raw_size == 0 || base.point_step == 0) continue;
    const uint64_t point_count = raw_size / base.point_step;

    auto& st = stats[topic];
    st.messages += 1;
    st.points += point_count;
    st.in_bytes += raw_size;

    bool encode_ok[kModeCount];
    std::fill(std::begin(encode_ok), std::end(encode_ok), true);
    size_t encoded_size[kModeCount] = {};

    for (int m = 0; m < kModeCount; ++m) {
      if (only_mode >= 0 && m != only_mode) {
        encode_ok[m] = false;
        continue;
      }
      // Per-mode pc_info copy (V4_VIZ overwrites pc_info via preprocessing).
      cloudini_ros::RosPointCloud2 pc_info = pc_info_orig;
      auto info = base;
      configureMode(info, static_cast<Mode>(m), show_zstd);

      // Time-includes preprocessing for V4_VIZ: --viz pays both costs.
      const auto t_pre0 = Clock::now();
      if (static_cast<Mode>(m) == Mode::V4_VIZ || static_cast<Mode>(m) == Mode::V5_VIZ) {
        cloudini_ros::applyVizLossyPreprocessing(pc_info);
        info = cloudini_ros::toEncodingInfo(pc_info);
        info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
        info.compression_opt =
            show_zstd ? Cloudini::CompressionOption::ZSTD : Cloudini::CompressionOption::NONE;
        info.version = (static_cast<Mode>(m) == Mode::V5_VIZ) ? 5 : 4;
      }
      const auto t_pre1 = Clock::now();
      st.per_mode[m].enc_ns += elapsedNs(t_pre0, t_pre1);

      Cloudini::ConstBufferView enc_in(pc_info.data.data(), pc_info.data.size());
      try {
        Cloudini::PointcloudEncoder encoder(info);
        const size_t points_to_encode = pc_info.data.size() / info.point_step;
        const size_t max_encoded_size = Cloudini::MaxCompressedSize(info, points_to_encode, true);
        if (enc_buf[m].size() < max_encoded_size) {
          enc_buf[m].resize(max_encoded_size);
        }
        Cloudini::BufferView enc_out(enc_buf[m].data(), enc_buf[m].size());
        const auto t0 = Clock::now();
        encoded_size[m] = encoder.encode(enc_in, enc_out, true);
        const auto t1 = Clock::now();
        st.per_mode[m].enc_ns += elapsedNs(t0, t1);
        st.per_mode[m].enc_input_bytes += raw_size;
        st.per_mode[m].out_bytes += encoded_size[m];
      } catch (const std::exception& e) {
        std::cerr << "  [warn] " << topic << " msg #" << this_seen << " encode " << kModeNames[m]
                  << " failed: " << e.what() << "\n";
        encode_ok[m] = false;
        continue;
      }
    }

    // Decode each successfully-encoded buffer. When info.compression_opt was
    // set to ZSTD on encode, the encoder's chunk frames carry ZSTD-compressed
    // bytes; PointcloudDecoder handles decompression internally so the dec
    // timer captures both ZSTD-decompress + codec-decode.
    if (encode_only) {
      total_messages_processed += 1;
      if ((total_messages_processed % 200) == 0) {
        std::cout << "  ... " << total_messages_processed << " messages processed\n";
      }
      continue;
    }

    for (int m = 0; m < kModeCount; ++m) {
      if (!encode_ok[m]) continue;
      Cloudini::ConstBufferView enc_view(enc_buf[m].data(), encoded_size[m]);
      Cloudini::EncodingInfo header_info;
      try {
        header_info = Cloudini::DecodeHeader(enc_view);
      } catch (const std::exception& e) {
        std::cerr << "  [warn] " << topic << " msg #" << this_seen << " header parse "
                  << kModeNames[m] << " failed: " << e.what() << "\n";
        continue;
      }
      const uint64_t out_points =
          static_cast<uint64_t>(header_info.width) * static_cast<uint64_t>(header_info.height);
      const uint64_t out_bytes_needed = out_points * header_info.point_step;
      if (decode_replay) {
        DecodeSample sample;
        sample.topic = topic;
        sample.mode = m;
        sample.raw_size = raw_size;
        sample.out_bytes_needed = out_bytes_needed;
        sample.header_info = std::move(header_info);
        sample.encoded.assign(enc_view.data(), enc_view.data() + enc_view.size());
        decode_samples.push_back(std::move(sample));
        continue;
      }
      if (dec_buf[m].size() < out_bytes_needed) dec_buf[m].resize(out_bytes_needed);
      Cloudini::BufferView out_view(dec_buf[m].data(), out_bytes_needed);
      try {
        Cloudini::PointcloudDecoder decoder;
        const auto t0 = Clock::now();
        decoder.decode(header_info, enc_view, out_view);
        const auto t1 = Clock::now();
        st.per_mode[m].dec_ns += elapsedNs(t0, t1);
        st.per_mode[m].dec_input_bytes += raw_size;
      } catch (const std::exception& e) {
        std::cerr << "  [warn] " << topic << " msg #" << this_seen << " decode " << kModeNames[m]
                  << " failed: " << e.what() << "\n";
      }
    }

    total_messages_processed += 1;
    if ((total_messages_processed % 200) == 0) {
      std::cout << "  ... " << total_messages_processed << " messages processed\n" << std::flush;
    }
  }

  if (stats.empty()) {
    std::cout << "\nNo PointCloud2 messages were processed.\n";
    return 0;
  }

  if (decode_replay) {
    const uint64_t replay_count = static_cast<uint64_t>(decode_samples.size()) * decode_repeat;
    std::cout << "\nDecode replay: samples=" << decode_samples.size()
              << "  repeat=" << decode_repeat << "  total-decodes=" << replay_count
              << "  (MCAP read/decompression excluded)\n";
    if (profile_sleep_ms > 0) {
      std::cout << "Sleeping " << profile_sleep_ms << " ms before decode replay"
                << " so perf can attach...\n" << std::flush;
      std::this_thread::sleep_for(std::chrono::milliseconds(profile_sleep_ms));
    }

    uint64_t replayed = 0;
    for (uint64_t repeat = 0; repeat < decode_repeat; ++repeat) {
      for (const auto& sample : decode_samples) {
        if (dec_buf[sample.mode].size() < sample.out_bytes_needed) {
          dec_buf[sample.mode].resize(sample.out_bytes_needed);
        }
        Cloudini::ConstBufferView enc_view(sample.encoded.data(), sample.encoded.size());
        Cloudini::BufferView out_view(dec_buf[sample.mode].data(), sample.out_bytes_needed);
        try {
          Cloudini::PointcloudDecoder decoder;
          const auto t0 = Clock::now();
          decoder.decode(sample.header_info, enc_view, out_view);
          const auto t1 = Clock::now();
          auto& ms = stats[sample.topic].per_mode[sample.mode];
          ms.dec_ns += elapsedNs(t0, t1);
          ms.dec_input_bytes += sample.raw_size;
        } catch (const std::exception& e) {
          std::cerr << "  [warn] " << sample.topic << " decode replay " << kModeNames[sample.mode]
                    << " failed: " << e.what() << "\n";
        }
        replayed += 1;
        if ((replayed % 200) == 0) {
          std::cout << "  ... " << replayed << " decode replays processed\n" << std::flush;
        }
      }
    }
  }

  TopicStats grand_total;
  for (const auto& [topic, st] : stats) {
    printTable("Topic: " + topic, st, show_zstd);
    grand_total.messages += st.messages;
    grand_total.points += st.points;
    grand_total.in_bytes += st.in_bytes;
    for (int m = 0; m < kModeCount; ++m) {
      grand_total.per_mode[m].out_bytes += st.per_mode[m].out_bytes;
      grand_total.per_mode[m].zstd_bytes += st.per_mode[m].zstd_bytes;
      grand_total.per_mode[m].enc_input_bytes += st.per_mode[m].enc_input_bytes;
      grand_total.per_mode[m].dec_input_bytes += st.per_mode[m].dec_input_bytes;
      grand_total.per_mode[m].enc_ns += st.per_mode[m].enc_ns;
      grand_total.per_mode[m].dec_ns += st.per_mode[m].dec_ns;
    }
  }
  if (stats.size() > 1) {
    printTable("FILE TOTAL", grand_total, show_zstd);
  }
  return 0;
}
