// Data for the README charts: Cloudini (V6, 1 mm, refined, ZSTD) vs ZSTD level 1 on the raw cloud.
// Size, and encode/decode MB/s as the best of 5 interleaved rounds; every decoded cloud is verified.
// Built and run by scripts/regenerate_readme_plots.py:
//
//   readme_bench <manifest> [min_mb]      -> one JSON line per dataset
//
// Manifest:
//   D <id> <point_step> <name...>      dataset
//   F <offset> <type> <field name>     field
//   C <width> <height> <path>          raw frame
//   H <id> <path> <name...>            dataset from one CDR sensor_msgs/PointCloud2 message
#include <zstd.h>

#include <algorithm>
#include <chrono>
#include <cloudini_lib/cloudini.hpp>
#include <cloudini_lib/ros_msg_utils.hpp>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace Cloudini;

struct Frame {
  uint32_t width, height;
  std::vector<uint8_t> data;
};
struct Dataset {
  std::string id, name;
  uint32_t point_step = 0;
  std::vector<PointField> fields;
  std::vector<Frame> frames;
  size_t rawBytes() const {
    size_t s = 0;
    for (auto& f : frames) s += f.data.size();
    return s;
  }
};

static std::vector<uint8_t> readFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(f), {});
}

static std::vector<Dataset> loadManifest(const std::string& path) {
  std::vector<Dataset> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string tag;
    ss >> tag;
    auto rest = [&ss] {
      std::string r;
      std::getline(ss >> std::ws, r);
      return r;
    };
    if (tag == "D") {
      Dataset d;
      ss >> d.id >> d.point_step;
      d.name = rest();
      out.push_back(std::move(d));
    } else if (tag == "F") {
      PointField f;
      int type;
      ss >> f.offset >> type;
      f.type = FieldType(type);
      f.name = rest();
      out.back().fields.push_back(f);
    } else if (tag == "C") {
      Frame fr;
      ss >> fr.width >> fr.height;
      fr.data = readFile(rest());
      if (fr.data.size() != size_t(fr.width) * fr.height * out.back().point_step) throw std::runtime_error("bad size");
      out.back().frames.push_back(std::move(fr));
    } else if (tag == "H") {
      Dataset d;
      std::string p;
      ss >> d.id >> p;
      d.name = rest();
      const auto raw = readFile(p);
      auto pc = cloudini_ros::getDeserializedPointCloudMessage(ConstBufferView(raw.data(), raw.size()));
      d.point_step = pc.point_step;
      for (auto f : pc.fields) {
        f.resolution.reset();
        d.fields.push_back(f);
      }
      Frame fr{pc.width, pc.height, std::vector<uint8_t>(pc.data.data(), pc.data.data() + pc.data.size())};
      d.frames.push_back(std::move(fr));
      out.push_back(std::move(d));
    }
  }
  return out;
}

// 1 mm on every FLOAT32 field; integers and FLOAT64 lossless
static EncodingInfo baseInfo(const Dataset& d, const Frame& fr, int version, CompressionOption c) {
  EncodingInfo info;
  info.fields = d.fields;
  for (auto& f : info.fields) {
    if (f.type == FieldType::FLOAT32) f.resolution = 0.001f;
  }
  info.width = fr.width;
  info.height = fr.height;
  info.point_step = d.point_step;
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = c;
  info.version = uint8_t(version);
  return info;
}

struct Config {
  std::string mode;
  CompressionOption c;
  bool refine;
  int version;
  PointcloudEncoderCache cache;
  std::vector<std::vector<uint8_t>> encoded;
};

static void encodeFrame(Config& cfg, const Dataset& d, const Frame& fr, std::vector<uint8_t>& out) {
  auto info = baseInfo(d, fr, cfg.version, cfg.c);
  const ConstBufferView view(fr.data.data(), fr.data.size());
  if (cfg.refine) RefineResolutionsToData(info, view);
  cfg.cache.get(info).encode(view, out);
}

static void decodeFrame(const std::vector<uint8_t>& enc, std::vector<uint8_t>& out) {
  ConstBufferView view(enc.data(), enc.size());
  const auto header = DecodeHeader(view);
  out.resize(size_t(header.width) * header.height * header.point_step);
  PointcloudDecoder().decode(header, view, BufferView(out.data(), out.size()));
}

// every FLOAT32 within resolution/2 (+1 ulp), NaN stays NaN, everything else exact
static void verify(const Dataset& d, const Frame& fr, const std::vector<uint8_t>& dec, const std::string& what) {
  if (dec.size() != fr.data.size()) throw std::runtime_error(what + ": size mismatch");
  const size_t n = size_t(fr.width) * fr.height;
  for (const auto& f : d.fields) {
    for (size_t i = 0; i < n; ++i) {
      const uint8_t* a = fr.data.data() + i * d.point_step + f.offset;
      const uint8_t* b = dec.data() + i * d.point_step + f.offset;
      if (f.type == FieldType::FLOAT32) {
        float x, y;
        memcpy(&x, a, 4);
        memcpy(&y, b, 4);
        if (std::isnan(x) != std::isnan(y)) throw std::runtime_error(what + ": NaN mismatch in " + f.name);
        if (std::isnan(x)) continue;
        if (std::isinf(x) ? x != y : std::abs(double(x) - y) > 0.0005 * 1.001 + std::abs(std::nextafter(x, INFINITY) - x))
          throw std::runtime_error(what + ": " + f.name + " off by " + std::to_string(std::abs(double(x) - y)));
      } else if (memcmp(a, b, SizeOf(f.type)) != 0) {
        throw std::runtime_error(what + ": " + f.name + " not exact");
      }
    }
  }
}

static double now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// README chart: ZSTD level 1 on the raw cloud vs Cloudini V6 + refine + ZSTD. Best of 5 interleaved rounds.
static int readme(std::vector<Dataset>& sets, double min_mb) {
  ZSTD_CCtx* cctx = ZSTD_createCCtx();
  ZSTD_DCtx* dctx = ZSTD_createDCtx();
  for (auto& d : sets) {
    const size_t raw = d.rawBytes();
    const int reps = std::max<int>(1, int(std::ceil(min_mb * 1e6 / raw)));
    Config cfg{"zstd-ref", CompressionOption::ZSTD, true, 6};
    cfg.encoded.resize(d.frames.size());
    std::vector<std::vector<uint8_t>> zenc(d.frames.size());
    std::vector<uint8_t> dec, tmp;
    size_t zbytes = 0, cbytes = 0;
    for (size_t i = 0; i < d.frames.size(); ++i) {
      encodeFrame(cfg, d, d.frames[i], cfg.encoded[i]);
      decodeFrame(cfg.encoded[i], dec);
      verify(d, d.frames[i], dec, d.id);
      cbytes += cfg.encoded[i].size();
      auto& fr = d.frames[i];
      zenc[i].resize(ZSTD_compressBound(fr.data.size()));
      zenc[i].resize(ZSTD_compressCCtx(cctx, zenc[i].data(), zenc[i].size(), fr.data.data(), fr.data.size(), 1));
      zbytes += zenc[i].size();
    }
    double ce = 1e30, cd = 1e30, ze = 1e30, zd = 1e30;
    tmp.resize(ZSTD_compressBound(d.frames[0].data.size()) * 2);
    dec.resize(d.frames[0].data.size() * 2);
    for (int round = 0; round < 5; ++round) {
      double t0 = now();
      for (int r = 0; r < reps; ++r)
        for (auto& fr : d.frames) encodeFrame(cfg, d, fr, tmp);
      double t1 = now();
      for (int r = 0; r < reps; ++r)
        for (auto& e : cfg.encoded) decodeFrame(e, dec);
      double t2 = now();
      tmp.resize(ZSTD_compressBound(raw));
      for (int r = 0; r < reps; ++r)
        for (auto& fr : d.frames) ZSTD_compressCCtx(cctx, tmp.data(), tmp.size(), fr.data.data(), fr.data.size(), 1);
      double t3 = now();
      dec.resize(raw);
      for (int r = 0; r < reps; ++r)
        for (auto& e : zenc) ZSTD_decompressDCtx(dctx, dec.data(), dec.size(), e.data(), e.size());
      double t4 = now();
      ce = std::min(ce, t1 - t0); cd = std::min(cd, t2 - t1); ze = std::min(ze, t3 - t2); zd = std::min(zd, t4 - t3);
    }
    const double mb = double(raw) * reps / 1e6;
    std::cout << "{\"id\":\"" << d.id << "\",\"name\":\"" << d.name << "\",\"raw\":" << raw
              << ",\"zstd\":{\"ratio\":" << 100.0 * zbytes / raw << ",\"enc\":" << mb / ze << ",\"dec\":" << mb / zd << "}"
              << ",\"cloudini\":{\"ratio\":" << 100.0 * cbytes / raw << ",\"enc\":" << mb / ce << ",\"dec\":" << mb / cd << "}}" << std::endl;
  }
  return 0;
}

int main(int argc, char** argv) {
  try {
    auto sets = loadManifest(argv[1]);
    return readme(sets, argc > 2 ? std::stod(argv[2]) : 128);
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}
