// Experiment: rANS instead of ZSTD as stage 2.
//
// Encodes clouds with V5/V6 (1 mm, refined), captures every stage-1 chunk together with its section
// starts (the offsets where the ZSTD path begins a new block), and compresses the captured chunks with:
//   zstd-N        one ZSTD frame per chunk, new block at each section (as CompressChunk), level N
//   lz4           LZ4 default
//   rans-o0       order-0 static rANS, one table per chunk
//   rans-o0/sec   order-0 static rANS, one table per section
//   rans-vpos/sec one table per (section, position of the byte inside its varint: 0, 1, 2, 3+)
//   rans-best/sec per section the smallest of stored / o0 / vpos (1 byte tag)
//   rans-val/sec  as rans-best/sec, plus value-level coding of sections that parse as varints (the V4
//                 per-point and V6 residual streams): rANS on an Exp-Golomb-like bucket of each value
//                 (context: bit length of the previous value) + raw low bits
//   rans-valrep/sec as rans-val/sec, plus a "same value as L positions back" symbol (L chosen per section)
// rans-val* need the stage-1 format (varint streams): they model stage 1, they are not a generic stage 2.
// rANS: 32-bit state, byte renormalization, 4 interleaved states, 12-bit frequencies; table header
// counted in the size. Every rANS stream is decoded and compared with its input.
//
// Build (after build_release; the --wrap captures the stage-1 chunks passed to detail::CompressChunk):
//   g++ -O2 -march=native -std=c++20 -Icloudini_lib/include scripts/rans_stage2_bench.cpp
//     build_release/libcloudini_lib.a -lzstd -llz4 -lpthread
//     -Wl,--wrap=<mangled name of CompressChunk, CC_MANGLED below>
//     -o build_release/rans_stage2_bench
// Run:
//   build_release/rans_stage2_bench <cloud> [<cloud> ...]     (.pcd binary, or a CDR PointCloud2 .bin)
//   SECTIONS=1 adds the per-section sizes (zstd-1 vs the best rANS), DUMP=<dir> also writes the sections.
// The section sizes a real rANS format would have to store are counted (as varints).
#include <lz4.h>
#include <zstd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cloudini_lib/cloudini.hpp>
#include <cloudini_lib/ros_msg_utils.hpp>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace Cloudini;

//------------------------------------------------------------------------------------------
// Capture of the stage-1 chunks (linker --wrap of detail::CompressChunk)

struct Stage1Chunk {
  std::vector<uint8_t> data;
  std::vector<size_t> starts;
};
static std::vector<Stage1Chunk>* g_capture = nullptr;
static std::map<int, size_t>* g_tag_stats = nullptr;
#define CC_MANGLED "_ZN8Cloudini6detail13CompressChunkENS_17CompressionOptionE4SpanIKhERS2_IhESt4spanIKmLm18446744073709551615EE"
uint32_t realCompressChunk(CompressionOption, ConstBufferView, BufferView&, std::span<const size_t>) __asm__(
    "__real_" CC_MANGLED);
uint32_t wrapCompressChunk(CompressionOption, ConstBufferView, BufferView&, std::span<const size_t>) __asm__(
    "__wrap_" CC_MANGLED);
uint32_t wrapCompressChunk(CompressionOption c, ConstBufferView in, BufferView& out, std::span<const size_t> starts) {
  if (g_capture) {
    g_capture->push_back({std::vector<uint8_t>(in.data(), in.data() + in.size()), {starts.begin(), starts.end()}});
  }
  return realCompressChunk(c, in, out, starts);
}

//------------------------------------------------------------------------------------------
// rANS (ryg_rans style, byte-wise renormalization)

namespace rans {
constexpr uint32_t kScaleBits = 12;
constexpr uint32_t kM = 1u << kScaleBits;
constexpr uint32_t kL = 1u << 23;

struct Table {
  std::array<uint16_t, 256> freq{};
  std::array<uint16_t, 256> start{};
  std::array<uint8_t, kM> slot2sym{};
};

void normalize(const std::array<uint32_t, 256>& counts, Table& t) {
  uint64_t total = 0;
  for (auto c : counts) total += c;
  t.freq.fill(0);
  if (total == 0) return;
  int64_t sum = 0;
  int best = 0;
  for (int s = 0; s < 256; ++s) {
    if (!counts[s]) continue;
    uint32_t f = uint32_t((uint64_t(counts[s]) * kM + total / 2) / total);
    t.freq[s] = uint16_t(std::max<uint32_t>(1, f));
    sum += t.freq[s];
    if (counts[s] > counts[best]) best = s;
  }
  // fix the sum: adjust the most frequent symbols (steal from those with freq > 1)
  while (sum != kM) {
    if (sum < kM) {
      t.freq[best] += uint16_t(kM - sum);
      sum = kM;
    } else {
      // take from the symbol with the smallest cost increase: largest freq first
      int s_max = -1;
      for (int s = 0; s < 256; ++s)
        if (t.freq[s] > 1 && (s_max < 0 || t.freq[s] > t.freq[s_max])) s_max = s;
      const int64_t take = std::min<int64_t>(sum - kM, t.freq[s_max] - 1);
      t.freq[s_max] -= uint16_t(take);
      sum -= take;
    }
  }
  uint32_t acc = 0;
  for (int s = 0; s < 256; ++s) {
    t.start[s] = uint16_t(acc);
    for (uint32_t i = 0; i < t.freq[s]; ++i) t.slot2sym[acc + i] = uint8_t(s);
    acc += t.freq[s];
  }
}

// Table header: count of symbols, then (symbol delta, freq-1) as varints
void putVarint(std::vector<uint8_t>& out, uint32_t v) {
  while (v >= 0x80) {
    out.push_back(uint8_t(v | 0x80));
    v >>= 7;
  }
  out.push_back(uint8_t(v));
}
uint32_t getVarint(const uint8_t*& p) {
  uint32_t v = 0;
  int sh = 0;
  while (*p & 0x80) {
    v |= uint32_t(*p++ & 0x7f) << sh;
    sh += 7;
  }
  v |= uint32_t(*p++) << sh;
  return v;
}
void writeTable(const Table& t, std::vector<uint8_t>& out) {
  int n = 0;
  for (int s = 0; s < 256; ++s) n += t.freq[s] > 0;
  putVarint(out, uint32_t(n));
  int prev = -1;
  for (int s = 0; s < 256; ++s) {
    if (!t.freq[s]) continue;
    putVarint(out, uint32_t(s - prev - 1));
    putVarint(out, t.freq[s] - 1u);
    prev = s;
  }
}
void readTable(const uint8_t*& p, Table& t) {
  t.freq.fill(0);
  const uint32_t n = getVarint(p);
  int s = -1;
  for (uint32_t i = 0; i < n; ++i) {
    s += int(getVarint(p)) + 1;
    t.freq[s] = uint16_t(getVarint(p) + 1);
  }
  uint32_t acc = 0;
  for (int k = 0; k < 256; ++k) {
    t.start[k] = uint16_t(acc);
    for (uint32_t i = 0; i < t.freq[k]; ++i) t.slot2sym[acc + i] = uint8_t(k);
    acc += t.freq[k];
  }
}

// Context models. ctx(i) must depend only on symbols before i.
struct CtxO0 {
  static constexpr int kN = 1;
  int pos = 0;
  int operator()() const { return 0; }
  void update(uint8_t) {}
};
struct CtxVarintPos {
  static constexpr int kN = 4;
  int pos = 0;
  int operator()() const { return std::min(pos, 3); }
  void update(uint8_t b) { pos = (b & 0x80) ? pos + 1 : 0; }
};

// Encodes `in` with model Ctx; appends tables + stream (u32 stream length) to out.
template <class Ctx>
void encode(std::span<const uint8_t> in, std::vector<uint8_t>& out) {
  std::vector<std::array<uint32_t, 256>> counts(Ctx::kN);
  for (auto& c : counts) c.fill(0);
  std::vector<uint8_t> ctx_of(in.size());
  {
    Ctx ctx;
    for (size_t i = 0; i < in.size(); ++i) {
      ctx_of[i] = uint8_t(ctx());
      counts[ctx_of[i]][in[i]]++;
      ctx.update(in[i]);
    }
  }
  std::vector<Table> tables(Ctx::kN);
  for (int c = 0; c < Ctx::kN; ++c) {
    normalize(counts[c], tables[c]);
    writeTable(tables[c], out);
  }
  std::vector<uint8_t> buf(in.size() + 64 + in.size() / 4);
  uint8_t* end = buf.data() + buf.size();
  uint8_t* ptr = end;
  uint32_t x[4] = {kL, kL, kL, kL};
  for (size_t i = in.size(); i-- > 0;) {
    const Table& t = tables[ctx_of[i]];
    const uint8_t s = in[i];
    uint32_t& st = x[i & 3];
    const uint32_t f = t.freq[s];
    const uint32_t x_max = ((kL >> kScaleBits) << 8) * f;
    while (st >= x_max) {
      *--ptr = uint8_t(st);
      st >>= 8;
    }
    st = ((st / f) << kScaleBits) + (st % f) + t.start[s];
  }
  for (int k = 3; k >= 0; --k) {
    ptr -= 4;
    memcpy(ptr, &x[k], 4);
  }
  const uint32_t len = uint32_t(end - ptr);
  out.insert(out.end(), reinterpret_cast<const uint8_t*>(&len), reinterpret_cast<const uint8_t*>(&len) + 4);
  out.insert(out.end(), ptr, end);
}

template <class Ctx>
const uint8_t* decode(const uint8_t* p, uint8_t* out, size_t n) {
  std::vector<Table> tables(Ctx::kN);
  for (auto& t : tables) readTable(p, t);
  uint32_t len;
  memcpy(&len, p, 4);
  p += 4;
  const uint8_t* end = p + len;
  uint32_t x[4];
  for (int k = 0; k < 4; ++k) {
    memcpy(&x[k], p, 4);
    p += 4;
  }
  constexpr uint32_t mask = kM - 1;
  Ctx ctx;
  for (size_t i = 0; i < n; ++i) {
    const Table& t = tables[ctx()];
    uint32_t& st = x[i & 3];
    const uint32_t slot = st & mask;
    const uint8_t s = t.slot2sym[slot];
    out[i] = s;
    ctx.update(s);
    st = t.freq[s] * (st >> kScaleBits) + slot - t.start[s];
    while (st < kL) st = (st << 8) | *p++;
  }
  if (p != end) throw std::runtime_error("rANS: stream length mismatch");
  return p;
}
}  // namespace rans


// Value-level coding of a section that is a sequence of varints (the V6 residual streams): each value u
// becomes a symbol (u < 4: u; else 4 + 4 * (bitlen - 3) + the 2 bits below the leading one) coded with rANS,
// context = bit length of the previous value (capped), and bitlen - 3 raw low bits in a separate bitstream.
namespace valcoder {
inline int bitlen(uint64_t u) { return u ? 64 - __builtin_clzll(u) : 0; }
inline uint8_t symOf(uint64_t u, int& extra) {
  if (u < 4) {
    extra = 0;
    return uint8_t(u);
  }
  const int n = bitlen(u);
  extra = n - 3;
  return uint8_t(4 + 4 * (n - 3) + ((u >> (n - 3)) & 3));
}
inline int symBitlen(uint8_t s) { return s < 4 ? bitlen(s) : (s - 4) / 4 + 3; }

bool parse(std::span<const uint8_t> in, std::vector<uint64_t>& vals) {
  vals.clear();
  size_t i = 0;
  while (i < in.size()) {
    uint64_t v = 0;
    int sh = 0;
    while (true) {
      if (i >= in.size() || sh > 63) return false;
      const uint8_t b = in[i++];
      v |= uint64_t(b & 0x7f) << sh;
      sh += 7;
      if (!(b & 0x80)) break;
    }
    // canonical encoding only, so that re-encoding gives the same bytes
    if (sh > 7 && (v >> (sh - 7)) == 0) return false;
    vals.push_back(v);
  }
  return true;
}

constexpr uint8_t kRep = 255;  // same value as `lag` positions back

template <int kCtx>
struct Ctx {
  static constexpr int kN = kCtx + 1;
  int prev = 0;
  int operator()() const { return prev; }
  void update(uint8_t s) { prev = s == kRep ? kCtx : std::min(symBitlen(s), kCtx - 1); }
};

// The lag in 1..256 at which most values repeat; 0 when repeats are too rare to pay for the symbol.
size_t chooseRepLag(const std::vector<uint64_t>& vals) {
  size_t best = 0, best_hits = vals.size() / 20;
  for (size_t lag = 1; lag <= 256 && lag < vals.size(); ++lag) {
    size_t hits = 0;
    for (size_t i = lag; i < vals.size(); ++i) hits += vals[i] == vals[i - lag];
    if (hits > best_hits) best = lag, best_hits = hits;
  }
  return best;
}

template <int kCtx>
void encode(const std::vector<uint64_t>& vals, std::vector<uint8_t>& out, bool rep) {
  const size_t lag = rep ? chooseRepLag(vals) : 0;
  std::vector<uint8_t> syms(vals.size());
  std::vector<uint8_t> bits;
  uint64_t acc = 0;
  int nacc = 0;
  for (size_t i = 0; i < vals.size(); ++i) {
    int extra;
    // a repeat is worth it unless the value is tiny anyway
    if (lag && i >= lag && vals[i] == vals[i - lag] && vals[i] >= 4) {
      syms[i] = kRep;
      continue;
    }
    syms[i] = symOf(vals[i], extra);
    if (extra) {
      const uint64_t low = vals[i] & ((uint64_t(1) << extra) - 1);
      for (int k = 0; k < extra; ++k) {  // bit by bit: simple, extra may be up to 61
        acc |= ((low >> k) & 1) << nacc;
        if (++nacc == 64) {
          bits.insert(bits.end(), reinterpret_cast<uint8_t*>(&acc), reinterpret_cast<uint8_t*>(&acc) + 8);
          acc = 0;
          nacc = 0;
        }
      }
    }
  }
  bits.insert(bits.end(), reinterpret_cast<uint8_t*>(&acc), reinterpret_cast<uint8_t*>(&acc) + (nacc + 7) / 8);
  rans::putVarint(out, uint32_t(vals.size()));
  rans::putVarint(out, uint32_t(lag));
  rans::encode<Ctx<kCtx>>(syms, out);
  out.insert(out.end(), bits.begin(), bits.end());
}

// Writes the varint bytes of the section to `out` (exactly `size` bytes).
template <int kCtx>
const uint8_t* decode(const uint8_t* p, uint8_t* out, size_t size) {
  const size_t n = rans::getVarint(p);
  const size_t lag = rans::getVarint(p);
  std::vector<uint8_t> syms(n);
  std::vector<uint64_t> vals(n);
  p = rans::decode<Ctx<kCtx>>(p, syms.data(), n);
  uint64_t bitpos = 0;
  auto getBits = [&](int k) {
    uint64_t v = 0;
    for (int j = 0; j < k; ++j, ++bitpos) v |= uint64_t((p[bitpos >> 3] >> (bitpos & 7)) & 1) << j;
    return v;
  };
  uint8_t* o = out;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t s = syms[i];
    uint64_t u;
    if (s == kRep) {
      if (i < lag || !lag) throw std::runtime_error("valcoder: bad repeat");
      u = vals[i - lag];
    } else if (s < 4) {
      u = s;
    } else {
      const int nb = (s - 4) / 4 + 3;
      u = (uint64_t(4 | ((s - 4) & 3)) << (nb - 3)) | getBits(nb - 3);
    }
    vals[i] = u;
    while (u > 0x7f) {
      *o++ = uint8_t(u | 0x80);
      u >>= 7;
    }
    *o++ = uint8_t(u);
  }
  if (size_t(o - out) != size) throw std::runtime_error("valcoder: size mismatch");
  return p + (bitpos + 7) / 8;
}
}  // namespace valcoder

// Section helpers
static std::vector<std::pair<size_t, size_t>> sections(const Stage1Chunk& c, bool split) {
  std::vector<std::pair<size_t, size_t>> out;
  size_t start = 0;
  if (split) {
    for (size_t b : c.starts) {
      if (b > start && b < c.data.size()) {
        out.push_back({start, b});
        start = b;
      }
    }
  }
  out.push_back({start, c.data.size()});
  return out;
}

enum class Mode { O0, VPos, Best, BestVal, BestValRep };

// Compresses a chunk: per section a tag byte (0 stored, 1 o0, 2 vpos) unless `untagged`.
static void ransCompress(const Stage1Chunk& c, bool split, Mode mode, std::vector<uint8_t>& out) {
  out.clear();
  std::vector<uint8_t> a, b;
  for (auto [s, e] : sections(c, split)) {
    std::span<const uint8_t> in(c.data.data() + s, e - s);
    if (mode == Mode::O0) {
      out.push_back(1);
      rans::encode<rans::CtxO0>(in, out);
    } else if (mode == Mode::VPos) {
      out.push_back(2);
      rans::encode<rans::CtxVarintPos>(in, out);
    } else {
      a.clear();
      b.clear();
      rans::encode<rans::CtxO0>(in, a);
      rans::encode<rans::CtxVarintPos>(in, b);
      uint8_t tag = 0;
      const std::vector<uint8_t>* best = nullptr;
      size_t best_size = in.size();
      if (a.size() < best_size) tag = 1, best = &a, best_size = a.size();
      if (b.size() < best_size) tag = 2, best = &b, best_size = b.size();
      std::vector<uint64_t> vals;
      std::vector<uint8_t> v1, v12, vr;
      if ((mode == Mode::BestVal || mode == Mode::BestValRep) && valcoder::parse(in, vals)) {
        valcoder::encode<1>(vals, v1, false);
        valcoder::encode<12>(vals, v12, false);
        if (v1.size() < best_size) tag = 3, best = &v1, best_size = v1.size();
        if (v12.size() < best_size) tag = 4, best = &v12, best_size = v12.size();
        if (mode == Mode::BestValRep) {
          valcoder::encode<12>(vals, vr, true);
          if (vr.size() < best_size) tag = 4, best = &vr, best_size = vr.size();
        }
      }
      out.push_back(tag);
      if (best) {
        out.insert(out.end(), best->begin(), best->end());
      } else {
        out.insert(out.end(), in.begin(), in.end());
      }
      if (g_tag_stats) (*g_tag_stats)[tag] += in.size();
    }
  }
}

// The decoder knows the section sizes (in a real format they would be stored: ~2-3 bytes each, counted
// below as varints).
static void ransDecompress(const std::vector<uint8_t>& comp, const Stage1Chunk& c, bool split, std::vector<uint8_t>& out) {
  out.resize(c.data.size());
  const uint8_t* p = comp.data();
  for (auto [s, e] : sections(c, split)) {
    const uint8_t tag = *p++;
    if (tag == 0) {
      memcpy(out.data() + s, p, e - s);
      p += e - s;
    } else if (tag == 1) {
      p = rans::decode<rans::CtxO0>(p, out.data() + s, e - s);
    } else if (tag == 2) {
      p = rans::decode<rans::CtxVarintPos>(p, out.data() + s, e - s);
    } else if (tag == 3) {
      p = valcoder::decode<1>(p, out.data() + s, e - s);
    } else {
      p = valcoder::decode<12>(p, out.data() + s, e - s);
    }
  }
}

static size_t sectionSizeOverhead(const Stage1Chunk& c, bool split) {
  // stream length u32 is already inside each rANS section; section sizes as varints
  size_t bytes = 0;
  for (auto [s, e] : sections(c, split)) {
    std::vector<uint8_t> v;
    rans::putVarint(v, uint32_t(e - s));
    bytes += v.size();
  }
  return bytes;
}

static size_t zstdBlocks(const Stage1Chunk& c, int level, std::vector<uint8_t>& out, ZSTD_CCtx* cctx) {
  out.resize(ZSTD_compressBound(c.data.size()));
  ZSTD_CCtx_reset(cctx, ZSTD_reset_session_and_parameters);
  ZSTD_CCtx_setParameter(cctx, ZSTD_c_compressionLevel, level);
  ZSTD_CCtx_setPledgedSrcSize(cctx, c.data.size());
  ZSTD_outBuffer o = {out.data(), out.size(), 0};
  auto run = [&](size_t s, size_t e, ZSTD_EndDirective d) {
    ZSTD_inBuffer in = {c.data.data() + s, e - s, 0};
    while (true) {
      size_t r = ZSTD_compressStream2(cctx, &o, &in, d);
      if (ZSTD_isError(r)) throw std::runtime_error(ZSTD_getErrorName(r));
      if (r == 0 && in.pos == in.size) return;
    }
  };
  size_t start = 0;
  for (size_t b : c.starts)
    if (b > start && b < c.data.size()) {
      run(start, b, ZSTD_e_flush);
      start = b;
    }
  run(start, c.data.size(), ZSTD_e_end);
  out.resize(o.pos);
  return o.pos;
}

//------------------------------------------------------------------------------------------
// Inputs

struct Cloud {
  std::string name;
  EncodingInfo info;  // fields, width, height, point_step
  std::vector<uint8_t> data;
};

static std::vector<uint8_t> readFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open " + path);
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(f), {});
}

static Cloud loadPcd(const std::string& path) {
  auto raw = readFile(path);
  Cloud c;
  c.name = std::filesystem::path(path).filename().string();
  size_t pos = 0;
  std::vector<std::string> names, types;
  std::vector<int> sizes;
  while (true) {
    size_t eol = std::find(raw.begin() + pos, raw.end(), '\n') - raw.begin();
    std::string line(raw.begin() + pos, raw.begin() + eol);
    pos = eol + 1;
    std::istringstream ss(line);
    std::string key, v;
    ss >> key;
    if (key == "FIELDS")
      while (ss >> v) names.push_back(v);
    else if (key == "SIZE")
      while (ss >> v) sizes.push_back(std::stoi(v));
    else if (key == "TYPE")
      while (ss >> v) types.push_back(v);
    else if (key == "WIDTH")
      ss >> c.info.width;
    else if (key == "HEIGHT")
      ss >> c.info.height;
    else if (key == "DATA") {
      ss >> v;
      if (v != "binary") throw std::runtime_error("only binary PCD");
      break;
    }
  }
  uint32_t off = 0;
  for (size_t i = 0; i < names.size(); ++i) {
    PointField f;
    f.name = names[i];
    f.offset = off;
    const char t = types[i][0];
    const int s = sizes[i];
    f.type = t == 'F' ? (s == 4 ? FieldType::FLOAT32 : FieldType::FLOAT64)
             : t == 'U' ? (s == 1 ? FieldType::UINT8 : s == 2 ? FieldType::UINT16 : FieldType::UINT32)
                        : (s == 1 ? FieldType::INT8 : s == 2 ? FieldType::INT16 : FieldType::INT32);
    c.info.fields.push_back(f);
    off += s;
  }
  c.info.point_step = off;
  c.data.assign(raw.begin() + pos, raw.begin() + pos + size_t(off) * c.info.width * c.info.height);
  return c;
}

static Cloud loadCdr(const std::string& path) {
  auto raw = readFile(path);
  auto pc = cloudini_ros::getDeserializedPointCloudMessage(ConstBufferView(raw.data(), raw.size()));
  Cloud c;
  c.name = std::filesystem::path(path).filename().string();
  c.info.width = pc.width;
  c.info.height = pc.height;
  c.info.point_step = pc.point_step;
  for (auto f : pc.fields) {
    f.resolution.reset();
    c.info.fields.push_back(f);
  }
  c.data.assign(pc.data.data(), pc.data.data() + pc.data.size());
  return c;
}

//------------------------------------------------------------------------------------------

static double now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Result {
  size_t bytes = 0;
  double enc_s = 0, dec_s = 0;
};

int main(int argc, char** argv) {
  std::vector<Cloud> clouds;
  for (int i = 1; i < argc; ++i) {
    std::string p = argv[i];
    clouds.push_back(p.ends_with(".pcd") ? loadPcd(p) : loadCdr(p));
  }
  ZSTD_CCtx* cctx = ZSTD_createCCtx();
  ZSTD_DCtx* dctx = ZSTD_createDCtx();
  const int reps = 5;

  for (int version : {5, 6}) {
    std::cout << "\n=== V" << version << " (1 mm, refined) ===\n";
    std::cout << std::left << std::setw(22) << "cloud" << std::setw(16) << "stage2" << std::right << std::setw(10)
              << "bytes" << std::setw(9) << "%raw" << std::setw(9) << "%stg1" << std::setw(11) << "encMB/s"
              << std::setw(11) << "decMB/s" << "\n";
    for (auto& cloud : clouds) {
      EncodingInfo info = cloud.info;
      for (auto& f : info.fields)
        if (f.type == FieldType::FLOAT32) f.resolution = 0.001f;
      info.encoding_opt = EncodingOptions::LOSSY;
      info.compression_opt = CompressionOption::ZSTD;
      info.use_threads = false;
      info.version = uint8_t(version);
      const ConstBufferView view(cloud.data.data(), cloud.data.size());
      RefineResolutionsToData(info, view);

      std::vector<Stage1Chunk> chunks;
      g_capture = &chunks;
      std::vector<uint8_t> encoded;
      PointcloudEncoder(info).encode(view, encoded);
      g_capture = nullptr;

      size_t stage1 = 0, header_and_framing = encoded.size();
      for (auto& c : chunks) stage1 += c.data.size();
      std::vector<uint8_t> tmp;
      size_t zstd1 = 0;
      for (auto& c : chunks) zstd1 += zstdBlocks(c, 1, tmp, cctx);
      header_and_framing -= zstd1;  // header + u32 chunk sizes

      if (getenv("SECTIONS")) {
        std::cout << cloud.name << " V" << version << " fields:";
        for (auto& f : info.fields)
          std::cout << " " << f.name << "(" << int(f.type) << (f.resolution ? ",r=" + std::to_string(*f.resolution) : "") << ")";
        std::cout << "\n";
        for (size_t ci = 0; ci < chunks.size(); ++ci) {
          auto secs = sections(chunks[ci], true);
          for (size_t si = 0; si < secs.size(); ++si) {
            auto [s, e] = secs[si];
            Stage1Chunk one{std::vector<uint8_t>(chunks[ci].data.begin() + s, chunks[ci].data.begin() + e), {}};
            std::vector<uint8_t> z, r;
            zstdBlocks(one, 1, z, cctx);
            ransCompress(one, false, Mode::BestValRep, r);
            if (getenv("DUMP")) {
              std::ofstream(std::string(getenv("DUMP")) + "/" + cloud.name + ".v" + std::to_string(version) + ".c" +
                                std::to_string(ci) + ".s" + std::to_string(si),
                            std::ios::binary)
                  .write((const char*)one.data.data(), one.data.size());
            }
            std::cout << "  chunk " << ci << " sec " << si << ": " << (e - s) << " -> zstd1 " << z.size() << "  rans " << r.size() << "\n";
          }
        }
      }
      std::map<std::string, Result> res;
      std::vector<std::string> order;
      auto run = [&](const std::string& name, auto&& comp, auto&& decomp) {
        order.push_back(name);
        Result r;
        std::vector<std::vector<uint8_t>> outs(chunks.size());
        double best_e = 1e30, best_d = 1e30;
        for (int k = 0; k < reps; ++k) {
          double t0 = now();
          for (size_t i = 0; i < chunks.size(); ++i) comp(chunks[i], outs[i]);
          double t1 = now();
          for (size_t i = 0; i < chunks.size(); ++i) decomp(chunks[i], outs[i], k == 0);
          double t2 = now();
          best_e = std::min(best_e, t1 - t0);
          best_d = std::min(best_d, t2 - t1);
        }
        for (size_t i = 0; i < chunks.size(); ++i) r.bytes += outs[i].size();
        r.enc_s = best_e;
        r.dec_s = best_d;
        res[name] = r;
      };
      std::vector<uint8_t> dec(stage1 + 64);
      auto zstdDecomp = [&](const Stage1Chunk& c, const std::vector<uint8_t>& comp, bool check) {
        size_t n = ZSTD_decompressDCtx(dctx, dec.data(), dec.size(), comp.data(), comp.size());
        if (check && (n != c.data.size() || memcmp(dec.data(), c.data.data(), n) != 0))
          throw std::runtime_error("zstd mismatch");
      };
      for (int level : {1, 3, 9, 19}) {
        run("zstd-" + std::to_string(level), [&](const Stage1Chunk& c, std::vector<uint8_t>& o) { zstdBlocks(c, level, o, cctx); },
            zstdDecomp);
      }
      run(
          "lz4",
          [&](const Stage1Chunk& c, std::vector<uint8_t>& o) {
            o.resize(LZ4_compressBound(int(c.data.size())));
            o.resize(LZ4_compress_default((const char*)c.data.data(), (char*)o.data(), int(c.data.size()), int(o.size())));
          },
          [&](const Stage1Chunk& c, const std::vector<uint8_t>& comp, bool check) {
            int n = LZ4_decompress_safe((const char*)comp.data(), (char*)dec.data(), int(comp.size()), int(dec.size()));
            if (check && (size_t(n) != c.data.size() || memcmp(dec.data(), c.data.data(), n) != 0))
              throw std::runtime_error("lz4 mismatch");
          });
      struct RansVariant {
        const char* name;
        bool split;
        Mode mode;
      };
      for (auto v : {RansVariant{"rans-o0", false, Mode::O0}, RansVariant{"rans-o0/sec", true, Mode::O0},
                     RansVariant{"rans-vpos/sec", true, Mode::VPos}, RansVariant{"rans-best/sec", true, Mode::Best},
                     RansVariant{"rans-val/sec", true, Mode::BestVal},
                     RansVariant{"rans-valrep/sec", true, Mode::BestValRep}}) {
        std::vector<uint8_t> rdec;
        run(
            v.name,
            [&](const Stage1Chunk& c, std::vector<uint8_t>& o) {
              ransCompress(c, v.split, v.mode, o);
              // account for the section sizes a real format would store
              o.resize(o.size() + sectionSizeOverhead(c, v.split));
            },
            [&](const Stage1Chunk& c, const std::vector<uint8_t>& comp, bool check) {
              ransDecompress(comp, c, v.split, rdec);
              if (check && rdec != c.data) throw std::runtime_error(std::string(v.name) + " mismatch");
            });
      }

      const double raw = double(cloud.data.size());
      std::cout << std::left << std::setw(22) << cloud.name << std::setw(16) << "(stage 1)" << std::right
                << std::setw(10) << stage1 << std::setw(9) << std::fixed << std::setprecision(2)
                << 100.0 * stage1 / raw << std::setw(9) << 100.0 << "   chunks=" << chunks.size()
                << " sections=" << [&] {
                     size_t s = 0;
                     for (auto& c : chunks) s += sections(c, true).size();
                     return s;
                   }() << "\n";
      for (auto& name : order) {
        auto& r = res[name];
        const size_t total = r.bytes + header_and_framing;
        std::cout << std::left << std::setw(22) << "" << std::setw(16) << name << std::right << std::setw(10) << total
                  << std::setw(9) << 100.0 * total / raw << std::setw(9) << 100.0 * r.bytes / stage1 << std::setw(11)
                  << std::setprecision(0) << stage1 / 1e6 / r.enc_s << std::setw(11) << stage1 / 1e6 / r.dec_s
                  << std::setprecision(2) << "\n";
      }
    }
  }
  ZSTD_freeCCtx(cctx);
  ZSTD_freeDCtx(dctx);
  return 0;
}

