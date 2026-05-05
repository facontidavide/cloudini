# Cloudini V5 Algorithm Analysis

## Overview

Cloudini V5 is a two-stage pointcloud codec.

1. Stage 1 applies field-aware preprocessing to the point data.
2. Stage 2 optionally compresses each encoded chunk with LZ4 or ZSTD.

The default V5 target is lossy pointcloud compression with millimeter-scale
float quantization and adaptive integer encoding. V5 keeps the fast float path
separate from the adaptive integer machinery, because float-heavy clouds are
already dominated by the optimized lossy float encoder.

The current magic/version prefix is `CLOUDINI_V05`.

## When V5 Is Used

The V5 adaptive path is used only when all of these are true:

- `EncodingInfo::version >= 5`
- `encoding_opt == EncodingOptions::LOSSY`
- At least one non-leading integer field can use the V5 adaptive integer modes

Float-only lossy clouds bypass the V5 adaptive machinery and use the regular
field encoders directly. This avoids paying adaptive setup and buffering costs
when there is no integer field that can benefit from it.

## Chunking

Pointcloud payloads are encoded in fixed chunks of `32 * 1024` points. Each
chunk has:

- A Stage 1 byte stream.
- An optional Stage 2 compressed representation.
- A chunk-size prefix when Stage 2 compression is enabled.

Chunking bounds scratch memory, lets the compressor run on completed chunks,
and gives the decoder an exact expected point count for each chunk.

## Stage 1 Layout

Within each V5 chunk, Stage 1 writes:

1. Regular field streams in point order.
2. V5 adaptive integer sections, appended once per adaptive field.

Regular fields are fields that are not handled by V5 adaptive integer encoding.
This includes the leading lossy float vector, non-adaptive floats, `INT8`,
`UINT8`, and any unsupported field type handled by the existing compatible
encoders.

Adaptive integer fields are collected or streamed separately so their complete
per-field section can use the best local representation.

## Lossy Float Encoding

Lossy float fields with a resolution are quantized to integers:

```text
quantized = round(value / resolution)
delta = quantized - previous_quantized
encoded = zigzag_varint(delta)
```

For common pointcloud schemas, the first 3 or 4 consecutive `FLOAT32` fields
with resolutions are encoded by `FieldEncoderFloatN_Lossy`. This path uses SIMD
for quantization and delta generation, and it has a fast no-NaN path.

NaN is represented with the reserved varint marker `0`. Real encoded deltas are
shifted by one so the marker remains available.

## Adaptive Integer Encoding

V5 adaptive integer encoding applies to these lossy-mode field types:

- `INT16`, `UINT16`
- `INT32`, `UINT32`
- `INT64`, `UINT64`

`INT8` and `UINT8` stay as raw byte copies. Their values are already one byte,
so adaptive varint or palette machinery would usually add overhead instead of
removing it.

Each adaptive integer field chooses one mode:

| Mode | Best For | Encoding |
|---|---|---|
| `DeltaVarint` | Smooth values with small deltas | Delta from previous value, then signed varint |
| `DeltaRle` | Repeated slopes or constant deltas | Runs of equal deltas, each as signed varint + run length |
| `Rle` | Repeated raw values | Raw value + run length |
| `Palette` | Low-cardinality fields | Unique raw values + bit-packed indexes |

### Mode Selection

For each adaptive integer field, V5 estimates all adaptive modes and commits the
smallest one.

- If the first chunk has more than `4096` points, V5 probes the first `4096`
  points, picks the mode, then streams the rest of the chunk using that mode.
- If the first chunk has `4096` points or fewer, V5 analyzes the whole chunk
  before writing the adaptive section.
- The selected mode is then reused for later chunks of the same pointcloud.

This keeps mode selection cheap and stable while still adapting to the field's
actual distribution.

### Committed Streaming

After a mode is committed, V5 does not store and rescan the whole chunk for
`DeltaVarint`, `DeltaRle`, or `Rle`. It builds the chosen section while walking
the input points, then appends that completed section after the regular field
stream.

`Palette` still needs the values for the chunk so it can build the unique-value
table and bit-packed indexes.

This preserves the V5 wire layout while removing avoidable per-chunk buffering
and second-pass work for the common non-palette modes.

## Stage 2 Compression

After Stage 1, each chunk can be passed through:

- `CompressionOption::NONE`
- `CompressionOption::LZ4`
- `CompressionOption::ZSTD`

The current benchmark default uses ZSTD level 1. The encoder can use a worker
thread and double buffering so one chunk can be compressed while the next chunk
is being encoded.

Scratch buffers are retained by capacity and are not zero-filled before each
chunk. Only the serialized byte range is passed to Stage 2.

## Decoding

Decoding reverses the chunk pipeline:

1. Parse the YAML or binary header.
2. For each chunk, optionally decompress Stage 2 into a reusable buffer.
3. Decode regular field streams point by point.
4. Decode each adaptive integer section into the correct output field offsets.

The decompression buffer keeps its capacity between chunks. The decoder returns
views sized to the actual decompressed chunk instead of shrinking the backing
storage after every chunk.

## Benchmarking

Use `mcap_codec_benchmark` for local ratio and speed checks:

```bash
./build_release/tools/mcap_codec_benchmark DATA/<file>.mcap --max-messages 1000 --zstd --mode V5
```

Timing is benchmark-internal codec timing. Encode includes Cloudini Stage 1 and
optional Cloudini Stage 2 compression. Decode includes optional Cloudini Stage 2
decompression and Cloudini Stage 1 decode.

MCAP read/decompression time should not be interpreted as codec time.

## Validation Commands

```bash
cmake --build build_release --target mcap_codec_benchmark -j 8
ctest --test-dir build_release --output-on-failure -j 8
git diff --check
```

For decode-only profiling without MCAP read/decompression in the measured
region:

```bash
./build_release/tools/mcap_codec_benchmark DATA/<file>.mcap \
  --max-messages 1000 --zstd --mode V5 --decode-replay --decode-repeat 5
```

`--decode-replay` first reads and encodes samples, stores the encoded Cloudini
payloads in memory, and only then times repeated Cloudini decode operations.
