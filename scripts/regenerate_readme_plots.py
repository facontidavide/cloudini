#!/usr/bin/env python3
"""Regenerate README compression-ratio + compression-time PNGs.

For each .mcap in DATA/:
  * ZSTD-only baseline -- ZSTD-3 compress the raw PointCloud2 .data payload
    in Python, accumulate compressed size and wall-clock time.
  * Cloudini-V5 + ZSTD -- parse `mcap_codec_benchmark --zstd --mode V5`
    output for total compressed size and encode throughput.

Outputs compression_ratio.png and compression_time.png at repo root.
"""

from __future__ import annotations

import re
import subprocess
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import zstandard as zstd
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


REPO_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = REPO_ROOT / "DATA"
BENCH = REPO_ROOT / "build_release" / "tools" / "mcap_codec_benchmark"
MAX_MESSAGES = 50  # per topic, matches a typical README sample run
ZSTD_LEVEL = 3


def discover_bags() -> list[Path]:
    bags = sorted(p for p in DATA_DIR.glob("*.mcap") if "_encoded" not in p.name)
    if not bags:
        raise SystemExit(f"No source mcaps in {DATA_DIR}")
    return bags


def measure_zstd_only(bag: Path) -> dict:
    """Encode + decode each PointCloud2 .data payload with ZSTD-3 in Python.

    Returns totals across up to MAX_MESSAGES messages per topic.
    """
    raw_total = 0
    comp_total = 0
    enc_elapsed = 0.0
    dec_elapsed = 0.0
    msg_total = 0
    per_topic_count: dict[str, int] = {}
    cctx = zstd.ZstdCompressor(level=ZSTD_LEVEL)
    dctx = zstd.ZstdDecompressor()

    with open(bag, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, _msg, ros_msg in reader.iter_decoded_messages():
            if not schema or "PointCloud2" not in schema.name:
                continue
            count = per_topic_count.get(channel.topic, 0)
            if count >= MAX_MESSAGES:
                continue
            per_topic_count[channel.topic] = count + 1
            msg_total += 1

            payload = bytes(ros_msg.data)
            raw_total += len(payload)
            t0 = time.perf_counter()
            comp = cctx.compress(payload)
            enc_elapsed += time.perf_counter() - t0
            comp_total += len(comp)
            t1 = time.perf_counter()
            dctx.decompress(comp)
            dec_elapsed += time.perf_counter() - t1

    return {
        "raw_bytes": raw_total,
        "comp_bytes": comp_total,
        "enc_s": enc_elapsed,
        "dec_s": dec_elapsed,
        "messages": msg_total,
    }


_RE_V5 = re.compile(
    r"^\s*V5\s+([\d.]+)\s+([\d.]+)%\s+(\d+)\s+(\d+)\s*$", re.M
)
_RE_RAW = re.compile(r"raw=([\d.]+)\s*MiB")
_RE_MSGS = re.compile(r"messages=(\d+)\s+points=")


def measure_cloudini(bag: Path) -> dict[str, float]:
    """Run mcap_codec_benchmark and parse the V5 row.

    The tool prints one block per PointCloud2 topic. We sum across topics
    so multi-topic bags (e.g. nav_from_dock with 4 lidars) aggregate
    correctly.
    """
    cmd = [
        str(BENCH), str(bag),
        "--max-messages", str(MAX_MESSAGES),
        "--zstd",
    ]
    out = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout

    raw_mib_total = sum(float(m) for m in _RE_RAW.findall(out))
    if raw_mib_total == 0:
        raise RuntimeError(f"No PointCloud2 topic parsed from {bag.name}")

    out_mib_total = 0.0
    enc_mb_s_weighted = 0.0  # weight by raw MiB so a topic with more data dominates
    raw_blocks = _RE_RAW.findall(out)
    v5_rows = _RE_V5.findall(out)
    if len(raw_blocks) != len(v5_rows):
        raise RuntimeError(f"Block/row mismatch parsing {bag.name}: "
                           f"{len(raw_blocks)} raw / {len(v5_rows)} V5")
    for raw_str, (out_mib, _ratio, enc_mb_s, _dec_mb_s) in zip(raw_blocks, v5_rows):
        raw_mib = float(raw_str)
        out_mib_total += float(out_mib)
        enc_mb_s_weighted += float(enc_mb_s) * raw_mib

    enc_mb_s_avg = enc_mb_s_weighted / raw_mib_total
    raw_bytes_total = int(raw_mib_total * (1 << 20))
    comp_bytes_total = int(out_mib_total * (1 << 20))
    # encode throughput is MB/s of raw input (decimal MB per the tool); convert
    # to wall seconds by dividing total raw decimal-MB by MB/s.
    elapsed = (raw_bytes_total / 1e6) / enc_mb_s_avg

    # Decode side: weight Dec MB/s by raw MiB the same way as Enc.
    dec_mb_s_weighted = 0.0
    for raw_str, (_o, _r, _e, dec_mb_s) in zip(raw_blocks, v5_rows):
        dec_mb_s_weighted += float(dec_mb_s) * float(raw_str)
    dec_mb_s_avg = dec_mb_s_weighted / raw_mib_total
    dec_elapsed = (raw_bytes_total / 1e6) / dec_mb_s_avg

    msg_total = sum(int(m) for m in _RE_MSGS.findall(out))
    return {
        "raw_bytes": raw_bytes_total,
        "comp_bytes": comp_bytes_total,
        "enc_s": elapsed,
        "dec_s": dec_elapsed,
        "messages": msg_total,
    }


def main() -> None:
    if not BENCH.exists():
        raise SystemExit(f"{BENCH} not found - build cloudini first")

    bags = discover_bags()
    rows = []
    for bag in bags:
        print(f"=== {bag.name} ===")
        z = measure_zstd_only(bag)
        c = measure_cloudini(bag)
        # Throughput in MB/s of *raw* input -- higher is better. Same denominator
        # convention as mcap_codec_benchmark so cross-checking lines up.
        z_enc_mbs = (z["raw_bytes"] / 1e6) / max(z["enc_s"], 1e-9)
        z_dec_mbs = (z["raw_bytes"] / 1e6) / max(z["dec_s"], 1e-9)
        c_enc_mbs = (c["raw_bytes"] / 1e6) / max(c["enc_s"], 1e-9)
        c_dec_mbs = (c["raw_bytes"] / 1e6) / max(c["dec_s"], 1e-9)
        rows.append({
            "bag": bag.stem,
            "raw_bytes_zstd": z["raw_bytes"],
            "zstd_only_bytes": z["comp_bytes"],
            "zstd_only_enc_mbs": z_enc_mbs,
            "zstd_only_dec_mbs": z_dec_mbs,
            "raw_bytes_cloudini": c["raw_bytes"],
            "cloudini_bytes": c["comp_bytes"],
            "cloudini_enc_mbs": c_enc_mbs,
            "cloudini_dec_mbs": c_dec_mbs,
        })
        print(f"  ZSTD-only:    {z['comp_bytes']/z['raw_bytes']:.1%} ratio, "
              f"enc {z_enc_mbs:.0f} MB/s / dec {z_dec_mbs:.0f} MB/s")
        print(f"  Cloudini+ZSTD: {c['comp_bytes']/c['raw_bytes']:.1%} ratio, "
              f"enc {c_enc_mbs:.0f} MB/s / dec {c_dec_mbs:.0f} MB/s")

    rows.sort(key=lambda r: r["zstd_only_bytes"] / r["raw_bytes_zstd"])
    print("\nOrdered by ZSTD-only ratio (ascending):")
    for i, r in enumerate(rows, 1):
        r["name"] = f"sample{i}"
        ratio = r["zstd_only_bytes"] / r["raw_bytes_zstd"]
        print(f"  {r['name']:8s} <- {r['bag']:18s} ZSTD ratio {ratio:.1%}")

    plot_ratio(rows)
    plot_time(rows)
    print("\nWrote compression_ratio.png and compression_time.png")


def plot_ratio(rows: list[dict]) -> None:
    names = [r["name"] for r in rows]
    zstd_only = [r["zstd_only_bytes"] / r["raw_bytes_zstd"] for r in rows]
    cloudini = [r["cloudini_bytes"] / r["raw_bytes_cloudini"] for r in rows]
    original = [1.0] * len(rows)

    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(11, 6.5),
                                         gridspec_kw={"height_ratios": [1, 1]})

    x = np.arange(len(names))
    w = 0.27

    ax_top.bar(x - w, original, width=w, label="Original size", color="#f5c518")
    ax_top.bar(x,     zstd_only, width=w, label="ZSTD only",    color="#3870c1")
    ax_top.bar(x + w, cloudini,  width=w, label="Cloudini+ZSTD", color="#d8362a")
    ax_top.set_xticks(x); ax_top.set_xticklabels(names)
    ax_top.set_ylim(0, 1.1)
    ax_top.set_yticks(np.arange(0, 1.01, 0.25))
    ax_top.legend(loc="upper center", ncol=3, frameon=False,
                  bbox_to_anchor=(0.5, 1.12))
    ax_top.grid(axis="y", linestyle="-", linewidth=0.5, color="#dddddd")
    ax_top.set_axisbelow(True)
    ax_top.spines["top"].set_visible(False)
    ax_top.spines["right"].set_visible(False)

    # zoomed view: just the two non-trivial bars
    w2 = 0.35
    ax_bot.bar(x - w2/2, zstd_only, width=w2, label="ZSTD only",    color="#3870c1")
    ax_bot.bar(x + w2/2, cloudini,  width=w2, label="Cloudini+ZSTD", color="#d8362a")
    ax_bot.set_xticks(x); ax_bot.set_xticklabels(names)
    ymax = max(max(zstd_only), max(cloudini)) * 1.25
    ax_bot.set_ylim(0, ymax)
    ax_bot.legend(loc="upper center", ncol=2, frameon=False,
                  bbox_to_anchor=(0.5, 1.12))
    ax_bot.grid(axis="y", linestyle="-", linewidth=0.5, color="#dddddd")
    ax_bot.set_axisbelow(True)
    ax_bot.spines["top"].set_visible(False)
    ax_bot.spines["right"].set_visible(False)

    fig.tight_layout()
    fig.savefig(REPO_ROOT / "compression_ratio.png", dpi=130,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)


def plot_time(rows: list[dict]) -> None:
    names = [r["name"] for r in rows]
    enc_zstd = [r["zstd_only_enc_mbs"] for r in rows]
    enc_cloud = [r["cloudini_enc_mbs"] for r in rows]

    fig, ax = plt.subplots(figsize=(11, 3.8))
    x = np.arange(len(names))
    w = 0.35
    ax.bar(x - w/2, enc_zstd,  width=w, label="ZSTD only",     color="#3870c1")
    ax.bar(x + w/2, enc_cloud, width=w, label="Cloudini+ZSTD", color="#d8362a")
    ax.set_xticks(x); ax.set_xticklabels(names)
    ax.set_title("Compression throughput (MB/s, higher is better)",
                 loc="left", fontsize=11, color="#444")
    ax.legend(loc="upper center", ncol=2, frameon=False,
              bbox_to_anchor=(0.5, 1.18))
    ax.grid(axis="y", linestyle="-", linewidth=0.5, color="#dddddd")
    ax.set_axisbelow(True)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout()
    fig.savefig(REPO_ROOT / "compression_time.png", dpi=130,
                bbox_inches="tight", facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    main()
