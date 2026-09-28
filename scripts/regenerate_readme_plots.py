#!/usr/bin/env python3
"""Regenerate the README charts compression_ratio.svg and compression_speed.svg.

Cloudini with the default settings (V6, 1 mm on FLOAT32 fields, refined to the data, ZSTD) against ZSTD
level 1 (the level Cloudini uses) on the raw cloud. The clouds are raw frames under DATA/v6_bench/<id>/
(frame_*.bin plus a meta.json with name, point_step, fields and frames; not in git) and the Hesai sample
in cloudini_lib/samples. scripts/readme_bench.cpp is built against build_release (build cloudini_lib
first) and run twice pinned to one core; speeds are the best of the two runs.

    python3 scripts/regenerate_readme_plots.py [--cpu 8]
"""
import argparse, json, os, subprocess, sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(REPO, "build_release")
DATA = os.path.join(REPO, "DATA", "v6_bench")

ap = argparse.ArgumentParser()
ap.add_argument("--cpu", default="8", help="core to pin the benchmark to")
args = ap.parse_args()

manifest = os.path.join(BUILD, "readme_bench_manifest.txt")
with open(manifest, "w") as f:
    for d in sorted(os.listdir(DATA)):
        meta = os.path.join(DATA, d, "meta.json")
        if not os.path.exists(meta):
            continue
        j = json.load(open(meta))
        f.write(f"D {d} {j['point_step']} {j['name']}\n")
        for fl in j["fields"]:
            f.write(f"F {fl['offset']} {fl['type']} {fl['name']}\n")
        for fr in j["frames"]:
            f.write(f"C {fr['width']} {fr['height']} {os.path.join(DATA, d, fr['file'])}\n")
    f.write(f"H hesai {os.path.join(REPO, 'cloudini_lib', 'samples', 'dds_message.bin')} Hesai sample\n")

exe = os.path.join(BUILD, "readme_bench")
subprocess.run(["g++", "-O3", "-DNDEBUG", "-std=gnu++20", "-I" + os.path.join(REPO, "cloudini_lib", "include"),
                "-I" + os.path.join(BUILD, "_deps", "zstd-src", "lib"), os.path.join(REPO, "scripts", "readme_bench.cpp"),
                "-o", exe] + [os.path.join(BUILD, l) for l in ("libcloudini_lib.a", "libcloudini_lz4.a", "libcloudini_zstd.a")]
               + ["-lpthread", "-ldl"], check=True)
runs = []
for _ in range(2):
    out = subprocess.run(["taskset", "-c", args.cpu, exe, manifest], check=True, capture_output=True, text=True).stdout
    runs.append({(d := json.loads(l))["id"]: d for l in out.splitlines()})
data = {}
for k in runs[0]:
    d = runs[0][k]
    for m in ("zstd", "cloudini"):
        for s in ("enc", "dec"):
            d[m][s] = max(r[k][m][s] for r in runs)
    data[k] = d

ROWS = [  # id, sensor, source (in chart order)
    ("ouster_os0_32", "Ouster OS0-32", "Ouster SDK sample"),
    ("ouster_os0_128", "Ouster OS0-128", "Ouster SDK sample"),
    ("ouster_os1_32", "Ouster OS1-32", "Ouster SDK sample"),
    ("ouster_os1_64", "Ouster OS1-64", "Ouster SDK sample"),
    ("ouster_os1_128", "Ouster OS1-128", "Ouster SDK sample"),
    ("ouster_os2_32", "Ouster OS2-32", "Ouster SDK sample"),
    ("ouster_os2_128", "Ouster OS2-128", "Ouster SDK sample"),
    ("kitti", "Velodyne HDL-64E", "KITTI"),
    ("nuscenes", "Velodyne HDL-32E", "nuScenes"),
    ("argoverse2", "2× Velodyne VLP-32C", "Argoverse 2"),
    ("hesai", "Hesai, 32 channels", "cloudini sample"),
    ("pcd_sample", "LiDAR, xyz + intensity", "cloudini PCD sample"),
    ("stereo", "Stereo camera, RGB", "PCL test data"),
]

STYLE = """<style>
  text { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif; fill: #1f2328; }
  .src { font-size: 11px; fill: #6b7480; }
  .model { font-size: 13px; font-weight: 600; }
  .val, .tick { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; font-size: 11px; fill: #4a5360; }
  .tick { fill: #6b7480; }
  .fac { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; font-size: 12px; font-weight: 600; fill: #148a60; }
  .title { font-size: 15px; font-weight: 600; }
  .sub, .key { font-size: 12.5px; fill: #4a5360; }
  .grid { stroke: #e6eaee; } .axis { stroke: #b9c1ca; } .sep { stroke: #d9dee3; }
  .z { fill: #2a78d6; } .c { fill: #1baf7a; }
  @media (prefers-color-scheme: dark) {
    text { fill: #e6edf3; } .src, .tick { fill: #8b949e; } .val, .sub, .key { fill: #c3c9d0; }
    .fac { fill: #3ccf98; }
    .grid { stroke: #21262d; } .axis { stroke: #3d444d; } .sep { stroke: #30363d; }
    .z { fill: #3987e5; } .c { fill: #199e70; }
  }
</style>"""

def esc(s): return s.replace("&", "&amp;").replace("<", "&lt;")
def bar(x, y, w, h, cls, r=3):
    r = min(r, w, h / 2)
    return (f'<path class="{cls}" d="M{x:.1f},{y:.1f}H{x+w-r:.1f}Q{x+w:.1f},{y:.1f} {x+w:.1f},{y+r:.1f}'
            f'V{y+h-r:.1f}Q{x+w:.1f},{y+h:.1f} {x+w-r:.1f},{y+h:.1f}H{x:.1f}Z"/>')
def nice(v, n=5):
    raw = v / n; p = 10 ** len(str(int(raw))) / 10 if raw >= 1 else 1
    for s in (1, 2, 2.5, 5, 10):
        if s * p >= raw: return s * p
    return 10 * p

LABEL_W, BAR_H, GAP, ROW_H = 170, 9, 3, 40

def header(out, w, title, sub, keys_y):
    out.append(f'<text class="title" x="0" y="18">{esc(title)}</text>')
    out.append(f'<text class="sub" x="0" y="37">{esc(sub)}</text>')
    x = 0
    for cls, name in (("z", "ZSTD alone"), ("c", "Cloudini + ZSTD")):
        out.append(f'<rect class="{cls}" x="{x}" y="{keys_y-10}" width="12" height="12" rx="3"/>')
        out.append(f'<text class="key" x="{x+18}" y="{keys_y}">{name}</text>')
        x += 150

def panel(out, x0, y0, width, metric, unit, digits, rows_top_labels=True):
    vmax = max(max(data[i]["zstd"][metric], data[i]["cloudini"][metric]) for i, _, _ in ROWS)
    step = nice(vmax); ticks = int(-(-vmax // step)); vmax = ticks * step
    scale = (width - 44) / vmax
    bottom = y0 + len(ROWS) * ROW_H
    for t in range(ticks + 1):
        x = x0 + t * step * scale
        out.append(f'<line class="{"axis" if t == 0 else "grid"}" x1="{x:.1f}" x2="{x:.1f}" y1="{y0-6}" y2="{bottom}"/>')
        lab = f"{t*step:g}" + (f" {unit}" if t == ticks and unit else "")
        out.append(f'<text class="tick" x="{x:.1f}" y="{bottom+16}" text-anchor="{"end" if t == ticks else "middle"}">{lab}</text>')
    for k, (i, _, _) in enumerate(ROWS):
        y = y0 + k * ROW_H + (ROW_H - 2 * BAR_H - GAP) / 2 - 4
        for j, (m, cls) in enumerate((("zstd", "z"), ("cloudini", "c"))):
            v = data[i][m][metric]; by = y + j * (BAR_H + GAP)
            out.append(bar(x0, by, v * scale, BAR_H, cls))
            out.append(f'<text class="val" x="{x0 + v*scale + 5:.1f}" y="{by+BAR_H-1}">{v:.{digits}f}</text>')
    return bottom

def labels(out, y0):
    for k, (_, model, src) in enumerate(ROWS):
        y = y0 + k * ROW_H
        if k and ROWS[k-1][1].split()[0] != model.split()[0] and not model.startswith("2×"):
            out.append(f'<line class="sep" x1="0" x2="{LABEL_W-14}" y1="{y-4}" y2="{y-4}"/>')
        out.append(f'<text class="model" x="{LABEL_W-14}" y="{y+13}" text-anchor="end">{esc(model)}</text>')
        out.append(f'<text class="src" x="{LABEL_W-14}" y="{y+27}" text-anchor="end">{esc(src)}</text>')

def svg(name, w, h, body):
    open(os.path.join(REPO, name), "w").write(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">\n{STYLE}\n' + "\n".join(body) + "\n</svg>\n")

# size
W = 880; top = 78; out = []
header(out, W, "Compressed size, % of the original cloud (smaller is better)",
       "Cloudini: V6, 1 mm resolution on float fields, refined to the data, then ZSTD. ZSTD alone: level 1 on the raw cloud.", 60)
labels(out, top)
bottom = panel(out, LABEL_W, top, W - LABEL_W - 70, "ratio", "%", 1)
out.append(f'<text class="tick" x="{W}" y="{top-10}" text-anchor="end">vs ZSTD</text>')
for k, (i, _, _) in enumerate(ROWS):
    f = data[i]["zstd"]["ratio"] / data[i]["cloudini"]["ratio"]
    out.append(f'<text class="fac" x="{W}" y="{top + k*ROW_H + 19}" text-anchor="end">{f:.1f}× smaller</text>')
svg("compression_ratio.svg", W, bottom + 24, out)

# speed: encode and decode panels
W = 880; top = 100; out = []; pw = (W - LABEL_W - 40) / 2
header(out, W, "Throughput, MB/s of original cloud (larger is faster)",
       "One core (pinned), same clouds as above; Cloudini includes its ZSTD stage.", 60)
labels(out, top)
for n, (metric, t) in enumerate((("enc", "Encode, MB/s"), ("dec", "Decode, MB/s"))):
    x0 = LABEL_W + n * (pw + 40)
    out.append(f'<text class="model" x="{x0}" y="{top-14}">{t}</text>')
    bottom = panel(out, x0, top, pw, metric, "", 0)
svg("compression_speed.svg", W, bottom + 24, out)
print("Wrote compression_ratio.svg and compression_speed.svg")
