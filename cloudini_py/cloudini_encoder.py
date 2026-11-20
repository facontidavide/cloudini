#!/usr/bin/env python3
"""
Cloudini Encoder - encode a point cloud (structured numpy array) with the Cloudini WASM module.

The result is a Cloudini payload: the content of a .cldn file, or the `data` field of a
point_cloud_interfaces/msg/CompressedPointCloud2. CloudiniDecoder.decode_data reads it back.

Run `python cloudini_encoder.py <cloudini_wasm.wasm>` for a round-trip self-check.

Copyright 2025 Davide Faconti
Licensed under the Apache License, Version 2.0
"""

from __future__ import annotations

import sys

import numpy as np

from cloudini_decoder import NUMPY_TYPES, CloudiniDecoder

# numpy kind + size ('f4') -> Cloudini field type
_FIELD_TYPES = {np.dtype(dtype).str[1:]: name for name, dtype in NUMPY_TYPES.items()}
# FLOAT32 fields that hold packed color bits, never quantized: Cloudini::isPackedColorField (case-insensitive)
_PACKED_COLORS = {'rgb', 'rgba', 'bgr', 'bgra', 'argb', 'abgr'}


class CloudiniEncoder(CloudiniDecoder):
    """Encoder of point clouds to the Cloudini format, using the WASM module. Decodes too (decode_data)."""

    def __init__(self, wasm_path: str):
        super().__init__(wasm_path)
        if not self.encode_data:
            raise RuntimeError("cldn_EncodePointcloudData not found in the WASM module")

    def encode(self, points: np.ndarray, resolution: float = 0.001,
               field_resolutions: dict | None = None, version: int = 6) -> bytes:
        """
        Encode a point cloud.

        Args:
            points: structured array, one named field per PointCloud2 field (as CloudiniDecoder returns).
                A 2D array is an organized cloud (height, width).
            resolution: quantization step of the FLOAT32 fields (lossy: values decode within resolution/2),
                except packed colors such as 'rgb'. Other types are lossless.
            field_resolutions: per-field resolution overriding the default, e.g. {'intensity': 0.1};
                None keeps that field lossless.
            version: Cloudini encoding version, 4 to 6; 5 for readers on 1.3.1 or earlier.

        Returns:
            The Cloudini payload (bytes).
        """
        if points.dtype.names is None:
            raise ValueError("points must be a structured array (one named field per PointCloud2 field)")
        if points.ndim not in (1, 2) or points.size == 0:
            raise ValueError("points must be a non-empty 1D array, or a 2D array for an organized cloud")
        if version not in (4, 5, 6):
            raise ValueError(f"version must be 4, 5 or 6, not {version}")
        height, width = points.shape if points.ndim == 2 else (1, points.shape[0])
        field_resolutions = field_resolutions or {}
        unknown = set(field_resolutions) - set(points.dtype.names)
        if unknown:
            raise ValueError(f"field_resolutions names fields the cloud does not have: {sorted(unknown)}")

        yaml = [f"version: {version}", f"width: {width}", f"height: {height}",
                f"point_step: {points.dtype.itemsize}", "encoding_opt: LOSSY", "compression_opt: ZSTD", "fields:"]
        for name in points.dtype.names:
            if not name or name != name.strip() or any(c in name for c in '#\n\r'):
                raise ValueError(f"field name {name!r} cannot be stored in the Cloudini header")
            dt, offset = points.dtype.fields[name][:2]
            field_type = _FIELD_TYPES.get(f"{dt.kind}{dt.itemsize}")
            if field_type is None or dt.shape or dt.byteorder == '>':
                raise ValueError(f"field '{name}' has type {dt}: use one little-endian (u)int8/16/32 or float32/64 value")
            default = resolution if field_type == 'FLOAT32' and name.lower() not in _PACKED_COLORS else None
            res = field_resolutions.get(name, default)
            yaml += [f"  - name: {name}", f"    offset: {offset}", f"    type: {field_type}",
                     f"    resolution: {'null' if res is None else repr(float(res))}"]

        header = ("\n".join(yaml) + "\n").encode() + b"\0"
        data = np.ascontiguousarray(points).tobytes()

        header_ptr, data_ptr, output_ptr = self.allocate(len(header)), self.allocate(len(data)), self.allocate(len(data))
        try:
            self.write_bytes(header_ptr, header)
            self.write_bytes(data_ptr, data)
            size = self._call(self.encode_data, header_ptr, data_ptr, len(data), output_ptr)
            if size == 0:
                # ponytail: the export writes at most len(data) bytes; a cloud of a few points can encode larger
                raise RuntimeError("encoding failed: the encoded cloud is larger than the raw one (a cloud of a "
                                   "few points), which is not supported")
            return self.read_bytes(output_ptr, size)
        finally:
            for ptr in (header_ptr, data_ptr, output_ptr):
                self.deallocate(ptr)


def _self_check(wasm_path: str):
    rng = np.random.default_rng(1)
    n = 5000
    cloud = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                               ('ring', '<u2'), ('timestamp', '<f8')])
    for axis in 'xyz':
        cloud[axis] = rng.uniform(-50, 50, n)
    cloud['intensity'] = rng.integers(0, 255, n)
    cloud['ring'] = np.arange(n) % 32
    cloud['timestamp'] = 1.7e9 + np.arange(n) * 1e-6

    encoder = CloudiniEncoder(wasm_path)
    payload = encoder.encode(cloud, resolution=0.001)
    decoded = encoder.decode_data(payload)[0]

    assert decoded.dtype == cloud.dtype and decoded.shape == cloud.shape
    for axis in 'xyz':  # within resolution / 2, plus the float32 rounding of the decoded value
        tolerance = 0.0005 + np.spacing(np.abs(cloud[axis]).max())
        assert np.abs(decoded[axis].astype(np.float64) - cloud[axis]).max() <= tolerance, axis
    for exact in ('intensity', 'ring', 'timestamp'):  # integer-valued floats are refined to resolution 1
        assert np.array_equal(decoded[exact], cloud[exact]), exact
    large = np.zeros(262144, dtype=cloud.dtype)  # 6.8 MB, as a 128-beam lidar frame: the heap must grow
    large['x'] = np.linspace(-80, 80, large.size)
    assert np.array_equal(encoder.decode_data(encoder.encode(large))[0]['ring'], large['ring'])
    try:
        encoder.decode_data(b'not a cloudini payload' * 20)
        raise AssertionError("garbage decoded")
    except RuntimeError:
        pass
    organized = cloud.reshape(50, 100)
    assert encoder.decode_data(encoder.encode(organized))[0].shape == (n,)  # decoder returns the points, unshaped
    print(f"self-check passed: {cloud.nbytes} bytes -> {len(payload)} bytes ({len(payload) / cloud.nbytes:.1%})")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit("usage: cloudini_encoder.py <cloudini_wasm.wasm>")
    _self_check(sys.argv[1])
