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

from cloudini_decoder import CloudiniDecoder

# numpy kind + size -> Cloudini field type
_FIELD_TYPES = {
    'i1': 'INT8', 'u1': 'UINT8', 'i2': 'INT16', 'u2': 'UINT16',
    'i4': 'INT32', 'u4': 'UINT32', 'f4': 'FLOAT32', 'f8': 'FLOAT64',
}
# FLOAT32 fields that hold packed color bits: never quantized (as in the C++ defaultFieldResolution)
_PACKED_COLORS = {'rgb', 'rgba', 'bgr', 'bgra', 'argb', 'abgr'}


class CloudiniEncoder:
    """Encoder of point clouds to the Cloudini format, using the WASM module."""

    def __init__(self, wasm_path: str):
        self._wasm = CloudiniDecoder(wasm_path)
        self._encode = self._wasm.exports.get('cldn_EncodePointcloudData')
        if not self._encode:
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
            version: Cloudini encoding version; 5 for readers on 1.3.1 or earlier.

        Returns:
            The Cloudini payload (bytes).
        """
        if points.dtype.names is None:
            raise ValueError("points must be a structured array (one named field per PointCloud2 field)")
        if points.ndim not in (1, 2) or points.size == 0:
            raise ValueError("points must be a non-empty 1D array, or a 2D array for an organized cloud")
        height, width = points.shape if points.ndim == 2 else (1, points.shape[0])
        field_resolutions = field_resolutions or {}

        yaml = [f"version: {version}", f"width: {width}", f"height: {height}",
                f"point_step: {points.dtype.itemsize}", "encoding_opt: LOSSY", "compression_opt: ZSTD", "fields:"]
        for name in points.dtype.names:
            dt, offset = points.dtype.fields[name][:2]
            field_type = _FIELD_TYPES.get(f"{dt.kind}{dt.itemsize}")
            if field_type is None or dt.shape or dt.byteorder == '>':
                raise ValueError(f"field '{name}' has type {dt}: use one little-endian (u)int8/16/32 or float32/64 value")
            default = resolution if field_type == 'FLOAT32' and name not in _PACKED_COLORS else None
            res = field_resolutions.get(name, default)
            yaml += [f"  - name: {name}", f"    offset: {offset}", f"    type: {field_type}",
                     f"    resolution: {'null' if res is None else repr(float(res))}"]

        header = ("\n".join(yaml) + "\n").encode() + b"\0"
        data = np.ascontiguousarray(points).tobytes()

        w = self._wasm
        header_ptr, data_ptr, output_ptr = w.allocate(len(header)), w.allocate(len(data)), w.allocate(len(data))
        try:
            w.write_bytes(header_ptr, header)
            w.write_bytes(data_ptr, data)
            size = self._encode(w.store, header_ptr, data_ptr, len(data), output_ptr)
            if size == 0:
                # ponytail: the export writes at most len(data) bytes; a cloud of a few points can encode larger
                raise RuntimeError("encoding failed (see the error printed above); clouds of a few points can "
                                   "encode to more bytes than their raw size, which is not supported")
            return w.read_bytes(output_ptr, size)
        finally:
            for ptr in (header_ptr, data_ptr, output_ptr):
                w.deallocate(ptr)

    def decode(self, payload: bytes) -> np.ndarray:
        """Decode a Cloudini payload (see CloudiniDecoder.decode_data)."""
        return self._wasm.decode_data(payload)[0]


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
    decoded = encoder.decode(payload)

    assert decoded.dtype == cloud.dtype and decoded.shape == cloud.shape
    for axis in 'xyz':  # within resolution / 2, plus the float32 rounding of the decoded value
        tolerance = 0.0005 + np.spacing(np.abs(cloud[axis]).max())
        assert np.abs(decoded[axis].astype(np.float64) - cloud[axis]).max() <= tolerance, axis
    for exact in ('intensity', 'ring', 'timestamp'):  # integer-valued floats are refined to resolution 1
        assert np.array_equal(decoded[exact], cloud[exact]), exact
    organized = cloud.reshape(50, 100)
    assert encoder.decode(encoder.encode(organized)).shape == (n,)  # decoder returns the points, unshaped
    print(f"self-check passed: {cloud.nbytes} bytes -> {len(payload)} bytes ({len(payload) / cloud.nbytes:.1%})")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit("usage: cloudini_encoder.py <cloudini_wasm.wasm>")
    _self_check(sys.argv[1])
