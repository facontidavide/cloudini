#!/usr/bin/env python3
"""
Cloudini Decoder - A reusable class for decoding Cloudini-compressed point clouds.

This module provides a CloudiniDecoder class that can decode CompressedPointCloud2
DDS messages into numpy arrays using a WebAssembly module.

Copyright 2025 Davide Faconti
Licensed under the Apache License, Version 2.0
"""

import numpy as np
from wasmtime import Store, Module, Linker, WasiConfig, Func, Engine, Config


class CloudiniDecoder:
    """Decoder for Cloudini-compressed point clouds using WASM."""

    def __init__(self, wasm_path: str):
        """
        Initialize the decoder with a WASM module.

        Args:
            wasm_path: Path to the cloudini WASM module
        """
        # Load the WASM module
        print(f"Loading WASM module from {wasm_path}")
        with open(wasm_path, 'rb') as f:
            wasm_bytes = f.read()

        # Create wasmtime engine with exception handling enabled
        config = Config()
        config.wasm_exceptions = True  # Enable WASM exception handling
        self.engine = Engine(config)

        # Create WASI configuration (required for Emscripten/WASI-compiled modules)
        wasi_config = WasiConfig()
        wasi_config.inherit_env()
        wasi_config.inherit_stdin()
        wasi_config.inherit_stdout()
        wasi_config.inherit_stderr()

        self.store = Store(self.engine)
        self.store.set_wasi(wasi_config)

        # Compile the module
        module = Module(self.engine, wasm_bytes)

        # Create a linker to handle imports
        linker = Linker(self.engine)

        # Add WASI functions to the linker
        linker.define_wasi()

        # Add stubs for C++ exception handling and other imports
        for import_ in module.imports:
            module_name = import_.module
            # Skip wasi_snapshot_preview1 as it's handled by define_wasi()
            if module_name != "wasi_snapshot_preview1":
                # Create stub functions for C++ runtime imports
                func_type = import_.type

                def make_stub(func_type):
                    def stub(*_):
                        if len(func_type.results) > 0:
                            # Return appropriate default based on result type
                            result_type = str(func_type.results[0])
                            if 'f32' in result_type or 'f64' in result_type:
                                return 0.0  # Float
                            return 0  # Integer
                        return None
                    return stub

                stub_func = make_stub(func_type)
                func = Func(self.store, func_type, stub_func)
                linker.define(self.store, module_name, import_.name, func)

        # Instantiate with linker
        self.instance = linker.instantiate(self.store, module)

        # Get exports
        exports = self.instance.exports(self.store)

        # List all exports with their types
        export_info = {}
        for name, obj in exports.items():
            export_info[name] = type(obj).__name__
        # print(f"Available exports with types: {export_info}")

        # Get memory export - try by name first, then by type
        self.memory = exports.get("memory")
        if not self.memory:
            # Find memory by type
            from wasmtime import Memory as MemoryType
            for name, obj in exports.items():
                if isinstance(obj, MemoryType):
                    self.memory = obj
                    print(f"Found memory export as '{name}'")
                    break

        if not self.memory:
            raise RuntimeError(f"Could not find memory export. Available: {list(export_info.keys())}")

        # Get malloc/free (may not exist in all modules)
        self.malloc = exports.get("malloc")
        self.free = exports.get("free")

        # Get the cloudini functions
        self.decode_compressed_msg = exports.get('cldn_DecodeCompressedMessage')
        self.get_header_as_yaml = exports.get('cldn_GetHeaderAsYAMLFromDDS')

        if not all([self.decode_compressed_msg, self.get_header_as_yaml]):
            raise RuntimeError("Could not find required Cloudini functions in WASM module")

        # Initialize WASM module (call constructors)
        init_func = exports.get("__wasm_call_ctors")
        if init_func:
            init_func(self.store)

        # Simple allocator offset (start at 2MB to avoid stack/heap)
        self.alloc_offset = 2 * 1024 * 1024

        self.output_ptr = self.allocate(32 * 1024 * 1024) # allocate 32 megabytes as worst case scenario

        print("WASM module loaded successfully!")

    def allocate(self, size: int) -> int:
        """Simple allocation strategy - just increment offset."""
        if self.malloc:
            return self.malloc(self.store, size)
        else:
            # Fallback: simple bump allocator
            ptr = self.alloc_offset
            self.alloc_offset += size + 16  # Add padding
            return ptr

    def deallocate(self, ptr: int):
        """Deallocate memory if free is available."""
        if self.free:
            self.free(self.store, ptr)

    def write_bytes(self, ptr: int, data: bytes):
        """Write bytes to WASM memory."""
        self.memory.write(self.store, data, ptr)

    def read_bytes(self, ptr: int, size: int) -> bytes:
        """Read bytes from WASM memory."""
        return self.memory.read(self.store, ptr, ptr + size)

    def decode_message(self, compressed_msg: bytes, verbose: bool = True) -> tuple[np.ndarray, dict]:
        """
        Decode a compressed DDS message to raw point cloud data.

        Args:
            compressed_msg: Raw DDS message containing CompressedPointCloud2
            verbose: Whether to print progress messages

        Returns:
            Tuple of (numpy array of point cloud data, header info dict)
        """
        # Allocate memory for input message
        input_ptr = self.allocate(len(compressed_msg))
        if input_ptr == 0:
            raise RuntimeError("Failed to allocate memory for input message")

        try:
            # Write compressed message to WASM memory
            self.write_bytes(input_ptr, compressed_msg)


            # Convert compressed message to PointCloud2 message
            actual_size = self.decode_compressed_msg(self.store, input_ptr, len(compressed_msg), self.output_ptr)

            if actual_size == 0:
                raise RuntimeError("Failed to convert compressed message to PointCloud2")

            # Read the decoded PointCloud2 DDS message
            points_msg_data = self.read_bytes(self.output_ptr, actual_size)

            # Get actual header info from the compressed message
            # This is mandatory - we need correct dimensions to decode properly
            try:
                header_info = self.get_header_info(compressed_msg)
            except Exception as e:
                raise RuntimeError(f"Failed to extract header info from compressed message: {e}")

            if not header_info or 'width' not in header_info:
                raise RuntimeError("Header info extraction returned incomplete data (missing 'width')")

            # Parse the PointCloud2 message to extract the point cloud data
            # The PointCloud2 message contains the data field with raw point cloud bytes
            point_cloud = self.extract_data_from_pc2_msg(points_msg_data, header_info)

            return point_cloud, header_info

        finally:
            self.deallocate(input_ptr)

    @staticmethod
    def _parse_yaml_value(value: str):
        """Convert a YAML scalar string to int, float, None, or str."""
        if value == 'null' or value == 'None':
            return None
        try:
            return int(value)
        except ValueError:
            pass
        try:
            return float(value)
        except ValueError:
            pass
        return value

    def get_header_info(self, compressed_msg: bytes) -> dict:
        """
        Extract header information from compressed message.

        Args:
            compressed_msg: Raw DDS message

        Returns:
            Dictionary with header info including 'fields' list
        """
        # Allocate memory for input and output
        input_ptr = self.allocate(len(compressed_msg))
        yaml_buffer_size = 4096  # Should be enough for YAML header
        yaml_ptr = self.allocate(yaml_buffer_size)

        try:
            self.write_bytes(input_ptr, compressed_msg)

            # Get header as YAML
            yaml_size = self.get_header_as_yaml(self.store, input_ptr, len(compressed_msg), yaml_ptr)
            if yaml_size == 0:
                return {}

            yaml_str = self.read_bytes(yaml_ptr, yaml_size).decode('utf-8')

            #print(f"Extracted YAML header:\n{yaml_str}")

            # Parse YAML (handles nested fields structure)
            header = {}
            fields = []
            current_field = None
            in_fields_section = False

            for line in yaml_str.split('\n'):
                stripped = line.strip()

                # Check if we're in the fields section
                if stripped.startswith('fields:'):
                    in_fields_section = True
                    continue

                if in_fields_section:
                    # New field starts with '- name:'
                    if stripped.startswith('- name:'):
                        if current_field:
                            fields.append(current_field)
                        current_field = {'name': stripped.split(':', 1)[1].strip()}
                    elif stripped and ':' in stripped and not stripped.startswith('-'):
                        # Field property
                        key, value = stripped.split(':', 1)
                        key = key.strip()
                        value = value.strip()

                        if current_field is not None:
                            current_field[key] = self._parse_yaml_value(value)
                else:
                    # Top-level properties
                    if ':' in stripped:
                        key, value = stripped.split(':', 1)
                        key = key.strip()
                        value = value.strip()

                        header[key] = self._parse_yaml_value(value)

            # Add last field if exists
            if current_field:
                fields.append(current_field)

            header['fields'] = fields
            return header

        finally:
            self.deallocate(input_ptr)
            self.deallocate(yaml_ptr)

    def extract_data_from_pc2_msg(self, pc2_msg: bytes, header_info: dict) -> np.ndarray:
        """
        Extract point cloud data from a PointCloud2 DDS message.

        Args:
            pc2_msg: Serialized PointCloud2 DDS message
            header_info: Header information

        Returns:
            Numpy array with point cloud data
        """
        # For now, we'll use a simplified approach and extract based on expected size
        # A full CDR deserializer would be needed for proper parsing
        width = header_info.get('width', 0)
        height = header_info.get('height', 0)
        point_step = header_info.get('point_step', 0)

        expected_data_size = width * height * point_step

        # The PointCloud2 message has the data field near the end
        # For now, just take the last expected_data_size bytes
        if len(pc2_msg) >= expected_data_size:
            # Extract the last expected_data_size bytes (the data field)
            data_bytes = pc2_msg[-expected_data_size:]
            return self.bytes_to_numpy(data_bytes, header_info)
        else:
            print(f"Warning: PC2 message size {len(pc2_msg)} < expected data size {expected_data_size}")
            return self.bytes_to_numpy(pc2_msg, header_info)

    def bytes_to_numpy(self, data: bytes, header_info: dict) -> np.ndarray:
        """
        Convert raw point cloud bytes to structured numpy array.

        Args:
            data: Raw point cloud data
            header_info: Header information from YAML

        Returns:
            Structured numpy array with named fields
        """
        width = header_info.get('width', 0)
        height = header_info.get('height', 0)
        point_step = header_info.get('point_step', 0)
        fields = header_info.get('fields', [])

        num_points = width * height

        if len(data) != num_points * point_step:
            print(f"Warning: Data size mismatch. Expected {num_points * point_step}, got {len(data)}")

        # Map Cloudini types to numpy dtypes
        type_map = {
            'FLOAT32': np.float32,
            'FLOAT64': np.float64,
            'UINT8': np.uint8,
            'UINT16': np.uint16,
            'UINT32': np.uint32,
            'INT8': np.int8,
            'INT16': np.int16,
            'INT32': np.int32,
        }

        # Create structured dtype from fields
        if fields:
            try:
                # Sort fields by offset to ensure correct order
                sorted_fields = sorted(fields, key=lambda f: f.get('offset', 0))

                # Build dtype specification with explicit offsets
                dtype_spec = {
                    'names': [f.get('name', f'field_{i}') for i, f in enumerate(sorted_fields)],
                    'formats': [type_map.get(f.get('type', 'UINT8'), np.uint8) for f in sorted_fields],
                    'offsets': [f.get('offset', 0) for f in sorted_fields],
                    'itemsize': point_step
                }

                dtype = np.dtype(dtype_spec)

                # Create structured array
                points = np.frombuffer(data, dtype=dtype, count=num_points)

                print(f"Created structured array with fields: {points.dtype.names}")
                return points

            except Exception as e:
                print(f"Warning: Failed to create structured array: {e}")
                print("Falling back to byte array")

        # Fallback: return as 2D array of bytes
        points = np.frombuffer(data, dtype=np.uint8)
        if num_points > 0 and point_step > 0:
            points = points.reshape((num_points, point_step))

        return points
