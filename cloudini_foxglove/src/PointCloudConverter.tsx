import { CloudiniCompressedPointCloud } from "./Schemas";
import CloudiniModule from "./cloudini_wasm_single.js";
import type { CloudiniWasmModule } from "./cloudini_wasm_single";

let wasmModule: CloudiniWasmModule | null = null;
let wasmLoadingPromise: Promise<void> | null = null;

export const loadCloudiniWasm = async () => {
  if (!wasmLoadingPromise) {
    wasmLoadingPromise = (CloudiniModule() as Promise<CloudiniWasmModule>).then((module: CloudiniWasmModule) => {
      wasmModule = module;
    });
  }
  return wasmLoadingPromise;
};

// Convert volirot timestamp (nanoseconds) to Foxglove/protobuf timestamp (seconds + nanos)
const convertTimestamp = (timestamp: { timestamp_ns: number }) => {
  const totalNanos = BigInt(timestamp.timestamp_ns);
  const seconds = Number(totalNanos / BigInt(1_000_000_000));
  const nanos = Number(totalNanos % BigInt(1_000_000_000));
  return { seconds, nanos };
};

// Convert volirot Transform to Foxglove Pose
const convertPose = (transform?: { translation: { values: number[] }; rotation: { values_xyzw: number[] } }) => {
  if (!transform) {
    // Return identity pose if no transform provided
    return {
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    };
  }
  const [tx = 0, ty = 0, tz = 0] = transform.translation.values;
  const [qx = 0, qy = 0, qz = 0, qw = 1] = transform.rotation.values_xyzw;
  return {
    position: { x: tx, y: ty, z: tz },
    orientation: { x: qx, y: qy, z: qz, w: qw },
  };
};

// Map Cloudini field types to Foxglove NumericType
// Cloudini: FLOAT32, FLOAT64, INT8, UINT8, INT16, UINT16, INT32, UINT32
// Foxglove: UNKNOWN=0, UINT8=1, INT8=2, UINT16=3, INT16=4, UINT32=5, INT32=6, FLOAT32=7, FLOAT64=8
const fieldTypeToNumericType = (type: string): number => {
  switch (type) {
    case "UINT8": return 1;
    case "INT8": return 2;
    case "UINT16": return 3;
    case "INT16": return 4;
    case "UINT32": return 5;
    case "INT32": return 6;
    case "FLOAT32": return 7;
    case "FLOAT64": return 8;
    default: return 0; // UNKNOWN
  }
};

// Simple YAML parser for Cloudini header format
const parseCloudiniHeader = (yaml: string) => {
  const lines = yaml.split('\n');
  let point_stride = 0;
  const fields: { name: string; offset: number; type: number }[] = [];

  let inFields = false;
  let currentField: { name?: string; offset?: number; type?: string } = {};

  for (const line of lines) {
    const trimmed = line.trim();

    if (trimmed.startsWith('point_step:')) {
      point_stride = parseInt(trimmed.split(':')[1]?.trim() ?? '0', 10);
    } else if (trimmed === 'fields:') {
      inFields = true;
    } else if (inFields) {
      if (trimmed.startsWith('- name:')) {
        // Save previous field if exists
        if (currentField.name !== undefined && currentField.offset !== undefined && currentField.type !== undefined) {
          fields.push({
            name: currentField.name,
            offset: currentField.offset,
            type: fieldTypeToNumericType(currentField.type),
          });
        }
        currentField = { name: trimmed.split(':')[1]?.trim() };
      } else if (trimmed.startsWith('offset:')) {
        currentField.offset = parseInt(trimmed.split(':')[1]?.trim() ?? '0', 10);
      } else if (trimmed.startsWith('type:')) {
        currentField.type = trimmed.split(':')[1]?.trim();
      }
    }
  }

  // Don't forget the last field
  if (currentField.name !== undefined && currentField.offset !== undefined && currentField.type !== undefined) {
    fields.push({
      name: currentField.name,
      offset: currentField.offset,
      type: fieldTypeToNumericType(currentField.type),
    });
  }

  return { point_stride, fields };
};

export const convertPointCloudWasm = (cloud: CloudiniCompressedPointCloud) => {
  if (!wasmModule) {
    loadCloudiniWasm();
    throw new Error('Cloudini WASM module is still loading. Please try again.');
  }

  const data = cloud.compressed_data;

  // Nothing to do if compressed data is empty
  if (data.byteLength === 0) {
    return {
      timestamp: convertTimestamp(cloud.timestamp),
      frame_id: cloud.frame_id,
      pose: convertPose(cloud.pose),
      point_stride: 0,
      fields: [],
      data: new Uint8Array(),
    };
  }

  let inputDataPtr: number | null = null;
  let outputDataPtr: number | null = null;
  let yamlOutputPtr: number | null = null;

  try {
    const bufferSize = data.byteLength;

    // Check if data is too large for WASM memory
    if (wasmModule.HEAPU8) {
      const maxAllowedSize = wasmModule.HEAPU8.length / 4;
      if (bufferSize > maxAllowedSize) {
        throw new Error(`Message too large (${bufferSize} bytes > ${maxAllowedSize} bytes)`);
      }
    }

    // Allocate memory for input data
    inputDataPtr = wasmModule._malloc(bufferSize);
    if (!inputDataPtr) {
      throw new Error('Failed to allocate memory for input data');
    }

    const wasmInputView = new Uint8Array(wasmModule.HEAPU8.buffer, inputDataPtr, bufferSize);
    wasmInputView.set(data);

    // Get the decompressed size from the compressed data header
    const decompressedSize = wasmModule._cldn_GetDecompressedSizeFromData(inputDataPtr, bufferSize);
    if (decompressedSize === 0) {
      throw new Error('Failed to get decompressed size from compressed data');
    }

    outputDataPtr = wasmModule._malloc(decompressedSize);
    if (!outputDataPtr) {
      throw new Error('Failed to allocate memory for output data');
    }

    const actualSize = wasmModule._cldn_DecodeCompressedData(inputDataPtr, bufferSize, outputDataPtr);
    if (actualSize === 0) {
      throw new Error('Decompression failed - function returned 0');
    }

    if (actualSize !== decompressedSize) {
      console.warn(`Decompressed size mismatch: expected ${decompressedSize}, got ${actualSize}`);
    }

    // Copy the result to a JavaScript array
    const decodedData = new Uint8Array(wasmModule.HEAPU8.buffer, outputDataPtr, actualSize);
    // Create a copy to ensure data persists after memory is freed
    const dataCopy = new Uint8Array(decodedData);

    // Get header info (point_stride, fields) from the compressed data
    // Allocate buffer for YAML output (should be enough for header)
    const yamlBufferSize = 8192;
    yamlOutputPtr = wasmModule._malloc(yamlBufferSize);
    if (!yamlOutputPtr) {
      throw new Error('Failed to allocate memory for YAML output');
    }

    const yamlSize = wasmModule._cldn_GetHeaderAsYAML(inputDataPtr, bufferSize, yamlOutputPtr);
    if (yamlSize === 0) {
      throw new Error('Failed to get header YAML from compressed data');
    }

    const yamlString = wasmModule.UTF8ToString(yamlOutputPtr, yamlSize);
    const headerInfo = parseCloudiniHeader(yamlString);

    return {
      timestamp: convertTimestamp(cloud.timestamp),
      frame_id: cloud.frame_id,
      pose: convertPose(cloud.pose),
      point_stride: headerInfo.point_stride,
      fields: headerInfo.fields,
      data: dataCopy,
    };
  } catch (error) {
    console.error('Cloudini decompression failed:', error);
    const errorMessage = error instanceof Error ? error.message : String(error);
    throw new Error(`Cloudini decompression failed: ${errorMessage}`);
  } finally {
    if (inputDataPtr) wasmModule._free(inputDataPtr);
    if (outputDataPtr) wasmModule._free(outputDataPtr);
    if (yamlOutputPtr) wasmModule._free(yamlOutputPtr);
  }
};
