import { CompressedPointCloud, PointCloud } from "./Schemas";
import CloudiniModule from "./cloudini_wasm_single.js";
import type { CloudiniWasmModule } from "./cloudini_wasm_single";

let wasmModule: CloudiniWasmModule | null = null;
let wasmLoadingPromise: Promise<void> | null = null;

export const loadCloudiniWasm = async () => {
  if (!wasmLoadingPromise) {
    wasmLoadingPromise = CloudiniModule().then((module: CloudiniWasmModule) => {
      wasmModule = module;
    });
  }
  return wasmLoadingPromise;
};

export const convertPointCloudWasm = (cloud: CompressedPointCloud): PointCloud => {
  if (!wasmModule) {
    loadCloudiniWasm();
    throw new Error('Cloudini WASM module is still loading. Please try again.');
  }


  const decodedMsg: PointCloud = {
    header: {
      frame_id: cloud.header.frame_id,
      stamp: cloud.header.stamp,
    },
    height: cloud.height,
    width: cloud.width,
    fields: cloud.fields,
    is_bigendian: false,
    point_step: cloud.point_step,
    row_step: cloud.point_step * cloud.width,
    is_dense: cloud.is_dense,
    data: new Uint8Array(),
  };

  // Nothing to do, the point cloud is empty
  if (cloud.width * cloud.height === 0) {
    return decodedMsg;
  }


  let inputDataPtr: number | null = null;
  let outputDataPtr: number | null = null;
  const data = cloud.compressed_data;

  try {

    const bufferSize = data.byteLength;
    console.info("Decompressing point cloud buffer of size:", bufferSize);

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

    const decompressedSize = cloud.height * cloud.width * cloud.point_step;

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
    decodedMsg.data = new Uint8Array(decodedData);
  } catch (error) {
    console.error('Cloudini decompression failed:', error);
    // Preserve original frame_id and add error as a comment
    const errorMessage = error instanceof Error ? error.message : String(error);
    throw new Error(`Cloudini decompression failed: ${errorMessage}`);
  } finally {
    if (inputDataPtr) wasmModule._free(inputDataPtr);
    if (outputDataPtr) wasmModule._free(outputDataPtr);
  }

  return decodedMsg;
};
