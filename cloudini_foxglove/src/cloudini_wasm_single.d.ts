export interface EmscriptenModule {
  // Memory management
  _malloc(size: number): number;
  _free(ptr: number): void;

  // Memory views
  HEAPU8: Uint8Array;
  HEAP8: Int8Array;
  HEAPU16: Uint16Array;
  HEAP16: Int16Array;
  HEAPU32: Uint32Array;
  HEAP32: Int32Array;
  HEAPF32: Float32Array;
  HEAPF64: Float64Array;

  // Emscripten runtime methods
  ccall: (ident: string, returnType: string | null, argTypes: string[], args: any[]) => any;
  cwrap: (ident: string, returnType: string | null, argTypes: string[]) => (...args: any[]) => any;
}

export interface CloudiniWasmModule extends EmscriptenModule {
  // Your specific exported functions
  _cldn_ComputeCompressedSize(inputPtr: number, inputSize: number, resolution: number): number;
  _cldn_DecodeCompressedData(inputPtr: number, inputSize: number, outputPtr: number): number;
  _cldn_DecodeCompressedMessage(inputPtr: number, inputSize: number, outputPtr: number): number;
  // Input: a serialized DDS message containing point_cloud_interfaces/CompressedPointCloud2.
  _cldn_GetDecompressedSize(inputPtr: number, inputSize: number): number;
  // Input: a bare Cloudini buffer. Writes {width, height, point_step, number of fields} as 4 uint32 at outInfoPtr;
  // returns 1, or 0 when the header does not parse.
  _cldn_GetDataInfo(inputPtr: number, inputSize: number, outInfoPtr: number): number;
  // Input: a bare Cloudini buffer. The output size _cldn_DecodeCompressedData needs, or 0 on error.
  _cldn_GetDecodedDataSize(inputPtr: number, inputSize: number): number;
}

declare function CloudiniModule(): Promise<CloudiniWasmModule>;
export default CloudiniModule;
