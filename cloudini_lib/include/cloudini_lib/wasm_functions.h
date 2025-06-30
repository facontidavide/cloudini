#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define WASM_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define WASM_EXPORT
#endif

extern "C" {

WASM_EXPORT size_t ComputeCompressedSize(uintptr_t dataPtr, size_t size);
WASM_EXPORT size_t DecompressPointCloudBuffer(uintptr_t compressedPtr, size_t compressedSize, uintptr_t outputPtr);

}
