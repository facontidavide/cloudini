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

WASM_EXPORT size_t ComputeCompressedSize(uintptr_t data_ptr, size_t size);
WASM_EXPORT size_t DecodePointCloudMessage(uintptr_t msg_ptr, size_t msg_size, uintptr_t output_ptr);
WASM_EXPORT size_t DecodePointCloudBuffer(uintptr_t encoded_ptr, size_t encoded_size, uintptr_t output_ptr);
WASM_EXPORT size_t GetDecompressedSize(uintptr_t encoded_ptr, size_t encoded_size);
}
