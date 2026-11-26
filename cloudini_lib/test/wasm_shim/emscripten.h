// Stand-in for Emscripten's header, so that src/wasm_functions.cpp builds natively for test_wasm_functions.cpp.
// EM_ASM runs JavaScript in the browser; natively it does nothing.
#pragma once
#define EM_ASM(...) ((void)0)
