#!/bin/bash
# Rebuild the cloudini WASM module with native WASM exceptions

set -e

cd /home/davide/ws_compression/src/clooudini

# Temporarily modify CMakeLists.txt to use -fwasm-exceptions
cd cloudini_lib

# Make a backup
cp CMakeLists.txt CMakeLists.txt.backup

# Replace NO_DISABLE_EXCEPTION_CATCHING with DISABLE_EXCEPTION_CATCHING (disable exceptions entirely)
# This should make the code not throw exceptions and instead return error codes
sed -i 's/-s NO_DISABLE_EXCEPTION_CATCHING/-s DISABLE_EXCEPTION_CATCHING/g' CMakeLists.txt

# Also remove SINGLE_FILE
sed -i 's/-s SINGLE_FILE=1 //g' CMakeLists.txt

# Rebuild
cd ..
echo "Rebuilding WASM module with exceptions disabled..."
cmake --build build_wasm --target cloudini_wasm --clean-first

# Restore backup
cd cloudini_lib
mv CMakeLists.txt.backup CMakeLists.txt

echo "WASM module rebuilt successfully!"
echo "Location: build_wasm/cloudini_wasm.wasm"
ls -lh ../build_wasm/cloudini_wasm.wasm
