#!/bin/bash
# Rebuild the cloudini WASM module without SINGLE_FILE to get separate .wasm file

set -e

cd /home/davide/ws_compression/src/clooudini

# Temporarily modify CMakeLists.txt to remove SINGLE_FILE option
cd cloudini_lib

# Make a backup
cp CMakeLists.txt CMakeLists.txt.backup

# Remove SINGLE_FILE from the link flags
sed -i 's/-s SINGLE_FILE=1 //g' CMakeLists.txt

# Rebuild
cd ..
cmake --build build_wasm --target cloudini_wasm

# Restore backup
cd cloudini_lib
mv CMakeLists.txt.backup CMakeLists.txt

echo "WASM module rebuilt successfully!"
echo "Location: build_wasm/cloudini_wasm.wasm"
ls -lh ../build_wasm/cloudini_wasm.wasm
