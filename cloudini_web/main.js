import { McapIndexedReader } from '@mcap/core';
import { BlobReadable } from '@mcap/browser';
// This is needed for compressed bags, but could not make it work with vite
// import { loadDecompressHandlers } from "@mcap/support";

const dropZone = document.getElementById('dropZone');
const fileInput = document.getElementById('fileInput');
const status = document.getElementById('status');
const results = document.getElementById('results');

// Variable binding to the cloudini wasm module
let wasmModule;



dropZone.addEventListener('dragover', (e) => {
    e.preventDefault();
    if (!dropZone.classList.contains('disabled')) {
        dropZone.style.backgroundColor = '#f0f0f0';
    }
});

dropZone.addEventListener('dragleave', () => {
    dropZone.style.backgroundColor = 'white';
});

dropZone.addEventListener('drop', (e) => {
    e.preventDefault();
    if (dropZone.classList.contains('disabled')) {
        return;
    }
    dropZone.style.backgroundColor = 'white';
    const files = e.dataTransfer.files;
    if (files.length > 0) {
        handleFile(files[0]);
    }
});

dropZone.addEventListener('click', () => {
    if (!dropZone.classList.contains('disabled')) {
        fileInput.click();
    }
});

fileInput.addEventListener('change', (e) => {
    if (e.target.files.length > 0) {
        handleFile(e.target.files[0]);
    }
});

function disableDropZone() {
    dropZone.classList.add('disabled');
    dropZone.innerHTML = '⏳ Processing file... Please wait';
}

function enableDropZone() {
    dropZone.classList.remove('disabled');
    dropZone.innerHTML = '📁 Drag your rosbag here or click to select file<input type="file" id="fileInput" accept=".mcap" style="display: none;">';
    // Re-attach the file input event listener
    const newFileInput = document.getElementById('fileInput');
    newFileInput.addEventListener('change', (e) => {
        if (e.target.files.length > 0) {
            handleFile(e.target.files[0]);
        }
    });
}


async function loadCloudiniWasm(){
    const script = document.createElement('script');
    script.src = '/cloudini_wasm.js';

    await new Promise((resolve, reject) => {
        script.onload = resolve;
        script.onerror = reject;
        document.head.appendChild(script);
    });

    // The module is now available as CloudiniModule
    wasmModule = await CloudiniModule();
}  

async function handleFile(file) {
    if (!file.name.endsWith('.mcap')) {
        status.innerHTML = 'Error: Please select an MCAP file';
        return;
    }

    disableDropZone();
    status.innerHTML = 'Analyzing file...';
    results.innerHTML = '';

    try {
        //const decompressHandlers = await loadDecompressHandlers();

        const reader = await McapIndexedReader.Initialize({
            readable: new BlobReadable(file),
            // decompressHandlers,
        });

        const mode = document.querySelector('input[name="mode"]:checked').value;
        console.log('Selected mode:', mode);

        await analyzeFile(reader, file);

    } catch (error) {
        console.error('Error:', error);
        status.innerHTML = 'Error reading MCAP file: ' + error.message;
    } finally {
        enableDropZone();
    }
}



function compressPointCloudBuffer(bufferData){
    let compressedSize;
    let dataSize = bufferData.byteLength;
    if (wasmModule._malloc && wasmModule._free && wasmModule.HEAPU8) {
        // Direct memory approach (Option 1)
        const dataPtr = wasmModule._malloc(dataSize);
        const wasmView = new Uint8Array(wasmModule.HEAPU8.buffer, dataPtr, dataSize);
        wasmView.set(bufferData);

        compressedSize = wasmModule._ComputeCompressedSize(dataPtr, dataSize);
        wasmModule._free(dataPtr);
    } else {
        // Fallback to ccall approach
        compressedSize = wasmModule.ccall(
            'ComputeCompressedSize',
            'number',
            ['array', 'number'],
            [dataView, dataSize]
        );
    }

    return compressedSize;
}

function decompressPointCloudBuffer(bufferData) {
    let inputDataPtr = null;
    let outputDataPtr = null;
    try{
        const bufferSize = bufferData.byteLength;
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
        wasmInputView.set(bufferData);

        const decompressedSize = wasmModule._GetDecompressedSize(inputDataPtr, bufferSize);
        console.info("Decompressed size:", decompressedSize);
            
        outputDataPtr = wasmModule._malloc(decompressedSize); // allow for some extra space
        if (!outputDataPtr) {
            throw new Error('Failed to allocate memory for output data');
        }

        const actualSize = wasmModule._DecodePointCloudMessage(inputDataPtr, bufferSize, outputDataPtr);
        if (actualSize === 0) {
            throw new Error('Decompression failed - function returned 0');
        }

        // Copy the result to a JavaScript array
        const result = new Uint8Array(wasmModule.HEAPU8.buffer, outputDataPtr, actualSize);
        const decompressed = new Uint8Array(result);
        
        return actualSize;
    } finally {
        if (inputDataPtr) wasmModule._free(inputDataPtr);
        if (outputDataPtr) wasmModule._free(outputDataPtr);
    }
}

async function analyzeFile(reader, file) {

    const mode = document.querySelector('input[name="mode"]:checked').value;
    
    let compress = false;
    if (mode === 'compress') {
        compress = true;
        console.log("Compressing point clouds.");
    } else if (mode === 'decompress') {
        console.log("Decompressing point clouds.");
    } else {
        status.innerHTML = `Selected unsupported mode: ${mode}`;
    }

    const targetSchema = compress ? 'sensor_msgs/msg/PointCloud2' : 'point_cloud_interfaces/msg/CompressedPointCloud2';
    console.log("Searching for schema:", targetSchema);

    let foundChannels = [];
    let totalChannels = 0;
    const allSchemas = new Set();

    // Load WASM module
    status.innerHTML = 'Loading WASM module...';
    try {
        await loadCloudiniWasm();
        // Debug: log available properties
        console.log('WASM module loaded. Available properties:', Object.keys(wasmModule));
        console.log('Has HEAPU8:', !!wasmModule.HEAPU8);
        console.log('Has ccall:', !!wasmModule.ccall);
        console.log('Has _malloc:', !!wasmModule._malloc);
        console.log('Has _free:', !!wasmModule._free);
        console.log('Functions starting with _:', Object.keys(wasmModule).filter(k => k.startsWith('_')));

    } catch (error) {
        console.error('Failed to load WASM module:', error);
        status.innerHTML = 'Error loading WASM module: ' + error.message;
        return;
    }

    // Find channels with target schema
    for (const [channelId, channel] of reader.channelsById) {
        totalChannels++;
        const schema = reader.schemasById.get(channel.schemaId);

        if (schema) {
            allSchemas.add(schema.name);
            console.log("Found schema:", JSON.stringify(schema));
            if (schema.name === targetSchema) {
                foundChannels.push({
                    channelId,
                    topic: channel.topic,
                    schema: schema.name,
                    encoding: schema.encoding
                });
            }
        }
    }

    const schemaName = targetSchema.split('/').pop();
    status.innerHTML = `Processing ${schemaName} channels inside the rosbag...`;

    if (foundChannels.length > 0) {
        // Process messages for each found channel
        const channelResults = [];

        for (const channel of foundChannels) {
            let messageCount = 0;
            let totalSize = 0;
            let totalFinalSize = 0;

            for await (const message of reader.readMessages({
                startTime: reader.start,
                endTime: reader.end,
                topics: [channel.topic]
            })) {
                // We already know this channel has the target schema
                messageCount++;
                const dataSize = message.data.length;
                totalSize += dataSize;

                // Call WASM function
                try {

                    // Check if HEAPU8 is available for memory size checking
                    if (wasmModule.HEAPU8) {
                        // Check if data is too large for WASM memory (leave some buffer)
                        const maxAllowedSize = wasmModule.HEAPU8.length / 4; // Use only 1/4 of available memory
                        if (dataSize > maxAllowedSize) {
                            console.warn(`Message too large (${dataSize} bytes > ${maxAllowedSize}), skipping...`);
                            // Skip this message but continue processing
                            continue;
                        }
                    }

                    if (compress){
                        // Create a Uint8Array view over the message data
                        const dataView = new Uint8Array(message.data);
                        totalFinalSize += compressPointCloudBuffer(dataView);
                    } else {
                        try{
                            const dataView = new Uint8Array(message.data);
                            totalFinalSize += decompressPointCloudBuffer(dataView)
                        } catch (error) {
                            console.error('Error decompressing point cloud:', error);
                            continue;
                        }
                    }

                } catch (error) {
                    console.error('Error calling WASM function:', error);
                    console.error('Data size:', dataSize);
                    console.error('WASM memory size:', wasmModule.HEAPU8?.length || 'unknown');
                    // Skip this message and continue processing others
                }
            }

            channelResults.push({
                ...channel,
                messageCount,
                totalSize,
                totalFinalSize,
                compressionRatio: totalSize > 0 ? (totalFinalSize / totalSize).toFixed(3) : 0
            });
        }

        status.innerHTML = `File: ${file.name} | Channels: ${totalChannels} | Schemas: ${allSchemas.size}`;

        // Calculate totals across all channels
        const grandTotalSize = channelResults.reduce((sum, ch) => sum + ch.totalSize, 0);
        const grandTotalFinal = channelResults.reduce((sum, ch) => sum + ch.totalFinalSize, 0);
        const ratio = grandTotalSize > 0 ? (grandTotalFinal / grandTotalSize).toFixed(3) : 0;
        const grandCompressionRatio = (ratio * 1.0).toFixed(3);

        results.innerHTML = `
            <div class="results-container">
                <h3 class="results-title">
                    ✅ Found ${foundChannels.length} ${schemaName} Channel${foundChannels.length !== 1 ? 's' : ''}
                </h3>
                <div class="channels-grid">
                    ${channelResults.map(ch =>
                        `<div class="channel-card">
                            <div class="channel-content">
                                <div>
                                    <div class="channel-topic">${ch.topic}</div>
                                    <div class="channel-details">
                                        <div><strong>Schema:</strong> ${ch.schema}</div>
                                        <div><strong>Encoding:</strong> ${ch.encoding}</div>
                                        <div><strong>Channel ID:</strong> ${ch.channelId}</div>
                                        <div><strong>Messages:</strong> <span class="message-count">${ch.messageCount.toLocaleString()}</span></div>
                                    </div>
                                </div>
                            </div>
                         </div>`
                    ).join('')}
                </div>

                <div class="compression-analysis">
                    <h3 class="compression-title">📊 Compression Analysis</h3>
                    <div class="compression-note">
                        This includes ONLY pointclouds, other messages in the rosbag are ignored.</div>
                    <div class="compression-quantization">
                        Quantization used: 1 millimeter</div>
                    <div class="compression-stats">
                        <div class="stat-card">
                            <div class="stat-label">Original Size (${compress? 'uncompressed' : 'compressed'})</div>
                            <div class="stat-value">${(grandTotalSize / (1024 * 1024)).toFixed(1)} MB</div>
                        </div>
                        <div class="stat-card">
                            <div class="stat-label">${compress? 'Compressed' : 'Decompressed'} Size</div>
                            <div class="stat-value">${(grandTotalFinal / (1024 * 1024)).toFixed(1)} MB</div>
                        </div>
                        <div class="stat-card">
                            <div class="stat-label">${compress? 'Compression' : 'Decompression'} Ratio</div>
                            <div class="stat-value">${grandCompressionRatio}</div>
                        </div>
                    </div>
                </div>
            </div>
            `;
    } else {
        status.innerHTML = `File: ${file.name} | Channels: ${totalChannels} | Schemas: ${allSchemas.size}`;
        results.innerHTML = `
            <div class="no-results">
                <div class="no-results-card">
                    <div class="no-results-icon">🔍</div>
                    <h3 class="no-results-title">No ${schemaName} Channels Found</h3>
                    <p class="no-results-text">This MCAP file doesn't contain any sensor_msgs/msg/PointCloud2 data.</p>
                </div>

                <div class="schemas-info">
                    <h4 class="schemas-title">📋 Available Schemas in this file:</h4>
                    <div class="schemas-list">
                        ${Array.from(allSchemas).sort().map(schema =>
                            `<span class="schema-tag">${schema}</span>`
                        ).join('')}
                    </div>
                </div>
            </div>
        `;
    }
}
