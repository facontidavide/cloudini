import { McapIndexedReader } from '@mcap/core';
import { BlobReadable } from '@mcap/browser';

const dropZone = document.getElementById('dropZone');
const fileInput = document.getElementById('fileInput');
const status = document.getElementById('status');
const results = document.getElementById('results');

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

async function handleFile(file) {
    if (!file.name.endsWith('.mcap')) {
        status.innerHTML = 'Error: Please select an MCAP file';
        return;
    }

    disableDropZone();
    status.innerHTML = 'Analyzing file...';
    results.innerHTML = '';

    try {
        const reader = await McapIndexedReader.Initialize({
            readable: new BlobReadable(file),
        });

        await analyzeFile(reader, file);

    } catch (error) {
        console.error('Error:', error);
        status.innerHTML = 'Error reading MCAP file: ' + error.message;
    } finally {
        enableDropZone();
    }
}

async function analyzeFile(reader, file) {
    const targetSchema = 'sensor_msgs/msg/PointCloud2';
    let foundChannels = [];
    let totalChannels = 0;
    const allSchemas = new Set();

    // Load WASM module
    status.innerHTML = 'Loading WASM module...';
    let wasmModule;
    try {
        // Create a script element to load the WASM module
        const script = document.createElement('script');
        script.src = '/cloudini_wasm.js';

        await new Promise((resolve, reject) => {
            script.onload = resolve;
            script.onerror = reject;
            document.head.appendChild(script);
        });

        // The module is now available as CloudiniModule
        wasmModule = await CloudiniModule();

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

    status.innerHTML = `Processing PointCloud2 channels inside the rosbag...`;

    if (foundChannels.length > 0) {
        // Process messages for each found channel
        const channelResults = [];

        for (const channel of foundChannels) {
            let messageCount = 0;
            let totalSize = 0;
            let totalCompressedSize = 0;

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
                    // Create a Uint8Array view over the message data
                    const dataView = new Uint8Array(message.data);

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

                    let compressedSize;

                    if (wasmModule._malloc && wasmModule._free && wasmModule.HEAPU8) {
                        // Direct memory approach (Option 1)
                        const dataPtr = wasmModule._malloc(dataSize);
                        const wasmView = new Uint8Array(wasmModule.HEAPU8.buffer, dataPtr, dataSize);
                        wasmView.set(dataView);

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

                    totalCompressedSize += compressedSize;
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
                totalCompressedSize,
                compressionRatio: totalSize > 0 ? (totalCompressedSize / totalSize).toFixed(3) : 0
            });
        }

        status.innerHTML = `File: ${file.name} | Channels: ${totalChannels} | Schemas: ${allSchemas.size}`;

        // Calculate totals across all channels
        const grandTotalSize = channelResults.reduce((sum, ch) => sum + ch.totalSize, 0);
        const grandTotalCompressed = channelResults.reduce((sum, ch) => sum + ch.totalCompressedSize, 0);
        const grandCompressionRatio = grandTotalSize > 0 ? (grandTotalCompressed / grandTotalSize).toFixed(3) : 0;

        results.innerHTML = `
            <div style="margin-top: 30px;">
                <h3 style="color: #2c3e50; font-size: 1.4em; margin-bottom: 20px; text-align: center;">
                    ✅ Found ${foundChannels.length} PointCloud2 Channel${foundChannels.length !== 1 ? 's' : ''}
                </h3>
                <div style="display: grid; gap: 15px; margin-bottom: 30px;">
                    ${channelResults.map(ch =>
                        `<div style="
                            background: linear-gradient(135deg, #ffffff 0%, #f8f9fa 100%);
                            border: 1px solid #e9ecef;
                            border-radius: 12px;
                            padding: 20px;
                            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.07);
                            transition: transform 0.2s ease, box-shadow 0.2s ease;
                        " onmouseover="this.style.transform='translateY(-2px)'; this.style.boxShadow='0 8px 15px rgba(0, 0, 0, 0.1)'"
                           onmouseout="this.style.transform='translateY(0)'; this.style.boxShadow='0 4px 6px rgba(0, 0, 0, 0.07)'">
                            <div style="display: grid; grid-template-columns: auto 1fr; gap: 15px; align-items: center;">
                                <div>
                                    <div style="font-size: 1.1em; font-weight: 600; color: #2c3e50; margin-bottom: 8px;">${ch.topic}</div>
                                    <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 8px; font-size: 0.9em; color: #6c757d;">
                                        <div><strong>Schema:</strong> ${ch.schema}</div>
                                        <div><strong>Encoding:</strong> ${ch.encoding}</div>
                                        <div><strong>Channel ID:</strong> ${ch.channelId}</div>
                                        <div><strong>Messages:</strong> <span style="color: #28a745; font-weight: 600;">${ch.messageCount.toLocaleString()}</span></div>
                                    </div>
                                </div>
                            </div>
                         </div>`
                    ).join('')}
                </div>

                <div style="
                    text-align: center;
                    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                    color: white;
                    border-radius: 15px;
                    padding: 30px;
                    margin: 30px 0;
                    font-size: 18px;
                    box-shadow: 0 10px 30px rgba(102, 126, 234, 0.3);
                ">
                    <h3 style="margin: 0 0 20px 0; font-size: 1.6em;">📊 Compression Analysis</h3>
                    <div style="font-size: 1.1em; opacity: 0.8; margin-bottom: 5px;">
                        This includes ONLY pointclouds, other messages in the rosbag are ignored.</div>
                    <div style="font-size: 1.0em; opacity: 0.8; margin-bottom: 5px;">
                        Quantization used: 1 millimeter</div>
                    <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; margin-top: 20px;">
                        <div style="background: rgba(255, 255, 255, 0.1); padding: 15px; border-radius: 10px; backdrop-filter: blur(10px);">
                            <div style="font-size: 0.9em; opacity: 0.8; margin-bottom: 5px;">Original Size (uncompressed)</div>
                            <div style="font-size: 1.4em; font-weight: 600;">${(grandTotalSize / (1024 * 1024)).toFixed(1)} MB</div>
                        </div>
                        <div style="background: rgba(255, 255, 255, 0.1); padding: 15px; border-radius: 10px; backdrop-filter: blur(10px);">
                            <div style="font-size: 0.9em; opacity: 0.8; margin-bottom: 5px;">Compressed Size</div>
                            <div style="font-size: 1.4em; font-weight: 600;">${(grandTotalCompressed / (1024 * 1024)).toFixed(1)} MB</div>
                        </div>
                        <div style="background: rgba(255, 255, 255, 0.1); padding: 15px; border-radius: 10px; backdrop-filter: blur(10px);">
                            <div style="font-size: 0.9em; opacity: 0.8; margin-bottom: 5px;">Compression Ratio</div>
                            <div style="font-size: 1.4em; font-weight: 600;">${grandCompressionRatio}</div>
                        </div>
                    </div>
                </div>
            </div>
            `;
    } else {
        status.innerHTML = `File: ${file.name} | Channels: ${totalChannels} | Schemas: ${allSchemas.size}`;
        results.innerHTML = `
            <div style="margin-top: 30px; text-align: center;">
                <div style="
                    background: linear-gradient(135deg, #ff9a56 0%, #ff6b6b 100%);
                    color: white;
                    border-radius: 15px;
                    padding: 30px;
                    margin: 20px 0;
                    box-shadow: 0 10px 30px rgba(255, 107, 107, 0.3);
                ">
                    <div style="font-size: 3em; margin-bottom: 15px;">🔍</div>
                    <h3 style="margin: 0 0 15px 0; font-size: 1.4em;">No PointCloud2 Channels Found</h3>
                    <p style="margin: 0; opacity: 0.9;">This MCAP file doesn't contain any sensor_msgs/msg/PointCloud2 data.</p>
                </div>

                <div style="
                    background: #f8f9fa;
                    border-radius: 12px;
                    padding: 25px;
                    margin-top: 20px;
                    border-left: 4px solid #6c757d;
                ">
                    <h4 style="color: #495057; margin: 0 0 15px 0;">📋 Available Schemas in this file:</h4>
                    <div style="
                        display: flex;
                        flex-wrap: wrap;
                        gap: 10px;
                        justify-content: center;
                        font-family: 'Courier New', monospace;
                        font-size: 0.9em;
                    ">
                        ${Array.from(allSchemas).sort().map(schema =>
                            `<span style="
                                background: #e9ecef;
                                color: #495057;
                                padding: 5px 10px;
                                border-radius: 15px;
                                border: 1px solid #dee2e6;
                            ">${schema}</span>`
                        ).join('')}
                    </div>
                </div>
            </div>
        `;
    }
}
