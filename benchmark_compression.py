#!/usr/bin/env python3
"""
Benchmark script for cloudini_rosbag_converter compression testing.
Tests compression on all .mcap files in DATA folder and saves results as JSON.
"""

import os
import subprocess
import time
import json
import glob
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading

# Global lock for thread-safe printing
print_lock = threading.Lock()

def get_file_size(filepath):
    """Get file size in bytes"""
    return os.path.getsize(filepath)

def run_compression(input_file, converter_path):
    """Run compression and measure execution time"""
    filename = os.path.basename(input_file)

    # Thread-safe printing
    with print_lock:
        print(f"Processing {filename}...")

    # Record start time
    start_time = time.time()

    # Run compression with time measurement
    cmd = [converter_path, "-c", "-y", "-f", input_file]
    result = subprocess.run(cmd, capture_output=True, text=True)

    # Record end time
    end_time = time.time()
    execution_time = end_time - start_time

    if result.returncode != 0:
        with print_lock:
            print(f"Error processing {filename}: {result.stderr}")
        return None

    with print_lock:
        print(f"Completed {filename} in {execution_time:.3f} seconds")

    return execution_time

def process_file(input_file, converter_path):
    """Process a single file and return complete result"""
    filename = os.path.basename(input_file)

    # Get original file size
    original_size = get_file_size(input_file)

    # Run compression
    execution_time = run_compression(input_file, converter_path)

    if execution_time is None:
        return None

    # Determine output filename (converter creates this)
    base_name = os.path.splitext(filename)[0]
    output_file = f"{base_name}_encoded.mcap"

    # Check if compressed file exists and get its size
    if os.path.exists(output_file):
        compressed_size = get_file_size(output_file)
        compression_ratio = compressed_size / original_size
    else:
        with print_lock:
            print(f"Warning: Expected output file {output_file} not found")
        compressed_size = None
        compression_ratio = None

    # Create result entry
    result_entry = {
        "filename": filename,
        "original_size_mb": round(original_size / (1024 * 1024), 2),
        "compressed_size_mb": round(compressed_size / (1024 * 1024), 2) if compressed_size else None,
        "execution_time_seconds": round(execution_time, 3),
        "compression_ratio": round(compression_ratio, 4) if compression_ratio else None
    }

    return result_entry

def main():
    # Paths
    data_dir = "DATA"
    converter_path = "./build_release/tools/cloudini_rosbag_converter"
    results_file = os.path.join(data_dir, "compression_results.json")

    # Check if converter exists
    if not os.path.exists(converter_path):
        print(f"Error: {converter_path} not found. Please build the project first.")
        return

    # Find all .mcap files in DATA directory
    mcap_files = glob.glob(os.path.join(data_dir, "*.mcap"))
    original_files = [f for f in mcap_files if not f.endswith("_encoded.mcap")]

    if not original_files:
        print(f"No .mcap files found in {data_dir}")
        return

    print(f"Found {len(original_files)} files to process")
    print(f"Using ThreadPoolExecutor for parallel processing...")

    results = []
    start_total_time = time.time()

    # Use ThreadPoolExecutor to process files in parallel
    # Limit max_workers to avoid overwhelming the system
    max_workers = min(len(original_files), os.cpu_count() or 4)

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        # Submit all compression tasks
        future_to_file = {
            executor.submit(process_file, input_file, converter_path): input_file
            for input_file in sorted(original_files)
        }

        # Collect results as they complete
        for future in as_completed(future_to_file):
            input_file = future_to_file[future]
            filename = os.path.basename(input_file)

            try:
                result_entry = future.result()
                if result_entry is not None:
                    results.append(result_entry)

                    # Print individual file summary
                    print(f"\n{'='*50}")
                    print(f"Completed: {filename}")
                    print(f"{'='*50}")
                    print(f"Original size: {result_entry['original_size_mb']} MB")
                    print(f"Compressed size: {result_entry['compressed_size_mb']} MB")
                    print(f"Compression ratio: {result_entry['compression_ratio']}")
                    print(f"Execution time: {result_entry['execution_time_seconds']} seconds")
                else:
                    print(f"Failed to process: {filename}")
            except Exception as exc:
                print(f"File {filename} generated an exception: {exc}")

    end_total_time = time.time()
    total_processing_time = end_total_time - start_total_time

    # Save results to JSON file
    with open(results_file, 'w') as f:
        json.dump({
            "benchmark_info": {
                "tool": "cloudini_rosbag_converter",
                "compression_method": "zstd",
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "total_files_processed": len(results),
                "max_workers": max_workers,
                "total_wall_time_seconds": round(total_processing_time, 3)
            },
            "results": results
        }, f, indent=2)

    print(f"\n{'='*50}")
    print(f"Results saved to: {results_file}")
    print(f"{'='*50}")

    # Print summary statistics
    if results:
        avg_compression = sum(r["compression_ratio"] for r in results if r["compression_ratio"]) / len([r for r in results if r["compression_ratio"]])
        total_cpu_time = sum(r["execution_time_seconds"] for r in results)

        print(f"\nSUMMARY STATISTICS:")
        print(f"Files processed: {len(results)}")
        print(f"Max workers used: {max_workers}")
        print(f"Average compression ratio: {avg_compression:.4f}")
        print(f"Total CPU time: {total_cpu_time:.3f} seconds")
        print(f"Total wall time: {total_processing_time:.3f} seconds")
        print(f"Speedup factor: {total_cpu_time/total_processing_time:.2f}x")

if __name__ == "__main__":
    main()
