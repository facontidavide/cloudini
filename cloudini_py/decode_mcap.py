#!/usr/bin/env python3
"""
Example showing how to decode Cloudini-compressed point clouds from MCAP files
using the CloudiniDecoder class.

Copyright 2025 Davide Faconti
Licensed under the Apache License, Version 2.0
"""

from pathlib import Path
from mcap.reader import make_reader
from cloudini_decoder import CloudiniDecoder


def decode_mcap_file(mcap_path: str, wasm_path: str, max_messages: int = None):
    """
    Decode all compressed point clouds from an MCAP file.

    Args:
        mcap_path: Path to the MCAP file
        wasm_path: Path to the cloudini WASM module
        max_messages: Maximum number of messages to decode (None = all)
    """
    decoder = CloudiniDecoder(wasm_path)

    print(f"\nReading MCAP file: {mcap_path}")

    with open(mcap_path, 'rb') as f:
        reader = make_reader(f)

        # Print summary
        summary = reader.get_summary()
        print(f"\nMCAP Summary:")
        print(f"  Duration: {summary.statistics.message_start_time / 1e9:.2f}s to {summary.statistics.message_end_time / 1e9:.2f}s")
        print(f"  Message count: {summary.statistics.message_count}")
        print(f"  Channels: {len(summary.channels)}")

        for channel_id, channel in summary.channels.items():
            print(f"    Channel {channel_id}: {channel.topic} ({channel.message_encoding})")

        # Process messages
        msg_count = 0

        for schema, channel, message in reader.iter_messages():
            # Look for CompressedPointCloud2 messages
            # Check if this is a compressed point cloud message by schema name
            is_compressed = (schema and 'CompressedPointCloud2' in schema.name)

            if is_compressed:
                msg_count += 1

                print(f"\n--- Message {msg_count} ---")
                print(f"Topic: {channel.topic}")
                print(f"Timestamp: {message.log_time / 1e9:.3f}s")

                try:
                    # Decode the message
                    point_cloud, header = decoder.decode_message(message.data)

                    print(f"✓ Decoded successfully!")
                    print(f"  Point cloud shape: {point_cloud.shape}")
                    print(f"  Data type: {point_cloud.dtype}")

                    # Calculate compression ratio (uncompressed / compressed)
                    compression_ratio = len(point_cloud.tobytes()) / len(message.data)
                    print(f"  Compression ratio: {compression_ratio:.2f}x")

                except Exception as e:
                    print(f"Failed to decode: {e}")
                    import traceback
                    traceback.print_exc()

                if max_messages and msg_count >= max_messages:
                    print(f"\nReached max messages limit ({max_messages})")
                    break

        print(f"\n=== Summary ===")
        print(f"Messages parsed: {msg_count}")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description='Decode Cloudini-compressed point clouds from MCAP files'
    )
    parser.add_argument(
        'mcap_file',
        help='Path to the MCAP file containing compressed point clouds'
    )
    parser.add_argument(
        '--wasm',
        default='../build_wasm/cloudini_wasm.wasm',
        help='Path to the cloudini WASM module (default: ../build_wasm/cloudini_wasm.wasm)'
    )
    parser.add_argument(
        '--max-messages',
        type=int,
        default=-1,
        help='Maximum number of messages to decode (default: -1 = all messages)'
    )

    args = parser.parse_args()

    # Resolve paths
    script_dir = Path(__file__).parent
    wasm_path = (script_dir / args.wasm).resolve()
    mcap_path = Path(args.mcap_file).resolve()

    if not wasm_path.exists():
        print(f"Error: WASM module not found at {wasm_path}")
        print("Please build the WASM module first:")
        print("  cmake -B build_wasm -S cloudini_lib -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
        print("  cmake --build build_wasm")
        return 1

    if not mcap_path.exists():
        print(f"Error: MCAP file not found at {mcap_path}")
        return 1

    max_msgs = None if args.max_messages < 0 else args.max_messages
    decode_mcap_file(str(mcap_path), str(wasm_path), max_msgs)

    return 0


if __name__ == '__main__':
    exit(main())
