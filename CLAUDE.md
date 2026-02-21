# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Cloudini is a high-performance pointcloud compression library with bindings for ROS, PCL, and WebAssembly. It implements a two-stage compression approach: custom encoding for pointcloud fields followed by general-purpose compression (LZ4/ZSTD).

## Architecture

The project consists of three main components:

- **cloudini_lib/**: Core compression library (C++20) with field encoders/decoders
- **cloudini_ros/**: ROS2 integration with point_cloud_transport plugins and conversion utilities
- **cloudini_web/**: WebAssembly interface for browser-based compression

### Core Library (cloudini_lib)

Key components:
- **PointcloudEncoder**: Multi-threaded encoder with double-buffering pattern for LZ4/ZSTD compression
- **PointcloudDecoder**: Decoder supporting chunked decompression
- **Field Encoders/Decoders**: Type-specific encoders for efficient field compression
  - `FieldEncoderFloatN_Lossy`: SIMD-optimized encoder for 3-4 consecutive FLOAT32 fields (typically XYZ/XYZRGB)
  - `FieldEncoderInt`: Delta + varint encoding for integer types
  - `FieldEncoderFloat_XOR`: XOR-based lossless float compression
  - `FieldEncoderCopy`: Pass-through for unsupported types

**Threading Model**:
- Encoder uses a worker thread (`compressionWorker()`) for LZ4/ZSTD compression
- Double-buffering pattern: main thread encodes fields while worker compresses previous chunk
- Condition variables `cv_ready_to_compress_` and `cv_done_compressing_` coordinate work
- **CRITICAL**: All shared state modifications (`compression_done_`, `compressed_size_`) MUST be protected by `mutex_`

### ROS2 Integration (cloudini_ros)

**Key Classes**:

1. **CloudiniSubscriberPCL** (`cloudini_ros/include/cloudini_ros/cloudini_subscriber_pcl.hpp`)
   - High-performance subscriber that converts CompressedPointCloud2 directly to `pcl::PCLPointCloud2`
   - Uses `rclcpp::GenericSubscription` for zero-copy raw DDS message access
   - Implements object pool pattern (max 4 objects) to avoid repeated allocations
   - Thread-safe with mutex-protected pool
   - Bypasses intermediate `sensor_msgs::PointCloud2` conversion

2. **Topic Converter Node** (`cloudini_ros/src/topic_converter.cpp`)
   - Converts between `sensor_msgs/PointCloud2` and `point_cloud_interfaces/CompressedPointCloud2`
   - Supports both compression and decompression modes
   - Optimizations:
     - Skips processing when `get_subscription_count() == 0`
     - Auto-detects publisher QoS settings via `adapt_request_to_offers()`
     - Zero-copy via raw DDS message manipulation

3. **Point Cloud Transport Plugin** (`cloudini_ros/src/cloudini_publisher_plugin.cpp`)
   - Integrates with ROS2 `point_cloud_transport` framework (similar to `image_transport`)
   - Automatically creates compressed topic variants (e.g., `/points/cloudini`)
   - Configurable via transport hints: `TransportHints("cloudini")`

**Message Parsing Utilities** (`cloudini_lib/include/cloudini_lib/ros_msg_utils.hpp`):
- `getDeserializedPointCloudMessage()`: Parse raw DDS messages without full deserialization
- `convertPointCloud2ToCompressedCloud()`: Convert + compress in single pass
- `convertCompressedCloudToPointCloud2()`: Decompress + convert
- `toEncodingInfo()`: Extract encoding metadata from point cloud

**PCL Integration** (`cloudini_lib/include/cloudini_lib/pcl_conversion.hpp`):
- `PCLPointCloudEncode()`: Encode `pcl::PCLPointCloud2` to Cloudini format
- `PCLPointCloudDecode()`: Decode Cloudini format to `pcl::PCLPointCloud2`
- Direct conversion without intermediate ROS message types

## Build Commands

### Core Library (Standalone)
```bash
cmake -B build_release -S cloudini_lib -DCMAKE_BUILD_TYPE=Release
cmake --build build_release --parallel
```

### Debug Build with Sanitizers
```bash
cmake -B build_debug -S cloudini_lib -DCMAKE_BUILD_TYPE=Debug
cmake --build build_debug --parallel
```

### ROS Build
```bash
colcon build --packages-select cloudini_lib cloudini_ros
```

### WebAssembly Build
```bash
# Requires Emscripten
cmake -B build_wasm -S cloudini_lib -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake
cmake --build build_wasm
```

### Web Interface
```bash
cd cloudini_web
npm install
npm run dev  # Development server
npm run build  # Production build
```

## Testing

### Core Library Tests
```bash
# Core library tests
cd build_debug && ctest

# Run specific benchmark
./build_release/benchmarks/pcd_benchmark
```

### ROS2 Test Nodes

**Test Publisher** (`cloudini_ros/test/test_plugin_publisher.cpp`):
```bash
ros2 run cloudini_ros test_plugin_publisher --ros-args -p input_topic:=/points -p output_topic:=/points_pct
```
- Subscribes to standard PointCloud2 topic
- Publishes via `point_cloud_transport`, automatically creating compressed variants
- Creates topics: `/points_pct` (raw) and `/points_pct/cloudini` (compressed)

**Test Subscriber** (`cloudini_ros/test/test_plugin_subscriber.cpp`):
```bash
ros2 run cloudini_ros test_plugin_subscriber --ros-args -p topic:=/points_pct
```
- Uses `point_cloud_transport` to subscribe with transport hint "cloudini"
- Manually converts to PCL format in callback

**Direct PCL Subscriber** (`cloudini_ros/test/test_cloudini_subscriber.cpp`):
```bash
ros2 run cloudini_ros test_cloudini_subscriber --ros-args -p topic:=/points/cloudini
```
- Uses `CloudiniSubscriberPCL` for most efficient direct PCL conversion
- Bypasses `point_cloud_transport` framework for minimal overhead

**Topic Converter**:
```bash
# Compress: sensor_msgs/PointCloud2 -> CompressedPointCloud2
ros2 run cloudini_ros cloudini_topic_converter --ros-args \
  -p compressing:=true \
  -p topic_input:=/points \
  -p topic_output:=/points/compressed \
  -p resolution:=0.001

# Decompress: CompressedPointCloud2 -> sensor_msgs/PointCloud2
ros2 run cloudini_ros cloudini_topic_converter --ros-args \
  -p compressing:=false \
  -p topic_input:=/points/compressed \
  -p topic_output:=/points/decompressed
```

**Direct Compression Publisher** (`cloudini_ros/test/test_direct_publisher.cpp`):
```bash
ros2 run cloudini_ros test_direct_publisher --ros-args \
  -p input_topic:=/points \
  -p output_topic:=/points/compressed \
  -p resolution:=0.001
```
- Compresses PointCloud2 directly using `SerializeCompressedPointCloud2`
- Publishes as serialized `CompressedPointCloud2` via generic publisher
- No topic_converter node needed

### Benchmarking

**MCAP Rosbag Conversion**:
```bash
# DATA folder may contain .mcap files
./build_release/tools/cloudini_rosbag_converter -c -y -f DATA/<name_of_the_file>
```

### Debugging with ROS2 CLI

```bash
# List available topics
ros2 topic list

# Check topic type
ros2 topic info /points/cloudini

# Monitor message rate
ros2 topic hz /points/cloudini

# Echo messages (won't work for compressed - binary data)
ros2 topic echo /points
```

## Key Dependencies

- **Required**: CMake 3.16+, C++20 compiler
- **Auto-downloaded via CPM**: LZ4, ZSTD, cxxopts, benchmark, googletest
- **Optional**: PCL (for pointcloud utilities), ROS 2 (for integration)
- **Web**: Node.js 18+, Vite
- **ROS2 Packages**:
  - `rclcpp` - Core ROS2 C++ API
  - `sensor_msgs` - Standard PointCloud2 messages
  - `point_cloud_interfaces` - CompressedPointCloud2 message definition
  - `point_cloud_transport` - Transport framework for point cloud plugins
  - `pcl_conversions` - PCL/ROS message conversion utilities (apt: `ros-${ROS_DISTRO}-pcl-conversions`)
  - `pluginlib` - Plugin loading framework

## API Usage Patterns

### Using CloudiniSubscriberPCL (Most Efficient)

```cpp
#include <cloudini_ros/cloudini_subscriber_pcl.hpp>

auto node = std::make_shared<rclcpp::Node>("my_node");

// Create subscriber with callback
auto subscriber = std::make_shared<cloudini_ros::CloudiniSubscriberPCL>(
    node,  // Can also use node->shared_from_this() if inside node class
    "/points/cloudini",  // Topic name
    [](const pcl::PCLPointCloud2::Ptr& cloud) {
        // Process PCL cloud - object is from pool, will be returned when lambda exits
        std::cout << "Received cloud with " << cloud->width * cloud->height << " points\n";
    },
    rclcpp::QoS(10)  // Optional QoS settings
);

rclcpp::spin(node);
```

**IMPORTANT**: Do NOT call `shared_from_this()` in node constructors - use raw `this` pointer instead, or call from `init()` method after node construction.

### Using Point Cloud Transport

```cpp
#include <point_cloud_transport/point_cloud_transport.hpp>

// Publisher side
auto pct = std::make_shared<point_cloud_transport::PointCloudTransport>(node);
auto pub = pct->advertise("/points", 10);
// Automatically creates /points/cloudini compressed topic

pub.publish(cloud_msg);

// Subscriber side with transport hint
auto transport_hint = std::make_shared<point_cloud_transport::TransportHints>("cloudini");
auto sub = pct->subscribe("/points", 10, callback, {}, transport_hint.get());
```

### Raw DDS Message Access Pattern

```cpp
#include <rclcpp/generic_subscription.hpp>
#include <cloudini_lib/ros_msg_utils.hpp>

// Generic subscription for zero-copy access
auto callback = [](std::shared_ptr<rclcpp::SerializedMessage> msg) {
    const auto& input_msg = msg->get_rcl_serialized_message();
    const Cloudini::ConstBufferView raw_dds_msg(input_msg.buffer, input_msg.buffer_length);

    // Parse without full deserialization
    auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(raw_dds_msg);

    // Process compressed data directly
    // pc_info.data contains the cloudini-compressed payload
};

auto subscription = node->create_generic_subscription(
    topic, "point_cloud_interfaces/msg/CompressedPointCloud2",
    rclcpp::QoS(10), callback
);
```

### Resolution Configuration

```cpp
#include <cloudini_lib/ros_msg_utils.hpp>

// Create encoding info from PointCloud2
auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(raw_msg);
auto encoding_info = cloudini_ros::toEncodingInfo(pc_info);

// Apply resolution profile (quantization for lossy compression)
cloudini_ros::applyResolutionProfile(
    cloudini_ros::ResolutionProfile{},  // Default profile
    encoding_info.fields,
    0.001  // 1mm resolution
);

// Convert with custom encoding
std::vector<uint8_t> output;
cloudini_ros::convertPointCloud2ToCompressedCloud(pc_info, encoding_info, output);
```

### Direct Compression (Without Topic Converter)

```cpp
#include <cloudini_ros/conversion_utils.hpp>
#include <rclcpp/rclcpp.hpp>

// Setup: create a generic publisher for CompressedPointCloud2
auto publisher = node->create_generic_publisher(
    "/points/compressed",
    "point_cloud_interfaces/msg/CompressedPointCloud2",
    rclcpp::QoS(10));

// Compress a PointCloud2 message
std::vector<uint8_t> buffer;
Cloudini::SerializeCompressedPointCloud2(pcd_msg, 0.001, buffer);

// Publish using zero-copy pointer swap
rclcpp::SerializedMessage ser_msg;
ser_msg.get_rcl_serialized_message().buffer = buffer.data();
ser_msg.get_rcl_serialized_message().buffer_length = buffer.size();
publisher->publish(ser_msg);
// NOTE: null out ser_msg buffer before destruction to prevent double-free
ser_msg.get_rcl_serialized_message().buffer = nullptr;
ser_msg.get_rcl_serialized_message().buffer_length = 0;
```

## Performance Optimization Patterns

### Object Pool Pattern (CloudiniSubscriberPCL)

CloudiniSubscriberPCL implements an object pool to avoid repeated PCL cloud allocations:

```cpp
pcl::PCLPointCloud2::Ptr CloudiniSubscriberPCL::acquireCloudFromPool() {
    pcl::PCLPointCloud2* raw_ptr = nullptr;

    {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        if (!cloud_pool_.empty()) {
            raw_ptr = cloud_pool_.back();
            cloud_pool_.pop_back();
        } else {
            raw_ptr = new pcl::PCLPointCloud2();
        }
    }

    // Custom deleter returns to pool
    return pcl::PCLPointCloud2::Ptr(raw_ptr, [this](pcl::PCLPointCloud2* ptr) {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        if (cloud_pool_.size() < MAX_POOL_SIZE) {
            ptr->data.clear();  // Clear but keep capacity
            ptr->fields.clear();
            cloud_pool_.push_back(ptr);
        } else {
            delete ptr;
        }
    });
}
```

**Benefits**: ~75% reduction in allocations during steady-state operation.

### Skip Processing When No Subscribers

```cpp
void callback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    // Skip expensive processing if no one is listening
    if (publisher_->get_subscription_count() == 0) {
        return;
    }

    // Proceed with compression/decompression
    // ...
}
```

### Zero-Copy via Raw DDS Access

Instead of deserializing to `sensor_msgs::PointCloud2`, parse the raw DDS buffer:

```cpp
// AVOID THIS (requires full deserialization + copy):
void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto pcl_cloud = std::make_shared<pcl::PCLPointCloud2>();
    pcl_conversions::toPCL(*msg, *pcl_cloud);
    // ...
}

// PREFER THIS (zero-copy):
void callback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    const auto& raw = msg->get_rcl_serialized_message();
    const Cloudini::ConstBufferView view(raw.buffer, raw.buffer_length);
    auto pc_info = cloudini_ros::getDeserializedPointCloudMessage(view);
    // Access fields directly from buffer without intermediate copies
}
```

## Development Notes

- Uses CPM for dependency management - set `CPM_SOURCE_CACHE` environment variable to avoid re-downloading
- Supports SSE4.1 optimizations on x86_64
- Address sanitizer enabled automatically in Debug builds on GCC/Clang
- Custom intrinsics for SIMD operations in `include/cloudini_lib/intrinsics.hpp`
- Field encoders support both lossy (quantized floats) and lossless compression modes

### Threading Safety Guidelines

**CRITICAL**: When modifying shared state accessed by multiple threads:
1. Always acquire mutex before modifying condition variable predicates
2. Release mutex before calling `notify_one()` or `notify_all()`
3. Use `std::lock_guard` for automatic RAII-style locking

Example from `PointcloudEncoder::compressionWorker()`:
```cpp
// CORRECT:
{
    std::lock_guard<std::mutex> lock(mutex_);
    compressed_size_ += chunk_size + sizeof(uint32_t);
    compression_done_ = true;
}
cv_done_compressing_.notify_one();  // Notify after releasing lock

// WRONG - DATA RACE:
compressed_size_ += chunk_size;  // No mutex!
compression_done_ = true;         // Race condition!
cv_done_compressing_.notify_one();
```

### ROS2 Node Lifecycle Safety

**IMPORTANT**: Do NOT call `shared_from_this()` in constructors:

```cpp
// WRONG - crashes with bad_weak_ptr:
MyNode::MyNode() : Node("my_node") {
    subscriber_ = std::make_shared<CloudiniSubscriberPCL>(
        this->shared_from_this(),  // BAD - shared_ptr not yet constructed
        "/topic", callback
    );
}

// CORRECT Option 1 - Use raw pointer:
MyNode::MyNode() : Node("my_node") {
    subscriber_ = std::make_shared<CloudiniSubscriberPCL>(
        this,  // GOOD - raw pointer
        "/topic", callback
    );
}

// CORRECT Option 2 - Separate init():
MyNode::MyNode() : Node("my_node") {}

void MyNode::init() {
    subscriber_ = std::make_shared<CloudiniSubscriberPCL>(
        this->shared_from_this(),  // GOOD - called after construction
        "/topic", callback
    );
}
```

## Common Issues and Fixes

### Issue: Timeout waiting for compression to complete

**Symptom**: Exception "Timeout waiting for compression to complete" during encoding.

**Cause**: Data race or lost notification in multi-threaded compression worker.

**Fix**: Ensure all modifications to `compression_done_` and `compressed_size_` are protected by mutex. Fixed in recent commit to `cloudini_lib/src/cloudini.cpp`.

### Issue: Point cloud transport subscriber not receiving messages

**Symptom**: Subscriber callback never fires despite topic being published.

**Cause**: Topic naming mismatch. Point cloud transport expects base topic name (e.g., `/points`), not the compressed variant (e.g., `/points/cloudini`).

**Solution**:
- Publisher: Use base topic name with `advertise("/points", 10)`
- Subscriber: Use base topic + transport hint `subscribe("/points", 10, callback, {}, TransportHints("cloudini"))`

### Issue: PCL conversions header not found

**Symptom**: Compilation error `pcl_conversions/pcl_conversions.h: No such file or directory`

**Solution**:
1. Install package: `sudo apt install ros-${ROS_DISTRO}-pcl-conversions`
2. Add to CMakeLists.txt:
   ```cmake
   find_package(pcl_conversions REQUIRED)
   target_link_libraries(target ${pcl_conversions_TARGETS})
   ```
3. Export in package.xml: `<depend>pcl_conversions</depend>`

### Issue: Generic subscription crashes with deserialization error

**Symptom**: Runtime crash when accessing serialized message buffer.

**Cause**: Incorrect topic type string or attempting to deserialize incompatible message.

**Solution**: Verify topic type matches exactly:
```bash
ros2 topic info /topic_name  # Check actual type
```

Use correct type string:
- `"sensor_msgs/msg/PointCloud2"` (not `sensor_msgs::msg::PointCloud2`)
- `"point_cloud_interfaces/msg/CompressedPointCloud2"`

## Tools

- **cloudini_rosbag_converter**: Convert MCAP rosbags between compressed/uncompressed pointclouds
- **mcap_cutter**: Extract portions of MCAP files
- **pcd_benchmark**: Benchmark compression on PCD files
- **run_encoder.sh**: Batch processing script for test data

## File Structure Reference

### cloudini_lib/
```
include/cloudini_lib/
├── cloudini.hpp              # Core encoder/decoder classes
├── encoding_utils.hpp        # Buffer views, encode/decode helpers
├── field_encoder.hpp         # Field-specific encoder implementations
├── field_decoder.hpp         # Field-specific decoder implementations
├── intrinsics.hpp            # SIMD intrinsics wrappers (Vector4f, Vector4i)
├── pcl_conversion.hpp        # PCL ↔ Cloudini conversion
└── ros_msg_utils.hpp         # ROS2 message parsing/conversion utilities

src/
├── cloudini.cpp              # PointcloudEncoder/Decoder implementation
├── field_encoder.cpp         # Encoder implementations (FloatN, Int, XOR)
├── field_decoder.cpp         # Decoder implementations
├── pcl_conversion.cpp        # PCL integration
└── ros_msg_utils.cpp         # ROS message utilities

tools/
├── cloudini_rosbag_converter # MCAP conversion tool
└── mcap_cutter              # MCAP slicing utility
```

### cloudini_ros/
```
include/cloudini_ros/
└── cloudini_subscriber_pcl.hpp  # High-performance PCL subscriber with object pool

src/
├── topic_converter.cpp          # Standalone compression/decompression node
├── cloudini_subscriber_pcl.cpp  # PCL subscriber implementation
├── cloudini_publisher_plugin.cpp # point_cloud_transport publisher plugin
└── cloudini_subscriber_plugin.cpp # point_cloud_transport subscriber plugin

test/
├── test_cloudini_subscriber.cpp # Direct CloudiniSubscriberPCL usage example
├── test_direct_publisher.cpp    # Direct compression publisher example
├── test_plugin_publisher.cpp    # point_cloud_transport publisher example
└── test_plugin_subscriber.cpp   # point_cloud_transport subscriber example
```

## Quick Reference

### Compression Modes

1. **None** (`EncodingOptions::NONE` + `CompressionOption::NONE`)
   - No encoding, no compression (for debugging)

2. **Lossless** (`EncodingOptions::LOSSLESS` + `CompressionOption::LZ4/ZSTD`)
   - Delta encoding for integers
   - XOR encoding for floats
   - LZ4 or ZSTD compression

3. **Lossy** (`EncodingOptions::LOSSY` + `CompressionOption::LZ4/ZSTD`)
   - Quantized floats based on resolution parameter
   - SIMD-optimized for XYZ/XYZRGB fields
   - Highest compression ratio
   - Typical resolution: 0.001 (1mm) for geometry

### QoS Profile Auto-Detection

`topic_converter.cpp` uses `adapt_request_to_offers()` to auto-detect publisher QoS:
- **Reliability**: Uses RELIABLE if all publishers are RELIABLE, else BEST_EFFORT
- **Durability**: Uses TRANSIENT_LOCAL if all publishers are TRANSIENT_LOCAL (for latched topics), else VOLATILE

### Typical Compression Ratios

Based on empirical data:
- **Lossy (1mm resolution)**: 5-10x compression
- **Lossless**: 2-4x compression
- **Depends on**: Point cloud structure, field types, data entropy

### Message Flow Diagram

```
Standard ROS2 Workflow:
┌─────────────────┐
│ Sensor/Driver   │
│ PointCloud2     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────────┐
│ point_cloud_    │─────▶│ cloudini plugin  │
│ transport       │      │ (compress)       │
└─────────────────┘      └────────┬─────────┘
                                  │
                                  ▼
                         ┌─────────────────┐
                         │ Compressed      │
                         │ PointCloud2     │
                         └────────┬────────┘
                                  │
                                  ▼
┌─────────────────┐      ┌──────────────────┐
│ CloudiniSub     │◀─────│ Generic Sub      │
│ PCL (fastest)   │      │ (zero-copy DDS)  │
└─────────────────┘      └──────────────────┘

Alternative (Manual Conversion):
┌─────────────────┐
│ Sensor          │
│ PointCloud2     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ topic_converter │ (standalone node)
│ compressing=true│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Compressed      │
│ PointCloud2     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ CloudiniSub     │
│ PCL             │
└─────────────────┘
```

## Performance Tips

1. **Use CloudiniSubscriberPCL for PCL workflows** - bypasses intermediate ROS conversions
2. **Enable intra-process comms** when publisher/subscriber in same process:
   ```cpp
   rclcpp::NodeOptions options;
   options.use_intra_process_comms(true);
   auto node = std::make_shared<MyNode>(options);
   ```
3. **Set resolution based on needs**:
   - Indoor robotics: 0.001 (1mm)
   - Outdoor/autonomous driving: 0.01 (1cm)
   - Architecture/mapping: 0.0001 (0.1mm)
4. **Use ZSTD for maximum compression** (slower), **LZ4 for real-time** (faster)
5. **Monitor with** `ros2 topic hz` and `ros2 topic bw` to verify compression gains

## Recent Fixes (as of 2025)

- **Threading fix in PointcloudEncoder**: Protected `compression_done_` and `compressed_size_` with mutex to prevent data races and deadlocks
- **Object pool optimization**: Added to CloudiniSubscriberPCL for ~75% allocation reduction
- **Subscriber-aware processing**: topic_converter now skips processing when no subscribers present
- **QoS auto-detection**: topic_converter automatically adapts to publisher QoS settings
- **pcl_conversions linking**: Added to CMakeLists.txt for proper PCL integration
