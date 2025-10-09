# ROS2 specific libraries and utilities

## cloudini_topic_converter

A simple node that subscribes to a compressed `point_cloud_interfaces/msg/CompressedPointCloud2` and publishes a `sensor_msgs/msg/PointCloud2`.

It is MUCH **more efficient** than using the **point_cloud_transport** because the latter would:

1. Receive a serialized DDS message.
2. Convert that to **CompressedPointCloud2**.
3. Do the actual decompression.
4. Convert **PointCloud2** to a serialized DDS message.

Instead, we work directly with **raw** serialized messages, bypassing the ROS type system, skipping steps 2 and 4 in the list above.

This means less latency and less CPU used to make unnecessary copies.

### Parameters

- `topic_input` (string): name of the topic to subscribe to.
- `topic_output` (string): name of the topic to publish into.
- `compressing`(bool): true if you are subscribing to a regular pointcloud2, and you want to compress it.
  False if you are subscribing to a compressed topic and you want to decompress it.
- `resolution` (double): resolution of floating point fields in meters. Default: 0.001

### Example usage

To convert a regular `sensor_msgs/msg/PointCloud2` to a compressed one:

```
ros2 run cloudini_ros cloudini_topic_converter --ros-args \
    -p compressing:=true  \
    -p topic_input:=/points  \
    -p topic_output:=/points_encoded
```

To decompress that topic back in to its original form:

```
ros2 run cloudini_ros cloudini_topic_converter --ros-args \
    -p compressing:=false  \
    -p topic_input:=/points_encoded  \
    -p topic_output:=/points_decoded
```

## cloudini_rosbag_converter

A command line tool that, given a rosbag (limited to MCAP format), converts
 all `sensor_msgs/msg/PointCloud2` topics into compressed `point_cloud_interfaces/msg/CompressedPointCloud2` of vice-versa.

Encoding/decoding is faster than general-purpose compression algorithms and achieves a better compression ratio at 1mm resolution.

Interestingly, it can be compiled **without** ROS installed in your system!

Example usage: round trip compression / decompression;

```
# Use option -c for compression
cloudini_rosbag_converter -f original_rosbag.mcap -o compressed_rosbag.mcap -c

# Use option -d for decompression
cloudini_rosbag_converter -f compressed_rosbag.mcap -o restored_rosbag.mcap -d
```

Note that the "restored_rosbag.mcap" might be smaller than the original one, because the chunk-based ZSTD compression provided
by MCAP is enabled.
