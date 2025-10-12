# ROS2 specific libraries and utilities

## cloudini_topic_converter

A simple node that subscribes to one of these two topic types and publishes the other:

- `sensor_msgs/msg/PointCloud2`
- `point_cloud_interfaces/msg/CompressedPointCloud2`

It is genrally **more efficient** than using the **point_cloud_transport** because the latter would:

1. Receive a serialized DDS message.
2. Convert that to **CompressedPointCloud2**.
3. Do the actual decompression.
4. Convert **PointCloud2** to a serialized DDS message.

Instead, we work directly with **raw** serialized messages, bypassing the ROS type system, skipping steps 2 and 4 in the list above.

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
    -p topic_output:=/points/compressed
```

To decompress that topic back in to its original form:

```
ros2 run cloudini_ros cloudini_topic_converter --ros-args \
    -p compressing:=false  \
    -p topic_input:=/points/compressed  \
    -p topic_output:=/points/decompressed
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


# How to read directly a CompressedPointCloud2 from your application

## Using the point_cloud_transport plugin

You can see a practical example in [test/test_plugin_publisher.cpp](test/test_plugin_publisher.cpp) and [test/test_plugin_subscriber.cpp](test/test_plugin_subscriber.cpp)

## Use CloudiniSubscriberPCL (when using PCL)

A special subscriber is provided to subscribe to a topic with type `point_cloud_interfaces/msg/CompressedPointCloud2` and convert
its content to `pcl::PointCloud2` automatically.

See example [test/test_cloudini_subscriber.cpp](test/test_cloudini_subscriber.cpp)

Alternaively, you can use the code as a template to see how convertion is implemented and crate your own version.

Example:

1. Publish a regular pointcloud from a rosbag:

```
# Assuming that this rosbag contains a topic called "/points"
ros2 bag play -l my_rosbag/
```

2. Run **topic_converter** as mentioned above to create a topic called "/points/compressed":

```
ros2 run cloudini_ros cloudini_topic_converter --ros-args \
    -p compressing:=true  \
    -p topic_input:=/points  \
    -p topic_output:=/points/compressed
```

3. Subscribe using **test_cloudini_subscriber**:

```
ros2 run cloudini_ros test_cloudini_subscriber --ros-args -p topic:=/points/compressed
```
