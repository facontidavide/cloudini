[![Ubuntu](https://github.com/facontidavide/cloudini/actions/workflows/ubuntu-build.yaml/badge.svg)](https://github.com/facontidavide/cloudini/actions/workflows/ubuntu-build.yaml)
[![ROS2 Humble](https://github.com/facontidavide/cloudini/actions/workflows/ros-humble.yaml/badge.svg)](https://github.com/facontidavide/cloudini/actions/workflows/ros-humble.yaml)
[![ROS2 Jazzy](https://github.com/facontidavide/cloudini/actions/workflows/ros-jazzy.yaml/badge.svg)](https://github.com/facontidavide/cloudini/actions/workflows/ros-jazzy.yaml)

![Cloudini](logo.png)

**Cloudini** (pronounced with Italian accent) is a pointcloud compression
library.

Its main focus is speed, but it still achieves very good compression ratios.

Its main use cases are:

- To improve the storage of datasets containing pointcloud data (being a notable example **rosbags**).

- Decrease the bandwidth used when streaming pointclouds over a network.

It works seamlessly with [PCL](https://pointclouds.org/) and
[ROS](https://www.ros.org/), but the main library can be compiled and used independently, if needed.

# What to expect

The compression ratio is hard to predict because it depends on the way the original data is encoded.

For example, ROS pointcloud messages are extremely inefficient, because
they include some "padding" in the message that, in extreme cases, may reach up to 50%.

(Yes, you heard correctly, almost 50% of that 10 Gb rosbag is useless padding).

But, in general, you may expect considerably **better compression and faster encoding/decoding**  than ZSTD or LZ4 alone.

These are two random examples using real-world data from LiDARs.

- **Channels**: XYZ, Intensity, no padding

```
  [LZ4 only]      ratio: 0.77 time (usec): 2165
  [ZSTD only]     ratio: 0.68 time (usec): 2967
  [Cloudini-LZ4]  ratio: 0.56 time (usec): 1254
  [Cloudini-ZSTD] ratio: 0.51 time (usec): 1576
```

- **Channels**: XYZ, intensity, ring (int16), timestamp (double), with padding

```
  [LZ4 only]      ratio: 0.31 time (usec): 2866
  [ZSTD only]     ratio: 0.24 time (usec): 3423
  [Cloudini-LZ4]  ratio: 0.16 time (usec): 2210
  [Cloudini-ZSTD] ratio: 0.14 time (usec): 2758
```

If you are a ROS user, you can test the compression ratio and speed yourself,
running the application `rosbag_benchmark` on any rosbag containing a `sensor_msgs::msg::PointCloud2` topic.

# How to test it yourself

There is a pre-compiled Linux [AppImage](https://appimage.org/) that can be downloaded in the
[release page](https://github.com/facontidavide/cloudini/releases/latest)

Alternatively, you can test the obtainable compression ratio in your browser here: https://cloudini.netlify.app/

NOTE: your data will **not** be uploaded to the cloud. The applications runs 100% inside your browser.

[![cloudini_web.png](cloudini_web.png)](https://cloudini.netlify.app/)

# How it works

The algorithm contains two steps:

1. Encoding the pointcloud, channel by channel.
2. Compression using either [LZ4](https://github.com/lz4/lz4) or [ZSTD](https://github.com/facebook/zstd).

The encoding is lossy for floating point channels (typically the X, Y, Z channels)
and lossless for RGBA and integer channels.

Now, I know that when you read the word "lossy" you may think about grainy JPEGS images. **Don't**.

The encoder applies a quantization using a resolution provided by the user.

Typical LiDARs have an accuracy/noise in the order of +/- 1 cm.
Therefore, using a resolution of **1 mm** (+/- 0.5 mm max quantization error) is usually a very conservative option.

It should also be noted that this two-step compression strategy has a
negative overhead, i.e. it is actually **faster** than using LZ4 or ZSTD alone.


# Compile instructions

Some dependencies are downloaded automatically using [CPM](https://github.com/cpm-cmake/CPM.cmake).
To avoid downloading them again when your rebuild your project, I suggest setting **CPM_SOURCE_CACHE** as described [here](https://github.com/cpm-cmake/CPM.cmake).

To build the main library (`cloudini_lib`)

```
cmake -B build -S cloudini_lib -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

To compile it with ROS, just pull this repo into your **ws/src** folder and execute `colcon build` as usual.


# ROS specific utilities

For more information, see the [cloudini_ros/README.md](cloudini_ros/README.md)

- **point_cloud_transport plugins**: see [point_cloud_transport plugins](https://github.com/ros-perception/point_cloud_transport_plugins) for reference about how they are used.

- **cloudini_topic_converter**: a node that subscribes to a compressed `point_cloud_interfaces/CompressedPointCloud2` and publishes a `sensor_msgs/PointCloud2`.

- **cloudini_rosbag_converter**: a command line tool that, given a rosbag (limited to MCAP format), converts all `sensor_msgs/PointCloud2` topics into compressed `point_cloud_interfaces/CompressedPointCloud2` of vice-versa.

# Frequently Asked Questions

### I want to record "raw data". Since Cloudini is "lossy", I think i should not use it...

I don't agree: you are working with noisy data in the first place.

Furthermore, I am pretty sure that your pointcloud processing algorithm is applying some sort of Voxel-based downsampling much larger
than the quantization applied by this library.

If you keep the the quantization error low enough, it will not affect your results in any meaningful way.

### So, which resolution do you recommend?

Look at the specifications of your sensor and use that value as a reference.

Considering that LiDArs accuracy is usually in the order of **+/- 1 cm** and that the resoultion used in Cloudini is in meters:

- If the goal of the recorded pointcloud is to do visualization, use a resolution of **0.01 (1 cm)**.
- If you want to record "raw data", a resolution of **0.001 (1mm)** will be more than enough.
- If you are stubborn, and you don't believe a single word I said, use a resolution of **0.0001 (100 microns)**.
  But you are being paranoid...

### How does it perform, compared to Draco?

[Google Draco](https://github.com/google/draco) has two main encoding methods: SEQUENTIAL and KD_TREE.

The latter could achieve excellent compression ratios, but it is very sloooow and it doesn't preserve the original order
of the points in the point cloud.

Compared with the Draco sequential mode, Cloudini achieves approximately the same compression, but is considerably faster in
my (currently limited) benchmark.

### Does the decoder need to know the parameters used while encoding?

No, that information is stored in the header of the compressed data, and the decoder will automatically select the right
decompression algorithm.
