
![Cloudini](logo.png)

**Cloudini** (pronounced with Italian accent) is a pointcloud compression
library.

Its main focus is speed, but it still achieve very good compression ratios.

Its main use cases are:

- To improve the storage of datasets containing pointcloud data (being a notable example **rosbags**).

- Decrease the bandwith used when of streaming pointclouds over a newtwork.

It works seamlessly with [PCL](https://pointclouds.org/) and 
[ROS](https://www.ros.org/), but the main library can be compiled and used independently, if needed.

# What to expect 

The compression ratio is hard the predict, because it depends on the way
the original data is encoded.

For example, ROS pointcloud messages are extremely inefficient, because
the include some "padding" in the message that in extreme cases such as 
XYZI clouds, may reach 50%.

(Yes, you heard correctly, almost 50% of that 10 Gb rosbag is useless padding).

But, in general, you may expect considerably **better compression and faster encoding/decoding**  than ZTD or LZ4 alone.

# How it works

Compression happens in 2 steps:

1. Encoding the pointcloud, channel by channel.
2. Compression using either [LZ4](https://github.com/lz4/lz4) or [ZSTD](https://github.com/facebook/zstd).

The encoding is lossy for floating point channels (typically the X, Y, Z channels) 
and lossless for RGBD and integer channels.

Now, I know that when you read the word "lossy" you may think about grainy JPEGS images. **Don't**.

The encoder apply a quantization using a resolution provided by the user.

Typical LIDAR sensors have an accuracy/noise is in the order of 1/2 cm,
therefore using a resolution of **1 mm** is very conservative.

But if you are paranoid, and decide to use a resolution of **100 microns**, you still get really good compression ratios!

It should also be noted that these two steps compression strategy has a
negative overhead, i.e. it is actually **faster** than using LZ4 or ZSTD alone.


# For ROS users

You can test the compression ration and speed yourself, running the application
`rosbag_benchmark` on any rosbag containing a `sensor_msgs::msg::PointCloud2`.

Make sure that the application is compiled using `-DCMAKE_BUILD_TYPE=Release`.

Typical output will look like this:

```
Found PointCloud2 topic: /points

Topic: /points
  Count: 1774
  [LZ4 only]      ratio: 0.37 time (usec): 2718
  [ZSTD only]     ratio: 0.29 time (usec): 3250
  [Cloudini-LZ4]  ratio: 0.23 time (usec): 2089
  [Cloudini-ZSTD] ratio: 0.20 time (usec): 2599
```
