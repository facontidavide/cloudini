#pragma once

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cloudini_lib/cloudini.hpp>

namespace Cloudini {

EncodingInfo ConvertToEncodingInfo(const pcl::PCLPointCloud2& cloud, double resolution_XYZ);

template <typename PointT>
EncodingInfo ConvertToEncodingInfo(const pcl::PointCloud<PointT>& cloud, double resolution_XYZ);

// specializations for different point types

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZ>(  //
    const pcl::PointCloud<pcl::PointXYZ>& cloud, double resolution_XYZ);

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZI>(  //
    const pcl::PointCloud<pcl::PointXYZI>& cloud, double resolution_XYZ);

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZRGBA>(  //
    const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, double resolution_XYZ);

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZRGB>(  //
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud, double resolution_XYZ);

template <>
EncodingInfo ConvertToEncodingInfo<pcl::Normal>(  //
    const pcl::PointCloud<pcl::Normal>& cloud, double resolution_XYZ);

//-------------------------------------------------------------------

size_t PointcloudEncode(
    const pcl::PCLPointCloud2& cloud, std::vector<uint8_t>& serialized_cloud, double resolution_XYZ);

template <typename PointT>
size_t PointcloudEncode(
    const pcl::PointCloud<PointT>& cloud, std::vector<uint8_t>& serialized_cloud, double resolution_XYZ);

size_t PointcloudDecode(ConstBufferView serialized_data, pcl::PCLPointCloud2& cloud);

template <typename PointT>
size_t PointcloudDecode(ConstBufferView serialized_data, pcl::PointCloud<PointT>& cloud);

//-------------------------------------------------------------------
//-------------------------------------------------------------------

// template <typename PointT>
// inline size_t PointcloudEncode(
//     const pcl::PointCloud<PointT>& cloud, std::vector<uint8_t>& serialized_cloud, double resolution_XYZ) {
//   // get the encoding info
//   EncodingInfo info = ConvertToEncodingInfo(cloud, resolution_XYZ);
//   // size in bytes of the data
//   info.decoded_size = cloud.data.size() * sizeof(PointT);
//   serialized_cloud.resize(ComputeHeaderSize(info.fields) + info.decoded_size);

//   ConstBufferView data_view(reinterpret_cast<const uint8_t*>(cloud.data.data()), cloud.data.size());
//   BufferView output_view(serialized_cloud.data(), serialized_cloud.size());

//   return PointcloudEncode(info, data_view, output_view);
// }

// template <typename PointT>
// inline size_t PointcloudDecode(ConstBufferView serialized_data, pcl::PointCloud<PointT>& cloud) {
//   // decode the header
//   EncodingInfo info = DecodeHeader(serialized_data);

//   // resize the cloud to the decoded size
//   cloud.data.resize(info.width, info.width * info.height);
//   cloud.width = info.width;
//   cloud.height = info.height;

//   // decode the data
//   BufferView output_view(reinterpret_cast<uint8_t*>(cloud.data.data()), cloud.data.size());
//   return PointcloudDecode(serialized_data, output_view);
// }

};  // namespace Cloudini
