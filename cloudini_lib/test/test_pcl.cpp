#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cloudini_lib/cloudini.hpp"
#include "cloudini_lib/pcl_conversion.hpp"
#include "data_path.hpp"

using namespace Cloudini;

TEST(Cloudini, PCD_Encode) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  const std::string filepath = Cloudini::tests::DATA_PATH + "lidar.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(filepath, cloud) == -1) {
    throw std::runtime_error(std::string("Failed to load PCD file:") + filepath);
  }

  const float resolution = 0.001f;

  EncodingInfo info;
  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = sizeof(pcl::PointXYZI);
  info.encoding_opt = EncodingOptions::LOSSY;
  info.compression_opt = CompressionOption::ZSTD;
  info.fields.push_back({"x", 0, FieldType::FLOAT32, resolution});
  info.fields.push_back({"y", 4, FieldType::FLOAT32, resolution});
  info.fields.push_back({"z", 8, FieldType::FLOAT32, resolution});
  info.fields.push_back({"intensity", 16, FieldType::FLOAT32, resolution});

  std::vector<uint8_t> compressed_data;

  {
    PointcloudEncoder encoder(info);
    ConstBufferView cloud_points_view(cloud.points.data(), cloud.points.size() * sizeof(pcl::PointXYZI));
    encoder.encode(cloud_points_view, compressed_data);

    auto ratio = static_cast<float>(compressed_data.size()) / static_cast<float>(cloud_points_view.size());
    std::cout << "Compressed data size: " << compressed_data.size() << " percent:" << 100.0 * ratio << std::endl;
  }

  ConstBufferView compressed_view(compressed_data.data(), compressed_data.size());
  auto info_decoded = DecodeHeader(compressed_view);

  ASSERT_EQ(info_decoded.width, info.width);
  ASSERT_EQ(info_decoded.height, info.height);
  ASSERT_EQ(info_decoded.point_step, info.point_step);
  ASSERT_EQ(info_decoded.encoding_opt, info.encoding_opt);
  ASSERT_EQ(info_decoded.compression_opt, info.compression_opt);
  ASSERT_EQ(info_decoded.fields.size(), info.fields.size());
  for (size_t i = 0; i < info.fields.size(); ++i) {
    ASSERT_EQ(info_decoded.fields[i].name, info.fields[i].name);
    ASSERT_EQ(info_decoded.fields[i].offset, info.fields[i].offset);
    ASSERT_EQ(info_decoded.fields[i].type, info.fields[i].type);
    ASSERT_EQ(info_decoded.fields[i].resolution, info.fields[i].resolution);
  }

  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  cloud_out.resize(info_decoded.width * info_decoded.height);

  {
    PointcloudDecoder decoder;
    BufferView cloud_points_view(cloud_out.points.data(), cloud_out.points.size() * sizeof(pcl::PointXYZI));
    decoder.decode(info_decoded, compressed_view, cloud_points_view);
  }

  ASSERT_EQ(cloud.points.size(), cloud_out.points.size());

  auto tolerance = resolution * 1.01F;

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    ASSERT_NEAR(cloud.points[i].x, cloud_out.points[i].x, tolerance) << "i:" << i;
    ASSERT_NEAR(cloud.points[i].y, cloud_out.points[i].y, tolerance) << "i:" << i;
    ASSERT_NEAR(cloud.points[i].z, cloud_out.points[i].z, tolerance) << "i:" << i;
    ASSERT_NEAR(cloud.points[i].intensity, cloud_out.points[i].intensity, tolerance) << "i:" << i;
  }
}

TEST(Cloudini, PCL_Conversion) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  const std::string filepath = Cloudini::tests::DATA_PATH + "lidar.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(filepath, cloud) == -1) {
    throw std::runtime_error(std::string("Failed to load PCD file:") + filepath);
  }

  const float resolution = 0.001f;

  std::vector<uint8_t> serialized_data;
  PointcloudEncode(cloud, serialized_data, 0.001);

  const auto tolerance = resolution * 1.01F;

  {
    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    PointcloudDecode(serialized_data, cloud_out);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      ASSERT_NEAR(cloud.points[i].x, cloud_out.points[i].x, tolerance) << "i:" << i;
      ASSERT_NEAR(cloud.points[i].y, cloud_out.points[i].y, tolerance) << "i:" << i;
      ASSERT_NEAR(cloud.points[i].z, cloud_out.points[i].z, tolerance) << "i:" << i;
      ASSERT_NEAR(cloud.points[i].intensity, cloud_out.points[i].intensity, tolerance) << "i:" << i;
    }
  }

  {
    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    PointcloudDecode(serialized_data, cloud_out);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      ASSERT_NEAR(cloud.points[i].x, cloud_out.points[i].x, tolerance) << "i:" << i;
      ASSERT_NEAR(cloud.points[i].y, cloud_out.points[i].y, tolerance) << "i:" << i;
      ASSERT_NEAR(cloud.points[i].z, cloud_out.points[i].z, tolerance) << "i:" << i;
    }
  }
}
