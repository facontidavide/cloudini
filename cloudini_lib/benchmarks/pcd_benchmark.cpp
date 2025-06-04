#include <benchmark/benchmark.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifdef DRACO_FOUND
#include <draco/compression/decode.h>
#include <draco/compression/encode.h>
#include <draco/compression/expert_encode.h>
#include <draco/point_cloud/point_cloud_builder.h>
#endif

#include "cloudini_lib/cloudini.hpp"
#include "data_path.hpp"

static pcl::PointCloud<pcl::PointXYZI> loadCloud() {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  const std::string filepath = Cloudini::tests::DATA_PATH + "lidar.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(filepath, cloud) == -1) {
    throw std::runtime_error(std::string("Failed to load PCD file:") + filepath);
  }
  return cloud;
}

static Cloudini::EncodingInfo defaultEncodingInfo(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
  Cloudini::EncodingInfo info;
  info.width = cloud.width;
  info.height = cloud.height;
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
  info.compression_opt = Cloudini::CompressionOption::ZSTD;
  info.point_step = sizeof(pcl::PointXYZI);
  info.fields.push_back(Cloudini::PointField{"x", 0, Cloudini::FieldType::FLOAT32, 0.001});
  info.fields.push_back(Cloudini::PointField{"y", 4, Cloudini::FieldType::FLOAT32, 0.001});
  info.fields.push_back(Cloudini::PointField{"z", 8, Cloudini::FieldType::FLOAT32, 0.001});
  pcl::PointXYZI p;
  uint32_t intensity_offset = reinterpret_cast<const uint8_t*>(&p.intensity) - reinterpret_cast<const uint8_t*>(&p);
  info.fields.push_back(Cloudini::PointField{"intensity", intensity_offset, Cloudini::FieldType::FLOAT32, 0.001});
  return info;
}

static void PCD_Encode_Impl(
    benchmark::State& state, const pcl::PointCloud<pcl::PointXYZI>& cloud, const Cloudini::EncodingInfo& info) {
  using namespace Cloudini;
  std::vector<uint8_t> output;

  PointcloudEncoder encoder(info);

  const ConstBufferView cloud_data(cloud.points.data(), cloud.points.size() * sizeof(pcl::PointXYZI));
  for (auto _ : state) {
    encoder.encode(cloud_data, output);
  }
  const auto percentage = 100 * (static_cast<double>(output.size()) / static_cast<double>(cloud_data.size()));
  std::cout << "Encoded size: " << output.size() << "  percentage: " << percentage << "%" << std::endl;
}

static void PCD_Decode_Impl(
    benchmark::State& state, const pcl::PointCloud<pcl::PointXYZI>& cloud, const Cloudini::EncodingInfo& info) {
  using namespace Cloudini;
  std::vector<uint8_t> compressed_vect;

  PointcloudEncoder encoder(info);
  ConstBufferView cloud_data(cloud.points.data(), cloud.points.size() * sizeof(pcl::PointXYZI));
  encoder.encode(cloud_data, compressed_vect);

  PointcloudDecoder decoder;

  pcl::PointCloud<pcl::PointXYZI> cloud2;
  cloud2.resize(cloud.size());

  for (auto _ : state) {
    ConstBufferView compressed_view(compressed_vect.data(), compressed_vect.size());
    BufferView cloud_out(cloud2.points.data(), cloud2.points.size() * sizeof(pcl::PointXYZI));

    auto header_info = Cloudini::DecodeHeader(compressed_view);
    decoder.decode(header_info, compressed_view, cloud_out);
  }
}

//------------------------------------------------------------------------------------------
static void PCD_Encode_Lossy_ZST(benchmark::State& state) {
  const auto cloud = loadCloud();
  auto info = defaultEncodingInfo(cloud);
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
  info.compression_opt = Cloudini::CompressionOption::ZSTD;
  PCD_Encode_Impl(state, cloud, info);
}

static void PCD_Decode_Lossy_ZST(benchmark::State& state) {
  const auto cloud = loadCloud();
  auto info = defaultEncodingInfo(cloud);
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
  info.compression_opt = Cloudini::CompressionOption::ZSTD;
  PCD_Decode_Impl(state, cloud, info);
}
//------------------------------------------------------------------------------------------
static void PCD_Encode_Lossles_ZST(benchmark::State& state) {
  const auto cloud = loadCloud();
  auto info = defaultEncodingInfo(cloud);
  info.encoding_opt = Cloudini::EncodingOptions::LOSSLES;
  info.compression_opt = Cloudini::CompressionOption::ZSTD;
  PCD_Encode_Impl(state, cloud, info);
}

static void PCD_Encode_Lossles_LZ4(benchmark::State& state) {
  const auto cloud = loadCloud();
  auto info = defaultEncodingInfo(cloud);
  info.encoding_opt = Cloudini::EncodingOptions::LOSSLES;
  info.compression_opt = Cloudini::CompressionOption::LZ4;
  PCD_Encode_Impl(state, cloud, info);
}

//------------------------------------------------------------------------------------------
static void PCD_Encode_Lossy_LZ4(benchmark::State& state) {
  const auto cloud = loadCloud();
  auto info = defaultEncodingInfo(cloud);
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
  info.compression_opt = Cloudini::CompressionOption::LZ4;
  PCD_Encode_Impl(state, cloud, info);
}

static void PCD_Decode_Lossy_LZ4(benchmark::State& state) {
  const auto cloud = loadCloud();
  auto info = defaultEncodingInfo(cloud);
  info.encoding_opt = Cloudini::EncodingOptions::LOSSY;
  info.compression_opt = Cloudini::CompressionOption::LZ4;
  PCD_Decode_Impl(state, cloud, info);
}

//------------------------------------------------------------------------------------------
static void PCD_Encode_ZSTD_only(benchmark::State& state) {
  const auto cloud = loadCloud();
  auto info = defaultEncodingInfo(cloud);
  info.encoding_opt = Cloudini::EncodingOptions::NONE;
  info.compression_opt = Cloudini::CompressionOption::ZSTD;
  PCD_Encode_Impl(state, cloud, info);
}

static void PCD_Decode_ZSTD_only(benchmark::State& state) {
  const auto cloud = loadCloud();
  auto info = defaultEncodingInfo(cloud);
  info.encoding_opt = Cloudini::EncodingOptions::NONE;
  info.compression_opt = Cloudini::CompressionOption::ZSTD;
  PCD_Decode_Impl(state, cloud, info);
}

//------------------------------------------------------------------------------------------
static void PCD_Encode_LZ4_only(benchmark::State& state) {
  const auto cloud = loadCloud();
  auto info = defaultEncodingInfo(cloud);
  info.encoding_opt = Cloudini::EncodingOptions::NONE;
  info.compression_opt = Cloudini::CompressionOption::LZ4;
  PCD_Encode_Impl(state, cloud, info);
}

static void PCD_Decode_LZ4_only(benchmark::State& state) {
  const auto cloud = loadCloud();
  auto info = defaultEncodingInfo(cloud);
  info.encoding_opt = Cloudini::EncodingOptions::NONE;
  info.compression_opt = Cloudini::CompressionOption::LZ4;
  PCD_Decode_Impl(state, cloud, info);
}

//------------------------------------------------------------------------------------------

#ifdef DRACO_FOUND

static void PCD_Encode_Draco(benchmark::State& state) {
  const auto cloud = loadCloud();
  // Create Draco point cloud
  draco::PointCloudBuilder builder;
  builder.Start(cloud.size());

  const int pos_att_id = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
  const int intensity_att_id = builder.AddAttribute(draco::GeometryAttribute::GENERIC, 1, draco::DT_FLOAT32);

  // Add points to the point cloud
  for (draco::PointIndex i(0); i < cloud.size(); ++i) {
    const auto& point = cloud.points[i.value()];
    builder.SetAttributeValueForPoint(pos_att_id, i, &point.x);
    builder.SetAttributeValueForPoint(intensity_att_id, i, &point.intensity);
  }

  std::unique_ptr<draco::PointCloud> draco_pc = builder.Finalize(false);
  if (!draco_pc) {
    throw std::runtime_error("Failed to create Draco point cloud");
  }

  // Encode the point cloud
  draco::ExpertEncoder encoder(*draco_pc);
  encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);
  encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 10);
  encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);

  draco::EncoderBuffer buffer;

  for (auto _ : state) {
    buffer.Clear();
    encoder.EncodeToBuffer(&buffer);
  }
  const auto percentage =
      100 * (static_cast<double>(buffer.size()) / static_cast<double>(cloud.size() * sizeof(pcl::PointXYZI)));
  std::cout << "Encoded size: " << buffer.size() << "  percentage: " << percentage << "%" << std::endl;
}

#endif

//------------------------------------------------------------------------------------------

BENCHMARK(PCD_Encode_Lossy_ZST);
BENCHMARK(PCD_Encode_ZSTD_only);
BENCHMARK(PCD_Encode_Lossy_LZ4);
BENCHMARK(PCD_Encode_LZ4_only);

#ifdef DRACO_FOUND
BENCHMARK(PCD_Encode_Draco);
#endif

BENCHMARK(PCD_Decode_Lossy_ZST);
BENCHMARK(PCD_Decode_Lossy_LZ4);
BENCHMARK(PCD_Decode_ZSTD_only);
BENCHMARK(PCD_Decode_LZ4_only);

BENCHMARK_MAIN();
