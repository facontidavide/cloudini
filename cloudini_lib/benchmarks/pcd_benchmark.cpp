
#include <benchmark/benchmark.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cloudini/cloudini.hpp"
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
  info.firts_stage = Cloudini::FirstStageOpt::LOSSY;
  info.second_stage = Cloudini::SecondStageOpt::ZSTD;
  info.point_step = sizeof(pcl::PointXYZI);
  info.fields.push_back(Cloudini::PointField{"x", 0, Cloudini::FieldType::FLOAT32, 0.001});
  info.fields.push_back(Cloudini::PointField{"y", 4, Cloudini::FieldType::FLOAT32, 0.001});
  info.fields.push_back(Cloudini::PointField{"z", 8, Cloudini::FieldType::FLOAT32, 0.001});
  pcl::PointXYZI p;
  uint32_t intensity_offset = reinterpret_cast<const uint8_t*>(&p.intensity) - reinterpret_cast<const uint8_t*>(&p);
  info.fields.push_back(Cloudini::PointField{"intensity", intensity_offset, Cloudini::FieldType::FLOAT32, 0.001});
  return info;
}

static void PCD_Encode(benchmark::State& state) {
  using namespace Cloudini;

  auto cloud = loadCloud();

  EncodingInfo info = defaultEncodingInfo(cloud);
  PointcloudEncoder encoder(info);

  std::vector<uint8_t> output;
  ConstBufferView cloud_data(
      reinterpret_cast<const uint8_t*>(cloud.points.data()), cloud.points.size() * sizeof(pcl::PointXYZI));

  for (auto _ : state) {
    encoder.encode(cloud_data, output);
  }
  const auto percentage = 100 * (static_cast<double>(output.size()) / static_cast<double>(cloud_data.size));
  std::cout << "Encoded size: " << output.size() << "  percentage: " << percentage << "%" << std::endl;
}

BENCHMARK(PCD_Encode);

//------------------------------------------------------------------------------------------
static void PCD_ZSTD_only(benchmark::State& state) {
  using namespace Cloudini;

  auto cloud = loadCloud();
  EncodingInfo info = defaultEncodingInfo(cloud);
  info.firts_stage = Cloudini::FirstStageOpt::NONE;
  info.second_stage = Cloudini::SecondStageOpt::ZSTD;
  PointcloudEncoder encoder(info);

  std::vector<uint8_t> output;
  ConstBufferView cloud_data(
      reinterpret_cast<const uint8_t*>(cloud.points.data()), cloud.points.size() * sizeof(pcl::PointXYZI));

  for (auto _ : state) {
    encoder.encode(cloud_data, output);
  }
  const auto percentage = 100 * (static_cast<double>(output.size()) / static_cast<double>(cloud_data.size));
  std::cout << "Encoded size: " << output.size() << "  percentage: " << percentage << "%" << std::endl;
}

BENCHMARK(PCD_ZSTD_only);

//------------------------------------------------------------------------------------------
BENCHMARK_MAIN();