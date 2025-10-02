// Copyright (c) 2025 Davide Faconti
//
// This file is part of Cloudini.
//
// Licensed under the FSL-1.1-MIT License.
// You may obtain a copy of the License at
// https://fsl.software/
//
// Two years from the release date of this software, you may use
// this file in accordance with the MIT License, as described in
// the LICENSE file in the root of this repository.

#include "cloudini_lib/pcl_conversion.hpp"

#include <pcl/io/pcd_io.h>

#include <sstream>

namespace Cloudini {

std::vector<std::string_view> splitString(std::string_view str, char delimiter = ' ') {
  std::vector<std::string_view> tokens;
  std::string_view::size_type start = 0;
  std::string_view::size_type end = 0;

  while ((end = str.find(delimiter, start)) != std::string_view::npos) {
    tokens.push_back(str.substr(start, end - start));
    start = end + 1;
  }
  tokens.push_back(str.substr(start));
  return tokens;
}

// https://pointclouds.org/documentation/tutorials/pcd_file_format.html
std::map<std::string, FieldType> ReadEncodingInfoFromPCD(std::istream& stream) {
  std::string line;
  std::vector<std::string> fields_name;
  std::vector<std::string> fields_size;
  std::vector<std::string> fields_type;

  do {
    std::getline(stream, line);
    line = line.substr(0, line.find('#'));   // Remove comments
    line = line.substr(0, line.find('\r'));  // Remove carriage return if present
    line = line.substr(0, line.find('\n'));  // Remove newline if present

    auto split_line = splitString(line);
    if (split_line.empty()) {
      continue;
    }
    const auto& line_key = split_line[0];
    if (line_key == "FIELDS") {
      fields_name.assign(split_line.begin() + 1, split_line.end());
    } else if (line_key == "SIZE") {
      fields_size.assign(split_line.begin() + 1, split_line.end());
    } else if (line_key == "TYPE") {
      fields_type.assign(split_line.begin() + 1, split_line.end());
    } else if (line_key == "DATA") {
      break;
    }
  } while (!line.empty());

  const size_t num_fields = fields_name.size();

  if (num_fields != fields_size.size() || num_fields != fields_type.size()) {
    throw std::runtime_error("PCD header parsing error: FIELDS, SIZE and TYPE must have the same number of elements");
  }

  std::map<std::string, FieldType> types_map;

  for (size_t i = 0; i < num_fields; ++i) {
    const std::string name = std::string(fields_name[i]);
    const int bytes = std::stoi(std::string(fields_size[i]));
    FieldType type = FieldType::UNKNOWN;
    if (fields_type[i] == "F" && bytes == 4) {
      if (bytes == 4) {
        type = FieldType::FLOAT32;
      } else if (bytes == 8) {
        type = FieldType::FLOAT64;
      }
    } else if (fields_type[i] == "I") {
      if (bytes == 1) {
        type = FieldType::INT8;
      } else if (bytes == 2) {
        type = FieldType::INT16;
      } else if (bytes == 4) {
        type = FieldType::INT32;
      }
    } else if (fields_type[i] == "U") {
      if (bytes == 1) {
        type = FieldType::UINT8;
      } else if (bytes == 2) {
        type = FieldType::UINT16;
      } else if (bytes == 4) {
        type = FieldType::UINT32;
      } else if (bytes == 8) {
        type = FieldType::UINT64;
      }
    }
    types_map[name] = type;
  }
  return types_map;
}

bool isSameEncodingInfo(const EncodingInfo& info1, const EncodingInfo& info2) {
  if (info1.fields.size() != info2.fields.size()) {
    return false;
  }

  for (size_t i = 0; i < info1.fields.size(); ++i) {
    if (info1.fields[i].name != info2.fields[i].name ||      //
        info1.fields[i].offset != info2.fields[i].offset ||  //
        info1.fields[i].type != info2.fields[i].type) {
      return false;
    }
  }

  return info1.width == info2.width &&    //
         info1.height == info2.height &&  //
         info1.point_step == info2.point_step;
}

EncodingInfo ConvertToEncodingInfo(const pcl::PCLPointCloud2& cloud, double resolution_XYZ) {
  EncodingInfo info;

  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = cloud.point_step;

  for (size_t i = 0; i < cloud.fields.size(); ++i) {
    const pcl::PCLPointField& field = cloud.fields[i];
    PointField point_field;
    point_field.name = field.name;
    point_field.offset = field.offset;

    switch (field.datatype) {
      case pcl::PCLPointField::FLOAT32:
        point_field.type = FieldType::FLOAT32;
        break;
      case pcl::PCLPointField::FLOAT64:
        point_field.type = FieldType::FLOAT64;
        break;
      case pcl::PCLPointField::INT8:
        point_field.type = FieldType::INT8;
        break;
      case pcl::PCLPointField::INT16:
        point_field.type = FieldType::INT16;
        break;
      case pcl::PCLPointField::INT32:
        point_field.type = FieldType::INT32;
        break;
      case pcl::PCLPointField::UINT8:
        point_field.type = FieldType::UINT8;
        break;
      case pcl::PCLPointField::UINT16:
        point_field.type = FieldType::UINT16;
        break;
      case pcl::PCLPointField::UINT32:
        point_field.type = FieldType::UINT32;
        break;
      default:
        point_field.type = FieldType::UNKNOWN;
        break;
    }

    // If the field is a FLOAT32 and has a resolution, set it
    if (point_field.type == FieldType::FLOAT32 && resolution_XYZ > 0.0) {
      point_field.resolution = resolution_XYZ;
    } else {
      point_field.resolution = std::nullopt;
    }

    info.fields.push_back(point_field);
  }
  return info;
}

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>& cloud, double resolution_XYZ) {
  EncodingInfo info;
  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = sizeof(pcl::PointXYZ);
  info.fields.push_back(PointField{"x", 0, FieldType::FLOAT32, resolution_XYZ});
  info.fields.push_back(PointField{"y", 4, FieldType::FLOAT32, resolution_XYZ});
  info.fields.push_back(PointField{"z", 8, FieldType::FLOAT32, resolution_XYZ});
  return info;
}

template <>
EncodingInfo ConvertToEncodingInfo<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>& cloud, double resolution_XYZ) {
  EncodingInfo info;
  info.width = cloud.width;
  info.height = cloud.height;
  info.point_step = sizeof(pcl::PointXYZI);
  info.fields.push_back(PointField{"x", 0, FieldType::FLOAT32, resolution_XYZ});
  info.fields.push_back(PointField{"y", 4, FieldType::FLOAT32, resolution_XYZ});
  info.fields.push_back(PointField{"z", 8, FieldType::FLOAT32, resolution_XYZ});

  pcl::PointXYZI dummy;
  const size_t intensity_offset = reinterpret_cast<uint8_t*>(&dummy.intensity) - reinterpret_cast<uint8_t*>(&dummy.x);
  info.fields.push_back(
      PointField{"intensity", static_cast<uint32_t>(intensity_offset), FieldType::FLOAT32, std::nullopt});
  return info;
}

size_t PCLPointCloudEncode(
    const pcl::PCLPointCloud2& cloud, std::vector<uint8_t>& serialized_cloud, double resolution_XYZ) {
  // get the encoding info
  EncodingInfo info = ConvertToEncodingInfo(cloud, resolution_XYZ);
  PointcloudEncoder encoder(info);
  ConstBufferView data_view(cloud.data.data(), cloud.data.size());
  return encoder.encode(data_view, serialized_cloud);
}

size_t PCLPointCloudEncode(
    const std::filesystem::path& input_pcd_file, std::vector<uint8_t>& serialized_cloud, double resolution_XYZ) {
  pcl::PCLPointCloud2 cloud;
  if (pcl::io::loadPCDFile(input_pcd_file.string(), cloud) == -1) {
    throw std::runtime_error("Error: Failed to load PCD file: " + input_pcd_file.string());
  }
  std::ifstream pcd_file_stream(input_pcd_file, std::ios::in);
  const auto pcd_fields_map = ReadEncodingInfoFromPCD(pcd_file_stream);

  EncodingInfo info = ConvertToEncodingInfo(cloud, resolution_XYZ);
  // fix the field types if necessary
  for (auto& field : info.fields) {
    if (field.type == FieldType::UNKNOWN) {
      field.type = pcd_fields_map.at(field.name);
    }
  }
  PointcloudEncoder encoder(info);
  ConstBufferView data_view(cloud.data.data(), cloud.data.size());
  return encoder.encode(data_view, serialized_cloud);
}

void PCLPointCloudDecode(ConstBufferView serialized_data, pcl::PCLPointCloud2& cloud) {
  // decode the header
  EncodingInfo header_info = DecodeHeader(serialized_data);

  // Set cloud metadata
  cloud.width = header_info.width;
  cloud.height = header_info.height;
  cloud.point_step = header_info.point_step;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  // Resize data
  cloud.data.resize(cloud.width * cloud.height * cloud.point_step);

  // Convert fields
  cloud.fields.clear();
  for (const auto& field : header_info.fields) {
    pcl::PCLPointField pcl_field;
    pcl_field.name = field.name;
    pcl_field.offset = field.offset;
    pcl_field.datatype = static_cast<uint8_t>(field.type);
    pcl_field.count = 1;
    cloud.fields.push_back(pcl_field);
  }

  // decode the data
  BufferView output_view(cloud.data.data(), cloud.data.size());
  PointcloudDecoder decoder;
  decoder.decode(header_info, serialized_data, output_view);
}

}  // namespace Cloudini
