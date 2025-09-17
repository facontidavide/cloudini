#include "cloudini_lib/pcl_conversion.hpp"

namespace Cloudini {

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

size_t PointcloudEncode(
    const pcl::PCLPointCloud2& cloud, std::vector<uint8_t>& serialized_cloud, double resolution_XYZ) {
  // get the encoding info
  EncodingInfo info = ConvertToEncodingInfo(cloud, resolution_XYZ);
  PointcloudEncoder encoder(info);
  ConstBufferView data_view(cloud.data.data(), cloud.data.size());
  return encoder.encode(data_view, serialized_cloud);
}

void PointcloudDecode(ConstBufferView serialized_data, pcl::PCLPointCloud2& cloud) {
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
