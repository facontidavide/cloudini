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
    point_field.type = static_cast<FieldType>(field.datatype);

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
  info.fields.push_back(PointField{"x", 0, FieldType::FLOAT32, resolution_XYZ});
  info.fields.push_back(PointField{"y", 4, FieldType::FLOAT32, resolution_XYZ});
  info.fields.push_back(PointField{"z", 8, FieldType::FLOAT32, resolution_XYZ});

  pcl::PointXYZI dummy;
  const size_t intensity_offset = reinterpret_cast<uint8_t*>(&dummy.intensity) - reinterpret_cast<uint8_t*>(&dummy.x);
  info.fields.push_back(PointField{"intensity", intensity_offset, FieldType::FLOAT32, std::nullopt});
  return info;
}

}  // namespace Cloudini
