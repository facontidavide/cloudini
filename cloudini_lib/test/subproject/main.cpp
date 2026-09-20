#include "cloudini_lib/cloudini.hpp"

int main() {
  Cloudini::EncodingInfo info;
  info.fields = {{"x", 0, Cloudini::FieldType::FLOAT32, 0.001f}};
  info.point_step = 4;
  Cloudini::PointcloudEncoder encoder(info);
  return 0;
}
