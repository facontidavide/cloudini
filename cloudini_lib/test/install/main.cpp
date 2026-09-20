#include <cloudini_lib/cloudini.hpp>

int main() {
  using namespace Cloudini;
  const std::vector<uint8_t> input{1, 2, 3, 4};
  for (auto compression : {CompressionOption::LZ4, CompressionOption::ZSTD}) {
    EncodingInfo info;
    info.width = input.size();
    info.height = 1;
    info.point_step = 1;
    info.fields.push_back({"value", 0, FieldType::UINT8, std::nullopt});
    info.compression_opt = compression;
    PointcloudEncoder encoder(info);
    std::vector<uint8_t> encoded, decoded;
    encoder.encode(ConstBufferView(input.data(), input.size()), encoded);
    ConstBufferView payload(encoded.data(), encoded.size());
    const auto header = DecodeHeader(payload);
    PointcloudDecoder decoder;
    decoder.decode(header, payload, decoded);
    if (decoded != input) {
      return 1;
    }
  }
}
