#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>

#include "cloudini/field_decoder.hpp"
#include "cloudini/field_encoder.hpp"

TEST(FieldEncoders, IntField) {
  const size_t kNumpoints = 1000;
  std::vector<uint32_t> input_data(kNumpoints);
  std::vector<uint32_t> output_data(kNumpoints, 0);

  const size_t kBufferSize = kNumpoints * sizeof(uint32_t);

  // create a sequence of random numbers
  std::generate(input_data.begin(), input_data.end(), []() { return std::rand() % 1000; });

  using namespace Cloudini;

  PointField field_info;
  field_info.name = "the_int";
  field_info.offset = 0;
  field_info.type = FieldType::UINT32;
  field_info.resolution = std::nullopt;

  std::vector<uint8_t> buffer(10000 * sizeof(uint32_t));

  //------------- Encode -------------
  {
    FieldEncoderInt<uint32_t> encoder(field_info);
    FieldDecoderInt<uint32_t> decoder(field_info);

    ConstBufferView input_buffer = {reinterpret_cast<const uint8_t*>(input_data.data()), kBufferSize};
    BufferView buffer_data = {buffer.data(), buffer.size()};

    size_t encoded_size = 0;
    for (size_t i = 0; i < kNumpoints; ++i) {
      encoded_size += encoder.encode(input_buffer, buffer_data);
      input_buffer.advance(sizeof(uint32_t));
    }
    buffer.resize(encoded_size);

    std::cout << "Original size: " << kBufferSize << "   encoded size: " << encoded_size << std::endl;
  }

  //------------- Decode -------------
  {
    ConstBufferView buffer_data = {buffer.data(), buffer.size()};
    BufferView output_buffer = {reinterpret_cast<uint8_t*>(output_data.data()), kBufferSize};
    FieldDecoderInt<uint32_t> decoder(field_info);

    for (size_t i = 0; i < kNumpoints; ++i) {
      decoder.decode(buffer_data, output_buffer);
      ASSERT_EQ(input_data[i], output_data[i]) << "Mismatch at index " << i;
    }
  }
}