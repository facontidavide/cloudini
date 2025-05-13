#include <gtest/gtest.h>

#include "cloudini/cloudini.hpp"

TEST(Cloudini, Header) {
  using namespace Cloudini;

  EncodingInfo header;
  header.width = 10;
  header.height = 20;
  header.point_step = sizeof(float) * 4;
  header.firts_stage = FirstStageOpt::LOSSY;
  header.second_stage = SecondStageOpt::ZSTD;

  header.fields.push_back({"x", 0, FieldType::FLOAT32, 0.01});
  header.fields.push_back({"y", 4, FieldType::FLOAT32, 0.01});
  header.fields.push_back({"z", 8, FieldType::FLOAT32, 0.01});
  header.fields.push_back({"intensity", 12, FieldType::FLOAT32, 0.01});

  std::vector<uint8_t> buffer(ComputeHeaderSize(header.fields));
  BufferView output(buffer.data(), buffer.size());

  size_t encoded_size = EncodeHeader(header, output);

  EXPECT_EQ(encoded_size, buffer.size());

  ConstBufferView input(buffer.data(), buffer.size());
  auto decoded_header = DecodeHeader(input);
  ASSERT_EQ(decoded_header.width, header.width);
  ASSERT_EQ(decoded_header.height, header.height);
  ASSERT_EQ(decoded_header.point_step, header.point_step);
  ASSERT_EQ(decoded_header.firts_stage, header.firts_stage);
  ASSERT_EQ(decoded_header.second_stage, header.second_stage);
  ASSERT_EQ(decoded_header.fields.size(), header.fields.size());
  for (size_t i = 0; i < header.fields.size(); ++i) {
    ASSERT_EQ(decoded_header.fields[i].name, header.fields[i].name);
    ASSERT_EQ(decoded_header.fields[i].offset, header.fields[i].offset);
    ASSERT_EQ(decoded_header.fields[i].type, header.fields[i].type);
    ASSERT_EQ(decoded_header.fields[i].resolution, header.fields[i].resolution);
  }
}