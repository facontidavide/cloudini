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

#pragma once

#include <QByteArray>
#include <mcap/writer.hpp>

class ByteArrayInterface : public mcap::IWritable {
 public:
  ByteArrayInterface(){};
  virtual ~ByteArrayInterface() = default;

  void end() override {
    bytes_.end();
  };

  uint64_t size() const override {
    return bytes_.size();
  };

  const QByteArray& byteArray() const {
    return bytes_;
  }

 protected:
  void handleWrite(const std::byte* data, uint64_t size) override {
    bytes_.append(reinterpret_cast<const char*>(data), size);
  }

  QByteArray bytes_;
};
