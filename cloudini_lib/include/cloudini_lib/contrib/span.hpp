// Copyright 2025 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

/// @brief A simple span-like class that provides a non-owning view of a buffer (std::span required C++20).
template <typename T>
class Span {
  using mutable_T = std::remove_const_t<T>;

  template <typename A, typename B>
  constexpr static bool is_same() {
    return std::is_same_v<A, B>;
  }

  constexpr static bool is_const() {
    return std::is_const_v<T>;
  }

  constexpr static bool is_single_byte = std::is_same_v<mutable_T, uint8_t> ||        //
                                         std::is_same_v<mutable_T, int8_t> ||         //
                                         std::is_same_v<mutable_T, char> ||           //
                                         std::is_same_v<mutable_T, unsigned char> ||  //
                                         std::is_same_v<mutable_T, std::byte>;

 public:
  Span() = default;
  Span(const Span& other) = default;
  Span(Span&& other) noexcept = default;
  Span& operator=(const Span& other) = default;
  Span& operator=(Span&& other) noexcept = default;
  ~Span() = default;

  Span(T* data, size_t size) : data_(data), size_(size) {}

  Span(void* data, size_t size) : data_(reinterpret_cast<T*>(data)), size_(size) {
    static_assert(is_single_byte, "reinterpret_cast is only allowed for single-byte types");
  }

  Span(const void* data, size_t size) : data_(reinterpret_cast<T*>(data)), size_(size) {
    static_assert(is_single_byte && is_const(), "reinterpret_cast is only allowed for single-byte types");
  }

  template <typename U, typename Alloc, typename = std::enable_if_t<!is_const() && is_same<T, U>()>>
  Span(std::vector<U, Alloc>& vec) : data_(vec.data()), size_(vec.size()) {}

  template <typename U, typename Alloc, typename = std::enable_if_t<is_const() && is_same<mutable_T, U>()>>
  Span(const std::vector<U, Alloc>& vec) : data_(vec.data()), size_(vec.size()) {}

  template <typename U, size_t N, typename = std::enable_if_t<!is_const() && is_same<T, U>()>>
  Span(std::array<mutable_T, N>& vec) : data_(vec.data()), size_(vec.size()) {}

  template <typename U, size_t N, typename = std::enable_if_t<is_const() && is_same<mutable_T, U>()>>
  Span(const std::array<U, N>& vec) : data_(vec.data()), size_(vec.size()) {}

  T* data() const {
    return data_;
  }

  template <typename U = T, typename = std::enable_if_t<!std::is_const_v<U>>>
  T* data() {
    return data_;
  }

  size_t size() const {
    return size_;
  }

  bool empty() const {
    return size_ == 0;
  }

  void trim_front(size_t n) {
    data_ += n;
    size_ -= n;
  }

  void trim_back(size_t n) {
    size_ -= n;
  }

 private:
  T* data_ = nullptr;
  size_t size_ = 0;
};
