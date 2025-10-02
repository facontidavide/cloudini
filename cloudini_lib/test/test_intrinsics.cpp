
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

#include <gtest/gtest.h>

#include <cloudini_lib/intrinsics.hpp>

TEST(Intrinsics, Vectors) {
  Vector4f a(1.1f, 2.2f, 3.3f, -4.4f);

  ASSERT_FLOAT_EQ(a[0], 1.1f);
  ASSERT_FLOAT_EQ(a[1], 2.2f);
  ASSERT_FLOAT_EQ(a[2], 3.3f);
  ASSERT_FLOAT_EQ(a[3], -4.4f);

  Vector4f b(-0.1f, -0.2f, -0.3f, 0.4f);

  Vector4f c = a + b;
  ASSERT_FLOAT_EQ(c[0], 1.0f);
  ASSERT_FLOAT_EQ(c[1], 2.0f);
  ASSERT_FLOAT_EQ(c[2], 3.0f);
  ASSERT_FLOAT_EQ(c[3], -4.0f);

  auto ai = cast_vector4f_to_vector4i(a);
  ASSERT_EQ(ai[0], 1);
  ASSERT_EQ(ai[1], 2);
  ASSERT_EQ(ai[2], 3);
  ASSERT_EQ(ai[3], -4);

  Vector4i di(1, 2, 3, -4);
  ASSERT_EQ(di[0], 1);
  ASSERT_EQ(di[1], 2);
  ASSERT_EQ(di[2], 3);
  ASSERT_EQ(di[3], -4);
}
