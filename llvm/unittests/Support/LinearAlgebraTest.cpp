//===- unittests/Support/LinearAlgebraTest.cpp - linear algebra tests -----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/Support/LinearAlgebra.h"
#include "gtest/gtest.h"
#include <initializer_list>

using namespace llvm;
using namespace linearalgebra;

namespace {

template <typename T, typename LHSTy>
std::vector<T> getAllValues(MatrixInterface<T, LHSTy> &LHS) {
  std::vector<T> Elements;
  for (int row = 0; row != LHS.getNumRows(); ++row) {
    for (int col = 0; col != LHS.getNumColumns(); ++col)
      Elements.emplace_back(LHS(row, col));
  }
  return Elements;
}

TEST(LinearAlgebraTest, Basic) {
  Matrix<int> M(2, 2);
  EXPECT_EQ(std::vector<int>({0, 0, 0, 0}), getAllValues(M));

  M(0, 1) = 42;
  EXPECT_EQ(std::vector<int>({0, 42, 0, 0}), getAllValues(M));

  auto TM = M.getTransposedMatrix();
  EXPECT_EQ(std::vector<int>({0, 0, 42, 0}), getAllValues(TM));
  EXPECT_EQ(std::vector<int>({0, 42, 0, 0}), getAllValues(M));

  auto &TTM = TM.getTransposedMatrix();
  EXPECT_EQ(std::vector<int>({0, 42, 0, 0}), getAllValues(TTM));
  EXPECT_EQ(&TTM, &M);

  M(1, 1) = 22;
  EXPECT_EQ(std::vector<int>({0, 0, 42, 22}), getAllValues(TM));
  EXPECT_EQ(std::vector<int>({0, 42, 0, 22}), getAllValues(M));

  auto MM = getAugmentedMatrix(M, M);
  EXPECT_EQ(std::vector<int>({0, 42, 0, 42, 0, 22, 0, 22}), getAllValues(MM));
  MM(0, 0) = -1;
  MM(0, 2) = -1;

  AugmentedMatrix<int, decltype(M), decltype(TM)> MTM(M, TM);
  EXPECT_EQ(std::vector<int>({-1, 42, -1, 0, 0, 22, 42, 22}),
            getAllValues(MTM));

  Matrix<int> M2(2, 1);
  M2(0, 0) = 3;
  M2(1, 0) = 6;

  auto MM2 = getAugmentedMatrix(M, M2);
  EXPECT_EQ(std::vector<int>({-1, 42, 3, 0, 22, 6}), getAllValues(MM2));

  auto M2M = getAugmentedMatrix(M2, M);
  EXPECT_EQ(std::vector<int>({3, -1, 42, 6, 0, 22}), getAllValues(M2M));
}

TEST(LinearAlgebraTest, z) {
  Matrix<double> M(2, 2);
  M(0, 0) = 1;
  M(0, 1) = 2;
  M(1, 0) = 3;
  M(1, 1) = 4;

  performGaussElimination(M);
  EXPECT_EQ(std::vector<double>({1, 4. / 3, 0, 1}), getAllValues(M));

  Matrix<double> M2(3, 3);
  M2(0, 0) = 4;
  M2(0, 1) = 1;
  M2(0, 2) = 9;
  M2(1, 0) = 2;
  M2(1, 1) = -1;
  M2(1, 2) = 3;
  M2(2, 0) = 5;
  M2(2, 1) = -3;
  M2(2, 2) = 7;

  performGaussElimination(M2);
  EXPECT_EQ(std::vector<double>({1, -0.6, 1.4, 0, 1, 1, 0, 0, 1}),
            getAllValues(M2));
}

} // namespace
