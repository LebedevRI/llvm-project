//===-- llvm/Support/LinearAlgebra.h - Matrices and operations --*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains some functions that are useful for math stuff.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_SUPPORT_LINEARALGEBRA_H
#define LLVM_SUPPORT_LINEARALGEBRA_H

#include <algorithm>
#include <cassert>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace llvm {
namespace linearalgebra {

///--------------------------------------------------------------------------///

template <typename T, typename Derived> struct MatrixInterface {
  using value_type = T;

  //  MatrixInterface() = delete;
  MatrixInterface &operator=(MatrixInterface) = delete;
  MatrixInterface &operator=(const MatrixInterface &) = delete;

  int getNumRows() const { return ((const Derived *)this)->getNumRows(); }
  int getNumColumns() const { return ((const Derived *)this)->getNumColumns(); }

  T &operator()(int row, int col) {
    return ((Derived *)this)->operator()(row, col);
  }
};

///--------------------------------------------------------------------------///

template <typename, typename> class TransposedMatrix;

template <typename T = double>
class Matrix : public MatrixInterface<T, Matrix<T>> {
public:
  Matrix(int num_rows, int num_cols) : num_rows(num_rows), num_cols(num_cols) {
    storage.resize(num_rows * num_cols);
  }

  // Disallow copying.
  Matrix(const Matrix &) = delete;
  Matrix(Matrix &&) = delete;
  Matrix &operator=(Matrix) = delete;
  Matrix &operator=(const Matrix &) = delete;
  Matrix &operator=(Matrix &&) = delete;

  int getNumRows() const { return num_rows; };
  int getNumColumns() const { return num_cols; };

  T &operator()(int row, int col) {
    assert(row >= 0 && row < num_rows);
    assert(col >= 0 && col < num_cols);

    return storage[num_cols * row + col];
  }

  /// X^T
  TransposedMatrix<T, Matrix> getTransposedMatrix() {
    return TransposedMatrix<T, Matrix>(*this);
  }

private:
  std::vector<T> storage;
  int num_rows;
  int num_cols;
};

template <typename T, typename InnerTy>
class TransposedMatrix
    : public MatrixInterface<T, TransposedMatrix<T, InnerTy>> {
  InnerTy &m;

public:
  TransposedMatrix(InnerTy &m) : m(m) {}

  /// (X^T)^T --> X
  InnerTy &getTransposedMatrix() { return m; }

  int getNumRows() const { return m.getNumColumns(); };
  int getNumColumns() const { return m.getNumRows(); };

  T &operator()(int row, int col) { return m(col, row); }
};

template <typename T, typename LHSTy, typename RHSTy>
class AugmentedMatrix
    : public MatrixInterface<T, AugmentedMatrix<T, LHSTy, RHSTy>> {
  MatrixInterface<T, LHSTy> &a;
  MatrixInterface<T, RHSTy> &b;

public:
  AugmentedMatrix(MatrixInterface<T, LHSTy> &a, MatrixInterface<T, RHSTy> &b)
      : a(a), b(b) {
    assert(a.getNumRows() == b.getNumRows());
  }

  int getNumRows() const { return a.getNumRows(); };
  int getNumColumns() const { return a.getNumColumns() + b.getNumColumns(); };

  T &operator()(int row, int col) {
    assert(col >= 0 && col < getNumColumns());
    if (col < a.getNumColumns())
      return a(row, col);
    return b(row, col - a.getNumColumns());
  }
};

template <typename T, typename LHSTy, typename RHSTy>
AugmentedMatrix<T, LHSTy, RHSTy>
getAugmentedMatrix(MatrixInterface<T, LHSTy> &a, MatrixInterface<T, RHSTy> &b) {
  return AugmentedMatrix<T, LHSTy, RHSTy>(a, b);
}

///--------------------------------------------------------------------------///

/// Get a square matrix with all elements being zero except the elements
/// on the diagonal, which are ones.
template <typename T, typename LHSTy>
Matrix<T> isSquareMatrix(const MatrixInterface<T, LHSTy> &LHS) {
  return LHS.getNumRows() == LHS.getNumColumns();
}

/// Are all the entries below the main diagonal are zero?
template <typename T, typename LHSTy>
bool isUpperTriangularMatrix(MatrixInterface<T, LHSTy> &LHS) {
  for (int row = 0; row != LHS.getNumRows(); ++row) {
    for (int col = 0; col != LHS.getNumColumns(); ++col) {
      // Is this element on the diagonal, or to the right of diagonal?
      if (col >= row)
        continue;
      if (LHS(row, col) != 0)
        return false;
    }
  }
  return true;
}

///--------------------------------------------------------------------------///

template <typename T, typename LHSTy>
int getLeadingCoeffientColumn(MatrixInterface<T, LHSTy> &LHS, int row) {
  int col = 0;
  for (; col != LHS.getNumColumns(); ++col) {
    if (LHS(row, col) != 0)
      break;
  }
  return col;
}

template <typename T, typename LHSTy>
int isZeroRow(MatrixInterface<T, LHSTy> &LHS, int row) {
  return getLeadingCoeffientColumn(LHS, row) == LHS.getNumColumns();
}

template <typename T, typename LHSTy>
bool isMatrixInRowEchelonForm(MatrixInterface<T, LHSTy> &LHS) {
  assert(isUpperTriangularMatrix(LHS));

  bool seenZeroRow = false;
  for (int row = 0; row != LHS.getNumRows(); ++row) {
    bool IsZeroRow = isZeroRow(LHS, row);
    seenZeroRow |= IsZeroRow;
    if (seenZeroRow) {
      if (!IsZeroRow)
        return false;
      continue;
    }
    if (row == 0)
      continue;
    if (getLeadingCoeffientColumn(LHS, row) <=
        getLeadingCoeffientColumn(LHS, row - 1))
      return false;
  }
  return true;
}

template <typename T, typename LHSTy>
bool isMatrixInReducedRowEchelonForm(MatrixInterface<T, LHSTy> &LHS) {
  if (!isMatrixInRowEchelonForm(LHS))
    return false;

  for (int pivot = 0; pivot != LHS.getNumRows(); ++pivot) {
    if (isZeroRow(LHS, pivot))
      break;
    int leadingCoeffientColumn = getLeadingCoeffientColumn(LHS, pivot);
    for (int row = 0; row != LHS.getNumRows(); ++row) {
      if (LHS(row, leadingCoeffientColumn) != (row == pivot) ? 1 : 0)
        return false;
    }
  }
  return true;
}

///--------------------------------------------------------------------------///

/// Get a square matrix with all elements being zero except the elements
/// on the diagonal, which are ones.
template <typename T> Matrix<T> getIdentityMatrix(int Size) {
  Matrix<T> m(Size, Size);
  for (int diagEltIdx = 0; diagEltIdx != Size; ++diagEltIdx)
    m(diagEltIdx, diagEltIdx) = 1;
  return m;
}

///--------------------------------------------------------------------------///

/// X * Y
template <typename T, typename LHSTy, typename RHSTy>
Matrix<T> operator*(const MatrixInterface<T, LHSTy> &LHS,
                    const MatrixInterface<T, RHSTy> &RHS) {
  assert(isSquareMatrix(LHS));

  Matrix<T> R(LHS.getNumRows(), RHS.getNumColumns());
  for (int row = 0; row != R.getNumRows(); ++row) {
    for (int col = 0; col != R.getNumColumns(); ++col) {
      for (int k = 0; k != LHS.getNumColumns(); ++k) {
        R(row, col) += LHS(row, k) * RHS(k, col);
      }
    }
  }

  return R;
}

///--------------------------------------------------------------------------///

/// X^T * X
template <typename T, typename LHSTy>
Matrix<T> getNormalMatrix(const MatrixInterface<T, LHSTy> &LHS) {
  return LHS.getTransposedMatrix() * LHS;
}

/// X^T * y
template <typename T, typename LHSTy, typename RHSTy>
Matrix<T> getMomentMatrix(const MatrixInterface<T, LHSTy> &LHS,
                          const MatrixInterface<T, RHSTy> &RHS) {
  assert(LHS.getNumRows() == RHS.getNumRows());
  assert(RHS.getNumColumns() == 1);

  return LHS.getTransposedMatrix() * RHS;
}

///--------------------------------------------------------------------------///

// Elementary row operations
template <typename T, typename LHSTy>
void swapRows(MatrixInterface<T, LHSTy> &LHS, int rowA, int rowB) {
  assert(rowA != rowB);
  for (int col = 0; col != LHS.getNumColumns(); ++col)
    std::swap(LHS(rowA, col), LHS(rowB, col));
}
template <typename T, typename LHSTy>
void multiplyRow(MatrixInterface<T, LHSTy> &LHS, int row, T scale) {
  assert(scale != 0);
  for (int col = 0; col != LHS.getNumColumns(); ++col)
    LHS(row, col) *= scale;
}
template <typename T, typename LHSTy>
void divideRow(MatrixInterface<T, LHSTy> &LHS, int row, T scale) {
  assert(scale != 0);
  for (int col = 0; col != LHS.getNumColumns(); ++col)
    LHS(row, col) /= scale;
}
template <typename T, typename LHSTy>
void addRowMultiple(MatrixInterface<T, LHSTy> &LHS, int dstRow, int srcRow,
                    T scale) {
  assert(dstRow != srcRow);
  assert(scale != 0);
  for (int col = 0; col != LHS.getNumColumns(); ++col)
    LHS(dstRow, col) += scale * LHS(srcRow, col);
}
template <typename T, typename LHSTy>
void subtractRowMultiple(MatrixInterface<T, LHSTy> &LHS, int dstRow, int srcRow,
                         T scale) {
  assert(dstRow != srcRow);
  assert(scale != 0);
  for (int col = 0; col != LHS.getNumColumns(); ++col)
    LHS(dstRow, col) -= scale * LHS(srcRow, col);
}

///--------------------------------------------------------------------------///

template <typename T, typename LHSTy>
int findPivotingElement(MatrixInterface<T, LHSTy> &LHS, int column,
                        int firstRow) {
  int pivotRow = firstRow;
  T pivotValMag = std::abs(LHS(firstRow, column));
  for (int row = firstRow + 1; row != LHS.getNumRows(); ++row) {
    T currValMag = std::abs(LHS(row, column));
    if (currValMag < pivotValMag)
      continue;
    pivotRow = row;
    pivotValMag = currValMag;
  }
  return pivotRow;
}

template <typename T, typename LHSTy>
void performGaussianForwardElimination(MatrixInterface<T, LHSTy> &LHS) {
  assert(LHS.getNumRows() <= LHS.getNumColumns());
  for (int pivot = 0; pivot != LHS.getNumRows(); ++pivot) {
    int column = pivot;
    int pivotRow = findPivotingElement(LHS, column, pivot);

    if (pivotRow != pivot) {
      swapRows(LHS, pivotRow, pivot);
      pivotRow = pivot;
    }

    T &pivotElement = LHS(pivotRow, column);
    if (pivotElement == 0)
      continue;

    divideRow(LHS, pivotRow, pivotElement);
    pivotElement = 1.0; // Account for floating point rounding issues.

    for (int row = pivotRow + 1; row != LHS.getNumRows(); ++row) {
      T &currElement = LHS(row, column);
      if (currElement == 0)
        continue;

      subtractRowMultiple(LHS, row, pivotRow, currElement);
      currElement = 0; // Account for floating point rounding issues.
    }
  }
  assert(isMatrixInRowEchelonForm(LHS));
}

template <typename T, typename LHSTy>
void performGaussianElimination(MatrixInterface<T, LHSTy> &LHS) {
  performGaussianForwardElimination(LHS);
}

/// |X|
template <typename T, typename LHSTy>
T getDeterminant(const MatrixInterface<T, LHSTy> &LHS) {
  assert(LHS.getNumRows() == LHS.getNumColumns());
  static int Size = LHS.getNumRows();

  performGaussianElimination(LHS);

  T determinant = 1;
  for (int diagEltIdx = 0; diagEltIdx != Size; ++diagEltIdx)
    determinant *= LHS(diagEltIdx, diagEltIdx);

  return determinant;
}

} // namespace linearalgebra
} // namespace llvm

#endif
