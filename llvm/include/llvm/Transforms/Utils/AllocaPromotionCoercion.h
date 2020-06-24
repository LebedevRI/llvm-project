//===- AllocaPromotionCoercion.h --------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Try to rewrite (uses of) static allocas in entry basic blocks to operate on
// the whole alloca, if that makes alloca eligible for mem2reg promotion.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TRANSFORMS_UTILS_ALLOCAPROMOTIONCOERCION_H
#define LLVM_TRANSFORMS_UTILS_ALLOCAPROMOTIONCOERCION_H

#include "llvm/IR/PassManager.h"

namespace llvm {

class Function;

class AllocaPromotionCoercionPass
    : public PassInfoMixin<AllocaPromotionCoercionPass> {
public:
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);
};

} // end namespace llvm

#endif // LLVM_TRANSFORMS_UTILS_ALLOCAPROMOTIONCOERCION_H
