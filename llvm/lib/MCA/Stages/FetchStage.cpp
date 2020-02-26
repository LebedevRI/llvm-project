//===--------------------- FetchStage.cpp -----------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
///
/// This file defines the FetchStage.
///
//===----------------------------------------------------------------------===//

#include "llvm/MCA/Stages/FetchStage.h"
#include "llvm/MCA/HardwareUnits/InstructionBuffer.h"
#include "llvm/MCA/Instruction.h"
#include <cassert>

namespace llvm {
namespace mca {

#define DEBUG_TYPE "llvm-mca"

FetchStage::FetchStage(InstructionBuffer &Buffer_) : Buffer(Buffer_) {}

Error FetchStage::cycleStart() {
  Buffer.cycleStart();
  return llvm::ErrorSuccess();
}

bool FetchStage::hasWorkToComplete() const {
  // This stage's sole purpose is keeping the Instruction Byte Buffer filled,
  // so we effectively never run out of work, let other stages answer instead.
  return false;
}

bool FetchStage::isAvailable(const InstRef & /*unused*/) const {
  // Just passthrough the question to the next stage, is it ready?
  return checkNextStage(InstRef());
};

Error FetchStage::execute(InstRef & /*unused*/) {
  assert(isAvailable(InstRef()) &&
         "Should not be executing if we don't have resources to do so.");

  // Just passthrough to the next stage.
  InstRef IR; // placeholder, not actually used.
  if (llvm::Error Val = moveToTheNextStage(IR))
    return Val;

  return llvm::ErrorSuccess();
}

Error FetchStage::cycleEnd() {
  Buffer.cycleEnd();
  return llvm::ErrorSuccess();
}

} // namespace mca
} // namespace llvm
