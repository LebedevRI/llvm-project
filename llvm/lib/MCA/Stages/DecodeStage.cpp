//===---------------------- DecodeStage.cpp ---------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
///
/// This file defines the DecodeStage.
///
//===----------------------------------------------------------------------===//

#include "llvm/MCA/Stages/DecodeStage.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/MCA/HardwareUnits/InstructionBuffer.h"
#include "llvm/MCA/SourceMgr.h"
#include <algorithm>
#include <cassert>
#include <iterator>
#include <numeric>
#include <utility>

namespace llvm {
namespace mca {

#define DEBUG_TYPE "llvm-mca"

DecodeStage::DecodeStage(InstructionBuffer &Buffer_, SourceMgr &SM_)
    : Buffer(Buffer_), SM(SM_), NumRetired(0) {}

void DecodeStage::getNextInstructionFromSourceManager() {
  assert(!CurrentInstruction && "There is already an instruction to process!");
  if (!SM.hasNext())
    return;
  SourceRef SR = SM.peekNext();
  std::unique_ptr<Instruction> Inst = std::make_unique<Instruction>(SR.second);
  CurrentInstruction = InstRef(SR.first, Inst.get());
  Instructions.emplace_back(std::move(Inst));
  SM.updateNext();
}

llvm::Error DecodeStage::cycleStart() {
  if (!CurrentInstruction)
    getNextInstructionFromSourceManager();
  return llvm::ErrorSuccess();
}

bool DecodeStage::microOpDecodersHaveWorkToComplete() const {
  return MicroOpEngine.IR || std::any_of(Decoders.begin(), Decoders.end(),
                                         [](const InstRef &IR) { return IR; });
}

bool DecodeStage::hasWorkToComplete() const {
  return static_cast<bool>(CurrentInstruction) ||
         microOpDecodersHaveWorkToComplete();
}

bool DecodeStage::IsMicroCoded(const InstRef &IR) {
  // FIXME: parametrize.
  return IR.getInstruction()->getDesc().NumMicroOps > 2;
}

InstRef DecodeStage::peekNextInstructionFromBuffer() const {
  if (!CurrentInstruction)
    return InstRef();

  if (Buffer.getNumBytesRemaining() <
      CurrentInstruction.getInstruction()->getEncodingByteLength())
    return InstRef();

  return CurrentInstruction;
}

bool DecodeStage::canEnqueueForMicroOpDecoding(const InstRef &IR) const {
  // If we are currently decoding microcoded instruction,
  // we can't start decoding *anything* else.
  if (MicroOpEngine.IR)
    return false;

  if (IsMicroCoded(IR)) {
    // We can not start decoding microcoded instruction until
    // we finish decoding *all* preceding instructions.
    return !microOpDecodersHaveWorkToComplete();
  }

  // There are only 4 decoders.
  // FIXME: parametrize.
  if (std::count_if(Decoders.begin(), Decoders.end(),
                    [](const InstRef &IR) { return IR; }) >= 4)
    return false;

  // FIXME: parametrize. It can be more complex than that.
  unsigned WouldBeNumMicroOpsTotal =
      std::accumulate(Decoders.begin(), Decoders.end(),
                      IR.getInstruction()->getDesc().NumMicroOps,
                      [](unsigned NumMicroOpsSoFar, const InstRef &IR) {
                        if (const Instruction *Instr = IR.getInstruction())
                          NumMicroOpsSoFar += Instr->getDesc().NumMicroOps;
                        return NumMicroOpsSoFar;
                      });
  // We can at most generate 4 microops per cycle.
  // That is, we can generate 2-2/2-1-1/1-1-1-1.
  return WouldBeNumMicroOpsTotal <= 4;
}

bool DecodeStage::isAvailable(const InstRef & /*unused*/) const {
  InstRef IR = peekNextInstructionFromBuffer();
  if (!IR)
    return false;

  return canEnqueueForMicroOpDecoding(IR);
};

InstRef DecodeStage::getNextInstructionFromBuffer() {
  InstRef IR = peekNextInstructionFromBuffer();
  assert(IR && "No next instruction?");
  Buffer.consumeNumBytes(IR.getInstruction()->getEncodingByteLength());
  return IR;
}

Error DecodeStage::execute(InstRef & /*unused*/) {
  assert(isAvailable(InstRef()) &&
         "Should not start decoding instruction unless we are ready to.");

  InstRef IR = getNextInstructionFromBuffer();

  // Move the program counter.
  CurrentInstruction.invalidate();
  getNextInstructionFromSourceManager();

  if (IsMicroCoded(IR)) {
    assert(Decoders.empty() && !MicroOpEngine.IR &&
           "Must not start decoding microcoded instruction if the decoder is "
           "already occupied.");

    MicroOpEngine.IR = IR;
    MicroOpEngine.MicroOpsLeftToGenerate =
        IR.getInstruction()->getDesc().NumMicroOps;
    return llvm::ErrorSuccess();
  }

  assert(!MicroOpEngine.IR &&
         "Must not start decoding non-microcoded instruction if already "
         "decoding microcoded instruction.");
  Decoders.emplace_back(IR);
  return llvm::ErrorSuccess();
}

Error DecodeStage::performMicroOpDecoding() {
  if (!microOpDecodersHaveWorkToComplete())
    return llvm::ErrorSuccess();

  if (InstRef &IR = MicroOpEngine.IR) {
    assert(Decoders.empty() &&
           "Microcoded instruction must be decoded standalone");

    // Is next stage ready to recieve all the microcodes?
    if (!checkNextStage(IR))
      return llvm::ErrorSuccess(); // Stall.

    // Okay, start/continue generating microops.

    // FIXME: is that so for BdVer2?
    // FIXME: parametrize.
    MicroOpEngine.MicroOpsLeftToGenerate -= 2;
    // Did we just finish generating microops for this Microcoded instruction?
    if (MicroOpEngine.MicroOpsLeftToGenerate > 0)
      return llvm::ErrorSuccess(); // More microops left to generate...

    // Done decoding/generating.
    if (llvm::Error Val = moveToTheNextStage(IR))
      return Val;
    IR.invalidate();
    return llvm::ErrorSuccess();
  }

  // Okay, must be a normal instruction.
  assert(!Decoders.empty() && "Should be decoding some plain instructions.");
  for (InstRef &IR : Decoders) {
    if (!IR)
      continue;

    // Is next stage ready to recieve microops of this decoded instruction?
    if (!checkNextStage(IR))
      break; // Stall.
    // Done decoding/generating in a single cycle.
    if (llvm::Error Val = moveToTheNextStage(IR))
      return Val;
    IR.invalidate();
  }

  return llvm::ErrorSuccess();
}

Error DecodeStage::cycleEnd() {
  if (Error E = performMicroOpDecoding())
    return E;

  // Find the first instruction which hasn't been fully decoded.
  auto DIt = find_if(Decoders, [](const InstRef &IR) { return IR; });
  unsigned NumDecoded = std::distance(Decoders.begin(), DIt);
  // Erase instructions up to the first that hasn't been decoded.
  if ((NumDecoded * 2) >= Decoders.size())
    Decoders.erase(Decoders.begin(), DIt);

  // Find the first instruction which hasn't been retired.
  auto RRange = make_range(&Instructions[NumRetired], Instructions.end());
  auto RIt = find_if(RRange, [](const std::unique_ptr<Instruction> &I) {
    return !I->isRetired();
  });
  NumRetired = std::distance(Instructions.begin(), RIt);
  // Erase instructions up to the first that hasn't been retired.
  if ((NumRetired * 2) >= Instructions.size()) {
    Instructions.erase(Instructions.begin(), RIt);
    NumRetired = 0;
  }

  return llvm::ErrorSuccess();
}

} // namespace mca
} // namespace llvm
