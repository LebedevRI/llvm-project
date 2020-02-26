//===---------------------- DecodeStage.h -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
///
/// This file defines a stage that implements instruction decoding
/// into micro-ops.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_MCA_DECODE_STAGE_H
#define LLVM_MCA_DECODE_STAGE_H

#include "llvm/ADT/SmallVector.h"
#include "llvm/MCA/Instruction.h"
#include "llvm/MCA/Stages/Stage.h"
#include "llvm/Support/Error.h"
#include <memory>

namespace llvm {
namespace mca {

class InstructionBuffer;
class SourceMgr;

/// A stage that simulates an instruction decoder.
class DecodeStage : public Stage {
  InstructionBuffer &Buffer;
  SourceMgr &SM;

  SmallVector<std::unique_ptr<Instruction>, 16> Instructions;
  unsigned NumRetired;

  InstRef CurrentInstruction;

  // Updates the program counter, and sets 'CurrentInstruction'.
  void getNextInstructionFromSourceManager();

  struct MicroOpEngine {
    InstRef IR;
    int MicroOpsLeftToGenerate;
  } MicroOpEngine;
  SmallVector<InstRef, 8> Decoders;

  DecodeStage(const DecodeStage &Other) = delete;
  DecodeStage &operator=(const DecodeStage &Other) = delete;

  bool microOpDecodersHaveWorkToComplete() const;

  // Is this instruction microcoded?
  static bool IsMicroCoded(const InstRef &IR);

  InstRef peekNextInstructionFromBuffer() const;
  InstRef getNextInstructionFromBuffer();

  // Would we be able to place this decoded (from it's byte encoding)
  // instruction onto micro-op decoders?
  bool canEnqueueForMicroOpDecoding(const InstRef &IR) const;

  Error performMicroOpDecoding();

public:
  DecodeStage(InstructionBuffer &Buffer, SourceMgr &SM);

  Error cycleStart() override;

  // Are there any instructions currently being decoded?
  bool hasWorkToComplete() const override;

  // Would decoder be able to start decoding next instruction?
  bool isAvailable(const InstRef & /*unused*/) const override;

  // Start decoding the next instruction.
  Error execute(InstRef & /*unused*/) override;

  // Actually generate microcodes, if any.
  Error cycleEnd() override;
};

} // namespace mca
} // namespace llvm

#endif // LLVM_MCA_DECODE_STAGE_H
