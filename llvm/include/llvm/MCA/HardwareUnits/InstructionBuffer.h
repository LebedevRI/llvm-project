//===---------------------- InstructionBuffer.h -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
///
/// This file simulates the hardware responsible for queueing instruction bytes
/// between fetch unit and decode unit.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_MCA_INSTRUCTION_BUFFER_H
#define LLVM_MCA_INSTRUCTION_BUFFER_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/MCA/HardwareUnits/HardwareUnit.h"
#include <vector>

namespace llvm {
namespace mca {

class InstructionBuffer final : public HardwareUnit {
public:
  struct Buffer {
    int NumBytesRemaining;
    bool FetchCompleted;

    Buffer();

    // Can we read from this buffer yet?
    bool isReady() const;
    // Is this buffer actually alive, or we just didn't GC it yet?
    bool isDepleted() const;
  };

private:
  std::vector<Buffer> Buffers;
  unsigned NumKnownDepletedBuffers;

  // Per cycle
  MutableArrayRef<Buffer> BuffersAvaliableToDecoderThisCycle;
  unsigned BytesRemaining;
  unsigned FetchesPerformed;

  static constexpr unsigned MaxBuffersTotal = 16U;
  static constexpr unsigned BytesPerBuffer = 16U;
  static constexpr unsigned BytesFetchedPerCycle = 32U;
  static constexpr unsigned BuffersFilledPerCycle =
      BytesFetchedPerCycle / BytesPerBuffer;
  static_assert(BuffersFilledPerCycle * BytesPerBuffer == BytesFetchedPerCycle,
                "Fetch amount should be a multiple of buffer size");

  static constexpr unsigned NumBuffersDecoderCanAccessPerCycle = 2U;
  static constexpr unsigned NumBytesDecoderCanAccessPerCycle =
      NumBuffersDecoderCanAccessPerCycle * BytesPerBuffer;
  static constexpr unsigned MaxFetchesPerCycle = 4U;

  bool canAccomodateInstructionFetch() const;
  void enqueueInstructionFetch();
  unsigned getNumDepletedBuffers() const;
  void recalculateNumDepletedBuffers();

  unsigned getNumOccupiedBuffers() const;
  unsigned getNumVacantBuffers() const;

public:
  InstructionBuffer();

  void cycleStart();

  unsigned getNumBytesRemaining() const;
  void consumeNumBytes(unsigned NumBytesToConsume);

  void cycleEnd();
};

} // namespace mca
} // namespace llvm

#endif // LLVM_MCA_INSTRUCTION_BUFFER_H
