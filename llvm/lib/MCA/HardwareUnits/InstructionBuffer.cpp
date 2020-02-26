//===------------------ InstructionBuffer.cpp -------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
///
/// This file defines the anchor for the base class that describes
/// simulated hardware units.
///
//===----------------------------------------------------------------------===//

#include "llvm/MCA/HardwareUnits/InstructionBuffer.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/iterator_range.h"
#include <algorithm>
#include <cassert>
#include <iterator>
#include <numeric>

namespace llvm {
namespace mca {

#define DEBUG_TYPE "llvm-mca"

const unsigned InstructionBuffer::MaxBuffersTotal;
const unsigned InstructionBuffer::BytesPerBuffer;
const unsigned InstructionBuffer::BytesFetchedPerCycle;
const unsigned InstructionBuffer::BuffersFilledPerCycle;
const unsigned InstructionBuffer::NumBuffersDecoderCanAccessPerCycle;
const unsigned InstructionBuffer::NumBytesDecoderCanAccessPerCycle;

InstructionBuffer::InstructionBuffer() : NumKnownDepletedBuffers(0) {
  // Really overallocate to avoid allocations at all.
  Buffers.reserve(4 * MaxBuffersTotal);
};

InstructionBuffer::Buffer::Buffer()
    : NumBytesRemaining(-1), FetchCompleted(false) {}

bool InstructionBuffer::Buffer::isReady() const { return FetchCompleted; };

bool InstructionBuffer::Buffer::isDepleted() const {
  if (!isReady()) // Can't deplete a buffer that hasn't fetched yet.
    return false;
  assert(NumBytesRemaining >= 0 &&
         "Shouldn't ever over-consume bytes in the buffer.");
  return NumBytesRemaining == 0;
};

unsigned InstructionBuffer::getNumDepletedBuffers() const {
#ifndef NDEBUG
  llvm::for_each(ArrayRef<Buffer>(Buffers).take_front(NumKnownDepletedBuffers),
                 [](const Buffer &Buf) {
                   assert(Buf.isDepleted() &&
                          "All buffers that we counted as depleted should "
                          "actually be depleted.");
                 });
#endif

  // Ignore known-depleted buffers.
  auto Range =
      make_range(Buffers.begin() + NumKnownDepletedBuffers, Buffers.end());
  // And find first non-depleted one.
  auto It = find_if(Range, [](const Buffer &Buf) { return !Buf.isDepleted(); });
  // So how many depleted buffers there are total?
  return std::distance(Buffers.begin(), It);
}

void InstructionBuffer::recalculateNumDepletedBuffers() {
  NumKnownDepletedBuffers = getNumDepletedBuffers();
}

unsigned InstructionBuffer::getNumOccupiedBuffers() const {
  unsigned NumOccupiedBuffers = Buffers.size() - getNumDepletedBuffers();
  assert(NumOccupiedBuffers <= MaxBuffersTotal &&
         "There is a hard limit on the buffer count.");
  return NumOccupiedBuffers;
}

unsigned InstructionBuffer::getNumVacantBuffers() const {
  int NumVacantBuffers = (int)MaxBuffersTotal - getNumOccupiedBuffers();
  assert(NumVacantBuffers >= 0 && "Can't have negative count of empty buffers");
  return NumVacantBuffers;
}

bool InstructionBuffer::canAccomodateInstructionFetch() const {
  return getNumVacantBuffers() >= BuffersFilledPerCycle;
}

void InstructionBuffer::enqueueInstructionFetch() {
  assert(canAccomodateInstructionFetch() &&
         "Should not be performing fetch if can't accomodate for it.");

  for (int NumBuffersFilled = 0; NumBuffersFilled != BuffersFilledPerCycle;
       ++NumBuffersFilled)
    Buffers.emplace_back();
}

void InstructionBuffer::cycleStart() {
  // The fetches from previous cycle (if any) have completed by now.
  for (Buffer &Buf : llvm::reverse(Buffers)) {
    if (Buf.FetchCompleted)
      break; // All the earlier buffers already completed fetching.
    Buf.NumBytesRemaining = BytesPerBuffer;
    Buf.FetchCompleted = true;
  }

  // And if can accomodate it, enqueue next fetch.
  if (canAccomodateInstructionFetch())
    enqueueInstructionFetch();

  // Finally, which buffers can decoder actually look into this cycle?
  BuffersAvaliableToDecoderThisCycle = Buffers;
  assert(NumKnownDepletedBuffers == getNumDepletedBuffers() &&
         "NumKnownDepletedBuffers should not be outdated yet.");
  BuffersAvaliableToDecoderThisCycle =
      BuffersAvaliableToDecoderThisCycle.drop_front(NumKnownDepletedBuffers)
          .take_front(NumBuffersDecoderCanAccessPerCycle)
          .take_while([](const Buffer &Buf) { return Buf.isReady(); });

#ifndef NDEBUG
  assert(BuffersAvaliableToDecoderThisCycle.size() <=
             NumBuffersDecoderCanAccessPerCycle &&
         "Predicate error?");
  llvm::for_each(BuffersAvaliableToDecoderThisCycle, [](const Buffer &Buf) {
    assert(Buf.isReady() && !Buf.isDepleted() &&
           "Should have only selected buffers that finished fetching and "
           "weren't depleted previously.");
  });
#endif

  // The number of eligible buffers must match decoder's expectations exactly.
  if (BuffersAvaliableToDecoderThisCycle.size() !=
      NumBuffersDecoderCanAccessPerCycle)
    BuffersAvaliableToDecoderThisCycle =
        decltype(BuffersAvaliableToDecoderThisCycle)();

  BytesRemaining =
      std::accumulate(BuffersAvaliableToDecoderThisCycle.begin(),
                      BuffersAvaliableToDecoderThisCycle.end(), unsigned(0),
                      [](unsigned BytesRemainingSoFar, const Buffer &Buf) {
                        return BytesRemainingSoFar + Buf.NumBytesRemaining;
                      });
  BytesRemaining = std::min(BytesRemaining, NumBytesDecoderCanAccessPerCycle);
  FetchesPerformed = 0;
}

unsigned InstructionBuffer::getNumBytesRemaining() const {
  return BytesRemaining;
}

void InstructionBuffer::consumeNumBytes(unsigned NumBytesToConsume) {
  assert(NumBytesToConsume <= getNumBytesRemaining() &&
         "Can't consume more bytes than avaliable.");

  BytesRemaining -= NumBytesToConsume;
  for (Buffer &Buf : BuffersAvaliableToDecoderThisCycle) {
    unsigned NumBytesCanConsumeFromThisBuffer =
        std::min((unsigned)Buf.NumBytesRemaining, NumBytesToConsume);
    Buf.NumBytesRemaining -= NumBytesCanConsumeFromThisBuffer;
    NumBytesToConsume -= NumBytesCanConsumeFromThisBuffer;
  }
  assert(NumBytesToConsume == 0 && "Consumption failure");

  ++FetchesPerformed;
  if (FetchesPerformed == MaxFetchesPerCycle)
    BytesRemaining = 0; // ratelimit
}

void InstructionBuffer::cycleEnd() {
  recalculateNumDepletedBuffers();
  // If at least half of the buffers we're tracking are depleted, GC them.
  if ((NumKnownDepletedBuffers * 2) >= Buffers.size()) {
    // Erase buffers up to the first that hasn't been depleted.
    Buffers.erase(Buffers.begin(), Buffers.begin() + NumKnownDepletedBuffers);
    NumKnownDepletedBuffers = 0;
  }
}

} // namespace mca
} // namespace llvm
