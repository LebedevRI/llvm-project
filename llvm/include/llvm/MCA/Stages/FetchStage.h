//===----------------- FetchStage.h ------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
///
/// FIXME
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_MCA_FETCH_STAGE_H
#define LLVM_MCA_FETCH_STAGE_H

#include "llvm/MCA/Stages/Stage.h"
#include "llvm/Support/Error.h"

namespace llvm {
namespace mca {

class InstRef;
class InstructionBuffer;

class FetchStage final : public Stage {
  InstructionBuffer &Buffer;

  FetchStage(const FetchStage &Other) = delete;
  FetchStage &operator=(const FetchStage &Other) = delete;

public:
  FetchStage(InstructionBuffer &Buffer);

  Error cycleStart() override;

  bool hasWorkToComplete() const override;

  bool isAvailable(const InstRef & /*unused*/) const override;

  Error execute(InstRef & /*unused*/) override;

  Error cycleEnd() override;
};

} // namespace mca
} // namespace llvm

#endif // LLVM_MCA_FETCH_STAGE_H
