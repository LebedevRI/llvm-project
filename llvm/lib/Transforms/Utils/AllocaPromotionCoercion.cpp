//===- AllocaPromotionCoercion.cpp ------------------------------*- C++ -*-===//
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

#include "llvm/Transforms/Utils/AllocaPromotionCoercion.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/None.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/Sequence.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Twine.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionDivision.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/TargetFolder.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Constant.h"
#include "llvm/IR/ConstantRange.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/InstVisitor.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/IntrinsicInst.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/Use.h"
#include "llvm/IR/User.h"
#include "llvm/IR/Value.h"
#include "llvm/IR/ValueHandle.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/PassRegistry.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/DebugCounter.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TypeSize.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Utils.h"
#include "llvm/Transforms/Utils/Local.h"
#include "llvm/Transforms/Utils/PromoteMemToReg.h"
#include "llvm/Transforms/Utils/ScalarEvolutionExpander.h"
#include <cassert>
#include <cstdint>
#include <utility>

namespace llvm {
class LLVMContext;
} // namespace llvm

using namespace llvm;

#define DEBUG_TYPE "alloca-promotion-coercion"

static cl::opt<bool> EnableAllocaPromotionCoercionPass(
    "enable-" DEBUG_TYPE, cl::init(true),
    cl::desc("Should the AllocaPromotionCoercion pass actually run?"));

static cl::opt<bool> RewriteOnlyIfAllowsPromotion(
    "rewrite-alloca-only-if-allows-promotion", cl::init(true),
    cl::desc(
        "Rewrite alloca only if that is guaranteed to make it promoteable"));

STATISTIC(NumAllocasSeen, "Total number of allocas encountered");
STATISTIC(NumAllocasIgnored, "Total number of allocas that were skipped");
STATISTIC(NumAllocasRewriteEvaluated,
          "Total number of allocas for which rewrite viability was evaluated");
STATISTIC(NumAllocasNoRewritingPossible,
          "Total number of allocas for which no rewriting was possible at all");
STATISTIC(NumAllocasPartialRewriteWillNotAllowPromotion,
          "Total number of allocas for which only a partial rewrite was "
          "possible, one which would not make it promoteable");
STATISTIC(NumAllocasFullRewritePossible,
          "Total number of allocas for which a full rewrite was possible");
STATISTIC(
    NumAllocasPartialRewritePossibleOnly,
    "Total number of allocas for which only a partial rewrite was possible");
STATISTIC(NumAllocasRewritten, "Total number of allocas that were rewritten");
STATISTIC(NumAllocasDCE, "Total number of allocas that were deleted as unused "
                         "as the result of the rewrite");
STATISTIC(NumAllocasBecamePromoteable,
          "Total number of allocas that actually became promoteble");

STATISTIC(
    NumInstrsVisitedDuringRewriteEvalation,
    "Total number of instructions visited during rewrite viability evaluation");
STATISTIC(NumInstrsNotAffectingRewriteViability,
          "Total number of instructions that do not themselves affect rewrite "
          "viability");
STATISTIC(NumRewriteableInstrs,
          "Total number of instructions that can be rewritten");
STATISTIC(NumNonRewritableInstrs,
          "Total number of instructions that can NOT be rewritten");

STATISTIC(NumAllocaInstRewriteEvaluated,
          "Total number of AllocaInst rewrite of which was assessed");
STATISTIC(NumAllocaInstViable,
          "Total number of AllocaInst rewrite of which was viabile");
STATISTIC(NumAllocaInstRewritten,
          "Total number of AllocaInst that actually got rewritten");

STATISTIC(NumMemSetInstRewriteEvaluated,
          "Total number of MemSetInst rewrite of which was assessed");
STATISTIC(NumMemSetInstViable,
          "Total number of MemSetInst rewrite of which was viabile");
STATISTIC(NumMemSetInstRewritten,
          "Total number of MemSetInst that actually got rewritten");

STATISTIC(NumLoadInstRewriteEvaluated,
          "Total number of LoadInst rewrite of which was assessed");
STATISTIC(NumLoadInstViable,
          "Total number of LoadInst rewrite of which was viabile");
STATISTIC(NumLoadInstRewritten,
          "Total number of LoadInst that actually got rewritten");

STATISTIC(NumStoreInstRewriteEvaluated,
          "Total number of StoreInst rewrite of which was assessed");
STATISTIC(NumStoreInstViable,
          "Total number of StoreInst rewrite of which was viabile");
STATISTIC(NumStoreInstRewritten,
          "Total number of StoreInst that actually got rewritten");

DEBUG_COUNTER(AllocaPromotionCoercionCounter, DEBUG_TYPE,
              "Controls alloca rewriting for promotion in " DEBUG_TYPE " pass");

namespace {

uint64_t GetNumBytesAvaliableInAnAlloca(const DataLayout &DL, AllocaInst &AI) {
  uint64_t Size = DL.getTypeSizeInBits(AI.getAllocatedType());
  if (AI.isArrayAllocation())
    Size *= cast<ConstantInt>(AI.getArraySize())->getZExtValue();
  assert(Size % 8 == 0);
  return Size / 8;
}

/// Visitor to rewrite scalar loads and stores as vector loads + lane
/// extracttion / vector load + lane replacement + vector store.
///
/// This pass aggressively rewrites all scalar loads and stores on
/// a particular pointer (or any pointer derived from it which we can identify)
/// with loads and stores of the whole alloca. This is meant to help -mem2reg.
class ScalarLoadStoreRewriter
    : public InstVisitor<ScalarLoadStoreRewriter, Optional<bool>> {
  // Befriend the base class so it can delegate to private visit methods.
  friend class InstVisitor<ScalarLoadStoreRewriter, Optional<bool>>;

  /// Used to calculate sizes of types.
  const DataLayout &DL;

  /// Used to analyze at which offset into alloca we are loading/storing.
  ScalarEvolution &SE;

  /// The builder used to form new instructions.
  using BuilderType = IRBuilder<TargetFolder>;
  BuilderType Builder;

  /// Used to actually produce indexes.
  SCEVExpander &Expander;

  /// Which alloca will we want to rewrite here?
  WeakTrackingVH &AI;

  /// How many bytes does it allocate?
  const SCEVConstant *AllocationByteSize;

  /// What type should it alloca ideally allocate? <AllocationByteSize x i8>.
  FixedVectorType *AllocaTy;

  /// Queue of pointer uses to analyze and potentially rewrite.
  SmallVector<Use *, 8> Queue;

  /// Set to prevent us from cycling with phi nodes and loops.
  SmallPtrSet<Use *, 8> Visited;

  /// The def-use graph of the alloca.
  /// Note that it must store raw pointers and not WeakTrackingVH!
  SmallSetVector<Instruction *, 8> AllocaDefUseGraph;

  /// The current pointer use being analyzed. This is used to dig up the used
  /// value (as opposed to the user).
  Use *U = nullptr;

  AllocaInst &getAllocaInst() { return cast<AllocaInst>(*AI); }

  struct AbstractSolution {
    /// The instruction we are dealing with.
    Instruction &I;

    /// Are we able to handle this particular case?
    enum class SolutionViability : uint8_t {
      Unviable,
      ViableKeepAsIs,
      Viable,
      ViableNoCommitNeeded
    };
    SolutionViability Status = SolutionViability::Unviable;

    const SCEV *getByteOffsetIntoAlloca(const ScalarLoadStoreRewriter &Rewriter,
                                        Value *Ptr) const {
      ScalarEvolution &SE = Rewriter.SE;

      const SCEV *PtrBase = SE.getSCEV(Rewriter.AI);
      assert(PtrBase->getType()->isPointerTy());
      const SCEV *PtrAccessFn = SE.getSCEV(Ptr);
      assert(PtrAccessFn->getType()->isPointerTy());
      assert(SE.getPointerBase(PtrAccessFn) == PtrBase);

      // ptr = base + offset  <=>  offset = ptr - base
      const SCEV *ByteOffset = SE.getMinusSCEV(PtrAccessFn, PtrBase);
      LLVM_DEBUG(dbgs() << "  SCEV expression for the offset into alloca: "
                        << *ByteOffset << "\n");
      assert(ByteOffset->getType()->isIntegerTy());

      return ByteOffset;
    }

    Value *reload(ScalarLoadStoreRewriter &Rewriter) const {
      // Alloca is <x x i8>, Load that whole vector.
      return Rewriter.Builder.CreateAlignedLoad(
          Rewriter.AllocaTy, Rewriter.AI, Rewriter.getAllocaInst().getAlign());
    }

    void spill(ScalarLoadStoreRewriter &Rewriter, Value *Contents) const {
      // And then, bitcast vector of integers to the storage type.
      Contents = Rewriter.Builder.CreateBitCast(Contents, Rewriter.AllocaTy);
      // And finally, store whole alloca.
      Rewriter.Builder.CreateAlignedStore(Contents, Rewriter.AI,
                                          Rewriter.getAllocaInst().getAlign());
    }

    AbstractSolution(Instruction &I_) : I(I_) {}

    virtual ~AbstractSolution() = default;

    virtual void commit(ScalarLoadStoreRewriter &Rewriter) const = 0;
  };

  struct AllocaSolution final : public AbstractSolution {
    AllocaSolution(const ScalarLoadStoreRewriter &Rewriter, AllocaInst &AI)
        : AbstractSolution(AI) {
      ++NumAllocaInstRewriteEvaluated;
      ++NumAllocaInstViable;

      if (AI.getAllocatedType() == Rewriter.AllocaTy) {
        LLVM_DEBUG(dbgs() << "  already allocating the right type/size!\n");
        Status = SolutionViability::ViableKeepAsIs;
      }

      // We can always change the allocation type.
      Status = SolutionViability::Viable;
    }

    void commit(ScalarLoadStoreRewriter &Rewriter) const final {
      assert(Status == SolutionViability::Viable);

      AllocaInst *OldAI = &Rewriter.getAllocaInst();

      BuilderType &Builder = Rewriter.Builder;
      Builder.SetInsertPoint(cast<Instruction>(OldAI));

      // First, create new alloca with the new, correct type.
      AllocaInst *NewAI = Builder.CreateAlloca(
          Rewriter.AllocaTy, OldAI->getType()->getAddressSpace(),
          /*ArraySize=*/nullptr, OldAI->getName() + ".apc.retyped");
      NewAI->setAlignment(OldAI->getAlign());
      // And then, replace uses of old alloca with an appropriately-typed
      // pointer to the new alloca.
      Value *AllocaTypeGlue = Builder.CreateBitCast(NewAI, OldAI->getType(),
                                                    OldAI->getName() + ".apc");
      OldAI->replaceAllUsesWith(AllocaTypeGlue);
      Rewriter.KillList.emplace_back(OldAI);
      Rewriter.AI = NewAI;

      ++NumAllocaInstRewritten;
    };
  };

  struct MemSetSolution final : public AbstractSolution {
    /// The range of bytes of an alloca we are setting.
    uint64_t Begin = 0, End = 0;

    MemSetSolution(ScalarLoadStoreRewriter &Rewriter, MemSetInst &MSI)
        : AbstractSolution(MSI) {
      ++NumMemSetInstRewriteEvaluated;

      Value *Ptr = MSI.getDest();
      assert(GetUnderlyingObject(Ptr, Rewriter.DL, /*DepthLimit=*/0) ==
             Rewriter.AI);

      // We can only deal with simple non-volatile, non-atomic memsets.
      if (MSI.isVolatile()) {
        LLVM_DEBUG(dbgs() << "  Not a simple memset.\n");
        return;
      }

      auto *ByteOffsetConstant =
          dyn_cast<SCEVConstant>(getByteOffsetIntoAlloca(Rewriter, Ptr));
      if (!ByteOffsetConstant) {
        LLVM_DEBUG(dbgs() << "  the offset into alloca is not a constant.\n");
        return;
      }

      const SCEV *ByteLength = Rewriter.SE.getSCEV(MSI.getLength());
      assert(ByteLength->getType()->isIntegerTy());
      LLVM_DEBUG(dbgs() << "  SCEV expression for the number of bytes to set: "
                        << *ByteLength << "\n");
      auto *ByteLengthConstant = dyn_cast<SCEVConstant>(ByteLength);
      if (!ByteLengthConstant) {
        LLVM_DEBUG(dbgs() << "  the offset into alloca is not a constant.\n");
        return;
      }

      // Okay, by now we know this is something that we could handle.
      // Schedule the old memset to be deleted.
      Rewriter.KillList.emplace_back(&MSI);

      // Which bytes of an alloca are we actually setting?
      ConstantRange AllocaRange(APInt::getNullValue(64),
                                Rewriter.AllocationByteSize->getAPInt());
      ConstantRange MemSetRange(ByteOffsetConstant->getAPInt(),
                                ByteOffsetConstant->getAPInt().uadd_sat(
                                    ByteLengthConstant->getAPInt()));
      MemSetRange = MemSetRange.intersectWith(AllocaRange);
      LLVM_DEBUG(dbgs() << "  alloca bytes to set: " << MemSetRange << "\n");
      if (MemSetRange.isEmptySet()) {
        // None of the alloca bytes are being modified. MemSet is NOP/dead.
        LLVM_DEBUG(dbgs() << "  no alloca bytes are touched, nothing to do!\n");
        Status = SolutionViability::ViableNoCommitNeeded;
        return;
      }

      // We know how to deal with it!
      Begin = MemSetRange.getLower().getZExtValue();
      End = MemSetRange.getUpper().getZExtValue();
      Status = SolutionViability::Viable;
      ++NumMemSetInstViable;
    }

    Value *getMemSetShuffleParameters(ScalarLoadStoreRewriter &Rewriter,
                                      SmallVectorImpl<int> &Mask) const {
      BuilderType &Builder = Rewriter.Builder;

      // Let's prepare to perform memset by expanding it to shuffles.

      LLVMContext &Ctx = Builder.getContext();

      uint64_t AllocaByteCount =
          Rewriter.AllocationByteSize->getAPInt().getZExtValue();

      // The pattern is a scalar byte, we need it as a single-element vector.
      Value *ValueVec = Builder.CreateInsertElement(
          /*Vec=*/UndefValue::get(
              FixedVectorType::get(Type::getInt8Ty(Ctx), /*NumElts=*/1)),
          /*NewElt=*/cast<MemSetInst>(I).getValue(), /*Idx=*/uint64_t(0));

      // Then, splat that pattern over the entire alloca-sized vector.
      Constant *ValueSplatMask = ConstantVector::getSplat(
          Rewriter.AllocaTy->getElementCount(),
          Constant::getNullValue(Type::getInt32Ty(Ctx)));
      assert(ShuffleVectorInst::isZeroEltSplatMask(ValueSplatMask));
      Value *ValueSplat = Builder.CreateShuffleVector(
          ValueVec, UndefValue::get(ValueVec->getType()), ValueSplatMask);

      // Now, actually convert the memset params into a shuffle mask.
      Mask.reserve(AllocaByteCount);
      for (uint64_t AllocaByte : seq(uint64_t(0), AllocaByteCount)) {
        int MaskElt = AllocaByte; // By default, keep byte from 1st source.
        if (AllocaByte >= Begin && AllocaByte < End) // Would memset set it?
          MaskElt += AllocaByteCount; // Then pick the byte from 2nd source.
        Mask.emplace_back(MaskElt);
      }
      assert(Mask.size() == AllocaByteCount);
      assert(ShuffleVectorInst::isIdentityMask(Mask) ||
             ShuffleVectorInst::isSelectMask(Mask));

      return ValueSplat;
    }

    void commit(ScalarLoadStoreRewriter &Rewriter) const final {
      assert(Status == SolutionViability::Viable);

      LLVM_DEBUG(dbgs() << " Rewriting " << I << "\n");

      BuilderType &Builder = Rewriter.Builder;
      Builder.SetInsertPoint(&I);

      // Get memset params as shuffle params.
      SmallVector<int, 512 / 8> Mask;
      Value *ValueSplat = getMemSetShuffleParameters(Rewriter, Mask);

      // Load alloca contents..
      Value *Contents = reload(Rewriter);
      // Actually perform memset. As per the semantics of the Mask,
      // 1st source should be the current contents, 2nd is the splatted pattern.
      Value *NewVectorContent =
          Builder.CreateShuffleVector(Contents, ValueSplat, Mask);
      // And store it.
      spill(Rewriter, NewVectorContent);

      ++NumMemSetInstRewritten;
    };
  };

  struct AbstractWideningSolution : public AbstractSolution {
    /// If we widen it, what in what vector type would it be?
    FixedVectorType *VectorTy = nullptr;

    /// And in which lane are we interested?
    const SCEV *ElementIndex = nullptr;

    /// Helpers to introspect into the instruction I.
    Value *getPointerOperand() const {
      if (auto *LI = dyn_cast<LoadInst>(&I))
        return LI->getPointerOperand();
      if (auto *SI = dyn_cast<StoreInst>(&I))
        return SI->getPointerOperand();
      llvm_unreachable("No other types possible.");
    }
    bool isSimple() const {
      if (auto *LI = dyn_cast<LoadInst>(&I))
        return LI->isSimple();
      if (auto *SI = dyn_cast<StoreInst>(&I))
        return SI->isSimple();
      llvm_unreachable("No other types possible.");
    }
    Type *getType() const {
      if (auto *LI = dyn_cast<LoadInst>(&I))
        return LI->getType();
      if (auto *SI = dyn_cast<StoreInst>(&I))
        return SI->getValueOperand()->getType();
      llvm_unreachable("No other types possible.");
    }

    AbstractWideningSolution(const ScalarLoadStoreRewriter &Rewriter,
                             Instruction &I_)
        : AbstractSolution(I_) {
      Value *Ptr = getPointerOperand();
      assert(GetUnderlyingObject(Ptr, Rewriter.DL, /*DepthLimit=*/0) ==
             Rewriter.AI);

      // We can only deal with simple non-volatile, non-atomic loads/stores.
      // FIXME: though, mem2reg is okay with atomics. So if we know we'll end
      // up promoting the alloca, we only need to ensure non-volatility.
      if (!isSimple()) {
        LLVM_DEBUG(dbgs() << "  Not a simple load/store.\n");
        return;
      }

      ScalarEvolution &SE = Rewriter.SE;

      // How many bytes are we loading here?
      Type *ElementTy = getType();
      uint64_t ElementBitCount = Rewriter.DL.getTypeSizeInBits(ElementTy);
      assert(ElementBitCount % 8 == 0);
      const auto *ElementByteSize =
          cast<SCEVConstant>(SE.getConstant(APInt(64, ElementBitCount / 8)));
      LLVM_DEBUG(dbgs() << "  Operating on " << *ElementTy << " element type ("
                        << *ElementByteSize << " bytes)\n");

      // Are we loading/storing directly from/to the correctly-typed alloca?
      if (Ptr == Rewriter.AI && ElementTy == Rewriter.AllocaTy) {
        assert(Rewriter.AllocationByteSize == ElementByteSize);
        LLVM_DEBUG(dbgs() << "  No action nessesary, already operating on the "
                             "whole alloca.\n");
        Status = SolutionViability::ViableKeepAsIs;
        return;
      }

      // Can we have a vector with this element type?
      if (!VectorType::isValidElementType(ElementTy)) {
        LLVM_DEBUG(
            dbgs() << "  Can not have a vector with that element type.\n");
        return;
      }

      // How many bytes into alloca is the Ptr?
      const SCEV *ByteOffset = getByteOffsetIntoAlloca(Rewriter, Ptr);

      // Which ElementByteSize-sized element is that?
      // I.e. try to decompose:
      //   ByteOffset = ElementByteSize * ElementIndex + RemainingByteOffset
      const SCEV *RemainingByteOffset;
      SCEVDivision::divide(SE, ByteOffset, ElementByteSize, &ElementIndex,
                           &RemainingByteOffset);
      LLVM_DEBUG(
          dbgs() << "  Decomposed offset: we're operating on element index "
                 << *ElementIndex << " with with additional byte offset of "
                 << *RemainingByteOffset << ".\n");
      if (!RemainingByteOffset->isZero()) {
        LLVM_DEBUG(dbgs() << "  Have non-zero byte offset.\n");
        return;
      }

      // How many ElementByteSize-sized elements can the alloca accomodate?
      const SCEV *MaxElements, *RemainingPaddingBytes;
      SCEVDivision::divide(SE, Rewriter.AllocationByteSize, ElementByteSize,
                           &MaxElements, &RemainingPaddingBytes);
      const auto *MaxElementsConst = cast<SCEVConstant>(MaxElements);
      LLVM_DEBUG(dbgs() << "  The alloca can accomodate " << *MaxElementsConst
                        << " elements, with remaining padding of "
                        << *RemainingPaddingBytes << " bytes.\n");
      if (!RemainingPaddingBytes->isZero()) {
        LLVM_DEBUG(dbgs() << "  Got remaining padding.\n");
        return;
      }

      // We know how to deal with it!
      VectorTy = FixedVectorType::get(
          ElementTy, MaxElementsConst->getAPInt().getZExtValue());
      LLVM_DEBUG(dbgs() << "  Choosing vector type " << *VectorTy << "\n");
      Status = SolutionViability::Viable;
    }
  };

  struct LoadSolution final : public AbstractWideningSolution {
    LoadSolution(const ScalarLoadStoreRewriter &Rewriter, LoadInst &LI)
        : AbstractWideningSolution(Rewriter, LI) {
      ++NumLoadInstRewriteEvaluated;
      if (Status != SolutionViability::Unviable)
        ++NumLoadInstViable;
    }

    LoadInst &getLoadInst() const { return cast<LoadInst>(I); }

    void commit(ScalarLoadStoreRewriter &Rewriter) const final {
      assert(Status == SolutionViability::Viable);

      LLVM_DEBUG(dbgs() << " Rewriting " << I << "\n");

      BuilderType &Builder = Rewriter.Builder;
      Builder.SetInsertPoint(&I);
      Rewriter.Expander.setInsertPoint(&I);

      // Load alloca contents..
      Value *Contents = reload(Rewriter);
      // Now, reinterpret the layout of the data we have loaded,
      // as a vector with the correct element type.

      if (VectorTy->isPtrOrPtrVectorTy()) {
        Contents = Builder.CreateBitCast(Contents,
                                         Rewriter.DL.getIntPtrType(VectorTy));
        Contents = Builder.CreateIntToPtr(Contents, VectorTy);
      } else // We can just bitcast it.
        Contents = Builder.CreateBitCast(Contents, VectorTy);

      // Which element are we after? Actually expand the SCEV expression.
      Value *ElementIndexVal = Rewriter.Expander.expandCodeFor(ElementIndex);

      // Finally, extract the element in questions.
      Value *ScalarElt =
          Builder.CreateExtractElement(Contents, ElementIndexVal);
      // And actually make old scalar load obsolete by replacing it's uses.
      getLoadInst().replaceAllUsesWith(ScalarElt);

      ++NumLoadInstRewritten;
    }
  };

  struct StoreSolution final : public AbstractWideningSolution {
    StoreSolution(const ScalarLoadStoreRewriter &Rewriter, StoreInst &SI)
        : AbstractWideningSolution(Rewriter, SI) {
      ++NumStoreInstRewriteEvaluated;
      if (Status != SolutionViability::Unviable)
        ++NumStoreInstViable;
    }

    StoreInst &getStoreInst() const { return cast<StoreInst>(I); }

    void commit(ScalarLoadStoreRewriter &Rewriter) const final {
      assert(Status == SolutionViability::Viable);

      LLVM_DEBUG(dbgs() << " Rewriting " << I << "\n");

      BuilderType &Builder = Rewriter.Builder;
      Builder.SetInsertPoint(&I);
      Rewriter.Expander.setInsertPoint(&I);

      // Load alloca contents..
      Value *Contents = reload(Rewriter);
      // Now, reinterpret the layout of the data we have loaded,
      // as a <y x ???> vector with the correct element type.

      if (VectorTy->isPtrOrPtrVectorTy()) {
        Contents = Builder.CreateBitCast(Contents,
                                         Rewriter.DL.getIntPtrType(VectorTy));
        Contents = Builder.CreateIntToPtr(Contents, VectorTy);
      } else // We can just bitcast it.
        Contents = Builder.CreateBitCast(Contents, VectorTy);

      // Which element are we after? Actually expand the SCEV expression.
      Value *ElementIndexVal = Rewriter.Expander.expandCodeFor(ElementIndex);

      // Now that we have the current contents of the alloca,
      // replace (only!) the element in question.
      Contents = Builder.CreateInsertElement(
          Contents, getStoreInst().getValueOperand(), ElementIndexVal);

      if (VectorTy->isPtrOrPtrVectorTy())
        Contents = Builder.CreatePtrToInt(Contents,
                                          Rewriter.DL.getIntPtrType(VectorTy));

      // And store it.
      spill(Rewriter, Contents);

      // And schedule the old store to be deleted.
      Rewriter.KillList.emplace_back(&getStoreInst());

      ++NumStoreInstRewritten;
    }
  };

  struct {
    SmallVector<MemSetSolution, 4> MemSets;
    SmallVector<LoadSolution, 4> Loads;
    SmallVector<StoreSolution, 4> Stores;
  } Solutions;

  SmallVector<Instruction *, 16> KillList;

public:
  ScalarLoadStoreRewriter(const DataLayout &DL_, ScalarEvolution &SE_,
                          SCEVExpander &Expander_, WeakTrackingVH &AI_)
      : DL(DL_), SE(SE_), Builder(SE.getContext(), TargetFolder(DL)),
        Expander(Expander_), AI(AI_),
        AllocationByteSize(cast<SCEVConstant>(SE.getConstant(
            APInt(64, GetNumBytesAvaliableInAnAlloca(DL, getAllocaInst()))))),
        AllocaTy(FixedVectorType::get(
            Type::getInt8Ty(AI->getContext()),
            AllocationByteSize->getAPInt().getZExtValue())) {
    // Fact: Mem2Reg does not like bitcasts.
    // So we must not have any bitcasts between the alloca and load/store.
    // Which means we need to bitcast the loaded/to-be-stored value.
    // Since we can't bitcast between an array and a vector, we should allocate
    // a vector to begin with. And to be most friendly to the poison rules,
    // the allocation type therefore should be a byte (i8).

    LLVM_DEBUG(dbgs() << "Analyzing alloca " << *AI << "  ("
                      << *AllocationByteSize << " bytes)\n");
  }

  /// Follow uses of the alloca and analyze which ones we'll need to rewrite.
  /// from it.
  Optional<bool> analyze() {
    Optional<bool> CanFullyRewrite;
    bool HasAnyRewrites = false;
    enqueueUses(*cast<Instruction>(AI));
    while (!Queue.empty()) {
      U = Queue.pop_back_val();
      User *User = U->getUser();
      LLVM_DEBUG(dbgs() << " Analyzing user " << *User << "  of " << *U->get()
                        << "\n");

      Optional<bool> CanRewriteInstruction = visit(cast<Instruction>(User));
      ++NumInstrsVisitedDuringRewriteEvalation;

      if (!CanRewriteInstruction.hasValue()) {
        LLVM_DEBUG(dbgs() << "  Does not affect rewrite viability.\n");
        ++NumInstrsNotAffectingRewriteViability;
        continue;
      }

      bool IsRewriteable = CanRewriteInstruction.getValue();
      HasAnyRewrites |= IsRewriteable;
      if (IsRewriteable) {
        LLVM_DEBUG(dbgs() << "  OK, can rewrite!\n");
        ++NumRewriteableInstrs;
      } else {
        LLVM_DEBUG(dbgs() << "  Unable to handle!\n");
        ++NumNonRewritableInstrs;

        if (RewriteOnlyIfAllowsPromotion) {
          LLVM_DEBUG(dbgs() << " Not analyzing alloca further.\n");
          return false;
        }
      }

      CanFullyRewrite = CanFullyRewrite.getValueOr(true) & IsRewriteable;
    }
    LLVM_DEBUG(dbgs() << "Finished full analysis of an alloca.\n");
    return HasAnyRewrites ? CanFullyRewrite : None;
  }

  /// Commit rewrite, actually apply changes.
  void rewrite() {
    // First, rewrite the alloca into the expected form, if needed.
    AllocaSolution AS(*this, getAllocaInst());
    if (AS.Status !=
        AbstractWideningSolution::SolutionViability::ViableKeepAsIs)
      AS.commit(*this);

    // Then, apply each solution, thus rewriting all the rewritable uses.
    for_each(Solutions.MemSets, [this](const auto &MSS) { MSS.commit(*this); });
    for_each(Solutions.Loads, [this](const auto &LS) { LS.commit(*this); });
    for_each(Solutions.Stores, [this](const auto &SS) { SS.commit(*this); });

    // Now that we've rewritten, but before we delete instructions,
    // migrate the def-use graph into a more stable form for later use.
    SmallVector<WeakTrackingVH, 8> DCEList(AllocaDefUseGraph.begin(),
                                           AllocaDefUseGraph.end());

    // Erase all the instructions we've scheduled to be killed.
    // This might null-out some entries in the DCEList already.
    while (!KillList.empty())
      KillList.pop_back_val()->eraseFromParent();

    // And finally cleanup all the hopefully now-dead instructions.
    RecursivelyDeleteTriviallyDeadInstructionsPermissive(DCEList);
  }

private:
  /// Enqueue all the users of the given instruction for further processing.
  /// This uses a set to de-duplicate users.
  void enqueueUses(Instruction &I) {
    // Does this instruction have any uses?
    for (Use &U : I.uses()) {
      // Queue them for analysis.
      if (Visited.insert(&U).second)
        Queue.push_back(&U);
      // Also, collect the def-use graph.
      AllocaDefUseGraph.insert(cast<Instruction>(U.getUser()));
    }
  }

  // Conservative default is to not do anything.
  bool visitInstruction(Instruction &I) {
    return false; // Don't know about these.
  }

  template <typename SolutionTy, typename InstrTy>
  Optional<bool>
  genericSolutionConstructionLogic(SmallVectorImpl<SolutionTy> &Storage,
                                   InstrTy &I) {
    Storage.emplace_back(*this, I);
    auto Viability = Storage.back().Status;

    if (Viability != AbstractWideningSolution::SolutionViability::Viable)
      Storage.pop_back();

    if (Viability ==
        AbstractWideningSolution::SolutionViability::ViableKeepAsIs)
      return None; // This instruction doesn't affect viability assessment.

    return Viability != AbstractWideningSolution::SolutionViability::Unviable;
  }

  bool visitCallInst(CallInst &CI) {
    if (const Function *F = CI.getCalledFunction()) {
      switch (F->getIntrinsicID()) {
      case Intrinsic::lifetime_start:
      case Intrinsic::lifetime_end:
        assert(CI.getOperand(1) == *U);
        KillList.emplace_back(&CI);
        return true; // Can just drop these.
        break;
      default:
        break;
      }
    }
    return false; // Don't know about these.
  }

  Optional<bool> visitMemSetInst(MemSetInst &MSI) {
    assert(MSI.getRawDest() == *U);
    return genericSolutionConstructionLogic(Solutions.MemSets, MSI);
  }

  Optional<bool> visitLoadInst(LoadInst &LI) {
    assert(LI.getPointerOperand() == *U);
    return genericSolutionConstructionLogic(Solutions.Loads, LI);
  }

  Optional<bool> visitStoreInst(StoreInst &SI) {
    if (SI.getPointerOperand() != *U) {
      LLVM_DEBUG(dbgs() << "  Storing the address of alloca itself!\n");
      return false; // Can't rewrite.
    }
    return genericSolutionConstructionLogic(Solutions.Stores, SI);
  }

  Optional<bool> visitBitCastInst(BitCastInst &BC) {
    enqueueUses(BC);
    return None; // This instruction doesn't affect viability assessment.
  }

  Optional<bool> visitAddrSpaceCastInst(AddrSpaceCastInst &ASC) {
    enqueueUses(ASC);
    return None; // This instruction doesn't affect viability assessment.
  }

  Optional<bool> visitGetElementPtrInst(GetElementPtrInst &GEPI) {
    enqueueUses(GEPI);
    return None; // This instruction doesn't affect viability assessment.
  }

  // FIXME: handle running pointer increment via PHI nodes.
};

class WidenAllocaLoadsStores {
  Function &F;
  ScalarEvolution &SE;
  SCEVExpander Expander;

  /// Should we try to improve alloca's chances of being promotable?
  static bool shouldTryToRewrite(AllocaInst &AI) {
    if (!AI.isStaticAlloca() || isAllocaPromotable(&AI))
      return false;
    uint64_t AllocaByteSize =
        GetNumBytesAvaliableInAnAlloca(AI.getModule()->getDataLayout(), AI);
    return AllocaByteSize > 0 && AllocaByteSize <= 32767;
  }

  bool tryToRewriteAlloca(WeakTrackingVH &AI) {
    ++NumAllocasRewriteEvaluated;

    ScalarLoadStoreRewriter Rewriter(
        cast<Instruction>(AI)->getModule()->getDataLayout(), SE, Expander, AI);

    Optional<bool> CanFullyRewrite = Rewriter.analyze();

    if (!CanFullyRewrite.hasValue()) {
      LLVM_DEBUG(dbgs() << "No rewrites to perform! Giving up.\n");
      ++NumAllocasNoRewritingPossible;
      return false; // No changes done.
    }

    // FIXME: if all rewrites end up operating on <1 x ???>, shouldn't rewrite.

    bool FullRewrite = CanFullyRewrite.getValue();
    if (FullRewrite) {
      LLVM_DEBUG(dbgs() << "Can fully rewrite alloca!\n");
      ++NumAllocasFullRewritePossible;
    } else {
      LLVM_DEBUG(dbgs() << "Can only partially rewrite alloca.\n");
      ++NumAllocasPartialRewritePossibleOnly;
    }

    if (RewriteOnlyIfAllowsPromotion && !FullRewrite) {
      LLVM_DEBUG(dbgs() << "Rewriting will not allow promotion. Giving up.\n");
      ++NumAllocasPartialRewriteWillNotAllowPromotion;
      return false; // No changes done.
    }

    LLVM_DEBUG(dbgs() << "Performing rewrite.\n");
    Rewriter.rewrite();
    ++NumAllocasRewritten;
    LLVM_DEBUG(dbgs() << "Done rewriting.\n");

    if (!AI) {
      LLVM_DEBUG(dbgs() << "Alloca got DCE'd!\n");
      ++NumAllocasDCE;
      return true; // Changes applied!
    }

    bool AllocaBecamePromoteable = isAllocaPromotable(cast<AllocaInst>(AI));
    (void)AllocaBecamePromoteable;

    if (AllocaBecamePromoteable) {
      LLVM_DEBUG(dbgs() << "Alloca did became promoteable!\n");
      ++NumAllocasBecamePromoteable;
    } else {
      LLVM_DEBUG(dbgs() << "Rewrite did not make alloca promoteable.\n");
    }

    assert((!RewriteOnlyIfAllowsPromotion || AllocaBecamePromoteable) &&
           "We rewrote alloca with expectation/understanding that it would "
           "become promotable, but it didn't.");

    return true; // Changes applied!
  }

  bool runImpl() {
    LLVM_DEBUG(dbgs() << "\nAnalyzing allocas in function " << F.getName()
                      << "\n");

    // First, collect all the allocas we might care about,
    // because we are likely to invalidate iteration order.
    SmallVector<WeakTrackingVH, 16> Allocas;
    for (auto &I : F.getEntryBlock()) {
      auto *AI = dyn_cast<AllocaInst>(&I);
      if (!AI)
        continue;

      ++NumAllocasSeen;

      if (!shouldTryToRewrite(*AI)) {
        LLVM_DEBUG(dbgs() << " Skipping " << *AI << "\n");
        ++NumAllocasIgnored;
        continue;
      }

      Allocas.emplace_back(AI);
    }

    // And now, attempt to rewrite each alloca.
    bool Changed = false;
    for (WeakTrackingVH &AI : Allocas) {
      Changed |= tryToRewriteAlloca(AI);
      assert((!AI || !tryToRewriteAlloca(AI)) &&
             "Should reach alloca-local fixpoint on the first run.");
      LLVM_DEBUG(dbgs() << "\n");
    }

    return Changed;
  }

public:
  WidenAllocaLoadsStores(Function &F_, ScalarEvolution &SE_)
      : F(F_), SE(SE_), Expander(SE, SE.getDataLayout(), DEBUG_TYPE) {
#ifndef NDEBUG
    Expander.setDebugType(DEBUG_TYPE);
#endif
  }

  bool run() {
    if (!EnableAllocaPromotionCoercionPass)
      return false;

    bool Changed = runImpl();
    assert(!runImpl() &&
           "Should reach function-global fixpoint on the first run.");
    return Changed;
  }
};

} // end anonymous namespace

PreservedAnalyses
AllocaPromotionCoercionPass::run(Function &F, FunctionAnalysisManager &AM) {
  auto &SE = AM.getResult<ScalarEvolutionAnalysis>(F);

  if (!WidenAllocaLoadsStores(F, SE).run())
    return PreservedAnalyses::all();

  PreservedAnalyses PA;
  PA.preserveSet<CFGAnalyses>();
  return PA;
}

namespace {

struct AllocaPromotionCoercionLegacyPass : public FunctionPass {
  // Pass identification, replacement for typeid
  static char ID;

  AllocaPromotionCoercionLegacyPass() : FunctionPass(ID) {
    initializeAllocaPromotionCoercionLegacyPassPass(
        *PassRegistry::getPassRegistry());
  }

  bool runOnFunction(Function &F) override {
    if (skipFunction(F))
      return false;

    auto *SE = &getAnalysis<ScalarEvolutionWrapperPass>().getSE();

    return WidenAllocaLoadsStores(F, *SE).run();
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();

    AU.addRequired<ScalarEvolutionWrapperPass>();
  }
};

} // end anonymous namespace

char AllocaPromotionCoercionLegacyPass::ID = 0;

INITIALIZE_PASS_BEGIN(AllocaPromotionCoercionLegacyPass, DEBUG_TYPE,
                      "rewrite allocas if that makes them promotable", false,
                      false)
INITIALIZE_PASS_DEPENDENCY(ScalarEvolutionWrapperPass)
INITIALIZE_PASS_END(AllocaPromotionCoercionLegacyPass, DEBUG_TYPE,
                    "rewrite allocas if that makes them promotable", false,
                    false)

// createAllocaPromotionCoercion - Provide an entry point to create this pass.
FunctionPass *llvm::createAllocaPromotionCoercionPass() {
  return new AllocaPromotionCoercionLegacyPass();
}
