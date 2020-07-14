//===-- LoopSink.cpp - Loop Sink Pass -------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This pass does the inverse transformation of what LICM does.
// It traverses all of the instructions in the loop's preheader and sinks
// them to the loop body. It differs from the Sink pass in the following ways:
//
// * It only handles sinking of instructions from the loop's preheader to the
//   loop's body
// * It uses alias set tracker to get more accurate alias info
//
//===----------------------------------------------------------------------===//

#include "llvm/Transforms/Scalar/LoopSink.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/ilist_iterator.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/AliasSetTracker.h"
#include "llvm/Analysis/DomTreeUpdater.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/LoopPass.h"
#include "llvm/Analysis/MemorySSA.h"
#include "llvm/Analysis/MemorySSAUpdater.h"
#include "llvm/Analysis/MustExecute.h"
#include "llvm/Analysis/OptimizationRemarkEmitter.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/User.h"
#include "llvm/IR/Value.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/PassRegistry.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/LoopUtils.h"
#include <cassert>
#include <vector>

using namespace llvm;

#define DEBUG_TYPE "loop-sink"

STATISTIC(NumLoopSunk, "Number of instructions sunk into loop body");

/// If out of all the successor basic blocks of \p BB basic block there is only
/// a single basic block that is _inside of the current loop_, return it.
static BasicBlock *GetSingleInLoopSuccessor(BasicBlock *BB, const Loop &L) {
  BasicBlock *SingleInLoopSuccessorBB = nullptr;
  for (BasicBlock *SuccessorBB : successors(BB)) {
    if (!L.contains(SuccessorBB))
      continue;                  // Exit basic block, ignore.
    if (SingleInLoopSuccessorBB) // Did we already find in-loop successor bb?
      return nullptr;            // More than one in-loop successor basic block!
    SingleInLoopSuccessorBB = SuccessorBB;
  }
  return SingleInLoopSuccessorBB;
}

class SinkLoopHeaderInstructions {
  /// On which loop are we operating?
  Loop &L;

  /// Misc provided analysises.
  AAResults &AA;
  DominatorTree &DT;
  LoopInfo &LI;
  MemorySSAUpdater *MSSAU; // Optional, but recommended.
  ScalarEvolution *SE;     // Optional.
  OptimizationRemarkEmitter &ORE;

  /// From which basic block are we sinking instructions from?
  BasicBlock &HeaderBB;

  /// The terminator instruction of the header basic block.
  BranchInst *HeaderBrInst;

  /// To which in-loop basic block would we branch from header basic block?
  /// Note that this is the *only* in-loop basic block we could branch to
  /// from loop header.
  BasicBlock *BodyBB;

  /// Into which in-loop basic block are we going to sink?
  /// Lazily populated, do not use directly.
  Optional<BasicBlock *> PostHeaderBB;
  /// Lazily creates in-loop sink basic block and returns a pointer to it.
  BasicBlock *getPostHeaderBB();

  /// (non-MemSSA-only) Alias set. Lazily populated, do not use directly.
  Optional<AliasSetTracker> AST;
  /// Lazily populates AST and returns a reference to it.
  AliasSetTracker *getAST();

  /// Loop safety information. Lazily populated, do not use directly.
  Optional<ICFLoopSafetyInfo> LSI;
  /// Lazily populates LSI and returns a reference to it.
  ICFLoopSafetyInfo &getLSI();

  /// (MemSSA only) LICM legality checks require certain configureation flags.
  Optional<SinkAndHoistLICMFlags> LICMFlags;
  SinkAndHoistLICMFlags *getLICMFlags();

  /// Check that everything is still in consistent state.
  void verify() const;

  /// Sinks \p I from the loop \p L's header to its uses.
  /// Returns true if sinking is successful.
  bool sinkInstruction(Instruction &I);

public:
  SinkLoopHeaderInstructions(Loop &L, AAResults &AA, DominatorTree &DT,
                             LoopInfo &LI, MemorySSAUpdater *MSSAU,
                             ScalarEvolution *SE,
                             OptimizationRemarkEmitter &ORE);

  /// Sink instructions from loop's header.
  bool run();
};

SinkLoopHeaderInstructions::SinkLoopHeaderInstructions(
    Loop &L_, AAResults &AA_, DominatorTree &DT_, LoopInfo &LI_,
    MemorySSAUpdater *MSSAU_, ScalarEvolution *SE_,
    OptimizationRemarkEmitter &ORE_)
    : L(L_), AA(AA_), DT(DT_), LI(LI_), MSSAU(MSSAU_), SE(SE_), ORE(ORE_),
      HeaderBB(*L.getHeader()),
      HeaderBrInst(dyn_cast<BranchInst>(HeaderBB.getTerminator())),
      BodyBB(GetSingleInLoopSuccessor(&HeaderBB, L)) {}

BasicBlock *SinkLoopHeaderInstructions::getPostHeaderBB() {
  if (!PostHeaderBB.hasValue()) {
    DomTreeUpdater DTU(DT, DomTreeUpdater::UpdateStrategy::Lazy);
    PostHeaderBB = InsertFallthroughBlock(&HeaderBB, BodyBB, &DTU, &LI);
    if (!getLSI().getBlockColors().empty())
      getLSI().copyColors(*PostHeaderBB, &HeaderBB);
  }

  return *PostHeaderBB;
}

AliasSetTracker *SinkLoopHeaderInstructions::getAST() {
  // If we have MemorySSA, then refuse to provide AST.
  if (MSSAU)
    return nullptr; // No, use MemorySSA instead!

  // Did we not populate AST yet?
  if (!AST.hasValue()) {
    AST.emplace(AA, /*MSSA=*/nullptr, &L);
    for (BasicBlock *BB : L.blocks())
      AST->add(*BB);
  }

  return AST.getPointer();
}

ICFLoopSafetyInfo &SinkLoopHeaderInstructions::getLSI() {
  // Did we not compute loop safety information yet?
  if (!LSI.hasValue()) {
    LSI.emplace();
    LSI->computeLoopSafetyInfo(&L);
  }

  return *LSI;
}

SinkAndHoistLICMFlags *SinkLoopHeaderInstructions::getLICMFlags() {
  if (!MSSAU)
    return nullptr;

  assert(!getAST() && "AST should not be provided when MemSSA is present!");

  if (!LICMFlags.hasValue())
    LICMFlags.emplace(SinkAndHoistLICMFlags{
        /*NoOfMemAccTooLarge=*/false, /*LicmMssaOptCounter=*/0,
        /*LicmMssaOptCap=*/~0U, /*LicmMssaNoAccForPromotionCap=*/~0U,
        /*IsSink=*/true});

  return LICMFlags.getPointer();
}

void SinkLoopHeaderInstructions::verify() const {
  assert(DT.verify(DominatorTree::Base::VerificationLevel::Full) &&
         "Incorrect domtree!");
  LI.verify(DT);
  L.isRecursivelyLCSSAForm(DT, LI);
  if (MSSAU)
    MSSAU->getMemorySSA()->verifyMemorySSA();
}

bool SinkLoopHeaderInstructions::sinkInstruction(Instruction &I) {
  bool Changed = false;

  LLVM_DEBUG(dbgs() << "LoopSink:   attempting to sink instruction.\n");
  LLVM_DEBUG(dbgs() << "LoopSink:   rewriting out-of-loop uses.\n");

  // First (try to) rewrite uses of instruction I outside of the loop, if any.
  Changed |= sinkOutOfLoop(I, &LI, &DT, &L, &getLSI(), MSSAU, &ORE);

  LLVM_DEBUG(dbgs() << "LoopSink:   done rewriting out-of-loop uses.\n");

  // Is instruction I *still* has uses outside of the loop?
  if (any_of(I.users(),
             [&](User *U) { return !L.contains(cast<Instruction>(U)); })) {
    LLVM_DEBUG(dbgs() << "LoopSink:   failed to rewrite all out-of-loop uses, "
                         "giving up on instruction!\n");
    return Changed;
  }

  LLVM_DEBUG(dbgs() << "LoopSink:   no out-of-loop uses remaining!\n");

  Changed |= true;

  LLVM_DEBUG(dbgs() << "LoopSink:   sinking instruction from loop header.\n");

  moveInstructionBefore(I, getPostHeaderBB()->front(), getLSI(), MSSAU,
                        /*SE=*/nullptr);

  LLVM_DEBUG(dbgs() << "LoopSink:   successfully sunk the instruction!\n");
  ++NumLoopSunk;

  return Changed;
}

bool SinkLoopHeaderInstructions::run() {
  LLVM_DEBUG(dbgs() << "LoopSink: in function  "
                    << HeaderBB.getParent()->getName() << "  analyzing  " << L);

  bool Changed = false;

  // The header's terminator must be a conditional branch and here must be
  // a single in-loop successor basic block of header basic block,
  if (!HeaderBrInst || HeaderBrInst->isUnconditional() || !BodyBB) {
    LLVM_DEBUG(dbgs() << "LoopSink: unexpected loop structure, giving up on "
                         "the whole loop.\n");
    return Changed;
  }

  // Traverse header's instructions in reverse order becaue if A depends on B
  // (A appears after B), A needs to be sinked first before B can be sinked.
  for (Instruction &I :
       make_early_inc_range(make_range(HeaderBB.rbegin(), HeaderBB.rend()))) {
    LLVM_DEBUG(dbgs() << "LoopSink:  analyzing instruction " << I << "\n");
    if (I.mayHaveSideEffects() || I.isUsedInBasicBlock(I.getParent()) ||
        !canSinkOrHoistInst(I, &AA, &DT, &L, getAST(), MSSAU,
                            /*TargetExecutesOncePerLoop=*/false, getLICMFlags(),
                            &ORE)) {
      LLVM_DEBUG(
          dbgs()
          << "LoopSink:   sink legality check failed, ignoring instruction.\n");
      continue;
    }
    LLVM_DEBUG(dbgs() << "LoopSink:   legal to sink!\n");
    if (!sinkInstruction(I))
      continue;
    Changed = true;
    verify();
  }

  if (Changed && SE) {
    SE->forgetLoopDispositions(&L);
    SE->verify();
  }

  return Changed;
}

PreservedAnalyses LoopSinkPass::run(Function &F, FunctionAnalysisManager &FAM) {
  auto &LI = FAM.getResult<LoopAnalysis>(F);
  // Nothing to do if there are no loops.
  if (LI.empty())
    return PreservedAnalyses::all();

  auto &AA = FAM.getResult<AAManager>(F);
  auto &DT = FAM.getResult<DominatorTreeAnalysis>(F);
  auto *SE = FAM.getCachedResult<ScalarEvolutionAnalysis>(F);

  Optional<MemorySSAUpdater> MSSAU;
  if (EnableMSSALoopDependency)
    MSSAU = MemorySSAUpdater(&FAM.getResult<MemorySSAAnalysis>(F).getMSSA());

  // For the new PM, we also can't use OptimizationRemarkEmitter as an analysis
  // pass.  Function analyses need to be preserved across loop transformations
  // but ORE cannot be preserved (see comment before the pass definition).
  OptimizationRemarkEmitter ORE(&F);

  // We want to do a postorder walk over the loops. Since loops are a tree this
  // is equivalent to a reversed preorder walk and preorder is easy to compute
  // without recursion. Since we reverse the preorder, we will visit siblings
  // in reverse program order. This isn't expected to matter at all but is more
  // consistent with sinking algorithms which generally work bottom-up.
  SmallVector<Loop *, 4> PreorderLoops = LI.getLoopsInPreorder();

  bool Changed = false;
  do {
    Loop &L = *PreorderLoops.pop_back_val();
    Changed |= SinkLoopHeaderInstructions(
                   L, AA, DT, LI,
                   MSSAU.hasValue() ? MSSAU.getPointer() : nullptr, SE, ORE)
                   .run();
  } while (!PreorderLoops.empty());

  if (!Changed)
    return PreservedAnalyses::all();

  PreservedAnalyses PA;
  PA.preserve<LoopAnalysis>();
  PA.preserve<AAManager>();
  PA.preserve<DominatorTreeAnalysis>();
  if (MSSAU)
    PA.preserve<MemorySSAAnalysis>();
  if (SE)
    PA.preserve<ScalarEvolutionAnalysis>();
  return PA;
}

namespace {
struct LegacyLoopSinkPass : public LoopPass {
  static char ID;

  LegacyLoopSinkPass() : LoopPass(ID) {
    initializeLegacyLoopSinkPassPass(*PassRegistry::getPassRegistry());
  }

  bool runOnLoop(Loop *L, LPPassManager &LPM) override {
    if (skipLoop(L))
      return false;

    auto &AA = getAnalysis<AAResultsWrapperPass>().getAAResults();
    auto &DT = getAnalysis<DominatorTreeWrapperPass>().getDomTree();
    auto &LI = getAnalysis<LoopInfoWrapperPass>().getLoopInfo();
    auto *SE = getAnalysisIfAvailable<ScalarEvolutionWrapperPass>();

    Optional<MemorySSAUpdater> MSSAU;
    if (EnableMSSALoopDependency)
      MSSAU = MemorySSAUpdater(&getAnalysis<MemorySSAWrapperPass>().getMSSA());

    // For the old PM, we can't use OptimizationRemarkEmitter as an analysis
    // pass.  Function analyses need to be preserved across loop transformations
    // but ORE cannot be preserved (see comment before the pass definition).
    OptimizationRemarkEmitter ORE(L->getHeader()->getParent());

    return SinkLoopHeaderInstructions(
               *L, AA, DT, LI, MSSAU.hasValue() ? MSSAU.getPointer() : nullptr,
               SE ? &SE->getSE() : nullptr, ORE)
        .run();
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    if (EnableMSSALoopDependency) {
      AU.addRequired<MemorySSAWrapperPass>();
      AU.addPreserved<MemorySSAWrapperPass>();
    }
    getLoopAnalysisUsage(AU);
  }
};
} // namespace

char LegacyLoopSinkPass::ID = 0;

INITIALIZE_PASS_BEGIN(LegacyLoopSinkPass, DEBUG_TYPE,
                      "Sink Instructions From Loop Header", false, false)
INITIALIZE_PASS_DEPENDENCY(LoopPass)
INITIALIZE_PASS_DEPENDENCY(AAResultsWrapperPass)
INITIALIZE_PASS_DEPENDENCY(DominatorTreeWrapperPass)
INITIALIZE_PASS_DEPENDENCY(LoopInfoWrapperPass)
INITIALIZE_PASS_DEPENDENCY(ScalarEvolutionWrapperPass)
INITIALIZE_PASS_DEPENDENCY(MemorySSAWrapperPass)
INITIALIZE_PASS_END(LegacyLoopSinkPass, DEBUG_TYPE,
                    "Sink Instructions From Loop Header", false, false)

Pass *llvm::createLoopSinkPass() { return new LegacyLoopSinkPass(); }
