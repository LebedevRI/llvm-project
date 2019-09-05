//===-- CountMIAnalysis.cpp - Count specific MachineInstrs 000000----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This implements a machine function analysis pass that visits every machine
// function, every machine basic block, every machine instruction,
// and counts how many branches and conditional moves there are.
//
//===----------------------------------------------------------------------===//

#include "X86InstrInfo.h"
#include "X86Subtarget.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "x86-mi-counting"

STATISTIC(NumMachineFunctions, "Number of machine functions");
STATISTIC(NumMachineBasicBlocks, "Number of machine basic blocks");
STATISTIC(NumMachineInstructions, "Number of machine instructions");
STATISTIC(NumUncondBR, "Number of unconditional branch instructions");
STATISTIC(NumCondBR, "Number of conditional branch instructions");
STATISTIC(NumCMOV, "Number of conditional move instructions");
STATISTIC(NumVecCMOV, "Number of vector conditional move instructions");
STATISTIC(NumVecBlend, "Number of vector conditional blend instructions");

namespace {

class X86CountMI : public MachineFunctionPass {
public:
  static char ID;

  /// Default construct and initialize the pass.
  X86CountMI() : MachineFunctionPass(ID) {}

  /// Tell the pass manager which passes we depend on and what information we
  /// preserve.
  void getAnalysisUsage(AnalysisUsage &AU) const override {
    // We preserve all information.
    AU.setPreservesAll();
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  /// Perform the counting of the interesting instructions
  /// in the given machine function.
  bool runOnMachineFunction(MachineFunction &MF) override;

private:
  /// Perform the counting of the interesting instructions
  /// in the given machine basic block.
  void runOnMachineBasicBlock(const MachineBasicBlock &MBB);

  /// Perform the counting of the interesting instructions,
  /// given the machine instruction.
  void runOnMachineInstruction(const MachineInstr &MI);
};

} // namespace

char X86CountMI::ID = 0;

INITIALIZE_PASS(X86CountMI, DEBUG_TYPE, "X86 Machine Instruction counting",
                false, false)

bool X86CountMI::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(dbgs() << "********** COUNT MACHINE INSTRUCTIONS: " << MF.getName()
                    << " **********\n");
  ++NumMachineFunctions;

  for (const MachineBasicBlock &MBB : MF)
    runOnMachineBasicBlock(MBB);

  return false; // No changes done.
}

void X86CountMI::runOnMachineBasicBlock(const MachineBasicBlock &MBB) {
  ++NumMachineBasicBlocks;

  for (const MachineInstr &MI : MBB)
    runOnMachineInstruction(MI);
}

void X86CountMI::runOnMachineInstruction(const MachineInstr &MI) {
  ++NumMachineInstructions;

  switch (MI.getOpcode()) {
  case X86::JMP16m:
  case X86::JMP16m_NT:
  case X86::JMP16r:
  case X86::JMP16r_NT:
  case X86::JMP32m:
  case X86::JMP32m_NT:
  case X86::JMP32r:
  case X86::JMP32r_NT:
  case X86::JMP64m:
  case X86::JMP64m_NT:
  case X86::JMP64m_REX:
  case X86::JMP64r:
  case X86::JMP64r_NT:
  case X86::JMP64r_REX:
  case X86::JMP_1:
  case X86::JMP_2:
  case X86::JMP_4:
  case X86::TAILJMPd:
  case X86::TAILJMPd64:
  case X86::TAILJMPm:
  case X86::TAILJMPm64:
  case X86::TAILJMPm64_REX:
  case X86::TAILJMPr:
  case X86::TAILJMPr64:
  case X86::TAILJMPr64_REX:
    LLVM_DEBUG(dbgs() << "Found unconditional branch instruction: " << MI
                      << "\n");
    ++NumUncondBR;
    break;
  case X86::JCXZ:
  case X86::JECXZ:
  case X86::JRCXZ:
  case X86::TAILJMPd64_CC:
  case X86::TAILJMPd_CC:
  case X86::JCC_1:
  case X86::JCC_2:
  case X86::JCC_4:
    LLVM_DEBUG(dbgs() << "Found conditional branch instruction: " << MI
                      << "\n");
    ++NumCondBR;
    break;
  case X86::CMOV16rm:
  case X86::CMOV16rr:
  case X86::CMOV32rm:
  case X86::CMOV32rr:
  case X86::CMOV64rm:
  case X86::CMOV64rr:
  case X86::CMOVBE_F:
  case X86::CMOVBE_Fp32:
  case X86::CMOVBE_Fp64:
  case X86::CMOVBE_Fp80:
  case X86::CMOVB_F:
  case X86::CMOVB_Fp32:
  case X86::CMOVB_Fp64:
  case X86::CMOVB_Fp80:
  case X86::CMOVE_F:
  case X86::CMOVE_Fp32:
  case X86::CMOVE_Fp64:
  case X86::CMOVE_Fp80:
  case X86::CMOVNBE_F:
  case X86::CMOVNBE_Fp32:
  case X86::CMOVNBE_Fp64:
  case X86::CMOVNBE_Fp80:
  case X86::CMOVNB_F:
  case X86::CMOVNB_Fp32:
  case X86::CMOVNB_Fp64:
  case X86::CMOVNB_Fp80:
  case X86::CMOVNE_F:
  case X86::CMOVNE_Fp32:
  case X86::CMOVNE_Fp64:
  case X86::CMOVNE_Fp80:
  case X86::CMOVNP_F:
  case X86::CMOVNP_Fp32:
  case X86::CMOVNP_Fp64:
  case X86::CMOVNP_Fp80:
  case X86::CMOVP_F:
  case X86::CMOVP_Fp32:
  case X86::CMOVP_Fp64:
  case X86::CMOVP_Fp80:
  case X86::CMOV_FR32:
  case X86::CMOV_FR32X:
  case X86::CMOV_FR64:
  case X86::CMOV_FR64X:
  case X86::CMOV_GR16:
  case X86::CMOV_GR32:
  case X86::CMOV_GR8:
  case X86::CMOV_RFP32:
  case X86::CMOV_RFP64:
  case X86::CMOV_RFP80:
    LLVM_DEBUG(dbgs() << "Found conditional move: " << MI << "\n");
    ++NumCMOV;
    break;
  case X86::CMOV_VK16:
  case X86::CMOV_VK2:
  case X86::CMOV_VK32:
  case X86::CMOV_VK4:
  case X86::CMOV_VK64:
  case X86::CMOV_VK8:
  case X86::CMOV_VR128:
  case X86::CMOV_VR128X:
  case X86::CMOV_VR256:
  case X86::CMOV_VR256X:
  case X86::CMOV_VR512:
    LLVM_DEBUG(dbgs() << "Found vector conditional move: " << MI << "\n");
    ++NumVecCMOV;
    break;
  case X86::VPCMOVYrmr:
  case X86::VPCMOVYrrm:
  case X86::VPCMOVYrrr:
  case X86::VPCMOVYrrr_REV:
  case X86::VPCMOVrmr:
  case X86::VPCMOVrrm:
  case X86::VPCMOVrrr:
  case X86::VPCMOVrrr_REV:
  case X86::BLENDPDrmi:
  case X86::BLENDPDrri:
  case X86::BLENDPSrmi:
  case X86::BLENDPSrri:
  case X86::BLENDVPDrm0:
  case X86::BLENDVPDrr0:
  case X86::BLENDVPSrm0:
  case X86::BLENDVPSrr0:
  case X86::PBLENDVBrm0:
  case X86::PBLENDVBrr0:
  case X86::PBLENDWrmi:
  case X86::PBLENDWrri:
  case X86::VBLENDMPDZ128rm:
  case X86::VBLENDMPDZ128rmb:
  case X86::VBLENDMPDZ128rmbk:
  case X86::VBLENDMPDZ128rmbkz:
  case X86::VBLENDMPDZ128rmk:
  case X86::VBLENDMPDZ128rmkz:
  case X86::VBLENDMPDZ128rr:
  case X86::VBLENDMPDZ128rrk:
  case X86::VBLENDMPDZ128rrkz:
  case X86::VBLENDMPDZ256rm:
  case X86::VBLENDMPDZ256rmb:
  case X86::VBLENDMPDZ256rmbk:
  case X86::VBLENDMPDZ256rmbkz:
  case X86::VBLENDMPDZ256rmk:
  case X86::VBLENDMPDZ256rmkz:
  case X86::VBLENDMPDZ256rr:
  case X86::VBLENDMPDZ256rrk:
  case X86::VBLENDMPDZ256rrkz:
  case X86::VBLENDMPDZrm:
  case X86::VBLENDMPDZrmb:
  case X86::VBLENDMPDZrmbk:
  case X86::VBLENDMPDZrmbkz:
  case X86::VBLENDMPDZrmk:
  case X86::VBLENDMPDZrmkz:
  case X86::VBLENDMPDZrr:
  case X86::VBLENDMPDZrrk:
  case X86::VBLENDMPDZrrkz:
  case X86::VBLENDMPSZ128rm:
  case X86::VBLENDMPSZ128rmb:
  case X86::VBLENDMPSZ128rmbk:
  case X86::VBLENDMPSZ128rmbkz:
  case X86::VBLENDMPSZ128rmk:
  case X86::VBLENDMPSZ128rmkz:
  case X86::VBLENDMPSZ128rr:
  case X86::VBLENDMPSZ128rrk:
  case X86::VBLENDMPSZ128rrkz:
  case X86::VBLENDMPSZ256rm:
  case X86::VBLENDMPSZ256rmb:
  case X86::VBLENDMPSZ256rmbk:
  case X86::VBLENDMPSZ256rmbkz:
  case X86::VBLENDMPSZ256rmk:
  case X86::VBLENDMPSZ256rmkz:
  case X86::VBLENDMPSZ256rr:
  case X86::VBLENDMPSZ256rrk:
  case X86::VBLENDMPSZ256rrkz:
  case X86::VBLENDMPSZrm:
  case X86::VBLENDMPSZrmb:
  case X86::VBLENDMPSZrmbk:
  case X86::VBLENDMPSZrmbkz:
  case X86::VBLENDMPSZrmk:
  case X86::VBLENDMPSZrmkz:
  case X86::VBLENDMPSZrr:
  case X86::VBLENDMPSZrrk:
  case X86::VBLENDMPSZrrkz:
  case X86::VBLENDPDYrmi:
  case X86::VBLENDPDYrri:
  case X86::VBLENDPDrmi:
  case X86::VBLENDPDrri:
  case X86::VBLENDPSYrmi:
  case X86::VBLENDPSYrri:
  case X86::VBLENDPSrmi:
  case X86::VBLENDPSrri:
  case X86::VBLENDVPDYrm:
  case X86::VBLENDVPDYrr:
  case X86::VBLENDVPDrm:
  case X86::VBLENDVPDrr:
  case X86::VBLENDVPSYrm:
  case X86::VBLENDVPSYrr:
  case X86::VBLENDVPSrm:
  case X86::VBLENDVPSrr:
  case X86::VPBLENDDYrmi:
  case X86::VPBLENDDYrri:
  case X86::VPBLENDDrmi:
  case X86::VPBLENDDrri:
  case X86::VPBLENDMBZ128rm:
  case X86::VPBLENDMBZ128rmk:
  case X86::VPBLENDMBZ128rmkz:
  case X86::VPBLENDMBZ128rr:
  case X86::VPBLENDMBZ128rrk:
  case X86::VPBLENDMBZ128rrkz:
  case X86::VPBLENDMBZ256rm:
  case X86::VPBLENDMBZ256rmk:
  case X86::VPBLENDMBZ256rmkz:
  case X86::VPBLENDMBZ256rr:
  case X86::VPBLENDMBZ256rrk:
  case X86::VPBLENDMBZ256rrkz:
  case X86::VPBLENDMBZrm:
  case X86::VPBLENDMBZrmk:
  case X86::VPBLENDMBZrmkz:
  case X86::VPBLENDMBZrr:
  case X86::VPBLENDMBZrrk:
  case X86::VPBLENDMBZrrkz:
  case X86::VPBLENDMDZ128rm:
  case X86::VPBLENDMDZ128rmb:
  case X86::VPBLENDMDZ128rmbk:
  case X86::VPBLENDMDZ128rmbkz:
  case X86::VPBLENDMDZ128rmk:
  case X86::VPBLENDMDZ128rmkz:
  case X86::VPBLENDMDZ128rr:
  case X86::VPBLENDMDZ128rrk:
  case X86::VPBLENDMDZ128rrkz:
  case X86::VPBLENDMDZ256rm:
  case X86::VPBLENDMDZ256rmb:
  case X86::VPBLENDMDZ256rmbk:
  case X86::VPBLENDMDZ256rmbkz:
  case X86::VPBLENDMDZ256rmk:
  case X86::VPBLENDMDZ256rmkz:
  case X86::VPBLENDMDZ256rr:
  case X86::VPBLENDMDZ256rrk:
  case X86::VPBLENDMDZ256rrkz:
  case X86::VPBLENDMDZrm:
  case X86::VPBLENDMDZrmb:
  case X86::VPBLENDMDZrmbk:
  case X86::VPBLENDMDZrmbkz:
  case X86::VPBLENDMDZrmk:
  case X86::VPBLENDMDZrmkz:
  case X86::VPBLENDMDZrr:
  case X86::VPBLENDMDZrrk:
  case X86::VPBLENDMDZrrkz:
  case X86::VPBLENDMQZ128rm:
  case X86::VPBLENDMQZ128rmb:
  case X86::VPBLENDMQZ128rmbk:
  case X86::VPBLENDMQZ128rmbkz:
  case X86::VPBLENDMQZ128rmk:
  case X86::VPBLENDMQZ128rmkz:
  case X86::VPBLENDMQZ128rr:
  case X86::VPBLENDMQZ128rrk:
  case X86::VPBLENDMQZ128rrkz:
  case X86::VPBLENDMQZ256rm:
  case X86::VPBLENDMQZ256rmb:
  case X86::VPBLENDMQZ256rmbk:
  case X86::VPBLENDMQZ256rmbkz:
  case X86::VPBLENDMQZ256rmk:
  case X86::VPBLENDMQZ256rmkz:
  case X86::VPBLENDMQZ256rr:
  case X86::VPBLENDMQZ256rrk:
  case X86::VPBLENDMQZ256rrkz:
  case X86::VPBLENDMQZrm:
  case X86::VPBLENDMQZrmb:
  case X86::VPBLENDMQZrmbk:
  case X86::VPBLENDMQZrmbkz:
  case X86::VPBLENDMQZrmk:
  case X86::VPBLENDMQZrmkz:
  case X86::VPBLENDMQZrr:
  case X86::VPBLENDMQZrrk:
  case X86::VPBLENDMQZrrkz:
  case X86::VPBLENDMWZ128rm:
  case X86::VPBLENDMWZ128rmk:
  case X86::VPBLENDMWZ128rmkz:
  case X86::VPBLENDMWZ128rr:
  case X86::VPBLENDMWZ128rrk:
  case X86::VPBLENDMWZ128rrkz:
  case X86::VPBLENDMWZ256rm:
  case X86::VPBLENDMWZ256rmk:
  case X86::VPBLENDMWZ256rmkz:
  case X86::VPBLENDMWZ256rr:
  case X86::VPBLENDMWZ256rrk:
  case X86::VPBLENDMWZ256rrkz:
  case X86::VPBLENDMWZrm:
  case X86::VPBLENDMWZrmk:
  case X86::VPBLENDMWZrmkz:
  case X86::VPBLENDMWZrr:
  case X86::VPBLENDMWZrrk:
  case X86::VPBLENDMWZrrkz:
  case X86::VPBLENDVBYrm:
  case X86::VPBLENDVBYrr:
  case X86::VPBLENDVBrm:
  case X86::VPBLENDVBrr:
  case X86::VPBLENDWYrmi:
  case X86::VPBLENDWYrri:
  case X86::VPBLENDWrmi:
  case X86::VPBLENDWrri:
    LLVM_DEBUG(dbgs() << "Found vector conditional blend: " << MI << "\n");
    ++NumVecBlend;
    break;
  }
}

FunctionPass *llvm::createX86CountMIAnalysis() { return new X86CountMI(); }
