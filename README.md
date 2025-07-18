# risc8
This is a risc-V inspired 8-bit CPU assembler and instruction set.

## Features 
- Assembler for 16-bit instructions
- Arithmetic, jump, stack and subroutine support
- Instruction encoding pdf

## How-to
- Save assembler instructions in "program.txt"
- Assembler instructions do not have commas
- Run "assembler.py" in the same directory
- Binary instructions are outputted to "program.mem"

## Overview
- Instructions are 16-bit fixed format
- Optional immediate operands for ALU & memory instructions
- Flag writing instruction "CMP" for conditional jumps that doesn't write registers
