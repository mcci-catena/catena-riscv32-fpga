/*

Module: define.v

Function:
    Common definitions for RISC-V RV32I CPU

Copyright Notice and License Information:
    Copyright 2019 MCCI Corporation

    This file is part of the MCCI Catena RISC-V FPGA core,
    https://github.com/mcci-catena/catena-riscv32-fpga.

    catena-risc32v-fpga is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation, either version 3 of
    the License, or (at your option) any later version.

    catena-risc32v-fpga is distributed in the hope that it will
    be useful, but WITHOUT ANY WARRANTY; without even the implied
    warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

`timescale 1 ns / 1 ps
`define LATTICE_FAMILY "ECP5UM"

`define opcode   [6 : 0]
`define rd       [11: 7]
`define funct3   [14:12]
`define rs1      [19:15]
`define rs2      [24:20]
`define funct7   [31:25]
// opcode+rd fields
`define opcoderd [11: 0]

`define R_TYPE  7'b0110011
`define I_TYPE  7'b0010011
`define LOAD    7'b0000011
`define STORE   7'b0100011
`define BRANCH  7'b1100011
`define LUI     7'b0110111
`define AUIPC   7'b0010111
`define JAL     7'b1101111
`define JALR    7'b1100111

`define nop     32'h00000033

// funct3[1:0] for the Data Memory operation
`define BYTE    2'b00   //LB, SB, LBU
`define HALF    2'b01   //LH, SH, LHU
`define WORD    2'b10   //LW, SW

`define BYTE0   2'b00   //BYTE 0 Read Address
`define BYTE1   2'b01   //BYTE 1 Read Address
`define BYTE2   2'b10   //BYTE 2 Read Address
`define BYTE3   2'b11   //BYTE 3 Read Address

`define HALF0   1'b0    //HALFWORD 0 Read Address
`define HALF1   1'b1    //HALFWORD 1 Read Address

// funct3 for the ALU operation
`define ADD     3'b000  //ADDI, ADD, SUB, MUL
`define SLL     3'b001  //SLLI, SLL, MULH
`define SLT     3'b010  //SLTI, SLT, MULHSU
`define SLTU    3'b011  //SLTIU, SLTU, MULHU
`define XOR     3'b100  //XORI, XOR, DIV
`define SRLA    3'b101  //SRLI, SRAI, SRL, SRA, DIVU
`define OR      3'b110  //ORI, OR, REM
`define AND     3'b111  //ANDI, AND, REMU

// funct7 for multiply
`define ADD_7   7'b0000000  //for adds and non-arithmetic shifts
`define MUL_7   7'b0000001  //for multiplies
`define SUB_7   7'b0100000  //for sub and arithmetic shifts