/*

Module: riscv_include.v

Function:
    Program Counter Calculation module for RISC-V CPU

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

`include "./define.v"       //Marco Definition
`include "./cpu.v"          //RISC-V 32I Core
`include "./inst_unit.v"    //instruction memory
`include "./data_unit.v"    //data memory
`include "./drom_unit.v"    //data memory
`include "./reg_file.v"     //register file
`include "./pc_calc.v"      //PC Calculation
`include "./alu.v"          //ALU
`include "./mult16.v"       //ALU MUL

`include "./pmi_ice_rom.v"

// synthesis translate_off
// synopsys  translate_off
// translate_off

`include "./display_unit.v"     //Display (printf, timer) Block

// translate_on
// synopsys  translate_on
// synthesis translate_on
