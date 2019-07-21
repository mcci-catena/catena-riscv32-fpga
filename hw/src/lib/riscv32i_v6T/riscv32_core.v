/*

Module: riscv_core.v

Function:
    Top-level module for RISC-V CPU core

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

//================================================================================
// Config parameters (0 - no support, 1 - support)
//================================================================================
// INC_ECC_ACC: Decides the support of ECC HW accelerator
//              If 0, SW code needs to support all functions in SW
// INC_TRNG   : Decides the support of HW based TRNG
// INC_AES_ACC: Decides the support of AES HW accelerator
//================================================================================

`include "./define.v"

`include "./cpu.v"          //RISC-V 32I Core
`include "./reg_file.v"     //register file
`include "./pc_calc.v"      //PC Calculation
`include "./alu.v"          //ALU
`include "./mult16.v"       //ALU MUL


//================================================================================
//================================================================================
//================================================================================
module riscv32_core #(
    parameter PC_RESET_ADDR  = 32'h00000000, // PC reset address
              HAS_MULT       = 1,            // 1 - Include MULT in RISCV
              UNSIGNED_MULT  = 1             // 1 - MULT supports only unsigned operation
)(
    input           clk_i,            // RISC-V & HW ACCs clock
    input           rst_i,            // Active high reset
    input  [31 : 0] inst_rdata_i,     // the instruction data bus
    input  [31 : 0] data_rdata_i,     // the data in bus
    input           data_stall_i,     // the data stall (not-ready) bit

    output [31 : 0] inst_raddr_o,     // the instruction read-address
    output [31 : 0] data_addr_o,      // the data adress
    output [ 2 : 0] data_funct3_o,    // the data function
    output [31 : 0] data_wdata_o,     // the write data bus
    output          data_wr_en_o      // the write-enable bit
);
    //================================================================================
    // Internal signals
    //================================================================================


    //================================================================================
    // CPU
    // - Reset CPU when IROM is loaded from outside (SCE)
    // - Prevent CPU reset in HCE
    //================================================================================

    cpu #(.PC_RESET_ADDR(PC_RESET_ADDR), .HAS_MULT(HAS_MULT), .UNSIGNED_MULT(UNSIGNED_MULT))
    cpu_inst(
        .clk_i        (clk_i            ), // Input Clock
        .rst_i        (rst_i            ), // Input Reset
        .inst_rdata_i (inst_rdata_i     ), // Input data from Instruction unit
        .data_rdata_i (data_rdata_i     ), // Input data from memory unit
        .ext_stall_i  (data_stall_i     ), // Stall Signal from Accelerator
        .inst_raddr_o (inst_raddr_o     ), // Read Address to Instruction Memory Unit
        .funct3_o     (data_funct3_o    ), // RISC-V funct3 to Data Memory Unit
        .data_addr_o  (data_addr_o      ), // Read Address to Data Memory Unit
        .data_wdata_o (data_wdata_o     ), // Write Data to Data Memory Unit
        .data_wr_en_o (data_wr_en_o     )  // Write Enable to Data Memory Unit
    );

endmodule
