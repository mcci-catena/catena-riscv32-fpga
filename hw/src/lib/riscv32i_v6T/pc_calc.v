/*

Module: pc_calc.v

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

/////////////////////////////////////////////////////
// Module interface
/////////////////////////////////////////////////////
module pc_calc(
    input   [ 6:0]  opcode_i,       //op-code from EX stage
    input   [31:0]  jtype_i,        //J-Type Immediate from EX stage
    input   [31:0]  btype_i,        //B-Type Immediate from EX stage
    input   [31:0]  itype_i,        //I-Type Immediate from EX stage
    input   [31:0]  rs1_i,          //rs1 Data from EX stage
    input   [31:0]  f_pc_i,         //program counter from IF stage
    input   [31:0]  x_pc_i,         //program counter from EX stage
    input           x_br_i,         //branch valid from EX stage
    output  [31:0]  pc_calc_o       //calculated pc for branching
    );

    /////////////////////////////////////////////////////
    // Wires and Registers
    /////////////////////////////////////////////////////
    reg [31:0] operand;         //operand value to be added (rs1 or pc)
    reg [31:0] immediate;       //immediate value to be added

    // ----- Initialize the Register for Simulation Purpose -----
    initial begin
        operand = 0;
        immediate = 0;
    end
    /////////////////////////////////////////////////////
    // FSM & Logic
    /////////////////////////////////////////////////////
    always @ ( * ) begin
        //Normal Operation
        operand = f_pc_i;
        immediate = 32'h4;
        case(opcode_i)
            `BRANCH: begin          //BRANCH Instruction
                if (x_br_i) begin
                    operand = x_pc_i;
                    immediate = btype_i;
                end
            end
            `JAL: begin             //JAL Instruction
                operand = x_pc_i;
                immediate = jtype_i;
            end
            `JALR:begin             //JALR Instruction
                operand = rs1_i;
                immediate = itype_i;
            end
            default: /* nothing */;
        endcase
    end

    assign pc_calc_o = operand + immediate;

endmodule
