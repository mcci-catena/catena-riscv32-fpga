/*

Module: mult16.v

Function:
    Multiplier for RISC-V CPU

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

`timescale 1ns/1ps

module mult16(
    input   [15:0]  data_a_i,
    input   [15:0]  data_b_i,
    output  [31:0]  q
    );

    parameter AB_SIGNED = 1'b0;

`ifdef USE_BEH_MULT
    assign q = data_a_i * data_b_i;
`else
    SB_MAC16
        #(
            .A_SIGNED(AB_SIGNED),
            .B_SIGNED(AB_SIGNED),
            .BOTOUTPUT_SELECT(2'b11),
            .TOPOUTPUT_SELECT(2'b11)
        )
        inst (
            .A(data_a_i),
            .B(data_b_i),
            .C(16'd0),
            .D(16'd0),
            .O(q),
            .CLK(1'b0),
            .CE(1'b0),
            .IRSTTOP(1'b0),
            .IRSTBOT(1'b0),
            .ORSTTOP(1'b0),
            .ORSTBOT(1'b0),
            .AHOLD(1'b0),
            .BHOLD(1'b0),
            .CHOLD(1'b0),
            .DHOLD(1'b0),
            .OHOLDTOP(1'b0),
            .OHOLDBOT(1'b0),
            .OLOADTOP(1'b0),
            .OLOADBOT(1'b0),
            .ADDSUBTOP(1'b0),
            .ADDSUBBOT(1'b0),
            /* verilator lint_off PINCONNECTEMPTY */
            .CO(),
            /* verilator lint_on PINCONNECTEMPTY */
            .CI(1'b0),
            .ACCUMCI(1'b0),
            /* verilator lint_off PINCONNECTEMPTY */
            .ACCUMCO(),
            /* verilator lint_on PINCONNECTEMPTY */
            .SIGNEXTIN(1'b0),
            /* verilator lint_off PINCONNECTEMPTY */
            .SIGNEXTOUT()
            /* verilator lint_on PINCONNECTEMPTY */
            );
`endif

endmodule
