/*

Module: pmi_ice_rom.v

Function:
    Program Memory Implementaion using ICE ROM for RISC-V CPU

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

//
// Parameter Definition
// Name                              Value                             Default
/*
------------------------------------------------------------------------------
pmi_addr_depth          <integer>                                         32
pmi_addr_width          <integer>                                          5
pmi_data_width          <integer>                                          8
pmi_regmode             "reg"|"noreg"                                    "reg"
pmi_init_file           <string>                                        "none"
pmi_init_file_format    "binary"|"hex"                                "binary"
------------------------------------------------------------------------------
*/

`timescale  1 ns / 1 ps

module pmi_ice_rom
  #(parameter pmi_addr_depth = 512,
    parameter pmi_addr_width = 9,
    parameter pmi_data_width = 8,
    parameter pmi_regmode = "noreg",
    parameter pmi_init_file = "none",
    parameter pmi_init_file_format = "hex",
    parameter pmi_family = "ICE40",
    parameter pmi_config_load = 1
    )

    (
    input [(pmi_addr_width-1):0] Address,
    input Clock,
    input ClockEn,
    input we,
    input  [(pmi_data_width-1):0] Din,
    output [(pmi_data_width-1):0] Q
    );

    //reg  [pmi_data_width-1:0]  mem [0:(2**pmi_addr_width)-1]/*--- synthesis syn_romstyle = "block_rom" */;
    reg  [pmi_data_width-1:0]  mem [0:pmi_addr_depth-1]/* synthesis syn_romstyle = "block_rom" */;
    reg  [pmi_data_width-1:0]  Data_r;
    reg  [pmi_data_width-1:0]  Data_r2;
    integer                    h;
    integer                    i;

    generate
        if(pmi_config_load) begin: load_from_config
            initial begin
            $write("Loading %s\n", pmi_init_file);
            $readmemh(pmi_init_file, mem);
            end
        end
    endgenerate

    //--- Port Read
    always @ (posedge Clock) begin
        if (ClockEn == 1'b1) begin
            if(!we) Data_r <= mem[Address];
        end
    end

    //--- Port write
    always @ (posedge Clock) begin
    if (ClockEn == 1'b1)
        if(we) begin
            mem[Address] <= Din;
            /*
            #0;
            $write("%t : DROM Addr=%h, Content=%h\n", $time, Address, mem[Address]);
            */
        end
    end

    always @ (posedge Clock) begin
        if (ClockEn == 1'b1) begin
            Data_r2<= Data_r;
        end
    end
    assign Q = (pmi_regmode=="reg") ? Data_r2 : Data_r;

endmodule
