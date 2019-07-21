/*

Module: inst_unit.v

Function:
    Instruction unit for RISC-V CPU

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
module inst_unit
    #(
        parameter INIT_FILE_NAME   = "none"           //Instruction memory file name
    )(
        input         clk_i,                          //Input Clock

        input         prom_extacc,
//      input         br_ready_i,
//      output        br_valid_o,
//      output [31:0] br_data_o,
//      output        bw_ready_o,
        input         bw_valid_i,
        input  [31:0] bw_data_i,

        input  [15:0] raddr_i,                        //Read Address
        output reg [31:0] probe,
        output [31:0] rdata_o                         //Output Data
    );

    /////////////////////////////////////////////////////
    // Local parameters
    /////////////////////////////////////////////////////

    /////////////////////////////////////////////////////
    // ICE Family Dual-Port EBR
    /////////////////////////////////////////////////////
    wire [15:0] rdata_0;
    wire [15:0] rdata_1;

    reg ext_access;
    wire ext_access_r;
    reg [13:0] ext_addr;
    // Do not use rst_i for external memory access
    always @(posedge clk_i)
        ext_access <= prom_extacc;
    assign ext_access_r = prom_extacc & ~ext_access; // rising edge of prom_extacc

    // Do not use rst_i for external memory access
    always @(posedge clk_i)
        if(ext_access_r)
            ext_addr <= 14'h0;
        else if(bw_valid_i & prom_extacc)
            ext_addr <= ext_addr + 1'b1;

`ifndef VERILATOR
`ifdef XILINX
    defparam inst_mem0.INIT_FILE_NAME = {"byte0_", INIT_FILE_NAME};
`else // LATTICE
    defparam inst_mem0.spram256k_core_inst.uut.INIT_FILE_NAME = {"byte0_", INIT_FILE_NAME};
`endif
`endif

    always @(posedge clk_i)
        if(prom_extacc && ext_addr == 0 && bw_valid_i) probe <= bw_data_i;

    SB_SPRAM256KA inst_mem0
        (
        // ----- Inputs -----
        .ADDRESS(prom_extacc?ext_addr:raddr_i[15:2]),   // 13-bit address
        .DATAIN(bw_data_i[15:0]),                       //Cannot write (for now)
        .MASKWREN({4{bw_valid_i&prom_extacc}}),         //4-bit Mask Address
        .WREN(bw_valid_i&prom_extacc),                  //Write/Read enable, 0 => Read
        .CHIPSELECT(1'b1),                              //Read Clock
        .CLOCK(clk_i),                                  //You cannot write (for now)
        .STANDBY(1'b0),                                 //Always Read
        .SLEEP(1'b0),                                   //Cannot write (for now)
        .POWEROFF(1'b1),                                // 0 => power-off
        // ----- Outputs -----
        .DATAOUT(rdata_0)
        );

`ifndef VERILATOR
`ifdef XILINX
    defparam inst_mem1.INIT_FILE_NAME = {"byte1_", INIT_FILE_NAME};
`else // LATTICE
    defparam inst_mem1.spram256k_core_inst.uut.INIT_FILE_NAME = {"byte1_", INIT_FILE_NAME};
`endif // LATTICE
`endif

    SB_SPRAM256KA inst_mem1
        (
        // ----- Inputs -----
        .ADDRESS(prom_extacc?ext_addr:raddr_i[15:2]),   // 14-bit address
        .DATAIN(bw_data_i[31:16]),                      //Cannot write (for now)
        .MASKWREN({4{bw_valid_i&prom_extacc}}),         //4-bit Mask Address
        .WREN(bw_valid_i&prom_extacc),                  //Write/Read enable, 0 => Read
        .CHIPSELECT(1'b1),                              //Read Clock
        .CLOCK(clk_i),                                  //You cannot write (for now)
        .STANDBY(1'b0),                                 //Always Read
        .SLEEP(1'b0),                                   //Cannot write (for now)
        .POWEROFF(1'b1),                                // 0 => power-off
        // ----- Outputs -----
        .DATAOUT(rdata_1)
        );

    assign rdata_o = {rdata_1, rdata_0};

endmodule
