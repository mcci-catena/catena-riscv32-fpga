/*

Module: drom_unit.v

Function:
    DROM unit for RISC-V CPU

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
module drom_unit
    #(
    parameter INIT_FILE_NAME   = "none",        //Instruction memory file name
        ADDR_DEPTH       = 256*3,
        ADDR_WIDTH       = 10,
        CONFIG_LOAD      = 0
    )(
        input clk_i,                            //Input Clock
        input  [ADDR_WIDTH+1:0] raddr_i,        //Read Address
        input  [2:0] funct3_i,                  //Read Address
        input  ext_acc,
        input  we,
        input  [31:0] din,
        output reg [31:0] probe,
        output [31:0] rdata_o                   //Output Data
    );

    /////////////////////////////////////////////////////
    // ICE Family Dual-Port EBR
    /////////////////////////////////////////////////////
    wire [15:0] rdata_0;
    wire [15:0] rdata_1;
    reg  [31:0] rdata;

//`ifdef XILINX
//defparam inst_mem0.INIT_FILE_NAME = {"byte0_", INIT_FILE_NAME};
//`else // XILINX
    defparam inst_mem0.pmi_init_file = {"./byte0_", INIT_FILE_NAME};
//`endif // XILINX

    reg ext_acc_d;
    wire ext_acc_r;

    // Do not use rst_i for external memory access
    always @(posedge clk_i)
        ext_acc_d <= ext_acc;
    assign ext_acc_r = ext_acc & ~ext_acc_d; // rising edge of prom_extacc

    reg [ADDR_WIDTH-1:0] ext_addr;
    // Do not use rst_i for external memory access
    always @(posedge clk_i)
        if(ext_acc_r)
            ext_addr <= 0;
        else if(we & ext_acc)
            ext_addr <= ext_addr + 1'b1;

    wire [ADDR_WIDTH-1:0] addr_f;
    assign addr_f = ext_acc ? ext_addr : raddr_i[ADDR_WIDTH+1:2];

    always @(posedge clk_i)
        if(ext_acc && addr_f == 0 && we) probe <= din;

    pmi_ice_rom #(.pmi_addr_depth(ADDR_DEPTH), .pmi_addr_width(ADDR_WIDTH), .pmi_data_width(16), .pmi_config_load(CONFIG_LOAD))
    inst_mem0
        (
        // ----- Inputs -----
        .Address(addr_f),               // 14-bit address
        .Clock(clk_i),                  //You cannot write (for now)
        .ClockEn(1'b1),                 //You cannot write (for now)
        .we(we),
        .Din(din[15:0]),
        // ----- Outputs -----
        .Q(rdata_0)
        );

//`ifdef XILINX
//defparam inst_mem1.INIT_FILE_NAME = {"byte1_", INIT_FILE_NAME};
//`else // XILINX
    defparam inst_mem1.pmi_init_file = {"./byte1_", INIT_FILE_NAME};
//`endif // XILINX

    pmi_ice_rom #(.pmi_addr_depth(ADDR_DEPTH), .pmi_addr_width(ADDR_WIDTH), .pmi_data_width(16), .pmi_config_load(CONFIG_LOAD))
    inst_mem1
        (
        // ----- Inputs -----
        .Address(addr_f),               // 14-bit address
        .Clock(clk_i),                  //You cannot write (for now)
        .ClockEn(1'b1),                 //You cannot write (for now)
        .we(we),
        .Din(din[31:16]),
        // ----- Outputs -----
        .Q(rdata_1)
        );

    reg [1:0] addr;                 //Use for shifting to correct byte after READ
    reg [2:0] funct3;               //Use for selecting the correct bytes after READ

    always @ (posedge clk_i) begin
        addr[1:0] <= raddr_i[1:0];
        funct3[2:0] <= funct3_i[2:0];
    end

    always @(*) begin
        // TODO(tmm@mcci.com) change to an `if` with explicit cases so we don't have
        // don't cares and possibly have cleaner synth.
        rdata = {rdata_1, rdata_0} >> {addr[1:0], 3'b000};  //Shift to the correct byte
        case (funct3[1:0]) //Use the 2 LSB of funct3 for decoding, MSB for Signed Extension
            `BYTE: rdata = {(funct3[2] ? 24'b0 : {24{rdata[7]}}), rdata[7:0]};
            `HALF: rdata = {(funct3[2] ? 16'b0 : {16{rdata[15]}}), rdata[15:0]};
            default: /* don't care */;
        endcase
    end
    assign rdata_o = rdata;

endmodule
