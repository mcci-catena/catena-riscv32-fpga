/*

Module: data_unit.v

Function:
    Data unit for RISC-V CPU

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

`include "./define.v"

module data_unit #(
    parameter INIT_FILE_NAME   = "none"       //Data memory file name
  )(
    //Input
    input clk_i,                              //Input Clock
    input rst_i,                              //Reset Signal
    input [2:0]  funct3_i,                    //RISC-V funct3
    input [14:0] addr_i,                      //Read/Write Address Port
    input [31:0] wdata_i,                     //Write Data
    input wr_en_i,                            //Write Enable

    //Output
    output [31:0] rdata_o                     //Output Data
  );

    /////////////////////////////////////////////////////
    // Local parameters
    /////////////////////////////////////////////////////


    /////////////////////////////////////////////////////
    // Wires
    /////////////////////////////////////////////////////
    wire [7:0] rdata_3, rdata_2, rdata_1, rdata_0;    //Read Data in Byte
    wire [31:0] wdata;                                //Data to be written
    reg [31:0] rdata;                                 //Output Read Data
    reg [3:0] wr_en;                                  //Write enable

    reg [1:0] addr;                                   //Use for shifting to correct byte after READ
    reg [2:0] funct3;                                 //Use for selecting the correct bytes after READ

    // ----- Initialize the Register for Simulation Purpose -----
    initial begin
        rdata = 0;
        wr_en = 0;
    end

    /////////////////////////////////////////////////////
    // Register File Decoding Logic
    // - Work for aligned memory only
    /////////////////////////////////////////////////////

    //Read Logic
    always @ (posedge clk_i or posedge rst_i) begin
        if(rst_i) begin
            addr[1:0] <= 2'b0;
            funct3[2:0] <= 3'b0;
        end else begin
            addr[1:0] <= addr_i[1:0];
            funct3[2:0] <= funct3_i[2:0];
        end
    end

    always @ (*) begin
        // TODO(tmm@mcci.com) change to an `if` with explicit cases so we don't have
        // don't cares and possibly have cleaner synth.
        // TODO(tmm@mcci.com) move this to the CPU so that it only exists once.
        // Can recode as
        // assign rdata_align = addr[1:0] == 1'b00 ? rdata_raw
        //                    : addr[1:0] == 1'b01 ? {8'h00, rdata_raw[31:8]}
        //                    : addr[1:0] == 1'b10 ? {16'h0000, rdata_raw[31:16]}
        //                    :                      {24'h000000, rdata_raw[31:8]}
        //                    ;
        // assign rdata = funct[1:0] == `BYTE ? { funct3[2] ? 24'b0 : {24{rdata[7]}}, rdata_align[7:0] }
        //                              `HALF ? { funct3[2] ? 16'b0 : {16{rdata[15]}}, rdata_align[15:0] }
        //                                    : {rdata_align}
        //                                    ;
        rdata = {rdata_3, rdata_2, rdata_1, rdata_0} >> {addr[1:0], 3'b000};  //Shift to the correct byte
        case (funct3[1:0]) //Use the 2 LSB of funct3 for decoding, MSB for Signed Extension
            `BYTE: rdata = {(funct3[2] ? 24'b0 : {24{rdata[7]}}), rdata[7:0]};
            `HALF: rdata = {(funct3[2] ? 16'b0 : {16{rdata[15]}}), rdata[15:0]};
            default: /* nothing */;  // don't care; illegal encoding.
        endcase
    end
    assign rdata_o = rdata;

    //Write Logic
    // TODO(tmm@mcci.com) this also belongs in the CPU, not in the DRAM.
    always @ (*) begin
        case (funct3_i[1:0]) //Use the lower 2 bit of funct3 for decoding
            `BYTE: wr_en = {3'b000,   wr_en_i};
            `HALF: wr_en = {2'b00,    {2{wr_en_i}}};
            `WORD: wr_en = {{4{wr_en_i}}};
            default: wr_en = 4'b0000;
        endcase
        wr_en = wr_en << addr_i[1:0];
    end
    assign wdata = wdata_i << {addr_i[1:0], 3'b000};

    /////////////////////////////////////////////////////
    // ICE Family Dual-Port EBR
    /////////////////////////////////////////////////////

`ifndef VERILATOR
`ifdef XILINX
    defparam inst_mem0.INIT_FILE_NAME = {"byte0_", INIT_FILE_NAME};
`else // XILINX
    defparam inst_mem0.spram256k_core_inst.uut.INIT_FILE_NAME = {"byte0_", INIT_FILE_NAME};
`endif // XILINX
`endif

    SB_SPRAM256KA inst_mem0 (
        // ----- Inputs -----
        .ADDRESS({1'b0, addr_i[14:2]}),                       // 14-bit address
        .DATAIN(wdata[15:0]),                                 //Cannot write (for now)
        .MASKWREN({wr_en[1], wr_en[1], wr_en[0], wr_en[0]}),  //4-bit Mask Address
        .WREN(|wr_en),                                        //Write/Read enable, 0 => Read
        .CHIPSELECT(1'b1),                                    //Read Clock
        .CLOCK(clk_i),                                        //You cannot write (for now)
        .STANDBY(1'b0),                                       //Always Read
        .SLEEP(1'b0),                                         //Cannot write (for now)
        .POWEROFF(1'b1),                                      // 0 => power-off
        // ----- Outputs -----
        .DATAOUT({rdata_1, rdata_0})
    );

`ifndef VERILATOR
`ifdef XILINX
    defparam inst_mem1.INIT_FILE_NAME = {"byte1_", INIT_FILE_NAME};
`else // XILINX
    defparam inst_mem1.spram256k_core_inst.uut.INIT_FILE_NAME = {"byte1_", INIT_FILE_NAME};
`endif // XILINX
`endif

    SB_SPRAM256KA inst_mem1 (
        // ----- Inputs -----
        .ADDRESS({1'b0, addr_i[14:2]}),                       // 14-bit address
        .DATAIN(wdata[31:16]),                                //Cannot write (for now)
        .MASKWREN({wr_en[3], wr_en[3], wr_en[2], wr_en[2]}),  //4-bit Mask Address
        .WREN(|wr_en),                                        //Write/Read enable, 0 => Read
        .CHIPSELECT(1'b1),                                    //Read Clock
        .CLOCK(clk_i),                                        //You cannot write (for now)
        .STANDBY(1'b0),                                       //Always Read
        .SLEEP(1'b0),                                         //Cannot write (for now)
        .POWEROFF(1'b1),                                      // 0 => power-off
        // ----- Outputs -----
        .DATAOUT({rdata_3, rdata_2})
    );

endmodule
