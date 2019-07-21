/*

Module: reg_file.v

Function:
    Register file module for RISC-V32I CPU

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
`define debug_sim

module reg_file
    (
    //Input
    input clk_i,                                //Input Clock
    input rst_i,                                //Reset Signal

    input [4:0] raddr_a_i,                      //Read Address A
    input [4:0] raddr_b_i,                      //Read Address B

    input [4:0] waddr_i,                        //Write Address
    input [31:0] wdata_i,                       //Write Data
    input wr_en_i,                              //Write Enable

    //Output
`define USE_POSEDGE_ONLY
`ifdef USE_POSEDGE_ONLY
    output reg [31:0] rdata_a_o,                //Output Data A
    output reg [31:0] rdata_b_o                 //Output Data B
`else
    output [31:0] rdata_a_o,                    //Output Data A
    output [31:0] rdata_b_o                     //Output Data B
`endif
    );

/////////////////////////////////////////////////////
// Local parameters
/////////////////////////////////////////////////////

    //Assumption : RISC-V 32-bit Instruction Set
    localparam ADDR_DEPTH = 32;                     //Register File Depth (# of 32-bit data)
    localparam ADDR_WIDTH = 5;                      //Register File Address Width ( log( # of Data ) )
    localparam DATA_WIDTH = 32;                     //Register Size
    localparam INIT_FILE_FORMAT = "hex";            //Data memory format
    localparam INIT_FILE_NAME   = "none";           //Data memory file name


/////////////////////////////////////////////////////
// BLOCK RAM
/////////////////////////////////////////////////////
    reg [DATA_WIDTH-1:0] mem [ADDR_DEPTH-1:0]   /* synthesis syn_ramstyle="block_ram" */;
`ifdef USE_POSEDGE_ONLY
`else
    reg [DATA_WIDTH-1:0] rdata_a, rdata_b;
`endif


/*
`ifdef debug_sim
reg [1:0] cnt;
reg one_time;
reg [31:0] reg1;

        always @ (negedge clk_i or posedge rst_i) begin
                if(rst_i)  begin
                   cnt  = 3'h0;
                   one_time  = 1'b1;
                   reg1  = 32'd0;
                end else if(w_reg_wr_en && w_instr`rd == 1 && one_time == 1'b1) begin
                   cnt = cnt + 1;
                   if (cnt == 3)
                   begin
                     reg1  = w_reg_wdata;
                     one_time = 1'b0;
                   end
                end
        end

    always @ (negedge clk_i) begin
                if(w_reg_wr_en && w_instr`rd == 2)
                begin
               if(w_reg_wdata < reg1 )
                   begin
            $display ("    OVERFLOW_ERROR  %h, %h", reg1, w_reg_wdata);
           end
        end
    end
`endif
*/
    integer i;
`ifdef USE_POSEDGE_ONLY
    always @ (posedge clk_i or posedge rst_i)
`else
    always @ (negedge clk_i or posedge rst_i)
`endif
    begin
        if(rst_i)  begin
            for (i=0; i<32; i=i+1) mem[i] <= 32'b0;
        end else if(wr_en_i) begin
            mem[waddr_i] <= wdata_i;
        end
    end


`ifdef USE_POSEDGE_ONLY
    always @ (posedge clk_i or posedge rst_i) begin
        if(rst_i) begin
            rdata_a_o <= 32'h0;
            rdata_b_o <= 32'h0;
        end else begin
            rdata_a_o <= (wr_en_i && (waddr_i == raddr_a_i)) ? wdata_i : mem[raddr_a_i];
            rdata_b_o <= (wr_en_i && (waddr_i == raddr_b_i)) ? wdata_i : mem[raddr_b_i];
        end
    end
`else // USE_POSEDGE_ONLY
    always @ (posedge clk_i or posedge rst_i) begin
        if(rst_i) begin
            rdata_a <= 32'h0;
            rdata_b <= 32'h0;
        end else begin
            rdata_a <= mem[raddr_a_i];
            rdata_b <= mem[raddr_b_i];
        end
    end
    assign rdata_a_o = rdata_a;
    assign rdata_b_o = rdata_b;
`endif

`ifdef SYNTHESIS
    // synthesis translate_off
    // translate_off
    wire [31 : 0] mem00 = mem[00];
    wire [31 : 0] mem01 = mem[01];
    wire [31 : 0] mem02 = mem[02];
    wire [31 : 0] mem03 = mem[03];
    wire [31 : 0] mem04 = mem[04];
    wire [31 : 0] mem05 = mem[05];
    wire [31 : 0] mem06 = mem[06];
    wire [31 : 0] mem07 = mem[07];
    wire [31 : 0] mem08 = mem[08];
    wire [31 : 0] mem09 = mem[09];

    wire [31 : 0] mem10 = mem[10];
    wire [31 : 0] mem11 = mem[11];
    wire [31 : 0] mem12 = mem[12];
    wire [31 : 0] mem13 = mem[13];
    wire [31 : 0] mem14 = mem[14];
    wire [31 : 0] mem15 = mem[15];
    wire [31 : 0] mem16 = mem[16];
    wire [31 : 0] mem17 = mem[17];
    wire [31 : 0] mem18 = mem[18];
    wire [31 : 0] mem19 = mem[19];

    wire [31 : 0] mem20 = mem[20];
    wire [31 : 0] mem21 = mem[21];
    wire [31 : 0] mem22 = mem[22];
    wire [31 : 0] mem23 = mem[23];
    wire [31 : 0] mem24 = mem[24];
    wire [31 : 0] mem25 = mem[25];
    wire [31 : 0] mem26 = mem[26];
    wire [31 : 0] mem27 = mem[27];
    wire [31 : 0] mem28 = mem[28];
    wire [31 : 0] mem29 = mem[29];

    wire [31 : 0] mem30 = mem[30];
    wire [31 : 0] mem31 = mem[31];
    // translate_on
    // synthesis translate_on
`endif // SYNTHESIS
endmodule
