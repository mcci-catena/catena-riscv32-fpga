/*

Module: simple_uart.v

Function:
    UART transmitter

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

// TX only UART

module simple_uart #(parameter ADDRESS_WIDTH=3) (
    input             clk,    // data interface clock
    input             reset,
    input      [10:0] i_period,
    input      [ 7:0] i_din,
    input             i_valid,
    output            fifo_empty,
    output            fifo_full,
    output reg        o_txd
);
    wire        fifo_rd;
    wire [ 7:0] fifo_dout;
    wire        fifo_empty;
    reg  [10:0] period_cnt;
    reg  [ 3:0] bit_cnt;    // 0: IDLE, 1: Start, 2~9: bit0~7, A:Stop
    reg         bit_tick;

    always @(posedge clk or posedge reset)
    if(reset)
        bit_cnt <= 4'b0;
    else if((bit_cnt == 4'b0) && (fifo_empty == 1'b0))
        bit_cnt <= 4'd1;
    else if((bit_cnt == 4'hA) && bit_tick)
        bit_cnt <= 4'd0;
    else if((bit_cnt != 4'b0) && bit_tick)
        bit_cnt <= bit_cnt + 4'd1;

    always @(posedge clk or posedge reset)
    if(reset)
        period_cnt <= 11'b0;
    else if(bit_cnt == 4'b0)
        period_cnt <= 11'b0;
    else if(period_cnt == 11'b0)
        period_cnt <= i_period;
    else
        period_cnt <= period_cnt - 11'd1;

    always @(posedge clk or posedge reset)
    if(reset)
        bit_tick <= 1'b0;
    else if(period_cnt == 11'd1)
        bit_tick <= 1'b1;
    else
        bit_tick <= 1'b0;

    assign fifo_rd = ((bit_cnt == 4'hA) && (period_cnt == 11'd1));

    always @(posedge clk or posedge reset)
    if(reset)
        o_txd <= 1'b0;
    else case(bit_cnt)
        4'd1   : o_txd <= 1'b0; // start
        4'd2   : o_txd <= fifo_dout[0];
        4'd3   : o_txd <= fifo_dout[1];
        4'd4   : o_txd <= fifo_dout[2];
        4'd5   : o_txd <= fifo_dout[3];
        4'd6   : o_txd <= fifo_dout[4];
        4'd7   : o_txd <= fifo_dout[5];
        4'd8   : o_txd <= fifo_dout[6];
        4'd9   : o_txd <= fifo_dout[7];
        default: o_txd <= 1'b1; // stop & idle
    endcase

    // output fifo (one clock fifo)
    parameter FIFO_DEPTH = (1 << ADDRESS_WIDTH);

    reg [ADDRESS_WIDTH-1 : 0] wr_ptr, rd_ptr;
    reg [ADDRESS_WIDTH   : 0] fifo_lvl;

`ifdef SYNTHESIS
    wire [8:0] wr_ptr9 = { {(9 - ADDRESS_WIDTH){1'b0}}, wr_ptr };
    wire [8:0] rd_ptr9 = { {(9 - ADDRESS_WIDTH){1'b0}}, rd_ptr };
    wire valid_not_full = i_valid && !fifo_full;

    SB_RAM512x8 fifo_mem_inst(
       .RDATA   (fifo_dout),
       .RADDR   (rd_ptr9),
       .RCLK    (clk),
       .RCLKE   (1'b1),
       .RE      (1'b1),

       .WADDR   (wr_ptr9),
       .WCLK    (clk),
       .WCLKE   (valid_not_full),
       .WDATA   (i_din),
       .WE      (1'b1)
   );
`else
    reg [7 : 0] fifo_mem [0 : FIFO_DEPTH-1];

    always @(posedge clk) begin
        if (i_valid && !fifo_full)
            fifo_mem[wr_ptr] <= i_din;
    end
    assign fifo_dout = fifo_mem[rd_ptr];
`endif

    always @(posedge clk or posedge reset)
        if(reset) begin
            wr_ptr <= {ADDRESS_WIDTH{1'b0}};
        end else if(i_valid && !fifo_full) begin
            wr_ptr <= wr_ptr + 1; // wrap around
        end

    // synthesis translate_off
    // translate_off
    /*
    initial begin
    repeat(    4) @(posedge clk);
    while (reset) @(posedge clk);
    forever begin
        @(posedge clk);
        if(i_valid) $write("%s", i_din);
    end
    end
    */
    // translate_on
    // synthesis translate_on


    always @(posedge clk or posedge reset)
        if(reset)
        rd_ptr <= {ADDRESS_WIDTH{1'b0}};
    else if(fifo_rd && !fifo_empty)
        rd_ptr <= rd_ptr + 1;

    always @(posedge clk or posedge reset)
        if(reset)
        fifo_lvl <= {ADDRESS_WIDTH+1{1'b0}};
    else if(fifo_rd && !i_valid && !fifo_empty)
        fifo_lvl <= fifo_lvl - 1;
    else if(!fifo_rd && i_valid && !fifo_full)
        fifo_lvl <= fifo_lvl + 1;

    assign fifo_full  = fifo_lvl == FIFO_DEPTH;
    assign fifo_empty = fifo_lvl == 0;

endmodule
