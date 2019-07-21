/*

Module: simple_uart_rx.v

Function:
    Simple receive-half of a UART.

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

// RX only UART

// bugs:
//
// - There's no overrun detect
// - Ideally, act of reading would accept the char.
// - At 230.4K, with no interrupts or DMA, we really
//   ought to have a fifo.
// - 8 bits only.
// - The 'measurment' scheme is undocumented.
// - The 24 MHz-derived clocks are not very accurate, so
//   the timing of the UART may need to be calibrated.
//

module simple_uart_rx (
    input             clk,    // data interface clock
    input             reset,
    input      [10:0] i_period,
    input             i_rxd,
    input             cfg_wr,
    input             cfg_d,        // stored into measure when cfg_wr == 1.
    input             cfg_accept,   // if set when cfg_wr, clear o_valid.
    output reg [ 7:0] o_din,
    output reg        o_valid
);
    reg  [10:0] period_cnt;
    reg  [10:0] act_period; // up to 512
    reg  [ 3:0] bit_cnt;    // 0: IDLE, 1: Start, 2~9: bit0~7, A:Stop
    reg         bit_tick;
    reg         measure;
    reg  [ 3:0] flt;
    reg         flt_rxd;
    reg         flt_rxd_d;
    reg  [ 7:0] r_din;

    wire        start_cond;

    // measure period control register
    always @(posedge clk or posedge reset)
    if     (reset ) measure <= 1'b0;
    else if(cfg_wr) measure <= cfg_d;

    // to detect changes in UART RX
    always @(posedge clk)
        flt <= {flt[2:0], i_rxd};
    always @(posedge clk)
        flt_rxd <= &flt ? 1'b1 : ~|flt ? 1'b0 : flt_rxd;

    always @(posedge clk)
        flt_rxd_d <= flt_rxd;

    assign start_cond = ~|bit_cnt && flt_rxd_d && !flt_rxd; // START (falling edge in IDLE)

    always @(posedge clk or posedge reset)
    if     (reset                        ) bit_cnt <= 4'b0;
    else if(start_cond && !measure       ) bit_cnt <= 4'd1;
    else if((bit_cnt == 4'hA) && bit_tick) bit_cnt <= 4'd0; // idle
    else if((bit_cnt != 4'b0) && bit_tick) bit_cnt <= bit_cnt + 4'd1;

    always @(posedge clk or posedge reset)
    if     (reset                       ) act_period <= 11'b0;
    else if(measure && ~flt_rxd_d && flt_rxd) act_period <= period_cnt; // sample at rising edge

    always @(posedge clk or posedge reset)
    if     (reset                                                ) period_cnt <= 11'b0;
    else if(flt_rxd_d ^ flt_rxd                                  ) period_cnt <= 11'b0;// align period cnt to the edge of RXD
//      else if(!measure && (period_cnt == act_period || ~|bit_cnt)) period_cnt <= 11'b0;
        else if(!measure && (period_cnt == i_period || ~|bit_cnt))     period_cnt <= 11'b0;
    else                                                           period_cnt <= period_cnt + 11'd1;

    always @(posedge clk or posedge reset)
        if(reset) bit_tick <= 1'b0;
//      else      bit_tick <= period_cnt == act_period[10:1]; // center
    else      bit_tick <= period_cnt == { 1'b0, i_period[10:1] }; // center

    always @(posedge clk or posedge reset)
    begin
    if     (reset   )   begin
                     o_din <= 8'b0;
                     r_din <= 8'b0;
                end
    else if(bit_tick)
        case(bit_cnt)
        4'd1                   : r_din <= 8'b0; // start
        4'd2, 4'd3, 4'd4, 4'd5,
        4'd6, 4'd7, 4'd8       : r_din <= {flt_rxd, r_din[7 : 1]};
        // on last bit, load output register.
        4'd9                   : o_din <= {flt_rxd, r_din[7 : 1]};
        default                : /* nothing */;
        endcase
    end

    always @(posedge clk or posedge reset)
    if(reset) o_valid <= 1'b0;
    else      o_valid <= (bit_cnt == 4'h9 && bit_tick) ||
                         (measure && |act_period) ||
                 (o_valid && ~(cfg_wr && cfg_accept));

endmodule
