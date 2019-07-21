/*

Module: pdm_audio_clk.v

Function:
    Clock pad for simple PDM audio converter

Author:
    Terry Moore, MCCI Corporation

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

module pdm_audio_clk #() (
  inout   pin_pdm_clk,
  output  o_pdm_clk,
  input   i_hf_clock,
  input   i_enable
);
`include "sb_io_param.vh"

  // set up the pad.
  SB_IO #(
    .PIN_TYPE( IOB_PIN_OUTPUT | IOB_PIN_INPUT /* 6'b011001 */ ),
    .PULLUP(1'b0)
  ) pad_pdm_clk_inst (
    .PACKAGE_PIN(pin_pdm_clk),
    .LATCH_INPUT_VALUE(),
    .CLOCK_ENABLE(),
    .INPUT_CLK(),
    .OUTPUT_CLK(),
    .OUTPUT_ENABLE(),
    .D_OUT_0(o_pdm_clk),
    .D_OUT_1(),
    .D_IN_0(),
    .D_IN_1()
  );

  reg pdm_clk_en_r = 0;
  reg pdm_clk_en_rr = 0;

  // synchonize the enable
  always @(posedge i_hf_clock) begin
    pdm_clk_en_r <= i_enable;
    pdm_clk_en_rr <= pdm_clk_en_r;
  end

  `ifdef PDMOSC_SIMULATION
  reg [1:0] pdm_clk_divider = 2'b0;

  // generate the 3 MHz clock from 12 MHz
  always @(posedge i_hf_clock) begin
    if (pdm_clk_en_rr || (|pdm_clk_divider) )
      pdm_clk_divider <= pdm_clk_divider + 2'b01;
    else
      pdm_clk_divider <= 2'b00;
  end
  assign o_pdm_clk = pdm_clk_divider[1];
  `else

  // active high synchronous reset for PDM clock.
  wire pdm_clk_rst = ~pdm_clk_en_rr;

  // divide i_hf_clk by 2, output to pdm_clk_q0. No need for
  // enable because we always toggle.
  wire pdm_clk_nq0, pdm_clk_q0;
  assign pdm_clk_nq0 = ~pdm_clk_q0;

  SB_DFFSR pdm_clk_b0_inst (
    .Q(pdm_clk_q0),
    .C(i_hf_clock),
    .D(pdm_clk_nq0),
    .R(pdm_clk_rst)
  );

  // divide pdm_clk_q0 by 2, output to o_pdm_clk.
  // run on hfosc; so this is really a 2-bit counter.
  // Explicitly instantiated because we want to be
  // able to specify constraints more easily; otherwise
  // Synplify likes to turn this into the hf clock plus
  // more conditions on clock enables. Saves on clock
  // domains, but makes PNR more difficult.
  wire pdm_clk, pdm_clk_n;
  assign pdm_clk_n = ~pdm_clk;

  SB_DFFESR pdm_clk_divider_inst (
    .Q(pdm_clk),
    .C(i_hf_clock),
    .E(pdm_clk_q0),
    .D(pdm_clk_n),
    .R(pdm_clk_rst)
  );

  // generate a global clock
  SB_GB pdm_clk_buf(
      .USER_SIGNAL_TO_GLOBAL_BUFFER(pdm_clk),
      .GLOBAL_BUFFER_OUTPUT        (o_pdm_clk)
  );
  `endif // PDMOSC_SIMULATION

endmodule
