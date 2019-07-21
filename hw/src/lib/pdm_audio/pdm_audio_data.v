/*

Module: pdm_audio_data.v

Function:
    Data pad for simple PDM audio converter

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

// `define MIC_DEBUG
// `define MIC_DEBUG_RAW
// MIC_DEBUG adds a port for snooping data, and moves register inboard
// MIC_DEBUG_RAW attempts to output exactly what the pin is bringing in;
// otherwise we output the registered data.

module pdm_audio_data #() (
  inout   pin_pdm_data,
  input   i_pdm_clk,
  output  o_pdm_data_r
`ifdef MIC_DEBUG
  , output o_pdm_data_raw
`endif
);
`include "sb_io_param.vh"

`ifndef MIC_DEBUG
  // Set up the pad. Note that we have one level of register at the pad,
  // synched to pdm_clk.
  SB_IO #(
    .PIN_TYPE( IOB_PIN_OUTPUT_NONE | IOB_PIN_INPUT_REGISTERED /* 6'b000000 */ ),
    .PULLUP(1'b0)
  ) pad_pdm_data_inst (
    .PACKAGE_PIN(pin_pdm_data),
    .LATCH_INPUT_VALUE(),
    .CLOCK_ENABLE(),
    .INPUT_CLK(i_pdm_clk),
    .OUTPUT_CLK(),
    .OUTPUT_ENABLE(),
    .D_OUT_0(),
    .D_OUT_1(),
    .D_IN_0(o_pdm_data_r),
    .D_IN_1()
  );
`else // def MIC_DEBUG
`ifdef MIC_DEBUG_RAW
  // Set up the pad. Not registered at the pad.
  SB_IO #(
    .PIN_TYPE( { IOB_PIN_OUTPUT_NONE, IOB_PIN_INPUT } ),
    .PULLUP(1'b0)
  ) pad_pdm_data_inst (
    .PACKAGE_PIN(pin_pdm_data),
    .LATCH_INPUT_VALUE(),
    .CLOCK_ENABLE(),
    .INPUT_CLK(),
    .OUTPUT_CLK(),
    .OUTPUT_ENABLE(),
    .D_OUT_0(),
    .D_OUT_1(),
    .D_IN_0(o_pdm_data_raw),
    .D_IN_1()
  );

  // use a fabric flop to sample the data.
  reg o_pdm_data_r = 0;

  always @(posedge i_pdm_clk)
    o_pdm_data_r <= o_pdm_data_raw;
`else // ndef MIC_DEBUG_RAW
  // Set up the pad. Note that we have one level of register at the pad,
  // synched to pdm_clk. This clause exists so we can quickly try
  // the debug build with a production pad layout.
  SB_IO #(
    .PIN_TYPE( { IOB_PIN_OUTPUT_NONE, IOB_PIN_INPUT_REGISTERED } ),
    .PULLUP(1'b0)
  ) pad_pdm_data_inst (
    .PACKAGE_PIN(pin_pdm_data),
    .LATCH_INPUT_VALUE(),
    .CLOCK_ENABLE(),
    .INPUT_CLK(i_pdm_clk),
    .OUTPUT_CLK(),
    .OUTPUT_ENABLE(),
    .D_OUT_0(),
    .D_OUT_1(),
    .D_IN_0(o_pdm_data_r),
    .D_IN_1()
  );

  // just copy o_pdm_data_r in this case.
  assign o_pdm_data_raw = o_pdm_data_r;

`endif // MIC_DEBUG_RAW
`endif // MIC_DEBUG

endmodule
