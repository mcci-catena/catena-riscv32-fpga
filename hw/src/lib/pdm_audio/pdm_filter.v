/*

Module: pdm_filter.v

Function:
    PDM input data C-I-C filter

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

`define LG2(n)                    \
    (                             \
    (n) <= 1    ? 1 :             \
    (n) <= 3    ? 2 :             \
    (n) <= 7    ? 3 :             \
    (n) <= 15   ? 4 :             \
    (n) <= 31   ? 5 :             \
    (n) <= 63   ? 6 :             \
    (n) <= 127  ? 7 :             \
    (n) <= 255  ? 8 :             \
    (n) <= 511  ? 9 :             \
    (n) <= 1023 ? 10 :            \
                  32'hXXXXXXXX    \
    )

module pdm_filter #(
  SAMPLE_CLKS = 375,
  SAMPLEHS_CLKS = 15,
  BITS = 18
) (
  input i_clk,
  input i_reset,
  input i_sample_ready,
  input [`LG2(SAMPLEHS_CLKS-1) - 1 : 0] i_sample,

  output reg [ BITS-1 : 0 ] o_filtered,
  output o_filtered_ready
);
  // this unpleasant sequence of assigns makes lint pass.
  localparam DECIMATION_UNSIZED = SAMPLE_CLKS / SAMPLEHS_CLKS;
  localparam DECIMATION_BITS = `LG2(DECIMATION_UNSIZED-1);
  localparam [DECIMATION_BITS-1 : 0] DECIMATION = DECIMATION_UNSIZED[DECIMATION_BITS-1 : 0];

  // get #/bits needed to represent hs clocks.
  localparam SAMPLEHS_CLKS_BITS = `LG2(SAMPLEHS_CLKS-1);

  reg  [ DECIMATION_BITS-1 : 0 ]  decimation_counter;
  reg  [ BITS - 1 : 0 ] acc_i1;
  reg  [ BITS - 1 : 0 ] acc_i2;
  reg  [ BITS - 1 : 0 ] acc_i3;

  reg  [ BITS - 1 : 0 ] acc_c1;
  reg  [ BITS - 1 : 0 ] last_c1;
  reg  [ BITS - 1 : 0 ] acc_c2;
  reg  [ BITS - 1 : 0 ] last_c2;
  reg  [ BITS - 1 : 0 ] acc_c3;
  reg  [ BITS - 1 : 0 ] last_c3;

  // useful constants
  wire [ BITS - 1 : 0 ] zero = 0;
  wire [ SAMPLEHS_CLKS_BITS - 1 : 0 ] sample_bias = SAMPLEHS_CLKS >> 1;
  wire [ BITS - SAMPLEHS_CLKS_BITS - 1 : 0] zeropad_hs = 0;
  wire [ DECIMATION_BITS-1 : 0 ] decimation_counter_max = DECIMATION - 1;

  // the integrators
  always @(posedge i_clk) begin
    if (i_reset)
      begin
        acc_i1 <= zero;
        acc_i2 <= zero;
        acc_i3 <= zero;
      end
    else if (i_sample_ready)
      begin
        acc_i1 <= acc_i1 + { zeropad_hs, i_sample } - {zeropad_hs, sample_bias };
        acc_i2 <= acc_i2 + acc_i1;
        acc_i3 <= acc_i3 + acc_i2;
      end
  end

  // the decimation counter
  wire decimation_counter_ismax = (decimation_counter == decimation_counter_max);

  always @(posedge i_clk) begin
    if (i_reset)
      begin
        decimation_counter <= 0;
      end
    else if (i_sample_ready)
      begin
        if (!decimation_counter_ismax)
          decimation_counter <= decimation_counter + 1;
        else
          decimation_counter <= 0;
      end
  end

  // the combs
  always @(posedge i_clk) begin
    if (i_reset)
      begin
        acc_c1 <= 0;
        last_c1 <= 0;
        acc_c2 <= 0;
        last_c2 <= 0;
        acc_c3 <= 0;
        last_c3 <= 0;
      end
    else if (i_sample_ready && decimation_counter_ismax)
      begin
        acc_c1 <= acc_i3 - last_c1;
        last_c1 <= acc_i3;

        acc_c2 <= acc_c1 - last_c2;
        last_c2 <= acc_c1;

        acc_c3 <= acc_c2 - last_c3;
        last_c3 <= acc_c2;
      end
  end

  // the output bit counter
  reg r_filtered_ready;

  always @(posedge i_clk) begin
    if (i_reset)
      begin
        r_filtered_ready <= 0;
        o_filtered <= 0;
      end
    else
      begin
        if (i_sample_ready && decimation_counter_ismax)
          o_filtered <= acc_c3;

        // one-clock pulse when the value is ready.
        r_filtered_ready <= i_sample_ready && decimation_counter_ismax;
      end
  end

  assign o_filtered_ready = r_filtered_ready;
endmodule
