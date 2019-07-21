/*

Module: pdm_audio.v

Function:
    Simple PDM audio converter

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

module pdm_audio #(
  parameter   HF_CLOCK_HZ = 12000000,
  parameter   PDM_CLOCK_HZ = 3000000,
  parameter   SAMPLE_HZ = 8000,
  parameter   SAMPLE_CLKS = 375,
  parameter   SAMPLEHS_CLKS = 15
) (
  // here's the CPU write interface
  input           i_busclk,     // data interface clock
  input           i_reset,      // a reset input, in case we need it; synchronous to busclk, active high
  input           i_en,         // the register enable
  input   [3:0]   i_addr,       // the address bus (register select)
  input   [31:0]  i_wrdata,     // the write data
  input   [3:0]   i_wrstrobe,   // the write strobe (one per byte)
  output  [31:0]  o_rddata,     // the read data

  // here's the PDM audio interface. The clock and the PDM pin are here, too
  // but are isolated in technology-specific modules.
  input           i_hf_clock,   // the high-frequency clock
  inout           pin_pdm_data, // the PDM data pin.
  output          pin_pdm_clk   // the PDM clock pin.
);

  // see http://dspguru.com/files/cic.pdf eqn 13.
  localparam  PDM_FILTER_BITS = 18;
  localparam  SAMPLE_BITS = `LG2(SAMPLEHS_CLKS-1);
  localparam  R_CTRL = 4'h0;
  localparam  R_DATA = 4'h4;
  localparam  R_CONFIG = 4'h8;

  // local wiring
  wire  pdm_clk;

  // the first-stage registers
  reg   [SAMPLE_BITS-1 : 0]   pdmData_counter = 0;
  reg   [SAMPLE_BITS-1 : 0]   pdmData_bitcount = 0;

  // the outputs from the filter
  reg   [PDM_FILTER_BITS-1 : 0] pdmData_filtered;
  wire  pdmData_filtered_ready;

  // some bits
  reg   pdmData_update_r, pdmData_update_rr, pdmData_update_rrr;
  wire  pdm_data_r;
  reg   pdm_data_rr;
  reg   pdmCtrl_run_r, pdmCtrl_run_rr;
  wire  pdmData_bitcount_max;

  // useful constant
  wire  [SAMPLE_BITS-1 : 0]   pdmData_zero = 0;
  wire  [SAMPLE_BITS-2 : 0]   pdmData_zero_drop1 = 0;

  reg                         pdmData_rdy = 0;
  reg                         pdmCtrl_run = 0;
  wire  [31:0]  rPdmCtrl = { 31'b0, pdmCtrl_run };
  wire  [31:0]  rPdmData = { pdmData_rdy, 15'b0, pdmData_filtered[PDM_FILTER_BITS-1 : PDM_FILTER_BITS-16] };
  wire  [15:0]  kSampleClks16 = SAMPLE_CLKS;
  wire  [15:0]  kSampleHz16 = SAMPLE_HZ;
  wire  [31:0]  rPdmConfig = { kSampleClks16, kSampleHz16 };

  // addressing help
  wire  [3:0]   regAddr = { i_addr[3:2], 2'b00 };

  // read
  assign o_rddata = (regAddr == R_CTRL) ? rPdmCtrl
                  : (regAddr == R_DATA) ? rPdmData
                  :                       rPdmConfig
                  ;

  // writes to CTRL, and CTRL fanout
  always @(posedge i_busclk) begin
    if (i_reset)
      begin
        pdmCtrl_run <= 1'b0;
      end
    else if (i_en && i_wrstrobe[0] && regAddr == R_CTRL)
      begin
        pdmCtrl_run <= i_wrdata[0];
      end
  end

  // synch up the changes from the counter
  always @(posedge i_busclk) begin
    pdmData_update_r <= pdmData_filtered_ready;
    pdmData_update_rr <= pdmData_update_r;
    pdmData_update_rrr <= pdmData_update_rr;
  end

  // writes to DATA
  always @(posedge i_busclk) begin
    if (i_reset)
      begin
        pdmData_rdy <= 0;
      end
    else if (pdmData_update_rr && ! pdmData_update_rrr)
      begin
        pdmData_rdy <= 1'b1;
      end
    else if (i_en && i_wrstrobe[3] && regAddr == R_DATA && i_wrdata[31])
      begin
        pdmData_rdy <= 0;
      end
    else
      begin
        pdmData_rdy <= pdmData_rdy;
      end
  end

  // the PDM clock
  pdm_audio_clk pdm_audio_clk_inst (
    .pin_pdm_clk(pin_pdm_clk),
    .o_pdm_clk(pdm_clk),
    .i_hf_clock(i_hf_clock),
    .i_enable(pdmCtrl_run)
  );

  // the PDM data
  pdm_audio_data pdm_audio_data_inst (
    .pin_pdm_data(pin_pdm_data),
    .i_pdm_clk(pdm_clk),
    .o_pdm_data_r(pdm_data_r)
  );

  // synchronize the PDM data counter
  always @(posedge pdm_clk) begin
    pdm_data_rr <= pdm_data_r;
    pdmCtrl_run_r <= pdmCtrl_run;
    pdmCtrl_run_rr <= pdmCtrl_run_r;
  end

  // count bits
  always @(posedge pdm_clk) begin
    if (! pdmCtrl_run_rr || pdmData_bitcount_max)
      begin
        pdmData_bitcount <= 0;
      end
    else
      begin
        pdmData_bitcount <= pdmData_bitcount + 1;
      end
  end

  assign pdmData_bitcount_max = pdmData_bitcount == (SAMPLEHS_CLKS - 1);

  always @(posedge pdm_clk) begin
    if (! pdmCtrl_run_rr)
      pdmData_counter <= 0;
    else if (pdmData_bitcount_max)
      pdmData_counter <= {pdmData_zero_drop1, pdm_data_rr};
    else
      pdmData_counter <= pdmData_counter + {pdmData_zero_drop1, pdm_data_rr};
  end

  wire [PDM_FILTER_BITS-1 : 0] pdmData_filtered_out;

  pdm_filter #(
    .SAMPLE_CLKS    (SAMPLE_CLKS),
    .SAMPLEHS_CLKS  (SAMPLEHS_CLKS),
    .BITS           (PDM_FILTER_BITS)
  ) pdm_filter_inst (
    .i_clk          (pdm_clk),
    .i_reset        (i_reset),
    .i_sample_ready (pdmData_bitcount_max),
    .i_sample       (pdmData_counter),
    .o_filtered     (pdmData_filtered_out),
    .o_filtered_ready (pdmData_filtered_ready)
  );

  always @(posedge pdm_clk) begin
    if (! pdmCtrl_run_rr)
      pdmData_filtered <= 0;
    else if (pdmData_filtered_ready)
      pdmData_filtered <= pdmData_filtered_out;
  end

endmodule
