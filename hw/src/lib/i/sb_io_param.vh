/*

Module: sb_io_param.vh

Function:
    Symbols for configuring IO pins for Sonic Blue/Lattice ICE.

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

// note that this is included inside a module, and therefore might
// be included multiple times.

//================================================================================
// defaults for the various pins -- OR them together.
//================================================================================
localparam IOB_PIN_INPUT                                        = 'b000001;
localparam IOB_PIN_INPUT_LATCH                                  = 'b000011;
localparam IOB_PIN_INPUT_REGISTERED                             = 'b000000;
localparam IOB_PIN_INPUT_DDR                                    = IOB_PIN_INPUT_REGISTERED;
localparam IOB_PIN_INPUT_REGISTERED_LATCH                       = 'b000001;

localparam IOB_PIN_OUTPUT_NONE                                  = 'b000000;
localparam IOB_PIN_OUTPUT                                       = 'b011000;
localparam IOB_PIN_OUTPUT_TRISTATE                              = 'b101000;
localparam IOB_PIN_OUTPUT_ENABLE_REGISTERED                     = 'b111000;
localparam IOB_PIN_OUTPUT_REGISTERED                            = 'b010100;
localparam IOB_PIN_OUTPUT_REGISTERED_ENABLE                     = 'b100100;
localparam IOB_PIN_OUTPUT_REGISTERED_ENABLE_REGISTERED          = 'b110100;
localparam IOB_PIN_OUTPUT_DDR                                   = 'b010000;
localparam IOB_PIN_OUTPUT_DDR_ENABLE                            = 'b100000;
localparam IOB_PIN_OUTPUT_DDR_ENABLE_REGISTERED                 = 'b110000;
localparam IOB_PIN_OUTPUT_REGISTERED_INVERTED                   = 'b101100;
localparam IOB_PIN_OUTPUT_REGISTERED_ENABLE_REGISTERED_INVERTED = 'b111100;

// end of file
