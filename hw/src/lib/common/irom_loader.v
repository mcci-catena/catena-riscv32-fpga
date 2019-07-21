/*

Module: irom_loader.v

Function:
    FSM to load SPI flash into instruction memory ("ROM").

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

`timescale 1ns/10ps

module irom_loader #(
        parameter DUMMY_CYCLE=20'd7,
              DROM_SZ_BYTES=20'd4096
    ) (
    input             RST,        //
    input             OSC_HF,     // Clock (= RISC-V clock)
    output reg        SPI_SS,     // SPI I/F for flash access
    output reg        SPI_SCK,    //
    input             SPI_SI,     //
    output reg        SPI_SO,     //
    output reg        done,
    output reg        rom_acc,    // SPSRAM (IROM) / EBR (DROM) fill out
    output reg        prom_wr_en, // - IROM (PROM) write strobe
    output reg        drom_wr_en, // - DROM        write strobe
    output reg [31:0] rom_data    //
);
    //================================================================================
    // Parameters
    //================================================================================
    parameter IDLE         =  3'd0,         // SPI access FSM
              PREP         =  3'd1,         //
              CMD          =  3'd2,         //
              SADDR        =  3'd3,         //
              DUMMY        =  3'd4,         //
              RDBYTE       =  3'd5,         //
              WAIT         =  3'd6,         //
              WAIT10       =  3'd7;         // 10us wait for wake up from power down
    parameter NO_SCK       =  1'b1,         // OR mask for SCK
              ON_SCK       =  1'b0;         //
    parameter FAST_RD      =  8'h0b,        // Fast read flash command
              RLS_DPD      =  8'hab;        // Release from deep power-down
    parameter ST_ADDR      = 24'h060000;    // Starting address of ROM in flash
    parameter IROM_SZ_BITS = 20'h80000,     // 64KB IROM size in bits (64*1024*8)
              DROM_SZ_BITS = 20'd8 * DROM_SZ_BYTES;
                        //  DROM size in bits
    parameter ROM_SZ_BITS  = IROM_SZ_BITS + //
                 DROM_SZ_BITS - //
                 20'd1;         //

    //================================================================================
    // Internal signals
    //================================================================================
    reg  [2  : 0] cst, nst;
    reg  [19 : 0] cnt;
    reg           sck_msk, sck_msk_pe;
    reg  [4  : 0] bit_cnt;  // Accumulate 32b of data
    reg           en;
    reg           phase;

    //================================================================================
    // Toggling "en" to make two cycle per state FSM
    // - FSM moves to next state if "en=1"
    //================================================================================
    always @(posedge OSC_HF)// or posedge RST)
//        if(RST) en <= 1'b0;
//  else
        en <= ~en;

    //================================================================================
    // Flash access FSM
    // - 9 cycles of dummy (not 8) for fast reading
    //================================================================================
    always @(posedge OSC_HF) // or posedge RST)
        if     (RST          ) phase <= 1'b0; // Wake up phase
    else if(cst == WAIT10) phase <= 1'b1; // Read    phase

    always @(posedge OSC_HF) // or posedge RST)
        if     (RST) cst <= IDLE;
    else if(en ) cst <= nst;

    always @(*)
        case(cst)
            IDLE   : nst  =           PREP;
            PREP   : nst  =           CMD;
            CMD    : nst  =  |cnt   ? CMD    :
                            phase ? SADDR  : WAIT10;
            SADDR  : nst  = ~|cnt   ? DUMMY  : SADDR;
            DUMMY  : nst  = ~|cnt   ? RDBYTE : DUMMY;
            RDBYTE : nst  = ~|cnt   ? WAIT   : RDBYTE;
            WAIT   : nst  =           WAIT;
            default: nst  = ~|cnt   ? IDLE   : WAIT10;
        endcase

    always @(posedge OSC_HF) // or posedge RST)
//        if(RST) cnt <= 20'b0;
//  else
    if(en)
        case(cst)
            IDLE   : cnt <=                  20'd00;         //
            PREP   : cnt <=                  20'd07;         //  8 bits  of CMD
            CMD    : cnt <= |cnt   ? cnt - 1 :
                            phase ? 20'd23  :               // 24 bits  of Start Address
                            20'd50  ;               // 10us+ delay after power up
            SADDR  : cnt <= |cnt   ? cnt - 1 : DUMMY_CYCLE;  //  m bits  of DUMMY
            DUMMY  : cnt <= |cnt   ? cnt - 1 : ROM_SZ_BITS;  //  n bytes of data
            RDBYTE : cnt <= |cnt   ? cnt - 1 : 20'd00;
            WAIT   : cnt <=                    20'd00;
            default: cnt <= |cnt   ? cnt - 1 : 20'd00;
        endcase

    //================================================================================
    // SPI signal generation
    // - SPI_SS is the CS_B
    //================================================================================
    always @(posedge OSC_HF) // or posedge RST)
        if(RST) {SPI_SS, SPI_SO} <= {1'b1, 1'b1};
    else if(en)
        case(cst)
            IDLE   : begin
                        SPI_SS  <= 1'b0;
                        SPI_SO  <= 1'b1;
                    end
            PREP   : begin
                    SPI_SS  <= 1'b0;
                    SPI_SO  <= phase ? FAST_RD[7] : RLS_DPD[7];
                end
            CMD    : if(|cnt) begin // Command
                        SPI_SS  <= 1'b0;
                        SPI_SO  <= phase ? FAST_RD[cnt-1] : RLS_DPD[cnt-1];
                    end else begin      // S-Addr
                        SPI_SS  <= phase ? 1'b0        : 1'b1;
                        SPI_SO  <= phase ? ST_ADDR[23] : 1'b1;
                    end
            SADDR  : if(|cnt) begin // S-Addr
                        SPI_SS  <= 1'b0;
                        SPI_SO  <= ST_ADDR[cnt-1];
                    end else begin      // Dummy
                        SPI_SS  <= 1'b0;
                        SPI_SO  <= 1'b1; // Dummy
                    end
            DUMMY  : if(|cnt) begin // Dummy
                        SPI_SS  <= 1'b0;
                        SPI_SO  <= 1'b1;
                    end else begin      // Read byte
                        SPI_SS  <= 1'b0;
                        SPI_SO  <= 1'b1; // Don't care
                    end
            RDBYTE : if(|cnt) begin // Read byte
                        SPI_SS  <= 1'b0;
                        SPI_SO  <= 1'b1;
                    end else begin
                        SPI_SS  <= 1'b1;
                        SPI_SO  <= 1'b1;
                    end
            WAIT   : {SPI_SS, SPI_SO} <= {1'b1, 1'b1};
            default: {SPI_SS, SPI_SO} <= {1'b1, 1'b1};
        endcase

    always @(posedge OSC_HF)// or posedge RST)
        //if(RST) SPI_SCK <= 1'b1;
    //else
        case(cst)
            PREP, SADDR, DUMMY  : SPI_SCK <= ~en;
            CMD                 : SPI_SCK <= phase || |cnt ? ~en : 1'b1;
            RDBYTE              : SPI_SCK <= |cnt ? ~en : 1'b1;
            default             : SPI_SCK <= 1'b1;
        endcase

    //================================================================================
    // SPSRAM access (write) FSM
    // - Direct access using rom_acc, prom_wr_en, and rom_data (32b)
    // - If rom_acc & prom_wr_en, rom_data is written to SPSRAM at every cycle w/
    //   auto increased address
    // - For hardware convenience, cnt is a down-counter. But the read addresses
    //   ascend. So we assert prom_wr_en as long as count is above DROM_SZ_BITs,
    //   then assert drom_wr_en. For convenience, the IROM size is fixed at 64K,
    //   but you can adjust how large the DROM size is.
    //================================================================================
    always @(posedge OSC_HF)// or posedge RST)
        if(RST) begin
            bit_cnt    <= 5'd31;
            rom_acc    <=  1'b1;
            prom_wr_en <=  1'b0;
            drom_wr_en <=  1'b0;
            rom_data   <= 32'b0;
        end else if(cst == RDBYTE && !en) begin
            bit_cnt    <= bit_cnt - 1;
            rom_acc    <=  1'b1;
            prom_wr_en <= (cnt >= DROM_SZ_BITS) && (~|bit_cnt);
            drom_wr_en <= (cnt <  DROM_SZ_BITS) && (~|bit_cnt);
            rom_data   <= {rom_data[30 : 0], SPI_SI};
        end else begin
            rom_acc    <=  (cst != WAIT && cst != DUMMY);
            prom_wr_en <=  1'b0;
            drom_wr_en <=  1'b0;
        end

    always @(posedge OSC_HF)
        done <= cst == WAIT;
endmodule
