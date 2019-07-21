/*

Module: catena4710_top.v

Function:
    Top level of design for Catena 4710.

Author:
    Terry Moore, MCCI Corporation

Copyright Notice and License Information:
    Copyright 2018-2019 MCCI Corporation

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

module catena4710_top #(
    parameter PC_RESET_ADDR  = 32'h00000000, // PC reset address
              HAS_MULT       = 1,            // 1 - Include MULT in RISCV
              UNSIGNED_MULT  = 1,            // 1 - MULT supports only unsigned operation
              INST_FILE_NAME = "none"    ,   // I-ROM file name - SPSRAM in SCE
              DROM_FILE_NAME = "drom.hex",   // D-ROM file name - EBR    in SCE
              DRAM_FILE_NAME = "none",       // D-RAM file name - SPSRAM in SCE - REMOVE - TO DO
              INC_ECC_ACC    = 0,            // 1 - Include ECC HW accelerator
              INC_TRNG       = 0,            // 1 - Include TRNG HW
              USE_HOSC_TRNG  = 1,            // 1 - Use SB_HF/LFOSC based TRNG, 0 - Use LUT based TRNG
              INC_AES_ACC    = 0,            // 1 - Include AES HW accelerator
              INC_UART_CON   = 1,            // 1 - Include UART controller
              DEBUG_CLOCK    = 0,            // 1 - generate debug clock
              DEBUG_CLOCK_MAX = 26'd6000000,  // maximum clock
              GPIO_WIDTH     = 7             // external GPIOs 6:0
)(
    inout          pin_usbp,    //        ; D+
    inout          pin_usbn,    //        ; D-
    output         uart1_tx, // ??        ; UART1 TX (message from RISC-V)
    input          uart1_rx, //           ; UART1 RX (message from PC)
    output         uart_tx, // 45         ; UART TX (message from RISC-V)
    input          uart_rx, // 46         ; UART RX (message from PC    )
    inout [GPIO_WIDTH-1 : 0]
                   gpio_pin, // ??        ; GPIO
    inout          i2c_scl_pin, // ??     ; I2C clock
    inout          i2c_sda_pin, // ??     ; I2C data
    output         SPI_SS,  // 16         ; flash_ss
    output         SPI_SCK, // 15         ; flash_sck
    output         SPI_SO,  // 14         ; flash_mosi
    input          SPI_SI,  // 17         ; flash_miso
    inout          lora_spi_sck,  //      ; lora SCK (i/o because CPU can read pin)
    inout          lora_spi_miso, //      ; lora miso (i/o because CPU can read pin)
    inout          lora_spi_mosi,  //     ; lora mosi
    inout [32 : 0] wire_D          //     ; the wire signals
);

`include "sb_io_param.vh"

    //================================================================================
    // Internal signals
    //================================================================================
    reg            rst_f;
    wire           ext_wr;           // External interface - API I/F w/ outside
    wire           ext_rd;           //
    wire  [ 7 : 0] ext_ad;           //
    wire  [ 7 : 0] ext_di;           //
    wire  [ 7 : 0] ext_do;           //
    wire           rom_ext_acc;      //
    wire           prom_ext_wr_en;   //
    wire           drom_ext_wr_en;   //
    wire  [31 : 0] rom_ext_wr_data;  //
    wire           fifo1_empty;
    wire           fifo_empty;
    wire  [31 : 0] inst_raddr; //
    wire  [31 : 0] inst_rdata_d0, inst_rdata_d1; //
    reg   [ 8 : 0] reset_cnt = 0;
    wire           cpu_spi_sck;
    wire           cpu_spi_mosi;
    wire           cpu_spi_miso;
    wire           irom_loader_done;
    wire           peri_uart0_wr;
    wire           peri_uart1_wr;
    wire           debugclk;
    wire  [31 : 0] bus_wrdata;
    wire  [ 3 : 0] bus_wrstrobe;
    wire  [31 : 0] bus_rddata;
    wire           mic_en;
    wire  [3 : 0]  mic_addr;

    //================================================================================
    // Parameters
    //================================================================================
    parameter OSC_48MHZ = 48000000,
              OSC_24MHZ = 24000000,
              OSC_12MHZ = 12000000,
              OSC_06MHZ =  6000000;
    parameter OSC_SPEED = OSC_12MHZ; // OSC clock speed; SET HERE
    parameter OSC_STR   = OSC_SPEED == OSC_48MHZ ? "0b00" :
                          OSC_SPEED == OSC_24MHZ ? "0b01" :
                          OSC_SPEED == OSC_12MHZ ? "0b10" : "0b11";
    parameter UART_BAUDRT = 9400;   //Actual baud appears to be ~1.018 * this value.
    parameter UART_PERIOD = OSC_SPEED / (UART_BAUDRT*2); // *2 is for /2 in clock gen part

    parameter UART1_BAUDRT = 115200 * 2;
    parameter UART1_PERIOD = OSC_SPEED / (UART1_BAUDRT*2); // *2 is for /2 in clock gen part

    parameter DUMMY_CYCLE = OSC_SPEED == OSC_48MHZ ? 20'd7 :
                            OSC_SPEED == OSC_24MHZ ? 20'd7 :
                            OSC_SPEED == OSC_12MHZ ? 20'd7 : 20'd7;

    parameter DROM_SZ_BYTES = INC_AES_ACC               ? 20'd1024 * 20'd6 :
                              (INC_ECC_ACC || INC_TRNG) ? 20'd1024 * 20'd4 :
                                                          20'd1024 * 20'd8;

    //================================================================================
    // Clock generator
    //================================================================================
    `ifdef HFOSC_SIMULATION
        reg  clk_hfosc;                 // high-frequency osc
        reg  clk_i = 0;            // RISC-V & HW ACCs clock
        initial begin
            clk_hfosc = 1'b0;
                    /* select the simulated clock rate */
            forever #(OSC_SPEED == OSC_24MHZ ? 20.8*1 :
                      OSC_SPEED == OSC_12MHZ ? 20.8*2 :
                      OSC_SPEED == OSC_06MHZ ? 20.8*4 :
                                               10.4) clk_hfosc = ~clk_hfosc;
        end
        initial clk_i = 1'b0;
    `else
        /* not simulating: select the appropriate clock */
        wire clk_hfosc;
        wire clk_i;
        SB_HFOSC #(.CLKHF_DIV(OSC_STR))
        OSCInst0( // 24Mhz ROSC
            .CLKHFEN(1'b1 ),
            .CLKHFPU(1'b1 ),
            .CLKHF  (clk_hfosc)
        ) /* synthesis ROUTE_THROUGH_FABRIC= 1 */;

        // clk_i runs at 50% of the selected rate.
        // it's much easier to set constraints with a named DFF as a
        // divider.
        wire clk_j;
        wire nclk_j = ~clk_j;
        SB_DFF clk_i_divider (
            .Q(clk_j),
            .C(clk_hfosc),
            .D(nclk_j)
        );
        // always @(posedge clk_hfosc) // 50% of OSC_SPEED
        //     clk_i <= ~clk_i;

        // then we drive the global buffer
        SB_GB clk_buf(
            .USER_SIGNAL_TO_GLOBAL_BUFFER(clk_j),
            .GLOBAL_BUFFER_OUTPUT        (clk_i)
        );
   `endif

    // Reset generation using globally initialized FFs
    //initial begin // for simulation
     //   reset_cnt = 9'b0;
    //end
    always @(posedge clk_i)
        if(!reset_cnt[8])
            reset_cnt <= reset_cnt + 1;
    always @(posedge clk_i)
        rst_f = !reset_cnt[8];


    //================================================================================
    // Boot loader for IROM from SPI flash
    // - Reads SPI flash and fills up SPSRAM
    //================================================================================
    irom_loader #(
            .DUMMY_CYCLE(DUMMY_CYCLE),
            .DROM_SZ_BYTES(DROM_SZ_BYTES)
            )
    irom_loader_u0(
        .RST        (rst_f           ), // i
        .OSC_HF     (clk_i           ), // i
        .SPI_SS     (SPI_SS          ), // o
        .SPI_SCK    (SPI_SCK),      // o
        .SPI_SO     (SPI_SO),       // o
        .SPI_SI     (SPI_SI),       // i
        .done       (irom_loader_done), // o
        .rom_acc    (rom_ext_acc     ), // o
        .prom_wr_en (prom_ext_wr_en  ), // o
        .drom_wr_en (drom_ext_wr_en  ), // o
        .rom_data   (rom_ext_wr_data )  // o [31:0]
    );

    //================================================================================
    // RISC-V w/ HW ACCs + UART
    //================================================================================
    riscv32 #(
        .HARDENED_HCE  (0             ), // 1 - Hardened CE (XO3s/XO4), 0 - Soft CE (T+)
        .PC_RESET_ADDR (PC_RESET_ADDR ), // PC reset address
        .HAS_MULT      (HAS_MULT      ), // 1 - Include MULT in RISCV
        .UNSIGNED_MULT (UNSIGNED_MULT ), // 1 - MULT supports only unsigned operation
        .INST_FILE_NAME(INST_FILE_NAME), // I-ROM file name - ROM in HCE, SPSRAM in SCE
        .DROM_FILE_NAME(DROM_FILE_NAME), // D-ROM file name - ROM in HCE, EBR    in SCE
        .DRAM_FILE_NAME(DRAM_FILE_NAME), // D-RAM file name - RAM in HCE, SPSRAM in SCE - REMOVE - TO DO
        .INC_ECC_ACC   (INC_ECC_ACC   ), // 1 - Include ECC HW accelerator
        .INC_TRNG      (INC_TRNG      ), // 1 - Include TRNG HW
        .USE_HOSC_TRNG (USE_HOSC_TRNG ), // 1 - Include TRNG HW
        .INC_AES_ACC   (INC_AES_ACC   ), // 1 - Include AES HW accelerator
        .INC_UART_CON  (INC_UART_CON  ), // 1 - Include UART controller
        .UART_PERIOD   (UART_PERIOD   ), // clk_i/UART_PERIOD = baud rate
        .UART1_PERIOD  (UART1_PERIOD  ), // clk_i/UART1_PERIOED = baud period
        .GPIO_WIDTH    (GPIO_WIDTH    )  // width of GPIO register
    ) riscv32_u0(
        .clk_i            (clk_i           ), // i          RISC-V & HW ACCs clock
        .rst_i            (rst_f         ), // i          Active high reset
        .crst_n           (1'b1),
        .rom_ext_acc      (rom_ext_acc    ), // i          I-ROM loading mode (used only in SCE)
        .prom_ext_wr_en   (prom_ext_wr_en  ), // i          I-ROM wr_en
        .drom_ext_wr_en   (drom_ext_wr_en  ), // i          I-ROM wr_en
        .rom_ext_wr_data  (rom_ext_wr_data), // i [31 : 0] I-ROM data
        .o_bus_wrstrobe   (bus_wrstrobe    ), // o [3:0] write strobes
        .o_bus_wrdata     (bus_wrdata      ), // o [31:0] write data, aligned
        .i_bus_rddata     (bus_rddata      ), // i [31:0] read data, aligned.
        .uart1_tx         (uart1_tx        ), // o          UART1 TX (message from RISC-V)
        .uart_tx          (uart_tx         ), // o          UART TX (message from RISC-V)
        .uart_rx          (uart_rx),     // i      UART0 RX (from world)
        .uart1_rx         (uart1_rx),    // i      UART1 RX (from world)
        .gpio_pin         (gpio_pin),     // i/o [ GPIO_WIDTH-1 : 0]
        .i2c_scl_pin      (i2c_scl_pin),
        .i2c_sda_pin      (i2c_sda_pin),
        .spi_sck_pin      (lora_spi_sck),  // o
        .spi_mosi_pin     (lora_spi_mosi), // o
        .spi_miso_pin     (lora_spi_miso), // i
        .fifo_empty       (fifo_empty      ),
        .fifo1_empty      (fifo1_empty     ),
        .inst_raddr       (inst_raddr      ),
        .inst_rdata_d0    (inst_rdata_d0),
        .inst_rdata_d1    (inst_rdata_d1),
        .peri_uart0_wr    (peri_uart0_wr),
        .peri_uart1_wr    (peri_uart1_wr),
        .peri_mic_en      (mic_en),
        .peri_mic_addr    (mic_addr        ),   // o [3 : 0]   mic reg addr
        .ext_wr           (ext_wr          ), // i          External interface - API I/F w/ outside
        .ext_rd           (ext_rd          ), // i
        .ext_ad           (ext_ad          ), // i [ 7 : 0]
        .ext_di           (ext_di          ), // i [ 7 : 0]
        .ext_do           (ext_do          )  // o [ 7 : 0]
    );

    //================================================================================
    // User logic
    // - Use CE's functions using EXT I/F API
    //================================================================================
    assign ext_wr = 1'b0;
    assign ext_rd = 1'b0;
    assign ext_ad = 8'h8f;
    assign ext_di = 8'b0;

    reg   [25 : 0] clock_counter;
    reg        clock_toggle;

    generate
    if(DEBUG_CLOCK)
        begin: debug_clock_inst
                assign debugclk = clock_toggle;
                always @(posedge clk_i or posedge rst_f)
                begin
                        clock_counter <= rst_f          ? 26'b0
                                        : (clock_counter == DEBUG_CLOCK_MAX) ? 26'b0
                                        : clock_counter + 26'b1
                                        ;
                        clock_toggle  <= rst_f ? 1'b0
                                        : (clock_counter == DEBUG_CLOCK_MAX) ? ~clock_toggle
                                        : clock_toggle
                                        ;
                end
        end
    else
            begin: no_debug_clock
                assign debugclk = 1'b0;
        end
    endgenerate

    //================================================================================
    // the mic module
    //================================================================================

    pdm_audio
    /* #() */
    pdm_audio_inst(
            .i_busclk   (clk_i),
            .i_reset    (1'b0),
            .i_en       (mic_en),
            .i_addr     (mic_addr),
            .i_wrdata   (bus_wrdata),
            .i_wrstrobe (bus_wrstrobe),
            .o_rddata   (bus_rddata),

            .i_hf_clock (clk_hfosc),
            .pin_pdm_data (wire_D[29]),
            .pin_pdm_clk (wire_D[28])
    );

    assign wire_D[ 0] = 1'bZ;
    assign wire_D[ 1] = 1'bZ;
    assign wire_D[ 2] = 1'bZ;
    assign wire_D[ 3] = 1'bZ;
    assign wire_D[ 4] = 1'bZ;
    assign wire_D[ 5] = 1'bZ;
    assign wire_D[ 6] = 1'bZ;
    assign wire_D[ 7] = 1'bZ;
    assign wire_D[ 8] = 1'bZ;
    assign wire_D[ 9] = 1'bZ;
    assign wire_D[10] = 1'bZ;
    assign wire_D[11] = 1'bZ;
    assign wire_D[12] = 1'bZ;
    assign wire_D[13] = 1'b0;           // D13: LED.
    assign wire_D[14] = 1'bZ;
    assign wire_D[15] = 1'bZ;
    assign wire_D[16] = 1'bZ;
    assign wire_D[17] = 1'bZ;
    assign wire_D[18] = 1'bZ;
    assign wire_D[19] = 1'bZ;
    assign wire_D[20] = 1'bZ;
    assign wire_D[21] = 1'bZ;
    assign wire_D[22] = 1'bZ;
    assign wire_D[23] = 1'bZ;
    assign wire_D[24] = 1'bZ;
    assign wire_D[25] = 1'bZ;

//  assign wire_D[26] = 1'bZ;
    SB_IO_OD #(
        .PIN_TYPE ( IOB_PIN_OUTPUT_NONE | IOB_PIN_INPUT )
        )
    pad_rgb2(
        .PACKAGEPIN (wire_D[26]),
        .OUTPUTENABLE (),
        .LATCHINPUTVALUE (),
        .CLOCKENABLE (),
        .INPUTCLK (),
        .OUTPUTCLK (),
        .DOUT0 (),
        .DOUT1 (),
        .DIN0 (),
        .DIN1 ()
        );

    assign wire_D[27] = 1'bZ;
//  assign wire_D[28] = 1'b0;           // D28 MIC clock
//  assign wire_D[29] = 1'bZ;           // D29 MIC data

//  assign wire_D[30] = 1'bZ;
    SB_IO_OD #(
        .PIN_TYPE ( IOB_PIN_OUTPUT_NONE | IOB_PIN_INPUT )
        )
    pad_rgb0(
        .PACKAGEPIN (wire_D[30]),
        .OUTPUTENABLE (),
        .LATCHINPUTVALUE (),
        .CLOCKENABLE (),
        .INPUTCLK (),
        .OUTPUTCLK (),
        .DOUT0 (),
        .DOUT1 (),
        .DIN0 (),
        .DIN1 ()
        );

//  assign wire_D[31] = 1'bZ;
    SB_IO_OD #(
        .PIN_TYPE ( IOB_PIN_OUTPUT_NONE | IOB_PIN_INPUT )
        )
    pad_rgb1(
        .PACKAGEPIN (wire_D[31]),
        .OUTPUTENABLE (),
        .LATCHINPUTVALUE (),
        .CLOCKENABLE (),
        .INPUTCLK (),
        .OUTPUTCLK (),
        .DOUT0 (),
        .DOUT1 (),
        .DIN0 (),
        .DIN1 ()
        );

    assign wire_D[32] = 1'b0;           // 48 MHz enable: off

    assign pin_usbp = 1'bZ;
    assign pin_usbn = 1'bZ;

endmodule
//================================================================================
// End of file
//================================================================================
