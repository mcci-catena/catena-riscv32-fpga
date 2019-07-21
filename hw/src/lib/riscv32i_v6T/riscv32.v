/*

Module: riscv32.v

Function:
    Top level module for RISC-V CPU

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

//================================================================================
// Config parameters (0 - no support, 1 - support)
//================================================================================
// INC_ECC_ACC: Decides the support of ECC HW accelerator
//              If 0, SW code needs to support all functions in SW
// INC_TRNG   : Decides the support of HW based TRNG
// INC_AES_ACC: Decides the support of AES HW accelerator
//================================================================================

`include "./riscv_include.v"

//================================================================================
//================================================================================
//================================================================================
module riscv32 #(
    parameter HARDENED_HCE   = 0,            // 1 - Hardened CE (XO3s/XO4), 0 - Soft CE (T+)
              PC_RESET_ADDR  = 32'h00000000, // PC reset address
              HAS_MULT       = 1,            // 1 - Include MULT in RISCV
              UNSIGNED_MULT  = 1,            // 1 - MULT supports only unsigned operation
              INST_FILE_NAME = "irom.hex",   // I-ROM file name - ROM in HCE, SPSRAM in SCE
              DROM_FILE_NAME = "drom.hex",   // D-ROM file name - ROM in HCE, EBR    in SCE
              DRAM_FILE_NAME = "none",       // D-RAM file name - RAM in HCE, SPSRAM in SCE - REMOVE - TO DO
              DROM_IN_SPSRAM = 0,            // 1 - DROM is in SPSRAM (w/ DRAM), 0 - DROM on EBR
              INC_ECC_ACC    = 0,            // 1 - Include ECC HW accelerator
              INC_TRNG       = 1,            // 1 - Include TRNG HW
              USE_HOSC_TRNG  = 1,            // 1 - Use SB_HF/LFOSC based TRNG, 0 - Use LUT based TRNG
              INC_AES_ACC    = 1,            // 1 - Include AES HW accelerator
              INC_UART_CON   = 1,            // 1 - Include UART controller
              UART_PERIOD    = 11'd104,      // clk_i/UART_PERIOD = 6MHz: 57600, 12MHz: 115200, 24MHz: 230400 (baud rate)
              UART1_PERIOD   = 11'd104,      // clk_i/UART_PERIOD => baudrate
              GPIO_WIDTH     = 4             // how many GPIOs?
)(
    input           clk_i,            // RISC-V & HW ACCs clock
    input           rst_i,            // Active high reset
    input           crst_n,
    input           rom_ext_acc,     // I-ROM loading mode (used only in SCE)
    input           prom_ext_wr_en,   // I-ROM wr_en
    input           drom_ext_wr_en,   // D-ROM wr_en
    input  [31 : 0] rom_ext_wr_data, // I-ROM data
    // TODO(tmm@mcci.com) unify the non-CPU peripheral accesses, and use a
    // modified wishbone architecure so that the core can be reused without
    // editing.  Remove the ext_ad/di/do portions.
    output [ 3 : 0] o_bus_wrstrobe,   // byte-by-byte strobes
    output [31 : 0] o_bus_wrdata,     // the write data, aligned for hw
    input  [31 : 0] i_bus_rddata,     // the read data, aligned for hw

    // legacy signals, to be refactored
    output          uart_tx,          // UART TX (message from RISC-V)
    output          uart1_tx,         // UART TX1 (message from RISC-V)
    input           uart_rx,          // UART RX (message from PC    )
    input           uart1_rx,         // UART RX1 (message from PC)
    inout  [GPIO_WIDTH-1 : 0] gpio_pin,
    inout           i2c_scl_pin,
    inout           i2c_sda_pin,
    inout           spi_sck_pin,
    inout           spi_mosi_pin,
    inout           spi_miso_pin,
    output          fifo1_empty,
    output          fifo_empty,
    output [31 : 0] inst_raddr,
    output [31 : 0] inst_rdata_d0,   // debug
    output [31 : 0] inst_rdata_d1,   // debug
    output          peri_uart0_wr,
    output          peri_uart1_wr,
    output          peri_mic_en,
    output [ 3 : 0] peri_mic_addr,
    input           ext_wr,           // External interface - API I/F w/ outside
    input           ext_rd,           //
    input  [ 7 : 0] ext_ad,           //
    input  [ 7 : 0] ext_di,           //
    output [ 7 : 0] ext_do            //
);
    //================================================================================
    // Internal signals
    //================================================================================
    wire   [31 : 0] display_data_to_cpu;
    wire   [31 : 0] peri_data_to_cpu;
    wire   [31 : 0] cpu_addr;
    wire   [31 : 0] inst_raddr;          // 64KB (2^16)
    wire   [31 : 0] inst_rdata;
    wire   [12 : 0] drom_addr;           //  4KB (2^12) - 8KB (2^13)
    wire   [31 : 0] drom_rdata;
    wire   [ 2 : 0] drom_wsize;
    wire            drom_we;
    wire   [31 : 0] drom_wdata;
    wire   [14 : 0] dram_addr;           // 32KB (2^15)
    wire   [31 : 0] dram_rdata;
    wire   [31 : 0] dram_wdata;
    wire            dram_wr_en;
    wire   [ 2 : 0] dram_wsize;
    wire   [ 2 : 0] cpu_funct3;
    wire   [31 : 0] cpu_wdata;
    wire            cpu_wr_en;
    wire   [14 : 0] extif_addr_to_dram;  // Arbitration is done manually by writing one bit register
    wire   [31 : 0] extif_wdata_to_dram; // [0] - DRAM arbitration, [1] - IROM arbitration
    wire            extif_wr_en;
    wire   [ 2 : 0] extif_funct3;
    wire   [31 : 0] cpu_rdata;
    wire            drom_en, dram_en;
    wire            display_en;
    wire            peri_sel;
    wire            peri_uart0_en;
    wire            peri_uart1_en;
    wire            peri_i2c_en;
    wire            peri_gpio_en;
    wire            peri_spi_en;
    wire            peri_counter_en;
    wire            peri_uart0_rd;
    wire            peri_uart0_stat_rd;
    wire            peri_uart1_rd;
    wire            peri_uart1_stat_rd;
    wire            peri_gpio_i2c_rd;
    wire            peri_gpio_i2c_wr;
    wire            peri_gpio_rd;
    wire            peri_gpio_wr;
    wire            peri_gpio_spi_rd;
    wire            peri_gpio_spi_wr;
    wire            peri_counter_rd;
    wire            peri_counter_wr;
    reg             drom_en_d;
    reg             peri_sel_d;
    reg             display_en_d;
    reg    [31 : 0] peri_data_to_cpu_d;
    wire            cpu_rst;
    wire   [ 7 : 0] uart_do;
    wire            peri_uart0_wr;
    wire            peri_uart0_cfg_wr;
    wire            peri_uart1_wr;
    wire            peri_uart1_cfg_wr;
    wire   [ 7 : 0] uart_di;
    wire            uart_rdy;
    wire   [ 7 : 0] uart1_di;
    wire            uart1_rdy;
    wire            fifo_full;
    wire            fifo1_full;
    wire            i2c_scl_in;
    wire            i2c_sda_in;
    wire   [31 : 0] peri_uart0_stat_in;
    wire   [31 : 0] peri_uart0_rd_data;
    wire   [31 : 0] peri_uart1_stat_in;
    wire   [31 : 0] peri_uart1_rd_data;


    //================================================================================
    //================================================================================
    reg [31 : 0] inst_raddr_d;

    //================================================================================
    //================================================================================
    //================================================================================
    initial begin
        $display ("--------------------------------------------");
        $display ("- MCCI Catena RISC-V32 Version : v0.6T     -");
        $display ("- RISC-V32 Architecture : I                -");
        $display ("--------------------------------------------");
    end

    //================================================================================
    // Memory access
    // - DROM and DRAM requires one cycle for RD access
    //================================================================================
    assign drom_en    = (~|cpu_addr[31:17] && cpu_addr[16:15] == 2'b10);
    assign dram_en    = (~|cpu_addr[31:17] && cpu_addr[16:15] == 2'b11);
    assign display_en = (cpu_addr[31:28] == 4'b1000 && ~|cpu_addr[27:12]);

    assign dram_addr  = cpu_addr[14:0];
    assign dram_wdata = cpu_wdata;
    assign dram_wr_en = (dram_en & cpu_wr_en);
    assign dram_wsize = cpu_funct3;
    assign drom_addr  = cpu_addr[12:0];
    assign drom_wsize = cpu_funct3;

    always @(posedge clk_i or posedge cpu_rst)
        if(cpu_rst) begin
            drom_en_d  <= 1'b0;
            display_en_d <= 1'b0;
            peri_sel_d <= 1'b0;
            peri_data_to_cpu_d <= 32'h0;
        end else begin
            drom_en_d  <= drom_en;
            display_en_d  <= display_en;
            peri_sel_d <= peri_sel;
            peri_data_to_cpu_d <= peri_data_to_cpu;
        end

    assign cpu_rdata =
        display_en_d ? display_data_to_cpu :
        peri_sel_d   ? peri_data_to_cpu_d  :
        drom_en_d    ? drom_rdata          :
                       dram_rdata;

    //================================================================================
    // CPU
    // - Reset CPU when IROM is loaded from outside (SCE)
    // - Prevent CPU reset in HCE
    //================================================================================
    generate
    if(HARDENED_HCE) begin: cpu_rst_off
        assign cpu_rst = rst_i;
    end else begin: cpu_rst_on
        assign cpu_rst = /*rst_i |*/ rom_ext_acc;
    end
    endgenerate

    cpu #(.PC_RESET_ADDR(PC_RESET_ADDR), .HAS_MULT(HAS_MULT), .UNSIGNED_MULT(UNSIGNED_MULT))
    cpu_inst(
        .clk_i        (clk_i     ), // Input Clock
        .rst_i        (cpu_rst   ), // Input Reset
        .inst_rdata_i (inst_rdata), // Input data from Instruction unit
        .data_rdata_i (cpu_rdata ), // Input data from memory unit
        .ext_stall_i  (1'b0      ), // Stall Signal from Accelerator
        .inst_raddr_o (inst_raddr), // Read Address to Instruction Memory Unit
        .funct3_o     (cpu_funct3), // RISC-V funct3 to Data Memory Unit
        .data_addr_o  (cpu_addr  ), // Read Address to Data Memory Unit
        .data_wdata_o (cpu_wdata ), // Write Data to Data Memory Unit
        .data_wr_en_o (cpu_wr_en )  // Write Enable to Data Memory Unit
    );

    //================================================================================
    // I-ROM (aka. PROM); 64KB [15:0]
    // - Mapped into ROM in HCE. No need for loading
    // - Mapped into SPSRAM in SCE. Need to load content from outside
    //================================================================================
    inst_unit #(.INIT_FILE_NAME(INST_FILE_NAME))
    inst_unit_inst(
        .clk_i       (clk_i              ),
        .prom_extacc (rom_ext_acc       ), // External loading
        .bw_valid_i  (prom_ext_wr_en     ),
        .bw_data_i   (rom_ext_wr_data   ), // External loading
        .raddr_i     (inst_raddr[15:0]   ),
        .probe       (inst_rdata_d0      ),
        .rdata_o     (inst_rdata         )
    );

    //================================================================================
    // D-ROM
    //================================================================================
    drom_unit #(.INIT_FILE_NAME(DROM_FILE_NAME),
                    .ADDR_DEPTH    (256*8         ),
                    .ADDR_WIDTH    (11            ),
                    .CONFIG_LOAD   (0             ))
        data_rom_unit_inst(
            .clk_i    (clk_i          ),
            .funct3_i (drom_wsize     ),
            .raddr_i  (drom_addr[12:0]),
            .ext_acc  (rom_ext_acc    ),
            .we       (drom_ext_wr_en ),
            .din      (rom_ext_wr_data),
            .probe    (inst_rdata_d1  ),
            .rdata_o  (drom_rdata     )
        );


    //================================================================================
    // D-RAM; 32KB [14:0]
    // - No need to loading
    // - Mapped into SRAM in HCE
    // - Mapped into SPSRAM in SCE
    //================================================================================
    data_unit #(.INIT_FILE_NAME(DRAM_FILE_NAME))
    data_ram_unit_inst(
        .clk_i       (clk_i     ),
        .rst_i       (cpu_rst   ),
        .funct3_i    (dram_wsize),
        .addr_i      (dram_addr ),
        .wdata_i     (dram_wdata),
        .wr_en_i     (dram_wr_en),
        .rdata_o     (dram_rdata)
    );

    //================================================================================
    // Peripheral address decoding
    //================================================================================

    // Peripheral addresses
    assign peri_sel         = (cpu_addr[31:28] == 4'b1000 && ~|cpu_addr[27:8]);        // 0x800000xx
    assign peri_uart0_en    = peri_sel && (cpu_addr[7:4] == 4'b0000);                  // 0x8000000x
    assign peri_uart1_en    = peri_sel && (cpu_addr[7:4] == 4'b0001);                  // 0x8000001x
    assign peri_i2c_en      = peri_sel && (cpu_addr[7:4] == 4'b0010);                  // 0x8000002x
    assign peri_gpio_en     = peri_sel && (cpu_addr[7:4] == 4'b0011);                  // 0x8000003x
    assign peri_spi_en      = peri_sel && (cpu_addr[7:4] == 4'b0100);                  // 0x8000004x
    assign peri_counter_en  = peri_sel && (cpu_addr[7:4] == 4'b0101);                  // 0x8000005x
    assign peri_mic_en      = peri_sel && (cpu_addr[7:4] == 4'b0111);                  // 0x8000006x

    // UART0 registers
    assign peri_uart0_rd        = peri_uart0_en && (cpu_addr[3:2] == 2'b00);           // 0x80000000
    assign peri_uart0_wr        = peri_uart0_en && (cpu_addr[3:2] == 2'b00) && cpu_wr_en;
    assign peri_uart0_stat_rd   = peri_uart0_en && (cpu_addr[3:2] == 2'b01);           // 0x80000004
    assign peri_uart0_cfg_wr    = peri_uart0_en && (cpu_addr[3:2] == 2'b01) && cpu_wr_en;
    assign peri_uart0_rd_data   = {24'b0, uart_di};
    assign peri_uart0_stat_in   = {24'b0, uart_rx, 5'b0, uart_rdy, fifo_full};

    // UART1 registers
    assign peri_uart1_rd        = peri_uart1_en && (cpu_addr[3:2] == 2'b00);           // 0x80000010
    assign peri_uart1_wr        = peri_uart1_en && (cpu_addr[3:2] == 2'b00) && cpu_wr_en;
    assign peri_uart1_stat_rd   = peri_uart1_en && (cpu_addr[3:2] == 2'b01);           // 0x80000014
    assign peri_uart1_cfg_wr    = peri_uart1_en && (cpu_addr[3:2] == 2'b01) && cpu_wr_en;
    assign peri_uart1_rd_data   = {24'b0, uart1_di};
    assign peri_uart1_stat_in   = {24'b0, uart1_rx, 5'b0, uart1_rdy, fifo1_full};

    // GPIO_I2C registers
    assign peri_gpio_i2c_rd     = peri_i2c_en && (cpu_addr[3:2] == 2'b00);             // 0x80000020
    assign peri_gpio_i2c_wr     = peri_i2c_en && (cpu_addr[3:2] == 2'b00) && cpu_wr_en;
    wire [31:0] peri_gpio_i2c_in;
    assign peri_gpio_i2c_in     = gpio_i2c_rd_data_reg;

    // GPIO registers
    assign peri_gpio_rd         = peri_gpio_en && (cpu_addr[3:2] == 2'b00);         // 0x80000030
    assign peri_gpio_wr         = peri_gpio_en && (cpu_addr[3:2] == 2'b00) && cpu_wr_en;
    wire [31:0] peri_gpio_in;
    assign peri_gpio_in         = gpio_rd_data_reg;

    // GPIO_SPI registers
    assign peri_gpio_spi_rd     = peri_spi_en && (cpu_addr[3:2] == 2'b00);          // 0x80000040
    assign peri_gpio_spi_wr     = peri_spi_en && (cpu_addr[3:2] == 2'b00) && cpu_wr_en;
    wire [31:0] peri_gpio_spi_in;
    assign peri_gpio_spi_in     = gpio_spi_rd_data_reg;

    // Counter register
    assign peri_counter_rd      = peri_counter_en && (cpu_addr[3:2] == 2'b00);     // 0x80000050
    assign peri_counter_wr      = peri_counter_en && (cpu_addr[3:2] == 2'b00) && cpu_wr_en;
    wire [31:0] peri_counter_in;
    assign peri_counter_in      = counter_data_reg;

    // Mic register addresses TODO(tmm@mcci.com) this should go to top level.
    assign peri_mic_addr[3:0]   = cpu_addr[3:0];

    //================================================================================
    // Peripheral Data Alignment
    // TODO(tmm@mcci.com) centralize this at CPU
    //================================================================================

    wire [31:0] bus_rddata_align;
    wire [31:0] bus_rddata;

    assign bus_rddata_align = cpu_addr[1:0] == 2'b00 ? i_bus_rddata
                            : cpu_addr[1:0] == 2'b01 ? {     8'h00, i_bus_rddata[31:8]}
                            : cpu_addr[1:0] == 2'b10 ? {  16'h0000, i_bus_rddata[31:16]}
                            :               /* 2'b11 */{24'h000000, i_bus_rddata[31:24]}
                            ;

    assign bus_rddata = cpu_funct3[1:0] == `BYTE ? { cpu_funct3[2] ? 24'b0 : {24{bus_rddata_align[7]}},  bus_rddata_align[7:0] }
                      : cpu_funct3[1:0] == `HALF ? { cpu_funct3[2] ? 16'b0 : {16{bus_rddata_align[15]}}, bus_rddata_align[15:0] }
                      :                            { bus_rddata_align }
                      ;

    assign o_bus_wrstrobe = (cpu_funct3[1:0] == `BYTE) ?
                                   (cpu_addr[1:0] == 2'b00 ? { 3'b000, cpu_wr_en } :
                                    cpu_addr[1:0] == 2'b01 ? { 2'b00, cpu_wr_en, 1'b0 } :
                                    cpu_addr[1:0] == 2'b10 ? { 1'b0, cpu_wr_en, 2'b00 }
                                                           : { cpu_wr_en, 3'b000 })
                          : (cpu_funct3[1:0] == `HALF) ?
                                    (cpu_addr[1:0] == 2'b00 ? {2'b00, {2{cpu_wr_en}}} :
                                     cpu_addr[1:0] == 2'b10 ? {{2{cpu_wr_en}}, 2'b00}
                                                            : 4'b0000)
                          : (cpu_funct3[1:0] == `WORD) ?      {4{cpu_wr_en}}
                          : /* otherwise */                   4'b0000
                          ;

    assign o_bus_wrdata = cpu_addr[1:0] == 2'b00 ? cpu_wdata
                        : cpu_addr[1:0] == 2'b01 ? { cpu_wdata[23:0], 8'h00 }
                        : cpu_addr[1:0] == 2'b10 ? { cpu_wdata[15:0], 16'h0000 }
                        :                          { cpu_wdata[7:0],  24'h000000 }
                        ;

    //================================================================================
    // Peripheral Read Data
    // TODO(tmm@mcci.com) move this up a level.
    //================================================================================

    assign peri_data_to_cpu =
        peri_uart0_rd       ? peri_uart0_rd_data :
        peri_uart0_stat_rd  ? peri_uart0_stat_in :
        peri_uart1_rd       ? peri_uart1_rd_data :
        peri_uart1_stat_rd  ? peri_uart1_stat_in :
        peri_gpio_i2c_rd    ? peri_gpio_i2c_in :
        peri_gpio_rd        ? peri_gpio_in :
        peri_gpio_spi_rd    ? peri_gpio_spi_in :
        peri_counter_rd     ? peri_counter_in :
        peri_mic_en         ? bus_rddata :
                              32'b0;

    //================================================================================
    // UART 8b data to UART controller
    //================================================================================
    assign uart_do     = cpu_wdata[7:0];

    generate
    if(INC_UART_CON) begin: uart_con_on
        // UART TX controller
        simple_uart
        simple_uart_u0(
            .clk      (clk_i      ), // i         data interface clock
            .reset    (cpu_rst    ), // i
            .i_period (UART_PERIOD[10:0]), // i [10:0]; baud rate
            .i_din    (uart_do    ), // i [ 7:0]
            .i_valid  (peri_uart0_wr), // i       write strobe
            .fifo_empty(fifo_empty),
            .fifo_full (fifo_full),
            .o_txd    (uart_tx    )  // o         to pad
        );
        // UART RX controller
        simple_uart_rx
        simple_uart_u1(
            .clk      (clk_i      ), // i         data interface clock
            .reset    (cpu_rst    ), // i
            .i_period (UART_PERIOD[10:0]), // i [10:0]; baud rate
            .i_rxd    (uart_rx    ), // i         from pad
            .cfg_d    (cpu_wdata[0]),
            .cfg_accept (cpu_wdata[1]),
            .cfg_wr   (peri_uart0_cfg_wr),
            .o_din    (uart_di),     // o [ 7:0]
            .o_valid  (uart_rdy)     // o          data is available
        );
        // UART1 TX controller
        simple_uart
        simple_uart_u2(
            .clk      (clk_i      ), // i         data interface clock
            .reset    (cpu_rst    ), // i
            .i_period (UART1_PERIOD[10:0]), // i [10:0]; baud rate
            .i_din    (uart_do    ), // i [ 7:0]
            .i_valid  (peri_uart1_wr),  // i
            .fifo_empty(fifo1_empty),
            .fifo_full (fifo1_full),
            .o_txd    (uart1_tx    )  // o       to pad
        );
        // UART1 RX controller
        simple_uart_rx
        simple_uart_u3(
            .clk      (clk_i      ), // i         data interface clock
            .reset    (cpu_rst    ), // i
            .i_period (UART1_PERIOD[10:0]), // i [10:0]; baud rate
            .i_rxd    (uart1_rx    ), // i       from pad
            .cfg_d    (cpu_wdata[0]),
            .cfg_accept (cpu_wdata[1]),
            .cfg_wr   (peri_uart1_cfg_wr),
            .o_din    (uart1_di),    // o [ 7:0] data from register
            .o_valid  (uart1_rdy)    // o        valid data avialable
        );
    end else begin: uart_con_off
        assign uart_tx = 1'b0;
        assign uart1_tx = 1'b0;
    end
    endgenerate

    //================================================================================
    // GPIOs
    //================================================================================
    wire [31:0] gpio_dir_out;
    wire [31:0] gpio_open_drain;
    wire [31:0] gpio_rd_data;
    wire [31:0] gpio_wr_data;

    reg [31:0] gpio_dir_out_reg; // 1=output; 0=input
    reg [31:0] gpio_open_drain_reg; // 1=open drain, 0=buffered
    reg [31:0] gpio_rd_data_reg;
    reg [31:0] gpio_wr_data_reg;

    // generate the assignments to set the pins.
    genvar i;
    generate
        for (i = 0; i < GPIO_WIDTH; i = i + 1)
            begin: gpio_bit
                assign gpio_pin[i] = gpio_dir_out_reg[i] ? (gpio_wr_data_reg[i] ? (gpio_open_drain_reg[i] ? 1'bZ : 1'b1) : 1'b0) : 1'bZ;
            end
    endgenerate

    // !!! hardcoded for now; make programmable later
    assign gpio_dir_out = 32'h03; // set [1:0] as outputs (rest are inputs)
    //assign gpio_dir_out = (peri_gpio_dir_wr) ? cpu_wdata : gpio_dir_out_reg;
    // !!! hardcoded for now; make programmable later
    assign gpio_open_drain = 32'h02; // set gpio[1] as open drain (rest are buffered)
    //assign gpio_open_drain = (peri_gpio_open_drain_wr) ? cpu_wdata : gpio_open_drain_reg;

    // zero pad gpio pin data on the left to make a 32-bit value using gpio_zero.
    generate
        for (i = 0; i < 32; i = i + 1)
            begin: gpio_rd_bit
                if (i < GPIO_WIDTH)
                    assign gpio_rd_data[i] = gpio_pin[i];
                else
                    assign gpio_rd_data[i] = 1'b0;
            end
    endgenerate

    assign gpio_wr_data = (peri_gpio_wr) ? cpu_wdata : gpio_wr_data_reg;

    always @(posedge clk_i or posedge cpu_rst) begin
        if (cpu_rst) begin
            gpio_dir_out_reg <= 32'h0;
            gpio_open_drain_reg <= 32'h0;
                                    //    ---reset values for gpio---
                                    //
                                    //     4    3     2     1      0
            gpio_rd_data_reg <= 32'h0;
            gpio_wr_data_reg <= {27'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0}; // 32'h0;
        end else begin
            gpio_dir_out_reg <= gpio_dir_out;
            gpio_open_drain_reg <= gpio_open_drain;
            gpio_rd_data_reg <= gpio_rd_data;
            gpio_wr_data_reg <= gpio_wr_data;
        end
    end

    //================================================================================
    // I2C GPIOs
    //================================================================================
    wire [31:0] gpio_i2c_dir_out;
    wire [31:0] gpio_i2c_rd_data;
    wire [31:0] gpio_i2c_wr_data;

    reg [31:0] gpio_i2c_rd_data_reg;
    reg [31:0] gpio_i2c_wr_data_reg;

    assign i2c_scl_pin = gpio_i2c_wr_data_reg[0] ? 1'bZ : 1'b0;
    assign i2c_sda_pin = gpio_i2c_wr_data_reg[1] ? 1'bZ : 1'b0;

    assign gpio_i2c_rd_data = {30'b0, i2c_sda_pin, i2c_scl_pin};
    assign gpio_i2c_wr_data = (peri_gpio_i2c_wr) ? cpu_wdata : gpio_i2c_wr_data_reg;

   always @(posedge clk_i or posedge cpu_rst) begin
      if (cpu_rst) begin
         gpio_i2c_rd_data_reg <= 32'h0;
         gpio_i2c_wr_data_reg <= 32'h3;
      end else begin
         gpio_i2c_rd_data_reg <= gpio_i2c_rd_data;
         gpio_i2c_wr_data_reg <= gpio_i2c_wr_data;
      end
   end

    //================================================================================
    // SPI GPIOs
    //================================================================================
    wire [31:0] gpio_spi_dir_out;
    wire [31:0] gpio_spi_rd_data;
    wire [31:0] gpio_spi_wr_data;

    reg [31:0] gpio_spi_dir_out_reg; // 1=output; 0=input
    reg [31:0] gpio_spi_rd_data_reg;
    reg [31:0] gpio_spi_wr_data_reg;

    assign spi_sck_pin  = gpio_spi_dir_out_reg[2] ? gpio_spi_wr_data_reg[2] : 1'bZ;
    assign spi_mosi_pin = gpio_spi_dir_out_reg[1] ? gpio_spi_wr_data_reg[1] : 1'bZ;
    assign spi_miso_pin = gpio_spi_dir_out_reg[0] ? gpio_spi_wr_data_reg[0] : 1'bZ;

    assign gpio_spi_dir_out = 32'hFFFFFFFE; // !!! hardcoded as outputs except for miso (for now); make programmable later
    //assign gpio_spi_dir_out = (peri_gpio_spi_dir_wr) ? cpu_wdata : gpio_spi_dir_out_reg;
    assign gpio_spi_rd_data = {29'b0, spi_sck_pin, spi_mosi_pin, spi_miso_pin};
    assign gpio_spi_wr_data = (peri_gpio_spi_wr) ? cpu_wdata : gpio_spi_wr_data_reg;

    always @(posedge clk_i or posedge cpu_rst) begin
        if (cpu_rst) begin
            gpio_spi_dir_out_reg <= 32'h0;
            gpio_spi_rd_data_reg <= 32'h0;
            gpio_spi_wr_data_reg <= 32'h0;
        end else begin
            gpio_spi_dir_out_reg <= gpio_spi_dir_out;
            gpio_spi_rd_data_reg <= gpio_spi_rd_data;
            gpio_spi_wr_data_reg <= gpio_spi_wr_data;
        end
    end

    //================================================================================
    // Counter
    //================================================================================
    reg [31:0] counter_data_reg;
    wire [31:0] counter_data;

    assign counter_data = (peri_counter_wr) ? cpu_wdata : (counter_data_reg + 1);

    always @(posedge clk_i or posedge cpu_rst) begin
        if (cpu_rst) begin
            counter_data_reg <= 32'h0;
        end else begin
            counter_data_reg <= counter_data;
        end
    end

    //================================================================================
    // Display (just for simulation)
    //================================================================================
    // synthesis translate_off
    // synopsys  translate_off
    // translate_off
    display_unit
    display_unit_inst(
        .clk_i      (clk_i                 ),
        .rst_i      (cpu_rst               ),
        .addr_i     (cpu_addr[3:0]         ),
        .wdata_cpu_i(cpu_wdata             ),
        .wr_en_i    (display_en & cpu_wr_en),
//      .wr_en_i    (peri_sel & cpu_wr_en),
        .stall_o    (                      ),
        .data_cpu_o (                      )
    );
    // translate_on
    // synopsys  translate_on
    // synthesis translate_on

endmodule
