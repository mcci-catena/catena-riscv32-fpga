/*

Module: display_unit.v

Function:
    Display Unit for RISC-V CPU

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


/////////////////////////////////////////////////////
// Module interface
/////////////////////////////////////////////////////

module display_unit
    (
        //Input
        input clk_i,                    //Input Clock
        input rst_i,                    //Reset Signal

        input [3:0]  addr_i,            //RISC-V Address of register
        input [31:0] wdata_cpu_i,       //Write Data from cpu
        input wr_en_i,                  //Write Enable

        output stall_o,                     //Output stall to cpu
        output [31:0] data_cpu_o            //Output data to cpu
    );

    reg stall_r;

    /* Debug Unit Register Map
        | Name      | Address   | R/W Access    | Register Function |
        | timel_r   | 0b0000    | Read Only     | Lower 32-bit of the timer register
        | timeh_r   | 0b0001    | Read Only     | Upper 32-bit of the timer register
        | csr_r     | 0b0010    | Read/Write    | Control/Status Register
        | buf_r     | 0b0011    | Read/Write    | A 128 bytes deep FIFO to store the display characters
    */

    wire [31:0] timel_r, timeh_r;
    reg [31:0] csr_r, csr_n;
    //Bit 0 - Clear the Buffer Register
    //Bit 1 - Display the Buffer Register in the Simulation I/O
    //Bit 2 - Open the file using the file name inside Buffer Register
    //Bit 3 - Read the opened file value into the Buffer
    //Bit 4 - Write the Buffer Value to the opened file
    //Bit 5 - Close the opened file
    reg [128*8-1:0] buf_r, buf_n;

    /* Timer Logic */
    reg [63:0] time_r, time_n;

    always @ (posedge clk_i or posedge rst_i) begin
        if(rst_i) begin
            time_r <= 0;
        end else begin
            time_r <= time_r + 1;
        end
    end

    assign timel_r = time_r[31:0];
    assign timeh_r = time_r[63:32];

    /* Read Logic */
    reg [1:0] addr_r;

    always @ (posedge clk_i or posedge rst_i) begin
        if(rst_i) begin
            addr_r <= 0;
        end else begin
            addr_r <= addr_i[3:2];
        end
    end

    assign data_cpu_o = (addr_r == 2'h0) ? timel_r :
                        (addr_r == 2'h1) ? timeh_r :
                        (addr_r == 2'h2) ? csr_r :
                        (addr_r == 2'h3) ? buf_r :
                        32'h0;

    /* Write Logic */
    always @ ( * ) begin
        csr_n = csr_r;
        buf_n = buf_r;
        if (wr_en_i) begin
            case (addr_i)
                4'h8: begin
                    csr_n = wdata_cpu_i;
                    buf_n = (wdata_cpu_i[0]) ? 0 : buf_r;
                end
                4'hc: begin
                    buf_n = {buf_r[127*8-1:0], wdata_cpu_i[7:0]};
                end
                default: begin
                    //$display ("@%0dns : Illegal Memory Access in Display Unit at address : 0x%x", $time, addr_i);
                end
            endcase
        end
    end

    always @ (posedge clk_i or posedge rst_i) begin
        if(rst_i) begin
            csr_r <= 0;
            buf_r <= 0;
        end else begin
            csr_r <= csr_n;
            buf_r <= buf_n;
        end
    end

    assign stall_o = 0;

    reg [128*8-1:0] buf_s;
    integer file;
    //Bit 0 - Clear the Buffer Register
    //Bit 1 - Display the Buffer Register in the Simulation I/O
    //Bit 2 - Open the file using the file name inside Buffer Register
    //Bit 3 - Read the opened file value into the Buffer
    //Bit 4 - Write the Buffer Value to the opened file
    //Bit 5 - Close the opened file
`ifndef VERILATOR
    //debug_printf
    initial forever begin
        @(posedge csr_r[1]);
        buf_s = buf_r;
                if(buf_s!=128'h0) begin
            while (buf_s[128*8-1:127*8] == 8'h0) buf_s = {buf_s[127*8-1:0], buf_s[128*8-1:127*8]};
            $display ("debug_printf: %s", buf_s);
                end
    end
`endif
    //debug_fopen
    /*
    initial forever begin
        @(posedge csr_r[2]);
        buf_s = buf_r;
        while (buf_s[128*8-1:127*8] == 8'h0) buf_s = {buf_s[127*8-1:0], buf_s[128*8-1:127*8]};
        file = $fopen (buf_s, "w");
        if (file == 0) begin
            $display("File cannot opened : %s", buf_s);
            $finish;
        end else begin
            $display ("File is opened : %s", buf_s);
        end
    end
    */

    //debug_fread - NOT IMPLEMENTED
    /*initial forever begin
        @(posedge csr_r[3]);
        $fscanf(file, "%d\n", buf_r);
    end*/

    //debug_fwrite
    /*
    initial forever begin
        @(posedge csr_r[4]);
        buf_s = buf_r;
        while (buf_s[128*8-1:127*8] == 8'h0) buf_s = {buf_s[127*8-1:0], buf_s[128*8-1:127*8]};
        $fwrite (file, "%s", buf_s);
    end
    */
endmodule
