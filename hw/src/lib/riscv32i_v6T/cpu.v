/*

Module: cpu.v

Function:
    CPU for RISC-V CPU

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
//`define debug_sim

module cpu
    #(
        parameter PC_RESET_ADDR     = 32'h0000_0000, //Program Counter Reset Address
                  HAS_MULT              = 0,
              UNSIGNED_MULT         = 0
    )(
        /************Input Port************/
        //Clock and Reset
        input clk_i,                                //Input Clock
        input rst_i,                                //Input Reset

        //Input from instruction Memory Unit
        input [31:0] inst_rdata_i,                  //Input data from instruction unit

        //Input from Data Memory Unit
        input [31:0] data_rdata_i,                  //Input data from memory unit

        input ext_stall_i,                          //Store Signal from Accelerator
        /************Output Port************/
        //Input from instruction Memory Unit
        output [31:0] inst_raddr_o,                 //Read Address to Data Memory Unit

        //Output to Data Memory Unit
        output [2:0] funct3_o,                      //RISC-V EX stage funct3 to Data Memory Unit
        output [31:0] data_addr_o,                  //Read Address to Data Memory Unit
        output [31:0] data_wdata_o,                 //Write Data to Data Memory Unit
        output data_wr_en_o                         //Write Enable to Data Memory Unit
    );

/////////////////////////////////////////////////////
// Wires and Register
/////////////////////////////////////////////////////
    wire stall = 1'b0;                      //for now no stalling is needed

    //************Instruction Fetch************
    reg[31:0]   f_pc;                      //pipeline register that holds the current PC
    wire[31:0]  f_instr;                   //instruction in IF stage, is pipelined to next stage
    wire[31:0]  f_next_pc;                 //Next program counter in IF stage

    //************Execution************
    reg[31:0]   x_instr;                   //pipeline register holding instruction in EX stage
    reg[31:0]   x_pc;                      //pipeline register holding PC where EX_instr was read

    wire[31:0]  x_alu_out;                 //output of ALU
    wire[31:0]  x_rs1_in;                  //rs1 value from register file
    wire[31:0]  x_rs2_in;                  //rs2 value from register file
    wire[31:0]  x_rs1_fwd;                 //rs1 value chosen between register output and a forwarded value
    wire[31:0]  x_rs2_fwd;                 //rs2 value chosen between register output and a forwarded value
    reg[31:0]   x_rs1;                     //final rs1 value going into ALU
    reg[31:0]   x_rs2;                     //final rs2 value going into ALU
    wire        x_r_type;                  //this is an R-type instruction
    wire        x_i_type;                  //this is an I-type instruction

    wire        x_IMM_SIGN;                //first bit of immediate
    wire[31:0]  x_I_TYPE_IMM;              //immediate formed from an I type instruction
    wire[31:0]  x_S_TYPE_IMM;              //immediate formed from an S type instruction
    wire[31:0]  x_SB_TYPE_IMM;             //immediate formed from an SB type instruction
    wire[31:0]  x_U_TYPE_IMM;              //immediate formed from an U type instruction
    wire[31:0]  x_UJ_TYPE_IMM;             //immediate formed from an UJ type instruction
    wire[31:0]  x_jumped_pc;               //output of the branch calculation module

    wire        x_br_eq;                   //output of ALU that says if rs1 == rs2
    wire        x_br_lt;                   //output of ALU that says if rs1 < rs2
    wire        x_br_ltu;                  //output of ALU that says if rs1 < rs2 (unsigned)
    reg         x_br_valid;                //correct branch control bit based on instruction and ALU outputs
    wire        x_no_op;                   //no-op for jumps and branches

    //************WriteBack************
    reg[31:0]   w_alu_out;                 //pipeline register holding output of ALU from previous stage
    reg[11:0]   w_instr;                   //pipeline register holding instruction in WB stage .. bits 31:12 not used

    wire[31:0]  w_reg_wdata;               //value being written back to register/forwarded
    wire        w_reg_wr_en;               //write enable bit of register file

    // ----- Initialize the Register for Simulation Purpose -----
    initial begin
        x_rs1 = 0;
        x_rs2 = 0;
        x_br_valid = 0;
    end
/////////////////////////////////////////////////////
// RISC-V32I Register File Module
/////////////////////////////////////////////////////
    reg_file reg_file_inst(
        .clk_i      (clk_i),
        .rst_i      (rst_i),
        .raddr_a_i  (f_instr`rs1),
        .raddr_b_i  (f_instr`rs2),
        .waddr_i    (w_instr`rd),
        .wdata_i    (w_reg_wdata),
        .wr_en_i    (w_reg_wr_en),
        .rdata_a_o  (x_rs1_in),
        .rdata_b_o  (x_rs2_in)
    );


`ifdef debug_sim

always @ (negedge clk_i)
begin
//   if (w_instr`rd ==5'h01)
//   $display ("    ADDR1  %h", w_reg_wdata);
//   if (w_instr`rd ==5'h02)
//   $display ("    ADDR2  %h", w_reg_wdata);
//   if (w_instr`rd ==5'h03)
//   $display ("    ADDR3  %h", w_reg_wdata);
   if (w_reg_wr_en)
   $display ("    ADDR DATA  %d, %d", w_instr`rd, w_reg_wdata);
end
`endif

/////////////////////////////////////////////////////
// RISC-V32I ALU Module
/////////////////////////////////////////////////////
    alu #(.has_mult(HAS_MULT), .unsigned_mult(UNSIGNED_MULT)) alu_inst(
        .data_a_i   (x_rs1),
        .data_b_i   (x_rs2),
        .funct3_i   (x_instr`funct3),
        .funct7_i   (x_instr`funct7),
        .r_type_i   (x_r_type),
        .i_type_i   (x_i_type),
        .result_o   (x_alu_out),
        /* verilator lint_off PINCONNECTEMPTY */
        .underflow_o    (),
        .overflow_o (),
        /* verilator lint_on PINCONNECTEMPTY */
        .br_eq_o    (x_br_eq),
        .br_lt_o    (x_br_lt),
        .br_ltu_o   (x_br_ltu)
    );

/////////////////////////////////////////////////////
// RISC-V32I Branch Calculation Module
/////////////////////////////////////////////////////
    pc_calc pc_calc_inst(
        .opcode_i   (x_instr`opcode),
        .jtype_i    (x_UJ_TYPE_IMM),
        .btype_i    (x_SB_TYPE_IMM),
        .itype_i    (x_I_TYPE_IMM),
        .rs1_i      (x_rs1_fwd),
        .f_pc_i     (f_pc),
        .x_pc_i     (x_pc),
        .x_br_i     (x_br_valid),
        .pc_calc_o  (x_jumped_pc)
    );

/////////////////////////////////////////////////////
//Instruction Fetch Stage (IF)
/////////////////////////////////////////////////////

    //pipeline registers
    always @ (posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            f_pc <= PC_RESET_ADDR - 4;                      //PC Reset Address
        end else begin
            f_pc <= f_next_pc;
        end
    end

    //assign f_next_pc = stall ? f_pc : x_jumped_pc;            //Next PC - Either stall it from the pc calculation block
    //assign f_instr = x_no_op ? `nop : inst_rdata_i;           //Next instruction to be fetched from instruction memory

    assign f_next_pc = ext_stall_i ? f_pc : x_jumped_pc;            //Next PC - Either stall it from the pc calculation block
    assign f_instr = x_no_op | ext_stall_i ? `nop : inst_rdata_i;   //Next instruction to be fetched from instruction memory


    //Output from CPU
    assign inst_raddr_o = f_next_pc;                        //Next PC goes into the instruction memory

/////////////////////////////////////////////////////
//Execution Stage (EX)
/////////////////////////////////////////////////////

    //pipeline registers
    always @ (posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            x_instr <= 32'b0;
            x_pc <= `nop;
        end else if (~stall) begin
            x_instr <= f_instr;
            x_pc <= f_pc;
        end
    end

    assign x_r_type = x_instr`opcode == `R_TYPE;            //Set to 1 if the instruction is a R-type
    assign x_i_type = x_instr`opcode == `I_TYPE;            //Set to 1 if the instruction is an I-type

    //Sign extend immediate
    assign x_IMM_SIGN =     x_instr[31];
    assign x_I_TYPE_IMM =   {{21{x_IMM_SIGN}},x_instr[30:20]};
    assign x_S_TYPE_IMM =   {{21{x_IMM_SIGN}},x_instr[30:25],x_instr[11:7]};
    assign x_SB_TYPE_IMM =  {{20{x_IMM_SIGN}},x_instr[7],x_instr[30:25],x_instr[11:8],1'b0};
    assign x_U_TYPE_IMM =   {x_instr[31:12],12'b0};
    assign x_UJ_TYPE_IMM =  {{12{x_IMM_SIGN}},x_instr[19:12],x_instr[20],x_instr[30:21],1'b0};

    //Branch calculations
    //Branch addition done in Branch_Block
    //Branch additions always go to PC, ALU calculations always go to memory
    //Choose which branch condition to use
    always @ ( * ) begin
        case(x_instr`funct3)
            0 :         x_br_valid = x_br_eq;
            1 :         x_br_valid = ~x_br_eq;
            4 :         x_br_valid = x_br_lt;
            5 :         x_br_valid = ~x_br_lt;
            6 :         x_br_valid = x_br_ltu;
            7 :         x_br_valid = ~x_br_ltu;
            default:    x_br_valid = 1'b0;
        endcase
    end

// check instructions code violation
/*
    always @ (negedge clk_i or posedge rst_i) begin
        begin
           if (f_instr`funct7 == 7'h37)
           begin
              x_lui <= 1'b0;
           end
           else if (f_instr`funct7 == 7'h17)
           begin
              x_auipc <= 1'b0;
           end
           else if (f_instr`funct7 == 7'h6F)
           begin
              x_jal <= 1'b0;
           end
           else if (f_instr`funct7 == 7'h67)
           begin
              if (x_instr`funct3 != 3'b000)
              begin
                 x_jalr <= 1'b1;
                      $display("ERROR : INSTRUCTION CODE VIOLATION for JALR", $time);
              end
              else
              begin
                 x_jalr <= 1'b0;
              end
           end
           else if (f_instr`funct7 == 7'h63)
           begin
              if ((x_instr`funct3 == 3'b000) || (x_instr`funct3 == 3'b011))
              begin
                 x_b <= 1'b1;
                      $display("ERROR : INSTRUCTION CODE VIOLATION for BE", $time);
              end
              else
              begin
                 x_b <= 1'b0;
              end
           end
           else if (f_instr`funct7 == 7'h03)
           begin
              if ((x_instr`funct3 == 3'b011) || (x_instr`funct3 == 3'b110) || (x_instr`funct3 == 3'b111))
              begin
                 x_lh <= 1'b1;
                      $display("ERROR : INSTRUCTION CODE VIOLATION for BE", $time);
              end
              else
              begin
                 x_lh <= 1'b0;
              end
           end
           else if (f_instr`funct7 == 7'h23)
           begin
              if ((x_instr`funct3 != 3'b000) && (x_instr`funct3 != 3'b001) && (x_instr`funct3 != 3'b010))
              begin
                 x_sb <= 1'b1;
                      $display("ERROR : INSTRUCTION CODE VIOLATION for BE", $time);
              end
              else
              begin
                 x_sb <= 1'b0;
              end
           end
           else if (f_instr`funct7 == 7'h13)
           begin
              if ((x_instr`funct3 != 3'b001) && (x_instr`funct3 != 3'b101))
              begin
                 x_addi <= 1'b1;
                      $display("ERROR : INSTRUCTION CODE VIOLATION for BE", $time);
              end
              else
              begin
                 x_addi <= 1'b0;
              end
           end
           else if (f_instr`funct7 == 7'h13)
           begin
              if ((x_instr`funct3 != 3'b001) && (x_instr`funct3 != 3'b101))
              begin
                 x_addi <= 1'b1;
                      $display("ERROR : INSTRUCTION CODE VIOLATION for BE", $time);
              end
              else
              begin
                 x_addi <= 1'b0;
              end
           end


           case(f_instr`funct7)
               7'h37 : // LUI
                   begin
                      x0 <= 1'b0;
                     // $display("ERROR : INSTRUCTION CODE VIOLATION for JALR", $time);
                   end
               7'h17 : // AUIPC
                   begin
                      x1 <= 1'b0;
                     // $display("ERROR : INSTRUCTION CODE VIOLATION for JALR", $time);
                   end
               7'h6F : // AUIPC
                   begin
                      x2 <= 1'b0;
                     // $display("ERROR : INSTRUCTION CODE VIOLATION for JALR", $time);
                   end
               7'h67 : if (x_instr`funct3 != 3'b000)
                   begin
                      x3 <= 1'b1;
                      $display("ERROR : INSTRUCTION CODE VIOLATION for JALR", $time);
                   end
               7'h63 : if ((x_instr`funct3 == 3'b010) || (x_instr`funct3 == 3'b011))
                   begin
                      x4 <= 1'b1;
                      $display("ERROR : INSTRUCTION CODE VIOLATION for BEQ, BNE, BLT, BGE, BLTU, BGEU", $time);
                   end
               7'h03 : if ((x_instr`funct3 == 3'b011) || (x_instr`funct3 == 3'b110) || (x_instr`funct3 == 3'b111))
                   begin
                      $display("ERROR : INSTRUCTION CODE VIOLATION for LB, LH, LW, LBU, LHU", $time);
                   end
               7'h23 : if ((x_instr`funct3 != 3'b000) && (x_instr`funct3 != 3'b001) && (x_instr`funct3 != 3'b010))
                   begin
                      $display("ERROR : INSTRUCTION CODE VIOLATION for SB, SH, SW", $time);
                   end
               7'h13 : if ((x_instr`funct3 == 3'b001) || (x_instr`funct3 == 3'b101) || (x_instr`funct3 == 3'b101))
                   begin
                      if ((x_instr[31:25] != 7'h00) && (x_instr[31:25] != 7'h20))
                         $display("ERROR : INSTRUCTION CODE VIOLATION for SLLI, SRLI, SRAI", $time);
                   end
               7'h33 : if (x_instr`funct3 == 3'b000)
                   begin
                      if ((x_instr[31:25] != 7'h00) && (x_instr[31:25] != 7'h20) && (x_instr[31:25] != 7'h01))
                         $display("ERROR : INSTRUCTION CODE VIOLATION for ADD, SUB, MUL", $time);
                         $display("ERROR : INSTRUCTION CODE VIOLATION for ADD, SUB, MUL : 0x%x ", x_instr, $time );
                   end
                   else if (x_instr`funct3 == 3'b001)
                   begin
                      if ((x_instr[31:25] != 7'h00) && (x_instr[31:25] != 7'h01))
                         $display("ERROR : INSTRUCTION CODE VIOLATION for SLL, MULH", $time);
                   end
                   else if (x_instr`funct3 == 3'b010)
                   begin
                      if ((x_instr[31:25] != 7'h00) && (x_instr[31:25] != 7'h01))
                         $display("ERROR : INSTRUCTION CODE VIOLATION for SLT, MULHSU", $time);
                   end
                   else if (x_instr`funct3 == 3'b011)
                   begin
                      if ((x_instr[31:25] != 7'h00) && (x_instr[31:25] != 7'h01))
                         $display("ERROR : INSTRUCTION CODE VIOLATION for SLTU, MULHU", $time);
                   end
                   else if (x_instr`funct3 == 3'b100)
                   begin
                      if (x_instr[31:25] != 7'h00)
                         $display("ERROR : INSTRUCTION CODE VIOLATION for XOR", $time);
                   end
                   else if (x_instr`funct3 == 3'b101)
                   begin
                      if ((x_instr[31:25] != 7'h00) && (x_instr[31:25] != 7'h20))
                         $display("ERROR : INSTRUCTION CODE VIOLATION for SRL, SRA", $time);
                   end
                   else if ((x_instr`funct3 == 3'b110) || (x_instr`funct3 == 3'b111))
                   begin
                      if (x_instr[31:25] != 7'h00)
                         $display("ERROR : INSTRUCTION CODE VIOLATION for OR, AND", $time);
                   end
//                end
//                   else
//                   begin
//                      $display("ERROR : INSTRUCTION CODE VIOLATION opcode", $time);
                      default : $display("ERROR : INSTRUCTION CODE VIOLATION opcode", $time);
//                   end
           endcase
    end
*/
    //If the instruction in EX stage is Jump or a valid branch,
    //a no-op instruction will be inserted immediately for the next fetch
    assign x_no_op = (x_instr`opcode == `JAL)
                    || (x_instr`opcode == `JALR)
                    || ((x_instr`opcode == `BRANCH) && x_br_valid);

    //Register Forwarding unit
    assign x_rs1_fwd =     ((x_instr`rs1 == w_instr`rd) && w_reg_wr_en) ? w_reg_wdata : x_rs1_in;
    assign x_rs2_fwd =     ((x_instr`rs2 == w_instr`rd) && w_reg_wr_en) ? w_reg_wdata : x_rs2_in;

    //Choose values for rs1 input to ALU: PC for AUIPC, 0 for LUI, and rs1 else
    always @ ( * ) begin
        case(x_instr`opcode)
            `AUIPC,`JAL,`JALR:    x_rs1 = x_pc;
            `LUI:                 x_rs1 = 0;
            default:              x_rs1 = x_rs1_fwd;
        endcase
    end

    //Choose values for rs2 input to ALU: register values, immediate, or 4 for jumps
    always @ ( * ) begin
        case(x_instr`opcode)
            `R_TYPE,`BRANCH :     x_rs2 = x_rs2_fwd;
            `I_TYPE,`LOAD :       x_rs2 = x_I_TYPE_IMM;
            `STORE :              x_rs2 = x_S_TYPE_IMM;
            `LUI,`AUIPC :         x_rs2 = x_U_TYPE_IMM;
            `JAL,`JALR :          x_rs2 = 32'h4;
            default :             x_rs2 = 32'h0;
        endcase
    end

    //Output from CPU
    assign data_addr_o = x_alu_out;                         //Data Address goes into the data memory
    assign data_wdata_o = x_rs2_fwd;                        //Data Value goes into the data memory
    assign data_wr_en_o = x_instr`opcode==`STORE;           //Data Write Enable goes into the data memory
    assign funct3_o = x_instr`funct3;                       //funct3 for data memory

`ifdef debug_sim
reg [31:0] data_addr_display;                   //Read Address to Data Memory Unit

always @ (negedge clk_i)
begin
   if (x_instr`opcode==`STORE)
   $display ("    STORE %d, 0x%h", data_wdata_o, data_addr_o[15:0]);
   if (w_instr`opcode==`LOAD)
   $display ("    LOAD %d, 0x%h", w_reg_wdata, data_addr_display[15:0]);

end
`endif // debug_sim

/////////////////////////////////////////////////////
//WriteBack Stage (WB)
/////////////////////////////////////////////////////

    //pipeline registers
    always @ (posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            w_instr <= 12'b0;
            w_alu_out <= 32'b0;
`ifdef debug_sim
            data_addr_display <= 32'b0;
`endif
        end else if (~stall) begin
            w_instr <= x_instr`opcoderd;  // only need bits [11:0].
            w_alu_out <= x_alu_out;
`ifdef debug_sim
            data_addr_display <= data_addr_o;
`endif
        end
    end

    assign w_reg_wr_en = (~(w_instr`opcode==`STORE)
                        && ~(w_instr`opcode==`BRANCH)
                        && (w_instr`rd != 0));

    assign w_reg_wdata = (w_instr`opcode == `LOAD) ? data_rdata_i : w_alu_out;

endmodule
