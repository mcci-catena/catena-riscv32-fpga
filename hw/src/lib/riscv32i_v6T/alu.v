/*

Module: alu.v

Function:
    ALU for RISC-V CPU

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
module alu #(parameter has_mult=0, unsigned_mult=0)(

    // ----- Inputs -------
    input [31:0] data_a_i,      // Data Input A
    input [31:0] data_b_i,      // Data Input B
    input [2:0]  funct3_i,      // RISC-V funct3
    input [6:0]  funct7_i,      // RISC-V funct7
    input r_type_i,             // If this bit is set, the op-code is R type
    input i_type_i,             // If this bit is set, the op-code is I type

        // ----- Outputs -------
    output reg [31:0] result_o, // Output result
    output underflow_o,         // Set to 1 if the subtraction result is underflow
    output overflow_o,          // Set to 1 if the addition result is overflow

    output br_eq_o,             // Set to 1 if Data A and B are equal
    output br_lt_o,             // Set to 1 if Data A < Data B (signed comparison)
    output br_ltu_o             // Set to 1 if Data A < Data B (unsigned comparison)
    );

    // ----- Multiplication -------
    // Need to optimize later
    //wire [31:0] quotient, quotient_u;
    //wire [31:0] remainder, remainder_u;
    //wire signed [32:0] data_a_sgn = {data_a_i[31],data_a_i};
    //wire [32:0] data_b_sgn = {1'b0, data_b_i};
    //wire [63:0] product, product_u;
    //wire signed [64:0] product_su;
    //assign product = data_a_i * data_b_i;
    //assign product_s = $signed(data_a_i) * $signed(data_b_i);
    //assign product_u = $unsigned(data_a_i) * $unsigned(data_b_i);
    //assign product_su = $signed(data_a_sgn) * $signed(data_b_sgn);
    //assign quotient = $signed(data_a_i) / $signed(data_b_i);
    //assign quotient_u = $unsigned(data_a_i) / $unsigned(data_b_i);
    //assign remainder = $signed(data_a_i) % $signed(data_b_i);
    //assign remainder_u = $unsigned(data_a_i) % $unsigned(data_b_i);

    //wire [31:0] p_31_0, p_63_32, pu_63_32, psu_63_32;
    //assign p_31_0 = product[31:0];
    //assign p_63_32 = product[63:32];
    //assign pu_63_32 = product_u[63:32];
    //assign psu_63_32 = product_su[63:32];

// Mult32 implementation for iCE device
   wire [31:0] p_lsb, p_lsb1, p_lsb2, p_msb;

   // these are unused unless we have a signed multiplier
   /* verilator lint_off UNUSED */
   wire [31:0] data_a_i1, data_b_i1;
   wire  sign_p;
   /* verilator lint_on UNUSED */

   wire [47:0] p_lsb1_48, p_lsb2_48, p_msb_48;
   wire [48:0] P_49;

   // TODO(tmm@mcci.com): get rid of bit 64, as it's not used.
   /* verilator lint_off UNUSED */
   wire [64:0] PRODUCT;
   wire [64:0] PRODUCT1_65bit;
   /* verilator lint_on UNUSED */

   wire [63:0] PRODUCT1;
   reg  sign_a, sign_b;

   assign sign_p = (sign_a & data_a_i[31]) ^ (sign_b & data_b_i[31]);

   assign data_a_i1 = (sign_a & data_a_i[31]) ? (~data_a_i + 1) : data_a_i;
   assign data_b_i1 = (sign_b & data_b_i[31]) ? (~data_b_i + 1) : data_b_i;

   assign p_lsb1_48 = {16'b0, p_lsb1};
   assign p_lsb2_48 = {16'b0, p_lsb2};
   assign p_msb_48 = {p_msb, p_lsb[31:16]};
   assign P_49 = p_lsb1_48 + p_lsb2_48 + p_msb_48;
   assign PRODUCT1_65bit = {P_49, p_lsb[15:0]};
   assign PRODUCT1 = PRODUCT1_65bit[63:0];

   // Unsigned

//   assign P_63_32 = PRODUCT[63:32];
//   assign P_31_0 = PRODUCT[31:0];

//   assign p_lsb = data_a_i1[15:0] * data_b_i1 [15:0];
//   assign p_lsb1 = data_a_i1[31:16] * data_b_i1 [15:0];
//   assign p_lsb2 = data_a_i1[15:0] * data_b_i1 [31:16];
//   assign p_msb = data_a_i1[31:16] * data_b_i1 [31:16];

// instance
generate
if(has_mult) begin: mult_on
    /*
    defparam mult16_lsb.AB_SIGNED = 1'b0;
    defparam mult16_lsb1.AB_SIGNED = 1'b0;
    defparam mult16_lsb2.AB_SIGNED = 1'b0;
    defparam mult16_msb.AB_SIGNED = 1'b0;
    */

    if(unsigned_mult) begin: unsigned_mult_on
        mult16 #(.AB_SIGNED(0)) mult16_lsb  (.data_a_i(data_a_i[15:0]), .data_b_i(data_b_i[15:0]), .q(p_lsb));
        mult16 #(.AB_SIGNED(0)) mult16_lsb1 (.data_a_i(data_a_i[31:16]), .data_b_i(data_b_i[15:0]), .q(p_lsb1));
        mult16 #(.AB_SIGNED(0)) mult16_lsb2 (.data_a_i(data_a_i[15:0]), .data_b_i(data_b_i[31:16]), .q(p_lsb2));
        mult16 #(.AB_SIGNED(0)) mult16_msb  (.data_a_i(data_a_i[31:16]), .data_b_i(data_b_i[31:16]), .q(p_msb));
        assign PRODUCT = { 1'b0, PRODUCT1 };
    end else begin: signed_mult_on
        mult16 #(.AB_SIGNED(0)) mult16_lsb  (.data_a_i(data_a_i1[15:0]), .data_b_i(data_b_i1[15:0]), .q(p_lsb));
        mult16 #(.AB_SIGNED(0)) mult16_lsb1 (.data_a_i(data_a_i1[31:16]), .data_b_i(data_b_i1[15:0]), .q(p_lsb1));
        mult16 #(.AB_SIGNED(0)) mult16_lsb2 (.data_a_i(data_a_i1[15:0]), .data_b_i(data_b_i1[31:16]), .q(p_lsb2));
        mult16 #(.AB_SIGNED(0)) mult16_msb  (.data_a_i(data_a_i1[31:16]), .data_b_i(data_b_i1[31:16]), .q(p_msb));
        assign PRODUCT = sign_p ? (~PRODUCT1 + 1) : PRODUCT1;
    end
end else begin: mult_off
    assign p_msb = 32'h0;
    assign p_lsb1 = 32'h0;
    assign p_lsb2 = 32'h0;
    assign p_lsb = 32'h0;
    assign PRODUCT = PRODUCT1;
end
endgenerate

// end
    // ----- Addition Logic -------
    wire [32:0] result_add;                                                 //33-bit addition result
    assign result_add = {1'b0, data_a_i[31:0]} + {1'b0, data_b_i[31:0]};    //If the MSB has the carry bit, it is overflowed
    assign overflow_o = result_add[32];

    // ----- Subtraction Logic -------
    wire [32:0] result_sub;                                                 //33-bit subtraction result
    assign result_sub = {1'b1, data_a_i[31:0]} - {1'b0, data_b_i[31:0]};    //If the MSB (bit 33) is borrowed, it is under flowed
    assign underflow_o = ~result_sub[32];

    // ----- Branch Comparison (Share with the subtraction logic)-------
    assign br_eq_o = ~(|result_sub[31:0]);                                              //If all the bits are zeros, data_a == data_b
    assign br_lt_o = (data_a_i[31] ^ data_b_i[31]) ? data_a_i[31] : ~result_sub[32];    //If the signed bits are same, check the borrow bit. Otherwise, check the signed bit of A
    assign br_ltu_o = ~result_sub[32];                                                  //If the MSB (bit 33) is borrowed, it is unsigned less than

    // ----- Logical/Arithmetic Shifter -------
    //Shifter Control Signal
    wire shift_dir = funct3_i[2];                                   //If funct3[2] bit set to 1, it shift right, otherwise shift left
    wire shift_bit = funct7_i[5] ? data_a_i[31] : 1'b0;             //If funct7[5] bit set to 1, it shift arithmetic, otherwise logical
    wire [31:0] result_shifter;                                     //Intermediate wire to for shift result

    //Barrel Shifter Logic
    wire [31:0] shval_intA, shval_intB, shval_intC, shval_intD, shval_intE; //Wire for intermediate shift result
    wire [31:0] shifted_A, shifted_B, shifted_C, shifted_D, shifted_E;      //Wire for intermediate shift value

    assign shifted_A = shift_dir ? {{16{shift_bit}}, shval_intA[31:16]} : {shval_intA[15:0] , 16'b0};
    assign shifted_B = shift_dir ? {{ 8{shift_bit}}, shval_intB[31: 8]} : {shval_intB[23:0] ,  8'b0};
    assign shifted_C = shift_dir ? {{ 4{shift_bit}}, shval_intC[31: 4]} : {shval_intC[27:0] ,  4'b0};
    assign shifted_D = shift_dir ? {{ 2{shift_bit}}, shval_intD[31: 2]} : {shval_intD[29:0] ,  2'b0};
    assign shifted_E = shift_dir ? {{ 1{shift_bit}}, shval_intE[31: 1]} : {shval_intE[30:0] ,  1'b0};

    assign shval_intA   = data_a_i;
    assign shval_intB   = data_b_i[4] ? shifted_A : shval_intA;
    assign shval_intC   = data_b_i[3] ? shifted_B : shval_intB;
    assign shval_intD   = data_b_i[2] ? shifted_C : shval_intC;
    assign shval_intE   = data_b_i[1] ? shifted_D : shval_intD;
    assign result_shifter   = data_b_i[0] ? shifted_E : shval_intE;


    // ----- ALU Operation based on the funct3-------
    always @ (*) begin
        if(~r_type_i & ~i_type_i)
            begin
                //If it is not R-Type or I-Type, it always add
                result_o = result_add[31:0];
            end
        else
            begin
                case (funct3_i)
                    `ADD:
                        begin     //ADDI, ADD, SUB, MUL
                        //result_o = (r_type_i & funct7_i[5])? result_sub[31:0] : result_add[31:0];
                        if (r_type_i)
                        begin
                            case (funct7_i)
                                `ADD_7:
                                    begin
                                        result_o = result_add[31:0];
                                        sign_a = 1'b0;
                                        sign_b = 1'b0;
                                    end
                                `MUL_7:
                                    begin
                                        //result_o = product[31:0];
                                        result_o = PRODUCT[31:0];
                                        sign_a = 1'b1;
                                        sign_b = 1'b1;
                                    end
                                `SUB_7:
                                    begin
                                        result_o = result_sub[31:0];
                                        sign_a = 1'b0;
                                        sign_b = 1'b0;
                                    end
                                default:
                                    begin
                                        sign_a = 1'b0;
                                        sign_b = 1'b0;
                                        result_o = 32'd0;
                                    end
                            endcase
                        end
                        else
                        begin
                            result_o = result_add[31:0];
                        end
                    end //ADDI, ADD, SUB, MUL

                    `SLL:
                        begin       //SLLI, SLL, MULH
                            sign_a = 1'b1;
                            sign_b = 1'b1;
                            //result_o = (r_type_i && (funct7_i == `MUL_7)) ? product[63:32] : result_shifter;
                            result_o = (r_type_i && (funct7_i == `MUL_7)) ? PRODUCT[63:32] : result_shifter;
                        end //SLLI, SLL, MULH

                    `SLT:
                        begin       //SLTI, SLT, MULHSU
                            sign_a = 1'b1;
                            sign_b = 1'b0;
                            //result_o = (r_type_i && (funct7_i == `MUL_7)) ? product_su[63:32] : {31'b0, br_lt_o};             //If the signed bit (bit 32) is 1, it is signed less than
                            result_o = (r_type_i && (funct7_i == `MUL_7)) ? PRODUCT[63:32] : {31'b0, br_lt_o};              //If the signed bit (bit 32) is 1, it is signed less than
                        end //SLTI, SLT, MULHSU

                    `SLTU:
                        begin   //SLTIU, SLTU, MULHU
                            sign_a = 1'b0;
                            sign_b = 1'b0;
                            //result_o = (r_type_i && (funct7_i == `MUL_7)) ? product_u[63:32] : {31'b0, br_ltu_o};         //If the MSB (bit 33) is borrowed, it is unsigned less than
                            result_o = (r_type_i && (funct7_i == `MUL_7)) ? PRODUCT[63:32] : {31'b0, br_ltu_o};         //If the MSB (bit 33) is borrowed, it is unsigned less than
                        end //SLTIU, SLTU, MULHU

                    `XOR:
                        begin       //XORI, XOR, DIV
                            sign_a = 1'b0;
                            sign_b = 1'b0;
                            result_o = data_a_i ^ data_b_i;
                        end //XORI, XOR, DIV

                    `SRLA:
                        begin   //SRLI, SRAI, SRL, SRA, DIVU
                            sign_a = 1'b0;
                            sign_b = 1'b0;
                            result_o = result_shifter;
                        end //SRLI, SRAI, SRL, SRA, DIVU

                    `OR:
                        begin       //ORI, OR, REM
                            result_o = data_a_i | data_b_i;
                        end //ORI, OR, REM

                    `AND:
                        begin       //ANDI, AND, REMU
                            result_o = data_a_i & data_b_i;
                        end //ANDI, AND, REMU

                    default:
                        begin
                            sign_a = 1'b0;
                            sign_b = 1'b0;
                            result_o = 32'h0;
                        end
                endcase
            end
    end

endmodule
