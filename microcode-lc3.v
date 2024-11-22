/*
    CompEng 361 - Northwestern Unviersity

    Mini LC3 Microcoded Control

    Described in Lecture 08 (10/21/2024)

    Supports:
     - Operate (ADD,AND,NOT)
     - Memory Transfer (LD/ST)
     - Control (JMP)

    Extended to support instructions:
     - BR, LDR, STR

    This is a really simple microcoded design which is very similar to the
    multicycle (FSM) control that we saw previously.
 
    The biggest difference is that our FSM design was a Mealy machine.
    What we have here is equivalent to us converting that Mealy machine to a Moore machine
    by adding states so that output is only a function of current state.
    Then converting it to a microcoded implmentation.

    The design does not support self-modifying code.

*/

`define WORD_WIDTH  16
`define REG_COUNT 8
`define OP_SIZE 2
`define RF_ADDR_WIDTH 3
`define MEM_SIZE 65536
`define FN_SIZE 2

`define ADD_OPCODE 4'b0001
`define AND_OPCODE 4'b0101
`define LD_OPCODE 4'b0010
`define ST_OPCODE 4'b0011
`define NOT_OPCODE 4'b1001
`define JMP_OPCODE 4'b1100
`define BR_OPCODE  4'b0000
`define LDR_OPCODE 4'b0110
`define STR_OPCODE 4'b0111

`define UOP_CONTROL_WIDTH 16
`define OPCODE_WIDTH 4
`define STATE_WIDTH  5

module LC3DataPath(
        output [15:0] addr,
        output [15:0] dataWrite,
        input  [15:0] dataRead,
        output dataWrEn,
        input clk, input rst);

    parameter StartAddr = 16'h3000;

   
    wire [`WORD_WIDTH-1:0] SR1_out, SR2_out, DR_in;
    wire [`RF_ADDR_WIDTH-1:0] SR1, SR2, DR;
    wire [`WORD_WIDTH-1:0] IR;
    wire [`WORD_WIDTH-1:0] alu_out, alu_opa, alu_opb;
    wire [`WORD_WIDTH-1:0] PC, NPC, PC_plus_one;
    wire [`WORD_WIDTH-1:0] immediate, offset, offset6;
    wire [1:0] alu_fn;

    wire [3:0] opcode;

    wire [`STATE_WIDTH-1:0] state, next_state;
    wire IR_we, PC_we, NZP_we;

    wire  addr_sel, DR_sel;

    wire [`UOP_CONTROL_WIDTH-1:0] control_word;
    wire [`STATE_WIDTH-1:0] dispatch_state;
    wire [1:0] state_sel;

    wire alu_opa_sel;
    wire [1:0] alu_opb_sel;
    wire branch_taken, BR_sel;

    // NZP Flags
    wire N, Z, P; // NZP flags are now wires
    wire [2:0] NZP, nzp_in; // Input value for NZP flags

    assign {P, Z, N} = NZP;

    assign invalid_op = (opcode != `ADD_OPCODE) &&
                        (opcode != `AND_OPCODE) && (opcode != `NOT_OPCODE) &&
                        (opcode != `LD_OPCODE)  && (opcode != `ST_OPCODE)  &&
                        (opcode != `JMP_OPCODE) && (opcode != `BR_OPCODE)  &&
                        (opcode != `LDR_OPCODE)  && (opcode != `STR_OPCODE);
   
      /*  state_sel:
            00 => state + 1
            01 => from dispatch rom
            otherwise => restart (fetch)
       */
    assign next_state = (state_sel == 2'b00) ? state + 1 :
                        (state_sel == 2'b01) ? dispatch_state :
                        5'b0000;


    // Basic decode fields
    assign opcode = IR[15:12];
    assign SR1 = IR[8:6];
    assign SR2 = (dataWrEn) ? DR : IR[2:0]; // If ST or STR then SR2 = DR, else SR2 = Standard SR2
    assign DR = IR[11:9];


    // Various datapath muxes
    assign addr       = (addr_sel) ? PC : alu_out;
    assign dataWrite  = SR2_out;
    assign alu_opa    = (alu_opa_sel) ? PC_plus_one : SR1_out;
    assign alu_opb    = (alu_opb_sel == 2'b11) ? offset :
                        (alu_opb_sel == 2'b10) ? offset6 :
                        (alu_opb_sel == 2'b01) ? immediate :
                        SR2_out;
    assign DR_in      = (DR_sel) ? dataRead : alu_out;

    // sign extend immediate and offset
    assign immediate = {{12{IR[4]}}, IR[3:0]};
    assign offset = {{8{IR[8]}}, IR[7:0]};
    assign offset6 = {{10{IR[5]}}, IR[5:0]};

    // Next PC Logic
    assign PC_plus_one = PC + 1;
    assign branch_taken = (BR_sel && ((IR[11] & N) | (IR[10] & Z) | (IR[9] & P)));
    assign NPC = (NPC_sel) ? SR1_out : 
                 (branch_taken) ? alu_out :
                 PC_plus_one;

    // Control word for microcode
    assign {IR_we, PC_we, DR_we, dataWrEn, alu_fn, 
     addr_sel, alu_opa_sel, alu_opb_sel, DR_sel,
     NPC_sel, state_sel, NZP_we, BR_sel}   = control_word;

    // Define NZP flags based on the result of the DR_in value
    assign nzp_in[0] = (DR_in[15] == 1'b1);               // N flag: Negative
    assign nzp_in[1] = (DR_in == 16'b0);                  // Z flag: Zero
    assign nzp_in[2] = (DR_in[15] == 1'b0 && DR_in != 0); // P flag: Positive

   // Dispatch ROM
   DispatchROM  DPROM(.state(dispatch_state),
          .opcode(opcode),
          .immediate(IR[5]));

   // Primary Microcode ROM
   UopROM UROM( .control_word(control_word),
    .state(state));
   
   
    RegFile RF( .SrcAddr1(SR1), .SrcData1(SR1_out), .SrcAddr2(SR2), .SrcData2(SR2_out), 
                .DstAddr(DR), .DstData(DR_in),
                .WriteEn(DR_we), .clk(clk));

    ALU LC3ALU( .result(alu_out),
                .opA(alu_opa),
                .opB(alu_opb),
                .fn(alu_fn));

    // The PC
    Register #(16,StartAddr)  PC_REG (.q(PC),     
                                      .d(NPC), 
                                      .we(PC_we),
                                      .clk(clk),
                                      .rst(rst));

    // The Instruction Register
    Register #(16)  IR_REG (.q(IR),
                            .d(dataRead),
                            .we(IR_we),
                            .clk(clk),
                            .rst(rst));

    // Control Register
    Register #(5) CONTRL_REG (.q(state),
                              .d(next_state),
                              .we(1'b1),
                              .clk(clk),
                              .rst(rst));
    
    // NZP Registers
    Register #(3) NZP_REG (.q(NZP),
                           .d(nzp_in),
                           .we(NZP_we),
                           .clk(clk),
                           .rst(rst));

endmodule // LC3DataPath

module UopROM(
        output [`UOP_CONTROL_WIDTH-1:0] control_word,
        input [`STATE_WIDTH-1:0]         state);
   reg [`UOP_CONTROL_WIDTH-1:0]         control_word;
   

    /* Control Word = {
           IR_we, PC_we, DR_we, dataWrEn, 
           alu_fn[1:0], 
           addr_sel, alu_opa_sel, alu_opb_sel[1:0], DR_sel,
     NPC_sel, state_sel, NZP_we, BR_sel}
     */

   
   always @* begin
     case (state)
       5'b00000: control_word = 16'b1000001000000000; // FETCH
       5'b00001: control_word = 16'b0000000000000100; // DECODE
       5'b00010: control_word = 16'b0100000000011100; // JMP
       5'b00011: control_word = 16'b0110110000001110; // NOT
       5'b00100: control_word = 16'b0110000001001110; // ADDI
       5'b00101: control_word = 16'b0110000000001110; // ADDR
       5'b00110: control_word = 16'b0110100001001110; // ANDI
       5'b00111: control_word = 16'b0110100000001110; // ANDR
       5'b01000: control_word = 16'b0000000111000000; // LD 0
       5'b01001: control_word = 16'b0110000111101110; // LD 1
       5'b01010: control_word = 16'b0000000010000000; // LDR 0
       5'b01011: control_word = 16'b0110000010101110; // LDR 1
       5'b01100: control_word = 16'b0000000111000000; // ST 0
       5'b01101: control_word = 16'b0101000111001100; // ST 1
       5'b01110: control_word = 16'b0000000010000000; // STR 0
       5'b01111: control_word = 16'b0101000010001100; // STR 1
       5'b10000: control_word = 16'b0000000111000000; // BR 0
       5'b10001: control_word = 16'b0100000111001101; // BR 0
       default: control_word = 0;
     endcase // case (state)
   end // always @ *

endmodule // UopROM

module DispatchROM(
       output [`STATE_WIDTH-1:0] state,
       input [`OPCODE_WIDTH-1:0] opcode,
       input          immediate);
   reg [`STATE_WIDTH-1:0]          state;
   
   always @* begin
     casex ({opcode,immediate})
       5'b00010: state = 5'b00101; // ADDR
       5'b00011: state = 5'b00100; // ADDI
       5'b01010: state = 5'b00111; // ANDR
       5'b01011: state = 5'b00110; // ANDI
       5'b1001x: state = 5'b00011; // NOT
       5'b1100x: state = 5'b00010; // JMP
       5'b0010x: state = 5'b01000; // LD
       5'b0011x: state = 5'b01100; // ST
       5'b0000x: state = 5'b10000; // BR
       5'b0110x: state = 5'b01010; // LDR
       5'b0111x: state = 5'b01110; // STR
       default: state = 0;
     endcase // casex ({opcode,immediate})
   end // always @ *

endmodule // DispatchROM


module LC3Memory(
    input [`WORD_WIDTH-1:0] addr, 
    input [`WORD_WIDTH-1:0] DataWrite, 
    output [`WORD_WIDTH-1:0] DataRead,
    input WriteEn, 
    input clk);

    reg [`WORD_WIDTH-1:0] mem[0:`MEM_SIZE-1];

    assign DataRead = mem[addr];

    always @ (posedge clk) begin
        if (WriteEn)
            mem[addr] <= DataWrite;
    end


endmodule

module RegFile(
    input [`RF_ADDR_WIDTH-1:0] SrcAddr1, 
    output [`WORD_WIDTH-1:0] SrcData1, 
    input [`RF_ADDR_WIDTH-1:0] SrcAddr2, 
    output [`WORD_WIDTH-1:0] SrcData2, 
    input [`RF_ADDR_WIDTH-1:0] DstAddr,
    input [`WORD_WIDTH-1:0] DstData,
    input WriteEn, 
    input clk);

    reg [`WORD_WIDTH-1:0] mem[0:`REG_COUNT-1];

    assign SrcData1 = mem[SrcAddr1];
    assign SrcData2 = mem[SrcAddr2];

    always @ (posedge clk)
        if (WriteEn)
            mem[DstAddr] <= DstData;
endmodule

module ALU(
    output [`WORD_WIDTH-1:0] result,
    input [`WORD_WIDTH-1:0] opA,
    input [`WORD_WIDTH-1:0] opB,
    input [`FN_SIZE-1:0] fn);

    wire [`WORD_WIDTH-1:0] addResult, andResult, notResult;

    assign addResult = opA + opB;
    assign andResult = opA & opB;
    assign notResult = ~opA;

    assign result = (fn == 2'b11) ? notResult : 
        (fn[1] == 1'b0) ? addResult : andResult;

endmodule

module Register(
    output [width-1:0] q, 
    input [width-1:0] d,
    input we,
    input clk,
    input rst);
    parameter width = 16;
    parameter rst_value = 0;

    reg [width-1:0] q;

    always @ (posedge clk or negedge rst)
        if (~rst)
            q <= rst_value;
        else 
            q <= (we) ? d : q;

endmodule
