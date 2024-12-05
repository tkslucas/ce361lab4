// Template for Northwestern - CompEng 361 - Lab4
// Groupname: RISCtakers
// NetIDs: rap4819, ltb8987

// ------------------------------- OPCODES ----------------------------------
`define WORD_WIDTH 32
`define NUM_REGS 32

`define OPCODE_COMPUTE     7'b0110011
`define OPCODE_BRANCH      7'b1100011
`define OPCODE_LOAD        7'b0000011
`define OPCODE_STORE       7'b0100011 
`define OPCODE_COMPUTE_IMM 7'b0010011
`define OPCODE_MULDIV      7'b0110011
`define OPCODE_JAL         7'b1101111
`define OPCODE_JALR        7'b1100111
`define OPCODE_LUI         7'b0110111
`define OPCODE_AUIPC       7'b0010111

// ------------------------------- FUNC3 --------------------------------------
`define FUNC_ADD      3'b000  // same for subtraction
`define FUNC_XOR      3'b100
`define FUNC_OR       3'b110
`define FUNC_AND      3'b111
`define FUNC_SLL      3'b001
`define FUNC_SRL      3'b101
`define FUNC_SRA      3'b101
`define FUNC_SLT      3'b010
`define FUNC_SLTU     3'b011

// jalr, I-type
`define FUNC_JALR     3'b000

// for clarity: redefined for loads
`define FUNC_LB      3'b000
`define FUNC_LH      3'b001
`define FUNC_LW      3'b010
`define FUNC_LBU     3'b100
`define FUNC_LHU     3'b101

// defined for stores
`define FUNC_SB     3'b000
`define FUNC_SH     3'b001
`define FUNC_SW     3'b010

// Branch func3 codes
`define FUNC_BEQ     3'b000  // Branch if equal
`define FUNC_BNE     3'b001  // Branch if not equal
`define FUNC_BLT     3'b100  // Branch if less than
`define FUNC_BGE     3'b101  // Branch if greater or equal
`define FUNC_BLTU    3'b110  // Branch if less than, unsigned
`define FUNC_BGEU    3'b111  // Branch if greater or equal, unsigned

// defined for M extension
`define FUNC_MUL      3'b000
`define FUNC_MULH     3'b001
`define FUNC_MULSU    3'b010
`define FUNC_MULU     3'b011
`define FUNC_DIV      3'b100
`define FUNC_DIVU     3'b101
`define FUNC_REM      3'b110
`define FUNC_REMU     3'b111

// ----------------------------- AUX_FUNC --------------------------------------
`define AUX_FUNC_ADD    7'b0000000  // same for xor,or,and,sll,srl,slt,sltu
`define AUX_FUNC_SUB    7'b0100000  // same for sra
`define AUX_FUNC_M_EXT  7'b0000001  // same for all RV32M instructions

`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10

module PipelinedCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [`WORD_WIDTH-1:0] PC, InstWord;
   wire [`WORD_WIDTH-1:0] DataAddr, StoreData, DataWord;
   wire [1:0]  MemSize, load_size, store_size;
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;
   wire [`WORD_WIDTH-1:0] Rdata1, Rdata2, RWrdata, EU_out;
   wire [`WORD_WIDTH-1 :0] immediate, immediate_i, immediate_j, immediate_b, immediate_st;
   wire        RWrEn;
   wire        ALUSrc, EACalc_control, muldiv_control;
   wire        Inv_R_type, Inv_I_type, Inv_Loads, Inv_B_type, Inv_J_type, Inv_U_type, Inv_S_type, Inv_MulDiv;

   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4;
   wire [6:0]  opcode;

   wire [6:0]  funct7;
   wire [2:0]  funct3;

   // Branch
   wire branch_taken;
   wire [`WORD_WIDTH-1:0] branch_target;

   // Jumps
   wire [`WORD_WIDTH-1:0] jal_target;
   wire [`WORD_WIDTH-1:0] jalr_target;

   // U-Type
   wire [31:0] imm_upper;   // 20-bit immediate shifted to upper bits
   wire [`WORD_WIDTH-1:0] lui_result; 
   wire [`WORD_WIDTH-1:0] auipc_result;
   
   wire invalid_op;

   //
   // TODO: Using behavioral reg for now, make them into actual structural registers later
   // IF/ID, need to store PC and InstWord
   //
   // Pipeline registers
   // reg[63:0] pipeline_IF_ID; // Let's split this into 2 registers to make it easier to understand
   reg [31:0] pipeline_IF_ID_PC;
   reg [31:0] pipeline_IF_ID_InstWord;

   // ID/EX, need to store PC, Rdata1, Rdata2, immediate
   reg [31:0] pipeline_ID_EX_PC; 
   reg [31:0] pipeline_ID_EX_Rdata1, ID_EX_Rdata2; 
   reg [31:0] pipeline_ID_EX_immediate;
   reg [4:0]  pipeline_ID_EX_Rdst;
   // Control signals for next pipeline register
   reg pipeline_ID_EX_RegDst, pipeline_ID_EX_ALUOp1, pipeline_ID_EX_ALUOp0, pipeline_ID_EX_ALUSrc,
       pipeline_ID_EX_Branch, pipeline_ID_EX_Mem_Read, pipeline_ID_EX_Mem_Write,
       pipeline_ID_EX_Reg_Write, pipeline_ID_EX_Reg_Read;
   
   // EX/MEM, need to store PC, ALUresult, Data Address, data for store
   reg [31:0] pipeline_EX_MEM_PC; 
   reg [31:0] pipeline_EX_MEM_ALUresult;
   reg [31:0] pipeline_EX_MEM_DataAddr;
   reg [31:0] pipeline_EX_MEM_StoreData; 
   reg [4:0]  pipeline_EX_MEM_Rdst;
   // Control signals for next pipeline register
   reg pipeline_EX_MEM_Branch, pipeline_EX_MEM_Mem_Read, pipeline_EX_MEM_Mem_Write,
       pipeline_EX_MEM_Reg_Write, pipeline_EX_MEM_Reg_Read;

   // MEM/WB, need to store address, and data from load
   reg [31:0] pipeline_MEM_WB_DataAddr;
   reg [31:0] pipeline_MEM_WB_LoadData; 
   reg [4:0]  pipeline_MEM_WB_Rdst; // Write back to register file
   // Control signals for next pipeline register
   reg pipeline_MEM_WB_Reg_Write, pipeline_MEM_WB_Reg_Read;

   // Only supports R-TYPE and I-TYPE
   assign halt = invalid_op;

   assign Inv_R_type = ((opcode == `OPCODE_COMPUTE) && 
                       (((funct7 == `AUX_FUNC_SUB) && ((funct3 == `FUNC_ADD) || (funct3 == `FUNC_SRA)))||
                       ((funct7 == `AUX_FUNC_ADD) && ((funct3 == `FUNC_ADD)||(funct3 == `FUNC_XOR) || (funct3 == `FUNC_OR) || 
                       (funct3 == `FUNC_AND) || (funct3 == `FUNC_SLL) || (funct3 == `FUNC_SRL) || 
                       (funct3 == `FUNC_SLT) || (funct3 == `FUNC_SLTU)))
                        ));

   assign Inv_I_type = ((opcode == `OPCODE_COMPUTE_IMM) && ((funct3 == `FUNC_ADD || funct3 == `FUNC_XOR 
                         || funct3 == `FUNC_OR || funct3 == `FUNC_AND || funct3 == `FUNC_SLL || funct3 == `FUNC_SRL ||
                         funct3 == `FUNC_SRA || funct3 == `FUNC_SLT || funct3 == `FUNC_SLTU)))
                        || ((opcode == `OPCODE_JALR) && (funct3 == `FUNC_JALR)) ;

   assign Inv_Loads =  ((opcode == `OPCODE_LOAD) && ((funct3 == `FUNC_LB || funct3 == `FUNC_LBU) || 
                        ((funct3 == `FUNC_LW) && (DataAddr[1:0] == 2'b00)) || 
                        ((funct3 == `FUNC_LH|| funct3 == `FUNC_LHU) && (DataAddr[0] == 1'b0))));

   assign Inv_B_type = ((opcode == `OPCODE_BRANCH) && ((funct3 == `FUNC_BEQ) || (funct3 == `FUNC_BNE)
                        || (funct3 == `FUNC_BLT) || (funct3 == `FUNC_BGE) || (funct3 == `FUNC_BLTU) || (funct3 == `FUNC_BGEU)));
   
   assign Inv_J_type = (opcode == `OPCODE_JAL);

   assign Inv_U_type = ((opcode == `OPCODE_LUI) || (opcode == `OPCODE_AUIPC));

   assign Inv_S_type = ((opcode == `OPCODE_STORE) && ((funct3 == `FUNC_SB) || 
                        ((funct3 == `FUNC_SH) && (DataAddr[0] == 1'b0)) 
                        || ((funct3 == `FUNC_SW) && (DataAddr[1:0] == 2'b00))));

   assign Inv_MulDiv = ((opcode == `OPCODE_MULDIV) && (funct7 == `AUX_FUNC_M_EXT) && (funct3 == `FUNC_MUL || funct3 == `FUNC_MULH
                        || funct3 == `FUNC_MULSU || funct3 == `FUNC_MULU || funct3 == `FUNC_DIV || funct3 == `FUNC_DIVU 
                        || funct3 == `FUNC_REM || funct3 == `FUNC_REMU));

   assign invalid_op = !(Inv_R_type || Inv_I_type || Inv_Loads || Inv_B_type || Inv_J_type || Inv_U_type || Inv_S_type || Inv_MulDiv ); 
     
   // System State 
   Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	           .AddrB(Rsrc2), .DataOutB(Rdata2), 
	           .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WE(1'b1), .CLK(clk), .RST(rst));

   // Instruction Decode
   assign opcode = InstWord[6:0];   
   assign Rdst = InstWord[11:7]; 
   assign Rsrc1 = InstWord[19:15]; 
   assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
   assign immediate_i = {{20{InstWord[31]}}, InstWord[31:20]};  // I-type
   assign immediate_b = {{19{InstWord[31]}}, InstWord[31], InstWord[7], InstWord[30:25], InstWord[11:8], 1'b0};
   assign immediate_j = {{11{InstWord[31]}}, InstWord[31], InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0}; // J-type
   assign immediate_st = {{20{InstWord[31]}}, InstWord[31:25], InstWord[11:7]}; // S-type
   assign Rsrc2 = InstWord[24:20];
   assign funct7 = InstWord[31:25];  // R-Type
   assign imm_upper = {InstWord[31:12], 12'b0}; 


   assign immediate = (opcode == `OPCODE_STORE) ? immediate_st :
                      ((opcode == `OPCODE_LOAD || opcode == `OPCODE_COMPUTE_IMM)) ? immediate_i : 32'hXX;

   // control signal for R-type vs I-type instruction (0 for R, 1 for immediate)
   assign ALUSrc = !invalid_op && (opcode == `OPCODE_COMPUTE_IMM) ? 1'b1 : 1'b0;
   assign EACalc_control =  (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? 1'b1: 1'b0;
   assign muldiv_control = !invalid_op && (opcode == `OPCODE_MULDIV) && (funct7 == `AUX_FUNC_M_EXT) ? 1'b1: 1'b0;

   assign MemWrEn = !invalid_op && (opcode == `OPCODE_STORE);

   assign DataAddr = (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? EU_out: 32'h0000;

   assign StoreData = ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SB)) ? {{24{Rdata2[7]}}, Rdata2[7:0]} :
                      ((opcode == `OPCODE_STORE) && ((funct3 == `FUNC_SH))) ? {{16{Rdata2[15]}}, Rdata2[15:0]} :
                      ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SW)) ? Rdata2 : 32'hXX;

   assign load_size = ((opcode == `OPCODE_LOAD) && ((funct3 == `FUNC_LB) || (funct3 == `FUNC_LBU))) ? `SIZE_BYTE :
                      ((opcode == `OPCODE_LOAD) && ((funct3 == `FUNC_LH) || (funct3 == `FUNC_LHU))) ? `SIZE_HWORD :
                      ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LW)) ? `SIZE_WORD : 2'bXX;
   
   assign store_size = ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SB)) ? `SIZE_BYTE :
                       ((opcode == `OPCODE_STORE) && ((funct3 == `FUNC_SH))) ? `SIZE_HWORD :
                       ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SW)) ? `SIZE_WORD : 2'bXX;

   assign MemSize = (opcode == `OPCODE_LOAD) ? load_size :
                    (opcode == `OPCODE_STORE) ? store_size : 2'bXX;
                     
   assign RWrEn = (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_COMPUTE_IMM || opcode == `OPCODE_LOAD ||
                   opcode == `OPCODE_LUI || opcode == `OPCODE_AUIPC || opcode== `OPCODE_MULDIV ||
                   opcode == `OPCODE_JAL || opcode == `OPCODE_JALR);
   
   assign RWrdata = (opcode == `OPCODE_LOAD && funct3 == `FUNC_LW) ? DataWord :
                    (opcode == `OPCODE_LOAD && funct3 == `FUNC_LH) ? {{16{DataWord[15]}}, DataWord[15:0]} :
                    (opcode == `OPCODE_LOAD && funct3 == `FUNC_LB) ? {{24{DataWord[7]}}, DataWord[7:0]} :
                    (opcode == `OPCODE_LOAD && funct3 == `FUNC_LBU) ? {24'b0, DataWord[7:0]} :
                    (opcode == `OPCODE_LOAD && funct3 == `FUNC_LHU) ? {16'b0, DataWord[15:0]} :
                    (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_COMPUTE_IMM || opcode == `OPCODE_MULDIV) ? EU_out : 
                    (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR) ? PC_Plus_4 :
                    (opcode == `OPCODE_LUI) ? lui_result :
                    (opcode == `OPCODE_AUIPC) ? auipc_result :
                    32'hXXXXXXXX;

   // Supports B-Type instructions
   // Calculate branch target address
   assign branch_target = PC + immediate_b;
   assign branch_taken = (opcode == `OPCODE_BRANCH) && (
     (funct3 == `FUNC_BEQ  && (Rdata1 == Rdata2)) ||   // BEQ
     (funct3 == `FUNC_BNE  && (Rdata1 != Rdata2)) ||   // BNE
     (funct3 == `FUNC_BLT  && ($signed(Rdata1) < $signed(Rdata2))) || // BLT
     (funct3 == `FUNC_BGE  && ($signed(Rdata1) >= $signed(Rdata2))) || // BGE
     (funct3 == `FUNC_BLTU && (Rdata1 < Rdata2)) ||    // BLTU (unsigned)
     (funct3 == `FUNC_BGEU && (Rdata1 >= Rdata2))      // BGEU (unsigned)
   );

   // Jumps
   assign jal_target = PC + immediate_j;
   assign jalr_target = (Rdata1 + immediate_i) & ~1;

   // Supports U-Type instructions
   assign lui_result = imm_upper;
   assign auipc_result = PC + imm_upper;

   // Supports R-Type and I-type instructions and Loads
   
   ExecutionUnit EU(.out(EU_out),
                    .opA(Rdata1), 
                    .opB(Rdata2), 
                    .func(funct3), 
                    .auxFunc(funct7), 
                    .imm(immediate), 
                    .aluSrc(ALUSrc),
                    .EACalc(EACalc_control),
                    .MulDiv(muldiv_control));

   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign NPC = (opcode == `OPCODE_JAL)  ? jal_target :
                (opcode == `OPCODE_JALR) ? jalr_target :
                branch_taken             ? branch_target :
                PC_Plus_4;
   
endmodule // SingleCycleCPU

//
// TODO: I think we might remove the second ALU to simplify the pipeline,
// cause we would need separate control signals for handling it
//

// responsible for immediate calculations
module ALU_imm(out1, rs1, imm1, funca, eaCalc);
   output [`WORD_WIDTH-1:0] out1;
   input [`WORD_WIDTH-1:0]  rs1, imm1;
   input [2:0] 	 funca;
   input eaCalc;

   wire [`WORD_WIDTH-1:0] add = rs1 + imm1; 
   wire [`WORD_WIDTH-1:0] subtract = rs1 - imm1;
   wire [`WORD_WIDTH-1:0] ored = rs1 | imm1;
   wire [`WORD_WIDTH-1:0] anded = rs1 & imm1;
   wire [`WORD_WIDTH-1:0] xored = rs1 ^ imm1;
   wire [`WORD_WIDTH-1:0] signed_lt = ($signed(rs1) < $signed(imm1)) ? 32'b1 : 32'b0; // imm
   wire [`WORD_WIDTH-1:0] unsigned_lt = (rs1 < imm1) ? 32'b1 : 32'b0;
   wire [`WORD_WIDTH-1:0] sll = rs1 << imm1[4:0];
   wire [`WORD_WIDTH-1:0] srl = rs1 >> imm1[4:0];
   wire [`WORD_WIDTH-1:0] sra =($signed(rs1)) >>> imm1[4:0];
   
   assign out1 = (eaCalc)? add:
                (funca == 3'b000) ? add :
                (funca == 3'b100) ? xored:
                (funca == 3'b110) ? ored :
                (funca == 3'b111) ? anded:
                (funca == 3'b010) ? signed_lt :
                (funca == 3'b011) ? unsigned_lt :
                (funca == 3'b001 && imm1[11:5] == 7'b0000000) ? sll :
                (funca == 3'b101 && imm1[11:5] == 7'b0000000) ? srl :
                (funca == 3'b101 && imm1[11:5] == 7'b0100000) ? sra : 32'h00000000;
endmodule

module MultDiv (out3, inputA, inputB, funcc, auxFuncc);
   output [`WORD_WIDTH-1:0] out3;
   input  [`WORD_WIDTH-1:0]  inputA, inputB;
   input  [2:0] 	 funcc;
   input  [6:0] 	 auxFuncc;
   
   wire signed [(`WORD_WIDTH * 2)-1:0] signed_inputA = {{`WORD_WIDTH{inputA[`WORD_WIDTH-1]}}, inputA};
   wire signed [(`WORD_WIDTH * 2)-1:0] signed_inputB = {{`WORD_WIDTH{inputB[`WORD_WIDTH-1]}}, inputB};
   wire  [(`WORD_WIDTH * 2)-1:0] unsigned_inputA = {{`WORD_WIDTH{1'b0}}, inputA};
   wire  [(`WORD_WIDTH * 2)-1:0] unsigned_inputB = {{`WORD_WIDTH{1'b0}}, inputB};

   wire [(`WORD_WIDTH * 2)-1:0] mul = inputA * inputB;
   wire [(`WORD_WIDTH * 2)-1:0] mul_ss = signed_inputA * signed_inputB;
   wire [(`WORD_WIDTH * 2)-1:0] mul_su = signed_inputA * unsigned_inputB;
   wire [(`WORD_WIDTH * 2)-1:0] mul_uu = unsigned_inputA * unsigned_inputB;

   
   wire [`WORD_WIDTH-1:0] signed_div = $signed(inputA) / $signed(inputB);
   wire [`WORD_WIDTH-1:0] unsigned_div = inputA / inputB;
   wire [`WORD_WIDTH-1:0] signed_remainder = $signed(inputA) % $signed(inputB);
   wire [`WORD_WIDTH-1:0] unsigned_remainder = inputA % inputB;

   assign out3 = (funcc == 3'b000 && auxFuncc == 7'b0000001) ? mul[31:0] :
                 (funcc == 3'b001 && auxFuncc == 7'b0000001) ? mul_ss[63:32] :
                 (funcc == 3'b010 && auxFuncc == 7'b0000001) ? mul_su[63:32] :
                 (funcc == 3'b011 && auxFuncc == 7'b0000001) ? mul_uu[63:32] :
                 (funcc == 3'b100 && auxFuncc == 7'b0000001) ? signed_div:
                 (funcc == 3'b101 && auxFuncc == 7'b0000001) ? unsigned_div :
                 (funcc == 3'b110 && auxFuncc == 7'b0000001) ? signed_remainder :
                 (funcc == 3'b111 && auxFuncc == 7'b0000001) ? unsigned_remainder : 32'h00;
endmodule

module ALU_reg(out2, inA, inB, funcb, auxFuncb);
   output [`WORD_WIDTH-1:0] out2;
   input [`WORD_WIDTH-1:0]  inA, inB;
   input [2:0] 	 funcb;
   input [6:0] 	 auxFuncb;

   wire [`WORD_WIDTH-1:0] add = inA + inB; 
   wire [`WORD_WIDTH-1:0] subtract = inA - inB;
   wire [`WORD_WIDTH-1:0] ored = inA | inB;
   wire [`WORD_WIDTH-1:0] anded = inA & inB;
   wire [`WORD_WIDTH-1:0] xored = inA ^ inB;
   wire [`WORD_WIDTH-1:0] signed_lt = ($signed(inA) < $signed(inB)) ? 32'b1 : 32'b0; // imm
   wire [`WORD_WIDTH-1:0] unsigned_lt = (inA < inB) ? 32'b1 : 32'b0;
   wire [`WORD_WIDTH-1:0] sll = inA << inB[4:0];
   wire [`WORD_WIDTH-1:0] srl = inA >> inB[4:0];
   wire [`WORD_WIDTH-1:0] sra =($signed(inA)) >>> inB[4:0];

   
   assign out2 = (funcb == 3'b000 && auxFuncb == 7'b0000000) ? add :
                (funcb == 3'b000 && auxFuncb == 7'b0100000) ? subtract :
                (funcb == 3'b001 && auxFuncb == 7'b0000000) ? sll :
                (funcb == 3'b010 && auxFuncb == 7'b0000000) ? signed_lt :
                (funcb == 3'b011 && auxFuncb == 7'b0000000) ? unsigned_lt :
                (funcb == 3'b100 && auxFuncb == 7'b0000000) ? xored:
                (funcb == 3'b101 && auxFuncb == 7'b0000000) ? srl :
                (funcb == 3'b101 && auxFuncb == 7'b0100000) ? sra :
                (funcb == 3'b110 && auxFuncb == 7'b0000000) ? ored :
                (funcb == 3'b111 && auxFuncb == 7'b0000000) ? anded : 32'h00000000;
endmodule

module ALU_mux(out4, aluSrc1, eaCalc, muldiv, immresult, regresult, muldivresult);
   output [`WORD_WIDTH-1 :0] out4;
   input  aluSrc1, eaCalc, muldiv;
   input [`WORD_WIDTH-1:0]  immresult, regresult, muldivresult;

   assign out4 = (muldiv) ? muldivresult :
                 (eaCalc) ? immresult:
                 (aluSrc1)? immresult: regresult;
endmodule

module ExecutionUnit(out, opA, opB, func, auxFunc, imm, aluSrc, EACalc, MulDiv);
   output [`WORD_WIDTH-1:0] out;
   input [`WORD_WIDTH-1:0]  opA, opB, imm;
   input [2:0] 	 func;
   input [6:0] 	 auxFunc;
   input aluSrc, EACalc, MulDiv;
   
   wire [`WORD_WIDTH-1:0] imm_result;
   wire [`WORD_WIDTH-1:0] reg_result;
   wire [`WORD_WIDTH-1:0] muldiv_result;

   ALU_imm imm_ops(.out1(imm_result), 
                   .rs1(opA), 
                   .imm1(imm), 
                   .funca(func),
                   .eaCalc(EACalc));
   
   ALU_reg reg_ops(.out2(reg_result),
                   .inA(opA),
                  .inB(opB), 
                  .funcb(func), 
                  .auxFuncb(auxFunc));

   MultDiv multiplier(.out3(muldiv_result),
                      .inputA(opA),
                      .inputB(opB),
                      .funcc(func),
                      .auxFuncc(auxFunc));

   ALU_mux mux (.out4(out),
               .aluSrc1(aluSrc),
               .eaCalc(EACalc),
               .muldiv(MulDiv),
               .immresult(imm_result),
               .regresult(reg_result),
               .muldivresult(muldiv_result));

endmodule // ExecutionUnit
