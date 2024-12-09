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

// ---------------------------- MEM_SIZES --------------------------------------
`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10
// ------------------------------------------------------------------------------
`define NOP 32'h00000013 // for stalls
`define ALU_CONTROL_WIDTH 5

module PipelinedCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [`WORD_WIDTH-1:0] PC_ID, PC, InstWord, InstWord_fetch;
   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4;
   wire [`WORD_WIDTH-1:0] DataAddr, StoreData, DataWord;
   wire [1:0]  MemSize, load_size, store_size;
   wire        MemWrEn, MemReadEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;
   wire [`WORD_WIDTH-1:0]  Rdata1, Rdata2, RWrdata;
   wire [`WORD_WIDTH-1 :0] immediate, immediate_i, immediate_j, immediate_b, immediate_st;
   wire                    RWrEn, MemToReg, RegRead;
   wire                    ALUSrc, EACalc_control, muldiv_control;
   wire                    Inv_R_type, Inv_I_type, Inv_Loads, Inv_B_type, Inv_J_type, Inv_U_type, Inv_S_type, Inv_MulDiv;

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

   
   // stall (WE) -- hold ; on the divs and mults
   // we >> jumps in the last stage
   // if we need something pushed out off the pipeline, put nops after it

   // ------------------------------------------------ IF/ID ----------------------------------------------------------------
   // need to store PC and InstWord

   // Fetch Address Datapath
   
   assign PC_Plus_4 = PC + 4;
   assign NPC = (opcode == `OPCODE_JAL)  ? jal_target :
                (opcode == `OPCODE_JALR) ? jalr_target :
                branch_taken             ? branch_target :
                PC_Plus_4;

   // Pipeline registers
   // reg[63:0] pipeline_IF_ID; // Let's split this into 2 registers to make it easier to understand
   Reg #(.width(32)) PC_Reg(.Din(NPC), .Qout(PC), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) pipeline_IF_ID_PC(.Din(PC), .Qout(PC_ID), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) pipeline_IF_ID_InstWord(.Din(InstWord), .Qout(InstWord_fetch), .WE(1'b1), .CLK(clk), .RST(rst));

   Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
              .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   
   // ------------------------------------------------ ID/EX ----------------------------------------------------------------
   assign opcode = InstWord_out[6:0];   
   assign Rdst = InstWord_out[11:7]; 
   assign Rsrc1 = InstWord_out[19:15]; 
   assign Rsrc2 = InstWord_out[24:20];
   assign funct3 = InstWord_out[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = InstWord_out[31:25];  // R-Type

   assign immediate_i = {{20{InstWord_out[31]}}, InstWord_out[31:20]};  // I-type
   assign immediate_b = {{19{InstWord_out[31]}}, InstWord_out[31], InstWord_out[7], InstWord_out[30:25], InstWord_out[11:8], 1'b0};
   assign immediate_j = {{11{InstWord_out[31]}}, InstWord_out[31], InstWord_out[19:12], InstWord_out[20], InstWord_out[30:21], 1'b0}; // J-type
   assign immediate_st = {{20{InstWord_out[31]}}, InstWord_out[31:25], InstWord_out[11:7]}; // S-type
   assign imm_upper = {InstWord_out[31:12], 12'b0};  // U-type

   assign immediate = (opcode == `OPCODE_STORE) ? immediate_st :
                      ((opcode == `OPCODE_LOAD || opcode == `OPCODE_COMPUTE_IMM)) ? immediate_i : 
                      (opcode == `OPCODE_BRANCH) ? immediate_b : 
                      (opcode ==  `OPCODE_JAL || opcode == `OPCODE_JALR) ? immediate_j : imm_upper;
   
   wire [`WORD_WIDTH-1:0] R1_data_out, PC_out, immediate_out, R2_data_out, InstWord_out;
   wire [4:0] Rdst_out;
   wire [2:0] funct3_out;
   wire [6:0] funct7_out;
   
   // ID/EX, need to store PC, Rdata1, Rdata2, immediate
   Reg #(.width(32)) pipeline_ID_EX_InstWord(.Din(InstWord_fetch), .Qout(InstWord_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) pipeline_ID_EX_PC(.Din(PC_ID), .Qout(PC_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) pipeline_ID_EX_Rdata1(.Din(Rdata1), .Qout(R1_data_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) ID_EX_Rdata2 (.Din(Rdata2), .Qout(R2_data_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) pipeline_ID_EX_immediate(.Din(immediate), .Qout(immediate_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(5))  pipeline_ID_EX_Rdst (.Din(Rdst), .Qout(Rdst_out), .WE(1'b1), .CLK(clk), .RST(rst)); // to WB stage
   Reg #(.width(3))  pipeline_ID_EX_funct3 (.Din(funct3), .Qout(funct3_out), .WE(1'b1), .CLK(clk), .RST(rst)); // to EX stage
   Reg #(.width(7))  pipeline_ID_EX_funct7 (.Din(funct7), .Qout(funct7_out), .WE(1'b1), .CLK(clk), .RST(rst)); // to EX stage
              
   // Control signals for Next Stage Pipeline Register
   Reg #(.width(2)) pipeline_ID_EX_ALUOp(.Din(ALUOp), .Qout(ALUOp_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_ID_EX_ALUSrc(.Din(ALUSrc), .Qout(ALUSrc_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_ID_EX_Branch(.Din(branch_taken), .Qout(branch_taken_out), .WE(1'b1), .CLK(clk), .RST(rst)); 
   Reg #(.width(1)) pipeline_ID_EX_Mem_Read(.Din(MemReadEn), .Qout(mem_read_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_ID_EX_Mem_Write(.Din(MemWrEn), .Qout(mem_write_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_ID_EX_Reg_Write(.Din(RWrEn), .Qout(reg_write_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_ID_EX_Mem_To_Reg(.Din(MemToReg), .Qout(mem_to_reg_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_ID_EX_Reg_Read(.Din(RegRead), .Qout(reg_read_out), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_ID_EX_Jump(.Din(isJump), .Qout(isJump_out), .WE(1'b1), .CLK(clk), .RST(rst));

   wire branch_taken_out, mem_read_out, mem_write_out,reg_write_out, mem_to_reg_out, reg_read_out;
   wire ALUSrc_out, isJump_out;
   wire [1:0] ALUOp, ALUOp_out;

   assign ALUOp  = (!invalid_op && (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE || opcode == `OPCODE_JAL || 
                                   opcode == `OPCODE_JALR)) ? 2'b00 :
                   (!invalid_op && (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_MULDIV || opcode == `OPCODE_COMPUTE_IMM)) ? 2'b10 : 2'b11;

   assign branch_taken = (opcode == `OPCODE_BRANCH) && (
                         (funct3 == `FUNC_BEQ  && (Rdata1 == Rdata2)) ||   // BEQ
                         (funct3 == `FUNC_BNE  && (Rdata1 != Rdata2)) ||   // BNE
                         (funct3 == `FUNC_BLT  && ($signed(Rdata1) < $signed(Rdata2))) || // BLT
                         (funct3 == `FUNC_BGE  && ($signed(Rdata1) >= $signed(Rdata2))) || // BGE
                         (funct3 == `FUNC_BLTU && (Rdata1 < Rdata2)) ||    // BLTU (unsigned)
                         (funct3 == `FUNC_BGEU && (Rdata1 >= Rdata2)));     // BGEU (unsigned)

   assign MemReadEn =  !invalid_op && (opcode == `OPCODE_LOAD);

   assign MemWrEn = !invalid_op && (opcode == `OPCODE_STORE);
  
   assign MemToReg = !invalid_op && (opcode == `OPCODE_LOAD);
   
   assign RegRead = !invalid_op && (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_COMPUTE_IMM || opcode == `OPCODE_LOAD || 
                    opcode == `OPCODE_STORE || opcode == `OPCODE_BRANCH);

   assign RWrEn = !invalid_op && (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_COMPUTE_IMM || opcode == `OPCODE_LOAD ||
                   opcode == `OPCODE_LUI || opcode == `OPCODE_AUIPC || opcode == `OPCODE_MULDIV ||
                   opcode == `OPCODE_JAL || opcode == `OPCODE_JALR);
   
   assign isJump = !invalid_op && (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR);

   assign ALUSrc = !invalid_op && (opcode == `OPCODE_COMPUTE_IMM || opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? 1'b1 : 1'b0;

   // ------------------------------------------------ EX/MEM ----------------------------------------------------------------
   // need to store PC, ALUresult, Data Address, data for store
   wire [`WORD_WIDTH-1:0] PC_mem, ALU_result_mem, DataAddr_mem, StoreData_mem;
   wire [4:0] Rdst_mem;
   wire branch_mem, mem_read_mem, mem_write_mem, reg_write_mem, mem_to_reg_mem, reg_read_mem, isJump_mem;

   Reg #(.width(32)) pipeline_EX_MEM_PC (.Din(PC_out), .Qout(PC_mem), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) pipeline_EX_MEM_ALUresult (.Din(ALU_result), .Qout(ALU_result_mem), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) pipeline_EX_MEM_DataAddr (.Din(DataAddr), .Qout(DataAddr_mem), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) pipeline_EX_MEM_StoreData (.Din(StoreData), .Qout(StoreData_mem), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(5))  pipeline_EX_MEM_Rdst(.Din(Rdst_out), .Qout(Rdst_mem), .WE(1'b1), .CLK(clk), .RST(rst));

   assign StoreData = ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SB)) ? {{24{Rdata2[7]}}, Rdata2[7:0]} :
                      ((opcode == `OPCODE_STORE) && ((funct3 == `FUNC_SH))) ? {{16{Rdata2[15]}}, Rdata2[15:0]} :
                      ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SW)) ? Rdata2 : 32'hXX;

   assign DataAddr = (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? ALU_result: 32'h0000;

   // Control signals for next pipeline register
   Reg #(.width(1)) pipeline_EX_MEM_Branch (.Din(branch_taken_out), .Qout(branch_taken), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_EX_MEM_Mem_Read (.Din(mem_read_out), .Qout(mem_read_mem), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_EX_MEM_Mem_Write (.Din(mem_write_out), .Qout(mem_write_mem), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_EX_MEM_Reg_Write (.Din(reg_write_out), .Qout(reg_write_mem), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_EX_MEM_Reg_Read (.Din(reg_read_out), .Qout(reg_read_mem), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_EX_MEM_Mem_To_Reg (.Din(mem_to_reg_out), .Qout(mem_to_reg_mem), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_EX_MEM_Jump(.Din(isJump_out), .Qout(isJump_mem), .WE(1'b1), .CLK(clk), .RST(rst));

   // ------------------------------------------------ MEM/WB ----------------------------------------------------------------
   // need to store address, and data from load, and data to be written into rd
   wire [`WORD_WIDTH-1:0] LoadData_wb, ALU_result_wb, DataAddr_wb;
   wire [4:0] Rdst_wb;
   wire reg_write_wb, reg_read_wb, isJump_wb;;

   Reg #(.width(32)) pipeline_MEM_WB_ALUresult (.Din(ALU_result_mem), .Qout(ALU_result_wb), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) pipeline_MEM_WB_DataAddr (.Din(DataAddr_mem), .Qout(DataAddr_wb), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(32)) pipeline_MEM_WB_LoadData (.Din(DataWord), .Qout(LoadData_wb), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(5))  pipeline_MEM_WB_Rdst (.Din(Rdst_mem), .Qout(Rdst_wb), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1))  pipeline_MEM_WB_Jump(.Din(isJump_mem), .Qout(isJump_wb), .WE(1'b1), .CLK(clk), .RST(rst));

   // Control signals for WB stage
   Reg #(.width(1)) pipeline_MEM_WB_Reg_Write (.Din(reg_write_mem), .Qout(reg_write_wb), .WE(1'b1), .CLK(clk), .RST(rst));
   Reg #(.width(1)) pipeline_MEM_WB_Reg_Read (.Din(reg_read_mem), .Qout(reg_write_wb), .WE(1'b1), .CLK(clk), .RST(rst));

   // System State
   Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
              .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
	           .AddrB(Rsrc2), .DataOutB(Rdata2), 
	           .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   assign EACalc_control =  (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) ? 1'b1: 1'b0;
   assign muldiv_control = !invalid_op && (opcode == `OPCODE_MULDIV) && (funct7 == `AUX_FUNC_M_EXT) ? 1'b1: 1'b0;

   assign load_size = ((opcode == `OPCODE_LOAD) && ((funct3 == `FUNC_LB) || (funct3 == `FUNC_LBU))) ? `SIZE_BYTE :
                      ((opcode == `OPCODE_LOAD) && ((funct3 == `FUNC_LH) || (funct3 == `FUNC_LHU))) ? `SIZE_HWORD :
                      ((opcode == `OPCODE_LOAD) && (funct3 == `FUNC_LW)) ? `SIZE_WORD : 2'bXX;
   
   assign store_size = ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SB)) ? `SIZE_BYTE :
                       ((opcode == `OPCODE_STORE) && ((funct3 == `FUNC_SH))) ? `SIZE_HWORD :
                       ((opcode == `OPCODE_STORE) && (funct3 == `FUNC_SW)) ? `SIZE_WORD : 2'bXX;

   assign MemSize = (opcode == `OPCODE_LOAD) ? load_size :
                    (opcode == `OPCODE_STORE) ? store_size : 2'bXX;
   
   assign RWrdata = (opcode == `OPCODE_LOAD && funct3 == `FUNC_LW) ? DataWord :
                    (opcode == `OPCODE_LOAD && funct3 == `FUNC_LH) ? {{16{DataWord[15]}}, DataWord[15:0]} :
                    (opcode == `OPCODE_LOAD && funct3 == `FUNC_LB) ? {{24{DataWord[7]}}, DataWord[7:0]} :
                    (opcode == `OPCODE_LOAD && funct3 == `FUNC_LBU) ? {24'b0, DataWord[7:0]} :
                    (opcode == `OPCODE_LOAD && funct3 == `FUNC_LHU) ? {16'b0, DataWord[15:0]} :
                    (opcode == `OPCODE_COMPUTE || opcode == `OPCODE_COMPUTE_IMM || opcode == `OPCODE_MULDIV) ? ALU_result : 
                    (opcode == `OPCODE_JAL || opcode == `OPCODE_JALR) ? PC_Plus_4 :
                    (opcode == `OPCODE_LUI) ? lui_result :
                    (opcode == `OPCODE_AUIPC) ? auipc_result :
                    32'hXXXXXXXX;

   // Supports B-Type instructions
   // Calculate branch target address
   assign branch_target = PC_ID + immediate_b;
  
   // Jumps
   assign jal_target = PC_ID + immediate_j;
   assign jalr_target = (Rdata1 + immediate_i) & ~1;

   // Supports U-Type instructions
   assign lui_result = imm_upper;
   assign auipc_result = PC_ID + imm_upper;

   // Supports R-Type and I-type instructions and Loads
   wire [`WORD_WIDTH-1:0] ALU_result;
   ExecutionUnit EU(.out(ALU_result),
                    .opA(R1_data_out), 
                    .opB(R2_data_out), 
                    .funct3(funct3_out), 
                    .funct7(funct7_out), 
                    .imm(immediate_out), 
                    .aluSrc(ALUSrc_out),
                    .aluOp(ALUOp_out));

endmodule // SingleCycleCPU

//
// ALU
// Handles all R-type instructions (including multiplications for simplicity),
// also used to calculate effective address
//
module ALU(out,             // output
           inA, inB,        // inputs
           ALUControl); // control
   output [`WORD_WIDTH-1:0] out;
   input  [`WORD_WIDTH-1:0] inA, inB;
   input  [`ALU_CONTROL_WIDTH-1:0] ALUControl;

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

   wire signed [(`WORD_WIDTH * 2)-1:0] signed_inA = {{`WORD_WIDTH{inA[`WORD_WIDTH-1]}}, inA};
   wire signed [(`WORD_WIDTH * 2)-1:0] signed_inB = {{`WORD_WIDTH{inB[`WORD_WIDTH-1]}}, inB};
   wire  [(`WORD_WIDTH * 2)-1:0] unsigned_inA = {{`WORD_WIDTH{1'b0}}, inA};
   wire  [(`WORD_WIDTH * 2)-1:0] unsigned_inB = {{`WORD_WIDTH{1'b0}}, inB};

   wire [(`WORD_WIDTH * 2)-1:0] mul = inA * inB;
   wire [(`WORD_WIDTH * 2)-1:0] mul_ss = signed_inA * signed_inB;
   wire [(`WORD_WIDTH * 2)-1:0] mul_su = signed_inA * unsigned_inB;
   wire [(`WORD_WIDTH * 2)-1:0] mul_uu = unsigned_inA * unsigned_inB;

   wire [`WORD_WIDTH-1:0] signed_div = $signed(inA) / $signed(inB);
   wire [`WORD_WIDTH-1:0] unsigned_div = inA / inB;
   wire [`WORD_WIDTH-1:0] signed_remainder = $signed(inA) % $signed(inB);
   wire [`WORD_WIDTH-1:0] unsigned_remainder = inA % inB;
   
   assign out = (ALUControl == 5'b00000) ? add :
                (ALUControl == 5'b00001) ? subtract :
                (ALUControl == 5'b00010) ? sll :
                (ALUControl == 5'b00011) ? signed_lt :
                (ALUControl == 5'b00100) ? unsigned_lt :
                (ALUControl == 5'b00101) ? xored:
                (ALUControl == 5'b00110) ? srl :
                (ALUControl == 5'b00111) ? sra :
                (ALUControl == 5'b01000) ? ored :
                (ALUControl == 5'b01001) ? anded : 
                (ALUControl == 5'b01010) ? mul[31:0] : // M extension
                (ALUControl == 5'b01011) ? mul_ss[63:32] :
                (ALUControl == 5'b01100) ? mul_su[63:32] :
                (ALUControl == 5'b01101) ? mul_uu[63:32] :
                (ALUControl == 5'b01110) ? signed_div:
                (ALUControl == 5'b01111) ? unsigned_div :
                (ALUControl == 5'b10000) ? signed_remainder :
                (ALUControl == 5'b10000) ? unsigned_remainder : 32'h00000000;
endmodule

//
// aluSrc = 0 -> out = rs2
// aluSrc = 1 -> out = imm
//
module ALU_imm_mux(out,       // output
                   rs2, imm,  // inputs
                   aluSrc);   // control
    output [`WORD_WIDTH-1:0] out;
    input  [`WORD_WIDTH-1:0] rs2, imm;
    input  aluSrc;
    assign out = (aluSrc) ? imm : rs2;
endmodule

module ALU_forward_mux_1();

endmodule

module ALU_forward_mux_2();

endmodule

module ALUControl(ALUControl, funct3, funct7, ALUOp);
    output [`ALU_CONTROL_WIDTH-1:0] ALUControl;
    input  [2:0] funct3;
    input  [6:0] funct7;
    input  [1:0] ALUOp;
    assign ALUControl = (ALUOp == 2'b00) ? 5'b00000 : // loads, stores, jal, jalr -> add
                        // Other ALUOp? If we move branches forward, no need for ALU
                        (ALUOp == 2'b10) ? // R-type instructions, need to check funct3 and funct7
                        ((funct3 == 3'b000 && funct7 == 7'b0000000) ? 5'b00000  : // add
                         (funct3 == 3'b000 && funct7 == 7'b0100000) ? 5'b00001  : // sub
                         (funct3 == 3'b001 && funct7 == 7'b0000000) ? 5'b00010  : // sll
                         (funct3 == 3'b010 && funct7 == 7'b0000000) ? 5'b00011  : // signed_lt
                         (funct3 == 3'b011 && funct7 == 7'b0000000) ? 5'b00100  : // unsigned_lt
                         (funct3 == 3'b100 && funct7 == 7'b0000000) ? 5'b00101  : // xored
                         (funct3 == 3'b101 && funct7 == 7'b0000000) ? 5'b00110  : // srl
                         (funct3 == 3'b101 && funct7 == 7'b0100000) ? 5'b00111  : // sra
                         (funct3 == 3'b110 && funct7 == 7'b0000000) ? 5'b01000  : // ored
                         (funct3 == 3'b111 && funct7 == 7'b0000000) ? 5'b01001  : // anded
                         (funct3 == 3'b000 && funct7 == 7'b0000001) ? 5'b01010  : // mul
                         (funct3 == 3'b001 && funct7 == 7'b0000001) ? 5'b01011  : // mul_ss
                         (funct3 == 3'b010 && funct7 == 7'b0000001) ? 5'b01100  : // mul_su
                         (funct3 == 3'b011 && funct7 == 7'b0000001) ? 5'b01101  : // mul_uu
                         (funct3 == 3'b100 && funct7 == 7'b0000001) ? 5'b01110  : // signed_div
                         (funct3 == 3'b101 && funct7 == 7'b0000001) ? 5'b01111  : // unsigned_div
                         (funct3 == 3'b110 && funct7 == 7'b0000001) ? 5'b10000  : // signed_remainder
                         (funct3 == 3'b111 && funct7 == 7'b0000001) ? 5'b10001 : 5'b00000) : // unsigned_remainder
                          5'b00000; // default to add

endmodule

module ExecutionUnit(out, opA, opB, funct3, funct7, imm, aluSrc, aluOp);
   output [`WORD_WIDTH-1:0] out;
   input  [`WORD_WIDTH-1:0] opA, opB, imm;
   input  [2:0] funct3;
   input  [6:0] funct7;
   input  [1:0] aluOp;
   input  aluSrc;

   wire [`WORD_WIDTH-1:0] opB_muxed;
   wire [`ALU_CONTROL_WIDTH-1:0] ALUControl;
   
   ALU alu(.out(out),
           .inA(opA),
           .inB(opB_muxed), 
           .ALUControl(ALUControl));

   ALUControl alu_control(.ALUControl(ALUControl),
                          .funct3(funct3),
                          .funct7(funct7),
                          .ALUOp(aluOp));

   ALU_imm_mux imm_mux(.out(opB_muxed),
                       .rs2(opB), .imm(imm),
                       .aluSrc(aluSrc));

endmodule // ExecutionUnit
