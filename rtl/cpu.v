// Copyright (c) 2026 chznight
// SPDX-License-Identifier: MIT

module cpu(
    input wire clk,
    input wire rst,
    // Memory interface
    output wire [31:0] instr_addr,
    input wire [31:0] instruction,
    output wire [31:0] data_addr,
    output wire [31:0] data_out,
    input wire [31:0] data_in,
    output wire mem_write,
    output wire mem_read,
    output wire [3:0] byte_enable
);
    parameter PREDICTOR_PHT_BITS = 10;
    parameter PREDICTOR_BTB_BITS = 8;

    // Pipeline stage registers
    // IF/ID
    reg [31:0] IF_ID_PC;
    reg [31:0] IF_ID_Instruction;
    reg [31:0] IF_ID_predict_branch_target;
    reg [PREDICTOR_PHT_BITS-1:0] IF_ID_gshare_index;
    reg IF_ID_predict_taken;
    
    // ID/EX
    reg [31:0] ID_EX_PC;
    reg [4:0] ID_EX_Rs1;
    reg [4:0] ID_EX_Rs2;
    reg [4:0] ID_EX_Rd;
    reg [31:0] ID_EX_RegR1;
    reg [31:0] ID_EX_RegR2;
    reg [31:0] ID_EX_Imm;
    reg ID_EX_RegWrite;
    reg ID_EX_ALUSrc;
    reg [3:0] ID_EX_ALUOp;
    reg [2:0] ID_EX_Funct3;
    reg ID_EX_MemRead;
    reg ID_EX_MemWrite;
    reg ID_EX_MemtoReg;
    reg ID_EX_Branch;
    reg ID_EX_Jal;
    reg ID_EX_Jalr;
    reg ID_EX_Auipc;
    reg [31:0] ID_EX_predict_branch_target;
    reg ID_EX_predict_taken;
    reg [PREDICTOR_PHT_BITS-1:0] ID_EX_gshare_index;
    
    // EX/MEM
    reg [31:0] EX_MEM_BranchTarget;
    reg EX_MEM_Zero;
    reg [31:0] EX_MEM_ALUResult;
    reg [4:0] EX_MEM_Rd;
    reg [2:0] EX_MEM_Funct3;
    reg EX_MEM_RegWrite;
    reg EX_MEM_MemRead;
    reg EX_MEM_MemtoReg;
    reg EX_MEM_Branch;
    reg EX_MEM_Jal;
    reg EX_MEM_Jalr;
    reg [31:0] EX_MEM_PC;
    reg [31:0] EX_MEM_predict_branch_target;
    reg EX_MEM_predict_taken;
    reg [PREDICTOR_PHT_BITS-1:0] EX_MEM_gshare_index;

    // MEM/WB
    reg [31:0] MEM_WB_ReadData;
    reg [31:0] MEM_WB_ALUResult;
    reg [4:0] MEM_WB_Rd;
    reg MEM_WB_RegWrite;
    reg MEM_WB_MemtoReg;

    // Internal signals
    // IF stage
    reg [31:0] PC;
    reg [31:0] PC_next;
    reg [31:0] PC_in_flight;
    reg PC_in_flight_valid;
    wire branch_taken;
    
    // ID stage
    wire [31:0] reg_data1, reg_data2;
    wire [31:0] imm_ext;
    wire [3:0] alu_op;
    wire reg_write, mem_to_reg, alu_src, branch, mem_read_ctrl, mem_write_ctrl;
    reg [31:0] alu_in2_fwding_mux;
    // ALU input MUX for forwarding
    reg [31:0] alu_in1_fwding_mux;
    // EX stage
    wire [31:0] alu_in1, alu_in2;
    wire [31:0] alu_result;
    wire zero_flag;
    wire [31:0] branch_target;
    wire jal;
    wire jalr;
    wire auipc;
    wire [31:0] predict_branch_target;
    wire predict_taken;
    wire [31:0] load_result;
    reg [31:0] predict_branch_target_inflight;
    reg predict_taken_inflight;
    reg [PREDICTOR_PHT_BITS-1:0] gshare_index_inflight;

    wire branch_predict_missed;
    wire branch_target_missed;
    wire [PREDICTOR_PHT_BITS-1:0] branch_gshare_index;

    // Hazard detection unit signals
    wire stall;
    reg flush;
    
    // Forwarding unit signals
    wire [1:0] forward_a, forward_b;
    
    // Pipeline control signals
    reg pipeline_stall;

    reg [1:0] branch_type;
    
    // Fetch stage (IF)
    assign instr_addr = stall ? PC_in_flight : PC;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            PC <= 32'b0;
        end else if (flush) begin
            PC <= PC_next;
        end else if (!pipeline_stall) begin
            PC <= PC_next;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            PC_in_flight <= 32'b0;
            PC_in_flight_valid <= 1'b0;
            predict_branch_target_inflight <= 32'b0;
            predict_taken_inflight <= 1'b0;
            gshare_index_inflight <= {PREDICTOR_PHT_BITS{1'b0}};
        end else if (flush) begin
            PC_in_flight <= 32'b0;
            PC_in_flight_valid <= 1'b0;
            predict_branch_target_inflight <= 32'b0;
            predict_taken_inflight <= 1'b0;
            gshare_index_inflight <= {PREDICTOR_PHT_BITS{1'b0}};
        end else if (!pipeline_stall) begin
            PC_in_flight <= PC;
            PC_in_flight_valid <= 1'b1;
            predict_branch_target_inflight <= predict_branch_target;
            predict_taken_inflight <= predict_taken;
            gshare_index_inflight <= branch_gshare_index;
        end
    end

    always @(*) begin
        if (branch_predict_missed | branch_target_missed) begin
            if (branch_taken)
                PC_next = EX_MEM_BranchTarget;
            else
                PC_next = EX_MEM_PC + 4;
        end else begin
            if (predict_taken) begin
                PC_next = predict_branch_target;
            end else begin
                PC_next = PC + 4;
            end
        end
    end

    branch_predictor  #(
        .PHT_BITS(PREDICTOR_PHT_BITS), 
        .BTB_BITS(PREDICTOR_BTB_BITS)
    ) branch_predictor (
        .clk(clk),
        .reset(rst),
        .pc(PC),
        .predict_taken(predict_taken),
        .predict_branch_target(predict_branch_target),
        .update_predictor_pc(EX_MEM_PC),
        .update_predictor_branch_target(EX_MEM_BranchTarget),
        .update_pht_en(EX_MEM_Branch),
        .update_btb_en(EX_MEM_Branch | EX_MEM_Jal | EX_MEM_Jalr),
        .update_predictor_taken(branch_taken),
        .predictor_missed(branch_predict_missed),
        .predictor_target_missed(branch_target_missed),
        .instruction_type(branch_type),
        .branch_gshare_index(branch_gshare_index),
        .predictor_update_gshare_index(EX_MEM_gshare_index)
    );

    always @(*) begin
        if (EX_MEM_Branch)
            branch_type = 2'b01;
        else if (EX_MEM_Jal | EX_MEM_Jalr)
            branch_type = 2'b10;
        else
            branch_type = 2'b00;
    end
    /*
    assign branch_taken = ((EX_MEM_Branch) & 
                            (((EX_MEM_Funct3 == 3'b000) & (EX_MEM_Zero == 1)) |
                            ((EX_MEM_Funct3 == 3'b001) & (EX_MEM_Zero == 0))  |
                            ((EX_MEM_Funct3 == 3'b100) & (EX_MEM_Zero == 0))  |
                            ((EX_MEM_Funct3 == 3'b101) & (EX_MEM_Zero == 1))  |
                            ((EX_MEM_Funct3 == 3'b110) & (EX_MEM_Zero == 0))  |
                            ((EX_MEM_Funct3 == 3'b111) & (EX_MEM_Zero == 1))))|
                            (EX_MEM_Jal) |
                            (EX_MEM_Jalr);
    */
    assign branch_taken = (EX_MEM_Jal | EX_MEM_Jalr) | (EX_MEM_Branch & (EX_MEM_Zero ^ EX_MEM_Funct3[0] ^ EX_MEM_Funct3[2]));

    // IF/ID Pipeline Register
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            IF_ID_PC <= 32'b0;
            IF_ID_Instruction <= 32'b0;
            IF_ID_predict_branch_target <= 32'b0;
            IF_ID_predict_taken <= 0;
            IF_ID_gshare_index <= {PREDICTOR_PHT_BITS{1'b0}};
        end else if (!pipeline_stall) begin
            if (flush | !PC_in_flight_valid) begin
                IF_ID_Instruction <= 32'b0; // NOP on flush
                IF_ID_PC <= 32'b0;
                IF_ID_predict_branch_target <= 32'b0;
                IF_ID_predict_taken <= 0;
                IF_ID_gshare_index <= {PREDICTOR_PHT_BITS{1'b0}};
            end else begin
                IF_ID_PC <= PC_in_flight;
                IF_ID_Instruction <= instruction;
                IF_ID_predict_branch_target <= predict_branch_target_inflight;
                IF_ID_predict_taken <= predict_taken_inflight;
                IF_ID_gshare_index <= gshare_index_inflight;
            end
        end
    end
    
    // Decode stage (ID)
    control_unit control(
        .instruction(IF_ID_Instruction),
        .reg_write(reg_write),
        .mem_to_reg(mem_to_reg),
        .mem_read(mem_read_ctrl),
        .mem_write(mem_write_ctrl),
        .alu_op(alu_op),
        .alu_src(alu_src),
        .branch(branch),
        .jal(jal),
        .jalr(jalr),
        .auipc(auipc)
    );
    
    register_file registers(
        .clk(clk),
        .rst(rst),
        .reg_write(MEM_WB_RegWrite),
        .read_reg1(IF_ID_Instruction[19:15]), // rs1
        .read_reg2(IF_ID_Instruction[24:20]), // rs2
        .write_reg(MEM_WB_Rd),
        .write_data(MEM_WB_MemtoReg ? MEM_WB_ReadData : MEM_WB_ALUResult),
        .read_data1(reg_data1),
        .read_data2(reg_data2)
    );
    
    immediate_gen imm_gen(
        .instruction(IF_ID_Instruction),
        .imm_ext(imm_ext)
    );
    
    hazard_detection hazard_unit(
        .ID_EX_MemRead(ID_EX_MemRead),
        .EX_MEM_MemRead(EX_MEM_MemRead),
        .EX_MEM_Rd(EX_MEM_Rd),
        .ID_EX_Rd(ID_EX_Rd),
        .IF_ID_Rs1(IF_ID_Instruction[19:15]),
        .IF_ID_Rs2(IF_ID_Instruction[24:20]),
        .stall(stall)
    );
    
    // ID/EX Pipeline Register
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ID_EX_PC <= 32'b0;
            ID_EX_Rs1 <= 5'b0;
            ID_EX_Rs2 <= 5'b0;
            ID_EX_Rd <= 5'b0;
            ID_EX_RegR1 <= 32'b0;
            ID_EX_RegR2 <= 32'b0;
            ID_EX_Imm <= 32'b0;
            ID_EX_RegWrite <= 1'b0;
            ID_EX_ALUSrc <= 1'b0;
            ID_EX_ALUOp <= 4'b0;
            ID_EX_MemRead <= 1'b0;
            ID_EX_MemWrite <= 1'b0;
            ID_EX_MemtoReg <= 1'b0;
            ID_EX_Branch <= 1'b0;
            ID_EX_Jal <= 1'b0;
            ID_EX_Jalr <= 1'b0;
            ID_EX_Funct3 <= 3'b0;
            ID_EX_Auipc <= 1'b0;
            ID_EX_predict_branch_target <= 32'b0;
            ID_EX_predict_taken <= 0;
            ID_EX_gshare_index <= {PREDICTOR_PHT_BITS{1'b0}};
        end else if (flush || stall) begin
            ID_EX_PC <= 32'b0;
            ID_EX_Rs1 <= 5'b0;
            ID_EX_Rs2 <= 5'b0;
            ID_EX_Rd <= 5'b0;
            ID_EX_RegR1 <= 32'b0;
            ID_EX_RegR2 <= 32'b0;
            ID_EX_Imm <= 32'b0;
            ID_EX_RegWrite <= 1'b0;
            ID_EX_ALUSrc <= 1'b0;
            ID_EX_ALUOp <= 4'b0;
            ID_EX_MemRead <= 1'b0;
            ID_EX_MemWrite <= 1'b0;
            ID_EX_MemtoReg <= 1'b0;
            ID_EX_Branch <= 1'b0;
            ID_EX_Jal <= 1'b0;
            ID_EX_Jalr <= 1'b0;
            ID_EX_Funct3 <= 3'b0;
            ID_EX_Auipc <= 1'b0;
            ID_EX_predict_branch_target <= 32'b0;
            ID_EX_predict_taken <= 0;
            ID_EX_gshare_index <= {PREDICTOR_PHT_BITS{1'b0}};
        end else if (!pipeline_stall) begin
            ID_EX_PC <= IF_ID_PC;
            ID_EX_Rs1 <= IF_ID_Instruction[19:15];
            ID_EX_Rs2 <= IF_ID_Instruction[24:20];
            ID_EX_Rd <= IF_ID_Instruction[11:7];
            ID_EX_RegR1 <= reg_data1;
            ID_EX_RegR2 <= reg_data2;
            ID_EX_Imm <= imm_ext;
            ID_EX_RegWrite <= reg_write;
            ID_EX_ALUSrc <= alu_src;
            ID_EX_ALUOp <= alu_op;
            ID_EX_MemRead <= mem_read_ctrl;
            ID_EX_MemWrite <= mem_write_ctrl;
            ID_EX_MemtoReg <= mem_to_reg;
            ID_EX_Branch <= branch;
            ID_EX_Jal <= jal;
            ID_EX_Jalr <= jalr;
            ID_EX_Auipc <= auipc;
            ID_EX_Funct3 <= IF_ID_Instruction[14:12];
            ID_EX_predict_branch_target <= IF_ID_predict_branch_target;
            ID_EX_predict_taken <= IF_ID_predict_taken;
            ID_EX_gshare_index <= IF_ID_gshare_index;
        end
    end
    
    // Execute stage (EX)
    forwarding_unit forwarding(
        .EX_MEM_RegWrite(EX_MEM_RegWrite),
        .MEM_WB_RegWrite(MEM_WB_RegWrite),
        .EX_MEM_Rd(EX_MEM_Rd),
        .MEM_WB_Rd(MEM_WB_Rd),
        .ID_EX_Rs1(ID_EX_Rs1),
        .ID_EX_Rs2(ID_EX_Rs2),
        .ForwardA(forward_a),
        .ForwardB(forward_b)
    );
    

    always @(*) begin
        case(forward_a)
            2'b00: alu_in1_fwding_mux = ID_EX_RegR1;
            2'b01: alu_in1_fwding_mux = MEM_WB_MemtoReg ? MEM_WB_ReadData : MEM_WB_ALUResult;
            2'b10: alu_in1_fwding_mux = EX_MEM_ALUResult;
            default: alu_in1_fwding_mux = ID_EX_RegR1;
        endcase
    end
    
    always @(*) begin
        case(forward_b)
            2'b00: alu_in2_fwding_mux = ID_EX_RegR2;
            2'b01: alu_in2_fwding_mux = MEM_WB_MemtoReg ? MEM_WB_ReadData : MEM_WB_ALUResult;
            2'b10: alu_in2_fwding_mux = EX_MEM_ALUResult;
            default: alu_in2_fwding_mux = ID_EX_RegR2;
        endcase
    end
    
    // ALU source MUX
    assign alu_in1 = (ID_EX_Jal | ID_EX_Jalr | ID_EX_Auipc) ? ID_EX_PC: alu_in1_fwding_mux;
    assign alu_in2 = (ID_EX_Jal | ID_EX_Jalr) ? 32'd4 : (ID_EX_ALUSrc ? ID_EX_Imm : alu_in2_fwding_mux);

    alu alu_unit(
        .a(alu_in1),
        .b(alu_in2),
        .alu_op(ID_EX_ALUOp),
        .result(alu_result),
        .zero(zero_flag)
    );
    
    // Branch target calculation
    assign branch_target = ID_EX_Jalr ? ((alu_in1_fwding_mux + ID_EX_Imm) & ~32'h1) : (ID_EX_PC + ID_EX_Imm) ;
    
    // Memory stage (MEM)
    assign data_addr = alu_result;
    //assign data_out = alu_in2_fwding_mux;
    assign mem_write = ID_EX_MemWrite & !flush;
    assign mem_read = ID_EX_MemRead & !flush;

    memory_write_adapter memory_write_adapter (
        .mem_write(ID_EX_MemWrite),
        .mem_data_out(alu_in2_fwding_mux),
        .wr_addr(alu_result),
        .funct3(ID_EX_Funct3),
        .data_out(data_out),
        .byte_enable(byte_enable)
    );

    // EX/MEM Pipeline Register
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            EX_MEM_BranchTarget <= 32'b0;
            EX_MEM_Zero <= 1'b0;
            EX_MEM_ALUResult <= 32'b0;
            EX_MEM_Rd <= 5'b0;
            EX_MEM_RegWrite <= 1'b0;
            EX_MEM_MemRead <= 1'b0;
            EX_MEM_MemtoReg <= 1'b0;
            EX_MEM_Branch <= 1'b0;
            EX_MEM_Jal <= 1'b0;
            EX_MEM_Jalr <= 1'b0;
            EX_MEM_Funct3 <= 3'b0;
            EX_MEM_PC <= 32'b0;
            EX_MEM_predict_branch_target <= 32'b0;
            EX_MEM_predict_taken <= 0;
            EX_MEM_gshare_index <= {PREDICTOR_PHT_BITS{1'b0}};
        end else if (flush) begin
            EX_MEM_BranchTarget <= 32'b0;
            EX_MEM_Zero <= 1'b0;
            EX_MEM_ALUResult <= 32'b0;
            EX_MEM_Rd <= 5'b0;
            EX_MEM_RegWrite <= 1'b0;
            EX_MEM_MemRead <= 1'b0;
            EX_MEM_MemtoReg <= 1'b0;
            EX_MEM_Branch <= 1'b0;
            EX_MEM_Jal <= 1'b0;
            EX_MEM_Jalr <= 1'b0;
            EX_MEM_Funct3 <= 3'b0;
            EX_MEM_PC <= 32'b0;
            EX_MEM_predict_branch_target <= 32'b0;
            EX_MEM_predict_taken <= 0;
            EX_MEM_gshare_index <= {PREDICTOR_PHT_BITS{1'b0}};
        end else begin
            EX_MEM_BranchTarget <= branch_target;
            EX_MEM_Zero <= zero_flag;
            EX_MEM_ALUResult <= alu_result;
            EX_MEM_Rd <= ID_EX_Rd;
            EX_MEM_RegWrite <= ID_EX_RegWrite;
            EX_MEM_MemRead <= ID_EX_MemRead;
            EX_MEM_MemtoReg <= ID_EX_MemtoReg;
            EX_MEM_Branch <= ID_EX_Branch;
            EX_MEM_Jal <= ID_EX_Jal;
            EX_MEM_Jalr <= ID_EX_Jalr;
            EX_MEM_Funct3 <= ID_EX_Funct3;
            EX_MEM_PC <= ID_EX_PC;
            EX_MEM_predict_branch_target <= ID_EX_predict_branch_target;
            EX_MEM_predict_taken <= ID_EX_predict_taken;
            EX_MEM_gshare_index <= ID_EX_gshare_index;
        end
    end

    assign branch_predict_missed = (EX_MEM_Branch | EX_MEM_Jal | EX_MEM_Jalr) & (EX_MEM_predict_taken != branch_taken);
    assign branch_target_missed = (EX_MEM_Branch | EX_MEM_Jal | EX_MEM_Jalr) & branch_taken & (EX_MEM_predict_branch_target != EX_MEM_BranchTarget);
    
    memory_read_adapter memory_read_adapter (
        .data_in(data_in),
        .funct3(EX_MEM_Funct3),
        .addr_bits_lsb(EX_MEM_ALUResult[1:0]),
        .mem_read(EX_MEM_MemRead),
        .load_result(load_result)
    );
    
    // MEM/WB Pipeline Register
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            MEM_WB_ReadData <= 32'b0;
            MEM_WB_ALUResult <= 32'b0;
            MEM_WB_Rd <= 5'b0;
            MEM_WB_RegWrite <= 1'b0;
            MEM_WB_MemtoReg <= 1'b0;
        end else begin
            MEM_WB_ReadData <= load_result;
            MEM_WB_ALUResult <= EX_MEM_ALUResult;
            MEM_WB_Rd <= EX_MEM_Rd;
            MEM_WB_RegWrite <= EX_MEM_RegWrite;
            MEM_WB_MemtoReg <= EX_MEM_MemtoReg;
        end
    end

    // Pipeline control logic
    always @(*) begin
        pipeline_stall = stall;
        flush = (EX_MEM_Branch | EX_MEM_Jal | EX_MEM_Jalr) && (branch_predict_missed | branch_target_missed);
    end

endmodule 
