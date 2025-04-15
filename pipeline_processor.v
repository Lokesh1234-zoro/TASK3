`timescale 1ns / 1ps

module pipelined_processor (
    input clk,              // Clock signal
    input rst,              // Reset signal
    output reg [31:0] pc    // Program Counter (visible for debugging)
);

// Pipeline stage registers
reg [31:0] if_id_reg;       // IF/ID: Holds instruction
reg [31:0] id_ex_reg [3:0]; // ID/EX: Opcode, Rsrc1_val, Rsrc2_val, Rdest/Imm
reg [31:0] ex_wb_reg [1:0]; // EX/WB: ALU result, Rdest

// Register file
reg [31:0] reg_file [0:7];  // 8 registers, 32 bits each
integer i;

// Instruction memory (preloaded with example program)
reg [31:0] instr_mem [0:15];
initial begin
    instr_mem[0] = 32'h11230000; // ADD R1, R2, R3
    instr_mem[1] = 32'h22100000; // SUB R2, R1, R0
    instr_mem[2] = 32'h33100004; // LOAD R3, R1, 4
end

// Data memory (for LOAD, simplified)
reg [31:0] data_mem [0:15];
initial begin
    data_mem[12] = 32'd10; // Value at address 12 for LOAD
end

// Wires for intermediate signals
wire [3:0] opcode;
wire [3:0] rdest, rsrc1, rsrc2;
wire [15:0] imm;
wire [31:0] rsrc1_val, rsrc2_val, alu_result;
reg [31:0] result;
// Decode instruction fields
assign opcode = if_id_reg[31:28];
assign rdest  = if_id_reg[27:24];
assign rsrc1  = if_id_reg[23:20];
assign rsrc2  = if_id_reg[19:16];
assign imm    = if_id_reg[15:0];

// Register file read (combinational)
assign rsrc1_val = reg_file[rsrc1];
assign rsrc2_val = reg_file[rsrc2];

// ALU (combinational)
always @(*) begin
    case (id_ex_reg[0][31:28]) // Opcode from ID/EX
        4'b0001: result = id_ex_reg[1] + id_ex_reg[2]; // ADD
        4'b0010: result = id_ex_reg[1] - id_ex_reg[2]; // SUB
        4'b0011: result = data_mem[id_ex_reg[1] + id_ex_reg[3]]; // LOAD (address from Rsrc1 + Imm)
        default: result = 32'b0;
    endcase
end
assign alu_result=result;
// Pipeline stages
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // Reset state
        pc <= 0;
        if_id_reg <= 0;
        for (i = 0; i < 4; i = i + 1) id_ex_reg[i] <= 0;
        for (i = 0; i < 2; i = i + 1) ex_wb_reg[i] <= 0;
        for (i = 0; i < 8; i = i + 1) reg_file[i] <= 0;
        reg_file[2] <= 5; // Initial values for simulation
        reg_file[3] <= 3;
    end else begin
        // Stage 1: Instruction Fetch (IF)
        if_id_reg <= instr_mem[pc >> 2]; // Fetch instruction, PC increments by 4
        pc <= pc + 4;

        // Stage 2: Instruction Decode (ID)
        id_ex_reg[0] <= {opcode, rdest, 20'b0}; // Opcode and Rdest
        id_ex_reg[1] <= rsrc1_val;              // Rsrc1 value
        id_ex_reg[2] <= rsrc2_val;              // Rsrc2 value
        id_ex_reg[3] <= {{16{imm[15]}}, imm};   // Sign-extended immediate

        // Stage 3: Execute (EX)
        ex_wb_reg[0] <= alu_result;             // ALU result
        ex_wb_reg[1] <= id_ex_reg[0][27:24];   // Rdest

        // Stage 4: Write Back (WB)
        if (ex_wb_reg[1] != 0)                  // Don't write to R0
            reg_file[ex_wb_reg[1]] <= ex_wb_reg[0];
    end
end

endmodule

//// Testbench for simulation
//module tb_pipelined_processor;
//    reg clk, rst;
//    wire [31:0] pc;

//    pipelined_processor uut (
//        .clk(clk),
//        .rst(rst),
//        .pc(pc)
//    );

//    // Clock generation
//    always #5 clk = ~clk;

//    initial begin
//        // Initialize signals
//        clk = 0;
//        rst = 1;
//        #10 rst = 0;

//        // Run simulation for 10 cycles
//        #100 $display("Final Register State:");
//        $display("R0: %d", uut.reg_file[0]);
//        $display("R1: %d", uut.reg_file[1]);
//        $display("R2: %d", uut.reg_file[2]);
//        $display("R3: %d", uut.reg_file[3]);
//        $stop;
//    end

//    // Monitor pipeline progress
//    always @(posedge clk) begin
//        $display("Cycle: PC=%d, IF/ID=%h, ID/EX_Op=%h, EX/WB_Result=%d",
//                 pc, uut.if_id_reg, uut.id_ex_reg[0], uut.ex_wb_reg[0]);
//    end
//endmodule



//module PipelinedProcessor(
//    input clk, reset
//);

//    // Instruction Memory
//    reg [31:0] instruction_memory [0:15];
//    reg [31:0] register_file [0:7];
//    reg [31:0] data_memory [0:15];

//    // Pipeline Registers
//    reg [31:0] IF_ID_IR, IF_ID_PC;
//    reg [31:0] ID_EX_A, ID_EX_B, ID_EX_IMM;
//    reg [2:0] ID_EX_RD;
//    reg [31:0] EX_MEM_ALU_OUT, EX_MEM_B;
//    reg [2:0] EX_MEM_RD;
//    reg [31:0] MEM_WB_ALU_OUT;
//    reg [2:0] MEM_WB_RD;
//    reg MEM_WB_RegWrite;

//    integer i;

//    initial begin
//        // Initialize instruction memory (Example Program)
//        instruction_memory[0] = 32'b000000_00001_00010_00011_00000_100000; // ADD R3, R1, R2
//        instruction_memory[1] = 32'b000000_00011_00001_00100_00000_100010; // SUB R4, R3, R1
//        instruction_memory[2] = 32'b100011_00001_00101_0000000000000100;  // LOAD R5, 4(R1)

//        // Initialize registers
//        for (i = 0; i < 8; i = i + 1)
//            register_file[i] = i;
//    end

//    // IF Stage
//    reg [31:0] PC = 0;
//    always @(posedge clk or posedge reset) begin
//        if (reset) PC <= 0;
//        else begin
//            IF_ID_IR <= instruction_memory[PC >> 2];
//            IF_ID_PC <= PC;
//            PC <= PC + 4;
//        end
//    end

//    // ID Stage
//    always @(posedge clk) begin
//        ID_EX_A <= register_file[IF_ID_IR[25:21]];
//        ID_EX_B <= register_file[IF_ID_IR[20:16]];
//        ID_EX_IMM <= {{16{IF_ID_IR[15]}}, IF_ID_IR[15:0]};
//        ID_EX_RD <= IF_ID_IR[15:11];
//    end

//    // EX Stage
//    always @(posedge clk) begin
//        case (IF_ID_IR[31:26])
//            6'b000000: // R-Type (ADD, SUB)
//                if (IF_ID_IR[5:0] == 6'b100000) // ADD
//                    EX_MEM_ALU_OUT <= ID_EX_A + ID_EX_B;
//                else if (IF_ID_IR[5:0] == 6'b100010) // SUB
//                    EX_MEM_ALU_OUT <= ID_EX_A - ID_EX_B;
//            6'b100011: // LOAD
//                EX_MEM_ALU_OUT <= ID_EX_A + ID_EX_IMM;
//        endcase
//        EX_MEM_B <= ID_EX_B;
//        EX_MEM_RD <= ID_EX_RD;
//    end

//    // MEM Stage
//    always @(posedge clk) begin
//        if (IF_ID_IR[31:26] == 6'b100011) // LOAD
//            MEM_WB_ALU_OUT <= data_memory[EX_MEM_ALU_OUT >> 2];
//        else
//            MEM_WB_ALU_OUT <= EX_MEM_ALU_OUT;
//        MEM_WB_RD <= EX_MEM_RD;
//        MEM_WB_RegWrite <= 1;
//    end

//    // WB Stage
//    always @(posedge clk) begin
//        if (MEM_WB_RegWrite)
//            register_file[MEM_WB_RD] <= MEM_WB_ALU_OUT;
//    end

//endmodule
