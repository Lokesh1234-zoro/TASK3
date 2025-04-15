# CODETECH-TASK3    :pipeline peocessor
**COMPANY**         :CODETECH IT SOLUTIONS
**NAME**            :MAMILLAPALLI LOKESH
**INTERN ID**       :CT04WA64
**DOMAIN**          :VLSI
**BATCH DURATION**  :MARCH 15th,2025 TO APRIL 15th,2025
**MENTOR**          :NELLA SANTHOSH KUMAR

**OVERVIEW**

This project implements a 4-stage pipelined processor in Verilog, supporting basic instructions such as:

ADD (Addition)

SUB (Subtraction)

LOAD (Memory Load)

**Overview of the 4-Stage Pipeline**
A pipelined processor breaks instruction execution into discrete stages, allowing multiple instructions to be processed concurrently. Our 4-stage pipeline will consist of:

IF (Instruction Fetch): Fetch the instruction from memory.
ID (Instruction Decode): Decode the instruction and read operands from registers.
EX (Execute): Perform the arithmetic or logic operation.
WB (Write Back): Write the result back to the register file.
Each stage takes one clock cycle, and with pipelining, a new instruction can enter the pipeline every cycle once it’s fully populated.

**Basic Components**
Instruction Memory: Stores the program (e.g., a simple array of 32-bit instructions).
Program Counter (PC): Points to the current instruction, incremented by 4 (assuming 32-bit instructions).
Register File: Contains 8 registers (R0–R7), each 32 bits wide. R0 is hardwired to 0.
ALU (Arithmetic Logic Unit): Performs ADD and SUB operations.
Pipeline Registers: Buffers between stages (IF/ID, ID/EX, EX/WB) to hold intermediate data.

**Instruction Set**
We’ll use a simple 32-bit RISC-like instruction format:

ADD Rdest, Rsrc1, Rsrc2: Adds Rsrc1 and Rsrc2, stores result in Rdest.
SUB Rdest, Rsrc1, Rsrc2: Subtracts Rsrc2 from Rsrc1, stores result in Rdest.
LOAD Rdest, Rsrc, Imm: Loads a value from memory address (Rsrc + Imm) into Rdest.

**Instruction Encoding:**

Bits 31–28: Opcode (ADD: 0001, SUB: 0010, LOAD: 0011)
Bits 27–24: Rdest (destination register)
Bits 23–20: Rsrc1 (first source register)
Bits 19–16: Rsrc2 (second source register or unused for LOAD)
Bits 15–0: Immediate (Imm, used only for LOAD, sign-extended)
Example:

ADD R1, R2, R3: 0001 0001 0010 0011 0000…0000 (hex: 0x11230000)
SUB R2, R1, R0: 0010 0010 0001 0000 0000…0000 (hex: 0x22100000)
LOAD R3, R1, 4: 0011 0011 0001 0000 0000…0100 (hex: 0x33100004)

 **Pipeline Stages in Detail**
**Stage 1: IF (Instruction Fetch)**
Input: PC value.
Operation: Fetch instruction from Instruction Memory at address PC. Increment PC by 4.
Output (to IF/ID register): Instruction, updated PC.
**Stage 2: ID (Instruction Decode)**
Input: Instruction from IF/ID register.
Operation: Decode opcode and operands. Read Rsrc1 and Rsrc2 from Register File.
Output (to ID/EX register): Opcode, Rsrc1 value, Rsrc2 value, Rdest, Immediate (if LOAD).
**Stage 3: EX (Execute)**
Input: Data from ID/EX register.
Operation:
ADD: ALU computes Rsrc1 + Rsrc2.
SUB: ALU computes Rsrc1 – Rsrc2.
LOAD: ALU computes address = Rsrc1 + Imm.
Output (to EX/WB register): ALU result, Rdest.
**Stage 4: WB (Write Back)**
Input: Data from EX/WB register.
Operation: Write ALU result to Rdest in Register File (except for LOAD, which would fetch from memory in a full design, but we’ll simplify here).
Output: None (updates Register File).

Step-by-Step Explanation:

Cycle 1: IF fetches ADD, PC = 4.
Cycle 2: IF fetches SUB, ID decodes ADD (reads R2=5, R3=3), PC = 8.
Cycle 3: IF fetches LOAD, ID decodes SUB (reads R1=0, R0=0), EX executes ADD (5+3=8).
Cycle 4: IF idle, ID decodes LOAD (reads R1=8), EX executes SUB (8-0=8), WB writes 8 to R1.
Cycle 5: IF idle, ID idle, EX executes LOAD (8+4=12, assume memory[12]=10), WB writes 8 to R2.
Cycle 6: IF idle, ID idle, EX idle, WB writes 10 to R3.
Final Register State:

R0 = 0, R1 = 8, R2 = 8, R3 = 10

**Functional Design Summary**
This 4-stage pipelined processor successfully executes ADD, SUB, and LOAD instructions. Each stage operates in one cycle, overlapping execution to achieve higher throughput. The simulation demonstrates the pipeline filling and draining, with correct results written back to registers.



