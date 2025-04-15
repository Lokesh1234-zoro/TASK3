module tb_pipelined_processor;
    reg clk, rst;
    wire [31:0] pc;

    pipelined_processor uut (
        .clk(clk),
        .rst(rst),
        .pc(pc)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;
        #10 rst = 0;

        // Run simulation for 10 cycles
        #100 $display("Final Register State:");
        $display("R0: %d", uut.reg_file[0]);
        $display("R1: %d", uut.reg_file[1]);
        $display("R2: %d", uut.reg_file[2]);
        $display("R3: %d", uut.reg_file[3]);
        $stop;
    end

    // Monitor pipeline progress
    always @(posedge clk) begin
        $display("Cycle: PC=%d, IF/ID=%h, ID/EX_Op=%h, EX/WB_Result=%d",
                 pc, uut.if_id_reg, uut.id_ex_reg[0], uut.ex_wb_reg[0]);
    end
endmodule