`timescale 1ns/1ps

module GPT_block_event_arbiter_tb;

    localparam CLK_PERIOD = 10;
    localparam P_NUM_PORTS = 4;
    localparam P_BLOCK_ID_WIDTH = 6;
    localparam P_BLOCK_SIZE = 32;

    reg clk;
    reg rst_n;
    reg i_clear;
    reg [P_NUM_PORTS-1:0] i_block_valid;
    reg [P_NUM_PORTS-1:0][P_BLOCK_ID_WIDTH-1:0] i_block_id;
    reg [P_NUM_PORTS-1:0][P_BLOCK_SIZE-1:0] i_block_mask;
    wire [P_NUM_PORTS-1:0] o_block_ready;
    wire o_block_valid;
    wire [P_BLOCK_ID_WIDTH-1:0] o_block_id;
    wire [P_BLOCK_SIZE-1:0] o_block_mask;
    reg i_block_ready;

    integer error_count;

    block_event_arbiter #(
        .P_NUM_PORTS       (P_NUM_PORTS),
        .P_BLOCK_ID_WIDTH  (P_BLOCK_ID_WIDTH),
        .P_BLOCK_SIZE      (P_BLOCK_SIZE)
    ) u_dut (
        .clk               (clk),
        .rst_n             (rst_n),
        .i_clear           (i_clear),
        .i_block_valid     (i_block_valid),
        .i_block_id        (i_block_id),
        .i_block_mask      (i_block_mask),
        .o_block_ready     (o_block_ready),
        .o_block_valid     (o_block_valid),
        .o_block_id        (o_block_id),
        .o_block_mask      (o_block_mask),
        .i_block_ready     (i_block_ready)
    );

    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    task clear_inputs;
        begin
            i_block_valid = {P_NUM_PORTS{1'b0}};
            i_block_id[0] = 6'd10;
            i_block_id[1] = 6'd11;
            i_block_id[2] = 6'd12;
            i_block_id[3] = 6'd13;
            i_block_mask[0] = 32'h0000_0001;
            i_block_mask[1] = 32'h0000_0002;
            i_block_mask[2] = 32'h0000_0004;
            i_block_mask[3] = 32'h0000_0008;
        end
    endtask

    task check_select;
        input [P_BLOCK_ID_WIDTH-1:0] exp_id;
        input [P_BLOCK_SIZE-1:0] exp_mask;
        input [P_NUM_PORTS-1:0] exp_ready;
        input [127:0] tag;
        begin
            #1;
            if (o_block_valid !== 1'b1 || o_block_id !== exp_id || o_block_mask !== exp_mask || o_block_ready !== exp_ready) begin
                $display("[%0t ns] SIM_ERROR: %0s mismatch, valid=%b id=%0d mask=%h ready=%b, expected id=%0d mask=%h ready=%b.",
                         $time, tag, o_block_valid, o_block_id, o_block_mask, o_block_ready, exp_id, exp_mask, exp_ready);
                error_count = error_count + 1;
            end
        end
    endtask

    initial begin
        $display("[%0t ns] SIM_INFO: GPT_block_event_arbiter_tb start.", $time);
        error_count = 0;
        i_clear = 1'b0;
        i_block_ready = 1'b0;
        clear_inputs();

        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // 后级不 ready 时，仍能选出有效 block，但不能向前级释放 ready。
        i_block_valid = 4'b1010;
        check_select(6'd11, 32'h0000_0002, 4'b0000, "not_ready");

        // 后级 ready 后，只给被选中的端口 ready。
        i_block_ready = 1'b1;
        check_select(6'd11, 32'h0000_0002, 4'b0010, "first_select");
        @(posedge clk);

        // 轮询指针前进后，应从端口2开始寻找，跳过无效端口后选端口3。
        check_select(6'd13, 32'h0000_0008, 4'b1000, "second_select");
        @(posedge clk);

        // 再下一次绕回后，仍选择端口1。
        check_select(6'd11, 32'h0000_0002, 4'b0010, "wrap_select");
        @(posedge clk);

        // clear 后轮询指针回到端口0，此时端口0/2有效，应优先选端口0。
        @(negedge clk);
        i_clear = 1'b1;
        @(posedge clk);
        @(negedge clk);
        i_clear = 1'b0;
        i_block_valid = 4'b0101;
        check_select(6'd10, 32'h0000_0001, 4'b0001, "after_clear");

        // 没有有效输入时，输出 valid 应为 0。
        i_block_valid = 4'b0000;
        #1;
        if (o_block_valid !== 1'b0 || o_block_ready !== 4'b0000) begin
            $display("[%0t ns] SIM_ERROR: output should be idle when no input is valid, valid=%b ready=%b.",
                     $time, o_block_valid, o_block_ready);
            error_count = error_count + 1;
        end

        if (error_count == 0) begin
            $display("[%0t ns] SIM_PASS: block_event_arbiter round-robin cases passed.", $time);
        end else begin
            $display("[%0t ns] SIM_FAIL: block_event_arbiter failed, error_count=%0d.", $time, error_count);
        end
        $finish;
    end

endmodule




