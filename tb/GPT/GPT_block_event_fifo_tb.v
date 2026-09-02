`timescale 1ns/1ps

module GPT_block_event_fifo_tb;

    localparam CLK_PERIOD = 10;
    localparam P_BLOCK_ID_WIDTH = 6;
    localparam P_BLOCK_SIZE = 32;
    localparam P_FIFO_DEPTH = 4;

    reg clk;
    reg rst_n;
    reg i_clear;

    reg i_block_valid;
    reg [P_BLOCK_ID_WIDTH-1:0] i_block_id;
    reg [P_BLOCK_SIZE-1:0] i_block_mask;
    reg i_block_ready;

    wire o_block_valid;
    wire [P_BLOCK_ID_WIDTH-1:0] o_block_id;
    wire [P_BLOCK_SIZE-1:0] o_block_mask;
    wire o_empty;
    wire o_full;
    wire [$clog2(P_FIFO_DEPTH + 1)-1:0] o_count;
    wire o_overflow;

    integer error_count;

    block_event_fifo #(
        .P_BLOCK_ID_WIDTH (P_BLOCK_ID_WIDTH),
        .P_BLOCK_SIZE     (P_BLOCK_SIZE),
        .P_FIFO_DEPTH     (P_FIFO_DEPTH)
    ) u_dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .i_clear          (i_clear),
        .i_block_valid    (i_block_valid),
        .i_block_id       (i_block_id),
        .i_block_mask     (i_block_mask),
        .i_block_ready    (i_block_ready),
        .o_block_valid    (o_block_valid),
        .o_block_id       (o_block_id),
        .o_block_mask     (o_block_mask),
        .o_empty          (o_empty),
        .o_full           (o_full),
        .o_count          (o_count),
        .o_overflow       (o_overflow)
    );

    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    task push_block;
        input [P_BLOCK_ID_WIDTH-1:0] block_id;
        input [P_BLOCK_SIZE-1:0] block_mask;
        begin
            i_block_id <= block_id;
            i_block_mask <= block_mask;
            i_block_valid <= 1'b1;
            @(posedge clk);
            i_block_valid <= 1'b0;
            i_block_id <= {P_BLOCK_ID_WIDTH{1'b0}};
            i_block_mask <= {P_BLOCK_SIZE{1'b0}};
            #1;
        end
    endtask

    task pop_check;
        input [P_BLOCK_ID_WIDTH-1:0] exp_id;
        input [P_BLOCK_SIZE-1:0] exp_mask;
        begin
            #1;
            if (o_block_valid !== 1'b1 || o_block_id !== exp_id || o_block_mask !== exp_mask) begin
                $display("[%0t ns] SIM_ERROR: pop mismatch, valid=%b id=%0d mask=%h, expected id=%0d mask=%h.",
                         $time, o_block_valid, o_block_id, o_block_mask, exp_id, exp_mask);
                error_count = error_count + 1;
            end
            i_block_ready <= 1'b1;
            @(posedge clk);
            i_block_ready <= 1'b0;
            #1;
        end
    endtask

    initial begin
        $display("[%0t ns] SIM_INFO: GPT_block_event_fifo_tb start.", $time);

        error_count = 0;
        i_clear = 1'b0;
        i_block_valid = 1'b0;
        i_block_id = {P_BLOCK_ID_WIDTH{1'b0}};
        i_block_mask = {P_BLOCK_SIZE{1'b0}};
        i_block_ready = 1'b0;

        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        if (o_empty !== 1'b1 || o_count !== 0) begin
            $display("[%0t ns] SIM_ERROR: fifo should be empty after reset.", $time);
            error_count = error_count + 1;
        end

        // 基本写入和先进先出读取。
        push_block(6'd1, 32'h0000_0003);
        push_block(6'd2, 32'h8000_0000);
        if (o_count !== 2) begin
            $display("[%0t ns] SIM_ERROR: expected count=2, got %0d.", $time, o_count);
            error_count = error_count + 1;
        end
        pop_check(6'd1, 32'h0000_0003);
        pop_check(6'd2, 32'h8000_0000);

        // 同一拍读写时，FIFO 数量应保持不变，旧数据先被读出。
        push_block(6'd3, 32'h0000_00f0);
        #1;
        i_block_id <= 6'd4;
        i_block_mask <= 32'h0000_0f00;
        i_block_valid <= 1'b1;
        i_block_ready <= 1'b1;
        @(posedge clk);
        i_block_valid <= 1'b0;
        i_block_ready <= 1'b0;
        i_block_id <= {P_BLOCK_ID_WIDTH{1'b0}};
        i_block_mask <= {P_BLOCK_SIZE{1'b0}};
        #1;
        if (o_count !== 1) begin
            $display("[%0t ns] SIM_ERROR: simultaneous read/write should keep count=1, got %0d.", $time, o_count);
            error_count = error_count + 1;
        end
        pop_check(6'd4, 32'h0000_0f00);

        // 写满 FIFO 后继续写入，应置 overflow，已有数据不应被覆盖。
        push_block(6'd10, 32'h0000_0001);
        push_block(6'd11, 32'h0000_0002);
        push_block(6'd12, 32'h0000_0004);
        push_block(6'd13, 32'h0000_0008);
        if (o_full !== 1'b1 || o_count !== P_FIFO_DEPTH) begin
            $display("[%0t ns] SIM_ERROR: fifo should be full, full=%b count=%0d.", $time, o_full, o_count);
            error_count = error_count + 1;
        end
        push_block(6'd14, 32'h0000_0010);
        if (o_overflow !== 1'b1) begin
            $display("[%0t ns] SIM_ERROR: overflow should be set when pushing into full fifo.", $time);
            error_count = error_count + 1;
        end
        pop_check(6'd10, 32'h0000_0001);
        pop_check(6'd11, 32'h0000_0002);
        pop_check(6'd12, 32'h0000_0004);
        pop_check(6'd13, 32'h0000_0008);

        // clear 应清空 FIFO，并清除 overflow。
        push_block(6'd20, 32'hffff_ffff);
        i_clear <= 1'b1;
        @(posedge clk);
        i_clear <= 1'b0;
        #1;
        if (o_empty !== 1'b1 || o_count !== 0 || o_overflow !== 1'b0) begin
            $display("[%0t ns] SIM_ERROR: clear failed, empty=%b count=%0d overflow=%b.",
                     $time, o_empty, o_count, o_overflow);
            error_count = error_count + 1;
        end

        if (error_count == 0) begin
            $display("[%0t ns] SIM_PASS: block_event_fifo basic cases passed.", $time);
        end else begin
            $display("[%0t ns] SIM_FAIL: block_event_fifo failed, error_count=%0d.", $time, error_count);
        end
        $finish;
    end

endmodule
