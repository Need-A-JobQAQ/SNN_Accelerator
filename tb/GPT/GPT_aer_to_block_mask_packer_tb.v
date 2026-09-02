`timescale 1ns/1ps

module GPT_aer_to_block_mask_packer_tb;

    localparam CLK_PERIOD = 10;
    localparam P_EVENT_ADDR_WIDTH = 11;
    localparam P_BLOCK_SIZE = 32;
    localparam P_BLOCK_ID_WIDTH = 6;

    reg clk;
    reg rst_n;
    reg i_clear;

    reg i_event_valid;
    reg [P_EVENT_ADDR_WIDTH-1:0] i_event_addr;
    reg i_event_frame_done;
    wire o_event_ready;

    wire o_block_valid;
    wire [P_BLOCK_ID_WIDTH-1:0] o_block_id;
    wire [P_BLOCK_SIZE-1:0] o_block_mask;
    reg i_block_ready;
    wire o_block_frame_done;

    integer error_count;
    integer recv_count;

    reg [P_BLOCK_ID_WIDTH-1:0] recv_block_id [0:15];
    reg [P_BLOCK_SIZE-1:0] recv_block_mask [0:15];

    aer_to_block_mask_packer #(
        .P_EVENT_ADDR_WIDTH (P_EVENT_ADDR_WIDTH),
        .P_BLOCK_SIZE       (P_BLOCK_SIZE),
        .P_BLOCK_ID_WIDTH   (P_BLOCK_ID_WIDTH)
    ) u_dut (
        .clk                (clk),
        .rst_n              (rst_n),
        .i_clear            (i_clear),
        .i_event_valid      (i_event_valid),
        .i_event_addr       (i_event_addr),
        .i_event_frame_done (i_event_frame_done),
        .o_event_ready      (o_event_ready),
        .o_block_valid      (o_block_valid),
        .o_block_id         (o_block_id),
        .o_block_mask       (o_block_mask),
        .i_block_ready      (i_block_ready),
        .o_block_frame_done (o_block_frame_done)
    );

    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // 收集被后级真正接收的 block 包。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            recv_count <= 0;
        end else if (i_clear) begin
            recv_count <= 0;
        end else if (o_block_valid && i_block_ready) begin
            recv_block_id[recv_count] <= o_block_id;
            recv_block_mask[recv_count] <= o_block_mask;
            recv_count <= recv_count + 1;
        end
    end

    task send_event;
        input [P_EVENT_ADDR_WIDTH-1:0] addr;
        begin
            i_event_addr <= addr;
            i_event_valid <= 1'b1;
            while (!o_event_ready) begin
                @(posedge clk);
            end
            @(posedge clk);
            i_event_valid <= 1'b0;
            i_event_addr <= {P_EVENT_ADDR_WIDTH{1'b0}};
        end
    endtask

    task send_frame_done;
        begin
            i_event_frame_done <= 1'b1;
            @(posedge clk);
            i_event_frame_done <= 1'b0;
        end
    endtask

    task check_block;
        input integer index;
        input [P_BLOCK_ID_WIDTH-1:0] exp_id;
        input [P_BLOCK_SIZE-1:0] exp_mask;
        begin
            if (recv_block_id[index] !== exp_id || recv_block_mask[index] !== exp_mask) begin
                $display("[%0t ns] SIM_ERROR: block[%0d] mismatch, got id=%0d mask=%h, expected id=%0d mask=%h.",
                         $time, index, recv_block_id[index], recv_block_mask[index], exp_id, exp_mask);
                error_count = error_count + 1;
            end
        end
    endtask

    initial begin
        $display("[%0t ns] SIM_INFO: GPT_aer_to_block_mask_packer_tb start.", $time);

        error_count = 0;
        i_clear = 1'b0;
        i_event_valid = 1'b0;
        i_event_addr = {P_EVENT_ADDR_WIDTH{1'b0}};
        i_event_frame_done = 1'b0;
        i_block_ready = 1'b1;

        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // 用连续地址流验证：同一个 block 内事件被合并，跨 block 时刷出旧包。
        send_event(11'd0);
        send_event(11'd2);
        send_event(11'd5);
        send_event(11'd31);
        send_event(11'd32);
        send_event(11'd33);
        send_event(11'd70);
        send_frame_done();
        repeat (4) @(posedge clk);

        if (recv_count !== 3) begin
            $display("[%0t ns] SIM_ERROR: expected 3 blocks, got %0d.", $time, recv_count);
            error_count = error_count + 1;
        end
        check_block(0, 6'd0, 32'h8000_0025);
        check_block(1, 6'd1, 32'h0000_0003);
        check_block(2, 6'd2, 32'h0000_0040);

        if (o_block_frame_done !== 1'b0) begin
            $display("[%0t ns] SIM_ERROR: frame_done should be a pulse, but still high.", $time);
            error_count = error_count + 1;
        end

        // 清空后验证反压：旧 block 未被接收时，新 block 事件不能被吞掉。
        i_clear <= 1'b1;
        @(posedge clk);
        i_clear <= 1'b0;
        repeat (2) @(posedge clk);

        send_event(11'd0);
        send_event(11'd1);

        i_block_ready <= 1'b0;
        i_event_addr <= 11'd32;
        i_event_valid <= 1'b1;
        @(posedge clk);
        if (o_event_ready !== 1'b0 || o_block_valid !== 1'b1) begin
            $display("[%0t ns] SIM_ERROR: backpressure failed, ready=%b block_valid=%b.",
                     $time, o_event_ready, o_block_valid);
            error_count = error_count + 1;
        end

        i_block_ready <= 1'b1;
        while (!o_event_ready) begin
            @(posedge clk);
        end
        @(posedge clk);
        i_event_valid <= 1'b0;
        i_event_addr <= {P_EVENT_ADDR_WIDTH{1'b0}};
        send_frame_done();
        repeat (4) @(posedge clk);

        if (recv_count !== 2) begin
            $display("[%0t ns] SIM_ERROR: backpressure case expected 2 blocks, got %0d.", $time, recv_count);
            error_count = error_count + 1;
        end
        check_block(0, 6'd0, 32'h0000_0003);
        check_block(1, 6'd1, 32'h0000_0001);

        // 清空后验证空帧：没有 pending block 时，frame_done 也应产生一个结束脉冲。
        i_clear <= 1'b1;
        @(posedge clk);
        i_clear <= 1'b0;
        repeat (2) @(posedge clk);
        send_frame_done();
        @(posedge clk);
        if (o_block_frame_done !== 1'b1) begin
            $display("[%0t ns] SIM_ERROR: empty frame did not generate block_frame_done.", $time);
            error_count = error_count + 1;
        end
        @(posedge clk);

        if (error_count == 0) begin
            $display("[%0t ns] SIM_PASS: aer_to_block_mask_packer basic cases passed.", $time);
        end else begin
            $display("[%0t ns] SIM_FAIL: aer_to_block_mask_packer failed, error_count=%0d.", $time, error_count);
        end
        $finish;
    end

endmodule
