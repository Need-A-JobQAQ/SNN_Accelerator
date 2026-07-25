`timescale 1ns/1ps

module spike_vector_to_aer_tb;

    localparam CLK_PERIOD = 10;
    localparam P_NUM_SPIKES = 8;
    localparam P_ADDR_WIDTH = 3;

    reg tb_clk;
    reg tb_rst_n;
    reg tb_start;
    reg [P_NUM_SPIKES-1:0] tb_spike_vector;
    reg tb_event_ready;

    wire tb_event_valid;
    wire [P_ADDR_WIDTH-1:0] tb_event_addr;
    wire tb_frame_done;
    wire tb_busy;

    integer event_count;
    integer frame_done_count;
    integer error_count;

    reg [P_ADDR_WIDTH-1:0] expected_addr [0:3];

    spike_vector_to_aer #(
        .P_NUM_SPIKES (P_NUM_SPIKES),
        .P_ADDR_WIDTH (P_ADDR_WIDTH)
    ) dut (
        .clk             (tb_clk),
        .rst_n           (tb_rst_n),
        .i_start         (tb_start),
        .i_spike_vector  (tb_spike_vector),
        .i_event_ready   (tb_event_ready),
        .o_event_valid   (tb_event_valid),
        .o_event_addr    (tb_event_addr),
        .o_frame_done    (tb_frame_done),
        .o_busy          (tb_busy)
    );

    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    /*
     * 测试向量为 8'b1010_0101。
     * 最高位对应逻辑地址 0，因此期望事件地址依次为 0、2、5、7。
     */
    initial begin
        expected_addr[0] = 3'd0;
        expected_addr[1] = 3'd2;
        expected_addr[2] = 3'd5;
        expected_addr[3] = 3'd7;

        event_count = 0;
        frame_done_count = 0;
        error_count = 0;

        tb_rst_n = 1'b0;
        tb_start = 1'b0;
        tb_spike_vector = 8'b0000_0000;
        tb_event_ready = 1'b1;

        $display("[%0t ns] SIM_INFO: spike_vector_to_aer_tb start.", $time);

        repeat (5) @(posedge tb_clk);
        tb_rst_n = 1'b1;
        repeat (2) @(posedge tb_clk);

        tb_spike_vector = 8'b1010_0101;
        tb_start = 1'b1;
        @(posedge tb_clk);
        tb_start = 1'b0;

        /*
         * 在地址 2 这个事件处故意拉低 ready 两拍。
         * 正确行为是 valid 和 addr 保持不变，直到 ready 恢复。
         */
        wait (tb_event_valid && tb_event_addr == 3'd2);
        #1;
        tb_event_ready = 1'b0;
        $display("[%0t ns] SIM_INFO: event_ready deasserted at addr %0d.", $time, tb_event_addr);
        repeat (2) @(posedge tb_clk);

        if (!(tb_event_valid && tb_event_addr == 3'd2)) begin
            $display("[%0t ns] SIM_ERROR: ready=0 时事件地址没有保持在 2。当前 valid=%b addr=%0d",
                     $time, tb_event_valid, tb_event_addr);
            error_count = error_count + 1;
        end

        #1;
        tb_event_ready = 1'b1;
        $display("[%0t ns] SIM_INFO: event_ready asserted again.", $time);

        wait (tb_frame_done);
        @(posedge tb_clk);
        repeat (2) @(posedge tb_clk);

        if (event_count != 4) begin
            $display("[%0t ns] SIM_ERROR: 事件数量错误，期望 4，实际 %0d。", $time, event_count);
            error_count = error_count + 1;
        end

        if (frame_done_count != 1) begin
            $display("[%0t ns] SIM_ERROR: frame_done 脉冲数量错误，期望 1，实际 %0d。", $time, frame_done_count);
            error_count = error_count + 1;
        end

        if (error_count == 0) begin
            $display("[%0t ns] SIM_PASS: spike_vector_to_aer_tb passed.", $time);
        end else begin
            $display("[%0t ns] SIM_FAIL: spike_vector_to_aer_tb failed, error_count=%0d.", $time, error_count);
        end

        $finish;
    end

    /*
     * 只在 ready=1 且 valid=1 时统计一个事件被后级接收。
     */
    always @(posedge tb_clk or negedge tb_rst_n) begin
        if (!tb_rst_n) begin
            event_count <= 0;
        end else if (tb_event_valid && tb_event_ready) begin
            if (event_count < 4) begin
                if (tb_event_addr != expected_addr[event_count]) begin
                    $display("[%0t ns] SIM_ERROR: 第 %0d 个事件地址错误，期望 %0d，实际 %0d。",
                             $time, event_count, expected_addr[event_count], tb_event_addr);
                    error_count = error_count + 1;
                end else begin
                    $display("[%0t ns] SIM_INFO: accept event[%0d], addr=%0d.",
                             $time, event_count, tb_event_addr);
                end
            end else begin
                $display("[%0t ns] SIM_ERROR: 收到多余事件，addr=%0d。", $time, tb_event_addr);
                error_count = error_count + 1;
            end

            event_count <= event_count + 1;
        end
    end

    always @(posedge tb_clk or negedge tb_rst_n) begin
        if (!tb_rst_n) begin
            frame_done_count <= 0;
        end else if (tb_frame_done) begin
            frame_done_count <= frame_done_count + 1;
            $display("[%0t ns] SIM_INFO: frame_done asserted.", $time);
        end
    end

endmodule
