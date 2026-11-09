`timescale 1ns / 1ps

module GPT_spike_event_scanner_tb;

    localparam P_NUM_INPUT_PIXELS = 16;
    localparam P_EVENT_ADDR_WIDTH = 4;

    reg clk;
    reg rst_n;
    reg i_start;
    reg [P_NUM_INPUT_PIXELS-1:0] i_spike_vector;
    reg i_event_ready;

    wire o_event_valid;
    wire [P_EVENT_ADDR_WIDTH-1:0] o_event_addr;
    wire o_done;
    wire o_busy;
    wire [31:0] o_event_count;

    integer error_count;

    spike_event_scanner #(
        .P_NUM_INPUT_PIXELS  (P_NUM_INPUT_PIXELS),
        .P_EVENT_ADDR_WIDTH  (P_EVENT_ADDR_WIDTH),
        .P_WORD_WIDTH        (8)
    ) u_dut (
        .clk                 (clk),
        .rst_n               (rst_n),
        .i_start             (i_start),
        .i_spike_vector      (i_spike_vector),
        .i_event_ready       (i_event_ready),
        .o_event_valid       (o_event_valid),
        .o_event_addr        (o_event_addr),
        .o_done              (o_done),
        .o_busy              (o_busy),
        .o_event_count       (o_event_count)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    task pulse_start;
        input [P_NUM_INPUT_PIXELS-1:0] spike_vector;
        begin
            @(posedge clk);
            i_spike_vector <= spike_vector;
            i_start <= 1'b1;
            @(posedge clk);
            i_start <= 1'b0;
            i_spike_vector <= {P_NUM_INPUT_PIXELS{1'b0}};
            #1;
        end
    endtask

    task expect_event;
        input [P_EVENT_ADDR_WIDTH-1:0] expected_addr;
        begin
            while (!o_event_valid) begin
                @(posedge clk);
            end

            if (o_event_addr !== expected_addr) begin
                $display("SIM_ERROR: expected event_addr=%0d, got=%0d",
                         expected_addr, o_event_addr);
                error_count = error_count + 1;
            end

            i_event_ready = 1'b1;
            @(posedge clk);
            #1;
            i_event_ready = 1'b0;
        end
    endtask

    initial begin
        rst_n = 1'b0;
        i_start = 1'b0;
        i_spike_vector = {P_NUM_INPUT_PIXELS{1'b0}};
        i_event_ready = 1'b0;
        error_count = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        // 普通稀疏向量。输出顺序为图像空间地址升序：左上到右下。
        pulse_start(16'b1000_0000_1000_1001);
        expect_event(4'd0);
        expect_event(4'd8);
        expect_event(4'd12);
        expect_event(4'd15);
        if (o_event_count !== 32'd4) begin
            $display("SIM_ERROR: expected event_count=4, got=%0d", o_event_count);
            error_count = error_count + 1;
        end
        while (!o_done) begin
            @(posedge clk);
        end

        // 位序确认：最高位是图像左上角，应输出图像地址 0。
        pulse_start(16'b1000_0000_0000_0000);
        expect_event(4'd0);
        while (!o_done) begin
            @(posedge clk);
        end

        // 位序确认：最低位是图像右下角，应输出最后一个图像地址。
        pulse_start(16'b0000_0000_0000_0001);
        expect_event(4'd15);
        while (!o_done) begin
            @(posedge clk);
        end

        // 空 spike vector 应直接 done，不应输出有效事件。
        pulse_start(16'b0000_0000_0000_0000);
        while (!o_done) begin
            @(posedge clk);
        end
        if (o_event_valid) begin
            $display("SIM_ERROR: empty spike vector should not emit an event.");
            error_count = error_count + 1;
        end

        // ready 反压：event_valid 拉高但未 ready 时，应保持当前事件地址。
        pulse_start(16'b0000_0000_0010_0010);
        repeat (3) @(posedge clk);
        if (!o_event_valid || o_event_addr !== 4'd10) begin
            $display("SIM_ERROR: backpressure hold failed, valid=%0b addr=%0d",
                     o_event_valid, o_event_addr);
            error_count = error_count + 1;
        end
        expect_event(4'd10);
        expect_event(4'd14);
        while (!o_done) begin
            @(posedge clk);
        end

        if (error_count == 0) begin
            $display("SIM_INFO: spike_event_scanner_tb PASS.");
        end else begin
            $display("SIM_FAIL: spike_event_scanner_tb errors=%0d", error_count);
        end

        $finish;
    end

    initial begin
        #3000;
        $display("SIM_TIMEOUT: valid=%0b addr=%0d done=%0b busy=%0b count=%0d",
                 o_event_valid, o_event_addr, o_done, o_busy, o_event_count);
        $finish;
    end

endmodule
