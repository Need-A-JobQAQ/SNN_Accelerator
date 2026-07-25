`timescale 1ns/1ps

module GPT_conv_parallel_compare_tb;

    localparam CLK_PERIOD = 10;
    localparam P_INPUT_HEIGHT = 28;
    localparam P_INPUT_WIDTH = 28;
    localparam P_NUM_INPUT_PIXELS = 784;
    localparam P_NUM_OUTPUT_CHANNELS = 2;
    localparam P_KERNEL_SIZE = 3;
    localparam P_PADDING = 1;
    localparam P_WEIGHT_BIT_WIDTH = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_NUM_OUTPUT_FEATURES = P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS;
    localparam P_CONV_WEIGHT_PACKED_WIDTH = P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH;

    localparam [P_CONV_WEIGHT_PACKED_WIDTH-1:0] P_CONV0_WEIGHTS_PACKED = {
        16'hFA52,16'h1085,16'hFF5B,
        16'h0602,16'h0A60,16'hF5AA,
        16'h1314,16'h05C8,16'hEA50
    };

    localparam [P_CONV_WEIGHT_PACKED_WIDTH-1:0] P_CONV1_WEIGHTS_PACKED = {
        16'hE764,16'h41B2,16'h272A,
        16'hF6DD,16'h61AB,16'hF8CB,
        16'h36E8,16'h2EFE,16'hE59C
    };

    reg tb_clk;
    reg tb_rst_n;
    reg tb_start;
    reg [P_NUM_INPUT_PIXELS-1:0] tb_input_spikes;

    wire signed [P_NUM_OUTPUT_FEATURES-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] ref_currents;
    wire signed [P_NUM_OUTPUT_FEATURES-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] parallel_currents;
    wire ref_valid;
    wire parallel_valid;

    reg ref_done_seen;
    reg parallel_done_seen;

    integer input_idx;
    integer check_idx;
    integer error_count;

    conv_layer #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_NUM_OUTPUT_CHANNELS      (P_NUM_OUTPUT_CHANNELS),
        .P_KERNEL_SIZE              (P_KERNEL_SIZE),
        .P_PADDING                  (P_PADDING),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_CONV0_WEIGHTS_PACKED     (P_CONV0_WEIGHTS_PACKED),
        .P_CONV1_WEIGHTS_PACKED     (P_CONV1_WEIGHTS_PACKED)
    ) u_ref_conv_layer (
        .clk                    (tb_clk),
        .rst_n                  (tb_rst_n),
        .i_calc_start           (tb_start),
        .i_input_spike_vector   (tb_input_spikes),
        .o_all_currents_I       (ref_currents),
        .o_all_currents_valid   (ref_valid)
    );

    conv_layer_parallel #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_NUM_OUTPUT_CHANNELS      (P_NUM_OUTPUT_CHANNELS),
        .P_KERNEL_SIZE              (P_KERNEL_SIZE),
        .P_PADDING                  (P_PADDING),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_CONV0_WEIGHTS_PACKED     (P_CONV0_WEIGHTS_PACKED),
        .P_CONV1_WEIGHTS_PACKED     (P_CONV1_WEIGHTS_PACKED)
    ) u_parallel_conv_layer (
        .clk                    (tb_clk),
        .rst_n                  (tb_rst_n),
        .i_calc_start           (tb_start),
        .i_input_spike_vector   (tb_input_spikes),
        .o_all_currents_I       (parallel_currents),
        .o_all_currents_valid   (parallel_valid)
    );

    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    always @(posedge tb_clk or negedge tb_rst_n) begin
        if (!tb_rst_n) begin
            ref_done_seen <= 1'b0;
            parallel_done_seen <= 1'b0;
        end else begin
            if (tb_start) begin
                ref_done_seen <= 1'b0;
                parallel_done_seen <= 1'b0;
            end
            if (ref_valid) begin
                ref_done_seen <= 1'b1;
            end
            if (parallel_valid) begin
                parallel_done_seen <= 1'b1;
            end
        end
    end

    initial begin
        error_count = 0;
        tb_rst_n = 1'b0;
        tb_start = 1'b0;
        tb_input_spikes = {P_NUM_INPUT_PIXELS{1'b0}};

        repeat (5) @(posedge tb_clk);
        tb_rst_n = 1'b1;
        repeat (2) @(posedge tb_clk);

        /*
         * 构造一个稀疏但分布较散的输入脉冲图，方便覆盖边界和中心区域。
         */
        for (input_idx = 0; input_idx < P_NUM_INPUT_PIXELS; input_idx = input_idx + 1) begin
            if ((input_idx % 17 == 0) || (input_idx % 29 == 3)) begin
                tb_input_spikes[P_NUM_INPUT_PIXELS - 1 - input_idx] = 1'b1;
            end
        end

        @(negedge tb_clk);
        tb_start = 1'b1;
        @(negedge tb_clk);
        tb_start = 1'b0;

        wait (ref_done_seen && parallel_done_seen);
        @(posedge tb_clk);

        for (check_idx = 0; check_idx < P_NUM_OUTPUT_FEATURES; check_idx = check_idx + 1) begin
            if (ref_currents[check_idx] !== parallel_currents[check_idx]) begin
                $display("[%0t ns] SIM_ERROR: idx=%0d ref=%0d parallel=%0d",
                         $time, check_idx, ref_currents[check_idx], parallel_currents[check_idx]);
                error_count = error_count + 1;
            end
        end

        if (error_count == 0) begin
            $display("[%0t ns] SIM_PASS: conv_layer_parallel matches conv_layer.", $time);
        end else begin
            $display("[%0t ns] SIM_FAIL: mismatch count=%0d.", $time, error_count);
        end

        $finish;
    end

endmodule
