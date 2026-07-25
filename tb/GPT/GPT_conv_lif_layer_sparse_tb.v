`timescale 1ns / 1ps

module GPT_conv_lif_layer_sparse_tb;

    localparam P_INPUT_HEIGHT = 4;
    localparam P_INPUT_WIDTH = 4;
    localparam P_NUM_INPUT_PIXELS = P_INPUT_HEIGHT * P_INPUT_WIDTH;
    localparam P_NUM_CHANNELS = 2;
    localparam P_NUM_NEURONS = P_NUM_INPUT_PIXELS * P_NUM_CHANNELS;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_NEURON_VALUE_FRAC_BITS = 12;
    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ONE_FIXED =
        (1'b1 << P_NEURON_VALUE_FRAC_BITS);

    reg clk;
    reg rst_n;
    reg enable_layer;
    reg [P_NUM_INPUT_PIXELS-1:0] input_spikes;
    reg signed [P_NUM_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] currents;

    wire [P_NUM_NEURONS-1:0] dense_spikes;
    wire dense_valid;
    wire dense_event_valid;
    wire [$clog2(P_NUM_NEURONS)-1:0] dense_event_addr;
    wire dense_frame_done;
    wire dense_ready;

    wire [P_NUM_NEURONS-1:0] sparse_spikes;
    wire sparse_valid;
    wire sparse_event_valid;
    wire [$clog2(P_NUM_NEURONS)-1:0] sparse_event_addr;
    wire sparse_frame_done;
    wire sparse_ready;
    wire [31:0] sparse_skip_count;
    wire [31:0] sparse_update_count;

    integer idx;
    integer timeout_count;

    conv_lif_layer #(
        .P_NUM_NEURONS               (P_NUM_NEURONS),
        .P_NEURON_VALUE_TOTAL_BITS   (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS    (P_NEURON_VALUE_FRAC_BITS)
    ) u_dense_lif (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_enable_layer         (enable_layer),
        .i_all_currents_I       (currents),
        .o_all_spikes_out       (dense_spikes),
        .o_all_spikes_valid     (dense_valid),
        .o_event_valid          (dense_event_valid),
        .o_event_addr           (dense_event_addr),
        .o_event_frame_done     (dense_frame_done),
        .o_layer_ready          (dense_ready)
    );

    conv_lif_layer_sparse #(
        .P_NUM_NEURONS               (P_NUM_NEURONS),
        .P_NUM_INPUT_PIXELS          (P_NUM_INPUT_PIXELS),
        .P_INPUT_HEIGHT              (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH               (P_INPUT_WIDTH),
        .P_KERNEL_SIZE               (3),
        .P_PADDING                   (1),
        .P_NEURON_VALUE_TOTAL_BITS   (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS    (P_NEURON_VALUE_FRAC_BITS),
        .P_SKIP_THRESHOLD_SHIFT      (5)
    ) u_sparse_lif (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_enable_layer         (enable_layer),
        .i_input_spike_vector   (input_spikes),
        .i_all_currents_I       (currents),
        .o_all_spikes_out       (sparse_spikes),
        .o_all_spikes_valid     (sparse_valid),
        .o_event_valid          (sparse_event_valid),
        .o_event_addr           (sparse_event_addr),
        .o_event_frame_done     (sparse_frame_done),
        .o_layer_ready          (sparse_ready),
        .o_skip_count           (sparse_skip_count),
        .o_update_count         (sparse_update_count)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    task clear_inputs;
        begin
            input_spikes = {P_NUM_INPUT_PIXELS{1'b0}};
            for (idx = 0; idx < P_NUM_NEURONS; idx = idx + 1) begin
                currents[idx] = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            end
        end
    endtask

    task pulse_enable;
        begin
            @(posedge clk);
            enable_layer <= 1'b1;
            @(posedge clk);
            enable_layer <= 1'b0;
        end
    endtask

    task wait_both_done;
        begin
            timeout_count = 0;
            while ((!dense_valid || !sparse_valid) && timeout_count < 200) begin
                @(posedge clk);
                timeout_count = timeout_count + 1;
            end

            if (timeout_count >= 200) begin
                $display("SIM_FAIL: wait done timeout");
                $finish;
            end
        end
    endtask

    task check_same_spikes;
        input [127:0] case_name;
        begin
            if (dense_spikes !== sparse_spikes) begin
                $display("SIM_FAIL: %0s spike mismatch dense=%b sparse=%b",
                         case_name, dense_spikes, sparse_spikes);
                $finish;
            end
        end
    endtask

    initial begin
        rst_n = 1'b0;
        enable_layer = 1'b0;
        clear_inputs();

        repeat (5) @(posedge clk);
        rst_n = 1'b1;

        wait (dense_ready && sparse_ready);

        /*
         * case0：全零输入、全零电流。
         * sparse 版应跳过所有神经元，不访问膜电位 RAM。
         */
        clear_inputs();
        pulse_enable();
        wait_both_done();
        check_same_spikes("zero_frame");

        if (sparse_skip_count != P_NUM_NEURONS || sparse_update_count != 0) begin
            $display("SIM_FAIL: zero_frame count error skip=%0d update=%0d",
                     sparse_skip_count, sparse_update_count);
            $finish;
        end

        /*
         * case1：左上角输入像素有脉冲。
         * 受 padding=1、3x3 感受野影响，两个通道各有 4 个输出像素需要更新。
         */
        clear_inputs();
        input_spikes[P_NUM_INPUT_PIXELS-1] = 1'b1;
        for (idx = 0; idx < P_NUM_NEURONS; idx = idx + 1) begin
            currents[idx] = ONE_FIXED;
        end

        pulse_enable();
        wait_both_done();

        if (sparse_update_count != 8 || sparse_skip_count != (P_NUM_NEURONS - 8)) begin
            $display("SIM_FAIL: active receptive field count error skip=%0d update=%0d",
                     sparse_skip_count, sparse_update_count);
            $finish;
        end

        /*
         * case2：下一帧没有输入脉冲。
         * case1 中被更新但未发放的神经元 active bit 为 1，因此这一帧仍需继续泄漏。
         */
        clear_inputs();
        pulse_enable();
        wait_both_done();

        if (sparse_update_count != 8 || sparse_skip_count != (P_NUM_NEURONS - 8)) begin
            $display("SIM_FAIL: active state count error skip=%0d update=%0d",
                     sparse_skip_count, sparse_update_count);
            $finish;
        end

        $display("SIM_PASS: conv_lif_layer_sparse skip/update behavior is correct.");
        $display("SIM_INFO: last skip=%0d update=%0d", sparse_skip_count, sparse_update_count);
        $finish;
    end

endmodule
