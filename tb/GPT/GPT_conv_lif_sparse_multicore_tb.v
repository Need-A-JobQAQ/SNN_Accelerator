`timescale 1ns / 1ps

module GPT_conv_lif_sparse_multicore_tb;

    localparam P_INPUT_HEIGHT = 4;
    localparam P_INPUT_WIDTH = 4;
    localparam P_NUM_INPUT_PIXELS = P_INPUT_HEIGHT * P_INPUT_WIDTH;
    localparam P_NUM_CHANNELS = 2;
    localparam P_NUM_NEURONS = P_NUM_INPUT_PIXELS * P_NUM_CHANNELS;
    localparam P_NUM_CORES = 4;
    localparam P_CORE_NUM_NEURONS = P_NUM_NEURONS / P_NUM_CORES;
    localparam P_ARB_POLICY = 2;
    localparam P_CORE_MAPPING_MODE = 1;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_NEURON_VALUE_FRAC_BITS = 12;
    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ONE_FIXED =
        (1'b1 << P_NEURON_VALUE_FRAC_BITS);

    reg clk;
    reg rst_n;
    reg enable_layer;
    reg [P_NUM_INPUT_PIXELS-1:0] input_spikes;
    reg signed [P_NUM_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] currents;

    wire [P_NUM_NEURONS-1:0] single_spikes;
    wire single_valid;
    wire single_event_valid;
    wire [$clog2(P_NUM_NEURONS)-1:0] single_event_addr;
    wire single_frame_done;
    wire single_ready;
    wire [31:0] single_skip_count;
    wire [31:0] single_update_count;

    wire [P_NUM_NEURONS-1:0] multi_spikes;
    wire multi_valid;
    wire multi_event_valid;
    wire [$clog2(P_NUM_NEURONS)-1:0] multi_event_addr;
    wire multi_frame_done;
    wire multi_ready;
    wire [31:0] multi_skip_count;
    wire [31:0] multi_update_count;
    wire [P_NUM_CORES-1:0][31:0] multi_core_event_count;
    wire [P_NUM_CORES-1:0][$clog2(16 + 1)-1:0] multi_core_fifo_count;
    wire [P_NUM_CORES-1:0][$clog2(16 + 1)-1:0] multi_core_fifo_max_count;
    wire multi_fifo_overflow;

    integer idx;
    integer timeout_count;
    reg single_done_seen;
    reg multi_done_seen;

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
    ) u_single_sparse (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_enable_layer         (enable_layer),
        .i_input_spike_vector   (input_spikes),
        .i_all_currents_I       (currents),
        .o_all_spikes_out       (single_spikes),
        .o_all_spikes_valid     (single_valid),
        .o_event_valid          (single_event_valid),
        .o_event_addr           (single_event_addr),
        .o_event_frame_done     (single_frame_done),
        .o_layer_ready          (single_ready),
        .o_skip_count           (single_skip_count),
        .o_update_count         (single_update_count)
    );

    conv_lif_sparse_multicore #(
        .P_NUM_NEURONS               (P_NUM_NEURONS),
        .P_NUM_CORES                 (P_NUM_CORES),
        .P_CORE_NUM_NEURONS          (P_CORE_NUM_NEURONS),
        .P_NUM_INPUT_PIXELS          (P_NUM_INPUT_PIXELS),
        .P_INPUT_HEIGHT              (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH               (P_INPUT_WIDTH),
        .P_KERNEL_SIZE               (3),
        .P_PADDING                   (1),
        .P_NEURON_VALUE_TOTAL_BITS   (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS    (P_NEURON_VALUE_FRAC_BITS),
        .P_SKIP_THRESHOLD_SHIFT      (5),
        .P_CORE_EVENT_FIFO_DEPTH     (16),
        .P_CORE_MAPPING_MODE         (P_CORE_MAPPING_MODE),
        .P_ARB_POLICY                (P_ARB_POLICY)
    ) u_multi_sparse (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_enable_layer         (enable_layer),
        .i_input_spike_vector   (input_spikes),
        .i_all_currents_I       (currents),
        .o_all_spikes_out       (multi_spikes),
        .o_all_spikes_valid     (multi_valid),
        .o_event_valid          (multi_event_valid),
        .o_event_addr           (multi_event_addr),
        .o_event_frame_done     (multi_frame_done),
        .o_layer_ready          (multi_ready),
        .o_skip_count           (multi_skip_count),
        .o_update_count         (multi_update_count),
        .o_core_event_count     (multi_core_event_count),
        .o_core_fifo_count      (multi_core_fifo_count),
        .o_core_fifo_max_count  (multi_core_fifo_max_count),
        .o_core_fifo_overflow   (multi_fifo_overflow)
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
            single_done_seen = 1'b0;
            multi_done_seen = 1'b0;

            while ((!single_done_seen || !multi_done_seen) && timeout_count < 300) begin
                @(posedge clk);
                if (single_valid) begin
                    single_done_seen = 1'b1;
                end
                if (multi_valid) begin
                    multi_done_seen = 1'b1;
                end
                timeout_count = timeout_count + 1;
            end

            if (timeout_count >= 300) begin
                $display("SIM_FAIL: wait done timeout");
                $finish;
            end
        end
    endtask

    task check_same_result;
        input [127:0] case_name;
        begin
            if (single_spikes !== multi_spikes) begin
                $display("SIM_FAIL: %0s spike mismatch single=%b multi=%b",
                         case_name, single_spikes, multi_spikes);
                $finish;
            end
            if (single_skip_count !== multi_skip_count ||
                single_update_count !== multi_update_count) begin
                $display("SIM_FAIL: %0s count mismatch single_skip=%0d multi_skip=%0d single_update=%0d multi_update=%0d",
                         case_name, single_skip_count, multi_skip_count,
                         single_update_count, multi_update_count);
                $finish;
            end
            if (multi_fifo_overflow) begin
                $display("SIM_FAIL: %0s core fifo overflow", case_name);
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
        wait (single_ready && multi_ready);

        clear_inputs();
        pulse_enable();
        wait_both_done();
        check_same_result("zero_frame");

        clear_inputs();
        input_spikes[P_NUM_INPUT_PIXELS-1] = 1'b1;
        for (idx = 0; idx < P_NUM_NEURONS; idx = idx + 1) begin
            currents[idx] = ONE_FIXED;
        end
        pulse_enable();
        wait_both_done();
        check_same_result("active_rf");

        clear_inputs();
        pulse_enable();
        wait_both_done();
        check_same_result("active_state");

        $display("SIM_PASS: conv_lif_sparse_multicore matches single sparse behavior.");
        $display("SIM_INFO: last skip=%0d update=%0d", multi_skip_count, multi_update_count);
        $display("SIM_INFO: core_event_count={%0d,%0d,%0d,%0d}",
                 multi_core_event_count[3], multi_core_event_count[2],
                 multi_core_event_count[1], multi_core_event_count[0]);
        $display("SIM_INFO: core_fifo_max_count={%0d,%0d,%0d,%0d}",
                 multi_core_fifo_max_count[3], multi_core_fifo_max_count[2],
                 multi_core_fifo_max_count[1], multi_core_fifo_max_count[0]);
        $finish;
    end

endmodule
