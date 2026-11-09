`timescale 1ns/1ps

module GPT_event_driven_conv_pe_compare_tb;

    localparam P_INPUT_HEIGHT = 28;
    localparam P_INPUT_WIDTH = 28;
    localparam P_NUM_INPUT_PIXELS = 784;
    localparam P_KERNEL_SIZE = 3;
    localparam P_PADDING = 1;
    localparam P_WEIGHT_BIT_WIDTH = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_KERNEL_PACKED_WIDTH = P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH;

    localparam [P_KERNEL_PACKED_WIDTH-1:0] P_KERNEL_WEIGHTS_PACKED = {
        16'hFA52, 16'h1085, 16'hFF5B,
        16'h0602, 16'h0A60, 16'hF5AA,
        16'h1314, 16'h05C8, 16'hEA50
    };

    reg clk;
    reg rst_n;
    reg start;
    reg [P_NUM_INPUT_PIXELS-1:0] input_spike_vector;

    wire ref_current_valid;
    wire [$clog2(P_NUM_INPUT_PIXELS)-1:0] ref_current_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ref_current_data;
    wire ref_current_active;
    wire ref_done;

    wire event_current_valid;
    wire [$clog2(P_NUM_INPUT_PIXELS)-1:0] event_current_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] event_current_data;
    wire event_current_active;
    wire event_done;

    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ref_data_mem [0:P_NUM_INPUT_PIXELS-1];
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] event_data_mem [0:P_NUM_INPUT_PIXELS-1];
    reg ref_active_mem [0:P_NUM_INPUT_PIXELS-1];
    reg event_active_mem [0:P_NUM_INPUT_PIXELS-1];
    reg event_seen_mem [0:P_NUM_INPUT_PIXELS-1];

    integer init_idx;
    integer row_idx;
    integer col_idx;
    integer compare_idx;
    integer ref_count;
    integer event_count;
    integer error_count;
    integer expected_active_count;
    integer event_active_count;
    reg ref_done_seen;
    reg event_done_seen;

    conv_pe #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_KERNEL_SIZE              (P_KERNEL_SIZE),
        .P_PADDING                  (P_PADDING),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_KERNEL_WEIGHTS_PACKED    (P_KERNEL_WEIGHTS_PACKED)
    ) u_ref_conv_pe (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_start                (start),
        .i_input_spike_vector   (input_spike_vector),
        .o_current_valid        (ref_current_valid),
        .o_current_addr         (ref_current_addr),
        .o_current_data         (ref_current_data),
        .o_current_active       (ref_current_active),
        .o_done                 (ref_done)
    );

    event_driven_conv_pe #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_KERNEL_SIZE              (P_KERNEL_SIZE),
        .P_PADDING                  (P_PADDING),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_KERNEL_WEIGHTS_PACKED    (P_KERNEL_WEIGHTS_PACKED)
    ) u_event_conv_pe (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_start                (start),
        .i_input_spike_vector   (input_spike_vector),
        .o_current_valid        (event_current_valid),
        .o_current_addr         (event_current_addr),
        .o_current_data         (event_current_data),
        .o_current_active       (event_current_active),
        .o_done                 (event_done)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    always @(posedge clk) begin
        if (ref_current_valid) begin
            ref_data_mem[ref_current_addr] <= ref_current_data;
            ref_active_mem[ref_current_addr] <= ref_current_active;
            ref_count <= ref_count + 1;
        end

        if (event_current_valid) begin
            event_data_mem[event_current_addr] <= event_current_data;
            event_active_mem[event_current_addr] <= event_current_active;
            event_seen_mem[event_current_addr] <= 1'b1;
            event_count <= event_count + 1;
        end

        if (ref_done) begin
            ref_done_seen <= 1'b1;
        end
        if (event_done) begin
            event_done_seen <= 1'b1;
        end
    end

    initial begin
        rst_n = 1'b0;
        start = 1'b0;
        input_spike_vector = {P_NUM_INPUT_PIXELS{1'b0}};
        ref_count = 0;
        event_count = 0;
        error_count = 0;
        expected_active_count = 0;
        event_active_count = 0;
        ref_done_seen = 1'b0;
        event_done_seen = 1'b0;

        for (init_idx = 0; init_idx < P_NUM_INPUT_PIXELS; init_idx = init_idx + 1) begin
            ref_data_mem[init_idx] = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            event_data_mem[init_idx] = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            ref_active_mem[init_idx] = 1'b0;
            event_active_mem[init_idx] = 1'b0;
            event_seen_mem[init_idx] = 1'b0;
        end

        for (row_idx = 0; row_idx < P_INPUT_HEIGHT; row_idx = row_idx + 1) begin
            for (col_idx = 0; col_idx < P_INPUT_WIDTH; col_idx = col_idx + 1) begin
                if (((row_idx * 11 + col_idx * 7 + row_idx * col_idx) % 23) < 3) begin
                    input_spike_vector[P_NUM_INPUT_PIXELS - 1 - (row_idx * P_INPUT_WIDTH + col_idx)] = 1'b1;
                end
            end
        end

        repeat (5) @(negedge clk);
        rst_n = 1'b1;

        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        wait (ref_done_seen && event_done_seen);
        repeat (3) @(posedge clk);

        if (ref_count !== P_NUM_INPUT_PIXELS) begin
            $display("SIM_ERROR: ref output count mismatch ref_count=%0d", ref_count);
            error_count = error_count + 1;
        end

        for (compare_idx = 0; compare_idx < P_NUM_INPUT_PIXELS; compare_idx = compare_idx + 1) begin
            if (ref_active_mem[compare_idx]) begin
                expected_active_count = expected_active_count + 1;
                if (!event_seen_mem[compare_idx]) begin
                    $display("SIM_ERROR: active addr=%0d was not emitted by event PE", compare_idx);
                    error_count = error_count + 1;
                end
            end

            if (event_active_mem[compare_idx]) begin
                event_active_count = event_active_count + 1;
            end

            if (event_seen_mem[compare_idx]) begin
                if (ref_data_mem[compare_idx] !== event_data_mem[compare_idx] ||
                    ref_active_mem[compare_idx] !== event_active_mem[compare_idx]) begin
                    $display("SIM_ERROR: addr=%0d ref_data=%0d event_data=%0d ref_active=%0b event_active=%0b",
                             compare_idx,
                             ref_data_mem[compare_idx],
                             event_data_mem[compare_idx],
                             ref_active_mem[compare_idx],
                             event_active_mem[compare_idx]);
                    error_count = error_count + 1;
                end
            end
        end

        if (event_active_count !== expected_active_count) begin
            $display("SIM_ERROR: active count mismatch expected=%0d event=%0d",
                     expected_active_count, event_active_count);
            error_count = error_count + 1;
        end

        if (error_count == 0) begin
            $display("SIM_INFO: event_driven_conv_pe_compare_tb PASS, ref_outputs=%0d event_outputs=%0d active=%0d",
                     ref_count, event_count, expected_active_count);
        end else begin
            $display("SIM_FAIL: event_driven_conv_pe_compare_tb errors=%0d", error_count);
        end

        $finish;
    end

    initial begin
        #200000;
        $display("SIM_TIMEOUT: ref_done=%0b event_done=%0b ref_count=%0d event_count=%0d state=%0d scanner_done=%0b flush_row_valid=%0b flush_col=%0d active_row_valid=%0b active_row=%0d buf_valid={%0b,%0b,%0b}",
                 ref_done_seen,
                 event_done_seen,
                 ref_count,
                 event_count,
                 u_event_conv_pe.state_reg,
                 u_event_conv_pe.scanner_done_w,
                 u_event_conv_pe.flush_row_valid_w,
                 u_event_conv_pe.flush_col_reg,
                 u_event_conv_pe.active_row_valid_reg,
                 u_event_conv_pe.active_event_row_reg,
                 u_event_conv_pe.buf0_valid_reg,
                 u_event_conv_pe.buf1_valid_reg,
                 u_event_conv_pe.buf2_valid_reg);
        $finish;
    end

endmodule
