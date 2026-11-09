`timescale 1ns/1ps

module GPT_event_driven_conv_pe_patterns_tb;

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
    integer col_idx;
    integer compare_idx;
    integer ref_count;
    integer event_count;
    integer error_count;
    integer total_error_count;
    integer case_id;
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

    always @(posedge clk) begin
        if (rst_n) begin
            if (u_event_conv_pe.state_reg == u_event_conv_pe.S_PROCESS_EVENT &&
                ((u_event_conv_pe.update_new_valid_0_comb && u_event_conv_pe.update_col_0_comb >= u_event_conv_pe.retire_col_reg) ||
                 (u_event_conv_pe.update_new_valid_1_comb && u_event_conv_pe.update_col_1_comb >= u_event_conv_pe.retire_col_reg) ||
                 (u_event_conv_pe.update_new_valid_2_comb && u_event_conv_pe.update_col_2_comb >= u_event_conv_pe.retire_col_reg)) &&
                u_event_conv_pe.frontier_overflow_comb) begin
                $fatal(1,
                       "SIM_FATAL: frontier overflow, retire_col=%0d event_row=%0d event_col=%0d",
                       u_event_conv_pe.retire_col_reg,
                       u_event_conv_pe.active_event_row_reg,
                       u_event_conv_pe.event_col_reg);
            end

            if (u_event_conv_pe.state_reg == u_event_conv_pe.S_PROCESS_EVENT &&
                ((u_event_conv_pe.update_old_valid_0_comb && u_event_conv_pe.update_col_0_comb < u_event_conv_pe.retire_col_reg) ||
                 (u_event_conv_pe.update_old_valid_1_comb && u_event_conv_pe.update_col_1_comb < u_event_conv_pe.retire_col_reg) ||
                 (u_event_conv_pe.update_old_valid_2_comb && u_event_conv_pe.update_col_2_comb < u_event_conv_pe.retire_col_reg))) begin
                $fatal(1,
                       "SIM_FATAL: retired old output modified, retire_col=%0d event_row=%0d event_col=%0d",
                       u_event_conv_pe.retire_col_reg,
                       u_event_conv_pe.active_event_row_reg,
                       u_event_conv_pe.event_col_reg);
            end

            if (u_event_conv_pe.state_reg == u_event_conv_pe.S_FINISH_ROW &&
                !u_event_conv_pe.finish_needed_w &&
                (u_event_conv_pe.frontier_valid_reg[0] ||
                 u_event_conv_pe.frontier_valid_reg[1] ||
                 u_event_conv_pe.frontier_valid_reg[2])) begin
                $fatal(1,
                       "SIM_FATAL: row switch with non-empty frontier, valid={%0b,%0b,%0b}",
                       u_event_conv_pe.frontier_valid_reg[0],
                       u_event_conv_pe.frontier_valid_reg[1],
                       u_event_conv_pe.frontier_valid_reg[2]);
            end
        end
    end

    task clear_case_state;
        begin
            ref_count = 0;
            event_count = 0;
            error_count = 0;
            ref_done_seen = 1'b0;
            event_done_seen = 1'b0;

            for (init_idx = 0; init_idx < P_NUM_INPUT_PIXELS; init_idx = init_idx + 1) begin
                ref_data_mem[init_idx] = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                event_data_mem[init_idx] = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                ref_active_mem[init_idx] = 1'b0;
                event_active_mem[init_idx] = 1'b0;
                event_seen_mem[init_idx] = 1'b0;
            end
        end
    endtask

    task set_spike;
        input integer row;
        input integer col;
        integer flat_idx;
        begin
            flat_idx = (row * P_INPUT_WIDTH) + col;
            input_spike_vector[P_NUM_INPUT_PIXELS - 1 - flat_idx] = 1'b1;
        end
    endtask

    task run_case;
        input integer id;
        begin
            case_id = id;
            clear_case_state();

            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;

            wait (ref_done_seen && event_done_seen);
            repeat (3) @(posedge clk);

            if (ref_count !== P_NUM_INPUT_PIXELS) begin
                $display("SIM_ERROR: case=%0d ref output count mismatch ref_count=%0d", case_id, ref_count);
                error_count = error_count + 1;
            end

            for (compare_idx = 0; compare_idx < P_NUM_INPUT_PIXELS; compare_idx = compare_idx + 1) begin
                if (ref_active_mem[compare_idx] && !event_seen_mem[compare_idx]) begin
                    $display("SIM_ERROR: case=%0d active addr=%0d was not emitted", case_id, compare_idx);
                    error_count = error_count + 1;
                end

                if (event_seen_mem[compare_idx]) begin
                    if (ref_data_mem[compare_idx] !== event_data_mem[compare_idx] ||
                        ref_active_mem[compare_idx] !== event_active_mem[compare_idx]) begin
                        $display("SIM_ERROR: case=%0d addr=%0d ref_data=%0d event_data=%0d ref_active=%0b event_active=%0b",
                                 case_id,
                                 compare_idx,
                                 ref_data_mem[compare_idx],
                                 event_data_mem[compare_idx],
                                 ref_active_mem[compare_idx],
                                 event_active_mem[compare_idx]);
                        error_count = error_count + 1;
                    end
                end
            end

            if (error_count == 0) begin
                $display("SIM_INFO: case=%0d PASS, ref_outputs=%0d event_outputs=%0d",
                         case_id, ref_count, event_count);
            end else begin
                $display("SIM_FAIL: case=%0d errors=%0d", case_id, error_count);
            end

            total_error_count = total_error_count + error_count;
            input_spike_vector = {P_NUM_INPUT_PIXELS{1'b0}};
            repeat (4) @(posedge clk);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        start = 1'b0;
        input_spike_vector = {P_NUM_INPUT_PIXELS{1'b0}};
        total_error_count = 0;

        repeat (5) @(negedge clk);
        rst_n = 1'b1;

        // case 1：连续事件。
        for (col_idx = 0; col_idx < P_INPUT_WIDTH; col_idx = col_idx + 1) begin
            set_spike(5, col_idx);
        end
        run_case(1);

        // case 2：跳跃事件。
        set_spike(7, 0);
        set_spike(7, 4);
        set_spike(7, 11);
        set_spike(7, 20);
        run_case(2);

        // case 3：中间整行没有 spike。
        set_spike(9, 3);
        set_spike(9, 18);
        set_spike(11, 5);
        set_spike(11, 22);
        run_case(3);

        // case 4：某行第一个 spike 很靠右。
        set_spike(13, 24);
        set_spike(13, 27);
        run_case(4);

        // case 5：左右边界和角落 spike。
        set_spike(0, 0);
        set_spike(0, 27);
        set_spike(27, 0);
        set_spike(27, 27);
        set_spike(14, 0);
        set_spike(14, 27);
        run_case(5);

        // case 6：相邻多个 spike 反复命中同一输出像素。
        set_spike(16, 5);
        set_spike(16, 6);
        set_spike(16, 7);
        set_spike(17, 5);
        set_spike(17, 6);
        set_spike(17, 7);
        run_case(6);

        if (total_error_count == 0) begin
            $display("SIM_INFO: event_driven_conv_pe_patterns_tb PASS.");
        end else begin
            $display("SIM_FAIL: event_driven_conv_pe_patterns_tb total_errors=%0d", total_error_count);
        end

        $finish;
    end

    initial begin
        #500000;
        $display("SIM_TIMEOUT: case=%0d ref_done=%0b event_done=%0b state=%0d retire_col=%0d retire_done=%0b frontier_valid={%0b,%0b,%0b}",
                 case_id,
                 ref_done_seen,
                 event_done_seen,
                 u_event_conv_pe.state_reg,
                 u_event_conv_pe.retire_col_reg,
                 u_event_conv_pe.retire_done_reg,
                 u_event_conv_pe.frontier_valid_reg[0],
                 u_event_conv_pe.frontier_valid_reg[1],
                 u_event_conv_pe.frontier_valid_reg[2]);
        $finish;
    end

endmodule
