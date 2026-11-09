`timescale 1ns/1ps

module GPT_event_driven_conv_pe_golden_tb;

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

    wire event_current_valid;
    wire [$clog2(P_NUM_INPUT_PIXELS)-1:0] event_current_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] event_current_data;
    wire event_current_active;
    wire event_done;

    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] golden_data_mem [0:P_NUM_INPUT_PIXELS-1];
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] event_data_mem [0:P_NUM_INPUT_PIXELS-1];
    reg golden_active_mem [0:P_NUM_INPUT_PIXELS-1];
    reg event_active_mem [0:P_NUM_INPUT_PIXELS-1];

    integer init_idx;
    integer row_idx;
    integer col_idx;
    integer kernel_row_idx;
    integer kernel_col_idx;
    integer kernel_idx;
    integer out_row_idx;
    integer out_col_idx;
    integer flat_idx;
    integer current_addr_idx;
    integer compare_idx;
    integer event_count;
    integer error_count;
    integer total_error_count;
    integer case_id;
    reg event_done_seen;
    reg saw_jump_retire_case;

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

    function signed [P_WEIGHT_BIT_WIDTH-1:0] get_kernel_weight;
        input integer idx;
        integer bit_base;
        begin
            bit_base = (P_KERNEL_SIZE * P_KERNEL_SIZE - 1 - idx) * P_WEIGHT_BIT_WIDTH;
            get_kernel_weight = P_KERNEL_WEIGHTS_PACKED[bit_base +: P_WEIGHT_BIT_WIDTH];
        end
    endfunction

    task clear_case_state;
        begin
            event_count = 0;
            error_count = 0;
            event_done_seen = 1'b0;
            saw_jump_retire_case = 1'b0;

            for (init_idx = 0; init_idx < P_NUM_INPUT_PIXELS; init_idx = init_idx + 1) begin
                golden_data_mem[init_idx] = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                event_data_mem[init_idx] = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                golden_active_mem[init_idx] = 1'b0;
                event_active_mem[init_idx] = 1'b0;
            end
        end
    endtask

    task set_spike;
        input integer row;
        input integer col;
        integer pixel_idx;
        begin
            pixel_idx = (row * P_INPUT_WIDTH) + col;
            input_spike_vector[P_NUM_INPUT_PIXELS - 1 - pixel_idx] = 1'b1;
        end
    endtask

    task build_golden;
        reg signed [P_WEIGHT_BIT_WIDTH-1:0] weight;
        reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] weight_ext;
        begin
            for (row_idx = 0; row_idx < P_INPUT_HEIGHT; row_idx = row_idx + 1) begin
                for (col_idx = 0; col_idx < P_INPUT_WIDTH; col_idx = col_idx + 1) begin
                    flat_idx = (row_idx * P_INPUT_WIDTH) + col_idx;
                    if (input_spike_vector[P_NUM_INPUT_PIXELS - 1 - flat_idx]) begin
                        for (kernel_row_idx = 0; kernel_row_idx < P_KERNEL_SIZE; kernel_row_idx = kernel_row_idx + 1) begin
                            for (kernel_col_idx = 0; kernel_col_idx < P_KERNEL_SIZE; kernel_col_idx = kernel_col_idx + 1) begin
                                out_row_idx = row_idx - kernel_row_idx + P_PADDING;
                                out_col_idx = col_idx - kernel_col_idx + P_PADDING;
                                if (out_row_idx >= 0 && out_row_idx < P_INPUT_HEIGHT &&
                                    out_col_idx >= 0 && out_col_idx < P_INPUT_WIDTH) begin
                                    kernel_idx = (kernel_row_idx * P_KERNEL_SIZE) + kernel_col_idx;
                                    weight = get_kernel_weight(kernel_idx);
                                    weight_ext = {{(P_NEURON_VALUE_TOTAL_BITS - P_WEIGHT_BIT_WIDTH){weight[P_WEIGHT_BIT_WIDTH-1]}}, weight};
                                    current_addr_idx = P_NUM_INPUT_PIXELS - 1 -
                                        ((out_row_idx * P_INPUT_WIDTH) + out_col_idx);
                                    golden_data_mem[current_addr_idx] =
                                        golden_data_mem[current_addr_idx] + weight_ext;
                                    golden_active_mem[current_addr_idx] = 1'b1;
                                end
                            end
                        end
                    end
                end
            end
        end
    endtask

    task run_case;
        input integer id;
        begin
            case_id = id;
            clear_case_state();
            build_golden();

            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;

            wait (event_done_seen);
            repeat (3) @(posedge clk);

            for (compare_idx = 0; compare_idx < P_NUM_INPUT_PIXELS; compare_idx = compare_idx + 1) begin
                if (golden_data_mem[compare_idx] !== event_data_mem[compare_idx] ||
                    golden_active_mem[compare_idx] !== event_active_mem[compare_idx]) begin
                    $display("SIM_ERROR: case=%0d addr=%0d golden_data=%0d event_data=%0d golden_active=%0b event_active=%0b",
                             case_id,
                             compare_idx,
                             golden_data_mem[compare_idx],
                             event_data_mem[compare_idx],
                             golden_active_mem[compare_idx],
                             event_active_mem[compare_idx]);
                    error_count = error_count + 1;
                end
            end

            if (case_id == 2 && !saw_jump_retire_case) begin
                $display("SIM_ERROR: case=2 did not observe multi-column retirement before jump event.");
                error_count = error_count + 1;
            end

            if (error_count == 0) begin
                $display("SIM_INFO: golden case=%0d PASS, event_outputs=%0d", case_id, event_count);
            end else begin
                $display("SIM_FAIL: golden case=%0d errors=%0d", case_id, error_count);
            end

            total_error_count = total_error_count + error_count;
            input_spike_vector = {P_NUM_INPUT_PIXELS{1'b0}};
            repeat (4) @(posedge clk);
        end
    endtask

    always @(posedge clk) begin
        if (event_current_valid) begin
            event_data_mem[event_current_addr] <= event_current_data;
            event_active_mem[event_current_addr] <= event_current_active;
            event_count <= event_count + 1;
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

            if (case_id == 2 &&
                u_event_conv_pe.state_reg == u_event_conv_pe.S_RETIRE_BEFORE &&
                u_event_conv_pe.event_col_reg == 5'd8 &&
                u_event_conv_pe.retire_col_reg > 5'd2) begin
                saw_jump_retire_case <= 1'b1;
            end
        end
    end

    initial begin
        rst_n = 1'b0;
        start = 1'b0;
        input_spike_vector = {P_NUM_INPUT_PIXELS{1'b0}};
        total_error_count = 0;
        case_id = 0;

        repeat (5) @(negedge clk);
        rst_n = 1'b1;

        // case 1：连续 spike。
        for (col_idx = 0; col_idx < P_INPUT_WIDTH; col_idx = col_idx + 1) begin
            set_spike(5, col_idx);
        end
        run_case(1);

        // case 2：跳跃 spike，显式覆盖 col=2 -> col=8 的连续退休。
        set_spike(7, 2);
        set_spike(7, 8);
        set_spike(7, 20);
        run_case(2);

        // case 3：第一颗 spike 很靠右。
        set_spike(10, 24);
        set_spike(10, 27);
        run_case(3);

        // case 4：左边界 spike。
        set_spike(12, 0);
        set_spike(13, 0);
        run_case(4);

        // case 5：右边界 spike。
        set_spike(14, 27);
        set_spike(15, 27);
        run_case(5);

        // case 6：只有一个 spike，且第一颗 event 不在 row0。
        set_spike(18, 13);
        run_case(6);

        // case 7：一整行没有 spike。
        set_spike(20, 5);
        set_spike(20, 17);
        set_spike(22, 8);
        set_spike(22, 23);
        run_case(7);

        // case 8：连续两行都有 spike。
        set_spike(3, 4);
        set_spike(3, 5);
        set_spike(4, 4);
        set_spike(4, 5);
        run_case(8);

        // case 9：相邻 spike 反复影响同一输出像素。
        set_spike(16, 5);
        set_spike(16, 6);
        set_spike(16, 7);
        set_spike(17, 5);
        set_spike(17, 6);
        set_spike(17, 7);
        run_case(9);

        if (total_error_count == 0) begin
            $display("SIM_INFO: event_driven_conv_pe_golden_tb PASS.");
        end else begin
            $display("SIM_FAIL: event_driven_conv_pe_golden_tb total_errors=%0d", total_error_count);
        end

        $finish;
    end

    initial begin
        #800000;
        $display("SIM_TIMEOUT: case=%0d event_done=%0b state=%0d retire_col=%0d retire_done=%0b frontier_valid={%0b,%0b,%0b}",
                 case_id,
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
