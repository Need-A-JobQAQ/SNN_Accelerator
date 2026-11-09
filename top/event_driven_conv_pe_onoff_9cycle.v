module event_driven_conv_pe_onoff_9cycle #(
    parameter P_INPUT_HEIGHT = 32,
    parameter P_INPUT_WIDTH = 32,
    parameter P_NUM_INPUT_PIXELS = 1024,
    parameter P_KERNEL_SIZE = 3,
    parameter P_PADDING = 1,
    parameter P_WEIGHT_BIT_WIDTH = 16,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_KERNEL_ON_WEIGHTS_PACKED =
        {P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}},
    parameter [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_KERNEL_OFF_WEIGHTS_PACKED =
        {P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}}
) (
    input wire                                           clk,
    input wire                                           rst_n,
    input wire                                           i_start,
    input wire [P_NUM_INPUT_PIXELS-1:0]                  i_input_spike_vector_on,
    input wire [P_NUM_INPUT_PIXELS-1:0]                  i_input_spike_vector_off,

    output reg                                           o_current_valid,
    output reg [$clog2(P_NUM_INPUT_PIXELS)-1:0]          o_current_addr,
    output reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0]    o_current_data,
    output reg                                           o_current_active,
    output reg                                           o_done
);

    /*
     * 双极性事件驱动卷积 PE：两行完整 line buffer + 3 个 frontier 累加寄存器。
     *
     * spike_event_scanner 输出图像空间地址，顺序为左上到右下。
     * 对输入行 y：
     *   mixed buffer 的未退休部分保存旧输出行 y-1；
     *   mixed buffer 的已退休部分复用为新输出行 y+1；
     *   middle buffer 保存当前输出行 y；
     *   frontier regs 暂存新输出行 y+1 中还没有释放存储位置的少量像素。
     */
    localparam P_KERNEL_ELEMS = P_KERNEL_SIZE * P_KERNEL_SIZE;
    localparam LP_ADDR_WIDTH = $clog2(P_NUM_INPUT_PIXELS);
    localparam LP_ROW_WIDTH = $clog2(P_INPUT_HEIGHT);
    localparam LP_COL_WIDTH = $clog2(P_INPUT_WIDTH);
    // 收尾还会经过 HEIGHT、HEIGHT+1，32行时不能只用6位有符号数。
    localparam LP_ROW_SIGNED_WIDTH = $clog2(P_INPUT_HEIGHT + 2) + 1;
    localparam signed [LP_ROW_SIGNED_WIDTH-1:0] LP_ROW_STEP_ONE =
        {{(LP_ROW_SIGNED_WIDTH-1){1'b0}}, 1'b1};

    localparam [2:0] S_IDLE          = 3'd0;
    localparam [2:0] S_WAIT_EVENT    = 3'd1;
    localparam [2:0] S_RETIRE_BEFORE = 3'd2;
    localparam [2:0] S_PROCESS_EVENT = 3'd3;
    localparam [2:0] S_RETIRE_AFTER  = 3'd4;
    localparam [2:0] S_FINISH_ROW    = 3'd5;
    localparam [2:0] S_DONE          = 3'd6;

    reg [2:0] state_reg;
    reg [2:0] state_next;

    reg scanner_start_reg;
    reg scanner_done_seen_reg;
    wire scanner_event_valid_w;
    wire scanner_event_ready_w;
    wire [LP_ADDR_WIDTH-1:0] scanner_event_addr_w;
    wire scanner_done_w;

    // 同一时间步的两极性快照，避免计算期间外部输入变化。
    reg [P_NUM_INPUT_PIXELS-1:0] input_on_reg;
    reg [P_NUM_INPUT_PIXELS-1:0] input_off_reg;
    reg [1:0] event_polarity_reg;
    wire [P_NUM_INPUT_PIXELS-1:0] input_union_w;

    reg active_row_valid_reg;
    reg signed [LP_ROW_SIGNED_WIDTH-1:0] active_event_row_reg;
    reg pending_event_valid_reg;
    reg signed [LP_ROW_SIGNED_WIDTH-1:0] event_row_reg;
    reg [LP_COL_WIDTH-1:0] event_col_reg;
    reg [3:0] update_idx_reg;
    reg [LP_COL_WIDTH-1:0] retire_col_reg;
    reg retire_done_reg;

    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] buf0_data_reg [0:P_INPUT_WIDTH-1];
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] buf1_data_reg [0:P_INPUT_WIDTH-1];
    reg buf0_active_reg [0:P_INPUT_WIDTH-1];
    reg buf1_active_reg [0:P_INPUT_WIDTH-1];
    reg buf0_valid_reg;
    reg buf1_valid_reg;
    reg buf2_valid_reg;  // 兼容旧测试的层级调试名，内部不再使用第三个完整 buffer。

    reg [1:0] mixed_buf_idx_reg;
    reg [1:0] middle_buf_idx_reg;

    reg frontier_valid_reg [0:2];
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] frontier_data_reg [0:2];
    reg frontier_active_reg [0:2];

    wire event_accept_w;
    wire pending_row_ahead_w;
    wire accepted_row_ahead_w;
    wire retire_before_needed_w;
    wire retire_after_needed_w;
    wire finish_needed_w;
    wire retire_fire_w;
    wire retire_last_col_w;
    wire flush_row_valid_w;
    wire [LP_COL_WIDTH-1:0] flush_col_reg;
    wire [LP_ROW_WIDTH-1:0] flush_row_unsigned_w;
    wire signed [LP_ROW_SIGNED_WIDTH-1:0] flush_row_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] update_weight_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] update_weight_ext_w;

    reg signed [LP_ROW_SIGNED_WIDTH-1:0] event_row_comb;
    reg [LP_COL_WIDTH-1:0] event_col_comb;
    reg update_valid_comb;
    reg [1:0] update_row_sel_comb;
    reg [LP_COL_WIDTH-1:0] update_col_comb;
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] update_weight_comb;
    reg mixed_valid_comb;
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] mixed_data_comb;
    reg mixed_active_comb;
    reg frontier_update_valid_comb;
    reg [1:0] frontier_update_idx_comb;
    reg frontier_overflow_comb;
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] release_data_comb;
    reg release_active_comb;

    integer reset_col_idx;
    integer frontier_idx;
    integer update_kernel_row_comb;
    integer update_kernel_col_comb;
    integer update_kernel_idx_comb;
    integer update_col_int_comb;
    integer frontier_offset_comb;

    assign scanner_event_ready_w = (state_reg == S_WAIT_EVENT) &&
                                   !pending_event_valid_reg &&
                                   !scanner_done_seen_reg;
    assign event_accept_w = scanner_event_valid_w && scanner_event_ready_w;
    assign pending_row_ahead_w = active_row_valid_reg &&
                                 pending_event_valid_reg &&
                                 (event_row_reg > active_event_row_reg);
    assign accepted_row_ahead_w = active_row_valid_reg &&
                                  event_accept_w &&
                                  (event_row_comb > active_event_row_reg);
    assign retire_before_needed_w = pending_event_valid_reg &&
                                    (event_col_reg >= 2) &&
                                    (retire_col_reg <= (event_col_reg - 2));
    assign retire_after_needed_w = pending_event_valid_reg &&
                                   (event_col_reg >= 1) &&
                                   (retire_col_reg <= (event_col_reg - 1));
    assign finish_needed_w = !retire_done_reg;
    assign retire_fire_w = ((state_reg == S_RETIRE_BEFORE) && retire_before_needed_w) ||
                           ((state_reg == S_RETIRE_AFTER) && retire_after_needed_w) ||
                           ((state_reg == S_FINISH_ROW) && finish_needed_w);
    assign retire_last_col_w = (retire_col_reg == P_INPUT_WIDTH - 1);
    assign flush_col_reg = retire_col_reg;
    assign flush_row_w = active_event_row_reg - LP_ROW_STEP_ONE;
    assign flush_row_valid_w = (flush_row_w >= 0) &&
                               (flush_row_w < P_INPUT_HEIGHT) &&
                               mixed_valid_comb;
    assign flush_row_unsigned_w = flush_row_w[LP_ROW_WIDTH-1:0];
    assign update_weight_w = update_weight_comb;
    assign update_weight_ext_w = update_weight_w;

    spike_event_scanner #(
        .P_NUM_INPUT_PIXELS  (P_NUM_INPUT_PIXELS),
        .P_EVENT_ADDR_WIDTH  (LP_ADDR_WIDTH),
        .P_WORD_WIDTH        (32)
    ) u_spike_event_scanner (
        .clk                 (clk),
        .rst_n               (rst_n),
        .i_start             (scanner_start_reg),
        .i_spike_vector      (input_union_w),
        .i_event_ready       (scanner_event_ready_w),
        .o_event_valid       (scanner_event_valid_w),
        .o_event_addr        (scanner_event_addr_w),
        .o_done              (scanner_done_w),
        .o_busy              (),
        .o_event_count       ()
    );

    // 一个空间事件携带两位极性；同位置的两路贡献合并后才允许退休。
    assign input_union_w = input_on_reg | input_off_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_on_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
            input_off_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
            event_polarity_reg <= 2'b00;
        end else begin
            if (state_reg == S_IDLE && i_start) begin
                input_on_reg <= i_input_spike_vector_on;
                input_off_reg <= i_input_spike_vector_off;
            end
            if (event_accept_w) begin
                event_polarity_reg <= {
                    input_off_reg[P_NUM_INPUT_PIXELS - 1 - scanner_event_addr_w],
                    input_on_reg[P_NUM_INPUT_PIXELS - 1 - scanner_event_addr_w]};
            end
        end
    end

    function automatic signed [P_NEURON_VALUE_TOTAL_BITS-1:0] get_kernel_weight;
        input integer kernel_idx;
        input [1:0] polarity;
        integer bit_base;
        reg signed [P_WEIGHT_BIT_WIDTH-1:0] weight_on;
        reg signed [P_WEIGHT_BIT_WIDTH-1:0] weight_off;
        reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] weight_on_ext;
        reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] weight_off_ext;
        begin
            bit_base = (P_KERNEL_ELEMS - 1 - kernel_idx) * P_WEIGHT_BIT_WIDTH;
            weight_on = P_KERNEL_ON_WEIGHTS_PACKED[bit_base +: P_WEIGHT_BIT_WIDTH];
            weight_off = P_KERNEL_OFF_WEIGHTS_PACKED[bit_base +: P_WEIGHT_BIT_WIDTH];
            // 先扩展到电流位宽，再相加；不能先在16位内求和。
            weight_on_ext = {{(P_NEURON_VALUE_TOTAL_BITS - P_WEIGHT_BIT_WIDTH){weight_on[P_WEIGHT_BIT_WIDTH-1]}}, weight_on};
            weight_off_ext = {{(P_NEURON_VALUE_TOTAL_BITS - P_WEIGHT_BIT_WIDTH){weight_off[P_WEIGHT_BIT_WIDTH-1]}}, weight_off};
            get_kernel_weight =
                (polarity[0] ? weight_on_ext : {P_NEURON_VALUE_TOTAL_BITS{1'b0}}) +
                (polarity[1] ? weight_off_ext : {P_NEURON_VALUE_TOTAL_BITS{1'b0}});
        end
    endfunction

    always @(*) begin
        event_row_comb = scanner_event_addr_w / P_INPUT_WIDTH;
        event_col_comb = scanner_event_addr_w % P_INPUT_WIDTH;
    end

    /*
     * row_sel: 0=旧输出行 y-1，1=当前输出行 y，2=新输出行 y+1。
     */
    always @(*) begin
        update_kernel_row_comb = update_idx_reg / P_KERNEL_SIZE;
        update_kernel_col_comb = update_idx_reg % P_KERNEL_SIZE;
        update_kernel_idx_comb = update_idx_reg;
        update_col_int_comb = event_col_reg - update_kernel_col_comb + P_PADDING;
        update_col_comb = update_col_int_comb[LP_COL_WIDTH-1:0];
        update_weight_comb = get_kernel_weight(update_kernel_idx_comb, event_polarity_reg);
        update_valid_comb = (update_col_int_comb >= 0) && (update_col_int_comb < P_INPUT_WIDTH);

        if (update_kernel_row_comb == 0) begin
            update_row_sel_comb = 2'd2;
            update_valid_comb = update_valid_comb &&
                                ((active_event_row_reg + LP_ROW_STEP_ONE) >= 0) &&
                                ((active_event_row_reg + LP_ROW_STEP_ONE) < P_INPUT_HEIGHT);
        end else if (update_kernel_row_comb == 1) begin
            update_row_sel_comb = 2'd1;
            update_valid_comb = update_valid_comb &&
                                (active_event_row_reg >= 0) &&
                                (active_event_row_reg < P_INPUT_HEIGHT);
        end else begin
            update_row_sel_comb = 2'd0;
            update_valid_comb = update_valid_comb &&
                                ((active_event_row_reg - LP_ROW_STEP_ONE) >= 0) &&
                                ((active_event_row_reg - LP_ROW_STEP_ONE) < P_INPUT_HEIGHT);
        end
    end

    always @(*) begin
        mixed_valid_comb = 1'b0;
        mixed_data_comb = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
        mixed_active_comb = 1'b0;

        if (mixed_buf_idx_reg == 2'd0) begin
            mixed_valid_comb = buf0_valid_reg;
            mixed_data_comb = buf0_data_reg[retire_col_reg];
            mixed_active_comb = buf0_active_reg[retire_col_reg];
        end else begin
            mixed_valid_comb = buf1_valid_reg;
            mixed_data_comb = buf1_data_reg[retire_col_reg];
            mixed_active_comb = buf1_active_reg[retire_col_reg];
        end
    end

    always @(*) begin
        frontier_update_valid_comb = 1'b0;
        frontier_update_idx_comb = 2'd0;
        frontier_overflow_comb = 1'b0;
        release_data_comb = frontier_data_reg[0];
        release_active_comb = frontier_active_reg[0];
        frontier_offset_comb = update_col_comb - retire_col_reg;

        if (update_col_comb >= retire_col_reg) begin
            if (frontier_offset_comb < 3) begin
                frontier_update_valid_comb = 1'b1;
                frontier_update_idx_comb = frontier_offset_comb[1:0];
            end else begin
                frontier_overflow_comb = 1'b1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg <= S_IDLE;
        end else begin
            state_reg <= state_next;
        end
    end

    always @(*) begin
        state_next = state_reg;

        case (state_reg)
            S_IDLE: begin
                if (i_start) begin
                    state_next = S_WAIT_EVENT;
                end
            end

            S_WAIT_EVENT: begin
                if (pending_row_ahead_w || accepted_row_ahead_w) begin
                    state_next = S_FINISH_ROW;
                end else if (pending_event_valid_reg) begin
                    if (retire_before_needed_w) begin
                        state_next = S_RETIRE_BEFORE;
                    end else begin
                        state_next = S_PROCESS_EVENT;
                    end
                end else if (event_accept_w) begin
                    state_next = S_WAIT_EVENT;
                end else if (scanner_done_w || scanner_done_seen_reg) begin
                    if (active_row_valid_reg && active_event_row_reg <= P_INPUT_HEIGHT) begin
                        state_next = S_FINISH_ROW;
                    end else begin
                        state_next = S_DONE;
                    end
                end
            end

            S_RETIRE_BEFORE: begin
                if (!retire_before_needed_w) begin
                    state_next = S_PROCESS_EVENT;
                end
            end

            S_PROCESS_EVENT: begin
                if (update_idx_reg == P_KERNEL_ELEMS - 1) begin
                    if (retire_after_needed_w) begin
                        state_next = S_RETIRE_AFTER;
                    end else begin
                        state_next = S_WAIT_EVENT;
                    end
                end
            end

            S_RETIRE_AFTER: begin
                if (!retire_after_needed_w) begin
                    state_next = S_WAIT_EVENT;
                end
            end

            S_FINISH_ROW: begin
                if (!finish_needed_w) begin
                    state_next = S_WAIT_EVENT;
                end
            end

            S_DONE: begin
                state_next = S_IDLE;
            end

            default: begin
                state_next = S_IDLE;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scanner_start_reg <= 1'b0;
            scanner_done_seen_reg <= 1'b0;
            active_row_valid_reg <= 1'b0;
            active_event_row_reg <= {LP_ROW_SIGNED_WIDTH{1'b0}};
            pending_event_valid_reg <= 1'b0;
            event_row_reg <= {LP_ROW_SIGNED_WIDTH{1'b0}};
            event_col_reg <= {LP_COL_WIDTH{1'b0}};
            update_idx_reg <= 4'd0;
            retire_col_reg <= {LP_COL_WIDTH{1'b0}};
            retire_done_reg <= 1'b0;
            mixed_buf_idx_reg <= 2'd0;
            middle_buf_idx_reg <= 2'd1;
            o_done <= 1'b0;
        end else begin
            scanner_start_reg <= 1'b0;
            o_done <= 1'b0;

            if (state_reg == S_IDLE && i_start) begin
                scanner_start_reg <= 1'b1;
                scanner_done_seen_reg <= 1'b0;
                active_row_valid_reg <= 1'b0;
                active_event_row_reg <= {LP_ROW_SIGNED_WIDTH{1'b0}};
                pending_event_valid_reg <= 1'b0;
                update_idx_reg <= 4'd0;
                retire_col_reg <= {LP_COL_WIDTH{1'b0}};
                retire_done_reg <= 1'b0;
                mixed_buf_idx_reg <= 2'd0;
                middle_buf_idx_reg <= 2'd1;
            end

            if (scanner_done_w) begin
                scanner_done_seen_reg <= 1'b1;
            end

            if (event_accept_w) begin
                pending_event_valid_reg <= 1'b1;
                event_row_reg <= event_row_comb;
                event_col_reg <= event_col_comb;
                update_idx_reg <= 4'd0;
                if (!active_row_valid_reg) begin
                    active_row_valid_reg <= 1'b1;
                    active_event_row_reg <= event_row_comb;
                    retire_col_reg <= {LP_COL_WIDTH{1'b0}};
                    retire_done_reg <= 1'b0;
                end
            end

            if (state_reg == S_PROCESS_EVENT) begin
                if (update_idx_reg == P_KERNEL_ELEMS - 1) begin
                    update_idx_reg <= 4'd0;
                end else begin
                    update_idx_reg <= update_idx_reg + 1'b1;
                end
            end

            if (retire_fire_w && finish_needed_w) begin
                if (retire_last_col_w) begin
                    retire_done_reg <= 1'b1;
                end else begin
                    retire_col_reg <= retire_col_reg + 1'b1;
                end
            end

            if (state_reg == S_RETIRE_AFTER && !retire_after_needed_w) begin
                pending_event_valid_reg <= 1'b0;
            end else if (state_reg == S_PROCESS_EVENT &&
                         update_idx_reg == P_KERNEL_ELEMS - 1 &&
                         !retire_after_needed_w) begin
                pending_event_valid_reg <= 1'b0;
            end

            if (state_reg == S_FINISH_ROW && !finish_needed_w) begin
                active_event_row_reg <= active_event_row_reg + LP_ROW_STEP_ONE;
                retire_col_reg <= {LP_COL_WIDTH{1'b0}};
                retire_done_reg <= 1'b0;
                mixed_buf_idx_reg <= middle_buf_idx_reg;
                middle_buf_idx_reg <= mixed_buf_idx_reg;
            end

            if (state_reg == S_DONE) begin
                o_done <= 1'b1;
                scanner_done_seen_reg <= 1'b0;
                active_row_valid_reg <= 1'b0;
                pending_event_valid_reg <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf0_valid_reg <= 1'b0;
            buf1_valid_reg <= 1'b0;
            buf2_valid_reg <= 1'b0;
            for (reset_col_idx = 0; reset_col_idx < P_INPUT_WIDTH; reset_col_idx = reset_col_idx + 1) begin
                buf0_data_reg[reset_col_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                buf1_data_reg[reset_col_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                buf0_active_reg[reset_col_idx] <= 1'b0;
                buf1_active_reg[reset_col_idx] <= 1'b0;
            end
            for (frontier_idx = 0; frontier_idx < 3; frontier_idx = frontier_idx + 1) begin
                frontier_valid_reg[frontier_idx] <= 1'b0;
                frontier_data_reg[frontier_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                frontier_active_reg[frontier_idx] <= 1'b0;
            end
        end else begin
            if (state_reg == S_IDLE && i_start) begin
                buf0_valid_reg <= 1'b1;
                buf1_valid_reg <= 1'b1;
                buf2_valid_reg <= 1'b0;
                for (reset_col_idx = 0; reset_col_idx < P_INPUT_WIDTH; reset_col_idx = reset_col_idx + 1) begin
                    buf0_data_reg[reset_col_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                    buf1_data_reg[reset_col_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                    buf0_active_reg[reset_col_idx] <= 1'b0;
                    buf1_active_reg[reset_col_idx] <= 1'b0;
                end
                for (frontier_idx = 0; frontier_idx < 3; frontier_idx = frontier_idx + 1) begin
                    frontier_valid_reg[frontier_idx] <= 1'b0;
                    frontier_data_reg[frontier_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                    frontier_active_reg[frontier_idx] <= 1'b0;
                end
            end else begin
                if (retire_fire_w && finish_needed_w) begin
                    if (mixed_buf_idx_reg == 2'd0) begin
                        buf0_data_reg[retire_col_reg] <= release_data_comb;
                        buf0_active_reg[retire_col_reg] <= release_active_comb;
                    end else begin
                        buf1_data_reg[retire_col_reg] <= release_data_comb;
                        buf1_active_reg[retire_col_reg] <= release_active_comb;
                    end

                    frontier_valid_reg[0] <= frontier_valid_reg[1];
                    frontier_data_reg[0] <= frontier_data_reg[1];
                    frontier_active_reg[0] <= frontier_active_reg[1];
                    frontier_valid_reg[1] <= frontier_valid_reg[2];
                    frontier_data_reg[1] <= frontier_data_reg[2];
                    frontier_active_reg[1] <= frontier_active_reg[2];
                    frontier_valid_reg[2] <= 1'b0;
                    frontier_data_reg[2] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                    frontier_active_reg[2] <= 1'b0;
                end

                if (state_reg == S_PROCESS_EVENT && update_valid_comb) begin
                    if (update_row_sel_comb == 2'd0) begin
                        if (mixed_buf_idx_reg == 2'd0) begin
                            buf0_data_reg[update_col_comb] <= buf0_data_reg[update_col_comb] + update_weight_ext_w;
                            buf0_active_reg[update_col_comb] <= 1'b1;
                        end else begin
                            buf1_data_reg[update_col_comb] <= buf1_data_reg[update_col_comb] + update_weight_ext_w;
                            buf1_active_reg[update_col_comb] <= 1'b1;
                        end
                    end else if (update_row_sel_comb == 2'd1) begin
                        if (middle_buf_idx_reg == 2'd0) begin
                            buf0_data_reg[update_col_comb] <= buf0_data_reg[update_col_comb] + update_weight_ext_w;
                            buf0_active_reg[update_col_comb] <= 1'b1;
                        end else begin
                            buf1_data_reg[update_col_comb] <= buf1_data_reg[update_col_comb] + update_weight_ext_w;
                            buf1_active_reg[update_col_comb] <= 1'b1;
                        end
                    end else if (update_col_comb < retire_col_reg) begin
                        if (mixed_buf_idx_reg == 2'd0) begin
                            buf0_data_reg[update_col_comb] <= buf0_data_reg[update_col_comb] + update_weight_ext_w;
                            buf0_active_reg[update_col_comb] <= 1'b1;
                        end else begin
                            buf1_data_reg[update_col_comb] <= buf1_data_reg[update_col_comb] + update_weight_ext_w;
                            buf1_active_reg[update_col_comb] <= 1'b1;
                        end
                    end else if (frontier_update_valid_comb) begin
                        frontier_valid_reg[frontier_update_idx_comb] <= 1'b1;
                        if (frontier_valid_reg[frontier_update_idx_comb]) begin
                            frontier_data_reg[frontier_update_idx_comb] <=
                                frontier_data_reg[frontier_update_idx_comb] + update_weight_ext_w;
                        end else begin
                            frontier_data_reg[frontier_update_idx_comb] <= update_weight_ext_w;
                        end
                        frontier_active_reg[frontier_update_idx_comb] <= 1'b1;
                    end
                end

                if (state_reg == S_FINISH_ROW && !finish_needed_w) begin
                    for (frontier_idx = 0; frontier_idx < 3; frontier_idx = frontier_idx + 1) begin
                        frontier_valid_reg[frontier_idx] <= 1'b0;
                        frontier_data_reg[frontier_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                        frontier_active_reg[frontier_idx] <= 1'b0;
                    end
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_current_valid <= 1'b0;
            o_current_addr <= {LP_ADDR_WIDTH{1'b0}};
            o_current_data <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            o_current_active <= 1'b0;
        end else begin
            o_current_valid <= 1'b0;
            o_current_active <= 1'b0;

            if (retire_fire_w && finish_needed_w && flush_row_valid_w) begin
                o_current_valid <= 1'b1;
                o_current_addr <= P_NUM_INPUT_PIXELS - 1 -
                    ((flush_row_unsigned_w * P_INPUT_WIDTH) + retire_col_reg);
                o_current_data <= mixed_data_comb;
                o_current_active <= mixed_active_comb;
            end
        end
    end

endmodule
