module conv_lif_layer_sparse #(
    parameter P_NUM_NEURONS = 1568,
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_INPUT_HEIGHT = 28,
    parameter P_INPUT_WIDTH = 28,
    parameter P_KERNEL_SIZE = 3,
    parameter P_PADDING = 1,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter P_NEURON_VALUE_FRAC_BITS = 12,
    parameter P_SKIP_THRESHOLD_SHIFT = 5
) (
    input wire clk,
    input wire rst_n,
    input wire i_enable_layer,
    input wire [P_NUM_INPUT_PIXELS-1:0] i_input_spike_vector,
    input wire signed [P_NUM_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] i_all_currents_I,

    output reg [P_NUM_NEURONS-1:0] o_all_spikes_out,
    output reg o_all_spikes_valid,
    output reg o_event_valid,
    output reg [$clog2(P_NUM_NEURONS)-1:0] o_event_addr,
    output reg o_event_frame_done,
    output wire o_layer_ready,
    output reg [31:0] o_skip_count,
    output reg [31:0] o_update_count
);

    /*
     * 稀疏版卷积 LIF 层。
     *
     * 原 conv_lif_layer 每个时间步都会访问 1568 个膜电位地址。
     * 本模块增加两个判断：
     * 1. 当前输出像素的 3x3 感受野内是否存在输入脉冲；
     * 2. 该神经元上一轮更新后膜电位是否仍然处于 active 状态。
     *
     * 如果感受野无脉冲，并且 active bit 为 0，则认为该神经元接近静息，
     * 本时间步跳过膜电位 RAM 读写和 LIF 更新。
     */
    localparam LP_ADDR_WIDTH = $clog2(P_NUM_NEURONS);
    localparam LP_COUNT_WIDTH = $clog2(P_NUM_NEURONS + 1);
    localparam BRAM_READ_LATENCY = 1;

    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] LP_V_THRESHOLD_FIXED =
        (1'b1 << P_NEURON_VALUE_FRAC_BITS);
    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] LP_V_RESET_FIXED =
        {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] LP_SKIP_THRESHOLD_FIXED =
        (P_NEURON_VALUE_FRAC_BITS > P_SKIP_THRESHOLD_SHIFT) ?
        (1'b1 << (P_NEURON_VALUE_FRAC_BITS - P_SKIP_THRESHOLD_SHIFT)) :
        {{(P_NEURON_VALUE_TOTAL_BITS-1){1'b0}}, 1'b1};

    localparam [2:0] S_CLEAR = 3'b000;
    localparam [2:0] S_IDLE = 3'b001;
    localparam [2:0] S_PROCESSING = 3'b010;
    localparam [2:0] S_FLUSHING = 3'b011;
    localparam [2:0] S_DONE = 3'b100;

    reg [2:0] current_state_reg;
    reg [2:0] next_state_reg;

    reg [P_NUM_INPUT_PIXELS-1:0] latched_input_spikes_reg;
    reg signed [P_NUM_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] latched_currents_reg;
    reg [P_NUM_NEURONS-1:0] active_state_bitmap_reg;

    reg [LP_ADDR_WIDTH-1:0] clear_addr_reg;
    reg [LP_COUNT_WIDTH-1:0] issued_addr_count_reg;

    reg [LP_ADDR_WIDTH-1:0] addr_pipeline_reg [BRAM_READ_LATENCY-1:0];
    reg valid_pipeline_reg [BRAM_READ_LATENCY-1:0];
    reg pipeline_busy_comb;

    wire scan_addr_valid_w;
    wire [LP_ADDR_WIDTH-1:0] scan_addr_w;
    wire scan_rf_active_w;
    wire scan_state_active_w;
    wire scan_need_update_w;
    wire pipeline_busy_w;

    wire clear_membrane_en_w;
    wire membrane_ram_write_en_w;
    wire [LP_ADDR_WIDTH-1:0] membrane_ram_write_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] membrane_ram_write_data_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] membrane_ram_read_data_w;
    wire lif_membrane_write_en_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] lif_membrane_write_data_w;

    wire [LP_ADDR_WIDTH-1:0] process_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_input_current_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_membrane_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_diff_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_delta_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_candidate_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_abs_candidate_w;
    wire process_spike_w;
    wire process_next_active_w;
    wire [LP_ADDR_WIDTH-1:0] process_event_addr_w;

    integer current_reset_idx;
    integer pipe_idx;
    integer busy_idx;

    assign o_layer_ready = (current_state_reg == S_IDLE);
    assign scan_addr_valid_w = (current_state_reg == S_PROCESSING) &&
                               (issued_addr_count_reg < P_NUM_NEURONS);
    assign scan_addr_w = issued_addr_count_reg[LP_ADDR_WIDTH-1:0];
    assign scan_rf_active_w = receptive_field_active(latched_input_spikes_reg, scan_addr_w);
    assign scan_state_active_w = active_state_bitmap_reg[scan_addr_w];
    assign scan_need_update_w = scan_addr_valid_w && (scan_rf_active_w || scan_state_active_w);

    assign pipeline_busy_w = pipeline_busy_comb;
    assign process_addr_w = addr_pipeline_reg[BRAM_READ_LATENCY-1];
    assign clear_membrane_en_w = (current_state_reg == S_CLEAR);
    assign membrane_ram_write_en_w = clear_membrane_en_w || lif_membrane_write_en_w;
    assign membrane_ram_write_addr_w = clear_membrane_en_w ? clear_addr_reg : process_addr_w;
    assign membrane_ram_write_data_w = clear_membrane_en_w ? {P_NEURON_VALUE_TOTAL_BITS{1'b0}} :
                                       lif_membrane_write_data_w;
    assign lif_membrane_write_en_w = ((current_state_reg == S_PROCESSING) ||
                                      (current_state_reg == S_FLUSHING)) &&
                                     valid_pipeline_reg[BRAM_READ_LATENCY-1];
    assign lif_membrane_write_data_w = process_spike_w ? LP_V_RESET_FIXED : process_candidate_w;
    assign process_input_current_w = latched_currents_reg[process_addr_w];
    assign process_membrane_w = membrane_ram_read_data_w;

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_ADDR_WIDTH),
        .P_DEPTH        (P_NUM_NEURONS)
    ) u_membrane_potential_ram (
        .clk            (clk),
        .i_write_en     (membrane_ram_write_en_w),
        .i_write_addr   (membrane_ram_write_addr_w),
        .i_write_data   (membrane_ram_write_data_w),
        .i_read_en      (scan_need_update_w),
        .i_read_addr    (scan_addr_w),
        .o_read_data    (membrane_ram_read_data_w)
    );

    /*
     * 输入脉冲向量沿用工程约定：
     * 最高位对应左上角像素，最低位对应右下角像素。
     */
    function get_input_spike;
        input [P_NUM_INPUT_PIXELS-1:0] spikes;
        input integer row;
        input integer col;
        integer flat_idx;
        begin
            if (row < 0 || row >= P_INPUT_HEIGHT || col < 0 || col >= P_INPUT_WIDTH) begin
                get_input_spike = 1'b0;
            end else begin
                flat_idx = (row * P_INPUT_WIDTH) + col;
                get_input_spike = spikes[P_NUM_INPUT_PIXELS - 1 - flat_idx];
            end
        end
    endfunction

    /*
     * 根据卷积后神经元地址反推出对应的输出像素位置，
     * 再检查该输出像素 3x3 感受野内是否存在输入脉冲。
     */
    function receptive_field_active;
        input [P_NUM_INPUT_PIXELS-1:0] spikes;
        input [LP_ADDR_WIDTH-1:0] neuron_addr;
        integer local_addr;
        integer spatial_idx;
        integer out_row;
        integer out_col;
        integer kernel_row;
        integer kernel_col;
        begin
            receptive_field_active = 1'b0;

            local_addr = neuron_addr % P_NUM_INPUT_PIXELS;
            spatial_idx = P_NUM_INPUT_PIXELS - 1 - local_addr;
            out_row = spatial_idx / P_INPUT_WIDTH;
            out_col = spatial_idx % P_INPUT_WIDTH;

            for (kernel_row = 0; kernel_row < P_KERNEL_SIZE; kernel_row = kernel_row + 1) begin
                for (kernel_col = 0; kernel_col < P_KERNEL_SIZE; kernel_col = kernel_col + 1) begin
                    if (get_input_spike(spikes,
                                        out_row + kernel_row - P_PADDING,
                                        out_col + kernel_col - P_PADDING)) begin
                        receptive_field_active = 1'b1;
                    end
                end
            end
        end
    endfunction

    /*
     * tau 固定为 2，因此泄漏积分仍然用算术右移 1 位实现。
     */
    assign process_diff_w = process_input_current_w - process_membrane_w;
    assign process_delta_w = process_diff_w >>> 1;
    assign process_candidate_w = process_membrane_w + process_delta_w;
    assign process_spike_w = (process_candidate_w >= LP_V_THRESHOLD_FIXED);
    assign process_abs_candidate_w = process_candidate_w[P_NEURON_VALUE_TOTAL_BITS-1] ?
                                     -process_candidate_w : process_candidate_w;
    assign process_next_active_w = process_spike_w ? 1'b0 :
                                   (process_abs_candidate_w >= LP_SKIP_THRESHOLD_FIXED);
    assign process_event_addr_w = (P_NUM_NEURONS - 1) - process_addr_w;

    /*
     * 判断 RAM 读请求流水线里是否还有未完成的神经元。
     */
    always @(*) begin
        pipeline_busy_comb = 1'b0;
        for (busy_idx = 0; busy_idx < BRAM_READ_LATENCY; busy_idx = busy_idx + 1) begin
            pipeline_busy_comb = pipeline_busy_comb || valid_pipeline_reg[busy_idx];
        end
    end

    /*
     * 两段式状态机的组合转移逻辑。
     */
    always @(*) begin
        next_state_reg = current_state_reg;

        case (current_state_reg)
            S_CLEAR: begin
                if (clear_addr_reg == P_NUM_NEURONS - 1) begin
                    next_state_reg = S_IDLE;
                end
            end

            S_IDLE: begin
                if (i_enable_layer) begin
                    next_state_reg = S_PROCESSING;
                end
            end

            S_PROCESSING: begin
                if (issued_addr_count_reg == P_NUM_NEURONS) begin
                    next_state_reg = S_FLUSHING;
                end
            end

            S_FLUSHING: begin
                if (!pipeline_busy_w) begin
                    next_state_reg = S_DONE;
                end
            end

            S_DONE: begin
                next_state_reg = S_IDLE;
            end

            default: begin
                next_state_reg = S_IDLE;
            end
        endcase
    end

    /*
     * 状态寄存器。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state_reg <= S_CLEAR;
        end else begin
            current_state_reg <= next_state_reg;
        end
    end

    /*
     * 复位后逐地址清零膜电位 RAM。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clear_addr_reg <= {LP_ADDR_WIDTH{1'b0}};
        end else if (current_state_reg == S_CLEAR) begin
            if (clear_addr_reg != P_NUM_NEURONS - 1) begin
                clear_addr_reg <= clear_addr_reg + 1'b1;
            end
        end else begin
            clear_addr_reg <= {LP_ADDR_WIDTH{1'b0}};
        end
    end

    /*
     * 每个时间步启动时锁存输入脉冲图和卷积电流，保证处理过程中输入稳定。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latched_input_spikes_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
            for (current_reset_idx = 0; current_reset_idx < P_NUM_NEURONS; current_reset_idx = current_reset_idx + 1) begin
                latched_currents_reg[current_reset_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            end
        end else if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
            latched_input_spikes_reg <= i_input_spike_vector;
            latched_currents_reg <= i_all_currents_I;
        end
    end

    /*
     * 地址扫描与 RAM 读请求流水线。
     * 每个地址都会扫描，但只有 need_update 为 1 时才真正访问膜电位 RAM。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            issued_addr_count_reg <= {LP_COUNT_WIDTH{1'b0}};
            for (pipe_idx = 0; pipe_idx < BRAM_READ_LATENCY; pipe_idx = pipe_idx + 1) begin
                addr_pipeline_reg[pipe_idx] <= {LP_ADDR_WIDTH{1'b0}};
                valid_pipeline_reg[pipe_idx] <= 1'b0;
            end
        end else begin
            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
                issued_addr_count_reg <= {LP_COUNT_WIDTH{1'b0}};
                for (pipe_idx = 0; pipe_idx < BRAM_READ_LATENCY; pipe_idx = pipe_idx + 1) begin
                    addr_pipeline_reg[pipe_idx] <= {LP_ADDR_WIDTH{1'b0}};
                    valid_pipeline_reg[pipe_idx] <= 1'b0;
                end
            end else if (current_state_reg == S_PROCESSING || current_state_reg == S_FLUSHING) begin
                for (pipe_idx = BRAM_READ_LATENCY - 1; pipe_idx > 0; pipe_idx = pipe_idx - 1) begin
                    addr_pipeline_reg[pipe_idx] <= addr_pipeline_reg[pipe_idx-1];
                    valid_pipeline_reg[pipe_idx] <= valid_pipeline_reg[pipe_idx-1];
                end

                valid_pipeline_reg[0] <= scan_need_update_w;
                if (scan_need_update_w) begin
                    addr_pipeline_reg[0] <= scan_addr_w;
                end else begin
                    addr_pipeline_reg[0] <= {LP_ADDR_WIDTH{1'b0}};
                end

                if (scan_addr_valid_w) begin
                    issued_addr_count_reg <= issued_addr_count_reg + 1'b1;
                end
            end
        end
    end

    /*
     * active bit 只在真正完成 LIF 更新后修改。
     * 发放脉冲后膜电位复位，因此 active bit 也清 0。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_state_bitmap_reg <= {P_NUM_NEURONS{1'b0}};
        end else if (current_state_reg == S_CLEAR) begin
            active_state_bitmap_reg <= {P_NUM_NEURONS{1'b0}};
        end else if (lif_membrane_write_en_w) begin
            active_state_bitmap_reg[process_addr_w] <= process_next_active_w;
        end
    end

    /*
     * 输出脉冲、AER 事件和统计计数。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_all_spikes_out <= {P_NUM_NEURONS{1'b0}};
            o_event_valid <= 1'b0;
            o_event_addr <= {LP_ADDR_WIDTH{1'b0}};
            o_skip_count <= 32'd0;
            o_update_count <= 32'd0;
        end else begin
            o_event_valid <= 1'b0;

            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
                o_all_spikes_out <= {P_NUM_NEURONS{1'b0}};
                o_event_addr <= {LP_ADDR_WIDTH{1'b0}};
                o_skip_count <= 32'd0;
                o_update_count <= 32'd0;
            end else begin
                if (scan_addr_valid_w && !scan_need_update_w) begin
                    o_skip_count <= o_skip_count + 1'b1;
                end

                if (lif_membrane_write_en_w) begin
                    o_update_count <= o_update_count + 1'b1;

                    if (process_spike_w) begin
                        o_all_spikes_out[process_addr_w] <= 1'b1;
                        o_event_valid <= 1'b1;
                        o_event_addr <= process_event_addr_w;
                    end else begin
                        o_all_spikes_out[process_addr_w] <= 1'b0;
                    end
                end
            end
        end
    end

    /*
     * 帧结束信号仍然保持和原模块一致，在 S_DONE 状态拉高一个周期。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_all_spikes_valid <= 1'b0;
            o_event_frame_done <= 1'b0;
        end else begin
            o_all_spikes_valid <= (current_state_reg == S_DONE);
            o_event_frame_done <= (current_state_reg == S_DONE);
        end
    end

endmodule
