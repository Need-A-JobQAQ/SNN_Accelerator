module conv_lif_sparse_core #(
    parameter P_GLOBAL_NUM_NEURONS          = 1568,
    parameter P_CORE_START_ADDR             = 0,
    parameter P_CORE_NUM_NEURONS            = 392,
    parameter P_NUM_INPUT_PIXELS            = 784,
    parameter P_INPUT_HEIGHT                = 28,
    parameter P_INPUT_WIDTH                 = 28,
    parameter P_KERNEL_SIZE                 = 3,
    parameter P_PADDING                     = 1,
    parameter P_NEURON_VALUE_TOTAL_BITS     = 26,
    parameter P_NEURON_VALUE_FRAC_BITS      = 12,
    parameter P_SKIP_THRESHOLD_SHIFT        = 5,
    parameter P_USE_CURRENT_RAM_INPUT       = 0
) (
    // 基础控制信号
    input  wire                                                       clk,
    input  wire                                                       rst_n,
    input  wire                                                       i_enable_core,

    // 本 core 的时间步输入与稀疏调度信息
    input  wire [P_NUM_INPUT_PIXELS-1:0]                              i_input_spike_vector,
    input  wire [P_CORE_NUM_NEURONS-1:0]                              i_current_valid_bitmap,

    // 旧版完整电流数组接口；P_USE_CURRENT_RAM_INPUT=0 时用于兼容旧路径
    input  wire signed [P_GLOBAL_NUM_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0]
                                                                      i_all_currents_I,

    // 外部 current RAM 读数据与仲裁握手；P_USE_CURRENT_RAM_INPUT=1 时使用
    input  wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]                i_current_rd_data,
    input  wire                                                       i_current_rd_valid,
    input  wire                                                       i_current_rd_ready,

    // 外部 current RAM 读请求
    output wire                                                       o_current_rd_en,
    output wire [$clog2(P_GLOBAL_NUM_NEURONS)-1:0]                    o_current_rd_addr,

    // 本 core 的脉冲向量输出与运行状态
    output reg  [P_CORE_NUM_NEURONS-1:0]                              o_core_spikes_out,
    output reg                                                        o_core_done,
    output wire                                                       o_core_ready,

    // 本 core 产生的 AER 事件流
    output reg                                                        o_event_valid,
    output reg  [$clog2(P_GLOBAL_NUM_NEURONS)-1:0]                    o_event_addr,

    // 本 core 的性能统计
    output reg  [31:0]                                                o_skip_count,
    output reg  [31:0]                                                o_update_count,
    output reg  [31:0]                                                o_event_count
);

    /*
     * 一个局部卷积 LIF 神经核心。
     * 每个 core 只处理全局神经元地址空间中的一段连续地址，
     * 并独立维护自己的膜电位 RAM 和 active bitmap。
     */
    localparam LP_LOCAL_ADDR_WIDTH = $clog2(P_CORE_NUM_NEURONS);
    localparam LP_GLOBAL_ADDR_WIDTH = $clog2(P_GLOBAL_NUM_NEURONS);
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

    reg [P_CORE_NUM_NEURONS-1:0] latched_current_valid_bitmap_reg;
    reg [P_CORE_NUM_NEURONS-1:0] active_state_bitmap_reg;
    reg [LP_LOCAL_ADDR_WIDTH-1:0] clear_addr_reg;
    reg [LP_LOCAL_ADDR_WIDTH-1:0] addr_pipeline_reg [BRAM_READ_LATENCY-1:0];
    reg valid_pipeline_reg [BRAM_READ_LATENCY-1:0];
    reg current_valid_pipeline_reg [BRAM_READ_LATENCY-1:0];
    reg pipeline_busy_comb;

    wire scan_addr_valid_w;
    wire scan_addr_valid_raw_w;
    wire [LP_LOCAL_ADDR_WIDTH-1:0] scan_local_addr_w;
    wire [LP_GLOBAL_ADDR_WIDTH-1:0] scan_global_addr_w;
    wire scan_current_valid_w;
    wire scan_can_issue_w;
    wire scan_current_accept_w;
    wire scan_need_update_w;
    wire pipeline_busy_w;
    wire [P_CORE_NUM_NEURONS-1:0] active_bitmap_start_w;

    wire compactor_start_w;
    wire compactor_addr_ready_w;
    wire compactor_addr_valid_w;
    wire [LP_LOCAL_ADDR_WIDTH-1:0] compactor_addr_w;
    wire compactor_done_w;
    wire compactor_busy_w;
    wire [31:0] compactor_active_count_w;

    wire clear_membrane_en_w;
    wire membrane_ram_write_en_w;
    wire [LP_LOCAL_ADDR_WIDTH-1:0] membrane_ram_write_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] membrane_ram_write_data_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] membrane_ram_read_data_w;
    wire lif_membrane_write_en_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] lif_membrane_write_data_w;

    wire [LP_LOCAL_ADDR_WIDTH-1:0] process_local_addr_w;
    wire [LP_GLOBAL_ADDR_WIDTH-1:0] process_global_addr_w;
    wire process_has_current_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_input_current_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_membrane_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_diff_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_delta_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_candidate_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_abs_candidate_w;
    wire process_spike_w;
    wire process_next_active_w;
    wire [LP_GLOBAL_ADDR_WIDTH-1:0] process_event_addr_w;

    integer reset_idx;
    integer pipe_idx;
    integer busy_idx;

    assign o_core_ready = (current_state_reg == S_IDLE);
    assign active_bitmap_start_w = i_current_valid_bitmap | active_state_bitmap_reg;
    assign compactor_start_w = (current_state_reg == S_IDLE) && (next_state_reg == S_PROCESSING);
    /*
     * current RAM 输入模式下，如果当前地址需要读取电流，
     * 必须等外部仲裁器允许读请求后才消耗该地址。
     * 膜电位衰减地址不需要读 current RAM，可以直接进入流水线。
     */
    assign scan_addr_valid_raw_w = (current_state_reg == S_PROCESSING) && compactor_addr_valid_w;
    assign scan_can_issue_w = (!P_USE_CURRENT_RAM_INPUT) ||
                              (!scan_current_valid_w) ||
                              i_current_rd_ready;
    assign compactor_addr_ready_w = (current_state_reg == S_PROCESSING) && scan_can_issue_w;
    assign scan_addr_valid_w = scan_addr_valid_raw_w && scan_can_issue_w;
    assign scan_local_addr_w = compactor_addr_w;
    assign scan_global_addr_w = P_CORE_START_ADDR + scan_local_addr_w;
    assign scan_current_valid_w = latched_current_valid_bitmap_reg[scan_local_addr_w];
    assign scan_current_accept_w = scan_addr_valid_w && scan_current_valid_w;
    assign scan_need_update_w = scan_addr_valid_w;
    assign o_current_rd_en = scan_addr_valid_raw_w && scan_current_valid_w;
    assign o_current_rd_addr = scan_global_addr_w;

    assign pipeline_busy_w = pipeline_busy_comb;
    assign process_local_addr_w = addr_pipeline_reg[BRAM_READ_LATENCY-1];
    assign process_global_addr_w = P_CORE_START_ADDR + process_local_addr_w;
    assign process_has_current_w = current_valid_pipeline_reg[BRAM_READ_LATENCY-1];
    assign clear_membrane_en_w = (current_state_reg == S_CLEAR);
    assign membrane_ram_write_en_w = clear_membrane_en_w || lif_membrane_write_en_w;
    assign membrane_ram_write_addr_w = clear_membrane_en_w ? clear_addr_reg : process_local_addr_w;
    assign membrane_ram_write_data_w = clear_membrane_en_w ? {P_NEURON_VALUE_TOTAL_BITS{1'b0}} :
                                       lif_membrane_write_data_w;
    assign lif_membrane_write_en_w = ((current_state_reg == S_PROCESSING) ||
                                      (current_state_reg == S_FLUSHING)) &&
                                     valid_pipeline_reg[BRAM_READ_LATENCY-1];
    assign lif_membrane_write_data_w = process_spike_w ? LP_V_RESET_FIXED : process_candidate_w;
    assign process_input_current_w = P_USE_CURRENT_RAM_INPUT ?
                                     ((process_has_current_w && i_current_rd_valid) ? i_current_rd_data :
                                      {P_NEURON_VALUE_TOTAL_BITS{1'b0}}) :
                                     i_all_currents_I[process_global_addr_w];
    assign process_membrane_w = membrane_ram_read_data_w;

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_LOCAL_ADDR_WIDTH),
        .P_DEPTH        (P_CORE_NUM_NEURONS)
    ) u_membrane_potential_ram (
        .clk            (clk),
        .i_write_en     (membrane_ram_write_en_w),
        .i_write_addr   (membrane_ram_write_addr_w),
        .i_write_data   (membrane_ram_write_data_w),
        .i_read_en      (scan_need_update_w),
        .i_read_addr    (scan_local_addr_w),
        .o_read_data    (membrane_ram_read_data_w)
    );

    /*
     * 本 core 只把需要更新的本地地址压缩成地址流。
     * current_valid_bitmap 表示本时间步有输入电流的地址；
     * active_state_bitmap_reg 表示上一时间步后膜电位仍不可忽略的地址。
     */
    bitmap_to_addr_stream #(
        .P_BITMAP_WIDTH  (P_CORE_NUM_NEURONS),
        .P_ADDR_WIDTH    (LP_LOCAL_ADDR_WIDTH)
    ) u_active_bitmap_compactor (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_start         (compactor_start_w),
        .i_active_bitmap (active_bitmap_start_w),
        .i_addr_ready    (compactor_addr_ready_w),
        .o_addr_valid    (compactor_addr_valid_w),
        .o_addr          (compactor_addr_w),
        .o_done          (compactor_done_w),
        .o_busy          (compactor_busy_w),
        .o_active_count  (compactor_active_count_w)
    );

    assign process_diff_w = process_input_current_w - process_membrane_w;
    assign process_delta_w = process_diff_w >>> 1;
    assign process_candidate_w = process_membrane_w + process_delta_w;
    assign process_spike_w = (process_candidate_w >= LP_V_THRESHOLD_FIXED);
    assign process_abs_candidate_w = process_candidate_w[P_NEURON_VALUE_TOTAL_BITS-1] ?
                                     -process_candidate_w : process_candidate_w;
    assign process_next_active_w = process_spike_w ? 1'b0 :
                                   (process_abs_candidate_w >= LP_SKIP_THRESHOLD_FIXED);
    assign process_event_addr_w = (P_GLOBAL_NUM_NEURONS - 1) - process_global_addr_w;

    always @(*) begin
        pipeline_busy_comb = 1'b0;
        for (busy_idx = 0; busy_idx < BRAM_READ_LATENCY; busy_idx = busy_idx + 1) begin
            pipeline_busy_comb = pipeline_busy_comb || valid_pipeline_reg[busy_idx];
        end
    end

    always @(*) begin
        next_state_reg = current_state_reg;
        case (current_state_reg)
            S_CLEAR: begin
                if (clear_addr_reg == P_CORE_NUM_NEURONS - 1) begin
                    next_state_reg = S_IDLE;
                end
            end
            S_IDLE: begin
                if (i_enable_core) begin
                    next_state_reg = S_PROCESSING;
                end
            end
            S_PROCESSING: begin
                if (compactor_done_w) begin
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

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state_reg <= S_CLEAR;
        end else begin
            current_state_reg <= next_state_reg;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clear_addr_reg <= {LP_LOCAL_ADDR_WIDTH{1'b0}};
        end else if (current_state_reg == S_CLEAR) begin
            if (clear_addr_reg != P_CORE_NUM_NEURONS - 1) begin
                clear_addr_reg <= clear_addr_reg + 1'b1;
            end
        end else begin
            clear_addr_reg <= {LP_LOCAL_ADDR_WIDTH{1'b0}};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latched_current_valid_bitmap_reg <= {P_CORE_NUM_NEURONS{1'b0}};
        end else if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
            latched_current_valid_bitmap_reg <= i_current_valid_bitmap;
        end else if (current_state_reg == S_DONE) begin
            latched_current_valid_bitmap_reg <= {P_CORE_NUM_NEURONS{1'b0}};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (pipe_idx = 0; pipe_idx < BRAM_READ_LATENCY; pipe_idx = pipe_idx + 1) begin
                addr_pipeline_reg[pipe_idx] <= {LP_LOCAL_ADDR_WIDTH{1'b0}};
                valid_pipeline_reg[pipe_idx] <= 1'b0;
                current_valid_pipeline_reg[pipe_idx] <= 1'b0;
            end
        end else begin
            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
                for (pipe_idx = 0; pipe_idx < BRAM_READ_LATENCY; pipe_idx = pipe_idx + 1) begin
                    addr_pipeline_reg[pipe_idx] <= {LP_LOCAL_ADDR_WIDTH{1'b0}};
                    valid_pipeline_reg[pipe_idx] <= 1'b0;
                    current_valid_pipeline_reg[pipe_idx] <= 1'b0;
                end
            end else if (current_state_reg == S_PROCESSING || current_state_reg == S_FLUSHING) begin
                for (pipe_idx = BRAM_READ_LATENCY - 1; pipe_idx > 0; pipe_idx = pipe_idx - 1) begin
                    addr_pipeline_reg[pipe_idx] <= addr_pipeline_reg[pipe_idx-1];
                    valid_pipeline_reg[pipe_idx] <= valid_pipeline_reg[pipe_idx-1];
                    current_valid_pipeline_reg[pipe_idx] <= current_valid_pipeline_reg[pipe_idx-1];
                end

                valid_pipeline_reg[0] <= scan_need_update_w;
                current_valid_pipeline_reg[0] <= scan_current_accept_w;
                if (scan_need_update_w) begin
                    addr_pipeline_reg[0] <= scan_local_addr_w;
                end else begin
                    addr_pipeline_reg[0] <= {LP_LOCAL_ADDR_WIDTH{1'b0}};
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_state_bitmap_reg <= {P_CORE_NUM_NEURONS{1'b0}};
        end else if (current_state_reg == S_CLEAR) begin
            active_state_bitmap_reg <= {P_CORE_NUM_NEURONS{1'b0}};
        end else if (lif_membrane_write_en_w) begin
            active_state_bitmap_reg[process_local_addr_w] <= process_next_active_w;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_core_spikes_out <= {P_CORE_NUM_NEURONS{1'b0}};
            o_event_valid <= 1'b0;
            o_event_addr <= {LP_GLOBAL_ADDR_WIDTH{1'b0}};
            o_skip_count <= 32'd0;
            o_update_count <= 32'd0;
            o_event_count <= 32'd0;
        end else begin
            o_event_valid <= 1'b0;
            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
                o_core_spikes_out <= {P_CORE_NUM_NEURONS{1'b0}};
                o_event_addr <= {LP_GLOBAL_ADDR_WIDTH{1'b0}};
                o_skip_count <= 32'd0;
                o_update_count <= 32'd0;
                o_event_count <= 32'd0;
            end else begin
                if (compactor_done_w) begin
                    o_skip_count <= P_CORE_NUM_NEURONS - compactor_active_count_w;
                end

                if (lif_membrane_write_en_w) begin
                    o_update_count <= o_update_count + 1'b1;
                    if (process_spike_w) begin
                        o_core_spikes_out[process_local_addr_w] <= 1'b1;
                        o_event_valid <= 1'b1;
                        o_event_addr <= process_event_addr_w;
                        o_event_count <= o_event_count + 1'b1;
                    end else begin
                        o_core_spikes_out[process_local_addr_w] <= 1'b0;
                    end
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_core_done <= 1'b0;
        end else begin
            o_core_done <= (current_state_reg == S_DONE);
        end
    end

endmodule
