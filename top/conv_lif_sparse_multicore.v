module conv_lif_sparse_multicore #(
    parameter P_NUM_NEURONS                 = 1568,
    parameter P_NUM_CORES                   = 4,
    parameter P_CORE_NUM_NEURONS            = P_NUM_NEURONS / P_NUM_CORES,
    parameter P_NUM_INPUT_PIXELS            = 784,
    parameter P_INPUT_HEIGHT                = 28,
    parameter P_INPUT_WIDTH                 = 28,
    parameter P_KERNEL_SIZE                 = 3,
    parameter P_PADDING                     = 1,
    parameter P_NEURON_VALUE_TOTAL_BITS     = 26,
    parameter P_NEURON_VALUE_FRAC_BITS      = 12,
    parameter P_SKIP_THRESHOLD_SHIFT        = 5,
    parameter P_CORE_EVENT_FIFO_DEPTH       = 512,
    parameter P_CORE_FIFO_COUNT_WIDTH       = $clog2(P_CORE_EVENT_FIFO_DEPTH + 1),
    parameter P_ARB_POLICY                  = 1
) (
    // 基础控制信号
    input  wire                                                       clk,
    input  wire                                                       rst_n,
    input  wire                                                       i_enable_layer,

    // 本时间步输入与稀疏调度信息
    input  wire [P_NUM_INPUT_PIXELS-1:0]                              i_input_spike_vector,
    input  wire [P_NUM_NEURONS-1:0]                                   i_current_valid_bitmap,

    // 旧版完整电流数组接口，仅用于兼容未更新路径；当前多核路径主要使用 current bank 读口
    input  wire signed [P_NUM_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0]
                                                                      i_all_currents_I,

    // current bank0 读口：供 core0/core1 通过仲裁器访问
    input  wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]                i_current_ch0_rd_data,
    input  wire                                                       i_current_ch0_rd_valid,
    output wire                                                       o_current_ch0_rd_en,
    output wire [$clog2(P_NUM_INPUT_PIXELS)-1:0]                      o_current_ch0_rd_addr,

    // current bank1 读口：供 core2/core3 通过仲裁器访问
    input  wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]                i_current_ch1_rd_data,
    input  wire                                                       i_current_ch1_rd_valid,
    output wire                                                       o_current_ch1_rd_en,
    output wire [$clog2(P_NUM_INPUT_PIXELS)-1:0]                      o_current_ch1_rd_addr,

    // 卷积后 LIF 层输出的完整脉冲向量
    output wire [P_NUM_NEURONS-1:0]                                   o_all_spikes_out,
    output reg                                                        o_all_spikes_valid,
    output wire                                                       o_layer_ready,

    // 多 core 汇聚后的 AER 事件流
    output wire                                                       o_event_valid,
    output wire [$clog2(P_NUM_NEURONS)-1:0]                           o_event_addr,
    output reg                                                        o_event_frame_done,

    // 层级性能统计
    output wire [31:0]                                                o_skip_count,
    output wire [31:0]                                                o_update_count,

    // core/FIFO 级调试与负载观察信息
    output wire [P_NUM_CORES-1:0][31:0]                               o_core_event_count,
    output wire [P_NUM_CORES-1:0][P_CORE_FIFO_COUNT_WIDTH-1:0]        o_core_fifo_count,
    output wire [P_NUM_CORES-1:0][P_CORE_FIFO_COUNT_WIDTH-1:0]        o_core_fifo_max_count,
    output wire                                                       o_core_fifo_overflow
);

    /*
     * 多核稀疏卷积 LIF 层。
     * 4 个 core 分别处理一段连续神经元地址，每个 core 后接本地事件 FIFO，
     * 再由轮询仲裁器汇聚成一路 AER 事件流。
    */
    localparam LP_ADDR_WIDTH = $clog2(P_NUM_NEURONS);
    localparam LP_BANK_ADDR_WIDTH = $clog2(P_NUM_INPUT_PIXELS);

    wire [P_NUM_CORES-1:0] core_ready_w;
    wire [P_NUM_CORES-1:0] core_done_w;
    wire [P_NUM_CORES-1:0] core_event_valid_w;
    wire [P_NUM_CORES-1:0][LP_ADDR_WIDTH-1:0] core_event_addr_w;
    wire [P_NUM_CORES * P_CORE_NUM_NEURONS - 1:0] core_spikes_flat_w;
    wire [P_NUM_CORES-1:0][31:0] core_skip_count_w;
    wire [P_NUM_CORES-1:0][31:0] core_update_count_w;
    wire [P_NUM_CORES-1:0][31:0] core_event_count_w;
    wire [P_NUM_CORES-1:0] core_current_rd_req_w;
    wire [P_NUM_CORES-1:0] core_current_rd_ready_w;
    wire [P_NUM_CORES-1:0] core_current_rd_valid_w;
    wire [P_NUM_CORES-1:0][LP_ADDR_WIDTH-1:0] core_current_rd_addr_w;
    wire signed [P_NUM_CORES-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] core_current_rd_data_w;
    wire [LP_BANK_ADDR_WIDTH-1:0] core0_current_bank_addr_w;
    wire [LP_BANK_ADDR_WIDTH-1:0] core1_current_bank_addr_w;
    wire [LP_BANK_ADDR_WIDTH-1:0] core2_current_bank_addr_w;
    wire [LP_BANK_ADDR_WIDTH-1:0] core3_current_bank_addr_w;

    wire [P_NUM_CORES-1:0] fifo_event_valid_w;
    wire [P_NUM_CORES-1:0][LP_ADDR_WIDTH-1:0] fifo_event_addr_w;
    wire [P_NUM_CORES-1:0] fifo_empty_w;
    wire [P_NUM_CORES-1:0] fifo_full_w;
    wire [P_NUM_CORES-1:0][P_CORE_FIFO_COUNT_WIDTH-1:0] fifo_count_w;
    wire [P_NUM_CORES-1:0] fifo_overflow_w;
    wire [P_NUM_CORES-1:0] arbiter_ready_w;

    wire all_cores_ready_w;
    wire all_cores_done_seen_w;
    wire all_fifos_empty_w;
    wire frame_complete_w;

    reg [P_NUM_CORES-1:0] core_done_seen_reg;
    reg frame_done_pending_reg;
    reg [P_NUM_CORES-1:0][P_CORE_FIFO_COUNT_WIDTH-1:0] core_fifo_max_count_reg;

    genvar core_idx;
    genvar spike_idx;

    assign all_cores_ready_w = &core_ready_w;
    assign all_cores_done_seen_w = &core_done_seen_reg;
    assign all_fifos_empty_w = &fifo_empty_w;
    assign frame_complete_w = frame_done_pending_reg && all_fifos_empty_w;
    assign o_layer_ready = all_cores_ready_w;
    assign o_core_fifo_count = fifo_count_w;
    assign o_core_fifo_max_count = core_fifo_max_count_reg;
    assign o_core_fifo_overflow = |fifo_overflow_w;
    assign o_core_event_count = core_event_count_w;
    assign core0_current_bank_addr_w = core_current_rd_addr_w[0][LP_BANK_ADDR_WIDTH-1:0];
    assign core1_current_bank_addr_w = core_current_rd_addr_w[1][LP_BANK_ADDR_WIDTH-1:0];
    assign core2_current_bank_addr_w = core_current_rd_addr_w[2] - P_NUM_INPUT_PIXELS[LP_ADDR_WIDTH-1:0];
    assign core3_current_bank_addr_w = core_current_rd_addr_w[3] - P_NUM_INPUT_PIXELS[LP_ADDR_WIDTH-1:0];

    /*
     * 统计计数按 core 求和。
     */
    assign o_skip_count = core_skip_count_w[0] + core_skip_count_w[1] +
                          core_skip_count_w[2] + core_skip_count_w[3];
    assign o_update_count = core_update_count_w[0] + core_update_count_w[1] +
                            core_update_count_w[2] + core_update_count_w[3];

    generate
        for (core_idx = 0; core_idx < P_NUM_CORES; core_idx = core_idx + 1) begin : gen_sparse_cores
            conv_lif_sparse_core #(
                .P_GLOBAL_NUM_NEURONS       (P_NUM_NEURONS),
                .P_CORE_START_ADDR          (core_idx * P_CORE_NUM_NEURONS),
                .P_CORE_NUM_NEURONS         (P_CORE_NUM_NEURONS),
                .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
                .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
                .P_INPUT_WIDTH              (P_INPUT_WIDTH),
                .P_KERNEL_SIZE              (P_KERNEL_SIZE),
                .P_PADDING                  (P_PADDING),
                .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS   (P_NEURON_VALUE_FRAC_BITS),
                .P_SKIP_THRESHOLD_SHIFT      (P_SKIP_THRESHOLD_SHIFT),
                .P_USE_CURRENT_RAM_INPUT    (1)
            ) u_sparse_core (
                .clk                    (clk),
                .rst_n                  (rst_n),
                .i_enable_core          (i_enable_layer),
                .i_input_spike_vector   (i_input_spike_vector),
                .i_current_valid_bitmap (i_current_valid_bitmap[(core_idx + 1) * P_CORE_NUM_NEURONS - 1:
                                                                core_idx * P_CORE_NUM_NEURONS]),
                .i_all_currents_I       (i_all_currents_I),
                .i_current_rd_data      (core_current_rd_data_w[core_idx]),
                .i_current_rd_valid     (core_current_rd_valid_w[core_idx]),
                .i_current_rd_ready     (core_current_rd_ready_w[core_idx]),
                .o_current_rd_en        (core_current_rd_req_w[core_idx]),
                .o_current_rd_addr      (core_current_rd_addr_w[core_idx]),
                .o_core_spikes_out      (core_spikes_flat_w[(core_idx + 1) * P_CORE_NUM_NEURONS - 1:
                                                            core_idx * P_CORE_NUM_NEURONS]),
                .o_core_done            (core_done_w[core_idx]),
                .o_event_valid          (core_event_valid_w[core_idx]),
                .o_event_addr           (core_event_addr_w[core_idx]),
                .o_core_ready           (core_ready_w[core_idx]),
                .o_skip_count           (core_skip_count_w[core_idx]),
                .o_update_count         (core_update_count_w[core_idx]),
                .o_event_count          (core_event_count_w[core_idx])
            );

            aer_event_fifo #(
                .P_ADDR_WIDTH           (LP_ADDR_WIDTH),
                .P_FIFO_DEPTH           (P_CORE_EVENT_FIFO_DEPTH)
            ) u_core_event_fifo (
                .clk                    (clk),
                .rst_n                  (rst_n),
                .i_clear                (i_enable_layer),
                .i_event_valid          (core_event_valid_w[core_idx]),
                .i_event_addr           (core_event_addr_w[core_idx]),
                .i_event_ready          (arbiter_ready_w[core_idx]),
                .o_event_valid          (fifo_event_valid_w[core_idx]),
                .o_event_addr           (fifo_event_addr_w[core_idx]),
                .o_empty                (fifo_empty_w[core_idx]),
                .o_full                 (fifo_full_w[core_idx]),
                .o_count                (fifo_count_w[core_idx]),
                .o_overflow             (fifo_overflow_w[core_idx])
            );
        end
    endgenerate

    assign o_all_spikes_out = core_spikes_flat_w;

    /*
     * current RAM bank 读仲裁。
     * core0/core1 对应通道0的 784 个卷积电流；
     * core2/core3 对应通道1的 784 个卷积电流。
     * 每个 bank 一拍最多服务一个 core，请求被 grant 后 core 才继续推进。
     */
    current_bank_2way_arbiter #(
        .P_ADDR_WIDTH (LP_BANK_ADDR_WIDTH),
        .P_DATA_WIDTH (P_NEURON_VALUE_TOTAL_BITS)
    ) u_current_ch0_arbiter (
        .clk                (clk),
        .rst_n              (rst_n),
        .i_clear            (i_enable_layer),
        .i_core0_req        (core_current_rd_req_w[0]),
        .i_core0_addr       (core0_current_bank_addr_w),
        .o_core0_grant      (core_current_rd_ready_w[0]),
        .o_core0_data_valid (core_current_rd_valid_w[0]),
        .o_core0_data       (core_current_rd_data_w[0]),
        .i_core1_req        (core_current_rd_req_w[1]),
        .i_core1_addr       (core1_current_bank_addr_w),
        .o_core1_grant      (core_current_rd_ready_w[1]),
        .o_core1_data_valid (core_current_rd_valid_w[1]),
        .o_core1_data       (core_current_rd_data_w[1]),
        .o_ram_rd_en        (o_current_ch0_rd_en),
        .o_ram_rd_addr      (o_current_ch0_rd_addr),
        .i_ram_rd_valid     (i_current_ch0_rd_valid),
        .i_ram_rd_data      (i_current_ch0_rd_data)
    );

    current_bank_2way_arbiter #(
        .P_ADDR_WIDTH (LP_BANK_ADDR_WIDTH),
        .P_DATA_WIDTH (P_NEURON_VALUE_TOTAL_BITS)
    ) u_current_ch1_arbiter (
        .clk                (clk),
        .rst_n              (rst_n),
        .i_clear            (i_enable_layer),
        .i_core0_req        (core_current_rd_req_w[2]),
        .i_core0_addr       (core2_current_bank_addr_w),
        .o_core0_grant      (core_current_rd_ready_w[2]),
        .o_core0_data_valid (core_current_rd_valid_w[2]),
        .o_core0_data       (core_current_rd_data_w[2]),
        .i_core1_req        (core_current_rd_req_w[3]),
        .i_core1_addr       (core3_current_bank_addr_w),
        .o_core1_grant      (core_current_rd_ready_w[3]),
        .o_core1_data_valid (core_current_rd_valid_w[3]),
        .o_core1_data       (core_current_rd_data_w[3]),
        .o_ram_rd_en        (o_current_ch1_rd_en),
        .o_ram_rd_addr      (o_current_ch1_rd_addr),
        .i_ram_rd_valid     (i_current_ch1_rd_valid),
        .i_ram_rd_data      (i_current_ch1_rd_data)
    );

    aer_event_arbiter #(
        .P_NUM_PORTS     (P_NUM_CORES),
        .P_ADDR_WIDTH    (LP_ADDR_WIDTH),
        .P_COUNT_WIDTH   (P_CORE_FIFO_COUNT_WIDTH),
        .P_ARB_POLICY    (P_ARB_POLICY)
    ) u_event_arbiter (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_clear         (i_enable_layer),
        .i_event_valid   (fifo_event_valid_w),
        .i_event_addr    (fifo_event_addr_w),
        .i_event_count   (fifo_count_w),
        .o_event_ready   (arbiter_ready_w),
        .o_event_valid   (o_event_valid),
        .o_event_addr    (o_event_addr),
        .i_event_ready   (1'b1)
    );

    /*
     * addr_stream 化以后，各 core 的处理地址数量不同，
     * done 脉冲不再保证同一拍出现，所以需要逐 core 锁存完成状态。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            core_done_seen_reg <= {P_NUM_CORES{1'b0}};
        end else if (i_enable_layer || frame_complete_w) begin
            core_done_seen_reg <= {P_NUM_CORES{1'b0}};
        end else begin
            core_done_seen_reg <= core_done_seen_reg | core_done_w;
        end
    end

    /*
     * 记录每个 core 本地 FIFO 在当前时间步内达到过的最高水位。
     * 新时间步启动时清零，后续只在当前水位更高时更新。
     */
    generate
        for (core_idx = 0; core_idx < P_NUM_CORES; core_idx = core_idx + 1) begin : gen_fifo_max_counter
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    core_fifo_max_count_reg[core_idx] <= {P_CORE_FIFO_COUNT_WIDTH{1'b0}};
                end else if (i_enable_layer) begin
                    core_fifo_max_count_reg[core_idx] <= {P_CORE_FIFO_COUNT_WIDTH{1'b0}};
                end else if (fifo_count_w[core_idx] > core_fifo_max_count_reg[core_idx]) begin
                    core_fifo_max_count_reg[core_idx] <= fifo_count_w[core_idx];
                end
            end
        end
    endgenerate

    /*
     * 所有 core 完成且各自事件 FIFO 清空后，才认为本时间步 AER 帧结束。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_done_pending_reg <= 1'b0;
        end else begin
            if (i_enable_layer) begin
                frame_done_pending_reg <= 1'b0;
            end else if (frame_complete_w) begin
                frame_done_pending_reg <= 1'b0;
            end else if (all_cores_done_seen_w) begin
                frame_done_pending_reg <= 1'b1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_all_spikes_valid <= 1'b0;
            o_event_frame_done <= 1'b0;
        end else begin
            o_all_spikes_valid <= frame_complete_w;
            o_event_frame_done <= frame_complete_w;
        end
    end

endmodule
