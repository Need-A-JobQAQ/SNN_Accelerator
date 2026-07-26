module conv_lif_sparse_multicore #(
    parameter P_NUM_NEURONS = 1568,
    parameter P_NUM_CORES = 4,
    parameter P_CORE_NUM_NEURONS = P_NUM_NEURONS / P_NUM_CORES,
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_INPUT_HEIGHT = 28,
    parameter P_INPUT_WIDTH = 28,
    parameter P_KERNEL_SIZE = 3,
    parameter P_PADDING = 1,
    parameter P_CORE_MAPPING_MODE = 0,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter P_NEURON_VALUE_FRAC_BITS = 12,
    parameter P_SKIP_THRESHOLD_SHIFT = 5,
    parameter P_CORE_EVENT_FIFO_DEPTH = 512,
    parameter P_CORE_FIFO_COUNT_WIDTH = $clog2(P_CORE_EVENT_FIFO_DEPTH + 1),
    parameter P_ARB_POLICY = 1
) (
    input wire clk,
    input wire rst_n,
    input wire i_enable_layer,
    input wire [P_NUM_INPUT_PIXELS-1:0] i_input_spike_vector,
    input wire signed [P_NUM_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] i_all_currents_I,

    output wire [P_NUM_NEURONS-1:0] o_all_spikes_out,
    output reg o_all_spikes_valid,
    output wire o_event_valid,
    output wire [$clog2(P_NUM_NEURONS)-1:0] o_event_addr,
    output reg o_event_frame_done,
    output wire o_layer_ready,
    output wire [31:0] o_skip_count,
    output wire [31:0] o_update_count,
    output wire [P_NUM_CORES-1:0][31:0] o_core_event_count,
    output wire [P_NUM_CORES-1:0][P_CORE_FIFO_COUNT_WIDTH-1:0] o_core_fifo_count,
    output wire [P_NUM_CORES-1:0][P_CORE_FIFO_COUNT_WIDTH-1:0] o_core_fifo_max_count,
    output wire o_core_fifo_overflow
);

    /*
     * 多核稀疏卷积 LIF 层。
     * 4 个 core 分别处理一段连续神经元地址，每个 core 后接本地事件 FIFO，
     * 再由轮询仲裁器汇聚成一路 AER 事件流。
     */
    localparam LP_ADDR_WIDTH = $clog2(P_NUM_NEURONS);

    wire [P_NUM_CORES-1:0] core_ready_w;
    wire [P_NUM_CORES-1:0] core_done_w;
    wire [P_NUM_CORES-1:0] core_event_valid_w;
    wire [P_NUM_CORES-1:0][LP_ADDR_WIDTH-1:0] core_event_addr_w;
    wire [P_NUM_CORES * P_CORE_NUM_NEURONS - 1:0] core_spikes_flat_w;
    wire [P_NUM_CORES-1:0][31:0] core_skip_count_w;
    wire [P_NUM_CORES-1:0][31:0] core_update_count_w;

    wire [P_NUM_CORES-1:0] fifo_event_valid_w;
    wire [P_NUM_CORES-1:0][LP_ADDR_WIDTH-1:0] fifo_event_addr_w;
    wire [P_NUM_CORES-1:0] fifo_empty_w;
    wire [P_NUM_CORES-1:0] fifo_full_w;
    wire [P_NUM_CORES-1:0][P_CORE_FIFO_COUNT_WIDTH-1:0] fifo_count_w;
    wire [P_NUM_CORES-1:0] fifo_overflow_w;
    wire [P_NUM_CORES-1:0] fifo_write_accept_w;
    wire [P_NUM_CORES-1:0] arbiter_ready_w;

    wire all_cores_ready_w;
    wire all_cores_done_w;
    wire all_fifos_empty_w;
    wire frame_complete_w;

    reg frame_done_pending_reg;
    reg [P_NUM_CORES-1:0][31:0] core_event_count_reg;
    reg [P_NUM_CORES-1:0][P_CORE_FIFO_COUNT_WIDTH-1:0] core_fifo_max_count_reg;

    genvar core_idx;
    genvar spike_idx;
    genvar pack_core_idx;
    genvar pack_local_idx;

    assign all_cores_ready_w = &core_ready_w;
    assign all_cores_done_w = &core_done_w;
    assign all_fifos_empty_w = &fifo_empty_w;
    assign frame_complete_w = frame_done_pending_reg && all_fifos_empty_w;
    assign o_layer_ready = all_cores_ready_w;
    assign o_core_event_count = core_event_count_reg;
    assign o_core_fifo_count = fifo_count_w;
    assign o_core_fifo_max_count = core_fifo_max_count_reg;
    assign o_core_fifo_overflow = |fifo_overflow_w;

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
                .P_CORE_ID                  (core_idx),
                .P_CORE_MAPPING_MODE        (P_CORE_MAPPING_MODE),
                .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS   (P_NEURON_VALUE_FRAC_BITS),
                .P_SKIP_THRESHOLD_SHIFT     (P_SKIP_THRESHOLD_SHIFT)
            ) u_sparse_core (
                .clk                    (clk),
                .rst_n                  (rst_n),
                .i_enable_core          (i_enable_layer),
                .i_input_spike_vector   (i_input_spike_vector),
                .i_all_currents_I       (i_all_currents_I),
                .o_core_spikes_out      (core_spikes_flat_w[(core_idx + 1) * P_CORE_NUM_NEURONS - 1:
                                                            core_idx * P_CORE_NUM_NEURONS]),
                .o_core_done            (core_done_w[core_idx]),
                .o_event_valid          (core_event_valid_w[core_idx]),
                .o_event_addr           (core_event_addr_w[core_idx]),
                .o_core_ready           (core_ready_w[core_idx]),
                .o_skip_count           (core_skip_count_w[core_idx]),
                .o_update_count         (core_update_count_w[core_idx])
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

            assign fifo_write_accept_w[core_idx] = core_event_valid_w[core_idx] && !fifo_full_w[core_idx];
        end
    endgenerate

    /*
     * core 内部输出是本地地址顺序。
     * 这里按映射关系重新放回全局 1568bit spike vector，保证后级看到的位序不变。
     */
    generate
        for (pack_core_idx = 0; pack_core_idx < P_NUM_CORES; pack_core_idx = pack_core_idx + 1) begin : gen_spike_pack_core
            for (pack_local_idx = 0; pack_local_idx < P_CORE_NUM_NEURONS; pack_local_idx = pack_local_idx + 1) begin : gen_spike_pack_bit
                localparam integer LP_PACK_GLOBAL_ADDR = map_local_to_global_addr(pack_core_idx, pack_local_idx);
                assign o_all_spikes_out[LP_PACK_GLOBAL_ADDR] =
                    core_spikes_flat_w[(pack_core_idx * P_CORE_NUM_NEURONS) + pack_local_idx];
            end
        end
    endgenerate

    /*
     * 多核封装层使用同样的映射函数恢复全局 spike 位序。
     */
    function integer map_local_to_global_addr;
        input integer core_id;
        input integer local_addr;
        integer positions_per_core;
        integer tile_size;
        integer tile_pixels;
        integer tile_cols;
        integer local_channel;
        integer local_spatial_idx;
        integer local_tile_idx;
        integer in_tile_idx;
        integer local_row;
        integer local_col;
        integer tile_row;
        integer tile_col;
        integer in_tile_row;
        integer in_tile_col;
        integer global_row;
        integer global_col;
        integer global_spatial_idx;
        integer global_local_addr;
        begin
            if (P_CORE_MAPPING_MODE == 1) begin
                positions_per_core = (P_INPUT_HEIGHT / 2) * (P_INPUT_WIDTH / 2);
                local_channel = local_addr / positions_per_core;
                local_spatial_idx = local_addr % positions_per_core;
                local_row = local_spatial_idx / (P_INPUT_WIDTH / 2);
                local_col = local_spatial_idx % (P_INPUT_WIDTH / 2);
                global_row = (local_row * 2) + (core_id / 2);
                global_col = (local_col * 2) + (core_id % 2);
                global_spatial_idx = (global_row * P_INPUT_WIDTH) + global_col;
                global_local_addr = P_NUM_INPUT_PIXELS - 1 - global_spatial_idx;
                map_local_to_global_addr = (local_channel * P_NUM_INPUT_PIXELS) + global_local_addr;
            end else if (P_CORE_MAPPING_MODE == 2) begin
                /*
                 * 7x7 tile空间块交错映射。
                 * 这里必须和conv_lif_sparse_core里的local_to_global_addr保持一致。
                 */
                tile_size = 7;
                tile_pixels = tile_size * tile_size;
                tile_cols = P_INPUT_WIDTH / tile_size;
                positions_per_core = P_CORE_NUM_NEURONS / (P_NUM_NEURONS / P_NUM_INPUT_PIXELS);
                local_channel = local_addr / positions_per_core;
                local_spatial_idx = local_addr % positions_per_core;
                local_tile_idx = local_spatial_idx / tile_pixels;
                in_tile_idx = local_spatial_idx % tile_pixels;
                tile_row = local_tile_idx;
                tile_col = (core_id + tile_cols - tile_row) % tile_cols;
                in_tile_row = in_tile_idx / tile_size;
                in_tile_col = in_tile_idx % tile_size;
                global_row = (tile_row * tile_size) + in_tile_row;
                global_col = (tile_col * tile_size) + in_tile_col;
                global_spatial_idx = (global_row * P_INPUT_WIDTH) + global_col;
                global_local_addr = P_NUM_INPUT_PIXELS - 1 - global_spatial_idx;
                map_local_to_global_addr = (local_channel * P_NUM_INPUT_PIXELS) + global_local_addr;
            end else begin
                map_local_to_global_addr = (core_id * P_CORE_NUM_NEURONS) + local_addr;
            end
        end
    endfunction

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
     * 统计每个 core 在当前时间步内实际写入本地 FIFO 的 AER 事件数。
     * 该指标用于判断负载是否真的被映射策略均衡到各个 core。
     */
    generate
        for (core_idx = 0; core_idx < P_NUM_CORES; core_idx = core_idx + 1) begin : gen_core_event_counter
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    core_event_count_reg[core_idx] <= 32'd0;
                end else if (i_enable_layer) begin
                    core_event_count_reg[core_idx] <= 32'd0;
                end else if (fifo_write_accept_w[core_idx]) begin
                    core_event_count_reg[core_idx] <= core_event_count_reg[core_idx] + 32'd1;
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
            end else if (all_cores_done_w) begin
                frame_done_pending_reg <= 1'b1;
            end else if (frame_complete_w) begin
                frame_done_pending_reg <= 1'b0;
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
