module snn_top #(
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_INPUT_HEIGHT = 28,
    parameter P_INPUT_WIDTH = 28,
    parameter P_PIXEL_INTENSITY_BITS = 8,
    parameter P_IMAGE_BRAM_DATA_WIDTH = 64,
    parameter P_IMAGE_BRAM_DEPTH = 98,
    parameter P_PRNG_LFSR_WIDTH = 32,
    parameter P_T_MAX = 100,
    parameter P_WEIGHT_BIT_WIDTH = 16,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter P_NEURON_VALUE_FRAC_BITS = 12,
    parameter P_WEIGHT_BRAM_DATA_WIDTH = 64,
    parameter P_WEIGHT_BRAM_EFFECTIVE_DEPTH = 392,
    parameter P_NUM_OUTPUT_NEURONS = 10,
    parameter P_CONV_OUT_CHANNELS = 2,
    parameter P_CONV_KERNEL_SIZE = 3,
    parameter P_CONV_PADDING = 1,
    parameter [P_CONV_KERNEL_SIZE * P_CONV_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV0_WEIGHTS_PACKED =
        {P_CONV_KERNEL_SIZE * P_CONV_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}},
    parameter [P_CONV_KERNEL_SIZE * P_CONV_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV1_WEIGHTS_PACKED =
        {P_CONV_KERNEL_SIZE * P_CONV_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}},
    parameter P_USE_MASKED_FC = 0,
    parameter P_USE_SPARSE_CONV_LIF = 0,
    parameter P_USE_MULTICORE_CONV_LIF = 0,
    parameter P_AER_ARB_POLICY = 1,
    parameter P_CONV_LIF_CORE_MAPPING_MODE = 0
) (
    input wire clk,
    input wire rst_n,
    input wire i_start_new_image_processing,

    output wire [$clog2(P_NUM_OUTPUT_NEURONS)-1:0] o_predicted_label,
    output reg [31:0] o_perf_total_cycles,
    output reg [31:0] o_perf_total_aer_events,
    output reg [31:0] o_perf_last_aer_events,
    output reg [31:0] o_perf_last_aer_fc_cycles,
    output reg [31:0] o_perf_actual_time_steps,
    output reg [31:0] o_perf_last_conv_lif_skip_count,
    output reg [31:0] o_perf_last_conv_lif_update_count,
    output reg [31:0] o_perf_total_conv_lif_skip_count,
    output reg [31:0] o_perf_total_conv_lif_update_count,
    output reg [31:0] o_perf_last_core_event_max_count,
    output reg [31:0] o_perf_last_core_fifo_max_count,
    output reg o_perf_early_stop,
    output wire o_perf_valid
);

    localparam LP_NUM_CONV_FEATURES = P_CONV_OUT_CHANNELS * P_NUM_INPUT_PIXELS;
    localparam LP_CONV_AER_ADDR_WIDTH = $clog2(LP_NUM_CONV_FEATURES);
    localparam [31:0] LP_NUM_CONV_FEATURES_32 = LP_NUM_CONV_FEATURES;
    localparam LP_CORE_FIFO_COUNT_WIDTH = $clog2(512 + 1);

    wire [$clog2(P_T_MAX)-1:0] cu_current_time_step_t;
    wire cu_poisson_encoder_en;
    wire cu_linear_layer_start;
    wire cu_lif_layer_en;
    wire cu_output_logic_accum_en;
    wire cu_output_logic_decision_en;
    wire cu_snn_core_busy;
    wire cu_global_processing_done;

    wire [$clog2(P_IMAGE_BRAM_DEPTH)-1:0] img_loader_bram_addr;
    wire img_loader_bram_ena;
    wire signed [P_IMAGE_BRAM_DATA_WIDTH-1:0] img_bram_dout_raw;
    wire [P_NUM_INPUT_PIXELS-1:0][P_PIXEL_INTENSITY_BITS-1:0] loaded_image_buffer;
    wire img_loader_loading_busy;
    wire img_loader_load_done;

    wire [P_NUM_INPUT_PIXELS-1:0] encoded_spikes;
    wire encoded_spikes_valid;

    wire signed [LP_NUM_CONV_FEATURES-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] conv_currents;
    wire conv_currents_valid;
    wire [LP_NUM_CONV_FEATURES-1:0] conv_lif_output_spikes;
    wire conv_lif_spikes_valid;
    wire conv_lif_layer_ready;

    wire conv_lif_event_valid;
    wire [LP_CONV_AER_ADDR_WIDTH-1:0] conv_lif_event_addr;
    wire conv_lif_event_frame_done;
    wire [31:0] conv_lif_skip_count;
    wire [31:0] conv_lif_update_count;
    wire [3:0][31:0] conv_lif_core_event_count;
    wire [31:0] conv_lif_core_event_max_count_w;
    wire [3:0][LP_CORE_FIFO_COUNT_WIDTH-1:0] conv_lif_core_fifo_count;
    wire [3:0][LP_CORE_FIFO_COUNT_WIDTH-1:0] conv_lif_core_fifo_max_count;
    wire [LP_CORE_FIFO_COUNT_WIDTH-1:0] conv_lif_core_fifo_max_count_w;
    wire conv_lif_core_fifo_overflow;
    wire fifo_event_valid;
    wire [LP_CONV_AER_ADDR_WIDTH-1:0] fifo_event_addr;
    wire fifo_empty;
    wire fifo_full;
    wire [$clog2(2048 + 1)-1:0] fifo_count;
    wire fifo_overflow;
    wire aer_event_valid;
    wire [LP_CONV_AER_ADDR_WIDTH-1:0] aer_event_addr;
    wire aer_event_frame_done;
    wire aer_event_ready;
    wire perf_aer_event_accept_w;

    wire signed [P_NUM_OUTPUT_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] neuron_currents;
    wire neuron_currents_valid;

    wire [P_NUM_OUTPUT_NEURONS-1:0] lif_output_spikes;
    wire lif_spikes_valid;
    wire [$clog2(P_T_MAX + 1)-1:0] output_max_spike_count;
    wire [$clog2(P_T_MAX + 1)-1:0] output_second_max_spike_count;
    wire output_prediction_valid;
    wire output_early_stop;

    reg conv_layer_start_pending_r;
    reg conv_layer_actual_start_r;
    reg conv_lif_enable_pending_r;
    reg conv_lif_actual_enable_r;
    reg lif_layer_enable_pending_r;
    reg lif_layer_actual_enable_r;
    reg output_logic_accum_pending_r;
    reg output_logic_actual_accum_en_r;
    reg conv_lif_event_frame_done_pending_r;
    reg perf_processing_active_r;
    reg perf_aer_fc_active_r;
    reg [31:0] perf_current_aer_events_r;
    reg [31:0] perf_current_aer_fc_cycles_r;

    photo_input u_image_bram (
        .clka   (clk),
        .ena    (img_loader_bram_ena),
        .addra  (img_loader_bram_addr),
        .douta  (img_bram_dout_raw)
    );

    image_loader #(
        .P_NUM_INPUT_PIXELS       (P_NUM_INPUT_PIXELS),
        .P_PIXEL_INTENSITY_BITS   (P_PIXEL_INTENSITY_BITS),
        .P_IMAGE_BRAM_DATA_WIDTH  (P_IMAGE_BRAM_DATA_WIDTH),
        .P_IMAGE_BRAM_DEPTH       (P_IMAGE_BRAM_DEPTH)
    ) u_image_loader (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_load_image_start     (i_start_new_image_processing),
        .i_bram_dout_raw        (img_bram_dout_raw),
        .o_bram_addr            (img_loader_bram_addr),
        .o_bram_ena             (img_loader_bram_ena),
        .o_image_buffer_out     (loaded_image_buffer),
        .o_loading_busy         (img_loader_loading_busy),
        .o_load_done            (img_loader_load_done)
    );

    control_unit #(
        .P_T_MAX (P_T_MAX)
    ) u_control_unit (
        .clk                         (clk),
        .rst_n                       (rst_n),
        .i_global_start_signal       (img_loader_load_done),
        .i_datapath_step_done        (lif_spikes_valid),
        .i_early_stop                (output_early_stop),
        .o_current_time_step_t       (cu_current_time_step_t),
        .o_poisson_encoder_en        (cu_poisson_encoder_en),
        .o_linear_layer_start        (cu_linear_layer_start),
        .o_lif_layer_en              (cu_lif_layer_en),
        .o_output_logic_accum_en     (cu_output_logic_accum_en),
        .o_output_logic_decision_en  (cu_output_logic_decision_en),
        .o_snn_busy                  (cu_snn_core_busy),
        .o_global_processing_done    (cu_global_processing_done)
    );

    poisson_encoder #(
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_PIXEL_INTENSITY_BITS     (P_PIXEL_INTENSITY_BITS),
        .P_PRNG_LFSR_WIDTH          (P_PRNG_LFSR_WIDTH),
        .P_T_MAX                    (P_T_MAX)
    ) u_poisson_encoder (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_enable_enc           (cu_poisson_encoder_en),
        .i_time_step_t          (cu_current_time_step_t),
        .i_image_pixel_data     (loaded_image_buffer),
        .o_spike_vector_reg     (encoded_spikes),
        .o_spikes_valid_reg     (encoded_spikes_valid)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            conv_layer_start_pending_r <= 1'b0;
            conv_layer_actual_start_r <= 1'b0;
        end else begin
            conv_layer_actual_start_r <= 1'b0;
            if (cu_linear_layer_start) begin
                conv_layer_start_pending_r <= 1'b1;
            end
            // 只有当当前时间步的泊松脉冲真正产生出来以后，
            // 才正式启动卷积层。
            if (conv_layer_start_pending_r && encoded_spikes_valid) begin
                conv_layer_actual_start_r <= 1'b1;
                conv_layer_start_pending_r <= 1'b0;
            end
        end
    end

    conv_layer_parallel #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_NUM_OUTPUT_CHANNELS      (P_CONV_OUT_CHANNELS),
        .P_KERNEL_SIZE              (P_CONV_KERNEL_SIZE),
        .P_PADDING                  (P_CONV_PADDING),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_CONV0_WEIGHTS_PACKED     (P_CONV0_WEIGHTS_PACKED),
        .P_CONV1_WEIGHTS_PACKED     (P_CONV1_WEIGHTS_PACKED)
    ) u_conv_layer_parallel (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_calc_start           (conv_layer_actual_start_r),
        .i_input_spike_vector   (encoded_spikes),
        .o_all_currents_I       (conv_currents),
        .o_all_currents_valid   (conv_currents_valid)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            conv_lif_enable_pending_r <= 1'b0;
            conv_lif_actual_enable_r <= 1'b0;
        end else begin
            conv_lif_actual_enable_r <= 1'b0;

            if (conv_currents_valid) begin
                conv_lif_enable_pending_r <= 1'b1;
            end

            /*
             * 并行卷积层可能早于 conv_lif_layer 的清零阶段完成。
             * 因此先记住卷积结果已有效，等卷积后 LIF 真正 ready 后再发启动脉冲。
             */
            if (conv_lif_enable_pending_r && conv_lif_layer_ready) begin
                conv_lif_actual_enable_r <= 1'b1;
                conv_lif_enable_pending_r <= 1'b0;
            end
        end
    end

    /*
     * 卷积后 LIF 层可切换版本。
     * P_USE_SPARSE_CONV_LIF=0 使用原始逐地址更新版本；
     * P_USE_SPARSE_CONV_LIF=1 使用感受野/膜电位状态联合感知的稀疏跳过版本。
     */
    generate
        if (P_USE_MULTICORE_CONV_LIF) begin : gen_multicore_conv_lif
            conv_lif_sparse_multicore #(
                .P_NUM_NEURONS               (LP_NUM_CONV_FEATURES),
                .P_NUM_CORES                 (4),
                .P_CORE_NUM_NEURONS          (LP_NUM_CONV_FEATURES / 4),
                .P_NUM_INPUT_PIXELS          (P_NUM_INPUT_PIXELS),
                .P_INPUT_HEIGHT              (P_INPUT_HEIGHT),
                .P_INPUT_WIDTH               (P_INPUT_WIDTH),
                .P_KERNEL_SIZE               (P_CONV_KERNEL_SIZE),
                .P_PADDING                   (P_CONV_PADDING),
                .P_CORE_MAPPING_MODE         (P_CONV_LIF_CORE_MAPPING_MODE),
                .P_NEURON_VALUE_TOTAL_BITS   (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS    (P_NEURON_VALUE_FRAC_BITS),
                .P_SKIP_THRESHOLD_SHIFT      (5),
                .P_CORE_EVENT_FIFO_DEPTH     (512),
                .P_ARB_POLICY                (P_AER_ARB_POLICY)
            ) u_conv_lif_sparse_multicore (
                .clk                    (clk),
                .rst_n                  (rst_n),
                .i_enable_layer         (conv_lif_actual_enable_r),
                .i_input_spike_vector   (encoded_spikes),
                .i_all_currents_I       (conv_currents),
                .o_all_spikes_out       (conv_lif_output_spikes),
                .o_all_spikes_valid     (conv_lif_spikes_valid),
                .o_event_valid          (conv_lif_event_valid),
                .o_event_addr           (conv_lif_event_addr),
                .o_event_frame_done     (conv_lif_event_frame_done),
                .o_layer_ready          (conv_lif_layer_ready),
                .o_skip_count           (conv_lif_skip_count),
                .o_update_count         (conv_lif_update_count),
                .o_core_event_count     (conv_lif_core_event_count),
                .o_core_fifo_count      (conv_lif_core_fifo_count),
                .o_core_fifo_max_count  (conv_lif_core_fifo_max_count),
                .o_core_fifo_overflow   (conv_lif_core_fifo_overflow)
            );
        end else if (P_USE_SPARSE_CONV_LIF) begin : gen_sparse_conv_lif
            conv_lif_layer_sparse #(
                .P_NUM_NEURONS               (LP_NUM_CONV_FEATURES),
                .P_NUM_INPUT_PIXELS          (P_NUM_INPUT_PIXELS),
                .P_INPUT_HEIGHT              (P_INPUT_HEIGHT),
                .P_INPUT_WIDTH               (P_INPUT_WIDTH),
                .P_KERNEL_SIZE               (P_CONV_KERNEL_SIZE),
                .P_PADDING                   (P_CONV_PADDING),
                .P_NEURON_VALUE_TOTAL_BITS   (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS    (P_NEURON_VALUE_FRAC_BITS),
                .P_SKIP_THRESHOLD_SHIFT      (5)
            ) u_conv_lif_layer_sparse (
                .clk                    (clk),
                .rst_n                  (rst_n),
                .i_enable_layer         (conv_lif_actual_enable_r),
                .i_input_spike_vector   (encoded_spikes),
                .i_all_currents_I       (conv_currents),
                .o_all_spikes_out       (conv_lif_output_spikes),
                .o_all_spikes_valid     (conv_lif_spikes_valid),
                .o_event_valid          (conv_lif_event_valid),
                .o_event_addr           (conv_lif_event_addr),
                .o_event_frame_done     (conv_lif_event_frame_done),
                .o_layer_ready          (conv_lif_layer_ready),
                .o_skip_count           (conv_lif_skip_count),
                .o_update_count         (conv_lif_update_count)
            );

            assign conv_lif_core_fifo_overflow = 1'b0;
            assign conv_lif_core_event_count = {4 * 32{1'b0}};
            assign conv_lif_core_fifo_count = {4 * LP_CORE_FIFO_COUNT_WIDTH{1'b0}};
            assign conv_lif_core_fifo_max_count = {4 * LP_CORE_FIFO_COUNT_WIDTH{1'b0}};
        end else begin : gen_dense_conv_lif
            conv_lif_layer #(
                .P_NUM_NEURONS               (LP_NUM_CONV_FEATURES),
                .P_NEURON_VALUE_TOTAL_BITS   (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS    (P_NEURON_VALUE_FRAC_BITS)
            ) u_conv_lif_layer (
                .clk                    (clk),
                .rst_n                  (rst_n),
                .i_enable_layer         (conv_lif_actual_enable_r),
                .i_all_currents_I       (conv_currents),
                .o_all_spikes_out       (conv_lif_output_spikes),
                .o_all_spikes_valid     (conv_lif_spikes_valid),
                .o_event_valid          (conv_lif_event_valid),
                .o_event_addr           (conv_lif_event_addr),
                .o_event_frame_done     (conv_lif_event_frame_done),
                .o_layer_ready          (conv_lif_layer_ready)
            );

            assign conv_lif_skip_count = 32'd0;
            assign conv_lif_update_count = conv_lif_spikes_valid ? LP_NUM_CONV_FEATURES_32 : 32'd0;
            assign conv_lif_core_fifo_overflow = 1'b0;
            assign conv_lif_core_event_count = {4 * 32{1'b0}};
            assign conv_lif_core_fifo_count = {4 * LP_CORE_FIFO_COUNT_WIDTH{1'b0}};
            assign conv_lif_core_fifo_max_count = {4 * LP_CORE_FIFO_COUNT_WIDTH{1'b0}};
        end
    endgenerate

    aer_event_fifo #(
        .P_ADDR_WIDTH           (LP_CONV_AER_ADDR_WIDTH),
        .P_FIFO_DEPTH           (2048)
    ) u_aer_event_fifo (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_clear                (conv_lif_actual_enable_r),
        .i_event_valid          (conv_lif_event_valid),
        .i_event_addr           (conv_lif_event_addr),
        .i_event_ready          (aer_event_ready),
        .o_event_valid          (fifo_event_valid),
        .o_event_addr           (fifo_event_addr),
        .o_empty                (fifo_empty),
        .o_full                 (fifo_full),
        .o_count                (fifo_count),
        .o_overflow             (fifo_overflow)
    );

    /*
     * AER 事件现在由 conv_lif_layer 边处理边写入 FIFO。
     * 等卷积 LIF 处理完并且 FIFO 已排空后，再通知 AER 全连接层本时间步结束。
     */
    assign aer_event_valid = fifo_event_valid;
    assign aer_event_addr = fifo_event_addr;
    assign aer_event_frame_done = conv_lif_event_frame_done_pending_r && fifo_empty;
    assign conv_lif_core_event_max_count_w =
        (conv_lif_core_event_count[0] >= conv_lif_core_event_count[1] &&
         conv_lif_core_event_count[0] >= conv_lif_core_event_count[2] &&
         conv_lif_core_event_count[0] >= conv_lif_core_event_count[3]) ? conv_lif_core_event_count[0] :
        (conv_lif_core_event_count[1] >= conv_lif_core_event_count[2] &&
         conv_lif_core_event_count[1] >= conv_lif_core_event_count[3]) ? conv_lif_core_event_count[1] :
        (conv_lif_core_event_count[2] >= conv_lif_core_event_count[3]) ? conv_lif_core_event_count[2] :
                                                                         conv_lif_core_event_count[3];
    assign conv_lif_core_fifo_max_count_w =
        (conv_lif_core_fifo_max_count[0] >= conv_lif_core_fifo_max_count[1] &&
         conv_lif_core_fifo_max_count[0] >= conv_lif_core_fifo_max_count[2] &&
         conv_lif_core_fifo_max_count[0] >= conv_lif_core_fifo_max_count[3]) ? conv_lif_core_fifo_max_count[0] :
        (conv_lif_core_fifo_max_count[1] >= conv_lif_core_fifo_max_count[2] &&
         conv_lif_core_fifo_max_count[1] >= conv_lif_core_fifo_max_count[3]) ? conv_lif_core_fifo_max_count[1] :
        (conv_lif_core_fifo_max_count[2] >= conv_lif_core_fifo_max_count[3]) ? conv_lif_core_fifo_max_count[2] :
                                                                                conv_lif_core_fifo_max_count[3];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            conv_lif_event_frame_done_pending_r <= 1'b0;
        end else begin
            if (conv_lif_actual_enable_r) begin
                conv_lif_event_frame_done_pending_r <= 1'b0;
            end else if (conv_lif_event_frame_done) begin
                conv_lif_event_frame_done_pending_r <= 1'b1;
            end else if (aer_event_frame_done) begin
                conv_lif_event_frame_done_pending_r <= 1'b0;
            end
        end
    end

    /*
     * AER 全连接层可切换版本。
     * P_USE_MASKED_FC=0 使用原始 dense AER 全连接层；
     * P_USE_MASKED_FC=1 使用带静态剪枝 mask 的 masked AER 全连接层。
     */
    generate
        if (P_USE_MASKED_FC) begin : gen_masked_aer_fc
            masked_aer_linear_layer #(
                .P_NUM_INPUT_EVENTS         (LP_NUM_CONV_FEATURES),
                .P_EVENT_ADDR_WIDTH         (LP_CONV_AER_ADDR_WIDTH),
                .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
                .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS   (P_NEURON_VALUE_FRAC_BITS),
                .P_BRAM_DATA_WIDTH          (P_WEIGHT_BRAM_DATA_WIDTH),
                .P_BRAM_ADDR_WIDTH          ($clog2(P_WEIGHT_BRAM_EFFECTIVE_DEPTH)),
                .P_BRAM_READ_LATENCY        (2),
                .P_NUM_OUTPUT_NEURONS       (P_NUM_OUTPUT_NEURONS),
                .P_MASK_WIDTH               (16)
            ) u_masked_aer_linear_layer (
                .clk                    (clk),
                .rst_n                  (rst_n),
                .i_start                (conv_lif_actual_enable_r),
                .i_event_valid          (aer_event_valid),
                .i_event_addr           (aer_event_addr),
                .i_event_frame_done     (aer_event_frame_done),
                .o_event_ready          (aer_event_ready),
                .o_all_currents_I       (neuron_currents),
                .o_all_currents_valid   (neuron_currents_valid)
            );
        end else begin : gen_dense_aer_fc
            aer_linear_layer #(
                .P_NUM_INPUT_EVENTS         (LP_NUM_CONV_FEATURES),
                .P_EVENT_ADDR_WIDTH         (LP_CONV_AER_ADDR_WIDTH),
                .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
                .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS   (P_NEURON_VALUE_FRAC_BITS),
                .P_BRAM_DATA_WIDTH          (P_WEIGHT_BRAM_DATA_WIDTH),
                .P_BRAM_ADDR_WIDTH          ($clog2(P_WEIGHT_BRAM_EFFECTIVE_DEPTH)),
                .P_BRAM_READ_LATENCY        (2),
                .P_NUM_OUTPUT_NEURONS       (P_NUM_OUTPUT_NEURONS)
            ) u_aer_linear_layer (
                .clk                    (clk),
                .rst_n                  (rst_n),
                .i_start                (conv_lif_actual_enable_r),
                .i_event_valid          (aer_event_valid),
                .i_event_addr           (aer_event_addr),
                .i_event_frame_done     (aer_event_frame_done),
                .o_event_ready          (aer_event_ready),
                .o_all_currents_I       (neuron_currents),
                .o_all_currents_valid   (neuron_currents_valid)
            );
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lif_layer_enable_pending_r <= 1'b0;
            lif_layer_actual_enable_r <= 1'b0;
        end else begin
            lif_layer_actual_enable_r <= 1'b0;
            if (cu_lif_layer_en) begin
                lif_layer_enable_pending_r <= 1'b1;
            end
            if (lif_layer_enable_pending_r && neuron_currents_valid) begin
                lif_layer_actual_enable_r <= 1'b1;
                lif_layer_enable_pending_r <= 1'b0;
            end
        end
    end

    lif_neuron_layer #(
        .P_NUM_OUTPUT_NEURONS       (P_NUM_OUTPUT_NEURONS),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS   (P_NEURON_VALUE_FRAC_BITS)
    ) u_lif_neuron_layer (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_enable_layer         (lif_layer_actual_enable_r),
        .i_all_currents_I       (neuron_currents),
        .o_all_spikes_out       (lif_output_spikes),
        .o_all_spikes_valid     (lif_spikes_valid)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_logic_accum_pending_r <= 1'b0;
            output_logic_actual_accum_en_r <= 1'b0;
        end else begin
            output_logic_actual_accum_en_r <= 1'b0;
            if (cu_output_logic_accum_en) begin
                output_logic_accum_pending_r <= 1'b1;
            end
            if (output_logic_accum_pending_r && lif_spikes_valid) begin
                output_logic_actual_accum_en_r <= 1'b1;
                output_logic_accum_pending_r <= 1'b0;
            end
        end
    end

    output_logic #(
        .P_NUM_OUTPUT_NEURONS (P_NUM_OUTPUT_NEURONS),
        .P_T_MAX              (P_T_MAX)
    ) u_output_logic (
        .clk                         (clk),
        .rst_n                       (rst_n),
        .i_accum_en                  (output_logic_actual_accum_en_r),
        .i_decision_en               (cu_output_logic_decision_en),
        .i_current_time_step         (cu_current_time_step_t),
        .i_lif_spike_vector          (lif_output_spikes),
        .o_predicted_label           (o_predicted_label),
        .o_max_spike_count           (output_max_spike_count),
        .o_second_max_spike_count    (output_second_max_spike_count),
        .o_prediction_valid          (output_prediction_valid),
        .o_early_stop                (output_early_stop)
    );

    assign o_perf_valid = cu_global_processing_done;
    assign perf_aer_event_accept_w = aer_event_valid && aer_event_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            perf_processing_active_r <= 1'b0;
            perf_aer_fc_active_r <= 1'b0;
            perf_current_aer_events_r <= 32'd0;
            perf_current_aer_fc_cycles_r <= 32'd0;
            o_perf_total_cycles <= 32'd0;
            o_perf_total_aer_events <= 32'd0;
            o_perf_last_aer_events <= 32'd0;
            o_perf_last_aer_fc_cycles <= 32'd0;
            o_perf_actual_time_steps <= 32'd0;
            o_perf_last_conv_lif_skip_count <= 32'd0;
            o_perf_last_conv_lif_update_count <= 32'd0;
            o_perf_total_conv_lif_skip_count <= 32'd0;
            o_perf_total_conv_lif_update_count <= 32'd0;
            o_perf_last_core_event_max_count <= 32'd0;
            o_perf_last_core_fifo_max_count <= 32'd0;
            o_perf_early_stop <= 1'b0;
        end else begin
            /*
             * 总处理周期：从顶层收到新图片启动脉冲开始计数，
             * 到控制单元给出全局完成信号为止。
             */
            if (i_start_new_image_processing) begin
                perf_processing_active_r <= 1'b1;
                o_perf_total_cycles <= 32'd0;
                o_perf_total_aer_events <= 32'd0;
                o_perf_actual_time_steps <= 32'd0;
                o_perf_last_conv_lif_skip_count <= 32'd0;
                o_perf_last_conv_lif_update_count <= 32'd0;
                o_perf_total_conv_lif_skip_count <= 32'd0;
                o_perf_total_conv_lif_update_count <= 32'd0;
                o_perf_last_core_event_max_count <= 32'd0;
                o_perf_last_core_fifo_max_count <= 32'd0;
                o_perf_early_stop <= 1'b0;
            end else if (cu_global_processing_done) begin
                perf_processing_active_r <= 1'b0;
            end

            if (output_logic_actual_accum_en_r) begin
                o_perf_actual_time_steps <= {{(32-$clog2(P_T_MAX)){1'b0}}, cu_current_time_step_t} + 32'd1;
                if (output_early_stop) begin
                    o_perf_early_stop <= 1'b1;
                end
            end

            if (perf_processing_active_r && !cu_global_processing_done) begin
                o_perf_total_cycles <= o_perf_total_cycles + 32'd1;
            end

            /*
             * AER 全连接层耗时：从 conv_lif_spikes_valid 启动 AER/FC 开始，
             * 到 neuron_currents_valid 输出本时间步全连接电流为止。
             */
            if (conv_lif_spikes_valid && neuron_currents_valid) begin
                perf_aer_fc_active_r <= 1'b0;
                o_perf_last_aer_fc_cycles <= 32'd1;
            end else if (conv_lif_spikes_valid) begin
                perf_aer_fc_active_r <= 1'b1;
                perf_current_aer_fc_cycles_r <= 32'd1;
            end else if (perf_aer_fc_active_r && neuron_currents_valid) begin
                perf_aer_fc_active_r <= 1'b0;
                o_perf_last_aer_fc_cycles <= perf_current_aer_fc_cycles_r;
            end else if (perf_aer_fc_active_r) begin
                perf_current_aer_fc_cycles_r <= perf_current_aer_fc_cycles_r + 32'd1;
            end

            /*
             * AER 事件数量：valid 和 ready 同时为 1 才说明一个事件真正被接收。
             * last 记录最近一个时间步，total 记录整张图片推理过程的总事件数。
             */
            if (conv_lif_actual_enable_r) begin
                perf_current_aer_events_r <= perf_aer_event_accept_w ? 32'd1 : 32'd0;
            end else if (perf_aer_event_accept_w) begin
                perf_current_aer_events_r <= perf_current_aer_events_r + 32'd1;
            end

            if (perf_aer_event_accept_w) begin
                o_perf_total_aer_events <= o_perf_total_aer_events + 32'd1;
            end

            if (neuron_currents_valid) begin
                o_perf_last_aer_events <= perf_current_aer_events_r + (perf_aer_event_accept_w ? 32'd1 : 32'd0);
            end

            if (conv_lif_spikes_valid) begin
                o_perf_last_conv_lif_skip_count <= conv_lif_skip_count;
                o_perf_last_conv_lif_update_count <= conv_lif_update_count;
                o_perf_total_conv_lif_skip_count <= o_perf_total_conv_lif_skip_count + conv_lif_skip_count;
                o_perf_total_conv_lif_update_count <= o_perf_total_conv_lif_update_count + conv_lif_update_count;
                o_perf_last_core_event_max_count <= conv_lif_core_event_max_count_w;
                o_perf_last_core_fifo_max_count <= {{(32-LP_CORE_FIFO_COUNT_WIDTH){1'b0}}, conv_lif_core_fifo_max_count_w};
            end
        end
    end

endmodule
