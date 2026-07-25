`timescale 1ns/1ps

module conv_lif_aer_tb();

    localparam CLK_PERIOD = 10 ; // 时钟周期（100MHZ)

    localparam P_INPUT_HEIGHT = 28;
    localparam P_INPUT_WIDTH = 28;
    localparam P_NUM_INPUT_PIXELS = 784;
    localparam P_NUM_OUTPUT_CHANNELS = 2;
    localparam P_KERNEL_SIZE = 3;
    localparam P_PADDING = 1;
    localparam P_WEIGHT_BIT_WIDTH = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_NUM_NEURONS = 1568;            //新增卷积神经元层参数
    localparam P_CONV_OUT_CHANNELS = 2;             //新增卷积输出通道数
    localparam P_NEURON_VALUE_FRAC_BITS = 12;   //
    localparam [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV0_WEIGHTS_PACKED = {16'hFA52,16'h1085,16'hFF5B,
                                                                                                    16'h0602,16'h0A60,16'hF5AA,
                                                                                                    16'h1314,16'h05C8,16'hEA50};

    localparam [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV1_WEIGHTS_PACKED = {16'hE764,16'h41B2,16'h272A,
                                                                                                    16'hF6DD,16'h61AB,16'hF8CB,
                                                                                                    16'h36E8,16'h2EFE,16'hE59C};
    
    localparam LP_NUM_CONV_FEATURES = P_CONV_OUT_CHANNELS * P_NUM_INPUT_PIXELS; //added for spike_vector_to_aer module
    localparam LP_CONV_AER_ADDR_WIDTH = $clog2(LP_NUM_CONV_FEATURES)          ; //

    localparam P_WEIGHT_BRAM_DATA_WIDTH = 64            ;//added for aer_linear module
    localparam P_WEIGHT_BRAM_EFFECTIVE_DEPTH = 392      ;//
    localparam P_NUM_OUTPUT_NEURONS = 10                ;//
    localparam P_USE_MULTICORE_CONV_LIF = 0             ;// 1: 使用多 core 稀疏卷积 LIF
    localparam P_NUM_CONV_LIF_CORES = 4                 ;// 多 core 版本的 core 数量
    localparam P_CORE_EVENT_FIFO_DEPTH = 512            ;// 每个 core 本地 AER FIFO 深度
    localparam P_CORE_FIFO_COUNT_WIDTH = $clog2(P_CORE_EVENT_FIFO_DEPTH + 1);
    localparam P_FORCE_MULTICORE_CONV_LIF = 1           ;// 本 TB 强制启用多 core 路径，便于观察 FIFO 峰值水位
    localparam P_AER_ARB_POLICY = 1                     ;// 0:固定优先级 1:轮询 2:FIFO负载感知
    localparam P_USE_SPARSE_CONV_LIF = 0                ;// 1: 使用稀疏跳过卷积 LIF；0: 使用原始卷积 LIF
    localparam P_USE_STATIC_MASK = 0                    ;// 1: 使用静态 mask AER 全连接；0: 使用原始 AER 全连接
    
    reg tb_clk                     ; 
    reg tb_rst_n                   ;
    reg tb_calc_start              ;
    reg [P_NUM_INPUT_PIXELS-1:0] tb_input_spike_vector;

    wire signed [P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS - 1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] 
          w_all_currents_I         ; //
    wire  w_all_currents_valid     ; //模块间内部信号
    wire w_aer_event_ready                              ;
    wire w_conv_lif_event_valid                         ;
    wire [LP_CONV_AER_ADDR_WIDTH-1:0] w_conv_lif_event_addr;
    wire w_conv_lif_event_frame_done                    ;
    wire w_conv_lif_layer_ready                         ;
    wire [31:0] w_conv_lif_skip_count                   ;
    wire [31:0] w_conv_lif_update_count                 ;
    wire [P_NUM_CONV_LIF_CORES-1:0][P_CORE_FIFO_COUNT_WIDTH-1:0] w_conv_lif_core_fifo_count;
    wire [P_NUM_CONV_LIF_CORES-1:0][P_CORE_FIFO_COUNT_WIDTH-1:0] w_conv_lif_core_fifo_max_count;
    wire w_conv_lif_core_fifo_overflow                  ;
    wire w_fifo_event_valid                             ;
    wire [LP_CONV_AER_ADDR_WIDTH-1:0] w_fifo_event_addr ;
    wire w_fifo_empty                                   ;
    wire w_fifo_full                                    ;
    wire [$clog2(64 + 1)-1:0] w_fifo_count              ;
    wire w_fifo_overflow                                ;
    wire w_aer_event_frame_done                         ;

    reg r_enable_layer          ;        
    reg r_enable_layer_pending  ;
    reg r_aer_frame_done_pending;

    wire [P_NUM_NEURONS-1:0] w_all_spikes_out   ;
    wire w_all_spikes_valid                     ;
    wire signed [P_NUM_OUTPUT_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] tb_neuron_currents;
    wire tb_neuron_currents_valid               ;


    conv_layer_parallel #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT)            ,
        .P_INPUT_WIDTH              (P_INPUT_WIDTH)             ,
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS)        ,
        .P_NUM_OUTPUT_CHANNELS      (P_NUM_OUTPUT_CHANNELS)     ,
        .P_KERNEL_SIZE              (P_KERNEL_SIZE)             ,
        .P_PADDING                  (P_PADDING)                 ,
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH)        ,
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS) ,
        .P_CONV0_WEIGHTS_PACKED     (P_CONV0_WEIGHTS_PACKED)    ,
        .P_CONV1_WEIGHTS_PACKED     (P_CONV1_WEIGHTS_PACKED)    
    ) conv_layer_inst(
        .clk                         (tb_clk)                    ,
        .rst_n                       (tb_rst_n)                  ,
        .i_calc_start                (tb_calc_start)             ,
        .i_input_spike_vector        (tb_input_spike_vector)     ,

        .o_all_currents_I            (w_all_currents_I)         ,
        .o_all_currents_valid        (w_all_currents_valid)
    );

    always @(posedge tb_clk or negedge tb_rst_n) begin
        if (!tb_rst_n) begin
            r_enable_layer <= 1'b0;
            r_enable_layer_pending <= 1'b0;
        end else begin
            r_enable_layer <= 1'b0;

            if (w_all_currents_valid) begin
                r_enable_layer_pending <= 1'b1;
            end

            // 卷积电流有效后先暂存请求，等卷积后 LIF 清零完成并 ready 后再启动。
            if (r_enable_layer_pending && w_conv_lif_layer_ready) begin
                r_enable_layer <= 1'b1;
                r_enable_layer_pending <= 1'b0;
            end
        end
    end

    /*
     * 卷积后 LIF 层可选：
     * P_USE_SPARSE_CONV_LIF=0 使用原始逐地址更新版本；
     * P_USE_SPARSE_CONV_LIF=1 使用感受野/active bit 稀疏跳过版本。
     */
    generate
        if (P_FORCE_MULTICORE_CONV_LIF) begin : gen_multicore_conv_lif
            conv_lif_sparse_multicore #(
                .P_NUM_NEURONS                   (P_NUM_NEURONS),
                .P_NUM_CORES                     (P_NUM_CONV_LIF_CORES),
                .P_CORE_NUM_NEURONS              (P_NUM_NEURONS / P_NUM_CONV_LIF_CORES),
                .P_NUM_INPUT_PIXELS              (P_NUM_INPUT_PIXELS),
                .P_INPUT_HEIGHT                  (P_INPUT_HEIGHT),
                .P_INPUT_WIDTH                   (P_INPUT_WIDTH),
                .P_KERNEL_SIZE                   (P_KERNEL_SIZE),
                .P_PADDING                       (P_PADDING),
                .P_NEURON_VALUE_TOTAL_BITS       (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS        (P_NEURON_VALUE_FRAC_BITS),
                .P_SKIP_THRESHOLD_SHIFT          (5),
                .P_CORE_EVENT_FIFO_DEPTH         (P_CORE_EVENT_FIFO_DEPTH),
                .P_CORE_FIFO_COUNT_WIDTH         (P_CORE_FIFO_COUNT_WIDTH),
                .P_ARB_POLICY                    (P_AER_ARB_POLICY)
            ) conv_lif_sparse_multicore_inst (
                .clk                             (tb_clk),
                .rst_n                           (tb_rst_n),
                .i_enable_layer                  (r_enable_layer),
                .i_input_spike_vector            (tb_input_spike_vector),
                .i_all_currents_I                (w_all_currents_I),
                .o_all_spikes_out                (w_all_spikes_out),
                .o_all_spikes_valid              (w_all_spikes_valid),
                .o_event_valid                   (w_conv_lif_event_valid),
                .o_event_addr                    (w_conv_lif_event_addr),
                .o_event_frame_done              (w_conv_lif_event_frame_done),
                .o_layer_ready                   (w_conv_lif_layer_ready),
                .o_skip_count                    (w_conv_lif_skip_count),
                .o_update_count                  (w_conv_lif_update_count),
                .o_core_fifo_count               (w_conv_lif_core_fifo_count),
                .o_core_fifo_max_count           (w_conv_lif_core_fifo_max_count),
                .o_core_fifo_overflow            (w_conv_lif_core_fifo_overflow)
            );
        end else
        if (P_USE_SPARSE_CONV_LIF) begin : gen_sparse_conv_lif
            conv_lif_layer_sparse #(
                .P_NUM_NEURONS                   (P_NUM_NEURONS),
                .P_NUM_INPUT_PIXELS              (P_NUM_INPUT_PIXELS),
                .P_INPUT_HEIGHT                  (P_INPUT_HEIGHT),
                .P_INPUT_WIDTH                   (P_INPUT_WIDTH),
                .P_KERNEL_SIZE                   (P_KERNEL_SIZE),
                .P_PADDING                       (P_PADDING),
                .P_NEURON_VALUE_TOTAL_BITS       (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS        (P_NEURON_VALUE_FRAC_BITS),
                .P_SKIP_THRESHOLD_SHIFT          (5)
            ) conv_lif_layer_sparse_inst (
                .clk                             (tb_clk),
                .rst_n                           (tb_rst_n),
                .i_enable_layer                  (r_enable_layer),
                .i_input_spike_vector            (tb_input_spike_vector),
                .i_all_currents_I                (w_all_currents_I),
                .o_all_spikes_out                (w_all_spikes_out),
                .o_all_spikes_valid              (w_all_spikes_valid),
                .o_event_valid                   (w_conv_lif_event_valid),
                .o_event_addr                    (w_conv_lif_event_addr),
                .o_event_frame_done              (w_conv_lif_event_frame_done),
                .o_layer_ready                   (w_conv_lif_layer_ready),
                .o_skip_count                    (w_conv_lif_skip_count),
                .o_update_count                  (w_conv_lif_update_count)
            );
            assign w_conv_lif_core_fifo_count = {P_NUM_CONV_LIF_CORES * P_CORE_FIFO_COUNT_WIDTH{1'b0}};
            assign w_conv_lif_core_fifo_max_count = {P_NUM_CONV_LIF_CORES * P_CORE_FIFO_COUNT_WIDTH{1'b0}};
            assign w_conv_lif_core_fifo_overflow = 1'b0;
        end else begin : gen_dense_conv_lif
            conv_lif_layer #(
                .P_NUM_NEURONS                   (P_NUM_NEURONS),
                .P_NEURON_VALUE_TOTAL_BITS       (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS        (P_NEURON_VALUE_FRAC_BITS)
            ) conv_lif_layer_inst (
                .clk                             (tb_clk),
                .rst_n                           (tb_rst_n),
                .i_enable_layer                  (r_enable_layer),
                .i_all_currents_I                (w_all_currents_I),
                .o_all_spikes_out                (w_all_spikes_out),
                .o_all_spikes_valid              (w_all_spikes_valid),
                .o_event_valid                   (w_conv_lif_event_valid),
                .o_event_addr                    (w_conv_lif_event_addr),
                .o_event_frame_done              (w_conv_lif_event_frame_done),
                .o_layer_ready                   (w_conv_lif_layer_ready)
            );

            assign w_conv_lif_skip_count = 32'd0;
            assign w_conv_lif_update_count = w_all_spikes_valid ? P_NUM_NEURONS : 32'd0;
            assign w_conv_lif_core_fifo_count = {P_NUM_CONV_LIF_CORES * P_CORE_FIFO_COUNT_WIDTH{1'b0}};
            assign w_conv_lif_core_fifo_max_count = {P_NUM_CONV_LIF_CORES * P_CORE_FIFO_COUNT_WIDTH{1'b0}};
            assign w_conv_lif_core_fifo_overflow = 1'b0;
        end
    endgenerate
    

    aer_event_fifo #(
        .P_ADDR_WIDTH           (LP_CONV_AER_ADDR_WIDTH),
        .P_FIFO_DEPTH           (64)
    ) u_aer_event_fifo (
        .clk                    (tb_clk),
        .rst_n                  (tb_rst_n),
        .i_clear                (r_enable_layer),
        .i_event_valid          (w_conv_lif_event_valid),
        .i_event_addr           (w_conv_lif_event_addr),
        .i_event_ready          (w_aer_event_ready),
        .o_event_valid          (w_fifo_event_valid),
        .o_event_addr           (w_fifo_event_addr),
        .o_empty                (w_fifo_empty),
        .o_full                 (w_fifo_full),
        .o_count                (w_fifo_count),
        .o_overflow             (w_fifo_overflow)
    );

    assign w_aer_event_frame_done = r_aer_frame_done_pending && w_fifo_empty;

    always @(posedge tb_clk or negedge tb_rst_n) begin
        if (!tb_rst_n) begin
            r_aer_frame_done_pending <= 1'b0;
        end else begin
            if (r_enable_layer) begin
                r_aer_frame_done_pending <= 1'b0;
            end else if (w_conv_lif_event_frame_done) begin
                r_aer_frame_done_pending <= 1'b1;
            end else if (w_aer_event_frame_done) begin
                r_aer_frame_done_pending <= 1'b0;
            end
        end
    end

    /*
     * AER 全连接层可选：
     * P_USE_STATIC_MASK=0 使用原始 dense AER 全连接；
     * P_USE_STATIC_MASK=1 使用带静态剪枝 mask 的 AER 全连接。
     * 打开 mask 时，Vivado 工程中需要存在 fc_mask_0p1 IP。
     */
    generate
        if (P_USE_STATIC_MASK) begin : gen_masked_aer_linear
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
                .clk                    (tb_clk),
                .rst_n                  (tb_rst_n),
                .i_start                (r_enable_layer),
                .i_event_valid          (w_fifo_event_valid),
                .i_event_addr           (w_fifo_event_addr),
                .i_event_frame_done     (w_aer_event_frame_done),
                .o_event_ready          (w_aer_event_ready),
                .o_all_currents_I       (tb_neuron_currents),
                .o_all_currents_valid   (tb_neuron_currents_valid)
            );
        end else begin : gen_dense_aer_linear
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
                .clk                    (tb_clk),
                .rst_n                  (tb_rst_n),
                .i_start                (r_enable_layer),
                .i_event_valid          (w_fifo_event_valid),
                .i_event_addr           (w_fifo_event_addr),
                .i_event_frame_done     (w_aer_event_frame_done),
                .o_event_ready          (w_aer_event_ready),
                .o_all_currents_I       (tb_neuron_currents),
                .o_all_currents_valid   (tb_neuron_currents_valid)
            );
        end
    endgenerate


    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    initial begin
        $display("[%0t ns] SIM_INFO: snn_top_tb simulation START!!!", $time);
        if (P_FORCE_MULTICORE_CONV_LIF) begin
            $display("[%0t ns] SIM_INFO: conv_lif mode = MULTICORE SPARSE, arb_policy=%0d.", $time, P_AER_ARB_POLICY);
        end else if (P_USE_SPARSE_CONV_LIF) begin
            $display("[%0t ns] SIM_INFO: conv_lif mode = SPARSE SKIP.", $time);
        end else begin
            $display("[%0t ns] SIM_INFO: conv_lif mode = DENSE UPDATE.", $time);
        end
        if (P_USE_STATIC_MASK) begin
            $display("[%0t ns] SIM_INFO: AER linear mode = STATIC MASK.", $time);
        end else begin
            $display("[%0t ns] SIM_INFO: AER linear mode = DENSE.", $time);
        end

        tb_rst_n = 1'b0;
        tb_calc_start =1'b0;
        tb_input_spike_vector = 784'b0;
        repeat(5) @(posedge tb_clk);

        tb_rst_n = 1'b1;
        $display("[%0t ns] SIM_INFO: rst_n is RELEASED!!!.", $time);
        repeat(2) @(posedge tb_clk);

        tb_input_spike_vector = {28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000111100000,
                                 28'b0000000000000111100111100000,
                                 28'b0000000000001100000011100000,
                                 28'b0000000001111100000110000000,
                                 28'b0000000001100000011110000000,
                                 28'b0000000111100001111000000000,
                                 28'b0000000111000001110000000000,
                                 28'b0000000110000011100000000000,
                                 28'b0000000111101111000000000000,
                                 28'b0000000111111110000000000000,
                                 28'b0000000000111110000000000000,
                                 28'b0000000000011111110000000000,
                                 28'b0000000000011100011000000000,
                                 28'b0000000000111000000100000000,
                                 28'b0000000000111000001110000000,
                                 28'b0000000000011000000110000000,
                                 28'b0000000000011000001100000000,
                                 28'b0000000000011100011000000000,
                                 28'b0000000000000100011000000000,
                                 28'b0000000000000001100000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000};     // 脚本aer_linear_output_tb.py生成的第0时间步输入点阵

        $display("[%0t ns] SIM_INFO: input vector is READY!!!.", $time);
        repeat(2) @(posedge tb_clk);

        tb_calc_start = 1'b1;
        $display("[%0t ns] SIM_INFO: enable ASSERTED,processing START!!!.", $time);
        repeat(2) @(posedge tb_clk);

        tb_calc_start = 1'b0;

        @(posedge tb_neuron_currents_valid);
        $display("[%0t ns] SIM_INFO: simulation DONE!!!.", $time);
        $display("SIM_INFO: conv_lif_skip=%0d, conv_lif_update=%0d, core_fifo_overflow=%b, out_fifo_overflow=%b",
                 w_conv_lif_skip_count, w_conv_lif_update_count,
                 w_conv_lif_core_fifo_overflow, w_fifo_overflow);
        $display("SIM_INFO: core_fifo_max_count={%0d,%0d,%0d,%0d}",
                 w_conv_lif_core_fifo_max_count[3], w_conv_lif_core_fifo_max_count[2],
                 w_conv_lif_core_fifo_max_count[1], w_conv_lif_core_fifo_max_count[0]);
        #50;
        $finish;

    end

endmodule
