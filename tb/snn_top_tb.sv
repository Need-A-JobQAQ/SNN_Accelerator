// snn_top_tb.v
// (中文注释版 - 适应 snn_top.v 最新端口定义)
// snn_top 模块的顶层测试平台

`timescale 1ns/1ps

module snn_top_tb;

    // --- 测试平台参数 ---
    localparam CLK_PERIOD = 10; // ns, 时钟周期 (100MHz)

    // DUT (snn_top) 参数
    localparam P_NUM_INPUT_PIXELS          = 784;
    localparam P_INPUT_HEIGHT              = 28;
    localparam P_INPUT_WIDTH               = 28;
    localparam P_PIXEL_INTENSITY_BITS      = 8;
    localparam P_IMAGE_BRAM_DATA_WIDTH     = 64;
    localparam P_IMAGE_BRAM_DEPTH          = 98;
    localparam P_PRNG_LFSR_WIDTH           = 32;
    localparam P_T_MAX                     = 100; 
    localparam P_WEIGHT_BIT_WIDTH          = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS   = 26;
    localparam P_NEURON_VALUE_FRAC_BITS    = 12;
    localparam P_WEIGHT_BRAM_DATA_WIDTH    = 64;
    localparam P_WEIGHT_BRAM_EFFECTIVE_DEPTH = 392;
    localparam P_NUM_OUTPUT_NEURONS        = 10;
    localparam P_CONV_OUT_CHANNELS         = 2;
    localparam P_CONV_KERNEL_SIZE          = 3;
    localparam P_CONV_PADDING              = 1;
    localparam P_USE_SPARSE_CONV_LIF       = 0;
    localparam P_USE_MULTICORE_CONV_LIF    = 1;
    localparam P_USE_BLOCK_AER_FC          = 1;
    localparam P_CONV_WEIGHT_PACKED_WIDTH  = P_CONV_KERNEL_SIZE * P_CONV_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH;
    localparam [P_CONV_WEIGHT_PACKED_WIDTH-1:0] P_CONV0_WEIGHTS_PACKED = {16'hFA52,16'h1085,16'hFF5B,
                                                                          16'h0602,16'h0A60,16'hF5AA,
                                                                          16'h1314,16'h05C8,16'hEA50};
    localparam [P_CONV_WEIGHT_PACKED_WIDTH-1:0] P_CONV1_WEIGHTS_PACKED = {16'hE764,16'h41B2,16'h272A,
                                                                          16'hF6DD,16'h61AB,16'hF8CB,
                                                                          16'h36E8,16'h2EFE,16'hE59C};
    localparam LP_NUM_CONV_FEATURES        = P_CONV_OUT_CHANNELS * P_NUM_INPUT_PIXELS;
    // 顶层已加入卷积层、卷积后 LIF 和 AER 全连接层。
    // 单个时间步最长路径变为：
    // 泊松编码 -> 卷积层 -> 卷积LIF -> spike_vector_to_aer -> aer_linear_layer -> 输出LIF。
    // 这里不再靠固定等待周期采样，而是等待顶层内部完成脉冲，并保留超时保护。
    localparam TOTAL_SIM_TIMEOUT_CYCLES =
        (P_IMAGE_BRAM_DEPTH + 20) +
        (P_T_MAX * (LP_NUM_CONV_FEATURES + (LP_NUM_CONV_FEATURES * 5) + 200)) +
        1000;

    // 信号声明 - 连接到 DUT 的输入
    reg                                   tb_clk;
    reg                                   tb_rst_n;
    reg                                   tb_i_start_new_image_processing;
    integer                               timeout_count;
    
    // 信号声明 - 连接到 DUT 的输出
    wire [$clog2(P_NUM_OUTPUT_NEURONS)-1:0] tb_o_predicted_label;
    wire [31:0]                            tb_perf_total_cycles;
    wire [31:0]                            tb_perf_total_aer_events;
    wire [31:0]                            tb_perf_last_aer_events;
    wire [31:0]                            tb_perf_last_aer_fc_cycles;
    wire [31:0]                            tb_perf_last_conv_cycles;
    wire [31:0]                            tb_perf_total_conv_cycles;
    wire [31:0]                            tb_perf_last_conv_lif_cycles;
    wire [31:0]                            tb_perf_total_conv_lif_cycles;
    wire [31:0]                            tb_perf_actual_time_steps;
    wire [31:0]                            tb_perf_last_conv_lif_skip_count;
    wire [31:0]                            tb_perf_last_conv_lif_update_count;
    wire [31:0]                            tb_perf_total_conv_lif_skip_count;
    wire [31:0]                            tb_perf_total_conv_lif_update_count;
    wire                                   tb_perf_early_stop;
    wire                                   tb_perf_valid;
    // wire                                   tb_o_prediction_valid; // 已从 snn_top 模块定义中移除
    // wire                                   tb_o_snn_overall_busy; // 已从 snn_top 模块定义中移除
    // --- Block-Mask AER 压缩率统计 ---
    // 按 16/32/64 个地址为一块，统计普通 AER 事件可合并成多少个块事件包。
    localparam LP_BLOCK16_COUNT = (LP_NUM_CONV_FEATURES + 15) / 16;
    localparam LP_BLOCK32_COUNT = (LP_NUM_CONV_FEATURES + 31) / 32;
    localparam LP_BLOCK64_COUNT = (LP_NUM_CONV_FEATURES + 63) / 64;
    localparam LP_WEIGHT_GROUP4_COUNT = (LP_NUM_CONV_FEATURES + 3) / 4;

    reg [LP_BLOCK16_COUNT-1:0] stat_block16_seen;
    reg [LP_BLOCK32_COUNT-1:0] stat_block32_seen;
    reg [LP_BLOCK64_COUNT-1:0] stat_block64_seen;
    reg [LP_WEIGHT_GROUP4_COUNT-1:0] stat_group4_seen;
    integer                    stat_aer_events_total;
    integer                    stat_block16_packets_total;
    integer                    stat_block32_packets_total;
    integer                    stat_block64_packets_total;
    integer                    stat_group4_reads_total;
    integer                    stat_aer_events_last;
    integer                    stat_block16_packets_last;
    integer                    stat_block32_packets_last;
    integer                    stat_block64_packets_last;
    integer                    stat_group4_reads_last;


    // --- DUT (snn_top) 实例化 ---
    snn_top #(
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_PIXEL_INTENSITY_BITS     (P_PIXEL_INTENSITY_BITS),
        .P_IMAGE_BRAM_DATA_WIDTH    (P_IMAGE_BRAM_DATA_WIDTH),
        .P_IMAGE_BRAM_DEPTH         (P_IMAGE_BRAM_DEPTH),
        .P_PRNG_LFSR_WIDTH          (P_PRNG_LFSR_WIDTH),
        .P_T_MAX                    (P_T_MAX),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS   (P_NEURON_VALUE_FRAC_BITS),
        .P_WEIGHT_BRAM_DATA_WIDTH   (P_WEIGHT_BRAM_DATA_WIDTH),
        .P_WEIGHT_BRAM_EFFECTIVE_DEPTH (P_WEIGHT_BRAM_EFFECTIVE_DEPTH),
        .P_NUM_OUTPUT_NEURONS       (P_NUM_OUTPUT_NEURONS),
        .P_CONV_OUT_CHANNELS        (P_CONV_OUT_CHANNELS),
        .P_CONV_KERNEL_SIZE         (P_CONV_KERNEL_SIZE),
        .P_CONV_PADDING             (P_CONV_PADDING),
        .P_CONV0_WEIGHTS_PACKED     (P_CONV0_WEIGHTS_PACKED),
        .P_CONV1_WEIGHTS_PACKED     (P_CONV1_WEIGHTS_PACKED),
        .P_USE_SPARSE_CONV_LIF      (P_USE_SPARSE_CONV_LIF),
        .P_USE_MULTICORE_CONV_LIF   (P_USE_MULTICORE_CONV_LIF),
        .P_USE_BLOCK_AER_FC         (P_USE_BLOCK_AER_FC)
    ) u_snn_top_inst (
        .clk                          (tb_clk),
        .rst_n                        (tb_rst_n),
        .i_start_new_image_processing (tb_i_start_new_image_processing),
        .o_predicted_label            (tb_o_predicted_label),
        .o_perf_total_cycles          (tb_perf_total_cycles),
        .o_perf_total_aer_events      (tb_perf_total_aer_events),
        .o_perf_last_aer_events       (tb_perf_last_aer_events),
        .o_perf_last_aer_fc_cycles    (tb_perf_last_aer_fc_cycles),
        .o_perf_last_conv_cycles      (tb_perf_last_conv_cycles),
        .o_perf_total_conv_cycles     (tb_perf_total_conv_cycles),
        .o_perf_last_conv_lif_cycles  (tb_perf_last_conv_lif_cycles),
        .o_perf_total_conv_lif_cycles (tb_perf_total_conv_lif_cycles),
        .o_perf_actual_time_steps     (tb_perf_actual_time_steps),
        .o_perf_last_conv_lif_skip_count   (tb_perf_last_conv_lif_skip_count),
        .o_perf_last_conv_lif_update_count (tb_perf_last_conv_lif_update_count),
        .o_perf_total_conv_lif_skip_count  (tb_perf_total_conv_lif_skip_count),
        .o_perf_total_conv_lif_update_count(tb_perf_total_conv_lif_update_count),
        .o_perf_early_stop            (tb_perf_early_stop),
        .o_perf_valid                 (tb_perf_valid)
        // .o_prediction_valid        ( ), // DUT端口已移除或悬空
        // .o_snn_overall_busy        ( )  // DUT端口已移除或悬空
    );

    // 时钟生成
    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end
    always @(posedge tb_clk or negedge tb_rst_n) begin
        if (!tb_rst_n) begin
            stat_block16_seen <= {LP_BLOCK16_COUNT{1'b0}};
            stat_block32_seen <= {LP_BLOCK32_COUNT{1'b0}};
            stat_block64_seen <= {LP_BLOCK64_COUNT{1'b0}};
            stat_group4_seen <= {LP_WEIGHT_GROUP4_COUNT{1'b0}};
            stat_aer_events_total <= 0;
            stat_block16_packets_total <= 0;
            stat_block32_packets_total <= 0;
            stat_block64_packets_total <= 0;
            stat_group4_reads_total <= 0;
            stat_aer_events_last <= 0;
            stat_block16_packets_last <= 0;
            stat_block32_packets_last <= 0;
            stat_block64_packets_last <= 0;
            stat_group4_reads_last <= 0;
        end else begin
            if (u_snn_top_inst.conv_lif_actual_enable_r) begin
                stat_block16_seen <= {LP_BLOCK16_COUNT{1'b0}};
                stat_block32_seen <= {LP_BLOCK32_COUNT{1'b0}};
                stat_block64_seen <= {LP_BLOCK64_COUNT{1'b0}};
                stat_group4_seen <= {LP_WEIGHT_GROUP4_COUNT{1'b0}};
                stat_aer_events_last <= 0;
                stat_block16_packets_last <= 0;
                stat_block32_packets_last <= 0;
                stat_block64_packets_last <= 0;
                stat_group4_reads_last <= 0;
            end

            if (u_snn_top_inst.perf_aer_event_accept_w) begin
                stat_aer_events_total <= stat_aer_events_total + 1;
                stat_aer_events_last <= stat_aer_events_last + 1;

                if (!stat_block16_seen[u_snn_top_inst.aer_event_addr[10:4]]) begin
                    stat_block16_seen[u_snn_top_inst.aer_event_addr[10:4]] <= 1'b1;
                    stat_block16_packets_total <= stat_block16_packets_total + 1;
                    stat_block16_packets_last <= stat_block16_packets_last + 1;
                end

                if (!stat_block32_seen[u_snn_top_inst.aer_event_addr[10:5]]) begin
                    stat_block32_seen[u_snn_top_inst.aer_event_addr[10:5]] <= 1'b1;
                    stat_block32_packets_total <= stat_block32_packets_total + 1;
                    stat_block32_packets_last <= stat_block32_packets_last + 1;
                end

                if (!stat_block64_seen[u_snn_top_inst.aer_event_addr[10:6]]) begin
                    stat_block64_seen[u_snn_top_inst.aer_event_addr[10:6]] <= 1'b1;
                    stat_block64_packets_total <= stat_block64_packets_total + 1;
                    stat_block64_packets_last <= stat_block64_packets_last + 1;
                end
                if (!stat_group4_seen[u_snn_top_inst.aer_event_addr[10:2]]) begin
                    stat_group4_seen[u_snn_top_inst.aer_event_addr[10:2]] <= 1'b1;
                    stat_group4_reads_total <= stat_group4_reads_total + 1;
                    stat_group4_reads_last <= stat_group4_reads_last + 1;
                end
            end
        end
    end

    // 激励和检查
    initial begin
        $display("[%0t ns] SIM_INFO: snn_top_tb simulation start", $time);
        if (P_USE_BLOCK_AER_FC) begin
            $display("[%0t ns] SIM_INFO: AER FC mode = BLOCK-MASK.", $time);
        end else begin
            $display("[%0t ns] SIM_INFO: AER FC mode = DENSE.", $time);
        end
        if (P_USE_MULTICORE_CONV_LIF) begin
            $display("[%0t ns] SIM_INFO: conv_lif mode = 4-CORE SPARSE SKIP.", $time);
        end else if (P_USE_SPARSE_CONV_LIF) begin
            $display("[%0t ns] SIM_INFO: conv_lif mode = SPARSE SKIP.", $time);
        end else begin
            $display("[%0t ns] SIM_INFO: conv_lif mode = DENSE UPDATE.", $time);
        end

        tb_i_start_new_image_processing = 1'b0;
        tb_rst_n = 1'b0; 
        repeat(5) @(posedge tb_clk);
        tb_rst_n = 1'b1; 
        $display("[%0t ns] SIM_INFO: reset_n is already released.", $time);
        repeat(2) @(posedge tb_clk);

        $display("[%0t ns] SIM_INFO: image_loader enable is to asserte (i_start_new_image_processing = 1).", $time);
        tb_i_start_new_image_processing = 1'b1;
        @(posedge tb_clk);
        tb_i_start_new_image_processing = 1'b0;

        $display("[%0t ns] SIM_INFO: SNN开始处理,等待 cu_global_processing_done,超时上限为 %0d 个时钟周期。",
                 $time, TOTAL_SIM_TIMEOUT_CYCLES);

        timeout_count = 0;
        while (u_snn_top_inst.cu_global_processing_done !== 1'b1 &&
               timeout_count < TOTAL_SIM_TIMEOUT_CYCLES) begin
            @(posedge tb_clk);
            timeout_count = timeout_count + 1;
        end

        if (u_snn_top_inst.cu_global_processing_done === 1'b1) begin
            $display("[%0t ns] SIM_INFO: 检测到 cu_global_processing_done,实际等待 %0d 个时钟周期。",
                     $time, timeout_count);
        end else begin
            $display("[%0t ns] SIM_ERROR: 等待超时，未检测到 cu_global_processing_done。",
                     $time);
        end

        if (P_USE_MULTICORE_CONV_LIF) begin
            if (P_USE_BLOCK_AER_FC) begin
                $display("SIM_INFO: mode_summary conv_lif=4-core_sparse, aer_fc=block_mask, current_buffer=pingpong");
            end else begin
                $display("SIM_INFO: mode_summary conv_lif=4-core_sparse, aer_fc=dense_aer, current_buffer=pingpong");
            end
        end else if (P_USE_SPARSE_CONV_LIF) begin
            if (P_USE_BLOCK_AER_FC) begin
                $display("SIM_INFO: mode_summary conv_lif=single_sparse, aer_fc=block_mask, current_buffer=pingpong");
            end else begin
                $display("SIM_INFO: mode_summary conv_lif=single_sparse, aer_fc=dense_aer, current_buffer=pingpong");
            end
        end else begin
            if (P_USE_BLOCK_AER_FC) begin
                $display("SIM_INFO: mode_summary conv_lif=dense, aer_fc=block_mask, current_buffer=pingpong");
            end else begin
                $display("SIM_INFO: mode_summary conv_lif=dense, aer_fc=dense_aer, current_buffer=pingpong");
            end
        end

        $display("SIM_INFO: final predicted label is o_predicted_label = %d (十六进制: %h)",
                 tb_o_predicted_label, tb_o_predicted_label);
        $display("SIM_INFO: 性能计数 total_cycles=%0d, actual_time_steps=%0d, early_stop=%b, total_aer_events=%0d, last_aer_events=%0d, last_aer_fc_cycles=%0d, last_conv_lif_skip=%0d, last_conv_lif_update=%0d, total_conv_lif_skip=%0d, total_conv_lif_update=%0d, perf_valid=%b",
                 tb_perf_total_cycles,
                 tb_perf_actual_time_steps,
                 tb_perf_early_stop,
                 tb_perf_total_aer_events,
                 tb_perf_last_aer_events,
                 tb_perf_last_aer_fc_cycles,
                 tb_perf_last_conv_lif_skip_count,
                 tb_perf_last_conv_lif_update_count,
                 tb_perf_total_conv_lif_skip_count,
                 tb_perf_total_conv_lif_update_count,
                 tb_perf_valid);        
        $display("SIM_INFO: phase_cycles last_conv=%0d, total_conv=%0d, last_conv_lif=%0d, total_conv_lif=%0d, last_aer_fc=%0d",
                 tb_perf_last_conv_cycles,
                 tb_perf_total_conv_cycles,
                 tb_perf_last_conv_lif_cycles,
                 tb_perf_total_conv_lif_cycles,
                 tb_perf_last_aer_fc_cycles);
        $display("SIM_INFO: block_mask_aer events_total=%0d, pkt16=%0d, pkt32=%0d, pkt64=%0d",
                 stat_aer_events_total,
                 stat_block16_packets_total,
                 stat_block32_packets_total,
                 stat_block64_packets_total);
        if (stat_aer_events_total != 0) begin
            $display("SIM_INFO: block_mask_aer compression pkt16=%0d%%, pkt32=%0d%%, pkt64=%0d%%",
                     100 - (stat_block16_packets_total * 100) / stat_aer_events_total,
                     100 - (stat_block32_packets_total * 100) / stat_aer_events_total,
                     100 - (stat_block64_packets_total * 100) / stat_aer_events_total);
        end
        $display("SIM_INFO: block_mask_aer last_events=%0d, last_pkt16=%0d, last_pkt32=%0d, last_pkt64=%0d",
                 stat_aer_events_last,
                 stat_block16_packets_last,
                 stat_block32_packets_last,
                 stat_block64_packets_last);
        $display("SIM_INFO: block_mask_aer weight_group4_reads total=%0d, last=%0d",
                 stat_group4_reads_total,
                 stat_group4_reads_last);
        if (stat_aer_events_total != 0) begin
            $display("SIM_INFO: block_mask_aer weight_read_reduce group4=%0d%%",
                     100 - (stat_group4_reads_total * 100) / stat_aer_events_total);
        end
        
        repeat(10) @(posedge tb_clk);
        $display("[%0t ns] SIM_INFO: snn_top_tb simulation done", $time);
        $finish;
    end

    initial begin
        #1; 
        $monitor("[%0t ns] TB_MONITOR: clk=%b, rst_n=%b, start_proc_img=%b || label=%d",
                 $time, tb_clk, tb_rst_n, tb_i_start_new_image_processing,
                 tb_o_predicted_label);
    end

endmodule






