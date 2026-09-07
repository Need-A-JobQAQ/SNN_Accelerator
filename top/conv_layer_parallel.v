module conv_layer_parallel #(
    parameter P_INPUT_HEIGHT = 28,
    parameter P_INPUT_WIDTH = 28,
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_NUM_OUTPUT_CHANNELS = 2,
    parameter P_KERNEL_SIZE = 3,
    parameter P_PADDING = 1,
    parameter P_WEIGHT_BIT_WIDTH = 16,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV0_WEIGHTS_PACKED =
        {P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}},
    parameter [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV1_WEIGHTS_PACKED =
        {P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}}
) (
    // 基础控制信号
    input wire                                           clk,
    input wire                                           rst_n,
    input wire                                           i_calc_start,

    // 卷积层输入脉冲向量
    input wire [P_NUM_INPUT_PIXELS-1:0]                  i_input_spike_vector,

    // 通道 0 卷积电流写流
    output wire                                          o_current_ch0_wr_en,
    output wire [$clog2(P_NUM_INPUT_PIXELS)-1:0]         o_current_ch0_wr_addr,
    output wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]   o_current_ch0_wr_data,
    output wire                                          o_current_ch0_wr_active,

    // 通道 1 卷积电流写流
    output wire                                          o_current_ch1_wr_en,
    output wire [$clog2(P_NUM_INPUT_PIXELS)-1:0]         o_current_ch1_wr_addr,
    output wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]   o_current_ch1_wr_data,
    output wire                                          o_current_ch1_wr_active,

    // 两个通道都完成当前时间步卷积后拉高
    output wire                                          o_current_wr_done
);

    /*
     * 并行卷积层。
     * 本模块只负责两个卷积 PE 的并行计算，并把结果以写流形式输出。
     * current RAM、current_valid_bitmap 和后级读口已经迁移到 conv_current_*_buffer。
     */
    localparam LP_ADDR_WIDTH = $clog2(P_NUM_INPUT_PIXELS);

    wire ch0_current_valid_w;
    wire [LP_ADDR_WIDTH-1:0] ch0_current_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ch0_current_data_w;
    wire ch0_current_active_w;
    wire ch0_done_w;

    wire ch1_current_valid_w;
    wire [LP_ADDR_WIDTH-1:0] ch1_current_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ch1_current_data_w;
    wire ch1_current_active_w;
    wire ch1_done_w;

    assign o_current_ch0_wr_en = ch0_current_valid_w;
    assign o_current_ch0_wr_addr = ch0_current_addr_w;
    assign o_current_ch0_wr_data = ch0_current_data_w;
    assign o_current_ch0_wr_active = ch0_current_active_w;

    assign o_current_ch1_wr_en = ch1_current_valid_w;
    assign o_current_ch1_wr_addr = ch1_current_addr_w;
    assign o_current_ch1_wr_data = ch1_current_data_w;
    assign o_current_ch1_wr_active = ch1_current_active_w;

    assign o_current_wr_done = ch0_done_w && ch1_done_w;

    conv_pe #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_KERNEL_SIZE              (P_KERNEL_SIZE),
        .P_PADDING                  (P_PADDING),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_KERNEL_WEIGHTS_PACKED    (P_CONV0_WEIGHTS_PACKED)
    ) u_conv_pe_ch0 (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_start                (i_calc_start),
        .i_input_spike_vector   (i_input_spike_vector),
        .o_current_valid        (ch0_current_valid_w),
        .o_current_addr         (ch0_current_addr_w),
        .o_current_data         (ch0_current_data_w),
        .o_current_active       (ch0_current_active_w),
        .o_done                 (ch0_done_w)
    );

    conv_pe #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_KERNEL_SIZE              (P_KERNEL_SIZE),
        .P_PADDING                  (P_PADDING),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_KERNEL_WEIGHTS_PACKED    (P_CONV1_WEIGHTS_PACKED)
    ) u_conv_pe_ch1 (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_start                (i_calc_start),
        .i_input_spike_vector   (i_input_spike_vector),
        .o_current_valid        (ch1_current_valid_w),
        .o_current_addr         (ch1_current_addr_w),
        .o_current_data         (ch1_current_data_w),
        .o_current_active       (ch1_current_active_w),
        .o_done                 (ch1_done_w)
    );

endmodule
