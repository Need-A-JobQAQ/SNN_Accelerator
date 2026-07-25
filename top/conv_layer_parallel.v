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
    input wire clk,
    input wire rst_n,
    input wire i_calc_start,
    input wire [P_NUM_INPUT_PIXELS-1:0] i_input_spike_vector,

    output reg signed [P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS - 1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] o_all_currents_I,
    output reg o_all_currents_valid
);

    /*
     * 两 PE 并行卷积层。
     * PE0 计算输出通道0，PE1 计算输出通道1；
     * 对外保持和原 conv_layer 基本一致的接口，方便顶层替换和结果对比。
     */
    wire signed [P_NUM_INPUT_PIXELS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] ch0_currents_w;
    wire signed [P_NUM_INPUT_PIXELS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] ch1_currents_w;
    wire ch0_done_w;
    wire ch1_done_w;

    integer out_idx;

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
        .o_channel_currents     (ch0_currents_w),
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
        .o_channel_currents     (ch1_currents_w),
        .o_done                 (ch1_done_w)
    );

    /*
     * 输出拼接保持现有工程约定：
     * 低 784 个位置保存 channel0，高 784 个位置保存 channel1。
     */
    always @(*) begin
        for (out_idx = 0; out_idx < P_NUM_INPUT_PIXELS; out_idx = out_idx + 1) begin
            o_all_currents_I[out_idx] = ch0_currents_w[out_idx];
            o_all_currents_I[P_NUM_INPUT_PIXELS + out_idx] = ch1_currents_w[out_idx];
        end
    end

    /*
     * 两个 PE 同时启动、工作量相同，正常会同拍 done。
     * 这里仍然等待两个 done 同时有效，避免任一 PE 未完成时提前通知后级。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_all_currents_valid <= 1'b0;
        end else begin
            o_all_currents_valid <= ch0_done_w && ch1_done_w;
        end
    end

endmodule
