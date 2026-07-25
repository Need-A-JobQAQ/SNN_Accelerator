module conv_layer #(
    parameter P_INPUT_HEIGHT = 28,
    parameter P_INPUT_WIDTH = 28,
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_NUM_OUTPUT_CHANNELS = 2,
    parameter P_KERNEL_SIZE = 3,
    parameter P_PADDING = 1,
    parameter P_WEIGHT_BIT_WIDTH = 16,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV0_WEIGHTS_PACKED = {P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}},
    parameter [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV1_WEIGHTS_PACKED = {P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}}
) (
    input wire clk,
    input wire rst_n,
    input wire i_calc_start,
    input wire [P_NUM_INPUT_PIXELS-1:0] i_input_spike_vector,

    output reg signed [P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS - 1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] o_all_currents_I,
    output reg o_all_currents_valid
);

    /* 卷积层在启动后按顺序扫描输出，每拍计算一个输出像素。 */
    localparam P_NUM_OUTPUT_FEATURES = P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS;
    localparam P_KERNEL_ELEMS = P_KERNEL_SIZE * P_KERNEL_SIZE;
    localparam LP_INDEX_WIDTH = $clog2(P_NUM_OUTPUT_FEATURES);

    localparam [1:0] S_IDLE = 2'b00;
    localparam [1:0] S_PROCESS = 2'b01;
    localparam [1:0] S_DONE = 2'b10;

    reg [1:0] current_state_reg;
    reg [1:0] next_state_reg;
    reg [LP_INDEX_WIDTH-1:0] output_feature_idx_reg;
    reg [P_NUM_INPUT_PIXELS-1:0] latched_spike_vector_reg;

    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] current_conv_sum_comb;
    logic signed [P_WEIGHT_BIT_WIDTH-1:0] conv_weight_got;
    logic input_spike_got;

    integer reset_idx;
    integer out_channel_comb;
    integer spatial_idx_comb;
    integer out_row_comb;
    integer out_col_comb;
    integer write_idx_comb;
    integer kernel_row_comb;
    integer kernel_col_comb;
    integer kernel_idx_comb;

    /* 输入脉冲按光栅顺序存放，最高位对应第一个像素。 */
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

    /* 获取指定输出通道、指定卷积核位置的权重。 */
    function signed [P_WEIGHT_BIT_WIDTH-1:0] get_conv_weight;
        input integer out_channel;
        input integer kernel_idx;
        integer bit_base;
        begin
            bit_base = (P_KERNEL_ELEMS - 1 - kernel_idx) * P_WEIGHT_BIT_WIDTH;
            case (out_channel)
                0: get_conv_weight = P_CONV0_WEIGHTS_PACKED[bit_base +: P_WEIGHT_BIT_WIDTH];
                1: get_conv_weight = P_CONV1_WEIGHTS_PACKED[bit_base +: P_WEIGHT_BIT_WIDTH];
                default: get_conv_weight = {P_WEIGHT_BIT_WIDTH{1'b0}};
            endcase
        end
    endfunction

    always @(*) begin
        current_conv_sum_comb = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
        conv_weight_got = {P_WEIGHT_BIT_WIDTH{1'b0}};
        input_spike_got = 1'b0;

        /* 输出特征按扁平顺序扫描：先通道，后通道内的光栅位置。 */
        out_channel_comb = output_feature_idx_reg / P_NUM_INPUT_PIXELS;
        spatial_idx_comb = output_feature_idx_reg % P_NUM_INPUT_PIXELS;
        out_row_comb = spatial_idx_comb / P_INPUT_WIDTH;
        out_col_comb = spatial_idx_comb % P_INPUT_WIDTH;
        write_idx_comb = (out_channel_comb * P_NUM_INPUT_PIXELS) +
                         (P_NUM_INPUT_PIXELS - 1 - spatial_idx_comb);

        /* 输入是二值脉冲，因此卷积可以简化成：有脉冲就累加对应权重。 */
        for (kernel_row_comb = 0; kernel_row_comb < P_KERNEL_SIZE; kernel_row_comb = kernel_row_comb + 1) begin
            for (kernel_col_comb = 0; kernel_col_comb < P_KERNEL_SIZE; kernel_col_comb = kernel_col_comb + 1) begin

                kernel_idx_comb = (kernel_row_comb * P_KERNEL_SIZE) + kernel_col_comb;
                input_spike_got = get_input_spike(
                    latched_spike_vector_reg,
                    out_row_comb + kernel_row_comb - P_PADDING,
                    out_col_comb + kernel_col_comb - P_PADDING);

                if (input_spike_got) begin
                    conv_weight_got = get_conv_weight(out_channel_comb, kernel_idx_comb);
                    current_conv_sum_comb = current_conv_sum_comb +
                        {{(P_NEURON_VALUE_TOTAL_BITS - P_WEIGHT_BIT_WIDTH){conv_weight_got[P_WEIGHT_BIT_WIDTH-1]}},
                         conv_weight_got};
                end
            end
        end
    end

    always @(*) begin
        next_state_reg = current_state_reg;

        case (current_state_reg)
            S_IDLE: begin
                if (i_calc_start) begin
                    next_state_reg = S_PROCESS;
                end
            end

            S_PROCESS: begin
                if (output_feature_idx_reg == P_NUM_OUTPUT_FEATURES - 1) begin
                    next_state_reg = S_DONE;
                end else begin
                    next_state_reg = S_PROCESS;
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
            current_state_reg <= S_IDLE;
            output_feature_idx_reg <= {LP_INDEX_WIDTH{1'b0}};
            latched_spike_vector_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
            o_all_currents_valid <= 1'b0;
            for (reset_idx = 0; reset_idx < P_NUM_OUTPUT_FEATURES; reset_idx = reset_idx + 1) begin
                o_all_currents_I[reset_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            end
        end else begin
            current_state_reg <= next_state_reg;
            o_all_currents_valid <= 1'b0;

            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESS) begin
                latched_spike_vector_reg <= i_input_spike_vector;
                output_feature_idx_reg <= {LP_INDEX_WIDTH{1'b0}};
            end

            if (current_state_reg == S_PROCESS) begin
                o_all_currents_I[write_idx_comb] <= current_conv_sum_comb;
            end

            if (current_state_reg == S_PROCESS && next_state_reg == S_PROCESS) begin
                output_feature_idx_reg <= output_feature_idx_reg + 1'b1;
            end

            if (current_state_reg == S_DONE) begin
                o_all_currents_valid <= 1'b1;
            end
        end
    end

endmodule
