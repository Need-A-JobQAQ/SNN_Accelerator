module conv_pe #(
    parameter P_INPUT_HEIGHT = 28,
    parameter P_INPUT_WIDTH = 28,
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_KERNEL_SIZE = 3,
    parameter P_PADDING = 1,
    parameter P_WEIGHT_BIT_WIDTH = 16,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_KERNEL_WEIGHTS_PACKED =
        {P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}}
) (
    input wire clk,
    input wire rst_n,
    input wire i_start,
    input wire [P_NUM_INPUT_PIXELS-1:0] i_input_spike_vector,

    output reg o_current_valid,
    output reg [$clog2(P_NUM_INPUT_PIXELS)-1:0] o_current_addr,
    output reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] o_current_data,
    output reg o_current_active,
    output reg o_done
);

    /*
     * 单个卷积 PE 只负责一个输出通道。
     * 每拍计算一个输出像素，并以 valid/addr/data 的形式输出卷积电流。
     */
    localparam P_KERNEL_ELEMS = P_KERNEL_SIZE * P_KERNEL_SIZE;
    localparam LP_PIXEL_INDEX_WIDTH = $clog2(P_NUM_INPUT_PIXELS);

    localparam [1:0] S_IDLE = 2'b00;
    localparam [1:0] S_PROCESS = 2'b01;
    localparam [1:0] S_DONE = 2'b10;

    reg [1:0] current_state_reg;
    reg [1:0] next_state_reg;
    reg [LP_PIXEL_INDEX_WIDTH-1:0] spatial_idx_reg;
    reg [$clog2(P_INPUT_HEIGHT)-1:0] row_reg;
    reg [$clog2(P_INPUT_WIDTH)-1:0] col_reg;
    reg [LP_PIXEL_INDEX_WIDTH-1:0] write_idx_reg;
    reg [P_NUM_INPUT_PIXELS-1:0] latched_spike_vector_reg;

    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] current_conv_sum_comb;
    reg signed [P_WEIGHT_BIT_WIDTH-1:0] conv_weight_comb;
    reg input_spike_comb;
    reg current_active_comb;

    integer kernel_row_comb;
    integer kernel_col_comb;
    integer kernel_idx_comb;

    /*
     * 输入脉冲向量沿用工程约定：
     * 最高位对应图像左上角第一个像素，最低位对应右下角像素。
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
     * 卷积核参数按导出顺序打包：
     * kernel_idx=0 对应 packed 参数最高 16bit。
     */
    function signed [P_WEIGHT_BIT_WIDTH-1:0] get_kernel_weight;
        input integer kernel_idx;
        integer bit_base;
        begin
            bit_base = (P_KERNEL_ELEMS - 1 - kernel_idx) * P_WEIGHT_BIT_WIDTH;
            get_kernel_weight = P_KERNEL_WEIGHTS_PACKED[bit_base +: P_WEIGHT_BIT_WIDTH];
        end
    endfunction

    /*
     * 组合卷积求和。
     * 输入是二值脉冲，因此乘法可以简化为：有脉冲则累加对应权重。
     */
    always @(*) begin
        current_conv_sum_comb = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
        conv_weight_comb = {P_WEIGHT_BIT_WIDTH{1'b0}};
        input_spike_comb = 1'b0;
        current_active_comb = 1'b0;

        for (kernel_row_comb = 0; kernel_row_comb < P_KERNEL_SIZE; kernel_row_comb = kernel_row_comb + 1) begin
            for (kernel_col_comb = 0; kernel_col_comb < P_KERNEL_SIZE; kernel_col_comb = kernel_col_comb + 1) begin
                kernel_idx_comb = (kernel_row_comb * P_KERNEL_SIZE) + kernel_col_comb;
                input_spike_comb = get_input_spike(
                    latched_spike_vector_reg,
                    row_reg + kernel_row_comb - P_PADDING,
                    col_reg + kernel_col_comb - P_PADDING);

                if (input_spike_comb) begin
                    current_active_comb = 1'b1;
                    conv_weight_comb = get_kernel_weight(kernel_idx_comb);
                    current_conv_sum_comb = current_conv_sum_comb +
                        {{(P_NEURON_VALUE_TOTAL_BITS - P_WEIGHT_BIT_WIDTH){conv_weight_comb[P_WEIGHT_BIT_WIDTH-1]}},
                         conv_weight_comb};
                end
            end
        end
    end

    /*
     * 两段式状态机的组合转移逻辑。
     */
    always @(*) begin
        next_state_reg = current_state_reg;

        case (current_state_reg)
            S_IDLE: begin
                if (i_start) begin
                    next_state_reg = S_PROCESS;
                end
            end

            S_PROCESS: begin
                if (spatial_idx_reg == P_NUM_INPUT_PIXELS - 1) begin
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
            current_state_reg <= S_IDLE;
        end else begin
            current_state_reg <= next_state_reg;
        end
    end

    /*
     * 输入锁存和输出像素扫描位置更新。
     * row/col/write_idx 用寄存器逐拍维护，避免每拍组合除法、取模和地址减法。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spatial_idx_reg <= {LP_PIXEL_INDEX_WIDTH{1'b0}};
            row_reg <= {$clog2(P_INPUT_HEIGHT){1'b0}};
            col_reg <= {$clog2(P_INPUT_WIDTH){1'b0}};
            write_idx_reg <= P_NUM_INPUT_PIXELS - 1;
            latched_spike_vector_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
        end else begin
            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESS) begin
                latched_spike_vector_reg <= i_input_spike_vector;
                spatial_idx_reg <= {LP_PIXEL_INDEX_WIDTH{1'b0}};
                row_reg <= {$clog2(P_INPUT_HEIGHT){1'b0}};
                col_reg <= {$clog2(P_INPUT_WIDTH){1'b0}};
                write_idx_reg <= P_NUM_INPUT_PIXELS - 1;
            end

            if (current_state_reg == S_PROCESS && next_state_reg == S_PROCESS) begin
                spatial_idx_reg <= spatial_idx_reg + 1'b1;
                write_idx_reg <= write_idx_reg - 1'b1;
                if (col_reg == P_INPUT_WIDTH - 1) begin
                    col_reg <= {$clog2(P_INPUT_WIDTH){1'b0}};
                    row_reg <= row_reg + 1'b1;
                end else begin
                    col_reg <= col_reg + 1'b1;
                end
            end
        end
    end

    /*
     * 卷积结果流式输出。
     * PE 不再保存完整通道电流数组，避免综合成大规模寄存器译码网络。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_current_valid <= 1'b0;
            o_current_addr <= {$clog2(P_NUM_INPUT_PIXELS){1'b0}};
            o_current_data <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            o_current_active <= 1'b0;
        end else begin
            o_current_valid <= 1'b0;
            o_current_active <= 1'b0;
            if (current_state_reg == S_PROCESS) begin
                o_current_valid <= 1'b1;
                o_current_addr <= write_idx_reg;
                o_current_data <= current_conv_sum_comb;
                o_current_active <= current_active_comb;
            end
        end
    end

    /*
     * 完成脉冲，S_DONE 状态下拉高 1 拍。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_done <= 1'b0;
        end else begin
            o_done <= 1'b0;
            if (current_state_reg == S_DONE) begin
                o_done <= 1'b1;
            end
        end
    end

endmodule
