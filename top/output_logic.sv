// output_logic.v
// 负责在 P_T_MAX 个时间步内累积来自LIF神经元层的输出脉冲，
// 并在所有时间步完成后，根据累积的脉冲数确定预测的类别。
// 计数器在决策输出后立即清零。

module output_logic #(
    parameter P_NUM_OUTPUT_NEURONS = 10,  // 输出神经元的数量
    parameter P_T_MAX              = 100  // SNN总时间步数 (用于确定计数器位宽)
) (
    input wire                               clk,
    input wire                               rst_n,

    // 控制信号 (来自 control_unit.v)
    input wire                               i_accum_en,                 // 单周期脉冲，使能当前时间步的脉冲累加，由 control 和 i_lif_spikes_valid 共同协商得出的信号
    input wire                               i_decision_en,              // 单周期脉冲，使能最终决策逻辑，也即在所有时间步计算后准备找出最大值
    input wire [$clog2(P_T_MAX)-1:0]         i_current_time_step,         // 当前时间步编号，用于计算剩余时间步

    // 来自LIF神经元层的数据 (lif_neuron_layer.v)  
    input wire [P_NUM_OUTPUT_NEURONS-1:0]    i_lif_spike_vector,         // LIF层的脉冲输出向量

    // 输出结果
    output reg [$clog2(P_NUM_OUTPUT_NEURONS)-1:0] o_predicted_label,      // 预测的类别标签
    output reg [$clog2(P_T_MAX + 1)-1:0]       o_max_spike_count,      // 最终的预测结果对应的脉冲计数值
    output reg [$clog2(P_T_MAX + 1)-1:0]       o_second_max_spike_count, // 当前第二大脉冲计数值，用于观察提前停止判断
    output reg                                   o_prediction_valid,     // 最终经过所有时间步的结果，指示 o_predicted_label 和 o_max_spike_count 是否有效 (单周期脉冲)
    output reg                                   o_early_stop            // 当前领先优势已经无法被剩余时间步反超时拉高
);

    // 内部常量定义
    // 脉冲计数器的位宽，需要能存下 P_T_MAX (因为一个神经元最多每步发一个脉冲)
    // 例如 P_T_MAX=100, $clog2(100+1)=7 (可以计数0到100)
    localparam LP_SPIKE_COUNTER_BITS = $clog2(P_T_MAX + 1); 
    // 预测标签的位宽
    localparam LP_LABEL_WIDTH        = $clog2(P_NUM_OUTPUT_NEURONS);

    // 内部寄存器和组合信号
    reg [LP_SPIKE_COUNTER_BITS-1:0] spike_counters_reg [P_NUM_OUTPUT_NEURONS-1:0]; // 每个输出神经元的脉冲计数值
    reg [LP_SPIKE_COUNTER_BITS-1:0] next_spike_counters_comb [P_NUM_OUTPUT_NEURONS-1:0]; // 当前时间步累加后的临时计数值
    reg [LP_SPIKE_COUNTER_BITS-1:0] max_count_comb;
    reg [LP_SPIKE_COUNTER_BITS-1:0] second_count_comb;
    reg [LP_LABEL_WIDTH-1:0]        max_index_comb;
    reg [LP_SPIKE_COUNTER_BITS-1:0] remaining_steps_comb;
    reg [LP_SPIKE_COUNTER_BITS-1:0] lead_margin_comb;
    reg early_stop_comb;

    integer accum_idx;
    integer max_idx;
    integer counter_idx;

    /*
     * 组合逻辑：预先算出“如果本拍累加有效”，每个计数器的新值。
     * 这里只计算，不真正保存状态。
     */
    always @(*) begin
        for (accum_idx = 0; accum_idx < P_NUM_OUTPUT_NEURONS; accum_idx = accum_idx + 1) begin
            next_spike_counters_comb[accum_idx] = spike_counters_reg[accum_idx];
            if (i_accum_en && i_lif_spike_vector[accum_idx]) begin
                next_spike_counters_comb[accum_idx] = spike_counters_reg[accum_idx] + 1'b1;
            end
        end
    end

    /*
     * 组合逻辑：基于最新计数值寻找第一名、第二名，并计算提前停止条件。
     * i_accum_en 有效时使用累加后的临时值，否则使用当前寄存器值。
     */
    always @(*) begin
        max_count_comb = i_accum_en ? next_spike_counters_comb[0] : spike_counters_reg[0];
        second_count_comb = {LP_SPIKE_COUNTER_BITS{1'b0}};
        max_index_comb = {LP_LABEL_WIDTH{1'b0}};

        for (max_idx = 1; max_idx < P_NUM_OUTPUT_NEURONS; max_idx = max_idx + 1) begin
            if ((i_accum_en ? next_spike_counters_comb[max_idx] : spike_counters_reg[max_idx]) > max_count_comb) begin
                second_count_comb = max_count_comb;
                max_count_comb = i_accum_en ? next_spike_counters_comb[max_idx] : spike_counters_reg[max_idx];
                max_index_comb = max_idx;
            end else if ((i_accum_en ? next_spike_counters_comb[max_idx] : spike_counters_reg[max_idx]) > second_count_comb) begin
                second_count_comb = i_accum_en ? next_spike_counters_comb[max_idx] : spike_counters_reg[max_idx];
            end
        end

        /*
         * 如果第一名领先第二名的数量，大于剩余时间步数，
         * 那么第二名即使之后每个时间步都放电也追不上第一名。
         */
        remaining_steps_comb = (P_T_MAX - 1) - i_current_time_step;
        lead_margin_comb = max_count_comb - second_count_comb;
        early_stop_comb = (lead_margin_comb > remaining_steps_comb);
    end

    /*
     * 时序逻辑：只负责保存和清零脉冲计数器。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (counter_idx = 0; counter_idx < P_NUM_OUTPUT_NEURONS; counter_idx = counter_idx + 1) begin
                spike_counters_reg[counter_idx] <= {LP_SPIKE_COUNTER_BITS{1'b0}};
            end
        end else if (i_decision_en) begin
            for (counter_idx = 0; counter_idx < P_NUM_OUTPUT_NEURONS; counter_idx = counter_idx + 1) begin
                spike_counters_reg[counter_idx] <= {LP_SPIKE_COUNTER_BITS{1'b0}};
            end
        end else if (i_accum_en) begin
            for (counter_idx = 0; counter_idx < P_NUM_OUTPUT_NEURONS; counter_idx = counter_idx + 1) begin
                spike_counters_reg[counter_idx] <= next_spike_counters_comb[counter_idx];
            end
        end
    end

    /*
     * 时序逻辑：只负责输出当前第一名、第二名和提前停止观察信号。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_predicted_label      <= {LP_LABEL_WIDTH{1'b0}};
            o_max_spike_count      <= {LP_SPIKE_COUNTER_BITS{1'b0}};
            o_second_max_spike_count <= {LP_SPIKE_COUNTER_BITS{1'b0}};
            o_early_stop           <= 1'b0;
        end else begin
            if (i_accum_en || i_decision_en) begin
                o_predicted_label <= max_index_comb;
                o_max_spike_count <= max_count_comb;
                o_second_max_spike_count <= second_count_comb;
            end

            if (i_decision_en) begin
                o_early_stop           <= 1'b0;
            end else if (i_accum_en) begin
                o_early_stop           <= early_stop_comb;
            end
        end
    end

    /*
     * 时序逻辑：只负责最终预测有效脉冲。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_prediction_valid <= 1'b0;
        end else begin
            o_prediction_valid <= i_decision_en;
        end
    end

endmodule
