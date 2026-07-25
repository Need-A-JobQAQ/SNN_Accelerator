// lif_neuron_cell.v
// 这个模块是构成LIF神经元层的最基本的功能单元�?�它完整地实现了单个LIF（漏积分发放）神经元的数学模型和行为逻辑�?
// 实现单个漏积分发�? (Leaky Integrate-and-Fire, LIF) 神经元的功能�?
// 假设时间常数 tau = 2.0
// 神经元处理公式：- v[t] 项代表了膜电位的泄漏，I[t] 项代表神经元得到它连接的�?有的权重的脉冲汇�?
// v[t+1] = v[t] + (I[t] - v[t]) / tau
// tau (膜电位时间常�?): 2.0，v_threshold (阈�?�电�?): 1.0，v_reset (重置电压): 0.0，decay_input (输入是否参与衰减): True      

module lif_neuron_cell #(
    // --- 用户可配置的逻辑参数 ---
    // 这些参数将在模块被例化时由上�?级模块（lif_neuron_layer.v）提供，并应与驱动此模块的电流源的格式匹配�??
    parameter P_NEURON_VALUE_TOTAL_BITS      = 26,  // LIF神经元膜电位v和输入电流I的�?�位�?
    parameter P_NEURON_VALUE_FRAC_BITS = 12   // LIF神经元膜电位v和输入电流I的小数部分位�?
) (
    input wire                                    clk,                    // 时钟信号 (移除了i_前缀)
    input wire                                    rst_n,                  // 异步低电平复位信�? (移除了i_前缀)
    input wire                                    i_enable_neuron,        // 此神经元单元的使能信�?
    input wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]  i_input_current_I,      // 输入电流 I[t] (定点数表�?)

    output reg                                    o_spike_out,            // 输出脉冲 (0 �? 1)，单周期脉冲行为
    output reg                                    o_spike_valid           // 指示 spike_out 是否有效的信�?
);  

    // --- 内部固定参数 (根据输入参数计算) ---
    // 阈�?�电�? V_THRESHOLD = 1.0
    // 这段代码的目的是安全且正确地将浮点数阈�??1.0转换为一个�?�合硬件使用的定点数表示
    // �? Q(P_NEURON_VALUE_TOTAL_BITS - P_NEURON_VALUE_FRAC_BITS).(P_NEURON_VALUE_FRAC_BITS) 格式表示，即与输入电流的格式保持�?致从而可以进行判�?
    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] LP_V_THRESHOLD_FIXED = 
        (P_NEURON_VALUE_FRAC_BITS >= 0 && P_NEURON_VALUE_FRAC_BITS < P_NEURON_VALUE_TOTAL_BITS) ? 
        (1'b1 << P_NEURON_VALUE_FRAC_BITS) : 
        {{25{1'b0}},1'b1};
    // 复位电压 V_RESET = 0.0
    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] LP_V_RESET_FIXED     = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
    
    // 时间常数 TAU = 2.0 (硬编码，因为除以2是简单的右移)

    // 内部寄存器，用于存储膜电�? 'v'
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] membrane_potential_v_reg;

    // 用于计算的中间信�? (组合逻辑)
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] diff_I_v_comb;             
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] delta_v_component_comb;    
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] v_candidate_potential_comb; 

    // --- 组合逻辑：更新膜电位的计�? ---
    assign diff_I_v_comb = i_input_current_I - membrane_potential_v_reg;
    assign delta_v_component_comb = diff_I_v_comb >>> 1; 
    assign v_candidate_potential_comb = membrane_potential_v_reg + delta_v_component_comb;

    // --- 时序逻辑：脉冲产生和电位复位/更新 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin 
            membrane_potential_v_reg   <= LP_V_RESET_FIXED;    
            o_spike_out                <= 1'b0;               
            o_spike_valid              <= 1'b0;               
        end else begin
            o_spike_valid <= 1'b0; 

            if (i_enable_neuron) begin 
                o_spike_valid <= 1'b1; 

                if (v_candidate_potential_comb >= LP_V_THRESHOLD_FIXED) begin 
                    o_spike_out                <= 1'b1; 
                    membrane_potential_v_reg   <= LP_V_RESET_FIXED;    
                end else begin 
                    o_spike_out   <= 1'b0; 
                    membrane_potential_v_reg   <= v_candidate_potential_comb; 
                end
            end
            // 如果 i_enable_neuron 为低�?
            // membrane_potential_v_reg 保持其先前的�? (标准寄存器行�?)�?
            // o_spike_out 会因为本周期�?始时�? o_spike_out <= 1'b0; 而为0�?
            // o_spike_valid 也会�? 1'b0�?
        end
    end

    // 移除了调试端�? o_membrane_potential_v_debug 及其 assign 语句

endmodule