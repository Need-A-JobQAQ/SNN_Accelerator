// lif_neuron_layer.v
// 实现 SNN 的 LIF 神经元层。
// 它例化 P_NUM_OUTPUT_NEURONS 个 lif_neuron_cell 模块，
// 每个模块负责一个输出神经元的动力学计算。

module lif_neuron_layer #(
    // --- 用户可配置的逻辑参数 ---
    parameter P_NUM_OUTPUT_NEURONS        = 10,  // 该层包含的神经元数量
    parameter P_NEURON_VALUE_TOTAL_BITS   = 26,  // 神经元膜电位v和输入电流I的总位宽
    parameter P_NEURON_VALUE_FRAC_BITS = 12   // 神经元膜电位v和输入电流I的小数部分位宽
) (
    input wire                                                                    clk,                // 时钟信号
    input wire                                                                    rst_n,              // 异步低电平复位信号
    input wire                                                                    i_enable_layer,     // 通用使能信号，用于层内所有LIF神经元，这个使能是由 countrul 模块和 linear_layer 共同得出的结果
    input wire signed [P_NUM_OUTPUT_NEURONS-1:0] [P_NEURON_VALUE_TOTAL_BITS-1:0]  i_all_currents_I,   // 输入电流向量：P_NUM_OUTPUT_NEURONS 个电流值，

    output wire [P_NUM_OUTPUT_NEURONS-1:0]                                        o_all_spikes_out,   // 输出脉冲向量：P_NUM_OUTPUT_NEURONS 位，每位代表一个神经元的脉冲输出 (0或1)
    output wire                                                                   o_all_spikes_valid  // 指示 o_all_spikes_out 是否有效
);

    // 内部信号，用于连接到每个 lif_neuron_cell 实例的输出
    wire [P_NUM_OUTPUT_NEURONS-1:0]       neuron_cell_spike_outs_w;
    wire [P_NUM_OUTPUT_NEURONS-1:0]       neuron_cell_spike_valids_w;

    genvar n_idx, i_assign_spike;

    // 实例化 P_NUM_OUTPUT_NEURONS 个 lif_neuron_cell 模块
    generate
        for (n_idx = 0; n_idx < P_NUM_OUTPUT_NEURONS; n_idx = n_idx + 1) begin : gen_lif_cells_in_layer
            
            lif_neuron_cell #(
                .P_NEURON_VALUE_TOTAL_BITS      (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS (P_NEURON_VALUE_FRAC_BITS)
            ) u_lif_cell_inst (
                .clk                        (clk),
                .rst_n                      (rst_n),
                .i_enable_neuron            (i_enable_layer),                    // 所有单元共享此层的使能信号
                .i_input_current_I          (i_all_currents_I[n_idx]),           // 为每个LIF单元传递其特定的输入电流值

                .o_spike_out                (neuron_cell_spike_outs_w[n_idx]),   // 收集每个单元的脉冲输出
                .o_spike_valid              (neuron_cell_spike_valids_w[n_idx])  // 收集每个单元的脉冲有效信号
            );
            
        end
    endgenerate

    // 将内部收集到的脉冲输出逐个连接到模块的输出端口
    generate
        for (i_assign_spike = 0; i_assign_spike < P_NUM_OUTPUT_NEURONS; i_assign_spike = i_assign_spike + 1) begin : gen_assign_output_spikes
            assign o_all_spikes_out[i_assign_spike] = neuron_cell_spike_outs_w[i_assign_spike];
        end
    endgenerate

    // 假设所有 lif_neuron_cell 实例是同步操作的，它们的 o_spike_valid 输出将同时有效或无效。
    // 因此，可以取其中任何一个实例 (例如第一个，索引为0) 的 o_spike_valid 信号作为整个神经元层的输出有效信号 o_all_spikes_valid。
    // 或者，如果需要更严格的同步保证（虽然在此设计中，它们应该是同步的）：
    // assign o_all_spikes_valid = &neuron_cell_spike_valids_w; // 仅当所有 neuron_cell_spike_valids_w 位都为1时，结果才为1
    assign o_all_spikes_valid = neuron_cell_spike_valids_w[0]; 


endmodule