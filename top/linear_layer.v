// linear_layer.v
// 实现SNN的线性层。
// 除了二维数据的使用外，其他的语句均为 Verilog-2001 规范编写，在 Vivado 中仍需要将文件设置为 system verilog 格式才可以正常运行，若要使用标准 verilog 需要对二维数组部分进行重构
// 它例化 P_NUM_OUTPUT_NEURONS 个独立的 BRAM ROM IP核 (weights_0, weights_1, ... ,weights_9)，
// 以及 P_NUM_OUTPUT_NEURONS 个通用的 neuron_processing_unit 模块。
// 每个 neuron_processing_unit 连接其专属的BRAM ROM。

module linear_layer #(
    // --- 用户可配置的逻辑参数 ---
    parameter P_NUM_INPUT_PIXELS        = 784,
    parameter P_WEIGHT_BIT_WIDTH        = 16,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter P_NEURON_VALUE_FRAC_BITS  = 12,
    parameter P_BRAM_DATA_WIDTH         = 64,  // BRAM IP核的数据输出端口位宽
    parameter P_BRAM_ADDR_WIDTH         = 8,   // BRAM IP核的地址端口位宽
    parameter P_BRAM_EFFECTIVE_DEPTH    = 196, // BRAM IP核的有效存储字数
    parameter P_NUM_OUTPUT_NEURONS      = 10   // 输出神经元数量
) (
    input wire                                   clk,
    input wire                                   rst_n,
    input wire                                   i_calc_start,           // 全局启动信号
    input wire  [P_NUM_INPUT_PIXELS-1:0]         i_input_spike_vector,   // 全局输入脉冲向量
    
    // 注意不可以写为 output wire signed [P_NUM_OUTPUT_NEURONS-1:0] o_all_currents_I [P_NEURON_VALUE_TOTAL_BITS-1:0]，这个写法只限于模块内部变量，不能直接用于 端口定义
    // SystemVerilog 中，端口声明中不允许将数据类型放在 右侧维度上，在模块端口声明中，只能用前向维度（packed），不能用后向维度（unpacked）
    // SystemVerilog 允许“多维 packed array”，即：logic [dim1][dim0] my_var; 所有维度都写在 type 的左边，全部维度是 packed 的（也就是说，它们组合起来被视为一个线性比特向量，只是逻辑上是二维的）
    // 而“unpacked array”才是不允许作为端口的，例如 output wire signed [15:0] o_all_currents_I [9:0];这个写法是：先声明每个元素是 16 位，再定义一个 unpacked 数组，长度为 10。SystemVerilog 不允许这样的“unpacked array”作为端口
    // 原因如下：端口需要与外部模块进行“线对线绑定，端口必须被综合成电路中的“导线”，而导线必须是确定的一维位向量，如果允许 unpacked array 出现在端口中，就会让绑定方式变得模糊或无法综合，工具不知道该如何将其连接成一组总线或如何打包成单个向量
    // 例如 output logic [1:0] o_data [3:0]; 它可以被理解为：4 个 2 位信号，也可能想要的是一个 8 位总线 [7:0]
    // 当你在端口声明中将所有维度都放在信号名称之前时，其解释顺序通常是从左到右，左边的维度是较高层级的数组维度，右边的维度是较低层级的元素位宽或更内部的数组维度。
    // 最左边的维度通常被视为主数组维度（有多少个元素），而它右边的维度则描述了每个元素的构成（例如元素的位宽，或者如果元素本身还是数组，则是更内层的数组维度）。
    output wire signed [P_NUM_OUTPUT_NEURONS-1:0] [P_NEURON_VALUE_TOTAL_BITS-1:0] o_all_currents_I,  // 输出：所有神经元的计算结果
    output wire                                   o_all_currents_valid                              // 输出有效信号：当所有神经元的电流都计算完毕时有效
);

    // 内部信号，用于连接 neuron_processing_unit 实例的输出
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] neuron_current_out_wires   [P_NUM_OUTPUT_NEURONS-1:0];
    wire                                       neuron_current_valid_wires [P_NUM_OUTPUT_NEURONS-1:0];

    genvar n_idx;

    // ------------------- 例化 P_NUM_OUTPUT_NEURONS 个神经元处理链路 -------------------
    // 每个链路包含一个专属的 BRAM ROM IP核 和一个 neuron_processing_unit
    generate
        for (n_idx = 0; n_idx < P_NUM_OUTPUT_NEURONS; n_idx = n_idx + 1) begin : gen_neuron_processing_chains
            
            // 内部信号，用于连接第 n_idx 个 BRAM IP核和第 n_idx 个 neuron_processing_unit
            wire [P_BRAM_ADDR_WIDTH-1:0]           bram_addr_for_npu;
            wire                                   bram_ena_for_npu;
            wire signed [P_BRAM_DATA_WIDTH-1:0]    bram_dout_to_npu;

            // --- 例化第 n_idx 个BRAM ROM IP核 ---
            if (n_idx == 0) begin : gen_bram_0
                weights_0 u_bram_neuron_0 (
                    .clka   (clk),
                    .ena    (bram_ena_for_npu),
                    .addra  (bram_addr_for_npu),
                    .douta  (bram_dout_to_npu)
                );
            end else if (n_idx == 1) begin : gen_bram_1
                weights_1 u_bram_neuron_1 (
                    .clka   (clk),
                    .ena    (bram_ena_for_npu),
                    .addra  (bram_addr_for_npu),
                    .douta  (bram_dout_to_npu)
                );
            end else if (n_idx == 2) begin : gen_bram_2
                weights_2 u_bram_neuron_2 (
                    .clka   (clk),
                    .ena    (bram_ena_for_npu),
                    .addra  (bram_addr_for_npu),
                    .douta  (bram_dout_to_npu)
                );
            end else if (n_idx == 3) begin : gen_bram_3
                weights_3 u_bram_neuron_3 (
                    .clka   (clk),
                    .ena    (bram_ena_for_npu),
                    .addra  (bram_addr_for_npu),
                    .douta  (bram_dout_to_npu)
                );
            end else if (n_idx == 4) begin : gen_bram_4
                weights_4 u_bram_neuron_4 (
                    .clka   (clk),
                    .ena    (bram_ena_for_npu),
                    .addra  (bram_addr_for_npu),
                    .douta  (bram_dout_to_npu)
                );
            end else if (n_idx == 5) begin : gen_bram_5
                weights_5 u_bram_neuron_5 (
                    .clka   (clk),
                    .ena    (bram_ena_for_npu),
                    .addra  (bram_addr_for_npu),
                    .douta  (bram_dout_to_npu)
                );
            end else if (n_idx == 6) begin : gen_bram_6
                weights_6 u_bram_neuron_6 (
                    .clka   (clk),
                    .ena    (bram_ena_for_npu),
                    .addra  (bram_addr_for_npu),
                    .douta  (bram_dout_to_npu)
                );
            end else if (n_idx == 7) begin : gen_bram_7
                weights_7 u_bram_neuron_7 (
                    .clka   (clk),
                    .ena    (bram_ena_for_npu),
                    .addra  (bram_addr_for_npu),
                    .douta  (bram_dout_to_npu)
                );
            end else if (n_idx == 8) begin : gen_bram_8
                weights_8 u_bram_neuron_8 (
                    .clka   (clk),
                    .ena    (bram_ena_for_npu),
                    .addra  (bram_addr_for_npu),
                    .douta  (bram_dout_to_npu)
                );
            end else if (n_idx == 9) begin : gen_bram_9 
                weights_9 u_bram_neuron_9 ( 
                    .clka   (clk),
                    .ena    (bram_ena_for_npu),
                    .addra  (bram_addr_for_npu),
                    .douta  (bram_dout_to_npu)
                );
            end

            // --- 例化第 n_idx 个 neuron_processing_unit ---
            neuron_processing_unit #(
                .P_NUM_INPUT_PIXELS       (P_NUM_INPUT_PIXELS),
                .P_WEIGHT_BIT_WIDTH       (P_WEIGHT_BIT_WIDTH),
                .P_NEURON_VALUE_TOTAL_BITS (P_NEURON_VALUE_TOTAL_BITS),
                .P_NEURON_VALUE_FRAC_BITS  (P_NEURON_VALUE_FRAC_BITS),
                .P_BRAM_DATA_WIDTH        (P_BRAM_DATA_WIDTH),
                .P_BRAM_ADDR_WIDTH        (P_BRAM_ADDR_WIDTH),
                .P_BRAM_EFFECTIVE_DEPTH   (P_BRAM_EFFECTIVE_DEPTH)
            ) npu_instance (
                .clk                    (clk),
                .rst_n                  (rst_n),
                .i_calc_start           (i_calc_start),              // 所有NPU同时启动
                .i_input_spike_vector   (i_input_spike_vector),      // 所有NPU接收相同的脉冲输入
                
                .i_bram_dout_raw        (bram_dout_to_npu),          // 连接到对应BRAM的数据输出
                .o_bram_addr_to_issue   (bram_addr_for_npu),         // 控制对应BRAM的地址
                .o_bram_ena             (bram_ena_for_npu),          // 控制对应BRAM的使能
                
                .o_output_current_I     (neuron_current_out_wires[n_idx]),
                .o_current_valid_out    (neuron_current_valid_wires[n_idx])
            );

        end
    endgenerate

    // 将各个神经元处理单元的输出电流连接到模块的输出端口
    // assign o_all_currents_I = neuron_current_out_wires;
    genvar i_assign; // 确保这个 genvar 名称是唯一的，并且在模块开始处声明过 (例如 genvar n_idx, i_assign;)
    generate
        for (i_assign = 0; i_assign < P_NUM_OUTPUT_NEURONS; i_assign = i_assign + 1) begin : gen_assign_output_currents
            assign o_all_currents_I[i_assign] = neuron_current_out_wires[i_assign];
        end
    endgenerate

    // o_all_currents_valid 的逻辑：
    // 假设所有 neuron_processing_unit 实例会同步地完成计算并发出单周期有效脉冲。
    // 因此，可以取第一个实例的有效信号作为整个层的有效信号。
    // 如果需要更严格的同步保证，可以对所有的 neuron_current_valid_wires 进行逻辑与。
    assign o_all_currents_valid = neuron_current_valid_wires[0]; 

endmodule