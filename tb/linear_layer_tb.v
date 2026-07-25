// linear_layer_tb.v
// (中文注释版 - 修正后)
// linear_layer 模块的测试平台

`timescale 1ns/1ps

// 如果 linear_layer.v 或其子模块 neuron_processing_unit.v 依赖于参数的默认值，
// 或者你想在测试平台中明确传递参数，可以这样做。
// 通常，测试平台会根据DUT的参数来声明连接信号。

module linear_layer_tb;

    // --- 测试平台参数 ---
    localparam CLK_PERIOD = 10; // ns, 时钟周期 (100MHz)

    // DUT 参数 (这些值应与 linear_layer.v 及其子模块期望的一致)
    localparam P_NUM_INPUT_PIXELS       = 784;
    localparam P_WEIGHT_BIT_WIDTH       = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_NEURON_VALUE_FRAC_BITS  = 12;
    localparam P_BRAM_DATA_WIDTH        = 64;
    localparam P_BRAM_ADDR_WIDTH        = 8;
    localparam P_BRAM_EFFECTIVE_DEPTH   = 196;
    localparam P_NUM_OUTPUT_NEURONS     = 10;

    // 信号声明 - 连接到 DUT 的输入
    reg                                   tb_clk;
    reg                                   tb_rst_n;
    reg                                   tb_i_calc_start;
    reg  [P_NUM_INPUT_PIXELS-1:0]         tb_i_input_spike_vector;
    
    // 信号声明 - 连接到 DUT 的输出
    wire signed [P_NUM_OUTPUT_NEURONS-1:0]
                [P_NEURON_VALUE_TOTAL_BITS-1:0] tb_o_all_currents_I;
    wire                                   tb_o_all_currents_valid;

    // --- DUT (linear_layer) 实例化 ---
    linear_layer #(
        .P_NUM_INPUT_PIXELS       (P_NUM_INPUT_PIXELS),
        .P_WEIGHT_BIT_WIDTH       (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS  (P_NEURON_VALUE_FRAC_BITS),
        .P_BRAM_DATA_WIDTH        (P_BRAM_DATA_WIDTH),
        .P_BRAM_ADDR_WIDTH        (P_BRAM_ADDR_WIDTH),
        .P_BRAM_EFFECTIVE_DEPTH   (P_BRAM_EFFECTIVE_DEPTH),
        .P_NUM_OUTPUT_NEURONS     (P_NUM_OUTPUT_NEURONS)
    ) u_linear_layer_inst (
        .clk                    (tb_clk),
        .rst_n                  (tb_rst_n),
        .i_calc_start           (tb_i_calc_start),
        .i_input_spike_vector   (tb_i_input_spike_vector),
        .o_all_currents_I       (tb_o_all_currents_I),
        .o_all_currents_valid   (tb_o_all_currents_valid)
    );

    // 时钟生成
    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    // 激励和检查
    initial begin
        $display("[%0t ns] SIM_INFO: linear_layer_tb 开始仿真。", $time);

        // 初始化和复位
        tb_i_calc_start = 1'b0;
        tb_i_input_spike_vector = {P_NUM_INPUT_PIXELS{1'b0}}; // 初始全0脉冲
        tb_rst_n = 1'b0; // 激活复位
        repeat(5) @(posedge tb_clk);
        tb_rst_n = 1'b1; // 释放复位
        $display("[%0t ns] SIM_INFO: 复位已释放.", $time);
        repeat(2) @(posedge tb_clk);

        // --- 第一次计算 ---
        // 设置一个简单的脉冲输入图案
        // 例如，只让前8个输入脉冲有效，以测试至少两个BRAM字（每个含4个权重）的读取
        tb_i_input_spike_vector = {P_NUM_INPUT_PIXELS{1'b0}}; // 先全0
        for (integer i = 0; i < 8; i = i + 1) begin
            tb_i_input_spike_vector[P_NUM_INPUT_PIXELS-1 - i] = 1'b1; 
        end
        $display("[%0t ns] SIM_INFO: 设置输入脉冲向量 (前8个脉冲激活).", $time);

        $display("[%0t ns] SIM_INFO: 发起计算请求 (i_calc_start = 1).", $time);
        tb_i_calc_start = 1'b1;
        @(posedge tb_clk);
        tb_i_calc_start = 1'b0; // i_calc_start 是单周期脉冲

        // 等待计算完成
        // neuron_processing_unit 的大致周期数是 P_BRAM_EFFECTIVE_DEPTH + BRAM_READ_LATENCY + FSM额外周期
        // 例如: 196 (处理字数) + 2 (BRAM延迟) + 1 (DONE状态) + 1 (IDLE->PROC) ~ 200 个周期
        // linear_layer 的 o_all_currents_valid 是基于 neuron_current_valid_wires[0]
        $display("[%0t ns] SIM_INFO: 等待计算完成 (tb_o_all_currents_valid)...", $time);
        wait (tb_o_all_currents_valid == 1'b1);
        $display("[%0t ns] SIM_INFO: 计算完成! ", $time);

        // 打印所有神经元的输出电流
        for (integer n_out = 0; n_out < P_NUM_OUTPUT_NEURONS; n_out = n_out + 1) begin
            $display("SIM_INFO: Neuron %0d 输出电流: %d (十六进制: %h)", 
                     n_out, tb_o_all_currents_I[n_out], tb_o_all_currents_I[n_out]);
        end
        
        // **期望值验证**: 
        // 在这里进行精确的期望值验证会非常复杂，因为它需要：
        // 1. 访问所有10个BRAM IP核 (`weights_0` 到 `weights_9`) 的 `.coe` 文件内容。
        // 2. 根据 `tb_i_input_spike_vector` 和每个神经元的权重，手动计算出10个期望的累加和。
        // 3. 考虑到 `neuron_processing_unit` 内部的流水线和数据对齐逻辑。
        // 对于这个基础测试平台，我们主要关注模块是否能运行并通过 `tb_o_all_currents_valid` 发出完成信号。
        // 更深入的验证通常结合Python参考模型和更复杂的脚本。

        @(posedge tb_clk); // 等待 valid 信号拉低

        repeat(10) @(posedge tb_clk);
        $display("[%0t ns] SIM_INFO: linear_layer_tb 仿真结束。", $time);
        $finish;
    end

    // (可选) 监控关键信号
    initial begin
        // 从 $monitor 中移除了 tb_o_snn_busy，因为它不是 linear_layer 的输出
        $monitor("[%0t ns] TB_MONITOR: clk=%b, rst_n=%b, start=%b, valid(linear)=%b, N0_current=%d",
                 $time, tb_clk, tb_rst_n, tb_i_calc_start,
                 tb_o_all_currents_valid, tb_o_all_currents_I[0]);
    end

endmodule