// lif_neuron_layer_tb.v
// (中文注释版)
// lif_neuron_layer 模块的测试平台

`timescale 1ns/1ps

module lif_neuron_layer_tb;

    // --- 测试平台参数 ---
    localparam CLK_PERIOD = 10; // ns, 时钟周期 (100MHz)

    // DUT 参数 (这些值应与 lif_neuron_layer.v 及其子模块 lif_neuron_cell.v 期望的一致)
    localparam P_NUM_OUTPUT_NEURONS      = 10;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26; // 对应 DUT 的 P_LAYER_V_I_TOTAL_BITS 或 lif_neuron_cell 的 P_LIF_V_I_TOTAL_BITS
    localparam P_NEURON_VALUE_FRAC_BITS  = 12; // 对应 DUT 的 P_LAYER_V_I_FRACTIONAL_BITS 或 lif_neuron_cell 的 P_LIF_V_I_FRACTIONAL_BITS

    // 信号声明 - 连接到 DUT 的输入
    reg                                   tb_clk;
    reg                                   tb_rst_n;
    reg                                   tb_i_enable_layer;
    reg                                   tb_i_currents_valid;
    reg signed [P_NUM_OUTPUT_NEURONS-1:0] 
               [P_NEURON_VALUE_TOTAL_BITS-1:0] tb_i_all_currents_I;
    
    // 信号声明 - 连接到 DUT 的输出
    wire [P_NUM_OUTPUT_NEURONS-1:0]        tb_o_all_spikes_out;
    wire                                   tb_o_all_spikes_valid;

    // --- DUT (lif_neuron_layer) 实例化 ---
    lif_neuron_layer #(
        .P_NUM_OUTPUT_NEURONS      (P_NUM_OUTPUT_NEURONS),
        .P_LAYER_V_I_TOTAL_BITS    (P_NEURON_VALUE_TOTAL_BITS),    // 将TB参数传递给DUT的层级参数
        .P_LAYER_V_I_FRACTIONAL_BITS (P_NEURON_VALUE_FRAC_BITS) // 这些参数在DUT内部会进一步传递给lif_neuron_cell
    ) u_lif_layer_inst (
        .clk                    (tb_clk),
        .rst_n                  (tb_rst_n),
        .i_enable_layer         (tb_i_enable_layer),
        .i_currents_valid       (tb_i_currents_valid),
        .i_all_currents_I       (tb_i_all_currents_I),
        .o_all_spikes_out       (tb_o_all_spikes_out),
        .o_all_spikes_valid     (tb_o_all_spikes_valid)
    );

    // 时钟生成
    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    // 激励和检查
    initial begin
        integer i;
        $display("[%0t ns] SIM_INFO: lif_neuron_layer_tb 开始仿真。", $time);

        // 初始化和复位
        tb_i_enable_layer     = 1'b0;
        tb_i_currents_valid   = 1'b0;
        tb_i_all_currents_I   = {P_NUM_OUTPUT_NEURONS{{P_NEURON_VALUE_TOTAL_BITS{1'b0}}}}; // 所有电流初始为0
        tb_rst_n = 1'b0; // 激活复位
        repeat(5) @(posedge tb_clk);
        tb_rst_n = 1'b1; // 释放复位
        $display("[%0t ns] SIM_INFO: 复位已释放.", $time);
        repeat(2) @(posedge tb_clk);

        // --- 测试序列 ---
        // 1. 提供一组输入电流，并使能神经元层
        tb_i_enable_layer   = 1'b1;
        tb_i_currents_valid = 1'b1;

        // 为不同的神经元设置不同的电流值，以观察不同的脉冲行为
        // 阈值 LP_V_THRESHOLD_FIXED = (1 << P_NEURON_VALUE_FRAC_BITS) 
        // 例如，如果 P_NEURON_VALUE_FRAC_BITS = 12, 阈值是 4096
        // 电流 I, v[t+1] = v[t] + (I - v[t]) / 2
        // 如果 v[t]=0, I > 2*阈值，则第一次就会发放脉冲
        // 如果 v[t]=0, I = 1.5*阈值, v[1]=0.75*阈值, 第二次若I不变, v[2]=0.75*阈值 + (1.5-0.75)*阈值/2 = 0.75*阈值 + 0.375*阈值 = 1.125*阈值 -> 发放
        
        // 神经元0: 给一个较大的电流，使其立即发放脉冲
        // 2.5 * 4096 = 10240. (Q14.12格式)
        if (P_NUM_OUTPUT_NEURONS > 0)
            tb_i_all_currents_I[0] = signed'(10240); 

        // 神经元1: 给一个中等电流，使其在几个周期后发放脉冲
        // 0.8 * 4096 = 3276 (近似值).
        if (P_NUM_OUTPUT_NEURONS > 1)
            tb_i_all_currents_I[1] = signed'(3276); 

        // 神经元2: 给一个较小的正电流，可能不发放或很久才发放
        // 0.2 * 4096 = 819
        if (P_NUM_OUTPUT_NEURONS > 2)
            tb_i_all_currents_I[2] = signed'(819);
            
        // 神经元3: 给一个负电流 (抑制)
        // -0.5 * 4096 = -2048
        if (P_NUM_OUTPUT_NEURONS > 3)
            tb_i_all_currents_I[3] = signed'(-2048);

        $display("[%0t ns] SIM_INFO: 设置输入电流并使能层。N0_I=%d, N1_I=%d, N2_I=%d, N3_I=%d", 
            $time, tb_i_all_currents_I[0], tb_i_all_currents_I[1], tb_i_all_currents_I[2], tb_i_all_currents_I[3]);
        
        // 观察几个周期的输出
        for (i = 0; i < 10; i = i + 1) begin
            @(posedge tb_clk);
            if (tb_o_all_spikes_valid) begin
                $display("[%0t ns] SIM_INFO: Spikes Valid! Output Spikes = %b", 
                         $time, tb_o_all_spikes_out);
            end else begin
                $display("[%0t ns] SIM_INFO: Spikes Not Valid. Output Spikes = %b (stale)", 
                         $time, tb_o_all_spikes_out);
            end
        end

        // 2. 撤销使能或有效信号，观察行为
        tb_i_currents_valid = 1'b0;
        $display("[%0t ns] SIM_INFO: 输入电流无效 (i_currents_valid = 0).", $time);
        for (i = 0; i < 3; i = i + 1) begin
            @(posedge tb_clk);
            if (tb_o_all_spikes_valid) begin // 理论上此时 valid 应该为0
                $display("[%0t ns] SIM_INFO: Spikes Valid! Output Spikes = %b", 
                         $time, tb_o_all_spikes_out);
            end else begin
                $display("[%0t ns] SIM_INFO: Spikes Not Valid. Output Spikes = %b (stale)", 
                         $time, tb_o_all_spikes_out);
            end
        end

        tb_i_enable_layer = 1'b0;
        $display("[%0t ns] SIM_INFO: 层使能关闭 (i_enable_layer = 0).", $time);
        for (i = 0; i < 3; i = i + 1) begin
            @(posedge tb_clk);
             if (tb_o_all_spikes_valid) begin // 理论上此时 valid 应该为0
                $display("[%0t ns] SIM_INFO: Spikes Valid! Output Spikes = %b", 
                         $time, tb_o_all_spikes_out);
            end else begin
                $display("[%0t ns] SIM_INFO: Spikes Not Valid. Output Spikes = %b (stale)", 
                         $time, tb_o_all_spikes_out);
            end
        end

        repeat(5) @(posedge tb_clk);
        $display("[%0t ns] SIM_INFO: lif_neuron_layer_tb 仿真结束。", $time);
        $finish;
    end

    // (可选) 监控关键信号
    initial begin
        #1; // 等待DUT实例化完成
        $monitor("[%0t ns] TB_MONITOR: clk=%b, rst=%b, enable=%b, valid_in=%b, I[0]=%d, I[1]=%d, SpkOut=%b, SpkValid=%b",
                 $time, tb_clk, tb_rst_n, tb_i_enable_layer, tb_i_currents_valid, 
                 tb_i_all_currents_I[0], tb_i_all_currents_I[1], 
                 tb_o_all_spikes_out, tb_o_all_spikes_valid);
    end

endmodule