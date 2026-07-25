// control_unit.v
// 控制整个SNN的操作时序和流程。
// 为数据路径模块生成单周期使能/启动脉冲，并等待数据路径处理完成当前时间步。
// i_global_start_signal 表示图像已加载完毕，可以开始SNN核心运算。
// i_datapath_step_done 输入，指示数据路径已完成对当前时间步的处理。

module control_unit #(
    parameter P_T_MAX = 100 // SNN总时间步数
) (
    input wire                               clk,
    input wire                               rst_n,
    // 图像加载完成信号 (单周期脉冲)，顶层中由 img_loader_load_done 传入，表示图像数据已准备好，可以开始SNN的P_T_MAX步处理
    input wire                               i_global_start_signal,
    input wire                               i_datapath_step_done,   // 需要指示神经网络已完成当前时间步的计算的信号传入 (来自lif_neuron_layer层的o_all_spikes_valid)
    input wire                               i_early_stop,           // 输出层判断当前领先类别已无法被反超时，提前结束剩余时间步

    output reg [$clog2(P_T_MAX)-1:0]         o_current_time_step_t,    // 当前正在处理的时间步 t (0 到 P_T_MAX-1)
    
    // 数据路径模块的使能/启动信号 (设计为单周期脉冲)
    output reg                               o_poisson_encoder_en,       // 泊松编码器使能信号 (单周期脉冲，在每个时间步t有效)
    output reg                               o_linear_layer_start,       // 线性层计算启动信号 (单周期脉冲，在每个时间步t有效)
    output reg                               o_lif_layer_en,             // LIF神经元层使能信号 (单周期脉冲，在每个时间步t有效)
    output reg                               o_output_logic_accum_en,    // 输出逻辑累加使能信号 (单周期脉冲，在每个时间步t有效)
    output reg                               o_output_logic_decision_en, // 输出逻辑决策使能信号 (单周期脉冲，在所有时间步处理完毕后有效)
    
    output reg                               o_snn_busy,                 // SNN忙信号（现在从P_T_MAX步处理开始到结束）
    output reg                               o_global_processing_done    // SNN处理完成信号 (单周期脉冲)
);

    // 内部常量和状态定义
    localparam LP_CLOG2_T_MAX = $clog2(P_T_MAX); // 时间步计数器的位宽，例如P_T_MAX=100,则为7 (0-99)

    // 状态机状态定义 (现在是6个主要状态，仍可用3位编码)
    localparam LP_STATE_BITS         = 3;
    localparam S_IDLE_VAL            = 3'b000; // 000: 空闲状态，等待i_global_start_signal存储图片完成进行启动
    localparam S_START_DATAPATH_VAL  = 3'b001; // 001: 执行当前SNN时间步，输出各模块使能脉冲
    localparam S_WAIT_DATAPATH_VAL   = 3'b010; // 010: 等待整个神经网络完成当前时间步处理 (来自lif_neuron_layer层的o_all_spikes_valid)
    localparam S_CHECK_LOOP_VAL      = 3'b011; // 011: 检查是否所有时间步完成，用于决定跳转为 010 还是 100
    localparam S_DECIDE_VAL          = 3'b100; // 100: 所有时间步完成，使能输出逻辑进行决策
    localparam S_DONE_VAL            = 3'b101; // 101: SNN处理和决策均完成，发出全局完成信号

    reg [LP_STATE_BITS-1:0] current_state_reg, next_state_reg; // FSM当前状态和下一状态寄存器
    reg [LP_CLOG2_T_MAX-1:0] time_counter_reg;                 // 时间步计数器 (0 到 P_T_MAX-1)，用于记录现在处理到了哪一个SNN时间步

    // --- 时序逻辑：状态机和计数器更新 ---
    // 这个always块负责在每个时钟上升沿更新状态机当前状态和时间步计数器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin // 异步复位逻辑
            current_state_reg <= S_IDLE_VAL;
            time_counter_reg  <= {LP_CLOG2_T_MAX{1'b0}};
        end else begin // 同步逻辑
            current_state_reg <= next_state_reg; // 每个周期用下一状态更新当前状态

            // 当状态机从任何状态转换回S_IDLE_VAL时，或者当刚从S_IDLE_VAL因启动信号有效而即将进入S_START_DATAPATH_VAL时，time_counter_reg 都应清零或确保为0。
            if (next_state_reg == S_IDLE_VAL || (current_state_reg == S_IDLE_VAL && next_state_reg == S_START_DATAPATH_VAL) ) begin 
                time_counter_reg <= {LP_CLOG2_T_MAX{1'b0}};
            // 当在S_CHECK_LOOP_VAL状态检查后，确定还要继续下一个时间步（即下一状态是S_START_DATAPATH_VAL）时，时间步计数器加1
            end else if (current_state_reg == S_CHECK_LOOP_VAL && next_state_reg == S_START_DATAPATH_VAL) begin
                time_counter_reg <= time_counter_reg + 1;
            end
            // 其他所有情况下，time_counter_reg 保持其值不变
        end
    end

    // --- 组合逻辑：下一状态逻辑 和 输出信号逻辑 ---
    // 这个always块描述了状态机的下一状态转换逻辑以及各个输出控制信号的生成逻辑
    // 它是组合逻辑，其输出会根据current_state_reg和输入信号i_global_start_signal立即变化
    always @(*) begin
        // 默认输出值：状态机保持当前状态，所有单周期脉冲输出为0
        next_state_reg                = current_state_reg; 
        o_current_time_step_t         = time_counter_reg;
        o_poisson_encoder_en          = 1'b0;
        o_linear_layer_start          = 1'b0;
        o_lif_layer_en                = 1'b0;
        o_output_logic_accum_en       = 1'b0;
        o_output_logic_decision_en    = 1'b0;
        // o_snn_busy 在IDLE和DONE之外都为高，表示SNN核心处理正在进行
        o_snn_busy                    = (current_state_reg != S_IDLE_VAL && current_state_reg != S_DONE_VAL);
        o_global_processing_done      = 1'b0; 

        case (current_state_reg)
            S_IDLE_VAL: begin
                if (i_global_start_signal) begin // i_global_start_signal 现在意味着图像已加载，可以开始核心处理
                    next_state_reg = S_START_DATAPATH_VAL; // 直接进入第一个时间步的执行准备
                    // o_snn_busy 将在下一状态变为1
                end
            end

            S_START_DATAPATH_VAL: begin
                // 为所有数据路径模块产生单周期使能/启动脉冲
                o_poisson_encoder_en      = 1'b1;
                o_linear_layer_start      = 1'b1;
                o_lif_layer_en            = 1'b1;
                o_output_logic_accum_en   = 1'b1;
                next_state_reg = S_WAIT_DATAPATH_VAL; // 发出使能后，进入等待数据路径完成的状态
            end

            S_WAIT_DATAPATH_VAL: begin
                // 等待来自数据路径的“当前时间步处理完成”信号 (来自lif_neuron_layer层的o_all_spikes_valid)
                if (i_datapath_step_done) begin 
                    next_state_reg = S_CHECK_LOOP_VAL; // 数据路径已处理完当前步，去检查是否所有时间步完成
                end else begin
                    next_state_reg = S_WAIT_DATAPATH_VAL; // 继续等待
                end
            end

            S_CHECK_LOOP_VAL: begin
                // 在这个状态，所有使能脉冲已经因状态转换而自动结束
                if (i_early_stop || time_counter_reg == (P_T_MAX - 1)) begin // 是否可以提前停止，或已完成所有 P_T_MAX 时间步?
                    next_state_reg = S_DECIDE_VAL;         // 是，则所有时间步处理完毕，进入决策状态
                end else begin
                    // 否，则准备进入下一个执行周期 (time_counter_reg 将在时序逻辑中更新)
                    next_state_reg = S_START_DATAPATH_VAL; 
                end
            end

            S_DECIDE_VAL: begin
                o_output_logic_decision_en = 1'b1; // 使能输出逻辑进行最终决策，即选择出结果的最大值
                next_state_reg             = S_DONE_VAL;
            end

            S_DONE_VAL: begin
                o_global_processing_done = 1'b1; // 发出全局处理完成的单周期脉冲
                next_state_reg           = S_IDLE_VAL;     
            end
            
            default: begin
                next_state_reg = S_IDLE_VAL;
            end
        endcase
    end

endmodule
