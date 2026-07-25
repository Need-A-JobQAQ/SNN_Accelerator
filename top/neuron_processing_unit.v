// neuron_processing_unit.v
// BRAM接口外露 - 流水线优化
// 除了二维数据的使用外，其他的语句均为 Verilog-2001 规范编写，在 Vivado 中仍需要将文件设置为 system verilog 格式才可以正常运行，若要使用标准 verilog 需要对二维数组部分进行重构
// 单个神经元的处理单元。它从【外部】BRAM ROM接收权重数据，并与输入的脉冲向量进行加权求和，以计算神经元的总输入电流。
// 采用流水线设计以提高吞吐量。
// 权重和输入都是按照光栅扫描顺序存取，高位(数字大的索引)对应的是光栅起始，低位(数字小的索引)对应的是光栅终止。
// BRAM字地址(0到P_BRAM_EFFECTIVE_DEPTH-1)是按照从小到大递增的，对应于权重存储的光栅扫描顺序，
// 地址0对应的是最开始的一批权重和输入（即图片的左上角部分）。
// 权重的存储和读取均是按照图片从左到右从上到下的顺序存和读的，因此在 .coe 文件中第一个数据包(BRAM地址0的内容)是图片左上角前四个权重。
// 权重格式为 Q4.12 格式：1位符号位，3位整数位，12位小数位。以补码形式存储
// 核心点一定要记住（数据）是滞后（地址与有效判定）两个时钟的，也就是说前者和后者是错位的

// 从外部BRAM输入的 i_bram_dout_raw [P_BRAM_DATA_WIDTH-1:0] (例如[63:0])，其内部4个权重的排列：
//   i_bram_dout_raw[63:48] 存储第0个权重 (组内逻辑顺序，例如光栅索引 k)
//   i_bram_dout_raw[47:32] 存储第1个权重 (组内逻辑顺序，例如光栅索引 k+1)
//   i_bram_dout_raw[31:16] 存储第2个权重 (组内逻辑顺序，例如光栅索引 k+2)
//   i_bram_dout_raw[15:0]  存储第3个权重 (组内逻辑顺序，例如光栅索引 k+3)
//   w_from_bram [WEIGHTS_NUMS_PER_WORD-1:0] 负责解包 i_bram_dout_raw 的四个权重，我们期望：
//   w_from_bram[WEIGHTS_NUMS_PER_WORD-1] (例如 w_from_bram[3]) 得到 i_bram_dout_raw[63:48] (即组内第0个逻辑权重)
//   w_from_bram[0]                         得到 i_bram_dout_raw[15:0]  (即组内第3个逻辑权重)

//**************************************************************************** DEMO ****************************************************************************
//         
//                          正常图片平铺视角/光栅扫描顺序                           
//                                       |
//                   -------------------------------------------
//                  |                                           |
//               权重矩阵                                    脉冲矩阵                    
//  ABCD(权重地址为0) EFGH(权重地址为1)  abcd(脉冲序号31,30,29,28) efgh(脉冲序号27,26,25,24)    第一次读取时 bram_douta_raw 从 63->0 存 A->D，也即 bram_douta_raw[63:48] = A，bram_douta_raw[15:0] = D
//  IJKL(权重地址为2) MNOP(权重地址为3)  ijkl(脉冲序号23,22,21,20) mnop(脉冲序号19,18,17,16)    w_pipe_out[3/2/1/0] 负责解包切分 bram_douta_raw 的四个权重，w_pipe_out[3] = A，w_pipe_out[0] = D
//  QRST(权重地址为4) UVWX(权重地址为5)  qrst(脉冲序号15,14,13,12) uvwx(脉冲序号11,10,9,8)      类似的，s_pipe_out[3] = a，s_pipe_out[0] = d，这个时候 w_pipe_out 和 s_pipe_out 实现了位置的一一对应
//  YZ?/(权重地址为6) *&%$(权重地址为7)  yz?/(脉冲序号7,6,5,4)     *&%$(脉冲序号3,2,1,0)        注意点在于权重的地址是从 0 递增的，而 0 的地址对应于脉冲输入图的高位地址，也就是说权重从低到高而输入从高到低才能实现对应
// 
//**************************************************************************************************************************************************************

// i_input_spike_vector,latched_spike_vector_reg 按照正常图片的顺序存储，从左到右从上到下，
// 最高位索引 [P_NUM_INPUT_PIXELS-1] (例如[783]) 对应图片的左上角(光栅索引0)，
// 最低位索引 [0] 对应图片的右下角(光栅索引 P_NUM_INPUT_PIXELS-1)。
// s_for_aligned_addr [WEIGHTS_NUMS_PER_WORD-1:0] 负责从 latched_spike_vector_reg 提取与当前权重组对应的四个脉冲。
// 我们期望 s_for_aligned_addr[WEIGHTS_NUMS_PER_WORD-1] (例如 s_for_aligned_addr[3]) 是权重组内第一个逻辑脉冲。

module neuron_processing_unit #(
    // --- 用户可配置的逻辑参数 ---
    parameter P_NUM_INPUT_PIXELS       = 784,
    parameter P_WEIGHT_BIT_WIDTH       = 16,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter P_NEURON_VALUE_FRAC_BITS  = 12,
    //parameter P_NUM_NEURONS            = 1568,

    // --- 描述【外部】BRAM IP核的固定特性 ---
    // 这些参数现在描述的是本模块期望连接的外部BRAM的特性
    parameter P_BRAM_DATA_WIDTH        = 64,
    parameter P_BRAM_ADDR_WIDTH        = 8,
    parameter P_BRAM_EFFECTIVE_DEPTH   = 392
) (
    input wire                                          clk,
    input wire                                          rst_n,
    input wire                                          i_calc_start,
    input wire  [P_NUM_INPUT_PIXELS-1:0]                i_input_spike_vector,
    input wire signed [P_BRAM_DATA_WIDTH-1:0]           i_bram_dout_raw,      // 从外部BRAM读取的数据输入端，BRAM原始数据输出

    output reg [P_BRAM_ADDR_WIDTH-1:0]                  o_bram_addr_to_issue, // 控制外部BRAM的输出端口，当前要发送给BRAM的地址
    output reg                                          o_bram_ena,           // 控制外部BRAM的输出端口，BRAM使能信号
    output reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0]    o_output_current_I,
    output reg                                          o_current_valid_out
);

    // ------------------- 内部常量和状态定义 (根据参数计算) -------------------
    localparam WEIGHTS_NUMS_PER_WORD = P_BRAM_DATA_WIDTH / P_WEIGHT_BIT_WIDTH;
    localparam ADDR_MAX_VAL           = P_BRAM_EFFECTIVE_DEPTH - 1;
    localparam BRAM_READ_LATENCY      = 2; 
    localparam NUM_INPUT_PIXELS_BITS  = $clog2(P_NUM_INPUT_PIXELS);

    localparam STATE_BITS           = 2;
    localparam S_IDLE_VAL           = 2'b00;  // 00: 空闲状态，等待开始信号
    localparam S_PROCESSING_VAL     = 2'b01;  // 01: 流水线处理中 (包括填充和稳定运行)
    localparam S_FLUSHING_VAL       = 2'b10;  // 10: 停止发起新的BRAM读取，等待流水线中剩余 2 个数据处理完毕
    localparam S_DONE_VAL           = 2'b11;  // 11: 所有数据处理完毕，输出结果 (此状态实际只持续一个周期用于置位valid信号)

    reg [STATE_BITS-1:0] current_state_reg, next_state_reg;

    // 内部寄存器
    reg [P_NUM_INPUT_PIXELS-1:0]                   latched_spike_vector_reg; // 接收并锁存住 i_input_spike_vector
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0]      accumulator_reg;          // 累加器，不断计算权重之和

    // 流水线寄存器，用于对齐延迟的BRAM数据和对应的原始地址（用于索引脉冲），定义为二维数组
    // 深度为BRAM_READ_LATENCY级，也即从时钟上升沿发送BRAM地址请求到得到数据需要两个时钟周期
    reg [P_BRAM_ADDR_WIDTH-1:0]                    addr_pipeline_reg  [BRAM_READ_LATENCY-1:0]; // 作为流水寄存器锁存要发给BRAM的地址 o_bram_addr_to_issue, addr_pipeline_reg[0]是第一级 (直接来自BRAM)，addr_pipeline_reg[1]是最后一级
    reg                                            valid_pipeline_reg [BRAM_READ_LATENCY-1:0]; // 作为流水寄存器锁存BRAM的数据使能信号 o_bram_ena, 值为1时说明进行了BRAM读取请求并使其与对应的地址在流水线中同步前进，但是注意该请求对应的数据在两个时钟之后。检测到这个值为0时意味着已经全部请求已经完了

    // processed_data_count_reg 会比 issued_addr_count_reg 少 2, 因为数据滞后于请求两个时钟
    reg [P_BRAM_ADDR_WIDTH-1:0]             issued_addr_count_reg;
    reg [P_BRAM_ADDR_WIDTH-1:0]             processed_data_count_reg;

    // 数据处理
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] terms_comb [WEIGHTS_NUMS_PER_WORD-1:0]; // 存储条件化和符号扩展后的权重
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] sum_of_N_active_weights_comb;           // 一个权重包和对应的输入脉冲的加和

    integer k_idx; 
    genvar j_gen, l_gen, m_gen;

    // ------------------- 时序逻辑：状态机和寄存器更新 -------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state_reg        <= S_IDLE_VAL;
            latched_spike_vector_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
            accumulator_reg          <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            o_bram_addr_to_issue     <= {P_BRAM_ADDR_WIDTH{1'b0}}; // 初始化输出端口
            issued_addr_count_reg    <= {P_BRAM_ADDR_WIDTH{1'b0}};
            processed_data_count_reg <= {P_BRAM_ADDR_WIDTH{1'b0}};
            o_output_current_I       <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            o_current_valid_out      <= 1'b0;
            for (k_idx = 0; k_idx < BRAM_READ_LATENCY; k_idx = k_idx + 1) begin
                addr_pipeline_reg[k_idx]  <= {P_BRAM_ADDR_WIDTH{1'b0}};
                valid_pipeline_reg[k_idx] <= 1'b0;
            end
        end else begin
            current_state_reg   <= next_state_reg;
            o_current_valid_out <= 1'b0; // 默认为无效，只在DONE状态的一个周期有效

            // --- 根据当前状态更新主要寄存器 ---
            // 下面的表达与 if (current_state_reg == S_IDLE_VAL && i_calc_start) 语句作用一样
            if (current_state_reg == S_IDLE_VAL && next_state_reg == S_PROCESSING_VAL) begin
                latched_spike_vector_reg <= i_input_spike_vector;
                accumulator_reg          <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                o_bram_addr_to_issue     <= {P_BRAM_ADDR_WIDTH{1'b0}};
                issued_addr_count_reg    <= {P_BRAM_ADDR_WIDTH{1'b0}};
                processed_data_count_reg <= {P_BRAM_ADDR_WIDTH{1'b0}};
                // 清空流水线valid标志，这个 valid_pipeline_reg 流水线寄存器才是真正掌控是否需要进行计算的，所以只需要保证它是清空的即可，剩下的不需要强制清空
                // 即使旧的值可能在 addr_pipeline_reg 和 bram_douta_raw 中留下了数据，但由于相应的 valid_pipeline_reg 位已经被清零，这些旧数据在到达流水线末端时不会被错误地处理
                // 显式地清零所有流水线寄存器会消耗更多的逻辑资源和可能略微增加功耗。
                for (k_idx = 0; k_idx < BRAM_READ_LATENCY; k_idx = k_idx + 1) begin
                    valid_pipeline_reg[k_idx] <= 1'b0;
                end
            end

            // --- 流水线移位逻辑 ---
            // 这个移位总是在进行，除非被特定条件控制 (例如，只在S_PROCESSING或S_FLUSHING状态)
            if (current_state_reg == S_PROCESSING_VAL || current_state_reg == S_FLUSHING_VAL) begin
                // 数据流水线移位
                for (k_idx = BRAM_READ_LATENCY - 1; k_idx > 0; k_idx = k_idx - 1) begin
                    addr_pipeline_reg[k_idx]  <= addr_pipeline_reg[k_idx-1];
                    valid_pipeline_reg[k_idx] <= valid_pipeline_reg[k_idx-1];
                end
                // addr_pipeline_reg 作为流水寄存器锁存要发给BRAM的地址 bram_addr_to_issue_reg，里面的值是一直递增的
                // valid_pipeline_reg 作为流水寄存器锁存BRAM的数据使能信号 o_bram_ena, 一共要发送 196 个 1，即申请 196 次权重并告知它们是有效的
                valid_pipeline_reg[0] <= o_bram_ena;            // o_bram_ena 是当前周期的使能信号(由组合逻辑FSM确定)
                addr_pipeline_reg[0]  <= o_bram_addr_to_issue;  // 如果BRAM被使能读取，则进入流水线的数据是有效的
            end

            //****************************************************************************************************************************************************************************
            //
            //            流水线第二级/流水线末端                           流水线第一级
            //                     [1]                                        [0]
            //               上一次发送的地址                               刚发送的地址                             准备发送的地址 o_bram_addr_to_issue(也可能结束不发了)
            //       上一次发送的地址对应的 o_bram_ena    <--      刚发送的地址对应的 o_bram_ena        <--          准备发送的地址对应的 o_bram_ena(也可能置0结束了)
            //                      |                                          |
            //                       ------------------------------------------ 
            //                                            |
            //              从上到下依次使用流水线寄存器 addr_pipeline_reg, valid_pipeline_reg 存储
            //
            //              此时流水线第二级的内容也即 addr_pipeline_reg[1] 和 valid_pipeline_reg[1] 对应的的数据在 i_bram_dout_raw 里面，对应的脉冲要用 addr_pipeline_reg[1] 所记录的地址信息位对应去搜索
            //
            //............................................................................................................................................................................
            //
            //  核心点一定要记住（数据）是滞后（地址与有效判定）两个时钟的，也就是说前者和后者是错位的
            //  所以最开始的时候地址和有效判定1进入流水线[0]的时候，此时它对应的数据还没来，因为最开始的数据要在两个时钟后才进入
            //  所以最末尾的时候空址和无效判定0进入流水线[0]的时候，此时还是有数据的，因为结束后仍然需要处理滞后的两个数据
            //  因此，为了保证在 valid_pipeline_reg[1] 有效时, accumulator_reg 累加的是与 addr_pipeline_reg[1] 中的地址相对应的正确的权重数据
            //  正确的做法是：当 addr_pipeline_reg[1] 和 valid_pipeline_reg[1] 准备好时，权重应该直接从当前的 i_bram_dout_raw 获取数据，因为此时的 i_bram_dout_raw 正是与这些延迟后的地址/有效信号匹配的数据，即通过了流水线消除了延后两个时钟的影响
            //  可以发现只要 valid_pipeline_reg[1] 处检测到的 o_bram_ena 为 0 时，意味着 BRAM 的使能已经关闭了，之后则无需读取了
            //
            //****************************************************************************************************************************************************************************

            // --- 累加和计数器更新 ---
            // 如果流水线末端的判定与地址仍然有效，值为1，才执行累加操作
            if ((current_state_reg == S_PROCESSING_VAL || current_state_reg == S_FLUSHING_VAL) && valid_pipeline_reg[BRAM_READ_LATENCY-1]) begin
                accumulator_reg          <= accumulator_reg + sum_of_N_active_weights_comb;
                processed_data_count_reg <= processed_data_count_reg + 1;
            end

            // 只有在 S_PROCESSING 状态并且实际发出了BRAM使能时才更新 o_bram_addr_to_issue 和 issued_addr_count_reg
            /*
                    o_bram_addr_to_issue:
                这是实际输出给BRAM IP核的地址信号。它从0开始，在S_PROCESSING状态下每个BRAM被有效使能的周期，如果它还没达到最大地址 ADDR_MAX_VAL，它就会递增，为下一次BRAM读取准备新地址
                但它的递增是有条件的，不能超过 ADDR_MAX_VAL。当它达到 ADDR_MAX_VAL(195) 时，它会保持在这个值，但 issued_addr_count_reg 还会再增加一次来表示最后一个地址（ADDR_MAX_VAL）的命令也已发出
                
                    issued_addr_count_reg:
                这个寄存器用于精确地统计已经向BRAM发出了多少个有效的读取地址命令。它的值从0开始，每次成功发出一个BRAM读取命令（即 o_bram_ena 为高，并且在 S_PROCESSING 状态），它就加1。
                它会一直计数，直到发出了 P_BRAM_EFFECTIVE_DEPTH(196) 个地址的读取请求，代表发出了196个地址的命令
            */
            if (current_state_reg == S_PROCESSING_VAL && o_bram_ena) begin
                issued_addr_count_reg  <= issued_addr_count_reg + 1;
                if (o_bram_addr_to_issue < ADDR_MAX_VAL) begin
                    o_bram_addr_to_issue <= o_bram_addr_to_issue + 1; 
                end
            end
            
            if (current_state_reg == S_DONE_VAL) begin  // 如果计算完成
                o_output_current_I  <= accumulator_reg; // 输出最终的累加和
                o_current_valid_out <= 1'b1;            // 指示输出有效
            end
        end
    end

    // ------------------- 组合逻辑：状态转移控制 和 当前周期的部分和计算 -------------------
    // 1 个地址对应 ----- 1 个权重包输入 -----> 解包出 4 个权重 -----> 对应的输入脉冲也要打包出 4 个进行对应
    wire signed [P_WEIGHT_BIT_WIDTH-1:0]       w_from_bram        [WEIGHTS_NUMS_PER_WORD-1:0];
    wire                                       s_for_aligned_addr [WEIGHTS_NUMS_PER_WORD-1:0]; 
    
    generate 
        for (j_gen = 0; j_gen < WEIGHTS_NUMS_PER_WORD; j_gen = j_gen + 1) begin : gen_unpack_weights
            // 直接从 BRAM 的当前输出中解包权重
            // w_from_bram 保持一致的存储风格，高位(数字大的)存的是权重包的高位，即 w_from_bram[3/2/1/0] 满足权重顺序也是从左到右从上到下，即高位存的是图片偏左上的权重，低位存的是图片偏右下的权重
            // j_gen = 3: w_from_bram[3] 取 bram_douta_raw[...][63:48] (最高位的16比特权重)
            // j_gen = 0: w_from_bram[0] 取 bram_douta_raw[...][15:0] (最低位的16比特权重)
            assign w_from_bram[j_gen] = i_bram_dout_raw[(j_gen+1)*P_WEIGHT_BIT_WIDTH-1 : j_gen*P_WEIGHT_BIT_WIDTH];  // 从输入端口读取
        end
    endgenerate

    // 根据流水线末端的地址，从锁存的脉冲向量中提取对应的4个脉冲  
    // 例如对于第一组权重包地址为 0 的 ABCD (w_pipe_out[3]=A, w_pipe_out[0]=D), 我们需要从 latched_spike_vector_reg[783/782/781/780]读取，最终实现 s_pipe_out[3] = a s_pipe_out[0] = d
    generate 
        for (l_gen = 0; l_gen < WEIGHTS_NUMS_PER_WORD; l_gen = l_gen + 1) begin : gen_select_spikes
            // logical_offset_in_group 用来实现每一组包内的数据对应
            localparam logical_offset_in_group_g = (WEIGHTS_NUMS_PER_WORD - 1) - l_gen;
            wire [P_BRAM_ADDR_WIDTH-1:0] current_word_addr_from_pipe_w; 
            assign current_word_addr_from_pipe_w = addr_pipeline_reg[BRAM_READ_LATENCY-1]; // 当前处理的BRAM字地址，每一个地址含4个权重，也即需要4个输入脉冲
            // global_raster_idx_of_spike 用于记录对应于 s_pipe_out[l_gen] 的脉冲在784个总输入中的绝对光栅扫描索引
            wire [NUM_INPUT_PIXELS_BITS-1:0] global_raster_idx_of_spike_w; 
            assign global_raster_idx_of_spike_w    = (current_word_addr_from_pipe_w * WEIGHTS_NUMS_PER_WORD) + logical_offset_in_group_g;
            assign s_for_aligned_addr[l_gen] = latched_spike_vector_reg[P_NUM_INPUT_PIXELS - 1 - global_raster_idx_of_spike_w]; 
        end
    endgenerate

    // 预处理每一批从BRAM中读出的权重
    // 检查与当前权重对应的输入脉冲是否有效（是否为1）
    // 如果脉冲有效，则将该权重进行符号位扩展，否则输出零
    generate
        for (m_gen = 0; m_gen < WEIGHTS_NUMS_PER_WORD; m_gen = m_gen + 1) begin : gen_terms_for_sum
            assign terms_comb[m_gen] = s_for_aligned_addr[m_gen] ? // 如果对应脉冲有效
                                {{(P_NEURON_VALUE_TOTAL_BITS - P_WEIGHT_BIT_WIDTH){w_from_bram[m_gen][P_WEIGHT_BIT_WIDTH-1]}}, w_from_bram[m_gen]} :  // 符号扩展
                                {P_NEURON_VALUE_TOTAL_BITS{1'b0}}; // 否则为0
        end
    endgenerate
    
    assign sum_of_N_active_weights_comb = terms_comb[0] + terms_comb[1] + terms_comb[2] + terms_comb[3];

    // 状态转移逻辑 和 BRAM使能控制 (组合逻辑 always 块，带显式敏感列表)
    always @(current_state_reg or i_calc_start or issued_addr_count_reg or processed_data_count_reg) begin
        next_state_reg = current_state_reg; // 默认保持当前状态
        o_bram_ena     = 1'b0;              // 控制外部BRAM的使能，默认为低
        case (current_state_reg)
            S_IDLE_VAL: begin
                if (i_calc_start) begin
                    next_state_reg = S_PROCESSING_VAL;
                end
            end
            S_PROCESSING_VAL: begin
                // 持续发出BRAM读取请求，直到所有地址都已发出
                // issued_addr_count_reg 从0开始计数，当它达到 P_BRAM_EFFECTIVE_DEPTH 时，表示已经发出了 P_BRAM_EFFECTIVE_DEPTH 个地址
                if (issued_addr_count_reg < P_BRAM_EFFECTIVE_DEPTH) begin
                    o_bram_ena     = 1'b1;              // 使能BRAM读取
                    next_state_reg = S_PROCESSING_VAL;  // 保持在处理状态
                end else begin // 所有地址都已发出 (即 issued_addr_count_reg == P_BRAM_EFFECTIVE_DEPTH)
                    o_bram_ena     = 1'b0;
                    next_state_reg = S_FLUSHING_VAL; 
                end
            end
            S_FLUSHING_VAL: begin
                o_bram_ena = 1'b0;  // 不再发起新的BRAM读取
                // processed_data_count_reg 从0开始计数，当它达到 P_BRAM_EFFECTIVE_DEPTH 时，表示所有发出的数据都已被处理
                if (processed_data_count_reg == P_BRAM_EFFECTIVE_DEPTH) begin
                    next_state_reg = S_DONE_VAL;    // 所有数据处理完毕
                end else begin
                    next_state_reg = S_FLUSHING_VAL; // 继续等待流水线中剩余数据处理完毕
                end
            end
            S_DONE_VAL: begin
                next_state_reg = S_IDLE_VAL; // 完成，返回空闲状态
            end
            default: next_state_reg = S_IDLE_VAL;
        endcase
    end

endmodule