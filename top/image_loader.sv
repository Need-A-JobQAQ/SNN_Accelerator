// image_loader.v
// 从外部BRAM IP核中读取图像像素数据，并存储到内部寄存器阵列中。
// 采用流水线方式读取BRAM，并按指定顺序填充内部图像缓冲区。
// 假设BRAM的输出数据 i_bram_dout_raw [63:0] 的打包方式为：
//   i_bram_dout_raw[63:56] 存储该8像素组中的第0个逻辑像素（光栅顺序最靠前）。
//   i_bram_dout_raw[7:0]   存储该8像素组中的第7个逻辑像素（光栅顺序最靠后）。
// 内部图像缓冲区 image_pixel_buffer_reg 和输出 o_image_buffer_out 的索引方式为：
//   [P_NUM_INPUT_PIXELS-1] (例如[783]) 对应图像的左上角(光栅索引0)。
//   [0] 对应图像的右下角(光栅索引 P_NUM_INPUT_PIXELS-1)。
// 输入是按照光栅扫描顺序存取，高位(数字大的索引)对应的是光栅起始也即图片左上角，低位(数字小的索引)对应的是光栅终止也即右下角。
// BRAM字地址(0到P_IMAGE_BRAM_DEPTH-1)是按照从小到大递增的，对应于图片像素存储的光栅扫描顺序，
// 地址0对应的是最开始的一批图片像素和输入（即图片的左上角部分）。
// 图片像素的存储和读取均是按照图片从左到右从上到下的顺序存和读的，因此在 .coe 文件中第一个数据包(BRAM地址0的内容)是图片左上角前四个权重。
// 每个图片像素格式为两位无符号数十六进制数格式

module image_loader #(
    // --- 用户可配置的逻辑参数 ---
    parameter P_NUM_INPUT_PIXELS       = 784, // 图像总像素数 (28x28)
    parameter P_PIXEL_INTENSITY_BITS   = 8,   // 单个像素的位宽

    // --- 描述【外部图像】BRAM IP核的固定特性 ---
    parameter P_IMAGE_BRAM_DATA_WIDTH  = 64,  // 图像BRAM的数据输出端口位宽
    parameter P_IMAGE_BRAM_DEPTH       = 98   // 图像BRAM的有效存储字数 (98个64位的字)
) (
    input wire                                        clk,
    input wire                                        rst_n,
    input wire                                        i_load_image_start,     // 启动图像加载的脉冲信号
    input wire [P_IMAGE_BRAM_DATA_WIDTH-1:0]          i_bram_dout_raw,        // 每一次从图像BRAM读出的数据

    output reg [($clog2(P_IMAGE_BRAM_DEPTH))-1:0]     o_bram_addr,            // 输出给图像BRAM的地址
    output reg                                        o_bram_ena,             // 输出给图像BRAM的使能信号
    
    output reg [P_NUM_INPUT_PIXELS-1:0] [P_PIXEL_INTENSITY_BITS-1:0] o_image_buffer_out,  // 完整的图片数据数组
    output reg                                        o_loading_busy,         // 图像加载忙信号
    output reg                                        o_load_done             // 图像加载完成脉冲信号，单周期脉冲
);

    // ------------------- 内部常量和状态定义 (根据参数计算) -------------------
    localparam PIXELS_PER_BRAM_WORD   = P_IMAGE_BRAM_DATA_WIDTH / P_PIXEL_INTENSITY_BITS; // 每BRAM字包含的像素数 (例如 64/8 = 8)
    localparam BRAM_ADDR_MAX_VAL      = P_IMAGE_BRAM_DEPTH - 1;                           // 图像BRAM的最大有效地址 (例如 98-1 = 97)
    localparam BRAM_READ_LATENCY      = 2;                                                // 图像BRAM的读取延迟周期数 (从发出ena/addr到douta有效)
    localparam LP_BRAM_CTRL_WIDTH    = $clog2(P_IMAGE_BRAM_DEPTH + 1); // 计算BRAM地址总线和计数到P_IMAGE_BRAM_DEPTH个事件的计数器的位宽的位宽

    // 状态机状态定义
    localparam STATE_BITS_FSM       = 2;     // 状态编码所需的位数 (基于下面定义的4个状态)
    localparam S_IDLE_FSM           = 2'b00; // 00: 空闲状态，等待i_load_image_start信号
    localparam S_LOADING_FSM        = 2'b01; // 01: 从BRAM加载像素到内部缓冲区的流水线处理中
    localparam S_FLUSH_PIPE_FSM     = 2'b10; // 10: 已停止发起新的BRAM读取，等待流水线中剩余数据处理完毕
    localparam S_DONE_LOAD_FSM      = 2'b11; // 11: 所有数据已加载并写入内部缓冲区，加载完成

    reg [STATE_BITS_FSM-1:0] current_state_reg, next_state_reg; // FSM当前状态和下一状态寄存器

    // ------------ 内部图像缓冲区 ------------
    // 用于逐行存储从BRAM加载的像素数据，索引方式: image_pixel_buffer_reg[P_NUM_INPUT_PIXELS-1] 对应光栅扫描的第一个像素(左上角)
    reg [P_PIXEL_INTENSITY_BITS-1:0] image_pixel_buffer_reg [P_NUM_INPUT_PIXELS-1:0];

    // ------------ BRAM地址和有效位流水线寄存器 ------------
    // 用于将发往BRAM的地址和其有效性标志打拍，以匹配BRAM的读取延迟，确保数据对齐。
    // 深度为 BRAM_READ_LATENCY 2级。
    // addr_pipeline_reg[0] 存储当前周期发出的地址，addr_pipeline_reg[1] 存储上一周期发出的地址。
    // valid_pipeline_reg[0] 存储当前周期发出的地址是否有效，valid_pipeline_reg[1] 存储上一周期发出的地址是否有效。
    reg [LP_BRAM_CTRL_WIDTH-1:0] addr_pipeline_reg  [BRAM_READ_LATENCY-1:0]; 
    reg                          valid_pipeline_reg [BRAM_READ_LATENCY-1:0];

    // ------------ 计数器 ------------
    // issued_bram_word_count_reg: 记录已经向BRAM发出了多少个数据字的读取命令。
    // 其值从0增加到P_IMAGE_BRAM_DEPTH，表示已发出P_IMAGE_BRAM_DEPTH次请求。
    reg [LP_BRAM_CTRL_WIDTH-1:0] issued_bram_word_count_reg;
    // written_bram_word_count_reg: 记录已经从BRAM成功读取并写入到内部缓冲区的有效数据字的数量。
    // 其值从0增加到P_IMAGE_BRAM_DEPTH。
    reg [LP_BRAM_CTRL_WIDTH-1:0] written_bram_word_count_reg;

    logic [P_PIXEL_INTENSITY_BITS-1:0] current_pixel_val_local; // 用于数据解包
    

    // ------------------- 时序逻辑：状态机和寄存器更新 -------------------
    always @(posedge clk or negedge rst_n) begin
        integer k_init_loop; 
        if (!rst_n) begin // 复位逻辑
            current_state_reg          <= S_IDLE_FSM;
            o_bram_addr                <= {LP_BRAM_CTRL_WIDTH{1'b0}};
            issued_bram_word_count_reg <= {LP_BRAM_CTRL_WIDTH{1'b0}};
            written_bram_word_count_reg<= {LP_BRAM_CTRL_WIDTH{1'b0}};
            o_loading_busy             <= 1'b0;
            o_load_done                <= 1'b0;
            // 初始化内部缓冲区和输出缓冲区为0
            for (k_init_loop = 0; k_init_loop < P_NUM_INPUT_PIXELS; k_init_loop = k_init_loop + 1) begin
                o_image_buffer_out[k_init_loop]     <= {P_PIXEL_INTENSITY_BITS{1'b0}};
                image_pixel_buffer_reg[k_init_loop] <= {P_PIXEL_INTENSITY_BITS{1'b0}};
            end
            // 初始化流水线寄存器
            for (k_init_loop = 0; k_init_loop < BRAM_READ_LATENCY; k_init_loop = k_init_loop + 1) begin
                addr_pipeline_reg[k_init_loop]  <= {LP_BRAM_CTRL_WIDTH{1'b0}};
                valid_pipeline_reg[k_init_loop] <= 1'b0;
            end
        end else begin // 时钟有效沿操作
            current_state_reg <= next_state_reg; // 更新当前状态
            o_load_done       <= 1'b0;          // o_load_done 是单周期脉冲，默认为0

            // 当从 S_IDLE_FSM 进入 S_LOADING_FSM 时，进行初始化操作
            if (current_state_reg == S_IDLE_FSM && next_state_reg == S_LOADING_FSM) begin
                o_bram_addr                <= {LP_BRAM_CTRL_WIDTH{1'b0}};    // BRAM地址从0开始
                issued_bram_word_count_reg <= {LP_BRAM_CTRL_WIDTH{1'b0}};    // 已发出BRAM字计数清零
                written_bram_word_count_reg<= {LP_BRAM_CTRL_WIDTH{1'b0}};    // 已写入缓冲区的BRAM字计数清零
                o_loading_busy             <= 1'b1;                          // 设置为加载忙状态
                // 清空地址和有效位流水线的valid标志
                for (k_init_loop = 0; k_init_loop < BRAM_READ_LATENCY; k_init_loop = k_init_loop + 1) begin
                    valid_pipeline_reg[k_init_loop] <= 1'b0;
                end
            end

            // 地址和有效位流水线移位逻辑 (在 S_LOADING_FSM 或 S_FLUSH_PIPE_FSM 状态下进行)
            if (current_state_reg == S_LOADING_FSM || current_state_reg == S_FLUSH_PIPE_FSM) begin
                for (k_init_loop = BRAM_READ_LATENCY - 1; k_init_loop > 0; k_init_loop = k_init_loop - 1) begin
                    addr_pipeline_reg[k_init_loop]  <= addr_pipeline_reg[k_init_loop-1];  // addr_pipeline_reg[1] <= addr_pipeline_reg[0]
                    valid_pipeline_reg[k_init_loop] <= valid_pipeline_reg[k_init_loop-1]; // valid_pipeline_reg[1] <= valid_pipeline_reg[0]
                end
                valid_pipeline_reg[0] <= o_bram_ena; // 将当前周期的BRAM使能信号存入流水线第一级
                addr_pipeline_reg[0]  <= o_bram_addr; // 将当前发往BRAM的地址存入流水线第一级
                // 如果 o_bram_ena 为0，addr_pipeline_reg[0] 的值不重要（或保持原值），因为 valid_pipeline_reg[0] 会是0
            end

            // 当流水线末端数据有效时，将从BRAM读出的数据解包并写入内部图像缓冲区
            // 注意由于数据滞后地址两个时钟，因此 valid_pipeline_reg[1] 对应的数据在 i_bram_dout_raw 中
            if ((current_state_reg == S_LOADING_FSM || current_state_reg == S_FLUSH_PIPE_FSM) && valid_pipeline_reg[BRAM_READ_LATENCY-1]) begin
                /*
                    当你在 module ... endmodule 之间，但在任何 always 块、initial 块或 assign 语句之外声明一个变量（如 integer my_var; 或 reg my_var;），你实际上是在声明一个可以在整个模块内被访问的静态变量或寄存器。
                    integer 类型在Verilog中通常被视为一个32位的有符号 reg。
                    base_pixel_global_idx_local 的值只在它被计算出来的那个时钟周期的那个特定代码路径下被用来计算 target_buffer_idx_local。它本身的值并不需要在时钟周期之间保持状态以供其他逻辑在后续周期使用。
                    如果 base_pixel_global_idx_local 的目的仅仅是在那个 if 条件块内部作为一个临时的、用于辅助计算的变量，那么将它声明在模块级别会使其作用域过大，并且可能误导阅读者（或综合器）认为它是一个需要保持状态的全局寄存器
                    更好的做法：将这种只在特定代码块内使用的临时计算变量声明在该代码块的内部。
                    我最开始以为所有的声明都必须在always块外面，其实是不对的
                */
                integer base_pixel_global_idx_local;
                integer pix_in_word_local;         
                integer target_buffer_idx_local;
                // 此处 valid_pipeline_reg[BRAM_READ_LATENCY-1] 为高，表示当前周期的 i_bram_dout_raw 上的数据是有效的，并且它对应于 addr_pipeline_reg[BRAM_READ_LATENCY-1] 中存储的BRAM字地址。
                base_pixel_global_idx_local = addr_pipeline_reg[BRAM_READ_LATENCY-1] * PIXELS_PER_BRAM_WORD;
                // 循环解包并存储一个BRAM字中包含的所有像素 (8个)
                for (pix_in_word_local = 0; pix_in_word_local < PIXELS_PER_BRAM_WORD; pix_in_word_local = pix_in_word_local + 1) begin
                    // 从i_bram_dout_raw中提取第pix_in_word_local个逻辑像素
                    // 假设 pix_in_word_local=0 对应 i_bram_dout_raw 的最高8位 (光栅最前)
                    //     pix_in_word_local=7 对应 i_bram_dout_raw 的最低8位 (光栅最后)
                    current_pixel_val_local = i_bram_dout_raw[(P_IMAGE_BRAM_DATA_WIDTH - 1 - (pix_in_word_local * P_PIXEL_INTENSITY_BITS)) -: P_PIXEL_INTENSITY_BITS];
                    // 该像素在内部图像缓冲区中的目标索引
                    // 目标索引计算：高索引存光栅起始像素
                    target_buffer_idx_local = P_NUM_INPUT_PIXELS - 1 - (base_pixel_global_idx_local + pix_in_word_local);
                    if (target_buffer_idx_local >= 0 && target_buffer_idx_local < P_NUM_INPUT_PIXELS) begin // 边界检查
                        image_pixel_buffer_reg[target_buffer_idx_local] <= current_pixel_val_local;
                    end
                end
                written_bram_word_count_reg <= written_bram_word_count_reg + 1; // 已写入缓冲区的BRAM字计数增加
            end

            // BRAM地址更新 和 已发出请求计数更新 (仅在S_LOADING_FSM且实际发出BRAM使能时)
            if (current_state_reg == S_LOADING_FSM && o_bram_ena) begin
                issued_bram_word_count_reg <= issued_bram_word_count_reg + 1;
                if (o_bram_addr < BRAM_ADDR_MAX_VAL) begin // 如果当前地址未达到最大有效地址
                    o_bram_addr <= o_bram_addr + 1;   // 准备下一个要读取的地址
                end
            end
            
            // 当加载完成状态时，输出结果并设置完成标志
            if (current_state_reg == S_DONE_LOAD_FSM) begin
                o_load_done    <= 1'b1;         // 发出加载完成脉冲
                o_loading_busy <= 1'b0;         // 加载不再繁忙
                // 将内部缓冲区的图像数据赋给输出端口
                for (k_init_loop = 0; k_init_loop < P_NUM_INPUT_PIXELS; k_init_loop = k_init_loop + 1) begin
                    o_image_buffer_out[k_init_loop] <= image_pixel_buffer_reg[k_init_loop];
                end
            end
        end
    end

    // ------------------- 组合逻辑：状态转移控制 和 BRAM使能控制 -------------------
    always @(*) begin
        next_state_reg = current_state_reg; // 默认保持当前状态
        o_bram_ena     = 1'b0;              // BRAM使能默认为低

        case (current_state_reg)
            S_IDLE_FSM: begin
                if (i_load_image_start) begin
                    next_state_reg = S_LOADING_FSM; // 收到启动信号，进入加载状态
                end
            end
            S_LOADING_FSM: begin
                if (issued_bram_word_count_reg < P_IMAGE_BRAM_DEPTH) begin // 如果还有BRAM字需要发出读取命令
                    o_bram_ena     = 1'b1;              // 使能BRAM进行读取
                    next_state_reg = S_LOADING_FSM;     // 保持在加载状态
                end else begin                          // 所有BRAM地址的读取命令均已发出
                    o_bram_ena     = 1'b0;              // 停止使能BRAM
                    next_state_reg = S_FLUSH_PIPE_FSM;  // 进入冲刷流水线状态，等待剩余数据写入
                end
            end
            S_FLUSH_PIPE_FSM: begin
                o_bram_ena = 1'b0; // 在冲刷状态，不再发起新的BRAM读取
                if (written_bram_word_count_reg == P_IMAGE_BRAM_DEPTH) begin // 等待全数据处理完毕并写入缓冲区
                    next_state_reg = S_DONE_LOAD_FSM;
                end else begin
                    next_state_reg = S_FLUSH_PIPE_FSM; // 继续冲刷
                end
            end
            S_DONE_LOAD_FSM: begin
                next_state_reg = S_IDLE_FSM; // 完成加载后，返回空闲状态
            end
            default: next_state_reg = S_IDLE_FSM;
        endcase
    end

endmodule