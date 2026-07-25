// poisson_encoder.v
// 实现泊松编码器功能。
// 对于一次 i_enable_enc 指令，经过一个时钟周期就能输出整个图片的随机脉冲输入结果
// 对于每个输入像素，它根据像素强度和伪随机数在每个时间步生成脉冲。
// 它为每个输入像素实例化一个 prng_lfsr 模块。

module poisson_encoder #(
    // --- 用户可配置的逻辑参数 ---
    parameter P_NUM_INPUT_PIXELS         = 784, // 输入像素的总数
    parameter P_PIXEL_INTENSITY_BITS     = 8,   // 单个像素强度值与PRNG输出随机数的位宽 (0-255对应与归一化的像素值0-1)
    parameter P_PRNG_LFSR_WIDTH          = 32,  // 传递给内部prng_lfsr实例的LFSR状态位宽
    parameter P_T_MAX                    = 100  // SNN总时间步数, 用于计算 i_time_step_t 的位宽
) (
    input wire                               clk,
    input wire                               rst_n,
    input wire                               i_enable_enc,           // 编码器使能信号 (来自control_unit)
    input wire [$clog2(P_T_MAX)-1:0]         i_time_step_t,          // 当前时间步t 
    
    // 最左边的维度通常被视为主数组维度（有多少个元素），而它右边的维度则描述了每个元素的构成（例如元素的位宽，或者如果元素本身还是数组，则是更内层的数组维度）。
    input wire [P_NUM_INPUT_PIXELS-1:0] [P_PIXEL_INTENSITY_BITS-1:0]  i_image_pixel_data,     // 输入图像像素数据

    output reg [P_NUM_INPUT_PIXELS-1:0]      o_spike_vector_reg,     // 输出的一张图片的脉冲向量 (寄存后输出)
    output reg                               o_spikes_valid_reg      // 输出脉冲有效信号 (寄存后输出, 单周期脉冲)
);

    wire [P_NUM_INPUT_PIXELS-1:0]         prng_individual_valids; // 存储所有 prng_lfsr 的输出有效信号
    wire                                  all_prngs_valid; // 假设所有PRNGs同步有效

    // 存储所有 prng_lfsr 对于整个图片的随机数输出
    wire [P_PIXEL_INTENSITY_BITS-1:0] prng_random_outputs [P_NUM_INPUT_PIXELS-1:0];
    // 存储当前时间步随机数和元素像素比较后的整个图片的输出脉冲向量
    wire [P_NUM_INPUT_PIXELS-1:0]         calculated_spike_vector;

    genvar pixel_idx_gen;

    // -------------------------------------------------------------------------
    // 1. 实例化 P_NUM_INPUT_PIXELS 个 prng_lfsr 模块
    // -------------------------------------------------------------------------
    generate
        for (pixel_idx_gen = 0; pixel_idx_gen < P_NUM_INPUT_PIXELS; pixel_idx_gen = pixel_idx_gen + 1) begin : gen_prng_pixel_instances
            
            prng_lfsr #(
                .INSTANCE_ID                  (pixel_idx_gen),  // 保证每一个 prng_lfsr 的初始 ID 不同
                .P_PRNG_LFSR_WIDTH            (P_PRNG_LFSR_WIDTH),
                .P_POISSON_ENCODER_PRNG_BITS  (P_PIXEL_INTENSITY_BITS) // PRNG输出位宽与像素强度位宽一致
            ) u_prng_for_pixel (
                .clk                    (clk),
                .rst_n                  (rst_n),
                .i_enable_prng          (i_enable_enc), 

                .o_random_output_reg    (prng_random_outputs[pixel_idx_gen]),
                .o_random_output_valid  (prng_individual_valids[pixel_idx_gen])
            );
            
        end
    endgenerate

    assign all_prngs_valid = (P_NUM_INPUT_PIXELS > 0) ? prng_individual_valids[0] : 1'b0;


    // -------------------------------------------------------------------------
    // 2. 脉冲生成逻辑 (组合逻辑部分)
    // -------------------------------------------------------------------------
    // 当随机数有效时，若随机数小于像素强度，则该像素发放脉冲，结果存入 calculated_spike_vector
    generate
        for (pixel_idx_gen = 0; pixel_idx_gen < P_NUM_INPUT_PIXELS; pixel_idx_gen = pixel_idx_gen + 1) begin : gen_spike_comparators
            assign calculated_spike_vector[pixel_idx_gen] = 
                (all_prngs_valid && (prng_random_outputs[pixel_idx_gen] < i_image_pixel_data[pixel_idx_gen])) ? 1'b1 : 1'b0;
        end
    endgenerate


    // -------------------------------------------------------------------------
    // 3. 输出寄存和有效信号生成 (时序逻辑)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_spike_vector_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
            o_spikes_valid_reg <= 1'b0;
        end else begin
            o_spikes_valid_reg <= all_prngs_valid; // 将PRNG的有效信号延迟一拍作为本模块的输出有效信号，阻断前面的组合逻辑与下游模块耦合过长
            if (all_prngs_valid) begin
                o_spike_vector_reg <= calculated_spike_vector;
            end
        end
    end

endmodule