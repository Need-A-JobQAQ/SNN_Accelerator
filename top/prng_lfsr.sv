// prng_lfsr.v
// 实现一个基于线性反馈移位寄存器 (LFSR) 的伪随机数生成器。
// 它能够根据 INSTANCE_ID 自我初始化种子，并生成 P_PIXEL_INTENSITY_BITS 位的随机数。
// LFSR结构: 采用斐波那契LFSR，通过对寄存器当前状态的特定几位（抽头）进行异或运算来产生下一位。

module prng_lfsr #(
    // --- 用户可配置的逻辑参数 ---
    parameter INSTANCE_ID                    = 0,   // 用于生成独特初始种子的实例ID，不同的实例应有不同的ID
    parameter P_PRNG_LFSR_WIDTH              = 32,  // LFSR内部状态寄存器的位宽，32位以获得较长的伪随机序列
    parameter P_POISSON_ENCODER_PRNG_BITS    = 8    // PRNG模块输出的随机数的位宽，8位以匹配像素强度
) (
    input wire                                      clk,                    // 时钟信号
    input wire                                      rst_n,                  // 异步低电平复位信号
    input wire                                      i_enable_prng,          // 当为高时，PRNG在时钟上升沿更新其状态

    output reg [P_POISSON_ENCODER_PRNG_BITS-1:0]    o_random_output_reg,    // 输出的随机数
    output reg                                      o_random_output_valid   // 指示 o_random_output_reg 有效的信号 (单周期脉冲)
);

    // LFSR 内部状态寄存器
    reg [P_PRNG_LFSR_WIDTH-1:0] lfsr_reg;

    // LFSR 反馈逻辑 (斐波那契 LFSR)
    // 新的最低有效位 (LSB) 是由当前状态的某些位的异或产生的，然后整个寄存器逻辑左移。
    // 下面的抽头是针对 P_PRNG_LFSR_WIDTH = 32 位时的一个常见最大长度多项式：
    // 为确保最大长度，这里使用一组公认的32位LFSR抽头(这是一个例子，实际常用的可能是其他组合)(对应多项式 x^32 + x^22 + x^2 + x + 1 的一种实现形式)：
    // 当寄存器逻辑左移，反馈位送入 bit[0] 时，反馈位 = lfsr_reg[31]^lfsr_reg[21]^lfsr_reg[1]^lfsr_reg[0] (若0是MSB, 31是LSB)
    // 我们这里采用：寄存器逻辑左移，新位进入LSB。反馈由MSB端的几位决定。
    // 使用的抽头 (0-indexed, for P_PRNG_LFSR_WIDTH = 32): lfsr_reg[31], lfsr_reg[30], lfsr_reg[28], lfsr_reg[24]

    wire feedback_bit;
    // 下面的抽头是针对32位LFSR的一个示例，如果 P_PRNG_LFSR_WIDTH 参数改变，
    //  或者需要严格的最大长度序列，此反馈逻辑需要根据新的位宽选择合适的多项式和抽头进行修改！
    assign feedback_bit = (P_PRNG_LFSR_WIDTH == 32) ?
                          (lfsr_reg[0] ^ lfsr_reg[1] ^ lfsr_reg[21] ^ lfsr_reg[31]) : // 32位抽头
                          lfsr_reg[P_PRNG_LFSR_WIDTH-1]; // 对于其他位宽的简化回退逻辑 (不是最优的)


    // LFSR状态更新 和 随机数输出
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 基于 INSTANCE_ID 和一个基础值生成初始种子，并确保种子不为全零
            // 全零状态会导致LFSR卡死。
            automatic logic [P_PRNG_LFSR_WIDTH-1:0] base_seed;
            automatic logic [P_PRNG_LFSR_WIDTH-1:0] id_contribution;
            automatic logic [P_PRNG_LFSR_WIDTH-1:0] initial_seed_calc;
            base_seed = P_PRNG_LFSR_WIDTH'('hACE1F00D); // 一个常用的非零基础值
            // 更复杂地混合 INSTANCE_ID 以产生更大差异
            // 1. 将 INSTANCE_ID 乘以一个大的奇数（帮助打乱比特位）
            id_contribution = P_PRNG_LFSR_WIDTH'(INSTANCE_ID) * P_PRNG_LFSR_WIDTH'('h10101011); // 'h10101011 是一个奇数
            // 2. 与INSTANCE_ID的不同移位版本进行异或，进一步扩散影响
            id_contribution = id_contribution ^ 
                            (P_PRNG_LFSR_WIDTH'(INSTANCE_ID) << 7) ^ 
                            (P_PRNG_LFSR_WIDTH'(INSTANCE_ID) << 15) ^
                            (P_PRNG_LFSR_WIDTH'(INSTANCE_ID) << 23); // 选择不同的移位数
            initial_seed_calc = base_seed ^ id_contribution; // 将基础种子与INSTANCE_ID的贡献混合
            if (initial_seed_calc == {P_PRNG_LFSR_WIDTH{1'b0}}) begin
                // 如果不幸仍然为0，使用一个更鲁棒的备用方案
                lfsr_reg <= P_PRNG_LFSR_WIDTH'(('hDEADBEEF + INSTANCE_ID + (INSTANCE_ID >> 3) + 1));
                if (lfsr_reg == {P_PRNG_LFSR_WIDTH{1'b0}}) begin // 极小概率下的最终保障
                    lfsr_reg <= P_PRNG_LFSR_WIDTH'(INSTANCE_ID + 1'b1); // 确保至少为1（如果ID为0）或ID+1
                     if (lfsr_reg == {P_PRNG_LFSR_WIDTH{1'b0}}) lfsr_reg <= 1; // 绝对的最后防线
                end
            end else begin
                lfsr_reg <= initial_seed_calc;
            end
            o_random_output_reg   <= {P_POISSON_ENCODER_PRNG_BITS{1'b0}}; // 复位时输出0
            o_random_output_valid <= 1'b0;                                // 复位时输出无效                    
        end else begin
            o_random_output_valid <= 1'b0;  // 在每个时钟周期的开始，默认将valid信号拉低

            if (i_enable_prng) begin
                lfsr_reg <= {lfsr_reg[P_PRNG_LFSR_WIDTH-2:0], feedback_bit};  // LFSR逻辑左移，新的LSB是feedback_bit
                // 选择输出位：例如，取LFSR状态的高位作为随机数输出
                // 这个选择可以是高位、低位或其他组合，取决于应用对随机数特性的需求。
                o_random_output_reg   <= lfsr_reg[P_PRNG_LFSR_WIDTH-1 : P_PRNG_LFSR_WIDTH - P_POISSON_ENCODER_PRNG_BITS]; 
                o_random_output_valid <= 1'b1; 
            // 如果 i_enable_prng 为低，lfsr_reg 保持其值，o_random_output_reg 保持其值，o_random_output_valid 为低。
            end
        end
    end

endmodule