// prng_lfsr_tb.v
// (中文注释版)
// prng_lfsr 模块的测试平台

`timescale 1ns/1ps

module prng_lfsr_tb;

    // --- 测试平台参数 ---
    localparam CLK_PERIOD = 10; // ns, 时钟周期 (100MHz)

    // DUT 参数 (可以根据需要修改以测试不同的配置)
    localparam TB_INSTANCE_ID_0             = 0;
    localparam TB_INSTANCE_ID_1             = 1;
    localparam TB_PRNG_LFSR_WIDTH           = 32;
    localparam TB_POISSON_ENCODER_PRNG_BITS = 8;

    // 信号声明 - 连接到 DUT 的输入
    reg                               clk_tb;
    reg                               rst_n_tb;
    reg                               i_enable_prng_tb;
    
    // 信号声明 - 连接到 DUT 的输出 (为两个实例准备)
    wire [TB_POISSON_ENCODER_PRNG_BITS-1:0] o_random_output_0_w;
    wire                                o_random_output_valid_0_w;

    wire [TB_POISSON_ENCODER_PRNG_BITS-1:0] o_random_output_1_w;
    wire                                o_random_output_valid_1_w;

    // --- DUT (prng_lfsr) 实例化 ---
    // 实例0
    prng_lfsr #(
        .INSTANCE_ID                  (TB_INSTANCE_ID_0),
        .P_PRNG_LFSR_WIDTH              (TB_PRNG_LFSR_WIDTH),
        .P_PIXEL_INTENSITY_BITS    (TB_POISSON_ENCODER_PRNG_BITS)
    ) u_prng_inst_0 (
        .clk                    (clk_tb),
        .rst_n                  (rst_n_tb),
        .i_enable_prng          (i_enable_prng_tb),
        .o_random_output_reg    (o_random_output_0_w),
        .o_random_output_valid  (o_random_output_valid_0_w)
    );

    // 实例1 (使用不同的INSTANCE_ID以观察不同的序列)
    prng_lfsr #(
        .INSTANCE_ID                  (TB_INSTANCE_ID_1),
        .P_PRNG_LFSR_WIDTH              (TB_PRNG_LFSR_WIDTH),
        .P_PIXEL_INTENSITY_BITS    (TB_POISSON_ENCODER_PRNG_BITS)
    ) u_prng_inst_1 (
        .clk                    (clk_tb),
        .rst_n                  (rst_n_tb),
        .i_enable_prng          (i_enable_prng_tb),
        .o_random_output_reg    (o_random_output_1_w),
        .o_random_output_valid  (o_random_output_valid_1_w)
    );


    // 时钟生成
    initial begin
        clk_tb = 1'b0;
        forever #(CLK_PERIOD/2) clk_tb = ~clk_tb;
    end

    // 激励和检查
    initial begin
        $display("[%0t ns] SIM_INFO: prng_lfsr_tb 开始仿真。", $time);

        // 1. 初始化和复位
        i_enable_prng_tb = 1'b0;
        rst_n_tb = 1'b0; // 激活复位
        repeat(5) @(posedge clk_tb);
        rst_n_tb = 1'b1; // 释放复位
        $display("[%0t ns] SIM_INFO: 复位已释放.", $time);
        @(posedge clk_tb); // 等待一个周期确保复位值稳定

        $display("[%0t ns] SIM_INFO: PRNG 0 (ID %0d) 初始输出 (应为0/无效): Rand0=%h, Valid0=%b", $time, TB_INSTANCE_ID_0, o_random_output_0_w, o_random_output_valid_0_w);
        $display("[%0t ns] SIM_INFO: PRNG 1 (ID %0d) 初始输出 (应为0/无效): Rand1=%h, Valid1=%b", $time, TB_INSTANCE_ID_1, o_random_output_1_w, o_random_output_valid_1_w);
        
        // 2. 使能PRNG并观察几个周期的输出
        i_enable_prng_tb = 1'b1;
        $display("[%0t ns] SIM_INFO: 使能 PRNG (i_enable_prng_tb = 1).", $time);

        for (integer i = 0; i < 100; i = i + 1) begin
            @(posedge clk_tb);
            // o_random_output_valid 会在 i_enable_prng 有效的那个周期的下一个周期变高
            // 所以，我们在 i_enable_prng 置高后的第一个时钟沿之后开始观察有效数据
            $display("[%0t ns] SIM_INFO: 周期 %2d - PRNG 0 (ID %0d): Rand0=%h (Valid=%b) | PRNG 1 (ID %0d): Rand1=%h (Valid=%b)", 
                     $time, i+1, TB_INSTANCE_ID_0, o_random_output_0_w, o_random_output_valid_0_w,
                     TB_INSTANCE_ID_1, o_random_output_1_w, o_random_output_valid_1_w);
        end

        // 3. 禁用PRNG观察
        i_enable_prng_tb = 1'b0;
        $display("[%0t ns] SIM_INFO: 禁用 PRNG (i_enable_prng_tb = 0).", $time);
        for (integer i = 0; i < 3; i = i + 1) begin
            @(posedge clk_tb);
            $display("[%0t ns] SIM_INFO: 周期 %2d (禁用后) - PRNG 0 (ID %0d): Rand0=%h (Valid=%b) | PRNG 1 (ID %0d): Rand1=%h (Valid=%b)", 
                     $time, i+1, TB_INSTANCE_ID_0, o_random_output_0_w, o_random_output_valid_0_w,
                     TB_INSTANCE_ID_1, o_random_output_1_w, o_random_output_valid_1_w);
        end

        repeat(5) @(posedge clk_tb);
        $display("[%0t ns] SIM_INFO: prng_lfsr_tb 仿真结束。", $time);
        $finish;
    end

endmodule