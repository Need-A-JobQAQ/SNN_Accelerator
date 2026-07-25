// poisson_encoder_tb.v
// (中文注释版 - 使用指定图像数据)
// poisson_encoder 模块的测试平台

`timescale 1ns/1ps

module poisson_encoder_tb;

    // --- 测试平台参数 ---
    localparam CLK_PERIOD = 10; // ns, 时钟周期 (100MHz)

    // DUT 参数
    localparam P_NUM_INPUT_PIXELS         = 784;
    localparam P_PIXEL_INTENSITY_BITS     = 8;
    localparam P_PRNG_LFSR_WIDTH          = 32;
    localparam P_T_MAX                    = 100;

    // 信号声明 - 连接到 DUT 的输入
    reg                                   tb_clk;
    reg                                   tb_rst_n;
    reg                                   tb_i_enable_enc;
    reg  [$clog2(P_T_MAX)-1:0]            tb_i_time_step_t;
    reg  [P_NUM_INPUT_PIXELS-1:0]
         [P_PIXEL_INTENSITY_BITS-1:0]     tb_i_image_pixel_data;
    
    // 信号声明 - 连接到 DUT 的输出
    wire [P_NUM_INPUT_PIXELS-1:0]         tb_o_spike_vector;
    wire                                  tb_o_spikes_valid;

    // --- DUT (poisson_encoder) 实例化 ---
    poisson_encoder #(
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_PIXEL_INTENSITY_BITS     (P_PIXEL_INTENSITY_BITS),
        .P_PRNG_LFSR_WIDTH          (P_PRNG_LFSR_WIDTH),
        .P_T_MAX                    (P_T_MAX)
    ) u_poisson_encoder_inst (
        .clk                    (tb_clk),
        .rst_n                  (tb_rst_n),
        .i_enable_enc           (tb_i_enable_enc),
        .i_time_step_t          (tb_i_time_step_t),
        .i_image_pixel_data     (tb_i_image_pixel_data),
        .o_spike_vector_reg     (tb_o_spike_vector),
        .o_spikes_valid_reg     (tb_o_spikes_valid)
    );

    // 时钟生成
    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    // 激励和检查
    initial begin
        integer i, r, c, pixel_idx; // r for row, c for col
        $display("[%0t ns] SIM_INFO: poisson_encoder_tb 开始仿真 (使用指定图像数据)。", $time);

        // 初始化和复位
        tb_i_enable_enc         = 1'b0;
        tb_i_time_step_t        = {$clog2(P_T_MAX){1'b0}};
        
        // --- 初始化图像数据 (使用你提供的像素值) ---
        // 将28x28的像素数据平铺到 tb_i_image_pixel_data
        // 注意：这里我只手动输入了你提供数据的前几行，你需要将所有784个值填入
        // 或者，更好的方法是使用 $readmemh 从文件加载。

        // 完整数据 (光栅扫描顺序，pixel_idx = row * 28 + col)
        // 为了简洁，我只展示如何手动填充，你需要一个包含所有784个值的数组或文件
        // 以下是一个示例，展示如何填充，你需要用你的实际数据替换
        // 这是一个非常冗长的方式，仅作演示，强烈建议用 $readmemh

        // Row 0 (全部为0)
        for (c = 0; c < 28; c = c + 1) tb_i_image_pixel_data[0*28 + c] = 8'd0;
        // Row 1 (全部为0)
        for (c = 0; c < 28; c = c + 1) tb_i_image_pixel_data[1*28 + c] = 8'd0;
        // Row 2 (全部为0)
        for (c = 0; c < 28; c = c + 1) tb_i_image_pixel_data[2*28 + c] = 8'd0;
        // Row 3 (全部为0)
        for (c = 0; c < 28; c = c + 1) tb_i_image_pixel_data[3*28 + c] = 8'd0;
        // Row 4 (全部为0)
        for (c = 0; c < 28; c = c + 1) tb_i_image_pixel_data[4*28 + c] = 8'd0;
        
        // Row 5 (索引从 5*28 = 140 开始)
        // 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  6 75  0 98 185 178 94 19  0  0  0  0
        for (c = 0; c < 16; c = c + 1) tb_i_image_pixel_data[5*28 + c] = 8'd0;
        tb_i_image_pixel_data[5*28 + 16] = 8'd6;
        tb_i_image_pixel_data[5*28 + 17] = 8'd75;
        tb_i_image_pixel_data[5*28 + 18] = 8'd0;
        tb_i_image_pixel_data[5*28 + 19] = 8'd98;
        tb_i_image_pixel_data[5*28 + 20] = 8'd185;
        tb_i_image_pixel_data[5*28 + 21] = 8'd178;
        tb_i_image_pixel_data[5*28 + 22] = 8'd94;
        tb_i_image_pixel_data[5*28 + 23] = 8'd19;
        for (c = 24; c < 28; c = c + 1) tb_i_image_pixel_data[5*28 + c] = 8'd0;

        // Row 6 (索引从 6*28 = 168 开始)
        // 0  0  0  0  0  0  0  0  0  0  0  0  0 15 111 195 238 94  0 208 249 254 254 116  0  0  0  0
        for (c = 0; c < 13; c = c + 1) tb_i_image_pixel_data[6*28 + c] = 8'd0;
        tb_i_image_pixel_data[6*28 + 13] = 8'd15;
        tb_i_image_pixel_data[6*28 + 14] = 8'd111;
        tb_i_image_pixel_data[6*28 + 15] = 8'd195;
        tb_i_image_pixel_data[6*28 + 16] = 8'd238;
        tb_i_image_pixel_data[6*28 + 17] = 8'd94;
        tb_i_image_pixel_data[6*28 + 18] = 8'd0;
        tb_i_image_pixel_data[6*28 + 19] = 8'd208;
        tb_i_image_pixel_data[6*28 + 20] = 8'd249;
        tb_i_image_pixel_data[6*28 + 21] = 8'd254;
        tb_i_image_pixel_data[6*28 + 22] = 8'd254;
        tb_i_image_pixel_data[6*28 + 23] = 8'd116;
        for (c = 24; c < 28; c = c + 1) tb_i_image_pixel_data[6*28 + c] = 8'd0;
        

        // Row 7 (r=7, pixel_idx_tb from 7*28=196 to 7*28+27=223)
        // 0  0  0  0  0  0  0  0  0  0 20 50 107 197 246 183 25  0  0 81 245 254 249 91  0  0  0  0
        for (c = 0; c < 10; c = c + 1) tb_i_image_pixel_data[7*28 + c] = 8'd0;
        tb_i_image_pixel_data[7*28 + 10] = 8'd20; tb_i_image_pixel_data[7*28 + 11] = 8'd50;
        tb_i_image_pixel_data[7*28 + 12] = 8'd107;tb_i_image_pixel_data[7*28 + 13] = 8'd197;
        tb_i_image_pixel_data[7*28 + 14] = 8'd246;tb_i_image_pixel_data[7*28 + 15] = 8'd183;
        tb_i_image_pixel_data[7*28 + 16] = 8'd25; tb_i_image_pixel_data[7*28 + 17] = 8'd0;
        tb_i_image_pixel_data[7*28 + 18] = 8'd0;  tb_i_image_pixel_data[7*28 + 19] = 8'd81;
        tb_i_image_pixel_data[7*28 + 20] = 8'd245;tb_i_image_pixel_data[7*28 + 21] = 8'd254;
        tb_i_image_pixel_data[7*28 + 22] = 8'd249;tb_i_image_pixel_data[7*28 + 23] = 8'd91;
        for (c = 24; c < 28; c = c + 1) tb_i_image_pixel_data[7*28 + c] = 8'd0;

        // Row 8 (r=8, pixel_idx_tb from 8*28=224 to 8*28+27=251)
        // 0  0  0  0  0  0  0  0 18 84 230 254 254 221 86  0  0  1 125 253 254 178 53  0  0  0  0  0
        for (c = 0; c < 8; c = c + 1) tb_i_image_pixel_data[8*28 + c] = 8'd0;
        tb_i_image_pixel_data[8*28 + 8] = 8'd18;  tb_i_image_pixel_data[8*28 + 9] = 8'd84;
        tb_i_image_pixel_data[8*28 + 10] = 8'd230;tb_i_image_pixel_data[8*28 + 11] = 8'd254;
        tb_i_image_pixel_data[8*28 + 12] = 8'd254;tb_i_image_pixel_data[8*28 + 13] = 8'd221;
        tb_i_image_pixel_data[8*28 + 14] = 8'd86; tb_i_image_pixel_data[8*28 + 15] = 8'd0;
        tb_i_image_pixel_data[8*28 + 16] = 8'd0;  tb_i_image_pixel_data[8*28 + 17] = 8'd1;
        tb_i_image_pixel_data[8*28 + 18] = 8'd125;tb_i_image_pixel_data[8*28 + 19] = 8'd253;
        tb_i_image_pixel_data[8*28 + 20] = 8'd254;tb_i_image_pixel_data[8*28 + 21] = 8'd178;
        tb_i_image_pixel_data[8*28 + 22] = 8'd53;
        for (c = 23; c < 28; c = c + 1) tb_i_image_pixel_data[8*28 + c] = 8'd0;
        
        // Row 9 (r=9, pixel_idx_tb from 9*28=252 to 9*28+27=279)
        // 0  0  0  0  0  0  0  0 133 254 254 217 118  4  0  0 62 202 254 241 131  8  0  0  0  0  0  0
        for (c = 0; c < 8; c = c + 1) tb_i_image_pixel_data[9*28 + c] = 8'd0;
        tb_i_image_pixel_data[9*28 + 8] = 8'd133; tb_i_image_pixel_data[9*28 + 9] = 8'd254;
        tb_i_image_pixel_data[9*28 + 10] = 8'd254;tb_i_image_pixel_data[9*28 + 11] = 8'd217;
        tb_i_image_pixel_data[9*28 + 12] = 8'd118;tb_i_image_pixel_data[9*28 + 13] = 8'd4;
        tb_i_image_pixel_data[9*28 + 14] = 8'd0;  tb_i_image_pixel_data[9*28 + 15] = 8'd0;
        tb_i_image_pixel_data[9*28 + 16] = 8'd62; tb_i_image_pixel_data[9*28 + 17] = 8'd202;
        tb_i_image_pixel_data[9*28 + 18] = 8'd254;tb_i_image_pixel_data[9*28 + 19] = 8'd241;
        tb_i_image_pixel_data[9*28 + 20] = 8'd131;tb_i_image_pixel_data[9*28 + 21] = 8'd8;
        for (c = 22; c < 28; c = c + 1) tb_i_image_pixel_data[9*28 + c] = 8'd0;

        // Row 10 (r=10, pixel_idx_tb from 10*28=280 to 10*28+27=307)
        // 0  0  0  0  0  0  0 107 244 254 213 45  0  0  0 62 240 254 220 29  0  0  0  0  0  0  0  0
        for (c = 0; c < 7; c = c + 1) tb_i_image_pixel_data[10*28 + c] = 8'd0;
        tb_i_image_pixel_data[10*28 + 7] = 8'd107;tb_i_image_pixel_data[10*28 + 8] = 8'd244;
        tb_i_image_pixel_data[10*28 + 9] = 8'd254;tb_i_image_pixel_data[10*28 + 10] = 8'd213;
        tb_i_image_pixel_data[10*28 + 11] = 8'd45;tb_i_image_pixel_data[10*28 + 12] = 8'd0;
        tb_i_image_pixel_data[10*28 + 13] = 8'd0;tb_i_image_pixel_data[10*28 + 14] = 8'd0;
        tb_i_image_pixel_data[10*28 + 15] = 8'd62;tb_i_image_pixel_data[10*28 + 16] = 8'd240;
        tb_i_image_pixel_data[10*28 + 17] = 8'd254;tb_i_image_pixel_data[10*28 + 18] = 8'd220;
        tb_i_image_pixel_data[10*28 + 19] = 8'd29;
        for (c = 20; c < 28; c = c + 1) tb_i_image_pixel_data[10*28 + c] = 8'd0;

        // Row 11 (r=11, pixel_idx_tb from 11*28=308 to 11*28+27=335)
        // 0  0  0  0  0  0 44 246 254 209 49  0  0  0 31 241 254 221 27  0  0  0  0  0  0  0  0  0
        for (c = 0; c < 6; c = c + 1) tb_i_image_pixel_data[11*28 + c] = 8'd0;
        tb_i_image_pixel_data[11*28 + 6] = 8'd44; tb_i_image_pixel_data[11*28 + 7] = 8'd246;
        tb_i_image_pixel_data[11*28 + 8] = 8'd254;tb_i_image_pixel_data[11*28 + 9] = 8'd209;
        tb_i_image_pixel_data[11*28 + 10] = 8'd49;tb_i_image_pixel_data[11*28 + 11] = 8'd0;
        tb_i_image_pixel_data[11*28 + 12] = 8'd0;tb_i_image_pixel_data[11*28 + 13] = 8'd0;
        tb_i_image_pixel_data[11*28 + 14] = 8'd31;tb_i_image_pixel_data[11*28 + 15] = 8'd241;
        tb_i_image_pixel_data[11*28 + 16] = 8'd254;tb_i_image_pixel_data[11*28 + 17] = 8'd221;
        tb_i_image_pixel_data[11*28 + 18] = 8'd27;
        for (c = 19; c < 28; c = c + 1) tb_i_image_pixel_data[11*28 + c] = 8'd0;

        // Row 12 (r=12, pixel_idx_tb from 12*28=336 to 12*28+27=363)
        // 0  0  0  0  0  0 95 254 254 55  0  0  0 17 198 254 218 28  0  0  0  0  0  0  0  0  0  0
        for (c = 0; c < 6; c = c + 1) tb_i_image_pixel_data[12*28 + c] = 8'd0;
        tb_i_image_pixel_data[12*28 + 6] = 8'd95; tb_i_image_pixel_data[12*28 + 7] = 8'd254;
        tb_i_image_pixel_data[12*28 + 8] = 8'd254;tb_i_image_pixel_data[12*28 + 9] = 8'd55;
        tb_i_image_pixel_data[12*28 + 10] = 8'd0; tb_i_image_pixel_data[12*28 + 11] = 8'd0;
        tb_i_image_pixel_data[12*28 + 12] = 8'd0; tb_i_image_pixel_data[12*28 + 13] = 8'd17;
        tb_i_image_pixel_data[12*28 + 14] = 8'd198;tb_i_image_pixel_data[12*28 + 15] = 8'd254;
        tb_i_image_pixel_data[12*28 + 16] = 8'd218;tb_i_image_pixel_data[12*28 + 17] = 8'd28;
        for (c = 18; c < 28; c = c + 1) tb_i_image_pixel_data[12*28 + c] = 8'd0;

        // Row 13 (r=13, pixel_idx_tb from 13*28=364 to 13*28+27=391)
        // 0  0  0  0  0  0 27 219 254 233 144 39 42 204 254 205 10  0  0  0  0  0  0  0  0  0  0  0
        for (c = 0; c < 6; c = c + 1) tb_i_image_pixel_data[13*28 + c] = 8'd0;
        tb_i_image_pixel_data[13*28 + 6] = 8'd27; tb_i_image_pixel_data[13*28 + 7] = 8'd219;
        tb_i_image_pixel_data[13*28 + 8] = 8'd254;tb_i_image_pixel_data[13*28 + 9] = 8'd233;
        tb_i_image_pixel_data[13*28 + 10] = 8'd144;tb_i_image_pixel_data[13*28 + 11] = 8'd39;
        tb_i_image_pixel_data[13*28 + 12] = 8'd42;tb_i_image_pixel_data[13*28 + 13] = 8'd204;
        tb_i_image_pixel_data[13*28 + 14] = 8'd254;tb_i_image_pixel_data[13*28 + 15] = 8'd205;
        tb_i_image_pixel_data[13*28 + 16] = 8'd10;
        for (c = 17; c < 28; c = c + 1) tb_i_image_pixel_data[13*28 + c] = 8'd0;

        // Row 14 (r=14, pixel_idx_tb from 14*28=392 to 14*28+27=419)
        // 0  0  0  0  0  0  0 115 248 254 254 244 233 254 223 24  0  0  0  0  0  0  0  0  0  0  0  0
        for (c = 0; c < 7; c = c + 1) tb_i_image_pixel_data[14*28 + c] = 8'd0;
        tb_i_image_pixel_data[14*28 + 7] = 8'd115;tb_i_image_pixel_data[14*28 + 8] = 8'd248;
        tb_i_image_pixel_data[14*28 + 9] = 8'd254;tb_i_image_pixel_data[14*28 + 10] = 8'd254;
        tb_i_image_pixel_data[14*28 + 11] = 8'd244;tb_i_image_pixel_data[14*28 + 12] = 8'd233;
        tb_i_image_pixel_data[14*28 + 13] = 8'd254;tb_i_image_pixel_data[14*28 + 14] = 8'd223;
        tb_i_image_pixel_data[14*28 + 15] = 8'd24;
        for (c = 16; c < 28; c = c + 1) tb_i_image_pixel_data[14*28 + c] = 8'd0;

        // Row 15 (r=15, pixel_idx_tb_init from 15*28=420 to 15*28+27=447)
        // 0  0  0  0  0  0  0  0  38 84 168 245 254 254 254 207 115  9  0  0  0  0  0  0  0  0  0  0
        for (c = 0; c < 8; c = c + 1) tb_i_image_pixel_data[15*28 + c] = 8'd0;
        tb_i_image_pixel_data[15*28 + 8]  = 8'd38;  tb_i_image_pixel_data[15*28 + 9]  = 8'd84;
        tb_i_image_pixel_data[15*28 + 10] = 8'd168; tb_i_image_pixel_data[15*28 + 11] = 8'd245;
        tb_i_image_pixel_data[15*28 + 12] = 8'd254; tb_i_image_pixel_data[15*28 + 13] = 8'd254;
        tb_i_image_pixel_data[15*28 + 14] = 8'd254; tb_i_image_pixel_data[15*28 + 15] = 8'd207;
        tb_i_image_pixel_data[15*28 + 16] = 8'd115; tb_i_image_pixel_data[15*28 + 17] = 8'd9;
        for (c = 18; c < 28; c = c + 1) tb_i_image_pixel_data[15*28 + c] = 8'd0;

        // Row 16 (r=16, pixel_idx_tb_init from 16*28=448 to 16*28+27=475)
        // 0  0  0  0  0  0  0  0  0  0  14 236 254 230 163 237 244 211 80  1  0  0  0  0  0  0  0  0
        for (c = 0; c < 10; c = c + 1) tb_i_image_pixel_data[16*28 + c] = 8'd0;
        tb_i_image_pixel_data[16*28 + 10] = 8'd14;  tb_i_image_pixel_data[16*28 + 11] = 8'd236;
        tb_i_image_pixel_data[16*28 + 12] = 8'd254; tb_i_image_pixel_data[16*28 + 13] = 8'd230;
        tb_i_image_pixel_data[16*28 + 14] = 8'd163; tb_i_image_pixel_data[16*28 + 15] = 8'd237;
        tb_i_image_pixel_data[16*28 + 16] = 8'd244; tb_i_image_pixel_data[16*28 + 17] = 8'd211;
        tb_i_image_pixel_data[16*28 + 18] = 8'd80;  tb_i_image_pixel_data[16*28 + 19] = 8'd1;
        for (c = 20; c < 28; c = c + 1) tb_i_image_pixel_data[16*28 + c] = 8'd0;

        // Row 17 (r=17, pixel_idx_tb_init from 17*28=476 to 17*28+27=503)
        // 0  0  0  0  0  0  0  0  0  0  99 254 254 99  0  0  37 225 254 130  8  0  0  0  0  0  0  0
        for (c = 0; c < 10; c = c + 1) tb_i_image_pixel_data[17*28 + c] = 8'd0;
        tb_i_image_pixel_data[17*28 + 10] = 8'd99;  tb_i_image_pixel_data[17*28 + 11] = 8'd254;
        tb_i_image_pixel_data[17*28 + 12] = 8'd254; tb_i_image_pixel_data[17*28 + 13] = 8'd99;
        tb_i_image_pixel_data[17*28 + 14] = 8'd0;   tb_i_image_pixel_data[17*28 + 15] = 8'd0;
        tb_i_image_pixel_data[17*28 + 16] = 8'd37;  tb_i_image_pixel_data[17*28 + 17] = 8'd225;
        tb_i_image_pixel_data[17*28 + 18] = 8'd254; tb_i_image_pixel_data[17*28 + 19] = 8'd130;
        tb_i_image_pixel_data[17*28 + 20] = 8'd8;
        for (c = 21; c < 28; c = c + 1) tb_i_image_pixel_data[17*28 + c] = 8'd0;

        // Row 18 (r=18, pixel_idx_tb_init from 18*28=504 to 18*28+27=531)
        // 0  0  0  0  0  0  0  0  0  0  71 254 229 12  0  0  0  2 170 254 51  0  0  0  0  0  0  0
        for (c = 0; c < 10; c = c + 1) tb_i_image_pixel_data[18*28 + c] = 8'd0;
        tb_i_image_pixel_data[18*28 + 10] = 8'd71;  tb_i_image_pixel_data[18*28 + 11] = 8'd254;
        tb_i_image_pixel_data[18*28 + 12] = 8'd229; tb_i_image_pixel_data[18*28 + 13] = 8'd12;
        tb_i_image_pixel_data[18*28 + 14] = 8'd0;   tb_i_image_pixel_data[18*28 + 15] = 8'd0;
        tb_i_image_pixel_data[18*28 + 16] = 8'd0;   tb_i_image_pixel_data[18*28 + 17] = 8'd2;
        tb_i_image_pixel_data[18*28 + 18] = 8'd170; tb_i_image_pixel_data[18*28 + 19] = 8'd254;
        tb_i_image_pixel_data[18*28 + 20] = 8'd51;
        for (c = 21; c < 28; c = c + 1) tb_i_image_pixel_data[18*28 + c] = 8'd0;

        // Row 19 (r=19, pixel_idx_tb_init from 19*28=532 to 19*28+27=559)
        // 0  0  0  0  0  0  0  0  0  0  96 254 254 18  0  0  0  0  81 254 198  0  0  0  0  0  0  0
        for (c = 0; c < 10; c = c + 1) tb_i_image_pixel_data[19*28 + c] = 8'd0;
        tb_i_image_pixel_data[19*28 + 10] = 8'd96;  tb_i_image_pixel_data[19*28 + 11] = 8'd254;
        tb_i_image_pixel_data[19*28 + 12] = 8'd254; tb_i_image_pixel_data[19*28 + 13] = 8'd18;
        tb_i_image_pixel_data[19*28 + 14] = 8'd0;   tb_i_image_pixel_data[19*28 + 15] = 8'd0;
        tb_i_image_pixel_data[19*28 + 16] = 8'd0;   tb_i_image_pixel_data[19*28 + 17] = 8'd0;
        tb_i_image_pixel_data[19*28 + 18] = 8'd81;  tb_i_image_pixel_data[19*28 + 19] = 8'd254;
        tb_i_image_pixel_data[19*28 + 20] = 8'd198;
        for (c = 21; c < 28; c = c + 1) tb_i_image_pixel_data[19*28 + c] = 8'd0;

        // Row 20 (r=20, pixel_idx_tb_init from 20*28=560 to 20*28+27=587)
        // 0  0  0  0  0  0  0  0  0  0  80 254 254 18  0  0  0  0 131 254 119  0  0  0  0  0  0  0
        for (c = 0; c < 10; c = c + 1) tb_i_image_pixel_data[20*28 + c] = 8'd0;
        tb_i_image_pixel_data[20*28 + 10] = 8'd80;  tb_i_image_pixel_data[20*28 + 11] = 8'd254;
        tb_i_image_pixel_data[20*28 + 12] = 8'd254; tb_i_image_pixel_data[20*28 + 13] = 8'd18;
        tb_i_image_pixel_data[20*28 + 14] = 8'd0;   tb_i_image_pixel_data[20*28 + 15] = 8'd0;
        tb_i_image_pixel_data[20*28 + 16] = 8'd0;   tb_i_image_pixel_data[20*28 + 17] = 8'd0; // Note: your data showed _0, assuming 0
        tb_i_image_pixel_data[20*28 + 18] = 8'd131; tb_i_image_pixel_data[20*28 + 19] = 8'd254;
        tb_i_image_pixel_data[20*28 + 20] = 8'd119;
        for (c = 21; c < 28; c = c + 1) tb_i_image_pixel_data[20*28 + c] = 8'd0;

        // Row 21 (r=21, pixel_idx_tb_init from 21*28=588 to 21*28+27=615)
        // 0  0  0  0  0  0  0  0  0  0   5 214 254 78  0  0  0  1 183 244  63  0  0  0  0  0  0  0
        for (c = 0; c < 10; c = c + 1) tb_i_image_pixel_data[21*28 + c] = 8'd0;
        tb_i_image_pixel_data[21*28 + 10] = 8'd5;   tb_i_image_pixel_data[21*28 + 11] = 8'd214;
        tb_i_image_pixel_data[21*28 + 12] = 8'd254; tb_i_image_pixel_data[21*28 + 13] = 8'd78;
        tb_i_image_pixel_data[21*28 + 14] = 8'd0;   tb_i_image_pixel_data[21*28 + 15] = 8'd0;
        tb_i_image_pixel_data[21*28 + 16] = 8'd0;   tb_i_image_pixel_data[21*28 + 17] = 8'd1;
        tb_i_image_pixel_data[21*28 + 18] = 8'd183; tb_i_image_pixel_data[21*28 + 19] = 8'd244;
        tb_i_image_pixel_data[21*28 + 20] = 8'd63;
        for (c = 21; c < 28; c = c + 1) tb_i_image_pixel_data[21*28 + c] = 8'd0;

        // Row 22 (r=22, pixel_idx_tb_init from 22*28=616 to 22*28+27=643)
        // 0  0  0  0  0  0  0  0  0  0   0  64 253 223 24  0  2 126 254 178   0  0  0  0  0  0  0  0
        for (c = 0; c < 10; c = c + 1) tb_i_image_pixel_data[22*28 + c] = 8'd0;
        tb_i_image_pixel_data[22*28 + 10] = 8'd0; // This was 0
        tb_i_image_pixel_data[22*28 + 11] = 8'd64; tb_i_image_pixel_data[22*28 + 12] = 8'd253;
        tb_i_image_pixel_data[22*28 + 13] = 8'd223;tb_i_image_pixel_data[22*28 + 14] = 8'd24;
        tb_i_image_pixel_data[22*28 + 15] = 8'd0;  tb_i_image_pixel_data[22*28 + 16] = 8'd2;
        tb_i_image_pixel_data[22*28 + 17] = 8'd126;tb_i_image_pixel_data[22*28 + 18] = 8'd254;
        tb_i_image_pixel_data[22*28 + 19] = 8'd178;tb_i_image_pixel_data[22*28 + 20] = 8'd0;
        for (c = 21; c < 28; c = c + 1) tb_i_image_pixel_data[22*28 + c] = 8'd0;

        // Row 23 (r=23, pixel_idx_tb_init from 23*28=644 to 23*28+27=671)
        // 0  0  0  0  0  0  0  0  0  0   0   0  87 238 222 119 177 254 217  27   0  0  0  0  0  0  0  0
        for (c = 0; c < 12; c = c + 1) tb_i_image_pixel_data[23*28 + c] = 8'd0; // First 12 are 0
        tb_i_image_pixel_data[23*28 + 12] = 8'd87; tb_i_image_pixel_data[23*28 + 13] = 8'd238;
        tb_i_image_pixel_data[23*28 + 14] = 8'd222;tb_i_image_pixel_data[23*28 + 15] = 8'd119;
        tb_i_image_pixel_data[23*28 + 16] = 8'd177;tb_i_image_pixel_data[23*28 + 17] = 8'd254;
        tb_i_image_pixel_data[23*28 + 18] = 8'd217;tb_i_image_pixel_data[23*28 + 19] = 8'd27;
        for (c = 20; c < 28; c = c + 1) tb_i_image_pixel_data[23*28 + c] = 8'd0;

        // Row 24 (r=24, pixel_idx_tb_init from 24*28=672 to 24*28+27=699)
        // 0  0  0  0  0  0  0  0  0  0   0   0   0  18 154 196 196 101  25   0   0  0  0  0  0  0  0
        for (c = 0; c < 13; c = c + 1) tb_i_image_pixel_data[24*28 + c] = 8'd0; // First 13 are 0
        tb_i_image_pixel_data[24*28 + 13] = 8'd18; tb_i_image_pixel_data[24*28 + 14] = 8'd154;
        tb_i_image_pixel_data[24*28 + 15] = 8'd196;tb_i_image_pixel_data[24*28 + 16] = 8'd196;
        tb_i_image_pixel_data[24*28 + 17] = 8'd101;tb_i_image_pixel_data[24*28 + 18] = 8'd25;
        for (c = 19; c < 28; c = c + 1) tb_i_image_pixel_data[24*28 + c] = 8'd0;
        
        // Row 25, 26, 27 (r=25, 26, 27 are all zeros)
        for (r = 25; r < 28; r = r + 1) begin
            for (c = 0; c < 28; c = c + 1) begin
                tb_i_image_pixel_data[r*28 + c] = 8'd0;
            end
        end

        $display("[%0t ns] SIM_INFO: 图像像素数据已（部分按你的示例）初始化.", $time);
        
        tb_rst_n = 1'b0; 
        repeat(5) @(posedge tb_clk);
        tb_rst_n = 1'b1; 
        $display("[%0t ns] SIM_INFO: 复位已释放.", $time);
        repeat(2) @(posedge tb_clk);

        // --- 模拟几个SNN时间步的编码过程 ---
        for (i = 0; i < 15; i = i + 1) begin 
            tb_i_time_step_t = i[$clog2(P_T_MAX)-1:0]; 
            tb_i_enable_enc  = 1'b1; 
            $display("[%0t ns] SIM_INFO: 使能编码器 (i_enable_enc=1) 用于时间步 t=%0d.", $time, tb_i_time_step_t);
            
            @(posedge tb_clk); 
            tb_i_enable_enc  = 1'b0; 
            
            // 由于你的 poisson_encoder.v 设计，o_spikes_valid_reg 和 o_spike_vector_reg
            // 会在 i_enable_enc 有效的那个周期的时钟沿之后更新，并在下一个周期初稳定。
            // 所以我们在这个时钟沿之后（即 tb_i_enable_enc 刚变回0的这一拍）观察
            // @(posedge tb_clk); // 不需要额外等待这一拍，因为DUT的输出寄存器是在上一个enable周期结束时更新的
            // 这意味着tb_o_spikes_valid现在应该反映的是上一个使能周期的结果。
            
            if (tb_o_spikes_valid) begin
                $display("[%0t ns] SIM_INFO: 时间步 t=%0d (数据对应上一使能周期), 编码器输出有效! 部分脉冲:", 
                         $time, tb_i_time_step_t); // tb_i_time_step_t 此时是 "当前" 的t, 但数据是前一拍的
                $display("    Spikes[16*5+16 to 16*5+23] (Row 5, mid-to-right): %b", tb_o_spike_vector[5*28+23 : 5*28+16]);
                $display("    Spikes[16*6+13 to 16*6+20] (Row 6, mid-to-right): %b", tb_o_spike_vector[6*28+20 : 6*28+13]);
            end else begin
                 $display("[%0t ns] SIM_INFO: 时间步 t=%0d, 编码器输出【无效】(或正在等待第一个有效输出).", 
                         $time, tb_i_time_step_t);
            end
             repeat(1) @(posedge tb_clk); // 使能脉冲之间间隔一个低电平周期
        end

        repeat(10) @(posedge tb_clk);
        $display("[%0t ns] SIM_INFO: poisson_encoder_tb 仿真结束。", $time);
        $finish;
    end

endmodule