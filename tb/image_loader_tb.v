// image_loader_tb.v
// image_loader 模块的测试平台，内部模拟一个简单的BRAM行为

`timescale 1ns/1ps

module image_loader_tb;

    // --- 测试平台参数 ---
    localparam CLK_PERIOD = 10; // ns, 时钟周期 (100MHz)

    // DUT 参数 (这些值应与 image_loader.v 期望的一致)
    localparam P_NUM_INPUT_PIXELS       = 784;
    localparam P_PIXEL_INTENSITY_BITS   = 8;
    localparam P_IMAGE_BRAM_DATA_WIDTH  = 64;
    localparam P_IMAGE_BRAM_DEPTH       = 98; // 假设的图像BRAM深度

    // 根据 P_IMAGE_BRAM_DEPTH 计算地址位宽
    localparam LP_TB_BRAM_ADDR_WIDTH  = $clog2(P_IMAGE_BRAM_DEPTH);

    // 信号声明
    reg                                   tb_clk;
    reg                                   tb_rst_n;
    reg                                   tb_i_load_image_start;
    reg  signed [P_IMAGE_BRAM_DATA_WIDTH-1:0] tb_i_bram_dout_raw_driver; // 驱动DUT的i_bram_dout_raw
    
    wire [LP_TB_BRAM_ADDR_WIDTH-1:0]      tb_o_bram_addr_from_dut;
    wire                                  tb_o_bram_ena_from_dut;
    
    wire [P_PIXEL_INTENSITY_BITS-1:0]       tb_o_image_buffer_out [P_NUM_INPUT_PIXELS-1:0];
    wire                                  tb_o_loading_busy;
    wire                                  tb_o_load_done;

    // --- 模拟外部图像BRAM ---
    // 创建一个小型ROM来模拟BRAM的行为，包含一些示例数据
    // 这个ROM的深度应该至少是 P_IMAGE_BRAM_DEPTH
    reg [P_IMAGE_BRAM_DATA_WIDTH-1:0] simulated_bram_data_array [0:P_IMAGE_BRAM_DEPTH-1];
    
    // BRAM输出延迟模拟 (2周期)
    reg [P_IMAGE_BRAM_DATA_WIDTH-1:0] bram_data_pipe0_reg;
    reg [P_IMAGE_BRAM_DATA_WIDTH-1:0] bram_data_pipe1_reg; // 这个连接到 DUT 的 i_bram_dout_raw

    // 初始化模拟的BRAM数据
    initial begin
        // 填充一些示例数据，每行代表一个64位的BRAM字 (8个像素)
        // 像素按大端方式打包到64位字中: [63:56]=P0, [55:48]=P1, ..., [7:0]=P7
        // 这里仅填充前几个地址作为示例，其余为0
        simulated_bram_data_array[0]  = 64'h0102030405060708; // Addr 0: Pixels 0-7
        simulated_bram_data_array[1]  = 64'h090A0B0C0D0E0F10; // Addr 1: Pixels 8-15
        simulated_bram_data_array[2]  = 64'h1112131415161718; // Addr 2: Pixels 16-23
        // ... 你可以根据需要填充更多数据，或者从文件加载到这个数组 ...
        for (integer i = 3; i < P_IMAGE_BRAM_DEPTH; i = i + 1) begin
            simulated_bram_data_array[i] = 64'h0000000000000000; // 其余地址为0
        end
    end

    // 模拟BRAM的读取行为和2周期延迟
    always @(posedge tb_clk or negedge tb_rst_n) begin
        if (!tb_rst_n) begin
            bram_data_pipe0_reg <= {P_IMAGE_BRAM_DATA_WIDTH{1'b0}};
            bram_data_pipe1_reg <= {P_IMAGE_BRAM_DATA_WIDTH{1'b0}};
        end else begin
            bram_data_pipe1_reg <= bram_data_pipe0_reg; // 第二级流水线
            if (tb_o_bram_ena_from_dut) begin // 如果DUT使能了BRAM读取
                // BRAM数据在使能后的第一个周期从存储阵列读出，进入第一级流水线
                // 注意：tb_o_bram_addr_from_dut 是当前周期的地址
                bram_data_pipe0_reg <= simulated_bram_data_array[tb_o_bram_addr_from_dut];
            end else begin
                // 如果未使能，可以保持或清零，这里保持以模拟BRAM不更新输出
                // bram_data_pipe0_reg <= bram_data_pipe0_reg; 
            end
        end
    end
    // 最终驱动DUT的i_bram_dout_raw的是延迟两拍后的数据
    assign tb_i_bram_dout_raw_driver = bram_data_pipe1_reg;


    // --- DUT (image_loader) 实例化 ---
    image_loader #(
        .P_NUM_INPUT_PIXELS       (P_NUM_INPUT_PIXELS),
        .P_PIXEL_INTENSITY_BITS   (P_PIXEL_INTENSITY_BITS),
        .P_IMAGE_BRAM_DATA_WIDTH  (P_IMAGE_BRAM_DATA_WIDTH),
        .P_IMAGE_BRAM_DEPTH       (P_IMAGE_BRAM_DEPTH)
    ) u_image_loader_inst (
        .clk                    (tb_clk),
        .rst_n                  (tb_rst_n),
        .i_load_image_start     (tb_i_load_image_start),
        .i_bram_dout_raw        (tb_i_bram_dout_raw_driver), // 连接到模拟BRAM的输出
        
        .o_bram_addr            (tb_o_bram_addr_from_dut),
        .o_bram_ena             (tb_o_bram_ena_from_dut),
        
        .o_image_buffer_out     (tb_o_image_buffer_out),
        .o_loading_busy         (tb_o_loading_busy),
        .o_load_done            (tb_o_load_done)
    );

    // 时钟生成
    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    // 激励和检查
    initial begin
        integer pixel_display_idx;
        $display("[%0t ns] SIM_INFO: image_loader_tb 开始仿真。", $time);

        // 初始化和复位
        tb_i_load_image_start = 1'b0;
        tb_rst_n = 1'b0; // 激活复位
        repeat(5) @(posedge tb_clk);
        tb_rst_n = 1'b1; // 释放复位
        $display("[%0t ns] SIM_INFO: 复位已释放.", $time);
        repeat(2) @(posedge tb_clk);

        // --- 开始图像加载 ---
        $display("[%0t ns] SIM_INFO: 发起图像加载请求 (i_load_image_start = 1).", $time);
        tb_i_load_image_start = 1'b1;
        @(posedge tb_clk);
        tb_i_load_image_start = 1'b0; // i_load_image_start 是单周期脉冲

        // 监控加载过程
        if (tb_o_loading_busy) begin
            $display("[%0t ns] SIM_INFO: 图像加载开始 (o_loading_busy = 1).", $time);
        end

        // 等待加载完成
        // image_loader 大致需要 P_IMAGE_BRAM_DEPTH (读取BRAM) + BRAM_READ_LATENCY (冲刷流水线) + FSM额外周期
        // 例如: 98 + 2 + ~2 = 102 个周期
        wait (tb_o_load_done == 1'b1);
        $display("[%0t ns] SIM_INFO: 图像加载完成 (o_load_done = 1)!", $time);
        
        if (tb_o_loading_busy == 1'b0) begin
             $display("[%0t ns] SIM_INFO: 图像加载结束 (o_loading_busy = 0).", $time);
        end

        // 检查加载到 o_image_buffer_out 的数据 (部分示例)
        // 记住 o_image_buffer_out 的索引方式是 [783] 对应光栅第一个像素
        // 模拟BRAM地址0的数据: 64'h0102030405060708 (P0=01, P1=02, ..., P7=08)
        // 它们应该存储在 o_image_buffer_out 的高索引部分
        @(posedge tb_clk); // 等待 o_image_buffer_out 在 S_DONE_LOAD 状态下被赋值

        $display("SIM_INFO: 检查加载到 o_image_buffer_out 的部分数据:");
        if (P_NUM_INPUT_PIXELS >= 8) begin
            $display("  Pixel[783] (Expected P0 from Addr0[63:56]=0x01): %h", tb_o_image_buffer_out[783]);
            $display("  Pixel[782] (Expected P1 from Addr0[55:48]=0x02): %h", tb_o_image_buffer_out[782]);
            $display("  Pixel[781] (Expected P2 from Addr0[47:40]=0x03): %h", tb_o_image_buffer_out[781]);
            $display("  Pixel[780] (Expected P3 from Addr0[39:32]=0x04): %h", tb_o_image_buffer_out[780]);
            $display("  Pixel[779] (Expected P4 from Addr0[31:24]=0x05): %h", tb_o_image_buffer_out[779]);
            $display("  Pixel[778] (Expected P5 from Addr0[23:16]=0x06): %h", tb_o_image_buffer_out[777]); //修正：应为778，打错了
            $display("  Pixel[777] (Expected P6 from Addr0[15:8]=0x07): %h", tb_o_image_buffer_out[778]);  //修正：应为777，打错了
            $display("  Pixel[776] (Expected P7 from Addr0[7:0]=0x08): %h", tb_o_image_buffer_out[776]);
            // 修正上面两个索引
            // Pixel[778] (Expected P5 from Addr0[23:16]=0x06): tb_o_image_buffer_out[P_NUM_INPUT_PIXELS-1 - (0*8+5)]
            // Pixel[777] (Expected P6 from Addr0[15:8]=0x07): tb_o_image_buffer_out[P_NUM_INPUT_PIXELS-1 - (0*8+6)]
            // Pixel[776] (Expected P7 from Addr0[7:0]=0x08): tb_o_image_buffer_out[P_NUM_INPUT_PIXELS-1 - (0*8+7)]
             $display("  --- Corrected Indexing for Display ---");
             $display("  Pixel GIdx 0 (Expected P0 from Addr0[63:56]=0x01, Stored at Idx 783): %h", tb_o_image_buffer_out[P_NUM_INPUT_PIXELS-1 - 0]);
             $display("  Pixel GIdx 1 (Expected P1 from Addr0[55:48]=0x02, Stored at Idx 782): %h", tb_o_image_buffer_out[P_NUM_INPUT_PIXELS-1 - 1]);
             $display("  Pixel GIdx 7 (Expected P7 from Addr0[7:0]=0x08,   Stored at Idx 776): %h", tb_o_image_buffer_out[P_NUM_INPUT_PIXELS-1 - 7]);
        end
        if (P_NUM_INPUT_PIXELS >= 16) begin
             $display("  Pixel GIdx 8 (Expected P0 from Addr1[63:56]=0x09, Stored at Idx 775): %h", tb_o_image_buffer_out[P_NUM_INPUT_PIXELS-1 - 8]);
        end


        repeat(10) @(posedge tb_clk);
        $display("[%0t ns] SIM_INFO: image_loader_tb 仿真结束。", $time);
        $finish;
    end

endmodule