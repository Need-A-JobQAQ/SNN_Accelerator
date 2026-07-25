// weights_0_tb.v
// (中文注释版 - 修改后用于观察多周期延迟)
// 用于测试 BRAM ROM IP 核 (weights_0) 能否正确读取权重的简单测试平台。

`timescale 1ns/1ps
// `include "snn_params.vh" // 如果需要参数，可以包含

module weights_0_tb;

    // 测试平台参数
    localparam CLK_PERIOD = 10; // 时钟周期 (ns) - 100MHz

    // 测试平台内部信号 - 连接到 weights_0 IP 核的输入
    reg                             tb_clk;
    reg                             tb_ena;         // BRAM 使能信号
    reg  [7:0]                      tb_addra;       // BRAM 地址输入 (根据模板是8位)

    // 测试平台内部信号 - 连接到 weights_0 IP 核的输出
    wire [63:0]                     tb_douta;       // BRAM 数据输出 (根据模板是64位)


    // 1. 例化待测试的 BRAM ROM IP 核 (weights_0)
    weights_0 u_weights_0_rom_instance (
        .clka(tb_clk),
        .ena(tb_ena),
        .addra(tb_addra),
        .douta(tb_douta)
    );

    // 2. 时钟生成逻辑
    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    // 3. 测试激励序列和检查逻辑 (修改版)
    initial begin
        $display("SIM_INFO: weights_0_tb (观察延迟版) 开始仿真 @ %0t ns", $time);

        // 初始化输入信号
        tb_ena   = 1'b0; // 初始时不使能
        tb_addra = 8'hXX; // 初始地址设为不定，避免误解
        
        // 施加复位 (如果你的BRAM IP核有复位端口且需要复位，这里需要添加复位逻辑)
        // 我们的例化模板中BRAM IP核没有明确的复位端口，其状态在时钟和使能下改变
        // BRAM内容由COE文件初始化，上电即有。

        repeat(2) @(posedge tb_clk); // 等待几个周期

        // --- 测试读取地址 8'h00 ---
        $display("---------------------------------------------------------");
        $display("SIM_INFO: [%0t ns] 设置地址为 8'h00, 使能 tb_ena=1", $time);
        tb_addra = 8'h00;
        tb_ena   = 1'b1; // 使能读取

        @(posedge tb_clk); // 第1个时钟上升沿 (在地址和使能设置之后)
        $display("SIM_INFO: [%0t ns] 地址 8'h%h, 第1个时钟沿后, douta = 64'h%h", $time, tb_addra, tb_douta);
        // 期望值 (地址0): 64'hFF8D00920032006A (你需要根据你的COE文件确认)
        // if (tb_douta !== 64'hFF8D00920032006A && $time > CLK_PERIOD*3) $error("错误：地址0的数据在第1周期后不匹配！");


        @(posedge tb_clk); // 第2个时钟上升沿
        $display("SIM_INFO: [%0t ns] 地址 8'h%h, 第2个时钟沿后, douta = 64'h%h", $time, tb_addra, tb_douta);
        // if (tb_douta !== 64'hFF8D00920032006A && $time > CLK_PERIOD*4) $error("错误：地址0的数据在第2周期后不匹配！");

        @(posedge tb_clk); // 第3个时钟上升沿 (额外观察周期，看数据是否稳定)
        $display("SIM_INFO: [%0t ns] 地址 8'h%h, 第3个时钟沿后, douta = 64'h%h (应与上一周期相同)", $time, tb_addra, tb_douta);

        // --- 测试读取地址 8'h01 ---
        $display("---------------------------------------------------------");
        $display("SIM_INFO: [%0t ns] 设置地址为 8'h01, tb_ena保持为1", $time);
        tb_addra = 8'h01; // 更改地址，tb_ena 仍然是 1'b1 (连续读取)

        @(posedge tb_clk); // 第1个时钟上升沿 (在新地址设置之后)
        $display("SIM_INFO: [%0t ns] 地址 8'h%h, 第1个时钟沿后, douta = 64'h%h", $time, tb_addra, tb_douta);
        // 期望值 (地址1): 64'hFF7D0071FF79002B (你需要根据你的COE文件确认)
        // 在这个周期，如果BRAM有1周期延迟，这里应该输出地址1的数据。
        // 如果有2周期延迟，这里应该仍然是地址0的数据。

        @(posedge tb_clk); // 第2个时钟上升沿
        $display("SIM_INFO: [%0t ns] 地址 8'h%h, 第2个时钟沿后, douta = 64'h%h", $time, tb_addra, tb_douta);
        // 在这个周期，如果BRAM有2周期延迟，这里应该输出地址1的数据。
        // 如果是1周期延迟，这里的数据应该和上一个周期（地址1，第1个时钟沿后）相同。
        
        @(posedge tb_clk); // 第3个时钟上升沿 (额外观察周期)
        $display("SIM_INFO: [%0t ns] 地址 8'h%h, 第3个时钟沿后, douta = 64'h%h (应与上一周期相同)", $time, tb_addra, tb_douta);

        // --- 测试读取地址 8'h02 ---
        $display("---------------------------------------------------------");
        $display("SIM_INFO: [%0t ns] 设置地址为 8'h02, tb_ena保持为1", $time);
        tb_addra = 8'h02;

        @(posedge tb_clk); // 第1个时钟上升沿 (在新地址设置之后)
        $display("SIM_INFO: [%0t ns] 地址 8'h%h, 第1个时钟沿后, douta = 64'h%h", $time, tb_addra, tb_douta);
        // 期望值 (地址2): 64'h0027FFD1FF710012

        @(posedge tb_clk); // 第2个时钟上升沿
        $display("SIM_INFO: [%0t ns] 地址 8'h%h, 第2个时钟沿后, douta = 64'h%h", $time, tb_addra, tb_douta);
        
        @(posedge tb_clk); // 第3个时钟上升沿 (额外观察周期)
        $display("SIM_INFO: [%0t ns] 地址 8'h%h, 第3个时钟沿后, douta = 64'h%h (应与上一周期相同)", $time, tb_addra, tb_douta);


        // 读取结束后，可以将使能拉低
        $display("---------------------------------------------------------");
        tb_ena = 1'b0;
        @(posedge tb_clk);
        $display("SIM_INFO: [%0t ns] tb_ena 设置为0", $time);

        #(CLK_PERIOD * 5); // 额外等待一段时间

        $display("SIM_INFO: weights_0_tb (观察延迟版) 仿真结束 @ %0t ns", $time);
        $finish; // 结束仿真
    end

endmodule

            //****************************************************************************************************************
            //                  1                                0
            //            上一次发送的地址                      新的地址                       准备发送的地址 bram_addr_to_issue_reg
            //             上一次的数据          <--           新的数据             <--       准备进入的数据 bram_douta_raw
            //       bram_ena_sig(可能为0/1)           bram_ena_sig(可能为0/1)               bram_ena_sig(可能为0/1)        
            //                |                                  |
            //                 ----------------------------------
            //                                  |
            //   从上到下依次使用流水线寄存器 addr_pipeline_reg，data_pipeline_reg，valid_pipeline_reg 存储
            //
            //................................................................................................................
            //
            //  可以发现只要 valid_pipeline_reg[1] 处检测到的 bram_ena_sig 为 0 时，意味着 BRAM 的使能已经关闭了，之后则无需读取了
            //
            //****************************************************************************************************************
