`timescale 1ns/1ps

module aer_linear_layer_tb;

    localparam CLK_PERIOD = 10;
    localparam P_NUM_INPUT_EVENTS = 8;
    localparam P_EVENT_ADDR_WIDTH = 3;
    localparam P_WEIGHT_BIT_WIDTH = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_NEURON_VALUE_FRAC_BITS = 12;
    localparam P_BRAM_DATA_WIDTH = 64;
    localparam P_BRAM_ADDR_WIDTH = 1;
    localparam P_BRAM_READ_LATENCY = 2;
    localparam P_NUM_OUTPUT_NEURONS = 10;

    reg tb_clk;
    reg tb_rst_n;
    reg tb_start;
    reg tb_event_valid;
    reg [P_EVENT_ADDR_WIDTH-1:0] tb_event_addr;
    reg tb_event_frame_done;

    wire tb_event_ready;
    wire signed [P_NUM_OUTPUT_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] tb_all_currents_I;
    wire tb_all_currents_valid;

    integer check_idx;
    integer expected_value;
    integer error_count;

    aer_linear_layer #(
        .P_NUM_INPUT_EVENTS          (P_NUM_INPUT_EVENTS),
        .P_EVENT_ADDR_WIDTH          (P_EVENT_ADDR_WIDTH),
        .P_WEIGHT_BIT_WIDTH          (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS   (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS    (P_NEURON_VALUE_FRAC_BITS),
        .P_BRAM_DATA_WIDTH           (P_BRAM_DATA_WIDTH),
        .P_BRAM_ADDR_WIDTH           (P_BRAM_ADDR_WIDTH),
        .P_BRAM_READ_LATENCY         (P_BRAM_READ_LATENCY),
        .P_NUM_OUTPUT_NEURONS        (P_NUM_OUTPUT_NEURONS)
    ) dut (
        .clk                    (tb_clk),
        .rst_n                  (tb_rst_n),
        .i_start                (tb_start),
        .i_event_valid          (tb_event_valid),
        .i_event_addr           (tb_event_addr),
        .i_event_frame_done     (tb_event_frame_done),
        .o_event_ready          (tb_event_ready),
        .o_all_currents_I       (tb_all_currents_I),
        .o_all_currents_valid   (tb_all_currents_valid)
    );

    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    /*
     * 向 AER 全连接层发送一个事件。
     * valid 保持到 ready 有效的时钟沿，模拟标准 valid/ready 握手。
     */
    task send_event;
        input [P_EVENT_ADDR_WIDTH-1:0] event_addr;
        begin
            @(negedge tb_clk);
            tb_event_addr = event_addr;
            tb_event_valid = 1'b1;
            while (!tb_event_ready) begin
                @(posedge tb_clk);
            end
            @(posedge tb_clk);
            @(negedge tb_clk);
            tb_event_valid = 1'b0;
            tb_event_addr = {P_EVENT_ADDR_WIDTH{1'b0}};
        end
    endtask

    /*
     * 发送当前时间步事件结束标志。
     */
    task send_frame_done;
        begin
            @(negedge tb_clk);
            while (!tb_event_ready) begin
                @(posedge tb_clk);
            end
            tb_event_frame_done = 1'b1;
            @(posedge tb_clk);
            @(negedge tb_clk);
            tb_event_frame_done = 1'b0;
        end
    endtask

    /*
     * 连续 5 拍发送事件 0、1、2、3、4。
     * 流水版 aer_linear_layer 应该能在 ready 持续为高时每拍接收一个事件。
     */
    task send_continuous_events_0_to_4;
        integer event_idx;
        begin
            @(negedge tb_clk);
            tb_event_valid = 1'b1;
            for (event_idx = 0; event_idx < 5; event_idx = event_idx + 1) begin
                tb_event_addr = event_idx[P_EVENT_ADDR_WIDTH-1:0];
                if (!tb_event_ready) begin
                    $display("[%0t ns] SIM_ERROR: 连续事件测试中 ready 意外拉低，event_idx=%0d。",
                             $time, event_idx);
                    error_count = error_count + 1;
                end
                @(posedge tb_clk);
                @(negedge tb_clk);
            end
            tb_event_valid = 1'b0;
            tb_event_addr = {P_EVENT_ADDR_WIDTH{1'b0}};
        end
    endtask

    initial begin
        error_count = 0;
        tb_rst_n = 1'b0;
        tb_start = 1'b0;
        tb_event_valid = 1'b0;
        tb_event_addr = {P_EVENT_ADDR_WIDTH{1'b0}};
        tb_event_frame_done = 1'b0;

        $display("[%0t ns] SIM_INFO: aer_linear_layer_tb start.", $time);

        repeat (5) @(posedge tb_clk);
        tb_rst_n = 1'b1;
        repeat (2) @(posedge tb_clk);

        tb_start = 1'b1;
        @(posedge tb_clk);
        tb_start = 1'b0;

        /*
         * 发送三个事件地址：0、2、5。
         * event_addr=0 -> bram_addr=0, offset=0
         * event_addr=2 -> bram_addr=0, offset=2
         * event_addr=5 -> bram_addr=1, offset=1
         */
        send_event(3'd0);
        send_event(3'd2);
        send_event(3'd5);
        send_frame_done();

        wait (tb_all_currents_valid);
        @(posedge tb_clk);

        /*
         * stub 权重规则：
         * 第 n 个输出神经元，地址 0 的四个逻辑权重为 100*n+0/1/2/3；
         * 地址 1 的四个逻辑权重为 100*n+10/11/12/13。
         * 因此事件 0、2、5 的期望累加和为：
         * (100*n+0) + (100*n+2) + (100*n+11) = 300*n + 13。
         */
        for (check_idx = 0; check_idx < P_NUM_OUTPUT_NEURONS; check_idx = check_idx + 1) begin
            expected_value = (300 * check_idx) + 13;
            if (tb_all_currents_I[check_idx] !== expected_value) begin
                $display("[%0t ns] SIM_ERROR: neuron%0d 电流错误，期望 %0d，实际 %0d。",
                         $time, check_idx, expected_value, tb_all_currents_I[check_idx]);
                error_count = error_count + 1;
            end else begin
                $display("[%0t ns] SIM_INFO: neuron%0d current=%0d checked.",
                         $time, check_idx, tb_all_currents_I[check_idx]);
                end
        end

        repeat (3) @(posedge tb_clk);

        /*
         * 第二轮测试：连续 5 拍发送事件 0、1、2、3、4。
         * 期望累加和：
         * (100*n+0) + (100*n+1) + (100*n+2) + (100*n+3) + (100*n+10)
         * = 500*n + 16。
         */
        tb_start = 1'b1;
        @(posedge tb_clk);
        tb_start = 1'b0;

        send_continuous_events_0_to_4();
        send_frame_done();

        wait (tb_all_currents_valid);
        @(posedge tb_clk);

        for (check_idx = 0; check_idx < P_NUM_OUTPUT_NEURONS; check_idx = check_idx + 1) begin
            expected_value = (500 * check_idx) + 16;
            if (tb_all_currents_I[check_idx] !== expected_value) begin
                $display("[%0t ns] SIM_ERROR: 连续事件测试 neuron%0d 电流错误，期望 %0d，实际 %0d。",
                         $time, check_idx, expected_value, tb_all_currents_I[check_idx]);
                error_count = error_count + 1;
            end else begin
                $display("[%0t ns] SIM_INFO: continuous test neuron%0d current=%0d checked.",
                         $time, check_idx, tb_all_currents_I[check_idx]);
            end
        end

        if (error_count == 0) begin
            $display("[%0t ns] SIM_PASS: aer_linear_layer_tb passed.", $time);
        end else begin
            $display("[%0t ns] SIM_FAIL: aer_linear_layer_tb failed, error_count=%0d.", $time, error_count);
        end

        $finish;
    end

endmodule

/*
 * 下面是仿真用的简化权重 BRAM stub。
 * 它们只用于这个 tb，让命令行仿真不依赖 Vivado 生成的 weights_0~weights_9 IP。
 */
module weight_bram_stub #(
    parameter P_NEURON_ID = 0,
    parameter P_ADDR_WIDTH = 1
) (
    input wire clka,
    input wire ena,
    input wire [P_ADDR_WIDTH-1:0] addra,
    output reg signed [63:0] douta
);

    reg signed [63:0] pipe0_reg;

    function signed [63:0] make_weight_word;
        input integer neuron_id;
        input integer addr;
        integer base;
        begin
            base = (100 * neuron_id) + (10 * addr);
            make_weight_word = {
                16'(base + 0),
                16'(base + 1),
                16'(base + 2),
                16'(base + 3)
            };
        end
    endfunction

    always @(posedge clka) begin
        if (ena) begin
            pipe0_reg <= make_weight_word(P_NEURON_ID, addra);
        end
        douta <= pipe0_reg;
    end

endmodule

module weights_0(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    weight_bram_stub #(.P_NEURON_ID(0), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_1(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    weight_bram_stub #(.P_NEURON_ID(1), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_2(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    weight_bram_stub #(.P_NEURON_ID(2), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_3(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    weight_bram_stub #(.P_NEURON_ID(3), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_4(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    weight_bram_stub #(.P_NEURON_ID(4), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_5(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    weight_bram_stub #(.P_NEURON_ID(5), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_6(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    weight_bram_stub #(.P_NEURON_ID(6), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_7(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    weight_bram_stub #(.P_NEURON_ID(7), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_8(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    weight_bram_stub #(.P_NEURON_ID(8), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_9(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    weight_bram_stub #(.P_NEURON_ID(9), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule
