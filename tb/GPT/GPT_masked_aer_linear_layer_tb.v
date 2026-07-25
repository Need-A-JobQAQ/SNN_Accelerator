`timescale 1ns/1ps

module GPT_masked_aer_linear_layer_tb;

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

    integer error_count;

    masked_aer_linear_layer #(
        .P_NUM_INPUT_EVENTS          (P_NUM_INPUT_EVENTS),
        .P_EVENT_ADDR_WIDTH          (P_EVENT_ADDR_WIDTH),
        .P_WEIGHT_BIT_WIDTH          (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS   (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS    (P_NEURON_VALUE_FRAC_BITS),
        .P_BRAM_DATA_WIDTH           (P_BRAM_DATA_WIDTH),
        .P_BRAM_ADDR_WIDTH           (P_BRAM_ADDR_WIDTH),
        .P_BRAM_READ_LATENCY         (P_BRAM_READ_LATENCY),
        .P_NUM_OUTPUT_NEURONS        (P_NUM_OUTPUT_NEURONS),
        .P_MASK_WIDTH                (16)
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
     * 发送一个 AER 事件，valid 保持到 ready 握手完成。
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
     * 通知当前事件帧结束。
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
     * 检查某个输出类别的累加电流。
     */
    task check_current;
        input integer class_idx;
        input signed [P_NEURON_VALUE_TOTAL_BITS-1:0] expected_value;
        begin
            if (tb_all_currents_I[class_idx] !== expected_value) begin
                error_count = error_count + 1;
                $display("[%0t ns] SIM_ERROR: class%0d current=%0d, expected=%0d.",
                         $time, class_idx, tb_all_currents_I[class_idx], expected_value);
            end else begin
                $display("[%0t ns] SIM_INFO: class%0d current=%0d checked.",
                         $time, class_idx, tb_all_currents_I[class_idx]);
            end
        end
    endtask

    initial begin
        error_count = 0;
        tb_rst_n = 1'b0;
        tb_start = 1'b0;
        tb_event_valid = 1'b0;
        tb_event_addr = {P_EVENT_ADDR_WIDTH{1'b0}};
        tb_event_frame_done = 1'b0;

        $display("[%0t ns] SIM_INFO: GPT_masked_aer_linear_layer_tb start.", $time);

        repeat (5) @(posedge tb_clk);
        tb_rst_n = 1'b1;
        repeat (2) @(posedge tb_clk);

        tb_start = 1'b1;
        @(posedge tb_clk);
        tb_start = 1'b0;

        /*
         * mask stub 规则：
         * event0 -> mask=0000000011，保留 class0/class1
         * event2 -> mask=0000000101，保留 class0/class2
         * event5 -> mask=0000001000，保留 class3
         *
         * weight stub 规则：
         * class n, bram_addr a, offset o 的权重 = 100*n + 10*a + o
         *
         * 因此：
         * class0 = event0(0) + event2(2) = 2
         * class1 = event0(100) = 100
         * class2 = event2(202) = 202
         * class3 = event5(311) = 311
         * 其他类别由于 mask=0，应保持 0。
         */
        send_event(3'd0);
        send_event(3'd2);
        send_event(3'd5);
        send_frame_done();

        wait (tb_all_currents_valid);
        @(posedge tb_clk);

        check_current(0, 26'sd2);
        check_current(1, 26'sd100);
        check_current(2, 26'sd202);
        check_current(3, 26'sd311);
        check_current(4, 26'sd0);
        check_current(5, 26'sd0);
        check_current(6, 26'sd0);
        check_current(7, 26'sd0);
        check_current(8, 26'sd0);
        check_current(9, 26'sd0);

        if (error_count == 0) begin
            $display("[%0t ns] SIM_PASS: masked_aer_linear_layer mask gating is correct.", $time);
        end else begin
            $display("[%0t ns] SIM_FAIL: masked_aer_linear_layer failed, error_count=%0d.", $time, error_count);
        end

        $finish;
    end

endmodule

/*
 * 下面是仿真用的简化权重 BRAM stub。
 * 读延迟模拟为 2 拍，以匹配 masked_aer_linear_layer 的 P_BRAM_READ_LATENCY=2。
 */
module masked_weight_bram_stub #(
    parameter P_NEURON_ID = 0,
    parameter P_ADDR_WIDTH = 1
) (
    input wire clka,
    input wire ena,
    input wire [P_ADDR_WIDTH-1:0] addra,
    output reg signed [63:0] douta
);

    reg signed [63:0] pipe0_reg;

    function signed [15:0] make_weight;
        input integer neuron_id;
        input integer addr;
        input integer offset;
        begin
            make_weight = (100 * neuron_id) + (10 * addr) + offset;
        end
    endfunction

    always @(posedge clka) begin
        if (ena) begin
            pipe0_reg <= {
                make_weight(P_NEURON_ID, addra, 0),
                make_weight(P_NEURON_ID, addra, 1),
                make_weight(P_NEURON_ID, addra, 2),
                make_weight(P_NEURON_ID, addra, 3)
            };
        end
        douta <= pipe0_reg;
    end

endmodule

/*
 * mask ROM stub，同样模拟 2 拍读延迟。
 */
module fc_mask_0p1 (
    input wire clka,
    input wire ena,
    input wire [2:0] addra,
    output reg [15:0] douta
);

    reg [15:0] pipe0_reg;

    function [15:0] make_mask;
        input [2:0] addr;
        begin
            case (addr)
                3'd0: make_mask = 16'h0003;
                3'd2: make_mask = 16'h0005;
                3'd5: make_mask = 16'h0008;
                default: make_mask = 16'h0000;
            endcase
        end
    endfunction

    always @(posedge clka) begin
        if (ena) begin
            pipe0_reg <= make_mask(addra);
        end
        douta <= pipe0_reg;
    end

endmodule

module weights_0(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    masked_weight_bram_stub #(.P_NEURON_ID(0), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_1(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    masked_weight_bram_stub #(.P_NEURON_ID(1), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_2(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    masked_weight_bram_stub #(.P_NEURON_ID(2), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_3(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    masked_weight_bram_stub #(.P_NEURON_ID(3), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_4(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    masked_weight_bram_stub #(.P_NEURON_ID(4), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_5(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    masked_weight_bram_stub #(.P_NEURON_ID(5), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_6(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    masked_weight_bram_stub #(.P_NEURON_ID(6), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_7(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    masked_weight_bram_stub #(.P_NEURON_ID(7), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_8(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    masked_weight_bram_stub #(.P_NEURON_ID(8), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_9(input wire clka, input wire ena, input wire [0:0] addra, output wire signed [63:0] douta);
    masked_weight_bram_stub #(.P_NEURON_ID(9), .P_ADDR_WIDTH(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule
