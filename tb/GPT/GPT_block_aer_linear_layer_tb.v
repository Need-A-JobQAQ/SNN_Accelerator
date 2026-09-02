`timescale 1ns/1ps

module GPT_block_aer_linear_layer_tb;

    localparam CLK_PERIOD = 10;
    localparam P_NUM_INPUT_EVENTS = 1568;
    localparam P_BLOCK_SIZE = 32;
    localparam P_BLOCK_ID_WIDTH = 6;
    localparam P_WEIGHT_BIT_WIDTH = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_NEURON_VALUE_FRAC_BITS = 12;
    localparam P_BRAM_DATA_WIDTH = 64;
    localparam P_BRAM_ADDR_WIDTH = 9;
    localparam P_BRAM_READ_LATENCY = 2;
    localparam P_NUM_OUTPUT_NEURONS = 10;

    reg clk;
    reg rst_n;
    reg i_start;
    reg i_block_valid;
    reg [P_BLOCK_ID_WIDTH-1:0] i_block_id;
    reg [P_BLOCK_SIZE-1:0] i_block_mask;
    reg i_block_frame_done;
    wire o_block_ready;
    wire signed [P_NUM_OUTPUT_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] o_all_currents_I;
    wire o_all_currents_valid;

    integer error_count;
    integer neuron_idx;
    integer expected_value;

    block_aer_linear_layer #(
        .P_NUM_INPUT_EVENTS        (P_NUM_INPUT_EVENTS),
        .P_BLOCK_SIZE              (P_BLOCK_SIZE),
        .P_BLOCK_ID_WIDTH          (P_BLOCK_ID_WIDTH),
        .P_WEIGHT_BIT_WIDTH        (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS  (P_NEURON_VALUE_FRAC_BITS),
        .P_BRAM_DATA_WIDTH         (P_BRAM_DATA_WIDTH),
        .P_BRAM_ADDR_WIDTH         (P_BRAM_ADDR_WIDTH),
        .P_BRAM_READ_LATENCY       (P_BRAM_READ_LATENCY),
        .P_NUM_OUTPUT_NEURONS      (P_NUM_OUTPUT_NEURONS)
    ) u_dut (
        .clk                       (clk),
        .rst_n                     (rst_n),
        .i_start                   (i_start),
        .i_block_valid             (i_block_valid),
        .i_block_id                (i_block_id),
        .i_block_mask              (i_block_mask),
        .i_block_frame_done        (i_block_frame_done),
        .o_block_ready             (o_block_ready),
        .o_all_currents_I          (o_all_currents_I),
        .o_all_currents_valid      (o_all_currents_valid)
    );

    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    task pulse_start;
        begin
            i_start <= 1'b1;
            @(posedge clk);
            i_start <= 1'b0;
        end
    endtask

    task send_block;
        input [P_BLOCK_ID_WIDTH-1:0] block_id;
        input [P_BLOCK_SIZE-1:0] block_mask;
        begin
            @(negedge clk);
            i_block_id <= block_id;
            i_block_mask <= block_mask;
            i_block_valid <= 1'b1;
            while (!o_block_ready) begin
                @(posedge clk);
                @(negedge clk);
            end
            @(posedge clk);
            @(negedge clk);
            i_block_valid <= 1'b0;
            i_block_id <= {P_BLOCK_ID_WIDTH{1'b0}};
            i_block_mask <= {P_BLOCK_SIZE{1'b0}};
        end
    endtask

    task send_frame_done;
        begin
            @(negedge clk);
            i_block_frame_done <= 1'b1;
            while (!o_block_ready) begin
                @(posedge clk);
                @(negedge clk);
            end
            @(posedge clk);
            @(negedge clk);
            i_block_frame_done <= 1'b0;
        end
    endtask

    initial begin
        $display("[%0t ns] SIM_INFO: GPT_block_aer_linear_layer_tb start.", $time);

        error_count = 0;
        i_start = 1'b0;
        i_block_valid = 1'b0;
        i_block_id = {P_BLOCK_ID_WIDTH{1'b0}};
        i_block_mask = {P_BLOCK_SIZE{1'b0}};
        i_block_frame_done = 1'b0;

        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        pulse_start();

        /*
         * stub 权重规则：weight = neuron_id * 1000 + event_addr。
         * block0 mask bit 0/1/3：贡献 0 + 1 + 3。
         * block2 mask bit 0/4/7：贡献 64 + 68 + 71。
         * 每个神经元共 6 个 spike，因此期望值为 neuron_id*6000 + 207。
         */
        send_block(6'd0, 32'h0000_000b);
        send_block(6'd2, 32'h0000_0091);
        send_frame_done();

        while (!o_all_currents_valid) begin
            @(posedge clk);
        end
        #1;

        for (neuron_idx = 0; neuron_idx < P_NUM_OUTPUT_NEURONS; neuron_idx = neuron_idx + 1) begin
            expected_value = (neuron_idx * 6000) + 207;
            if (o_all_currents_I[neuron_idx] !== expected_value[P_NEURON_VALUE_TOTAL_BITS-1:0]) begin
                $display("[%0t ns] SIM_ERROR: neuron%0d current mismatch, got=%0d expected=%0d.",
                         $time, neuron_idx, o_all_currents_I[neuron_idx], expected_value);
                error_count = error_count + 1;
            end
        end

        pulse_start();
        send_frame_done();
        while (!o_all_currents_valid) begin
            @(posedge clk);
        end
        #1;
        for (neuron_idx = 0; neuron_idx < P_NUM_OUTPUT_NEURONS; neuron_idx = neuron_idx + 1) begin
            if (o_all_currents_I[neuron_idx] !== {P_NEURON_VALUE_TOTAL_BITS{1'b0}}) begin
                $display("[%0t ns] SIM_ERROR: empty frame current should be zero, neuron%0d got=%0d.",
                         $time, neuron_idx, o_all_currents_I[neuron_idx]);
                error_count = error_count + 1;
            end
        end

        if (error_count == 0) begin
            $display("[%0t ns] SIM_PASS: block_aer_linear_layer group4 accumulation passed.", $time);
        end else begin
            $display("[%0t ns] SIM_FAIL: block_aer_linear_layer failed, error_count=%0d.", $time, error_count);
        end
        $finish;
    end

endmodule


module GPT_block_aer_weight_bram_stub #(
    parameter P_NEURON_ID = 0
) (
    input wire clka,
    input wire ena,
    input wire [8:0] addra,
    output reg signed [63:0] douta
);

    reg signed [63:0] pipe0_reg;

    function signed [15:0] make_weight;
        input [8:0] word_addr;
        input [1:0] offset;
        begin
            make_weight = (P_NEURON_ID * 1000) + (word_addr * 4) + offset;
        end
    endfunction

    always @(posedge clka) begin
        if (ena) begin
            pipe0_reg <= {make_weight(addra, 2'd0),
                          make_weight(addra, 2'd1),
                          make_weight(addra, 2'd2),
                          make_weight(addra, 2'd3)};
            douta <= pipe0_reg;
        end
    end

endmodule

module weights_0(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_block_aer_weight_bram_stub #(.P_NEURON_ID(0)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_1(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_block_aer_weight_bram_stub #(.P_NEURON_ID(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_2(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_block_aer_weight_bram_stub #(.P_NEURON_ID(2)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_3(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_block_aer_weight_bram_stub #(.P_NEURON_ID(3)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_4(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_block_aer_weight_bram_stub #(.P_NEURON_ID(4)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_5(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_block_aer_weight_bram_stub #(.P_NEURON_ID(5)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_6(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_block_aer_weight_bram_stub #(.P_NEURON_ID(6)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_7(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_block_aer_weight_bram_stub #(.P_NEURON_ID(7)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_8(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_block_aer_weight_bram_stub #(.P_NEURON_ID(8)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_9(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_block_aer_weight_bram_stub #(.P_NEURON_ID(9)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

