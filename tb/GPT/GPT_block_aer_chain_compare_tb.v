`timescale 1ns/1ps

module GPT_block_aer_chain_compare_tb;

    localparam CLK_PERIOD = 10;
    localparam P_NUM_INPUT_EVENTS = 1568;
    localparam P_EVENT_ADDR_WIDTH = 11;
    localparam P_BLOCK_SIZE = 32;
    localparam P_BLOCK_ID_WIDTH = 6;
    localparam P_WEIGHT_BIT_WIDTH = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_NEURON_VALUE_FRAC_BITS = 12;
    localparam P_BRAM_DATA_WIDTH = 64;
    localparam P_BRAM_ADDR_WIDTH = 9;
    localparam P_BRAM_READ_LATENCY = 2;
    localparam P_NUM_OUTPUT_NEURONS = 10;
    localparam P_BLOCK_FIFO_DEPTH = 8;

    reg clk;
    reg rst_n;
    reg dense_start;
    reg block_start;

    reg event_valid;
    reg [P_EVENT_ADDR_WIDTH-1:0] event_addr;
    reg event_frame_done;
    wire dense_event_ready;
    wire packer_event_ready;
    wire input_event_ready;

    wire dense_currents_valid;
    wire signed [P_NUM_OUTPUT_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] dense_currents;

    wire packer_block_valid;
    wire [P_BLOCK_ID_WIDTH-1:0] packer_block_id;
    wire [P_BLOCK_SIZE-1:0] packer_block_mask;
    wire packer_block_ready;
    wire packer_block_frame_done;

    wire fifo_block_valid;
    wire [P_BLOCK_ID_WIDTH-1:0] fifo_block_id;
    wire [P_BLOCK_SIZE-1:0] fifo_block_mask;
    wire fifo_empty;
    wire fifo_full;
    wire [$clog2(P_BLOCK_FIFO_DEPTH + 1)-1:0] fifo_count;
    wire fifo_overflow;
    wire block_linear_ready;
    wire fifo_read_ready;

    reg block_done_pending;
    wire block_frame_done_to_linear;
    wire block_currents_valid;
    wire signed [P_NUM_OUTPUT_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] block_currents;

    reg dense_done_seen;
    reg block_done_seen;
    reg signed [P_NUM_OUTPUT_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] dense_currents_latched;
    reg signed [P_NUM_OUTPUT_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] block_currents_latched;

    integer error_count;
    integer neuron_idx;
    integer timeout_count;

    assign input_event_ready = dense_event_ready && packer_event_ready;
    assign packer_block_ready = !fifo_full;
    assign fifo_read_ready = block_linear_ready && !block_frame_done_to_linear;
    assign block_frame_done_to_linear = block_done_pending && fifo_empty;

    aer_linear_layer #(
        .P_NUM_INPUT_EVENTS        (P_NUM_INPUT_EVENTS),
        .P_EVENT_ADDR_WIDTH        (P_EVENT_ADDR_WIDTH),
        .P_WEIGHT_BIT_WIDTH        (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS  (P_NEURON_VALUE_FRAC_BITS),
        .P_BRAM_DATA_WIDTH         (P_BRAM_DATA_WIDTH),
        .P_BRAM_ADDR_WIDTH         (P_BRAM_ADDR_WIDTH),
        .P_BRAM_READ_LATENCY       (P_BRAM_READ_LATENCY),
        .P_NUM_OUTPUT_NEURONS      (P_NUM_OUTPUT_NEURONS)
    ) u_dense_linear (
        .clk                       (clk),
        .rst_n                     (rst_n),
        .i_start                   (dense_start),
        .i_event_valid             (event_valid),
        .i_event_addr              (event_addr),
        .i_event_frame_done        (event_frame_done),
        .o_event_ready             (dense_event_ready),
        .o_all_currents_I          (dense_currents),
        .o_all_currents_valid      (dense_currents_valid)
    );

    aer_to_block_mask_packer #(
        .P_EVENT_ADDR_WIDTH        (P_EVENT_ADDR_WIDTH),
        .P_BLOCK_SIZE              (P_BLOCK_SIZE),
        .P_BLOCK_ID_WIDTH          (P_BLOCK_ID_WIDTH)
    ) u_packer (
        .clk                       (clk),
        .rst_n                     (rst_n),
        .i_clear                   (1'b0),
        .i_event_valid             (event_valid),
        .i_event_addr              (event_addr),
        .i_event_frame_done        (event_frame_done),
        .o_event_ready             (packer_event_ready),
        .o_block_valid             (packer_block_valid),
        .o_block_id                (packer_block_id),
        .o_block_mask              (packer_block_mask),
        .i_block_ready             (packer_block_ready),
        .o_block_frame_done        (packer_block_frame_done)
    );

    block_event_fifo #(
        .P_BLOCK_ID_WIDTH          (P_BLOCK_ID_WIDTH),
        .P_BLOCK_SIZE              (P_BLOCK_SIZE),
        .P_FIFO_DEPTH              (P_BLOCK_FIFO_DEPTH)
    ) u_block_fifo (
        .clk                       (clk),
        .rst_n                     (rst_n),
        .i_clear                   (1'b0),
        .i_block_valid             (packer_block_valid),
        .i_block_id                (packer_block_id),
        .i_block_mask              (packer_block_mask),
        .i_block_ready             (fifo_read_ready),
        .o_block_valid             (fifo_block_valid),
        .o_block_id                (fifo_block_id),
        .o_block_mask              (fifo_block_mask),
        .o_empty                   (fifo_empty),
        .o_full                    (fifo_full),
        .o_count                   (fifo_count),
        .o_overflow                (fifo_overflow)
    );

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
    ) u_block_linear (
        .clk                       (clk),
        .rst_n                     (rst_n),
        .i_start                   (block_start),
        .i_block_valid             (fifo_block_valid),
        .i_block_id                (fifo_block_id),
        .i_block_mask              (fifo_block_mask),
        .i_block_frame_done        (block_frame_done_to_linear),
        .o_block_ready             (block_linear_ready),
        .o_all_currents_I          (block_currents),
        .o_all_currents_valid      (block_currents_valid)
    );

    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            block_done_pending <= 1'b0;
        end else if (block_start) begin
            block_done_pending <= 1'b0;
        end else if (packer_block_frame_done) begin
            block_done_pending <= 1'b1;
        end else if (block_frame_done_to_linear && block_linear_ready) begin
            block_done_pending <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dense_done_seen <= 1'b0;
        end else if (dense_start) begin
            dense_done_seen <= 1'b0;
        end else if (dense_currents_valid) begin
            dense_done_seen <= 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            block_done_seen <= 1'b0;
        end else if (block_start) begin
            block_done_seen <= 1'b0;
        end else if (block_currents_valid) begin
            block_done_seen <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (dense_currents_valid) begin
            dense_currents_latched <= dense_currents;
        end
        if (block_currents_valid) begin
            block_currents_latched <= block_currents;
        end
    end

    task pulse_start;
        begin
            @(negedge clk);
            dense_start <= 1'b1;
            block_start <= 1'b1;
            @(posedge clk);
            @(negedge clk);
            dense_start <= 1'b0;
            block_start <= 1'b0;
        end
    endtask

    task send_event;
        input [P_EVENT_ADDR_WIDTH-1:0] addr;
        begin
            @(negedge clk);
            event_addr <= addr;
            event_valid <= 1'b1;
            while (!input_event_ready) begin
                @(posedge clk);
                @(negedge clk);
            end
            @(posedge clk);
            @(negedge clk);
            event_valid <= 1'b0;
            event_addr <= {P_EVENT_ADDR_WIDTH{1'b0}};
        end
    endtask

    task send_frame_done;
        begin
            @(negedge clk);
            event_frame_done <= 1'b1;
            while (!input_event_ready) begin
                @(posedge clk);
                @(negedge clk);
            end
            @(posedge clk);
            @(negedge clk);
            event_frame_done <= 1'b0;
        end
    endtask

    initial begin
        $display("[%0t ns] SIM_INFO: GPT_block_aer_chain_compare_tb start.", $time);

        error_count = 0;
        dense_start = 1'b0;
        block_start = 1'b0;
        event_valid = 1'b0;
        event_addr = {P_EVENT_ADDR_WIDTH{1'b0}};
        event_frame_done = 1'b0;

        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        pulse_start();

        // 地址序列覆盖同 group4、同 block 不同 group4、跨 block 三类情况。
        send_event(11'd0);
        send_event(11'd1);
        send_event(11'd3);
        send_event(11'd64);
        send_event(11'd68);
        send_event(11'd71);
        send_event(11'd96);
        send_event(11'd99);
        send_frame_done();

        timeout_count = 0;
        while (!(dense_done_seen && block_done_seen) && timeout_count < 200) begin
            @(posedge clk);
            timeout_count = timeout_count + 1;
        end

        if (!(dense_done_seen && block_done_seen)) begin
            $display("[%0t ns] SIM_ERROR: timeout waiting outputs, dense_done=%b block_done=%b.",
                     $time, dense_done_seen, block_done_seen);
            error_count = error_count + 1;
        end

        if (fifo_overflow) begin
            $display("[%0t ns] SIM_ERROR: block fifo overflow.", $time);
            error_count = error_count + 1;
        end

        for (neuron_idx = 0; neuron_idx < P_NUM_OUTPUT_NEURONS; neuron_idx = neuron_idx + 1) begin
            if (dense_currents_latched[neuron_idx] !== block_currents_latched[neuron_idx]) begin
                $display("[%0t ns] SIM_ERROR: neuron%0d mismatch, dense=%0d block=%0d.",
                         $time, neuron_idx, dense_currents_latched[neuron_idx], block_currents_latched[neuron_idx]);
                error_count = error_count + 1;
            end
        end

        if (error_count == 0) begin
            $display("[%0t ns] SIM_PASS: block AER chain matches dense AER linear output.", $time);
        end else begin
            $display("[%0t ns] SIM_FAIL: block AER chain compare failed, error_count=%0d.", $time, error_count);
        end
        $finish;
    end

endmodule

module GPT_chain_weight_bram_stub #(
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
    GPT_chain_weight_bram_stub #(.P_NEURON_ID(0)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_1(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_chain_weight_bram_stub #(.P_NEURON_ID(1)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_2(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_chain_weight_bram_stub #(.P_NEURON_ID(2)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_3(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_chain_weight_bram_stub #(.P_NEURON_ID(3)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_4(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_chain_weight_bram_stub #(.P_NEURON_ID(4)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_5(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_chain_weight_bram_stub #(.P_NEURON_ID(5)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_6(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_chain_weight_bram_stub #(.P_NEURON_ID(6)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_7(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_chain_weight_bram_stub #(.P_NEURON_ID(7)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_8(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_chain_weight_bram_stub #(.P_NEURON_ID(8)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule

module weights_9(input wire clka, input wire ena, input wire [8:0] addra, output wire signed [63:0] douta);
    GPT_chain_weight_bram_stub #(.P_NEURON_ID(9)) u_stub(.clka(clka), .ena(ena), .addra(addra), .douta(douta));
endmodule
