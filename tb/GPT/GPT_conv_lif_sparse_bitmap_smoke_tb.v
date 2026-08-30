`timescale 1ns / 1ps

module GPT_conv_lif_sparse_bitmap_smoke_tb;

    localparam P_NUM_NEURONS = 16;
    localparam P_NUM_INPUT_PIXELS = 16;
    localparam P_INPUT_HEIGHT = 4;
    localparam P_INPUT_WIDTH = 4;
    localparam P_KERNEL_SIZE = 3;
    localparam P_PADDING = 1;
    localparam P_VALUE_BITS = 26;
    localparam P_FRAC_BITS = 12;
    localparam P_ADDR_WIDTH = 4;

    reg clk;
    reg rst_n;
    reg i_enable_layer;
    reg [P_NUM_INPUT_PIXELS-1:0] i_input_spike_vector;
    reg [P_NUM_NEURONS-1:0] i_current_valid_bitmap;
    reg signed [P_VALUE_BITS-1:0] i_current_ram_rd_data;
    reg i_current_ram_rd_valid;

    wire o_current_ram_rd_en;
    wire [P_ADDR_WIDTH-1:0] o_current_ram_rd_addr;
    wire [P_NUM_NEURONS-1:0] o_all_spikes_out;
    wire o_all_spikes_valid;
    wire o_event_valid;
    wire [P_ADDR_WIDTH-1:0] o_event_addr;
    wire o_event_frame_done;
    wire o_layer_ready;
    wire [31:0] o_skip_count;
    wire [31:0] o_update_count;

    wire signed [P_NUM_NEURONS-1:0][P_VALUE_BITS-1:0] unused_currents;

    integer error_count;

    assign unused_currents = {P_NUM_NEURONS * P_VALUE_BITS{1'b0}};

    conv_lif_layer_sparse #(
        .P_NUM_NEURONS(P_NUM_NEURONS),
        .P_NUM_INPUT_PIXELS(P_NUM_INPUT_PIXELS),
        .P_INPUT_HEIGHT(P_INPUT_HEIGHT),
        .P_INPUT_WIDTH(P_INPUT_WIDTH),
        .P_KERNEL_SIZE(P_KERNEL_SIZE),
        .P_PADDING(P_PADDING),
        .P_NEURON_VALUE_TOTAL_BITS(P_VALUE_BITS),
        .P_NEURON_VALUE_FRAC_BITS(P_FRAC_BITS),
        .P_SKIP_THRESHOLD_SHIFT(5)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .i_enable_layer(i_enable_layer),
        .i_input_spike_vector(i_input_spike_vector),
        .i_current_valid_bitmap(i_current_valid_bitmap),
        .i_all_currents_I(unused_currents),
        .i_current_ram_rd_data(i_current_ram_rd_data),
        .i_current_ram_rd_valid(i_current_ram_rd_valid),
        .o_current_ram_rd_en(o_current_ram_rd_en),
        .o_current_ram_rd_addr(o_current_ram_rd_addr),
        .o_all_spikes_out(o_all_spikes_out),
        .o_all_spikes_valid(o_all_spikes_valid),
        .o_event_valid(o_event_valid),
        .o_event_addr(o_event_addr),
        .o_event_frame_done(o_event_frame_done),
        .o_layer_ready(o_layer_ready),
        .o_skip_count(o_skip_count),
        .o_update_count(o_update_count)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    initial begin
        rst_n = 1'b0;
        i_enable_layer = 1'b0;
        i_input_spike_vector = {P_NUM_INPUT_PIXELS{1'b0}};
        i_current_valid_bitmap = {P_NUM_NEURONS{1'b0}};
        i_current_ram_rd_data = 26'sd0;
        i_current_ram_rd_valid = 1'b0;
        error_count = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        wait (o_layer_ready);
        @(posedge clk);
        i_current_valid_bitmap = 16'b0000_0000_0010_1010;
        i_enable_layer = 1'b1;
        @(posedge clk);
        i_enable_layer = 1'b0;
        i_current_valid_bitmap = 16'b0;

        wait (o_all_spikes_valid);
        #1;
        if (o_update_count !== 32'd3) begin
            $display("ERROR: expected update_count=3, got %0d", o_update_count);
            error_count = error_count + 1;
        end
        if (o_skip_count !== 32'd13) begin
            $display("ERROR: expected skip_count=13, got %0d", o_skip_count);
            error_count = error_count + 1;
        end

        if (error_count == 0) begin
            $display("SIM PASS: conv_lif sparse bitmap smoke test passed, skip=%0d update=%0d", o_skip_count, o_update_count);
        end else begin
            $display("SIM FAIL: error_count=%0d", error_count);
        end
        $finish;
    end

    initial begin
        #2000;
        $display("TIMEOUT: ready=%0b valid=%0b skip=%0d update=%0d", o_layer_ready, o_all_spikes_valid, o_skip_count, o_update_count);
        $finish;
    end

endmodule
