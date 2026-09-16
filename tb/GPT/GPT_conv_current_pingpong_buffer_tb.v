`timescale 1ns/1ps

module GPT_conv_current_pingpong_buffer_tb;

    localparam P_NUM_INPUT_PIXELS = 8;
    localparam P_NUM_OUTPUT_CHANNELS = 2;
    localparam P_NEURON_VALUE_TOTAL_BITS = 16;
    localparam LP_ADDR_WIDTH = $clog2(P_NUM_INPUT_PIXELS);
    localparam LP_TOTAL_ADDR_WIDTH = $clog2(P_NUM_INPUT_PIXELS * P_NUM_OUTPUT_CHANNELS);

    reg clk;
    reg rst_n;
    reg i_clear;
    reg i_write_start;
    reg [P_NUM_INPUT_PIXELS-1:0] i_write_spike_vector;
    reg i_current_ch0_wr_en;
    reg [LP_ADDR_WIDTH-1:0] i_current_ch0_wr_addr;
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] i_current_ch0_wr_data;
    reg i_current_ch0_wr_active;
    reg i_current_ch1_wr_en;
    reg [LP_ADDR_WIDTH-1:0] i_current_ch1_wr_addr;
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] i_current_ch1_wr_data;
    reg i_current_ch1_wr_active;
    reg i_current_wr_done;
    reg i_read_start;
    reg i_release_read_buffer;
    reg i_current_ch0_rd_en;
    reg [LP_ADDR_WIDTH-1:0] i_current_ch0_rd_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] o_current_ch0_rd_data;
    wire o_current_ch0_rd_valid;
    reg i_current_ch1_rd_en;
    reg [LP_ADDR_WIDTH-1:0] i_current_ch1_rd_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] o_current_ch1_rd_data;
    wire o_current_ch1_rd_valid;
    reg i_current_rd_en;
    reg [LP_TOTAL_ADDR_WIDTH-1:0] i_current_rd_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] o_current_rd_data;
    wire o_current_rd_valid;
    wire o_current_buffer_ready;
    wire o_write_ready;
    wire o_read_valid;
    wire [P_NUM_INPUT_PIXELS-1:0] o_read_spike_vector;
    wire o_write_buffer_sel;
    wire o_read_buffer_sel;
    wire [1:0] o_buffer_valid_bits;
    wire [P_NUM_INPUT_PIXELS * P_NUM_OUTPUT_CHANNELS - 1:0] o_current_valid_bitmap;

    conv_current_pingpong_buffer #(
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_NUM_OUTPUT_CHANNELS      (P_NUM_OUTPUT_CHANNELS),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS)
    ) dut (
        .clk                        (clk),
        .rst_n                      (rst_n),
        .i_flush                    (1'b0),
        .i_clear                    (i_clear),
        .i_write_start              (i_write_start),
        .i_write_spike_vector       (i_write_spike_vector),
        .i_current_ch0_wr_en        (i_current_ch0_wr_en),
        .i_current_ch0_wr_addr      (i_current_ch0_wr_addr),
        .i_current_ch0_wr_data      (i_current_ch0_wr_data),
        .i_current_ch0_wr_active    (i_current_ch0_wr_active),
        .i_current_ch1_wr_en        (i_current_ch1_wr_en),
        .i_current_ch1_wr_addr      (i_current_ch1_wr_addr),
        .i_current_ch1_wr_data      (i_current_ch1_wr_data),
        .i_current_ch1_wr_active    (i_current_ch1_wr_active),
        .i_current_wr_done          (i_current_wr_done),
        .i_read_start               (i_read_start),
        .i_release_read_buffer      (i_release_read_buffer),
        .i_current_ch0_rd_en        (i_current_ch0_rd_en),
        .i_current_ch0_rd_addr      (i_current_ch0_rd_addr),
        .o_current_ch0_rd_data      (o_current_ch0_rd_data),
        .o_current_ch0_rd_valid     (o_current_ch0_rd_valid),
        .i_current_ch1_rd_en        (i_current_ch1_rd_en),
        .i_current_ch1_rd_addr      (i_current_ch1_rd_addr),
        .o_current_ch1_rd_data      (o_current_ch1_rd_data),
        .o_current_ch1_rd_valid     (o_current_ch1_rd_valid),
        .i_current_rd_en            (i_current_rd_en),
        .i_current_rd_addr          (i_current_rd_addr),
        .o_current_rd_data          (o_current_rd_data),
        .o_current_rd_valid         (o_current_rd_valid),
        .o_current_buffer_ready     (o_current_buffer_ready),
        .o_write_ready              (o_write_ready),
        .o_read_valid               (o_read_valid),
        .o_read_spike_vector        (o_read_spike_vector),
        .o_write_buffer_sel         (o_write_buffer_sel),
        .o_read_buffer_sel          (o_read_buffer_sel),
        .o_buffer_valid_bits        (o_buffer_valid_bits),
        .o_current_valid_bitmap     (o_current_valid_bitmap)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    task clear_inputs;
        begin
            i_clear = 1'b0;
            i_write_start = 1'b0;
            i_write_spike_vector = {P_NUM_INPUT_PIXELS{1'b0}};
            i_current_ch0_wr_en = 1'b0;
            i_current_ch0_wr_addr = {LP_ADDR_WIDTH{1'b0}};
            i_current_ch0_wr_data = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            i_current_ch0_wr_active = 1'b0;
            i_current_ch1_wr_en = 1'b0;
            i_current_ch1_wr_addr = {LP_ADDR_WIDTH{1'b0}};
            i_current_ch1_wr_data = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            i_current_ch1_wr_active = 1'b0;
            i_current_wr_done = 1'b0;
            i_read_start = 1'b0;
            i_release_read_buffer = 1'b0;
            i_current_ch0_rd_en = 1'b0;
            i_current_ch0_rd_addr = {LP_ADDR_WIDTH{1'b0}};
            i_current_ch1_rd_en = 1'b0;
            i_current_ch1_rd_addr = {LP_ADDR_WIDTH{1'b0}};
            i_current_rd_en = 1'b0;
            i_current_rd_addr = {LP_TOTAL_ADDR_WIDTH{1'b0}};
        end
    endtask

    task write_start;
        input [P_NUM_INPUT_PIXELS-1:0] spike_vector;
        begin
            @(negedge clk);
            i_write_start = 1'b1;
            i_write_spike_vector = spike_vector;
            @(negedge clk);
            i_write_start = 1'b0;
        end
    endtask

    task write_ch0_sample;
        input [LP_ADDR_WIDTH-1:0] addr;
        input signed [P_NEURON_VALUE_TOTAL_BITS-1:0] data;
        begin
            @(negedge clk);
            i_current_ch0_wr_en = 1'b1;
            i_current_ch0_wr_addr = addr;
            i_current_ch0_wr_data = data;
            i_current_ch0_wr_active = 1'b1;
            @(negedge clk);
            i_current_ch0_wr_en = 1'b0;
            i_current_ch0_wr_active = 1'b0;
        end
    endtask

    task write_done;
        begin
            @(negedge clk);
            i_current_wr_done = 1'b1;
            @(negedge clk);
            i_current_wr_done = 1'b0;
        end
    endtask

    task read_ch0_check;
        input [LP_ADDR_WIDTH-1:0] addr;
        input signed [P_NEURON_VALUE_TOTAL_BITS-1:0] expected;
        begin
            @(negedge clk);
            i_current_ch0_rd_en = 1'b1;
            i_current_ch0_rd_addr = addr;
            @(posedge clk);
            #1;
            if (!o_current_ch0_rd_valid || o_current_ch0_rd_data !== expected) begin
                $display("SIM_ERROR: read addr=%0d expected=%0d got=%0d valid=%0b",
                         addr, expected, o_current_ch0_rd_data, o_current_ch0_rd_valid);
                $finish;
            end
            @(negedge clk);
            i_current_ch0_rd_en = 1'b0;
        end
    endtask

    initial begin
        rst_n = 1'b0;
        clear_inputs();

        repeat (4) @(negedge clk);
        rst_n = 1'b1;

        // 写入第一个时间步，形成第一个可读 buffer。
        write_start(8'b1010_0101);
        write_ch0_sample(3'd1, 16'sd11);
        write_done();
        repeat (2) @(negedge clk);

        if (!o_read_valid || o_read_spike_vector !== 8'b1010_0101) begin
            $display("SIM_ERROR: first buffer metadata mismatch");
            $finish;
        end
        read_ch0_check(3'd1, 16'sd11);

        // 保持读旧 buffer，同时写入第二个时间步；写完成不能抢占 read_sel。
        write_start(8'b0101_1010);
        write_ch0_sample(3'd3, 16'sd33);
        write_done();
        repeat (2) @(negedge clk);

        if (o_read_spike_vector !== 8'b1010_0101) begin
            $display("SIM_ERROR: read buffer switched before release");
            $finish;
        end

        @(negedge clk);
        i_release_read_buffer = 1'b1;
        @(negedge clk);
        i_release_read_buffer = 1'b0;
        i_read_start = 1'b1;
        @(negedge clk);
        i_read_start = 1'b0;
        repeat (1) @(negedge clk);

        if (!o_read_valid || o_read_spike_vector !== 8'b0101_1010) begin
            $display("SIM_ERROR: second buffer metadata mismatch");
            $finish;
        end
        read_ch0_check(3'd3, 16'sd33);

        $display("SIM_INFO: conv_current_pingpong_buffer_tb PASS");
        $finish;
    end

endmodule
