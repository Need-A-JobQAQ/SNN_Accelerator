`timescale 1ns/1ps

module conv_layer_tb;

    localparam CLK_PERIOD = 10; // 时钟周期�?100MHz

    localparam P_INPUT_HEIGHT = 28;
    localparam P_INPUT_WIDTH = 28;
    localparam P_NUM_INPUT_PIXELS = 784;
    localparam P_NUM_OUTPUT_CHANNELS = 2;
    localparam P_KERNEL_SIZE = 3;
    localparam P_PADDING = 1;
    localparam P_WEIGHT_BIT_WIDTH = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam LP_NUM_OUTPUT_FEATURES = P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS;
    localparam LP_OUTPUT_ADDR_WIDTH = $clog2(LP_NUM_OUTPUT_FEATURES);
    localparam [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV0_WEIGHTS_PACKED = {16'hFA52,16'h1085,16'hFF5B,
                                                                                                    16'h0602,16'h0A60,16'hF5AA,
                                                                                                    16'h1314,16'h05C8,16'hEA50};

    localparam [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV1_WEIGHTS_PACKED = {16'hE764,16'h41B2,16'h272A,
                                                                                                    16'hF6DD,16'h61AB,16'hF8CB,
                                                                                                    16'h36E8,16'h2EFE,16'hE59C};

    reg tb_clk;
    reg tb_rst_n;
    reg tb_calc_start;
    reg [P_NUM_INPUT_PIXELS-1:0] tb_input_spike_vector;

    //wire [P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS - 1:0][P_NEURON_VALUE_TOTAL_BITS-1:0]
    //     tb_all_currents_I;
    //wire tb_all_currents_valid;
    wire tb_parallel_current_ch0_wr_en;
    wire [$clog2(P_NUM_INPUT_PIXELS)-1:0] tb_parallel_current_ch0_wr_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] tb_parallel_current_ch0_wr_data;
    wire tb_parallel_current_ch0_wr_active;
    wire tb_parallel_current_ch1_wr_en;
    wire [$clog2(P_NUM_INPUT_PIXELS)-1:0] tb_parallel_current_ch1_wr_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] tb_parallel_current_ch1_wr_data;
    wire tb_parallel_current_ch1_wr_active;
    wire tb_parallel_current_wr_done;
    wire tb_parallel_current_buffer_ready;
    reg tb_parallel_current_rd_en;
    reg [LP_OUTPUT_ADDR_WIDTH-1:0] tb_parallel_current_rd_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] tb_parallel_current_rd_data;
    wire tb_parallel_current_rd_valid;

    // 仅用于仿真观察：从 current buffer 读回后重新拼成完整二维数组。
    reg signed [LP_NUM_OUTPUT_FEATURES-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0]
        tb_parallel_all_currents_I;
    reg tb_parallel_all_currents_valid;

    integer readback_idx;

    //conv_layer #(
    //    .P_INPUT_HEIGHT             (P_INPUT_HEIGHT)            ,
    //    .P_INPUT_WIDTH              (P_INPUT_WIDTH)             ,
    //    .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS)        ,
    //    .P_NUM_OUTPUT_CHANNELS      (P_NUM_OUTPUT_CHANNELS)     ,
    //    .P_KERNEL_SIZE              (P_KERNEL_SIZE)             ,
    //    .P_PADDING                  (P_PADDING)                 ,
    //    .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH)        ,
    //    .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS) ,
    //    .P_CONV0_WEIGHTS_PACKED     (P_CONV0_WEIGHTS_PACKED)    ,
    //    .P_CONV1_WEIGHTS_PACKED     (P_CONV1_WEIGHTS_PACKED)
    //) conv_layer_inst(
    //    .clk                         (tb_clk)                    ,
    //    .rst_n                       (tb_rst_n)                  ,
    //    .i_calc_start                (tb_calc_start)             ,
    //    .i_input_spike_vector        (tb_input_spike_vector)     ,

    //    .o_all_currents_I            (tb_all_currents_I)         ,
    //    .o_all_currents_valid        (tb_all_currents_valid)
    //);

    conv_layer_parallel #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT)            ,
        .P_INPUT_WIDTH              (P_INPUT_WIDTH)             ,
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS)        ,
        .P_NUM_OUTPUT_CHANNELS      (P_NUM_OUTPUT_CHANNELS)     ,
        .P_KERNEL_SIZE              (P_KERNEL_SIZE)             ,
        .P_PADDING                  (P_PADDING)                 ,
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH)        ,
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS) ,
        .P_CONV0_WEIGHTS_PACKED     (P_CONV0_WEIGHTS_PACKED)    ,
        .P_CONV1_WEIGHTS_PACKED     (P_CONV1_WEIGHTS_PACKED)
    ) conv_layer_parallel_inst(
        .clk                         (tb_clk)                    ,
        .rst_n                       (tb_rst_n)                  ,
        .i_calc_start                (tb_calc_start)             ,
        .i_input_spike_vector        (tb_input_spike_vector)     ,
        .o_current_ch0_wr_en         (tb_parallel_current_ch0_wr_en),
        .o_current_ch0_wr_addr       (tb_parallel_current_ch0_wr_addr),
        .o_current_ch0_wr_data       (tb_parallel_current_ch0_wr_data),
        .o_current_ch0_wr_active     (tb_parallel_current_ch0_wr_active),
        .o_current_ch1_wr_en         (tb_parallel_current_ch1_wr_en),
        .o_current_ch1_wr_addr       (tb_parallel_current_ch1_wr_addr),
        .o_current_ch1_wr_data       (tb_parallel_current_ch1_wr_data),
        .o_current_ch1_wr_active     (tb_parallel_current_ch1_wr_active),
        .o_current_wr_done           (tb_parallel_current_wr_done)
    );

    conv_current_pingpong_buffer #(
        .P_NUM_INPUT_PIXELS          (P_NUM_INPUT_PIXELS),
        .P_NUM_OUTPUT_CHANNELS       (P_NUM_OUTPUT_CHANNELS),
        .P_NEURON_VALUE_TOTAL_BITS   (P_NEURON_VALUE_TOTAL_BITS)
    ) current_buffer_inst (
        .clk                         (tb_clk),
        .rst_n                       (tb_rst_n),
        .i_clear                     (tb_calc_start),
        .i_current_ch0_wr_en         (tb_parallel_current_ch0_wr_en),
        .i_current_ch0_wr_addr       (tb_parallel_current_ch0_wr_addr),
        .i_current_ch0_wr_data       (tb_parallel_current_ch0_wr_data),
        .i_current_ch0_wr_active     (tb_parallel_current_ch0_wr_active),
        .i_current_ch1_wr_en         (tb_parallel_current_ch1_wr_en),
        .i_current_ch1_wr_addr       (tb_parallel_current_ch1_wr_addr),
        .i_current_ch1_wr_data       (tb_parallel_current_ch1_wr_data),
        .i_current_ch1_wr_active     (tb_parallel_current_ch1_wr_active),
        .i_current_wr_done           (tb_parallel_current_wr_done),
        .i_current_ch0_rd_en         (1'b0),
        .i_current_ch0_rd_addr       ({($clog2(P_NUM_INPUT_PIXELS)){1'b0}}),
        .o_current_ch0_rd_data       (),
        .o_current_ch0_rd_valid      (),
        .i_current_ch1_rd_en         (1'b0),
        .i_current_ch1_rd_addr       ({($clog2(P_NUM_INPUT_PIXELS)){1'b0}}),
        .o_current_ch1_rd_data       (),
        .o_current_ch1_rd_valid      (),
        .i_current_rd_en             (tb_parallel_current_rd_en),
        .i_current_rd_addr           (tb_parallel_current_rd_addr),
        .o_current_rd_data           (tb_parallel_current_rd_data),
        .o_current_rd_valid          (tb_parallel_current_rd_valid),
        .o_current_buffer_ready      (tb_parallel_current_buffer_ready),
        .o_write_buffer_sel          (),
        .o_read_buffer_sel           (),
        .o_buffer_valid_bits         (),
        .o_current_valid_bitmap      ()
    );

    task readback_current_buffer;
        begin
            tb_parallel_all_currents_valid = 1'b0;

            for (readback_idx = 0; readback_idx < LP_NUM_OUTPUT_FEATURES; readback_idx = readback_idx + 1) begin
                @(negedge tb_clk);
                tb_parallel_current_rd_en = 1'b1;
                tb_parallel_current_rd_addr = readback_idx[LP_OUTPUT_ADDR_WIDTH-1:0];

                @(posedge tb_clk);
                @(negedge tb_clk);
                if (tb_parallel_current_rd_valid) begin
                    tb_parallel_all_currents_I[readback_idx] = tb_parallel_current_rd_data;
                end else begin
                    $display("[%0t ns] SIM_WARN: current buffer read valid is low, addr=%0d.",
                             $time, readback_idx);
                end
            end

            @(negedge tb_clk);
            tb_parallel_current_rd_en = 1'b0;
            tb_parallel_current_rd_addr = {LP_OUTPUT_ADDR_WIDTH{1'b0}};
            tb_parallel_all_currents_valid = 1'b1;
        end
    endtask

    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    initial begin
        $display("[%0t ns] SIM_INFO: snn_top_tb simulation START!!!", $time);

        tb_rst_n = 1'b0;
        tb_calc_start = 1'b0;
        tb_input_spike_vector = 784'b0;
        tb_parallel_current_rd_en = 1'b0;
        tb_parallel_current_rd_addr = {LP_OUTPUT_ADDR_WIDTH{1'b0}};
        tb_parallel_all_currents_valid = 1'b0;
        repeat(5) @(posedge tb_clk);

        tb_rst_n = 1'b1;
        $display("[%0t ns] SIM_INFO: rst_n is RELEASED!!!.", $time);
        repeat(2) @(posedge tb_clk);

        tb_input_spike_vector = {28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000010100000,
                                 28'b0000000000000011100111110000,
                                 28'b0000000000100111000011110000,
                                 28'b0000000011111100000111000000,
                                 28'b0000000011110000011100000000,
                                 28'b0000000111100000111000000000,
                                 28'b0000001111000001110000000000,
                                 28'b0000001110000011100000000000,
                                 28'b0000000111000110000000000000,
                                 28'b0000000011111110000000000000,
                                 28'b0000000000011011100000000000,
                                 28'b0000000000111101110000000000,
                                 28'b0000000000011100011100000000,
                                 28'b0000000000111000001100000000,
                                 28'b0000000000011000000110000000,
                                 28'b0000000000111000001100000000,
                                 28'b0000000000011100001110000000,
                                 28'b0000000000001100011000000000,
                                 28'b0000000000000111111000000000,
                                 28'b0000000000000010110000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000};

        $display("[%0t ns] SIM_INFO: input vector is READY!!!.", $time);
        repeat(2) @(posedge tb_clk);

        tb_calc_start = 1'b1;
        $display("[%0t ns] SIM_INFO: enable ASSERTED,processing START!!!.", $time);

        @(posedge tb_parallel_current_buffer_ready);
        readback_current_buffer();
        #50;
        $display("[%0t ns] SIM_INFO: simulation DONE!!!.", $time);
        $finish;
    end

endmodule
