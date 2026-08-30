`timescale 1ns/1ps

module conv_lif_tb();

    localparam CLK_PERIOD = 10; // 时钟周期：100MHz

    localparam P_INPUT_HEIGHT = 28;
    localparam P_INPUT_WIDTH = 28;
    localparam P_NUM_INPUT_PIXELS = 784;
    localparam P_NUM_OUTPUT_CHANNELS = 2;
    localparam P_KERNEL_SIZE = 3;
    localparam P_PADDING = 1;
    localparam P_WEIGHT_BIT_WIDTH = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_NUM_NEURONS = 1568;
    localparam P_NEURON_VALUE_FRAC_BITS = 12;

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

    wire [P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS - 1:0][P_NEURON_VALUE_TOTAL_BITS-1:0]
         all_currents_I;
    wire all_currents_valid;

    wire current_ram_ready;
    wire current_ram_rd_en;
    wire [$clog2(P_NUM_NEURONS)-1:0] current_ram_rd_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] current_ram_rd_data;
    wire current_ram_rd_valid;
    wire [P_NUM_NEURONS-1:0] current_valid_bitmap;

    reg r_enable_layer;
    reg r_enable_layer_pending;

    wire [P_NUM_NEURONS-1:0] tb_all_spikes_out;
    wire tb_all_spikes_valid;
    wire tb_event_valid;
    wire [$clog2(P_NUM_NEURONS)-1:0] tb_event_addr;
    wire tb_event_frame_done;
    wire tb_layer_ready;
    wire [31:0] tb_skip_count;
    wire [31:0] tb_update_count;

    conv_layer_parallel #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_NUM_OUTPUT_CHANNELS      (P_NUM_OUTPUT_CHANNELS),
        .P_KERNEL_SIZE              (P_KERNEL_SIZE),
        .P_PADDING                  (P_PADDING),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_ENABLE_COMPAT_READBACK   (0),
        .P_CONV0_WEIGHTS_PACKED     (P_CONV0_WEIGHTS_PACKED),
        .P_CONV1_WEIGHTS_PACKED     (P_CONV1_WEIGHTS_PACKED)
    ) conv_layer_inst(
        .clk                        (tb_clk),
        .rst_n                      (tb_rst_n),
        .i_calc_start               (tb_calc_start),
        .i_input_spike_vector       (tb_input_spike_vector),
        .i_current_rd_en            (current_ram_rd_en),
        .i_current_rd_addr          (current_ram_rd_addr),
        .o_current_rd_data          (current_ram_rd_data),
        .o_current_rd_valid         (current_ram_rd_valid),
        .o_current_ram_ready        (current_ram_ready),
        .o_current_valid_bitmap    (current_valid_bitmap),
        .o_all_currents_I           (all_currents_I),
        .o_all_currents_valid       (all_currents_valid)
    );

    always @(posedge tb_clk or negedge tb_rst_n) begin
        if (!tb_rst_n) begin
            r_enable_layer <= 1'b0;
            r_enable_layer_pending <= 1'b0;
        end else begin
            r_enable_layer <= 1'b0;

            // current RAM 写完可能早于 LIF 清零完成，所以先记住请求。
            if (current_ram_ready) begin
                r_enable_layer_pending <= 1'b1;
            end

            if (r_enable_layer_pending && tb_layer_ready) begin
                r_enable_layer <= 1'b1;
                r_enable_layer_pending <= 1'b0;
            end
        end
    end

    conv_lif_layer_sparse #(
        .P_NUM_NEURONS              (P_NUM_NEURONS),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_KERNEL_SIZE              (P_KERNEL_SIZE),
        .P_PADDING                  (P_PADDING),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_NEURON_VALUE_FRAC_BITS   (P_NEURON_VALUE_FRAC_BITS),
        .P_SKIP_THRESHOLD_SHIFT     (5)
    ) conv_lif_layer_sparse_inst(
        .clk                        (tb_clk),
        .rst_n                      (tb_rst_n),
        .i_enable_layer             (r_enable_layer),
        .i_input_spike_vector       (tb_input_spike_vector),
        .i_all_currents_I           (all_currents_I),
        .i_current_ram_rd_data      (current_ram_rd_data),
        .i_current_ram_rd_valid     (current_ram_rd_valid),
        .o_current_ram_rd_en        (current_ram_rd_en),
        .o_current_ram_rd_addr      (current_ram_rd_addr),
        .o_all_spikes_out           (tb_all_spikes_out),
        .o_all_spikes_valid         (tb_all_spikes_valid),
        .o_event_valid              (tb_event_valid),
        .o_event_addr               (tb_event_addr),
        .o_event_frame_done         (tb_event_frame_done),
        .o_layer_ready              (tb_layer_ready),
        .o_skip_count               (tb_skip_count),
        .o_update_count             (tb_update_count)
    );

    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    initial begin
        $display("[%0t ns] SIM_INFO: conv_lif_tb simulation START!!!", $time);

        tb_rst_n = 1'b0;
        tb_calc_start = 1'b0;
        tb_input_spike_vector = 784'b0;
        repeat(5) @(posedge tb_clk);

        tb_rst_n = 1'b1;
        $display("[%0t ns] SIM_INFO: rst_n is RELEASED!!!.", $time);
        repeat(2) @(posedge tb_clk);

        tb_input_spike_vector = {28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000111010000,
                                 28'b0000000000000001110111100000,
                                 28'b0000000000001111000111100000,
                                 28'b0000000000111100001111000000,
                                 28'b0000000001111000001100000000,
                                 28'b0000000011000000011000000000,
                                 28'b0000000111000001110000000000,
                                 28'b0000000110000011000000000000,
                                 28'b0000000111000110000000000000,
                                 28'b0000000011111110000000000000,
                                 28'b0000000001011111000000000000,
                                 28'b0000000000011111111000000000,
                                 28'b0000000000111000001100000000,
                                 28'b0000000000011000001100000000,
                                 28'b0000000000011000000110000000,
                                 28'b0000000000111000000100000000,
                                 28'b0000000000011000001100000000,
                                 28'b0000000000011100001100000000,
                                 28'b0000000000000010010000000000,
                                 28'b0000000000000111010000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000};
        $display("[%0t ns] SIM_INFO: input vector is READY!!!.", $time);
        repeat(2) @(posedge tb_clk);

        tb_calc_start = 1'b1;
        $display("[%0t ns] SIM_INFO: conv processing START!!!.", $time);
        repeat(2) @(posedge tb_clk);
        tb_calc_start = 1'b0;

        @(posedge tb_all_spikes_valid);
        $display("[%0t ns] SIM_INFO: simulation DONE, skip=%0d, update=%0d.", $time, tb_skip_count, tb_update_count);
        #50;
        $finish;
    end

endmodule

