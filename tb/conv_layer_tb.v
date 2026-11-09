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
    localparam P_USE_EVENT_DRIVEN_PE = 1;
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

    wire tb_parallel_current_ch0_wr_en;
    wire [$clog2(P_NUM_INPUT_PIXELS)-1:0] tb_parallel_current_ch0_wr_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] tb_parallel_current_ch0_wr_data;
    wire tb_parallel_current_ch0_wr_active;
    wire tb_parallel_current_ch1_wr_en;
    wire [$clog2(P_NUM_INPUT_PIXELS)-1:0] tb_parallel_current_ch1_wr_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] tb_parallel_current_ch1_wr_data;
    wire tb_parallel_current_ch1_wr_active;
    wire tb_parallel_current_wr_done;
    wire tb_conv_ch0_done_w;
    wire tb_conv_ch1_done_w;
    wire tb_parallel_current_buffer_valid;
    reg tb_parallel_current_rd_en;
    reg [LP_OUTPUT_ADDR_WIDTH-1:0] tb_parallel_current_rd_addr;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] tb_parallel_current_rd_data;
    wire tb_parallel_current_rd_valid;

    // 仅用于仿真观察：从 current buffer 读回后重新拼成完整二维数组。
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0]
        tb_parallel_all_currents_I [0:LP_NUM_OUTPUT_FEATURES-1];
    reg tb_parallel_all_currents_valid;

    // 直接从卷积 PE 写流抓取，便于不经过 current buffer 观察卷积电流二维数组。
    reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0]
        tb_direct_all_currents_I [0:LP_NUM_OUTPUT_FEATURES-1];
    reg [LP_NUM_OUTPUT_FEATURES-1:0] tb_direct_current_active_bitmap;
    reg tb_direct_all_currents_valid;
    reg tb_event_ch0_done_seen;
    reg tb_event_ch1_done_seen;

    integer readback_idx;
    integer direct_init_idx;

    function signed [P_NEURON_VALUE_TOTAL_BITS-1:0] get_direct_current;
        input integer ch;
        input integer row;
        input integer col;
        integer flat_idx;
        integer current_addr;
        integer feature_idx;
        begin
            flat_idx = (row * P_INPUT_WIDTH) + col;
            current_addr = P_NUM_INPUT_PIXELS - 1 - flat_idx;
            feature_idx = (ch * P_NUM_INPUT_PIXELS) + current_addr;
            get_direct_current = tb_direct_all_currents_I[feature_idx];
        end
    endfunction

    function get_direct_current_active;
        input integer ch;
        input integer row;
        input integer col;
        integer flat_idx;
        integer current_addr;
        integer feature_idx;
        begin
            flat_idx = (row * P_INPUT_WIDTH) + col;
            current_addr = P_NUM_INPUT_PIXELS - 1 - flat_idx;
            feature_idx = (ch * P_NUM_INPUT_PIXELS) + current_addr;
            get_direct_current_active = tb_direct_current_active_bitmap[feature_idx];
        end
    endfunction

    task print_direct_current_point;
        input integer ch;
        input integer row;
        input integer col;
        begin
            $display("[%0t ns] SIM_INFO: direct_current ch=%0d row=%0d col=%0d active=%0b data=%0d",
                     $time,
                     ch,
                     row,
                     col,
                     get_direct_current_active(ch, row, col),
                     get_direct_current(ch, row, col));
        end
    endtask

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

    generate
        if (P_USE_EVENT_DRIVEN_PE) begin : g_event_driven_conv
            event_driven_conv_pe #(
                .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
                .P_INPUT_WIDTH              (P_INPUT_WIDTH),
                .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
                .P_KERNEL_SIZE              (P_KERNEL_SIZE),
                .P_PADDING                  (P_PADDING),
                .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
                .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
                .P_KERNEL_WEIGHTS_PACKED    (P_CONV0_WEIGHTS_PACKED)
            ) u_event_conv_pe_ch0 (
                .clk                    (tb_clk),
                .rst_n                  (tb_rst_n),
                .i_start                (tb_calc_start),
                .i_input_spike_vector   (tb_input_spike_vector),
                .o_current_valid        (tb_parallel_current_ch0_wr_en),
                .o_current_addr         (tb_parallel_current_ch0_wr_addr),
                .o_current_data         (tb_parallel_current_ch0_wr_data),
                .o_current_active       (tb_parallel_current_ch0_wr_active),
                .o_done                 (tb_conv_ch0_done_w)
            );

            event_driven_conv_pe #(
                .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
                .P_INPUT_WIDTH              (P_INPUT_WIDTH),
                .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
                .P_KERNEL_SIZE              (P_KERNEL_SIZE),
                .P_PADDING                  (P_PADDING),
                .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
                .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
                .P_KERNEL_WEIGHTS_PACKED    (P_CONV1_WEIGHTS_PACKED)
            ) u_event_conv_pe_ch1 (
                .clk                    (tb_clk),
                .rst_n                  (tb_rst_n),
                .i_start                (tb_calc_start),
                .i_input_spike_vector   (tb_input_spike_vector),
                .o_current_valid        (tb_parallel_current_ch1_wr_en),
                .o_current_addr         (tb_parallel_current_ch1_wr_addr),
                .o_current_data         (tb_parallel_current_ch1_wr_data),
                .o_current_active       (tb_parallel_current_ch1_wr_active),
                .o_done                 (tb_conv_ch1_done_w)
            );

            assign tb_parallel_current_wr_done = tb_conv_ch0_done_w && tb_conv_ch1_done_w;
        end else begin : g_legacy_conv
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

            assign tb_conv_ch0_done_w = tb_parallel_current_wr_done;
            assign tb_conv_ch1_done_w = tb_parallel_current_wr_done;
        end
    endgenerate

    conv_current_pingpong_buffer #(
        .P_NUM_INPUT_PIXELS          (P_NUM_INPUT_PIXELS),
        .P_NUM_OUTPUT_CHANNELS       (P_NUM_OUTPUT_CHANNELS),
        .P_NEURON_VALUE_TOTAL_BITS   (P_NEURON_VALUE_TOTAL_BITS)
    ) current_buffer_inst (
        .clk                         (tb_clk),
        .rst_n                       (tb_rst_n),
        .i_flush                     (1'b0),
        .i_clear                     (tb_calc_start),
        .i_write_start               (1'b0),
        .i_write_spike_vector        (tb_input_spike_vector),
        .i_current_ch0_wr_en         (tb_parallel_current_ch0_wr_en),
        .i_current_ch0_wr_addr       (tb_parallel_current_ch0_wr_addr),
        .i_current_ch0_wr_data       (tb_parallel_current_ch0_wr_data),
        .i_current_ch0_wr_active     (tb_parallel_current_ch0_wr_active),
        .i_current_ch1_wr_en         (tb_parallel_current_ch1_wr_en),
        .i_current_ch1_wr_addr       (tb_parallel_current_ch1_wr_addr),
        .i_current_ch1_wr_data       (tb_parallel_current_ch1_wr_data),
        .i_current_ch1_wr_active     (tb_parallel_current_ch1_wr_active),
        .i_current_wr_done           (tb_parallel_current_wr_done),
        .i_read_start                (1'b0),
        .i_release_read_buffer       (1'b0),
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
        .o_current_buffer_valid      (tb_parallel_current_buffer_valid),
        .o_write_ready               (),
        .o_read_valid                (),
        .o_read_spike_vector         (),
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

    always @(posedge tb_clk or negedge tb_rst_n) begin
        if (!tb_rst_n) begin
            tb_direct_all_currents_valid <= 1'b0;
            tb_direct_current_active_bitmap <= {LP_NUM_OUTPUT_FEATURES{1'b0}};
            tb_event_ch0_done_seen <= 1'b0;
            tb_event_ch1_done_seen <= 1'b0;
            for (direct_init_idx = 0; direct_init_idx < LP_NUM_OUTPUT_FEATURES; direct_init_idx = direct_init_idx + 1) begin
                tb_direct_all_currents_I[direct_init_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            end
        end else begin
            if (tb_calc_start) begin
                tb_direct_all_currents_valid <= 1'b0;
                tb_direct_current_active_bitmap <= {LP_NUM_OUTPUT_FEATURES{1'b0}};
                tb_event_ch0_done_seen <= 1'b0;
                tb_event_ch1_done_seen <= 1'b0;
                for (direct_init_idx = 0; direct_init_idx < LP_NUM_OUTPUT_FEATURES; direct_init_idx = direct_init_idx + 1) begin
                    tb_direct_all_currents_I[direct_init_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                end
            end else begin
                if (tb_parallel_current_ch0_wr_en) begin
                    tb_direct_all_currents_I[tb_parallel_current_ch0_wr_addr] <= tb_parallel_current_ch0_wr_data;
                    tb_direct_current_active_bitmap[tb_parallel_current_ch0_wr_addr] <= tb_parallel_current_ch0_wr_active;
                end

                if (tb_parallel_current_ch1_wr_en) begin
                    tb_direct_all_currents_I[P_NUM_INPUT_PIXELS + tb_parallel_current_ch1_wr_addr] <= tb_parallel_current_ch1_wr_data;
                    tb_direct_current_active_bitmap[P_NUM_INPUT_PIXELS + tb_parallel_current_ch1_wr_addr] <= tb_parallel_current_ch1_wr_active;
                end

                if (tb_conv_ch0_done_w) begin
                    tb_event_ch0_done_seen <= 1'b1;
                end
                if (tb_conv_ch1_done_w) begin
                    tb_event_ch1_done_seen <= 1'b1;
                end

                if (tb_event_ch0_done_seen && tb_event_ch1_done_seen) begin
                    tb_direct_all_currents_valid <= 1'b1;
                end
            end
        end
    end

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
        tb_direct_all_currents_valid = 1'b0;
        tb_direct_current_active_bitmap = {LP_NUM_OUTPUT_FEATURES{1'b0}};
        tb_event_ch0_done_seen = 1'b0;
        tb_event_ch1_done_seen = 1'b0;
        repeat(5) @(posedge tb_clk);

        tb_rst_n = 1'b1;
        $display("[%0t ns] SIM_INFO: rst_n is RELEASED!!!.", $time);
        repeat(2) @(posedge tb_clk);

        tb_input_spike_vector = {28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000010110000000,
                                 28'b0000000000000001100111100000,
                                 28'b0000000000100110000111100000,
                                 28'b0000000000111100000111000000,
                                 28'b0000000011110000101110000000,
                                 28'b0000000111100000111000000000,
                                 28'b0000000111000001100000000000,
                                 28'b0000001111000011100000000000,
                                 28'b0000000111001110000000000000,
                                 28'b0000000011111110000000000000,
                                 28'b0000000001111111100000000000,
                                 28'b0000000000011111100000000000,
                                 28'b0000000000011100011100000000,
                                 28'b0000000000011000001100000000,
                                 28'b0000000000011000000110000000,
                                 28'b0000000000011000001110000000,
                                 28'b0000000000001100001100000000,
                                 28'b0000000000001100011100000000,
                                 28'b0000000000001110011100000000,
                                 28'b0000000000000000110000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000,
                                 28'b0000000000000000000000000000};

        $display("[%0t ns] SIM_INFO: input vector is READY!!!.", $time);
        repeat(2) @(posedge tb_clk);

        tb_calc_start = 1'b1;
        $display("[%0t ns] SIM_INFO: enable ASSERTED,processing START!!!.", $time);
        @(posedge tb_clk);
        tb_calc_start = 1'b0;

        wait (tb_parallel_current_buffer_valid);
        wait (tb_direct_all_currents_valid);
        $display("[%0t ns] SIM_INFO: direct current capture is VALID!!!.", $time);
        print_direct_current_point(0, 6, 14);
        print_direct_current_point(0, 14, 14);
        print_direct_current_point(1, 6, 14);
        print_direct_current_point(1, 14, 14);
        readback_current_buffer();
        #50;
        $display("[%0t ns] SIM_INFO: simulation DONE!!!.", $time);
        $finish;
    end

    initial begin
        #2000000;
        $display("[%0t ns] SIM_TIMEOUT: buffer_valid=%0b direct_valid=%0b wr_done=%0b ch0_done=%0b ch1_done=%0b.",
                 $time,
                 tb_parallel_current_buffer_valid,
                 tb_direct_all_currents_valid,
                 tb_parallel_current_wr_done,
                 tb_event_ch0_done_seen,
                 tb_event_ch1_done_seen);
        $finish;
    end

endmodule
