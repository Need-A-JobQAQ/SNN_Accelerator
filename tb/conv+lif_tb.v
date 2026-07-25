`timescale 1ns/1ps

module conv_lif_tb();

    localparam CLK_PERIOD = 10 ; // 时钟周期（100MHZ)

    localparam P_INPUT_HEIGHT = 28;
    localparam P_INPUT_WIDTH = 28;
    localparam P_NUM_INPUT_PIXELS = 784;
    localparam P_NUM_OUTPUT_CHANNELS = 2;
    localparam P_KERNEL_SIZE = 3;
    localparam P_PADDING = 1;
    localparam P_WEIGHT_BIT_WIDTH = 16;
    localparam P_NEURON_VALUE_TOTAL_BITS = 26;
    localparam P_NUM_NEURONS = 1568;            //新增卷积神经元层参数
    localparam P_NEURON_VALUE_FRAC_BITS = 12;   //
    localparam [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV0_WEIGHTS_PACKED = {16'hFA52,16'h1085,16'hFF5B,
                                                                                                    16'h0602,16'h0A60,16'hF5AA,
                                                                                                    16'h1314,16'h05C8,16'hEA50};

    localparam [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV1_WEIGHTS_PACKED = {16'hE764,16'h41B2,16'h272A,
                                                                                                    16'hF6DD,16'h61AB,16'hF8CB,
                                                                                                    16'h36E8,16'h2EFE,16'hE59C};
    
    reg tb_clk                     ; 
    reg tb_rst_n                   ;
    reg tb_calc_start              ;
    reg [P_NUM_INPUT_PIXELS-1:0] tb_input_spike_vector;

    wire [P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS - 1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] 
         all_currents_I         ; //
    wire all_currents_valid     ; //模块间内部信号

    reg r_enable_layer          ;        

    wire [P_NUM_NEURONS-1:0] tb_all_spikes_out;
    wire tb_all_spikes_valid                  ;


    conv_layer #(
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
    ) conv_layer_inst(
        .clk                         (tb_clk)                    ,
        .rst_n                       (tb_rst_n)                  ,
        .i_calc_start                (tb_calc_start)             ,
        .i_input_spike_vector        (tb_input_spike_vector)     ,

        .o_all_currents_I            (all_currents_I)         ,
        .o_all_currents_valid        (all_currents_valid)
    );

    always @(posedge tb_clk or negedge tb_rst_n) begin
        if (!tb_rst_n) begin
            r_enable_layer <= 1'b0;
        end else begin
            // 新增前端在这里形成一个局部流水：
            // 卷积电流有效 -> 启动卷积后 LIF
            r_enable_layer <= all_currents_valid;
        end
    end

    conv_lif_layer #(
    .P_NUM_NEURONS                   (P_NUM_NEURONS)             ,
    .P_NEURON_VALUE_TOTAL_BITS       (P_NEURON_VALUE_TOTAL_BITS) ,
    .P_NEURON_VALUE_FRAC_BITS        (P_NEURON_VALUE_FRAC_BITS)
    ) conv_lif_layer_inst(
    .clk                             (tb_clk)                    ,
    .rst_n                           (tb_rst_n)                  ,
    .i_enable_layer                  (r_enable_layer)            ,
    .i_all_currents_I                (all_currents_I)            ,

    .o_all_spikes_out                (tb_all_spikes_out)         ,
    .o_all_spikes_valid              (tb_all_spikes_valid)
);

    initial begin
        tb_clk = 1'b0;
        forever #(CLK_PERIOD/2) tb_clk = ~tb_clk;
    end

    initial begin
        $display("[%0t ns] SIM_INFO: snn_top_tb simulation START!!!", $time);

        tb_rst_n = 1'b0;
        tb_calc_start =1'b0;
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
        $display("[%0t ns] SIM_INFO: enable ASSERTED,processing START!!!.", $time);
        repeat(2) @(posedge tb_clk);

        tb_calc_start = 1'b0;

        @(posedge tb_all_spikes_valid);
        $display("[%0t ns] SIM_INFO: simulation DONE!!!.", $time);
        #50;
        $finish;

    end

endmodule