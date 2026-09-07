module conv_current_buffer #(
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_NUM_OUTPUT_CHANNELS = 2,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26
) (
    // 基础控制信号
    input wire                                                          clk,
    input wire                                                          rst_n,
    input wire                                                          i_clear,

    // 卷积电流写流输入
    input wire                                                          i_current_ch0_wr_en,
    input wire [$clog2(P_NUM_INPUT_PIXELS)-1:0]                         i_current_ch0_wr_addr,
    input wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]                   i_current_ch0_wr_data,
    input wire                                                          i_current_ch0_wr_active,
    input wire                                                          i_current_ch1_wr_en,
    input wire [$clog2(P_NUM_INPUT_PIXELS)-1:0]                         i_current_ch1_wr_addr,
    input wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]                   i_current_ch1_wr_data,
    input wire                                                          i_current_ch1_wr_active,
    input wire                                                          i_current_wr_done,

    // current RAM 双通道读口，供 sparse/multicore 卷积后 LIF 使用
    input wire                                                          i_current_ch0_rd_en,
    input wire [$clog2(P_NUM_INPUT_PIXELS)-1:0]                         i_current_ch0_rd_addr,
    output wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]                  o_current_ch0_rd_data,
    output reg                                                          o_current_ch0_rd_valid,
    input wire                                                          i_current_ch1_rd_en,
    input wire [$clog2(P_NUM_INPUT_PIXELS)-1:0]                         i_current_ch1_rd_addr,
    output wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]                  o_current_ch1_rd_data,
    output reg                                                          o_current_ch1_rd_valid,

    // 兼容旧版单读口，地址 0~783 读通道0，地址 784~1567 读通道1
    input wire                                                          i_current_rd_en,
    input wire [$clog2(P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS)-1:0] i_current_rd_addr,
    output reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0]                   o_current_rd_data,
    output reg                                                          o_current_rd_valid,

    // current 缓存状态与有效位图
    output reg                                                          o_current_buffer_ready,
    output reg [P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS - 1:0]        o_current_valid_bitmap
);

    /*
     * 单缓存版卷积电流缓存。
     * 该模块把卷积 PE 的写流保存到 ch0/ch1 两个 current RAM 中，
     * 同时维护 current_valid_bitmap，作为后续双缓存 ping-pong 边界的基础版本。
     */
    localparam LP_ADDR_WIDTH = $clog2(P_NUM_INPUT_PIXELS);
    localparam LP_TOTAL_FEATURES = P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS;
    localparam LP_TOTAL_ADDR_WIDTH = $clog2(LP_TOTAL_FEATURES);

    wire compat_read_ch1_w;
    wire [LP_ADDR_WIDTH-1:0] compat_read_local_addr_w;
    wire ch0_ram_read_en_w;
    wire ch1_ram_read_en_w;
    wire [LP_ADDR_WIDTH-1:0] ch0_ram_read_addr_w;
    wire [LP_ADDR_WIDTH-1:0] ch1_ram_read_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ch0_ram_read_data_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ch1_ram_read_data_w;

    reg compat_read_valid_reg;
    reg compat_read_ch1_dly_reg;

    assign compat_read_ch1_w = (i_current_rd_addr >= P_NUM_INPUT_PIXELS[LP_TOTAL_ADDR_WIDTH-1:0]);
    assign compat_read_local_addr_w = compat_read_ch1_w ?
        (i_current_rd_addr - P_NUM_INPUT_PIXELS[LP_TOTAL_ADDR_WIDTH-1:0]) :
        i_current_rd_addr[LP_ADDR_WIDTH-1:0];

    assign ch0_ram_read_en_w = i_current_ch0_rd_en ||
                               (i_current_rd_en && !compat_read_ch1_w);
    assign ch1_ram_read_en_w = i_current_ch1_rd_en ||
                               (i_current_rd_en && compat_read_ch1_w);
    assign ch0_ram_read_addr_w = i_current_ch0_rd_en ?
        i_current_ch0_rd_addr : compat_read_local_addr_w;
    assign ch1_ram_read_addr_w = i_current_ch1_rd_en ?
        i_current_ch1_rd_addr : compat_read_local_addr_w;
    assign o_current_ch0_rd_data = ch0_ram_read_data_w;
    assign o_current_ch1_rd_data = ch1_ram_read_data_w;

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_ADDR_WIDTH),
        .P_DEPTH        (P_NUM_INPUT_PIXELS)
    ) u_ch0_current_ram (
        .clk            (clk),
        .i_write_en     (i_current_ch0_wr_en),
        .i_write_addr   (i_current_ch0_wr_addr),
        .i_write_data   (i_current_ch0_wr_data),
        .i_read_en      (ch0_ram_read_en_w),
        .i_read_addr    (ch0_ram_read_addr_w),
        .o_read_data    (ch0_ram_read_data_w)
    );

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_ADDR_WIDTH),
        .P_DEPTH        (P_NUM_INPUT_PIXELS)
    ) u_ch1_current_ram (
        .clk            (clk),
        .i_write_en     (i_current_ch1_wr_en),
        .i_write_addr   (i_current_ch1_wr_addr),
        .i_write_data   (i_current_ch1_wr_data),
        .i_read_en      (ch1_ram_read_en_w),
        .i_read_addr    (ch1_ram_read_addr_w),
        .o_read_data    (ch1_ram_read_data_w)
    );

    /*
     * current 有效位图。
     * bit=1 表示该神经元地址对应的卷积感受野在当前时间步存在输入脉冲。
     * 低 784 位对应通道 0，高 784 位对应通道 1。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_current_valid_bitmap <= {LP_TOTAL_FEATURES{1'b0}};
        end else if (i_clear) begin
            o_current_valid_bitmap <= {LP_TOTAL_FEATURES{1'b0}};
        end else begin
            if (i_current_ch0_wr_en) begin
                o_current_valid_bitmap[i_current_ch0_wr_addr] <= i_current_ch0_wr_active;
            end

            if (i_current_ch1_wr_en) begin
                o_current_valid_bitmap[P_NUM_INPUT_PIXELS + i_current_ch1_wr_addr] <= i_current_ch1_wr_active;
            end
        end
    end

    /*
     * current 缓存完成脉冲。
     * 当前单缓存版本直接把卷积写流完成信号打一拍输出给后级控制。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_current_buffer_ready <= 1'b0;
        end else begin
            o_current_buffer_ready <= i_current_wr_done;
        end
    end

    /*
     * 双通道读有效信号。
     * simple_dual_port_ram 是同步读，读使能后一拍数据有效。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_current_ch0_rd_valid <= 1'b0;
            o_current_ch1_rd_valid <= 1'b0;
        end else begin
            o_current_ch0_rd_valid <= i_current_ch0_rd_en;
            o_current_ch1_rd_valid <= i_current_ch1_rd_en;
        end
    end

    /*
     * 兼容单读口的通道选择延迟。
     * 通道选择必须和同步 RAM 读数据保持同拍对齐。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            compat_read_valid_reg <= 1'b0;
            compat_read_ch1_dly_reg <= 1'b0;
        end else begin
            compat_read_valid_reg <= i_current_rd_en;
            if (i_current_rd_en) begin
                compat_read_ch1_dly_reg <= compat_read_ch1_w;
            end
        end
    end

    /*
     * 兼容单读口输出数据选择。
     */
    always @(*) begin
        o_current_rd_valid = compat_read_valid_reg;
        if (compat_read_valid_reg) begin
            if (compat_read_ch1_dly_reg) begin
                o_current_rd_data = ch1_ram_read_data_w;
            end else begin
                o_current_rd_data = ch0_ram_read_data_w;
            end
        end else begin
            o_current_rd_data = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
        end
    end

endmodule
