module conv_current_pingpong_buffer #(
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_NUM_OUTPUT_CHANNELS = 2,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26
) (
    // 基础控制信号
    input wire                                                          clk,
    input wire                                                          rst_n,
    input wire                                                          i_flush,
    input wire                                                          i_clear,                // 兼容旧接口：等价于 i_write_start

    // 卷积电流写流输入
    input wire                                                          i_write_start,
    input wire [P_NUM_INPUT_PIXELS-1:0]                                 i_write_spike_vector,
    input wire                                                          i_current_ch0_wr_en,
    input wire [$clog2(P_NUM_INPUT_PIXELS)-1:0]                         i_current_ch0_wr_addr,
    input wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]                   i_current_ch0_wr_data,
    input wire                                                          i_current_ch0_wr_active,
    input wire                                                          i_current_ch1_wr_en,
    input wire [$clog2(P_NUM_INPUT_PIXELS)-1:0]                         i_current_ch1_wr_addr,
    input wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0]                   i_current_ch1_wr_data,
    input wire                                                          i_current_ch1_wr_active,
    input wire                                                          i_current_wr_done,
    input wire                                                          i_read_start,
    input wire                                                          i_release_read_buffer,

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
    output wire                                                         o_write_ready,
    output wire                                                         o_read_valid,
    output wire [P_NUM_INPUT_PIXELS-1:0]                                 o_read_spike_vector,
    output wire                                                         o_write_buffer_sel,
    output wire                                                         o_read_buffer_sel,
    output wire [1:0]                                                   o_buffer_valid_bits,
    output wire [P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS - 1:0]       o_current_valid_bitmap
);

    /*
     * 双缓存版卷积电流缓存。
     * 写侧保存 conv(t) 产生的电流，读侧供 conv_lif(t) 使用。
     * 当前顶层仍按串行 timestep 调度，因此该模块先作为等价替换；
     * 后续控制器允许 conv(t+1) 提前启动后，读写两侧即可落在不同 buffer 上。
     */
    localparam LP_ADDR_WIDTH = $clog2(P_NUM_INPUT_PIXELS);
    localparam LP_TOTAL_FEATURES = P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS;
    localparam LP_TOTAL_ADDR_WIDTH = $clog2(LP_TOTAL_FEATURES);

    wire write_start_w;
    wire next_write_sel_w;
    wire compat_read_ch1_w;
    wire [LP_ADDR_WIDTH-1:0] compat_read_local_addr_w;

    wire buf0_ch0_wr_en_w;
    wire buf0_ch1_wr_en_w;
    wire buf1_ch0_wr_en_w;
    wire buf1_ch1_wr_en_w;

    wire buf0_ch0_rd_en_w;
    wire buf0_ch1_rd_en_w;
    wire buf1_ch0_rd_en_w;
    wire buf1_ch1_rd_en_w;

    wire [LP_ADDR_WIDTH-1:0] ch0_ram_read_addr_w;
    wire [LP_ADDR_WIDTH-1:0] ch1_ram_read_addr_w;

    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] buf0_ch0_read_data_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] buf0_ch1_read_data_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] buf1_ch0_read_data_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] buf1_ch1_read_data_w;

    reg write_sel_reg;
    reg read_sel_reg;
    reg [1:0] buffer_valid_reg;
    reg [LP_TOTAL_FEATURES-1:0] valid_bitmap_buf0_reg;
    reg [LP_TOTAL_FEATURES-1:0] valid_bitmap_buf1_reg;
    reg [P_NUM_INPUT_PIXELS-1:0] spike_vector_buf0_reg;
    reg [P_NUM_INPUT_PIXELS-1:0] spike_vector_buf1_reg;
    reg compat_read_valid_reg;
    reg compat_read_ch1_dly_reg;

    assign write_start_w = i_clear || i_write_start;
    assign next_write_sel_w = ~write_sel_reg;
    assign o_write_ready = !buffer_valid_reg[next_write_sel_w];
    assign o_read_valid = buffer_valid_reg[read_sel_reg];
    assign o_read_spike_vector = read_sel_reg ? spike_vector_buf1_reg : spike_vector_buf0_reg;
    assign o_write_buffer_sel = write_sel_reg;
    assign o_read_buffer_sel = read_sel_reg;
    assign o_buffer_valid_bits = buffer_valid_reg;
    assign o_current_valid_bitmap = read_sel_reg ? valid_bitmap_buf1_reg : valid_bitmap_buf0_reg;

    assign compat_read_ch1_w = (i_current_rd_addr >= P_NUM_INPUT_PIXELS[LP_TOTAL_ADDR_WIDTH-1:0]);
    assign compat_read_local_addr_w = compat_read_ch1_w ?
        (i_current_rd_addr - P_NUM_INPUT_PIXELS[LP_TOTAL_ADDR_WIDTH-1:0]) :
        i_current_rd_addr[LP_ADDR_WIDTH-1:0];

    assign ch0_ram_read_addr_w = i_current_ch0_rd_en ?
        i_current_ch0_rd_addr : compat_read_local_addr_w;
    assign ch1_ram_read_addr_w = i_current_ch1_rd_en ?
        i_current_ch1_rd_addr : compat_read_local_addr_w;

    assign buf0_ch0_wr_en_w = i_current_ch0_wr_en && !write_sel_reg;
    assign buf0_ch1_wr_en_w = i_current_ch1_wr_en && !write_sel_reg;
    assign buf1_ch0_wr_en_w = i_current_ch0_wr_en && write_sel_reg;
    assign buf1_ch1_wr_en_w = i_current_ch1_wr_en && write_sel_reg;

    assign buf0_ch0_rd_en_w = !read_sel_reg &&
                              (i_current_ch0_rd_en || (i_current_rd_en && !compat_read_ch1_w));
    assign buf0_ch1_rd_en_w = !read_sel_reg &&
                              (i_current_ch1_rd_en || (i_current_rd_en && compat_read_ch1_w));
    assign buf1_ch0_rd_en_w = read_sel_reg &&
                              (i_current_ch0_rd_en || (i_current_rd_en && !compat_read_ch1_w));
    assign buf1_ch1_rd_en_w = read_sel_reg &&
                              (i_current_ch1_rd_en || (i_current_rd_en && compat_read_ch1_w));

    assign o_current_ch0_rd_data = read_sel_reg ? buf1_ch0_read_data_w : buf0_ch0_read_data_w;
    assign o_current_ch1_rd_data = read_sel_reg ? buf1_ch1_read_data_w : buf0_ch1_read_data_w;

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_ADDR_WIDTH),
        .P_DEPTH        (P_NUM_INPUT_PIXELS)
    ) u_buf0_ch0_current_ram (
        .clk            (clk),
        .i_write_en     (buf0_ch0_wr_en_w),
        .i_write_addr   (i_current_ch0_wr_addr),
        .i_write_data   (i_current_ch0_wr_data),
        .i_read_en      (buf0_ch0_rd_en_w),
        .i_read_addr    (ch0_ram_read_addr_w),
        .o_read_data    (buf0_ch0_read_data_w)
    );

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_ADDR_WIDTH),
        .P_DEPTH        (P_NUM_INPUT_PIXELS)
    ) u_buf0_ch1_current_ram (
        .clk            (clk),
        .i_write_en     (buf0_ch1_wr_en_w),
        .i_write_addr   (i_current_ch1_wr_addr),
        .i_write_data   (i_current_ch1_wr_data),
        .i_read_en      (buf0_ch1_rd_en_w),
        .i_read_addr    (ch1_ram_read_addr_w),
        .o_read_data    (buf0_ch1_read_data_w)
    );

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_ADDR_WIDTH),
        .P_DEPTH        (P_NUM_INPUT_PIXELS)
    ) u_buf1_ch0_current_ram (
        .clk            (clk),
        .i_write_en     (buf1_ch0_wr_en_w),
        .i_write_addr   (i_current_ch0_wr_addr),
        .i_write_data   (i_current_ch0_wr_data),
        .i_read_en      (buf1_ch0_rd_en_w),
        .i_read_addr    (ch0_ram_read_addr_w),
        .o_read_data    (buf1_ch0_read_data_w)
    );

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_ADDR_WIDTH),
        .P_DEPTH        (P_NUM_INPUT_PIXELS)
    ) u_buf1_ch1_current_ram (
        .clk            (clk),
        .i_write_en     (buf1_ch1_wr_en_w),
        .i_write_addr   (i_current_ch1_wr_addr),
        .i_write_data   (i_current_ch1_wr_data),
        .i_read_en      (buf1_ch1_rd_en_w),
        .i_read_addr    (ch1_ram_read_addr_w),
        .o_read_data    (buf1_ch1_read_data_w)
    );

    /*
     * 写 buffer 选择。
     * 每次开始写入一个新时间步时切换写 buffer。
     * 顶层真正流水后，需要先检查 o_write_ready，避免覆盖尚未释放的 buffer。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_sel_reg <= 1'b0;
        end else if (i_flush) begin
            write_sel_reg <= 1'b0;
        end else if (write_start_w) begin
            write_sel_reg <= next_write_sel_w;
        end
    end

    /*
     * 读 buffer 选择。
     * 写完成只产生 full 标志，不主动切换读侧；
     * 后级真正启动时再锁定一个 full buffer，避免读旧时间步时被新时间步写完成打断。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_sel_reg <= 1'b0;
        end else if (i_flush) begin
            read_sel_reg <= 1'b0;
        end else if (i_read_start) begin
            if (buffer_valid_reg[0]) begin
                read_sel_reg <= 1'b0;
            end else if (buffer_valid_reg[1]) begin
                read_sel_reg <= 1'b1;
            end
        end else if (i_current_wr_done && !buffer_valid_reg[read_sel_reg]) begin
            // 兼容旧串行调度：若当前没有待读 buffer，写完后自动指向新结果。
            read_sel_reg <= write_sel_reg;
        end
    end

    /*
     * 两套 current_valid_bitmap。
     * 开始写某个 buffer 时只清对应 bitmap，不清 RAM 数据；
     * 后级是否使用 current 数据完全由 bitmap 决定，因此旧 RAM 数据不会被误用。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_bitmap_buf0_reg <= {LP_TOTAL_FEATURES{1'b0}};
            valid_bitmap_buf1_reg <= {LP_TOTAL_FEATURES{1'b0}};
        end else if (i_flush) begin
            valid_bitmap_buf0_reg <= {LP_TOTAL_FEATURES{1'b0}};
            valid_bitmap_buf1_reg <= {LP_TOTAL_FEATURES{1'b0}};
        end else begin
            if (write_start_w && !next_write_sel_w) begin
                valid_bitmap_buf0_reg <= {LP_TOTAL_FEATURES{1'b0}};
            end else begin
                if (buf0_ch0_wr_en_w) begin
                    valid_bitmap_buf0_reg[i_current_ch0_wr_addr] <= i_current_ch0_wr_active;
                end
                if (buf0_ch1_wr_en_w) begin
                    valid_bitmap_buf0_reg[P_NUM_INPUT_PIXELS + i_current_ch1_wr_addr] <= i_current_ch1_wr_active;
                end
            end

            if (write_start_w && next_write_sel_w) begin
                valid_bitmap_buf1_reg <= {LP_TOTAL_FEATURES{1'b0}};
            end else begin
                if (buf1_ch0_wr_en_w) begin
                    valid_bitmap_buf1_reg[i_current_ch0_wr_addr] <= i_current_ch0_wr_active;
                end
                if (buf1_ch1_wr_en_w) begin
                    valid_bitmap_buf1_reg[P_NUM_INPUT_PIXELS + i_current_ch1_wr_addr] <= i_current_ch1_wr_active;
                end
            end
        end
    end

    /*
     * 输入脉冲向量随 current buffer 一起保存。
     * 后级处理某个时间步时，应使用与 read buffer 对应的 spike vector，
     * 避免流水后被下一时间步的编码结果覆盖。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spike_vector_buf0_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
            spike_vector_buf1_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
        end else if (i_flush) begin
            spike_vector_buf0_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
            spike_vector_buf1_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
        end else if (write_start_w) begin
            if (next_write_sel_w) begin
                spike_vector_buf1_reg <= i_write_spike_vector;
            end else begin
                spike_vector_buf0_reg <= i_write_spike_vector;
            end
        end
    end

    /*
     * buffer 有效标志。
     * 写完成后置 full；后级处理完成后通过 i_release_read_buffer 释放。
     * 这个状态是后续跨时间步流水防止覆盖的核心依据。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buffer_valid_reg <= 2'b00;
        end else if (i_flush) begin
            buffer_valid_reg <= 2'b00;
        end else begin
            if (write_start_w) begin
                buffer_valid_reg[next_write_sel_w] <= 1'b0;
            end

            if (i_release_read_buffer) begin
                buffer_valid_reg[read_sel_reg] <= 1'b0;
            end

            if (i_current_wr_done) begin
                buffer_valid_reg[write_sel_reg] <= 1'b1;
            end
        end
    end

    /*
     * current 缓存完成脉冲。
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
                o_current_rd_data = read_sel_reg ? buf1_ch1_read_data_w : buf0_ch1_read_data_w;
            end else begin
                o_current_rd_data = read_sel_reg ? buf1_ch0_read_data_w : buf0_ch0_read_data_w;
            end
        end else begin
            o_current_rd_data = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
        end
    end

endmodule
