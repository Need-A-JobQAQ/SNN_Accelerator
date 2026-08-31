module conv_layer_parallel #(
    parameter P_INPUT_HEIGHT = 28,
    parameter P_INPUT_WIDTH = 28,
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_NUM_OUTPUT_CHANNELS = 2,
    parameter P_KERNEL_SIZE = 3,
    parameter P_PADDING = 1,
    parameter P_WEIGHT_BIT_WIDTH = 16,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter P_ENABLE_COMPAT_READBACK = 1,
    parameter [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV0_WEIGHTS_PACKED =
        {P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}},
    parameter [P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH - 1:0] P_CONV1_WEIGHTS_PACKED =
        {P_KERNEL_SIZE * P_KERNEL_SIZE * P_WEIGHT_BIT_WIDTH{1'b0}}
) (
    input wire clk,
    input wire rst_n,
    input wire i_calc_start,
    input wire [P_NUM_INPUT_PIXELS-1:0] i_input_spike_vector,

    input wire i_current_rd_en,
    input wire [$clog2(P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS)-1:0] i_current_rd_addr,
    output reg signed [P_NEURON_VALUE_TOTAL_BITS-1:0] o_current_rd_data,
    output reg o_current_rd_valid,
    input wire i_current_ch0_rd_en,
    input wire [$clog2(P_NUM_INPUT_PIXELS)-1:0] i_current_ch0_rd_addr,
    output wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] o_current_ch0_rd_data,
    output reg o_current_ch0_rd_valid,
    input wire i_current_ch1_rd_en,
    input wire [$clog2(P_NUM_INPUT_PIXELS)-1:0] i_current_ch1_rd_addr,
    output wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] o_current_ch1_rd_data,
    output reg o_current_ch1_rd_valid,
    output reg o_current_ram_ready,
    output reg [P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS - 1:0] o_current_valid_bitmap,

    output reg signed [P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS - 1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] o_all_currents_I,
    output reg o_all_currents_valid
);

    /*
     * 并行卷积层。
     * 两个 conv_pe 分别计算两个输出通道，并把每拍结果写入各自的 current RAM。
     * sparse 后级可以通过外部读口直接读取 current RAM；旧接口仍可通过兼容读回保留。
     */
    localparam LP_ADDR_WIDTH = $clog2(P_NUM_INPUT_PIXELS);
    localparam LP_TOTAL_FEATURES = P_NUM_OUTPUT_CHANNELS * P_NUM_INPUT_PIXELS;
    localparam LP_TOTAL_ADDR_WIDTH = $clog2(LP_TOTAL_FEATURES);
    localparam LP_COUNT_WIDTH = $clog2(P_NUM_INPUT_PIXELS + 1);

    localparam [2:0] S_IDLE = 3'b000;
    localparam [2:0] S_WAIT_PE = 3'b001;
    localparam [2:0] S_READ_BACK = 3'b010;
    localparam [2:0] S_FLUSH = 3'b011;
    localparam [2:0] S_DONE = 3'b100;

    reg [2:0] current_state_reg;
    reg [2:0] next_state_reg;

    wire ch0_current_valid_w;
    wire [LP_ADDR_WIDTH-1:0] ch0_current_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ch0_current_data_w;
    wire ch0_current_active_w;
    wire ch0_done_w;

    wire ch1_current_valid_w;
    wire [LP_ADDR_WIDTH-1:0] ch1_current_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ch1_current_data_w;
    wire ch1_current_active_w;
    wire ch1_done_w;

    wire compat_read_en_w;
    wire external_read_ch1_w;
    wire [LP_ADDR_WIDTH-1:0] external_read_local_addr_w;
    wire ch0_ram_read_en_w;
    wire ch1_ram_read_en_w;
    wire [LP_ADDR_WIDTH-1:0] ch0_ram_read_addr_w;
    wire [LP_ADDR_WIDTH-1:0] ch1_ram_read_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ch0_ram_read_data_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] ch1_ram_read_data_w;

    reg [LP_COUNT_WIDTH-1:0] read_issue_count_reg;
    reg compat_read_valid_reg;
    reg [LP_ADDR_WIDTH-1:0] compat_read_addr_dly_reg;
    reg external_read_valid_reg;
    reg external_read_ch1_dly_reg;

    assign compat_read_en_w = (current_state_reg == S_READ_BACK) &&
                              (read_issue_count_reg < P_NUM_INPUT_PIXELS);
    assign external_read_ch1_w = (i_current_rd_addr >= P_NUM_INPUT_PIXELS[LP_TOTAL_ADDR_WIDTH-1:0]);
    assign external_read_local_addr_w = external_read_ch1_w ?
        (i_current_rd_addr - P_NUM_INPUT_PIXELS[LP_TOTAL_ADDR_WIDTH-1:0]) :
        i_current_rd_addr[LP_ADDR_WIDTH-1:0];

    assign ch0_ram_read_en_w = compat_read_en_w ||
                               i_current_ch0_rd_en ||
                               (i_current_rd_en && !external_read_ch1_w);
    assign ch1_ram_read_en_w = compat_read_en_w ||
                               i_current_ch1_rd_en ||
                               (i_current_rd_en && external_read_ch1_w);
    assign ch0_ram_read_addr_w = compat_read_en_w ?
        read_issue_count_reg[LP_ADDR_WIDTH-1:0] :
        (i_current_ch0_rd_en ? i_current_ch0_rd_addr : external_read_local_addr_w);
    assign ch1_ram_read_addr_w = compat_read_en_w ?
        read_issue_count_reg[LP_ADDR_WIDTH-1:0] :
        (i_current_ch1_rd_en ? i_current_ch1_rd_addr : external_read_local_addr_w);
    assign o_current_ch0_rd_data = ch0_ram_read_data_w;
    assign o_current_ch1_rd_data = ch1_ram_read_data_w;

    conv_pe #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_KERNEL_SIZE              (P_KERNEL_SIZE),
        .P_PADDING                  (P_PADDING),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_KERNEL_WEIGHTS_PACKED    (P_CONV0_WEIGHTS_PACKED)
    ) u_conv_pe_ch0 (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_start                (i_calc_start),
        .i_input_spike_vector   (i_input_spike_vector),
        .o_current_valid        (ch0_current_valid_w),
        .o_current_addr         (ch0_current_addr_w),
        .o_current_data         (ch0_current_data_w),
        .o_current_active       (ch0_current_active_w),
        .o_done                 (ch0_done_w)
    );

    conv_pe #(
        .P_INPUT_HEIGHT             (P_INPUT_HEIGHT),
        .P_INPUT_WIDTH              (P_INPUT_WIDTH),
        .P_NUM_INPUT_PIXELS         (P_NUM_INPUT_PIXELS),
        .P_KERNEL_SIZE              (P_KERNEL_SIZE),
        .P_PADDING                  (P_PADDING),
        .P_WEIGHT_BIT_WIDTH         (P_WEIGHT_BIT_WIDTH),
        .P_NEURON_VALUE_TOTAL_BITS  (P_NEURON_VALUE_TOTAL_BITS),
        .P_KERNEL_WEIGHTS_PACKED    (P_CONV1_WEIGHTS_PACKED)
    ) u_conv_pe_ch1 (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .i_start                (i_calc_start),
        .i_input_spike_vector   (i_input_spike_vector),
        .o_current_valid        (ch1_current_valid_w),
        .o_current_addr         (ch1_current_addr_w),
        .o_current_data         (ch1_current_data_w),
        .o_current_active       (ch1_current_active_w),
        .o_done                 (ch1_done_w)
    );

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_ADDR_WIDTH),
        .P_DEPTH        (P_NUM_INPUT_PIXELS)
    ) u_ch0_current_ram (
        .clk            (clk),
        .i_write_en     (ch0_current_valid_w),
        .i_write_addr   (ch0_current_addr_w),
        .i_write_data   (ch0_current_data_w),
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
        .i_write_en     (ch1_current_valid_w),
        .i_write_addr   (ch1_current_addr_w),
        .i_write_data   (ch1_current_data_w),
        .i_read_en      (ch1_ram_read_en_w),
        .i_read_addr    (ch1_ram_read_addr_w),
        .o_read_data    (ch1_ram_read_data_w)
    );

    /*
     * 两段式状态机的组合转移逻辑。
     */
    always @(*) begin
        next_state_reg = current_state_reg;

        case (current_state_reg)
            S_IDLE: begin
                if (i_calc_start) begin
                    next_state_reg = S_WAIT_PE;
                end
            end

            S_WAIT_PE: begin
                if (ch0_done_w && ch1_done_w) begin
                    if (P_ENABLE_COMPAT_READBACK) begin                   //compat是compatability的简写，意味兼容。readback是为了维护o_all_currents_I端口，为了兼容没有更新的模块
                        next_state_reg = S_READ_BACK;
                    end else begin
                        next_state_reg = S_DONE;
                    end
                end
            end

            S_READ_BACK: begin
                if (read_issue_count_reg == P_NUM_INPUT_PIXELS) begin
                    next_state_reg = S_FLUSH;
                end
            end

            S_FLUSH: begin
                if (!compat_read_valid_reg) begin
                    next_state_reg = S_DONE;
                end
            end

            S_DONE: begin
                next_state_reg = S_IDLE;
            end

            default: begin
                next_state_reg = S_IDLE;
            end
        endcase
    end

    /*
     * 状态寄存器。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state_reg <= S_IDLE;
        end else begin
            current_state_reg <= next_state_reg;
        end
    end

    /*
     * current RAM 写完即可给后级读取。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_current_ram_ready <= 1'b0;
        end else begin
            o_current_ram_ready <= 1'b0;
            if (current_state_reg == S_WAIT_PE && ch0_done_w && ch1_done_w) begin
                o_current_ram_ready <= 1'b1;
            end
        end
    end

    /*
     * current 有效位图。
     * bit=1 表示该神经元地址对应的 3x3 感受野在当前时间步存在输入脉冲。
     * 低 784 位对应通道 0，高 784 位对应通道 1。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_current_valid_bitmap <= {LP_TOTAL_FEATURES{1'b0}};
        end else begin
            if (current_state_reg == S_IDLE && next_state_reg == S_WAIT_PE) begin
                o_current_valid_bitmap <= {LP_TOTAL_FEATURES{1'b0}};
            end else begin
                if (ch0_current_valid_w) begin
                    o_current_valid_bitmap[ch0_current_addr_w] <= ch0_current_active_w;
                end

                if (ch1_current_valid_w) begin
                    o_current_valid_bitmap[P_NUM_INPUT_PIXELS + ch1_current_addr_w] <= ch1_current_active_w;
                end
            end
        end
    end

    /*
     * 兼容旧接口的 RAM 读地址发起计数。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_issue_count_reg <= {LP_COUNT_WIDTH{1'b0}};
        end else if (current_state_reg == S_IDLE && next_state_reg == S_WAIT_PE) begin
            read_issue_count_reg <= {LP_COUNT_WIDTH{1'b0}};
        end else if (compat_read_en_w) begin
            read_issue_count_reg <= read_issue_count_reg + 1'b1;
        end
    end

    /*
     * 兼容读回的有效信号和地址延迟，用来对齐同步 RAM 的读数据。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            compat_read_valid_reg <= 1'b0;
            compat_read_addr_dly_reg <= {LP_ADDR_WIDTH{1'b0}};
        end else begin
            compat_read_valid_reg <= compat_read_en_w;
            if (compat_read_en_w) begin
                compat_read_addr_dly_reg <= read_issue_count_reg[LP_ADDR_WIDTH-1:0];
            end
        end
    end

    /*
     * 外部 current RAM 读有效和通道选择延迟一拍。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            external_read_valid_reg <= 1'b0;
            external_read_ch1_dly_reg <= 1'b0;
        end else begin
            external_read_valid_reg <= i_current_rd_en;
            if (i_current_rd_en) begin
                external_read_ch1_dly_reg <= external_read_ch1_w;
            end
        end
    end

    /*
     * 双 bank 外部读有效信号。
     * ch0/ch1 current RAM 是同步读，读使能后一拍数据有效。
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
     * 外部 current RAM 读数据输出。
     * RAM 本身已经同步读晚一拍，这里不再额外打一拍，避免和膜电位 RAM 数据错位。
     */
    always @(*) begin
        o_current_rd_valid = external_read_valid_reg;
        if (external_read_valid_reg) begin
            if (external_read_ch1_dly_reg) begin
                o_current_rd_data = ch1_ram_read_data_w;
            end else begin
                o_current_rd_data = ch0_ram_read_data_w;
            end
        end else begin
            o_current_rd_data = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
        end
    end

    /*
     * 兼容旧接口的读回写入。
     * 低 784 个地址保存通道0，高 784 个地址保存通道1。
     */
    always @(posedge clk) begin
        if (compat_read_valid_reg) begin
            o_all_currents_I[compat_read_addr_dly_reg] <= ch0_ram_read_data_w;
            o_all_currents_I[P_NUM_INPUT_PIXELS + compat_read_addr_dly_reg] <= ch1_ram_read_data_w;
        end
    end

    /*
     * 旧接口输出有效信号。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_all_currents_valid <= 1'b0;
        end else begin
            o_all_currents_valid <= (current_state_reg == S_DONE);
        end
    end

endmodule
