module masked_aer_linear_layer #(
    parameter P_NUM_INPUT_EVENTS = 1568,
    parameter P_EVENT_ADDR_WIDTH = $clog2(P_NUM_INPUT_EVENTS),
    parameter P_WEIGHT_BIT_WIDTH = 16,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter P_NEURON_VALUE_FRAC_BITS = 12,
    parameter P_BRAM_DATA_WIDTH = 64,
    parameter P_BRAM_ADDR_WIDTH = $clog2(P_NUM_INPUT_EVENTS / (P_BRAM_DATA_WIDTH / P_WEIGHT_BIT_WIDTH)),
    parameter P_BRAM_READ_LATENCY = 2,
    parameter P_NUM_OUTPUT_NEURONS = 10,
    parameter P_MASK_WIDTH = 16
) (
    input wire clk,
    input wire rst_n,
    input wire i_start,

    input wire i_event_valid,
    input wire [P_EVENT_ADDR_WIDTH-1:0] i_event_addr,
    input wire i_event_frame_done,
    output reg o_event_ready,

    output reg signed [P_NUM_OUTPUT_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] o_all_currents_I,
    output reg o_all_currents_valid
);

    /*
     * 带 valid_mask 的 AER 全连接层。
     *
     * AER 地址仍然表示输入脉冲在 1568 维展平向量中的位置。
     * 权重读取方式和 aer_linear_layer 保持一致：10 个输出类别的权重 BRAM 并行读取。
     * 新增 mask ROM：每个 event_addr 对应 16bit mask，低 10bit 表示 class0~class9 是否保留。
     * 当 mask 对应 bit 为 0 时，该类别跳过累加，相当于静态剪枝后的硬件门控。
     */
    localparam LP_WEIGHTS_PER_WORD = P_BRAM_DATA_WIDTH / P_WEIGHT_BIT_WIDTH;
    localparam LP_OFFSET_WIDTH = $clog2(LP_WEIGHTS_PER_WORD);
    localparam LP_PIPE_DEPTH = P_BRAM_READ_LATENCY;

    localparam [2:0] S_IDLE = 3'b000;
    localparam [2:0] S_PROCESS = 3'b001;
    localparam [2:0] S_DRAIN = 3'b010;
    localparam [2:0] S_DONE = 3'b011;

    reg [2:0] current_state_reg;
    reg [2:0] next_state_reg;

    reg [P_BRAM_ADDR_WIDTH-1:0] bram_addr_reg;
    reg [P_EVENT_ADDR_WIDTH-1:0] mask_addr_reg;
    reg [LP_OFFSET_WIDTH-1:0] event_offset_pipeline_reg [LP_PIPE_DEPTH-1:0];
    reg valid_pipeline_reg [LP_PIPE_DEPTH-1:0];
    reg pipeline_busy_comb;

    wire event_accept_w;
    wire [P_BRAM_ADDR_WIDTH-1:0] bram_addr_w;
    wire [P_EVENT_ADDR_WIDTH-1:0] mask_addr_w;
    wire bram_ena_w;
    wire mask_ena_w;
    wire pipeline_busy_w;
    wire accum_valid_w;

    wire signed [P_BRAM_DATA_WIDTH-1:0] bram_dout_wires [P_NUM_OUTPUT_NEURONS-1:0];
    wire signed [P_WEIGHT_BIT_WIDTH-1:0] selected_weight_wires [P_NUM_OUTPUT_NEURONS-1:0];
    wire [P_MASK_WIDTH-1:0] mask_dout_w;
    wire [P_NUM_OUTPUT_NEURONS-1:0] selected_mask_w;

    integer reset_idx;
    integer accum_idx;
    integer pipe_idx;
    integer busy_idx;
    genvar n_idx;
    genvar w_idx;

    assign event_accept_w = (current_state_reg == S_PROCESS) && i_event_valid && o_event_ready;
    assign bram_addr_w = event_accept_w ? i_event_addr[P_EVENT_ADDR_WIDTH-1:LP_OFFSET_WIDTH] : bram_addr_reg;
    assign mask_addr_w = event_accept_w ? i_event_addr : mask_addr_reg;
    assign bram_ena_w = event_accept_w || pipeline_busy_comb;
    assign mask_ena_w = bram_ena_w;
    assign accum_valid_w = valid_pipeline_reg[LP_PIPE_DEPTH-1];
    assign pipeline_busy_w = pipeline_busy_comb;
    assign selected_mask_w = mask_dout_w[P_NUM_OUTPUT_NEURONS-1:0];

    /*
     * 判断权重和 mask 读取流水线中是否还有未完成事件。
     */
    always @(*) begin
        pipeline_busy_comb = 1'b0;
        for (busy_idx = 0; busy_idx < LP_PIPE_DEPTH; busy_idx = busy_idx + 1) begin
            pipeline_busy_comb = pipeline_busy_comb || valid_pipeline_reg[busy_idx];
        end
    end

    /*
     * 一个 64bit 权重字包含 4 个 16bit 权重。
     * offset=0 对应最高 16bit，offset=3 对应最低 16bit。
     */
    function signed [P_WEIGHT_BIT_WIDTH-1:0] select_weight_from_word;
        input signed [P_BRAM_DATA_WIDTH-1:0] bram_word;
        input [LP_OFFSET_WIDTH-1:0] weight_offset;
        begin
            case (weight_offset)
                2'd0: select_weight_from_word = bram_word[63:48];
                2'd1: select_weight_from_word = bram_word[47:32];
                2'd2: select_weight_from_word = bram_word[31:16];
                2'd3: select_weight_from_word = bram_word[15:0];
                default: select_weight_from_word = {P_WEIGHT_BIT_WIDTH{1'b0}};
            endcase
        end
    endfunction

    /*
     * 10 个原全连接权重 BRAM 并行读取。
     * mask 只负责决定是否累加，不改变权重地址和 AER 地址含义。
     */
    generate
        for (n_idx = 0; n_idx < P_NUM_OUTPUT_NEURONS; n_idx = n_idx + 1) begin : gen_weight_brams
            if (n_idx == 0) begin : gen_bram_0
                weights_0 u_bram_neuron_0 (
                    .clka   (clk),
                    .ena    (bram_ena_w),
                    .addra  (bram_addr_w),
                    .douta  (bram_dout_wires[n_idx])
                );
            end else if (n_idx == 1) begin : gen_bram_1
                weights_1 u_bram_neuron_1 (
                    .clka   (clk),
                    .ena    (bram_ena_w),
                    .addra  (bram_addr_w),
                    .douta  (bram_dout_wires[n_idx])
                );
            end else if (n_idx == 2) begin : gen_bram_2
                weights_2 u_bram_neuron_2 (
                    .clka   (clk),
                    .ena    (bram_ena_w),
                    .addra  (bram_addr_w),
                    .douta  (bram_dout_wires[n_idx])
                );
            end else if (n_idx == 3) begin : gen_bram_3
                weights_3 u_bram_neuron_3 (
                    .clka   (clk),
                    .ena    (bram_ena_w),
                    .addra  (bram_addr_w),
                    .douta  (bram_dout_wires[n_idx])
                );
            end else if (n_idx == 4) begin : gen_bram_4
                weights_4 u_bram_neuron_4 (
                    .clka   (clk),
                    .ena    (bram_ena_w),
                    .addra  (bram_addr_w),
                    .douta  (bram_dout_wires[n_idx])
                );
            end else if (n_idx == 5) begin : gen_bram_5
                weights_5 u_bram_neuron_5 (
                    .clka   (clk),
                    .ena    (bram_ena_w),
                    .addra  (bram_addr_w),
                    .douta  (bram_dout_wires[n_idx])
                );
            end else if (n_idx == 6) begin : gen_bram_6
                weights_6 u_bram_neuron_6 (
                    .clka   (clk),
                    .ena    (bram_ena_w),
                    .addra  (bram_addr_w),
                    .douta  (bram_dout_wires[n_idx])
                );
            end else if (n_idx == 7) begin : gen_bram_7
                weights_7 u_bram_neuron_7 (
                    .clka   (clk),
                    .ena    (bram_ena_w),
                    .addra  (bram_addr_w),
                    .douta  (bram_dout_wires[n_idx])
                );
            end else if (n_idx == 8) begin : gen_bram_8
                weights_8 u_bram_neuron_8 (
                    .clka   (clk),
                    .ena    (bram_ena_w),
                    .addra  (bram_addr_w),
                    .douta  (bram_dout_wires[n_idx])
                );
            end else if (n_idx == 9) begin : gen_bram_9
                weights_9 u_bram_neuron_9 (
                    .clka   (clk),
                    .ena    (bram_ena_w),
                    .addra  (bram_addr_w),
                    .douta  (bram_dout_wires[n_idx])
                );
            end
        end
    endgenerate

    /*
     * mask ROM。
     * Vivado 中建议创建同名 BRAM IP：fc_mask_0p1。
     * 数据宽度 16bit，深度 2048，初始化文件使用 fc_mask_threshold_0p1_2048.coe。
     */
    fc_mask_0p1 u_fc_mask_0p1 (
        .clka   (clk),
        .ena    (mask_ena_w),
        .addra  (mask_addr_w),
        .douta  (mask_dout_w)
    );

    /*
     * 从每个权重 BRAM 输出字中选出当前 event_addr 对应的 16bit 权重。
     */
    generate
        for (w_idx = 0; w_idx < P_NUM_OUTPUT_NEURONS; w_idx = w_idx + 1) begin : gen_select_weight
            assign selected_weight_wires[w_idx] =
                select_weight_from_word(bram_dout_wires[w_idx], event_offset_pipeline_reg[LP_PIPE_DEPTH-1]);
        end
    endgenerate

    /*
     * 状态转移和 ready 输出。
     */
    always @(*) begin
        next_state_reg = current_state_reg;
        o_event_ready = 1'b0;

        case (current_state_reg)
            S_IDLE: begin
                if (i_start) begin
                    next_state_reg = S_PROCESS;
                end
            end

            S_PROCESS: begin
                o_event_ready = 1'b1;
                if (i_event_frame_done) begin
                    next_state_reg = S_DRAIN;
                end
            end

            S_DRAIN: begin
                if (pipeline_busy_w) begin
                    next_state_reg = S_DRAIN;
                end else begin
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
     * BRAM 地址、mask 地址、offset 和 valid 流水线。
     * mask ROM 和权重 BRAM 使用同一个 ena 节奏，默认二者读延迟一致。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bram_addr_reg <= {P_BRAM_ADDR_WIDTH{1'b0}};
            mask_addr_reg <= {P_EVENT_ADDR_WIDTH{1'b0}};
            for (pipe_idx = 0; pipe_idx < LP_PIPE_DEPTH; pipe_idx = pipe_idx + 1) begin
                event_offset_pipeline_reg[pipe_idx] <= {LP_OFFSET_WIDTH{1'b0}};
                valid_pipeline_reg[pipe_idx] <= 1'b0;
            end
        end else begin
            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESS) begin
                bram_addr_reg <= {P_BRAM_ADDR_WIDTH{1'b0}};
                mask_addr_reg <= {P_EVENT_ADDR_WIDTH{1'b0}};
                for (pipe_idx = 0; pipe_idx < LP_PIPE_DEPTH; pipe_idx = pipe_idx + 1) begin
                    event_offset_pipeline_reg[pipe_idx] <= {LP_OFFSET_WIDTH{1'b0}};
                    valid_pipeline_reg[pipe_idx] <= 1'b0;
                end
            end else if (current_state_reg == S_PROCESS || current_state_reg == S_DRAIN) begin
                for (pipe_idx = LP_PIPE_DEPTH - 1; pipe_idx > 0; pipe_idx = pipe_idx - 1) begin
                    event_offset_pipeline_reg[pipe_idx] <= event_offset_pipeline_reg[pipe_idx-1];
                    valid_pipeline_reg[pipe_idx] <= valid_pipeline_reg[pipe_idx-1];
                end

                if (event_accept_w) begin
                    bram_addr_reg <= i_event_addr[P_EVENT_ADDR_WIDTH-1:LP_OFFSET_WIDTH];
                    mask_addr_reg <= i_event_addr;
                end

                event_offset_pipeline_reg[0] <= i_event_addr[LP_OFFSET_WIDTH-1:0];
                valid_pipeline_reg[0] <= event_accept_w;
            end
        end
    end

    /*
     * 输出电流累加。
     * mask bit 为 1 时才累加该类别；mask bit 为 0 时跳过该类别。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_all_currents_valid <= 1'b0;
            for (reset_idx = 0; reset_idx < P_NUM_OUTPUT_NEURONS; reset_idx = reset_idx + 1) begin
                o_all_currents_I[reset_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            end
        end else begin
            o_all_currents_valid <= 1'b0;

            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESS) begin
                for (reset_idx = 0; reset_idx < P_NUM_OUTPUT_NEURONS; reset_idx = reset_idx + 1) begin
                    o_all_currents_I[reset_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                end
            end else if (accum_valid_w) begin
                for (accum_idx = 0; accum_idx < P_NUM_OUTPUT_NEURONS; accum_idx = accum_idx + 1) begin
                    if (selected_mask_w[accum_idx]) begin
                        o_all_currents_I[accum_idx] <= o_all_currents_I[accum_idx] +
                            {{(P_NEURON_VALUE_TOTAL_BITS - P_WEIGHT_BIT_WIDTH){selected_weight_wires[accum_idx][P_WEIGHT_BIT_WIDTH-1]}},
                             selected_weight_wires[accum_idx]};
                    end
                end
            end

            if (current_state_reg == S_DONE) begin
                o_all_currents_valid <= 1'b1;
            end
        end
    end

endmodule
