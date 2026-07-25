module aer_linear_layer #(
    parameter P_NUM_INPUT_EVENTS = 1568,
    parameter P_EVENT_ADDR_WIDTH = $clog2(P_NUM_INPUT_EVENTS),
    parameter P_WEIGHT_BIT_WIDTH = 16,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter P_NEURON_VALUE_FRAC_BITS = 12,
    parameter P_BRAM_DATA_WIDTH = 64,
    parameter P_BRAM_ADDR_WIDTH = $clog2(P_NUM_INPUT_EVENTS / (P_BRAM_DATA_WIDTH / P_WEIGHT_BIT_WIDTH)),
    parameter P_BRAM_READ_LATENCY = 2,
    parameter P_NUM_OUTPUT_NEURONS = 10
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
     * AER 版全连接层。
     * 每收到一个事件地址，只读取该输入位置对应的权重并累加；
     * 不再像原 linear_layer 那样扫描全部输入位置。
     */
    localparam LP_WEIGHTS_PER_WORD = P_BRAM_DATA_WIDTH / P_WEIGHT_BIT_WIDTH; // 64 ÷ 16 = 4
    localparam LP_OFFSET_WIDTH = $clog2(LP_WEIGHTS_PER_WORD);                //一个地址包含四个权重，offset取值最大为3，所以LP_OFFSET_WIDTH应能表示这个数字
    /*
     * 方案B：事件被接收时，当拍把地址送给 BRAM。
     * offset/valid 在同一个时钟沿后进入流水线，天然比 BRAM 地址请求晚一拍。
     * 所以这里不再使用方案A的请求寄存器，而是把控制流水线深度设为 BRAM 读延迟。
     */
    localparam LP_PIPE_DEPTH = P_BRAM_READ_LATENCY;

    localparam [2:0] S_IDLE = 3'b000;
    localparam [2:0] S_PROCESS = 3'b001;
    localparam [2:0] S_DRAIN = 3'b010;
    localparam [2:0] S_DONE = 3'b011;

    reg [2:0] current_state_reg;
    reg [2:0] next_state_reg;

    reg [P_BRAM_ADDR_WIDTH-1:0] bram_addr_reg;
    reg [LP_OFFSET_WIDTH-1:0] event_offset_pipeline_reg [LP_PIPE_DEPTH-1:0];
    reg valid_pipeline_reg [LP_PIPE_DEPTH-1:0];
    reg pipeline_busy_comb;

    wire event_accept_w;
    wire [P_BRAM_ADDR_WIDTH-1:0] bram_addr_w;
    wire bram_ena_w;
    wire pipeline_busy_w;
    wire accum_valid_w;

    wire signed [P_BRAM_DATA_WIDTH-1:0] bram_dout_wires [P_NUM_OUTPUT_NEURONS-1:0];
    wire signed [P_WEIGHT_BIT_WIDTH-1:0] selected_weight_wires [P_NUM_OUTPUT_NEURONS-1:0];

    integer reset_idx;
    integer accum_idx;
    integer pipe_idx;
    integer busy_idx;
    genvar n_idx;
    genvar w_idx;

    assign event_accept_w = (current_state_reg == S_PROCESS) && i_event_valid && o_event_ready;
    assign bram_addr_w = event_accept_w ? i_event_addr[P_EVENT_ADDR_WIDTH-1:LP_OFFSET_WIDTH] : bram_addr_reg;
    assign bram_ena_w = event_accept_w || pipeline_busy_comb;
    assign accum_valid_w = valid_pipeline_reg[LP_PIPE_DEPTH-1];
    assign pipeline_busy_w = pipeline_busy_comb;

    always @(*) begin
        pipeline_busy_comb = 1'b0;
        for (busy_idx = 0; busy_idx < LP_PIPE_DEPTH; busy_idx = busy_idx + 1) begin
            pipeline_busy_comb = pipeline_busy_comb || valid_pipeline_reg[busy_idx];
        end
    end

    /*
     * 一个 64bit 权重字中包含 4 个 16bit 权重。
     * 逻辑 offset=0 对应最高 16bit，offset=3 对应最低 16bit。
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
     * 复用原工程的 10 个全连接层权重 BRAM IP。
     * 每个输出类别对应一个独立权重 BRAM，所有 BRAM 同时读取同一个输入事件地址。
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
     * 先把每个 BRAM 输出字中的目标 16bit 权重取出来。
     * 这样累加逻辑里不用直接对函数返回值取符号位，工具兼容性更好。
     */
    generate
        for (w_idx = 0; w_idx < P_NUM_OUTPUT_NEURONS; w_idx = w_idx + 1) begin : gen_select_weight
            assign selected_weight_wires[w_idx] = select_weight_from_word(bram_dout_wires[w_idx], event_offset_pipeline_reg[LP_PIPE_DEPTH-1]);
        end
    endgenerate

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
     * BRAM 地址、事件 offset 和 valid 流水线。
     * event_ready 在 S_PROCESS 状态持续为 1，因此稳定后可以每拍接收一个事件。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bram_addr_reg <= {P_BRAM_ADDR_WIDTH{1'b0}};
            for (pipe_idx = 0; pipe_idx < LP_PIPE_DEPTH; pipe_idx = pipe_idx + 1) begin
                event_offset_pipeline_reg[pipe_idx] <= {LP_OFFSET_WIDTH{1'b0}};
                valid_pipeline_reg[pipe_idx] <= 1'b0;
            end
        end else begin
            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESS) begin
                bram_addr_reg <= {P_BRAM_ADDR_WIDTH{1'b0}};
                for (pipe_idx = 0; pipe_idx < LP_PIPE_DEPTH; pipe_idx = pipe_idx + 1) begin
                    event_offset_pipeline_reg[pipe_idx] <= {LP_OFFSET_WIDTH{1'b0}};
                    valid_pipeline_reg[pipe_idx] <= 1'b0;
                end
            end else if (current_state_reg == S_PROCESS || current_state_reg == S_DRAIN) begin
                for (pipe_idx = LP_PIPE_DEPTH - 1; pipe_idx > 0; pipe_idx = pipe_idx - 1) begin
                    event_offset_pipeline_reg[pipe_idx] <= event_offset_pipeline_reg[pipe_idx-1];
                    valid_pipeline_reg[pipe_idx] <= valid_pipeline_reg[pipe_idx-1];
                end

                /*
                 * 方案B：新事件当拍直接驱动 BRAM 地址。
                 * 这里保存一份地址，是为了没有新事件但流水线还没排空时，继续保持 BRAM ena 和地址稳定。
                 */
                if (event_accept_w) begin
                    bram_addr_reg <= i_event_addr[P_EVENT_ADDR_WIDTH-1:LP_OFFSET_WIDTH];
                end
                event_offset_pipeline_reg[0] <= i_event_addr[LP_OFFSET_WIDTH-1:0];
                valid_pipeline_reg[0] <= event_accept_w;
            end
        end
    end

    /*
     * 输出电流累加。
     * 每个 AER 事件代表一个输入脉冲，因此只需要把该地址对应的权重加入累加器。
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
                    o_all_currents_I[accum_idx] <= o_all_currents_I[accum_idx] +
                        {{(P_NEURON_VALUE_TOTAL_BITS - P_WEIGHT_BIT_WIDTH){selected_weight_wires[accum_idx][P_WEIGHT_BIT_WIDTH-1]}},
                         selected_weight_wires[accum_idx]};
                end
            end

            if (current_state_reg == S_DONE) begin
                o_all_currents_valid <= 1'b1;
            end
        end
    end

endmodule
