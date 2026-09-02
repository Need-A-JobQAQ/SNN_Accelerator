module block_aer_linear_layer #(
    parameter P_NUM_INPUT_EVENTS = 1568,
    parameter P_BLOCK_SIZE = 32,
    parameter P_BLOCK_ID_WIDTH = $clog2((P_NUM_INPUT_EVENTS + P_BLOCK_SIZE - 1) / P_BLOCK_SIZE),
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

    input wire i_block_valid,
    input wire [P_BLOCK_ID_WIDTH-1:0] i_block_id,
    input wire [P_BLOCK_SIZE-1:0] i_block_mask,
    input wire i_block_frame_done,
    output reg o_block_ready,

    output reg signed [P_NUM_OUTPUT_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] o_all_currents_I,
    output reg o_all_currents_valid
);

    /*
     * Block-Mask AER 版全连接层。
     * 一个 block32 被拆成 8 个 group4；每个 group4 对应一个 64bit 权重 BRAM word。
     * group4 内只要存在任意 spike，就只读一次 BRAM，再由 4bit mask 选择性累加权重。
     */
    localparam LP_WEIGHTS_PER_WORD = P_BRAM_DATA_WIDTH / P_WEIGHT_BIT_WIDTH;
    localparam LP_GROUPS_PER_BLOCK = P_BLOCK_SIZE / LP_WEIGHTS_PER_WORD;
    localparam LP_GROUP_IDX_WIDTH = $clog2(LP_GROUPS_PER_BLOCK);
    localparam LP_PIPE_DEPTH = P_BRAM_READ_LATENCY;

    localparam [2:0] S_IDLE = 3'b000;
    localparam [2:0] S_WAIT_BLOCK = 3'b001;
    localparam [2:0] S_SCAN_GROUP = 3'b010;
    localparam [2:0] S_DRAIN = 3'b011;
    localparam [2:0] S_DONE = 3'b100;

    reg [2:0] current_state_reg;
    reg [2:0] next_state_reg;

    reg [P_BLOCK_ID_WIDTH-1:0] block_id_reg;
    reg [P_BLOCK_SIZE-1:0] block_mask_reg;
    reg frame_done_pending_reg;

    reg [LP_GROUP_IDX_WIDTH-1:0] group_idx_reg;
    reg [P_BRAM_ADDR_WIDTH-1:0] bram_addr_reg;
    reg [LP_WEIGHTS_PER_WORD-1:0] mask_pipeline_reg [LP_PIPE_DEPTH-1:0];
    reg valid_pipeline_reg [LP_PIPE_DEPTH-1:0];
    reg pipeline_busy_comb;
    reg accept_block_has_active_comb;
    reg scan_has_later_active_comb;
    reg [LP_GROUP_IDX_WIDTH-1:0] accept_first_group_comb;
    reg [LP_GROUP_IDX_WIDTH-1:0] scan_next_group_comb;

    wire block_accept_w;
    wire scan_group_active_w;
    wire scan_last_active_group_w;
    wire issue_group_read_w;
    wire frame_done_accept_w;
    wire [LP_WEIGHTS_PER_WORD-1:0] scan_group_mask_w;
    wire [P_BRAM_ADDR_WIDTH-1:0] scan_bram_addr_w;
    wire [P_BRAM_ADDR_WIDTH-1:0] bram_addr_w;
    wire bram_ena_w;
    wire pipeline_busy_w;
    wire accum_valid_w;

    wire signed [P_BRAM_DATA_WIDTH-1:0] bram_dout_wires [P_NUM_OUTPUT_NEURONS-1:0];
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] selected_sum_wires [P_NUM_OUTPUT_NEURONS-1:0];

    integer reset_idx;
    integer accum_idx;
    integer pipe_idx;
    integer busy_idx;
    integer accept_search_idx;
    integer scan_search_idx;
    genvar n_idx;
    genvar s_idx;

    assign block_accept_w = (current_state_reg == S_WAIT_BLOCK) && i_block_valid && o_block_ready;
    assign scan_group_mask_w = block_mask_reg[group_idx_reg * LP_WEIGHTS_PER_WORD +: LP_WEIGHTS_PER_WORD];
    assign scan_group_active_w = |scan_group_mask_w;
    assign scan_last_active_group_w = !scan_has_later_active_comb;
    assign issue_group_read_w = (current_state_reg == S_SCAN_GROUP) && scan_group_active_w;
    assign frame_done_accept_w = (current_state_reg == S_WAIT_BLOCK) && i_block_frame_done && o_block_ready;
    assign scan_bram_addr_w = (block_id_reg * LP_GROUPS_PER_BLOCK) + group_idx_reg;
    assign bram_addr_w = issue_group_read_w ? scan_bram_addr_w : bram_addr_reg;
    assign bram_ena_w = issue_group_read_w || pipeline_busy_comb;
    assign pipeline_busy_w = pipeline_busy_comb;
    assign accum_valid_w = valid_pipeline_reg[LP_PIPE_DEPTH-1];

    always @(*) begin
        pipeline_busy_comb = 1'b0;
        for (busy_idx = 0; busy_idx < LP_PIPE_DEPTH; busy_idx = busy_idx + 1) begin
            pipeline_busy_comb = pipeline_busy_comb || valid_pipeline_reg[busy_idx];
        end
    end

    /*
     * 新 block 到来时，先找出第一个有效 group4。
     * 如果整个 block 为空，就不进入扫描状态，避免白白扫 8 个空 group。
     */
    always @(*) begin
        accept_block_has_active_comb = 1'b0;
        accept_first_group_comb = {LP_GROUP_IDX_WIDTH{1'b0}};

        for (accept_search_idx = 0; accept_search_idx < LP_GROUPS_PER_BLOCK; accept_search_idx = accept_search_idx + 1) begin
            if (!accept_block_has_active_comb &&
                (|i_block_mask[accept_search_idx * LP_WEIGHTS_PER_WORD +: LP_WEIGHTS_PER_WORD])) begin
                accept_block_has_active_comb = 1'b1;
                accept_first_group_comb = accept_search_idx[LP_GROUP_IDX_WIDTH-1:0];
            end
        end
    end

    /*
     * 当前 group4 处理后，直接跳到下一个有效 group4。
     * 这一步把 block 内部的空 group 跳过去，减少 block_aer_linear_layer 的控制周期。
     */
    always @(*) begin
        scan_has_later_active_comb = 1'b0;
        scan_next_group_comb = group_idx_reg;

        for (scan_search_idx = 0; scan_search_idx < LP_GROUPS_PER_BLOCK; scan_search_idx = scan_search_idx + 1) begin
            if (!scan_has_later_active_comb &&
                (scan_search_idx > group_idx_reg) &&
                (|block_mask_reg[scan_search_idx * LP_WEIGHTS_PER_WORD +: LP_WEIGHTS_PER_WORD])) begin
                scan_has_later_active_comb = 1'b1;
                scan_next_group_comb = scan_search_idx[LP_GROUP_IDX_WIDTH-1:0];
            end
        end
    end

    /*
     * 从一个 64bit 权重字中按 4bit mask 选择多个 16bit 权重并求和。
     * mask[0] 对应最高 16bit，mask[3] 对应最低 16bit，与 aer_linear_layer 的 offset 规则一致。
     */
    function signed [P_NEURON_VALUE_TOTAL_BITS-1:0] sum_weight_word_by_mask;
        input signed [P_BRAM_DATA_WIDTH-1:0] bram_word;
        input [LP_WEIGHTS_PER_WORD-1:0] weight_mask;
        reg signed [P_WEIGHT_BIT_WIDTH-1:0] one_weight;
        integer weight_idx;
        begin
            sum_weight_word_by_mask = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            for (weight_idx = 0; weight_idx < LP_WEIGHTS_PER_WORD; weight_idx = weight_idx + 1) begin
                one_weight = bram_word[P_BRAM_DATA_WIDTH - 1 - (weight_idx * P_WEIGHT_BIT_WIDTH) -: P_WEIGHT_BIT_WIDTH];
                if (weight_mask[weight_idx]) begin
                    sum_weight_word_by_mask = sum_weight_word_by_mask +
                        {{(P_NEURON_VALUE_TOTAL_BITS - P_WEIGHT_BIT_WIDTH){one_weight[P_WEIGHT_BIT_WIDTH-1]}},
                         one_weight};
                end
            end
        end
    endfunction

    /*
     * 复用原工程的 10 个全连接层权重 BRAM IP。
     * 每个输出类别一个 BRAM，所有类别同拍读取同一个 group4 权重地址。
     */
    generate
        for (n_idx = 0; n_idx < P_NUM_OUTPUT_NEURONS; n_idx = n_idx + 1) begin : gen_weight_brams
            if (n_idx == 0) begin : gen_bram_0
                weights_0 u_bram_neuron_0 (.clka(clk), .ena(bram_ena_w), .addra(bram_addr_w), .douta(bram_dout_wires[n_idx]));
            end else if (n_idx == 1) begin : gen_bram_1
                weights_1 u_bram_neuron_1 (.clka(clk), .ena(bram_ena_w), .addra(bram_addr_w), .douta(bram_dout_wires[n_idx]));
            end else if (n_idx == 2) begin : gen_bram_2
                weights_2 u_bram_neuron_2 (.clka(clk), .ena(bram_ena_w), .addra(bram_addr_w), .douta(bram_dout_wires[n_idx]));
            end else if (n_idx == 3) begin : gen_bram_3
                weights_3 u_bram_neuron_3 (.clka(clk), .ena(bram_ena_w), .addra(bram_addr_w), .douta(bram_dout_wires[n_idx]));
            end else if (n_idx == 4) begin : gen_bram_4
                weights_4 u_bram_neuron_4 (.clka(clk), .ena(bram_ena_w), .addra(bram_addr_w), .douta(bram_dout_wires[n_idx]));
            end else if (n_idx == 5) begin : gen_bram_5
                weights_5 u_bram_neuron_5 (.clka(clk), .ena(bram_ena_w), .addra(bram_addr_w), .douta(bram_dout_wires[n_idx]));
            end else if (n_idx == 6) begin : gen_bram_6
                weights_6 u_bram_neuron_6 (.clka(clk), .ena(bram_ena_w), .addra(bram_addr_w), .douta(bram_dout_wires[n_idx]));
            end else if (n_idx == 7) begin : gen_bram_7
                weights_7 u_bram_neuron_7 (.clka(clk), .ena(bram_ena_w), .addra(bram_addr_w), .douta(bram_dout_wires[n_idx]));
            end else if (n_idx == 8) begin : gen_bram_8
                weights_8 u_bram_neuron_8 (.clka(clk), .ena(bram_ena_w), .addra(bram_addr_w), .douta(bram_dout_wires[n_idx]));
            end else if (n_idx == 9) begin : gen_bram_9
                weights_9 u_bram_neuron_9 (.clka(clk), .ena(bram_ena_w), .addra(bram_addr_w), .douta(bram_dout_wires[n_idx]));
            end
        end
    endgenerate

    generate
        for (s_idx = 0; s_idx < P_NUM_OUTPUT_NEURONS; s_idx = s_idx + 1) begin : gen_sum_weight
            assign selected_sum_wires[s_idx] =
                sum_weight_word_by_mask(bram_dout_wires[s_idx], mask_pipeline_reg[LP_PIPE_DEPTH-1]);
        end
    endgenerate

    /*
     * 两段式状态机的组合转移逻辑。
     */
    always @(*) begin
        next_state_reg = current_state_reg;
        o_block_ready = 1'b0;

        case (current_state_reg)
            S_IDLE: begin
                if (i_start) begin
                    next_state_reg = S_WAIT_BLOCK;
                end
            end

            S_WAIT_BLOCK: begin
                o_block_ready = 1'b1;
                if (i_block_valid) begin
                    if (accept_block_has_active_comb) begin
                        next_state_reg = S_SCAN_GROUP;
                    end else begin
                        next_state_reg = S_WAIT_BLOCK;
                    end
                end else if (i_block_frame_done) begin
                    if (pipeline_busy_w) begin
                        next_state_reg = S_DRAIN;
                    end else begin
                        next_state_reg = S_DONE;
                    end
                end
            end

            S_SCAN_GROUP: begin
                if (scan_last_active_group_w) begin
                    if (frame_done_pending_reg) begin
                        next_state_reg = S_DRAIN;
                    end else begin
                        next_state_reg = S_WAIT_BLOCK;
                    end
                end
            end

            S_DRAIN: begin
                if (!pipeline_busy_w) begin
                    if (frame_done_pending_reg) begin
                        next_state_reg = S_DONE;
                    end else begin
                        next_state_reg = S_WAIT_BLOCK;
                    end
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
     * block 包缓存。
     * 当前版本一次只扫描一个 block 包。
     * block 扫描结束后不等待 BRAM 流水线完全排空，可以先接收下一个 block。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            block_id_reg <= {P_BLOCK_ID_WIDTH{1'b0}};
            block_mask_reg <= {P_BLOCK_SIZE{1'b0}};
        end else if (current_state_reg == S_IDLE && next_state_reg == S_WAIT_BLOCK) begin
            block_id_reg <= {P_BLOCK_ID_WIDTH{1'b0}};
            block_mask_reg <= {P_BLOCK_SIZE{1'b0}};
        end else if (block_accept_w) begin
            block_id_reg <= i_block_id;
            block_mask_reg <= i_block_mask;
        end
    end

    /*
     * group4 扫描计数。
     * group_idx_reg 只落在有效 group4 上，空 group4 不再占用扫描周期。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            group_idx_reg <= {LP_GROUP_IDX_WIDTH{1'b0}};
        end else if (block_accept_w) begin
            group_idx_reg <= accept_first_group_comb;
        end else if (current_state_reg == S_SCAN_GROUP) begin
            if (scan_last_active_group_w) begin
                group_idx_reg <= {LP_GROUP_IDX_WIDTH{1'b0}};
            end else begin
                group_idx_reg <= scan_next_group_comb;
            end
        end
    end

    /*
     * frame_done 等待标志。
     * 如果最后一个 block 和 frame_done 分开到达，则记录结束请求。
     * frame_done 也遵循 ready 握手，只有 o_block_ready 为 1 时才会被接收。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_done_pending_reg <= 1'b0;
        end else if (current_state_reg == S_IDLE && next_state_reg == S_WAIT_BLOCK) begin
            frame_done_pending_reg <= 1'b0;
        end else if (frame_done_accept_w) begin
            frame_done_pending_reg <= 1'b1;
        end else if (current_state_reg == S_DONE) begin
            frame_done_pending_reg <= 1'b0;
        end
    end

    /*
     * BRAM 地址、group mask 和 valid 流水线。
     * 和 aer_linear_layer 保持一致：读请求当拍驱动 BRAM，mask/valid 进入同深度流水线。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bram_addr_reg <= {P_BRAM_ADDR_WIDTH{1'b0}};
            for (pipe_idx = 0; pipe_idx < LP_PIPE_DEPTH; pipe_idx = pipe_idx + 1) begin
                mask_pipeline_reg[pipe_idx] <= {LP_WEIGHTS_PER_WORD{1'b0}};
                valid_pipeline_reg[pipe_idx] <= 1'b0;
            end
        end else begin
            if (current_state_reg == S_IDLE && next_state_reg == S_WAIT_BLOCK) begin
                bram_addr_reg <= {P_BRAM_ADDR_WIDTH{1'b0}};
                for (pipe_idx = 0; pipe_idx < LP_PIPE_DEPTH; pipe_idx = pipe_idx + 1) begin
                    mask_pipeline_reg[pipe_idx] <= {LP_WEIGHTS_PER_WORD{1'b0}};
                    valid_pipeline_reg[pipe_idx] <= 1'b0;
                end
            end else if (current_state_reg == S_WAIT_BLOCK ||
                         current_state_reg == S_SCAN_GROUP ||
                         current_state_reg == S_DRAIN) begin
                for (pipe_idx = LP_PIPE_DEPTH - 1; pipe_idx > 0; pipe_idx = pipe_idx - 1) begin
                    mask_pipeline_reg[pipe_idx] <= mask_pipeline_reg[pipe_idx-1];
                    valid_pipeline_reg[pipe_idx] <= valid_pipeline_reg[pipe_idx-1];
                end

                if (issue_group_read_w) begin
                    bram_addr_reg <= scan_bram_addr_w;
                end
                mask_pipeline_reg[0] <= scan_group_mask_w;
                valid_pipeline_reg[0] <= issue_group_read_w;
            end
        end
    end

    /*
     * 输出电流累加。
     * 每次有效 BRAM 返回对应一个 group4，最多一次累加 4 个输入事件对应的权重。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_all_currents_valid <= 1'b0;
            for (reset_idx = 0; reset_idx < P_NUM_OUTPUT_NEURONS; reset_idx = reset_idx + 1) begin
                o_all_currents_I[reset_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            end
        end else begin
            o_all_currents_valid <= 1'b0;

            if (current_state_reg == S_IDLE && next_state_reg == S_WAIT_BLOCK) begin
                for (reset_idx = 0; reset_idx < P_NUM_OUTPUT_NEURONS; reset_idx = reset_idx + 1) begin
                    o_all_currents_I[reset_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
                end
            end else if (accum_valid_w) begin
                for (accum_idx = 0; accum_idx < P_NUM_OUTPUT_NEURONS; accum_idx = accum_idx + 1) begin
                    o_all_currents_I[accum_idx] <= o_all_currents_I[accum_idx] + selected_sum_wires[accum_idx];
                end
            end

            if (current_state_reg == S_DONE) begin
                o_all_currents_valid <= 1'b1;
            end
        end
    end

endmodule
