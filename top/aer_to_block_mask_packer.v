module aer_to_block_mask_packer #(
    parameter P_EVENT_ADDR_WIDTH = 11,
    parameter P_BLOCK_SIZE = 32,
    parameter P_BLOCK_ID_WIDTH = 6
) (
    input wire clk,
    input wire rst_n,
    input wire i_clear,

    input wire i_event_valid,
    input wire [P_EVENT_ADDR_WIDTH-1:0] i_event_addr,
    input wire i_event_frame_done,
    output wire o_event_ready,

    output wire o_block_valid,
    output wire [P_BLOCK_ID_WIDTH-1:0] o_block_id,
    output wire [P_BLOCK_SIZE-1:0] o_block_mask,
    input wire i_block_ready,

    output reg o_block_frame_done
);

    /*
     * AER 到 Block-Mask AER 的轻量级流式打包器。
     * 输入仍然是普通 AER 地址事件；输出变成 block_id + block_mask。
     * 第一版只缓存一个 block，因此只合并连续到来的同一个 block_id。
     */
    localparam LP_BIT_IDX_WIDTH = $clog2(P_BLOCK_SIZE);

    localparam [1:0] S_COLLECT = 2'b00;
    localparam [1:0] S_FLUSH = 2'b01;
    localparam [1:0] S_DONE = 2'b10;

    reg [1:0] current_state_reg;
    reg [1:0] next_state_reg;

    reg [P_BLOCK_ID_WIDTH-1:0] block_id_reg;
    reg [P_BLOCK_SIZE-1:0] block_mask_reg;
    reg has_block_reg;
    reg frame_done_pending_reg;

    wire [P_BLOCK_ID_WIDTH-1:0] input_block_id_w;
    wire [LP_BIT_IDX_WIDTH-1:0] input_bit_idx_w;
    wire input_same_block_w;
    wire input_new_block_w;
    wire event_accept_w;
    wire block_accept_w;
    wire flush_need_w;

    assign input_block_id_w = i_event_addr[P_EVENT_ADDR_WIDTH-1:LP_BIT_IDX_WIDTH];
    assign input_bit_idx_w = i_event_addr[LP_BIT_IDX_WIDTH-1:0];
    assign input_same_block_w = has_block_reg && (input_block_id_w == block_id_reg);
    assign input_new_block_w = has_block_reg && i_event_valid && !input_same_block_w;

    /*
     * 需要输出当前 block 的两种情况：
     * 1. 新事件属于下一个 block，需要先把旧 block 发走；
     * 2. 当前时间步结束，需要把最后一个 pending block 刷出去。
     */
    assign flush_need_w = has_block_reg &&
                          ((current_state_reg == S_FLUSH) ||
                           (i_event_frame_done && !i_event_valid) ||
                           input_new_block_w);

    assign o_block_valid = flush_need_w;
    assign o_block_id = block_id_reg;
    assign o_block_mask = block_mask_reg;

    assign block_accept_w = o_block_valid && i_block_ready;

    /*
     * 前级事件 ready。
     * 同 block 事件可以直接合并；
     * 不同 block 事件只有在旧 block 同拍被后级接收时才允许进入，
     * 否则拉低 ready 反压前级，避免覆盖当前缓存。
     */
    assign o_event_ready = (current_state_reg == S_COLLECT) &&
                           (!input_new_block_w || i_block_ready) &&
                           !i_event_frame_done;
    assign event_accept_w = i_event_valid && o_event_ready;

    /*
     * 两段式状态机的组合转移逻辑。
     */
    always @(*) begin
        next_state_reg = current_state_reg;

        case (current_state_reg)
            S_COLLECT: begin
                if (i_event_frame_done) begin
                    if (has_block_reg) begin
                        if (i_block_ready) begin
                            next_state_reg = S_DONE;
                        end else begin
                            next_state_reg = S_FLUSH;
                        end
                    end else begin
                        next_state_reg = S_DONE;
                    end
                end else if (input_new_block_w && !i_block_ready) begin
                    next_state_reg = S_FLUSH;
                end
            end

            S_FLUSH: begin
                if (i_block_ready) begin
                    if (frame_done_pending_reg) begin
                        next_state_reg = S_DONE;
                    end else begin
                        next_state_reg = S_COLLECT;
                    end
                end
            end

            S_DONE: begin
                next_state_reg = S_COLLECT;
            end

            default: begin
                next_state_reg = S_COLLECT;
            end
        endcase
    end

    /*
     * 状态寄存器。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state_reg <= S_COLLECT;
        end else if (i_clear) begin
            current_state_reg <= S_COLLECT;
        end else begin
            current_state_reg <= next_state_reg;
        end
    end

    /*
     * block_id 缓存。
     * 新 block 的第一个事件被接收时，记录它所属的 block_id。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            block_id_reg <= {P_BLOCK_ID_WIDTH{1'b0}};
        end else if (i_clear) begin
            block_id_reg <= {P_BLOCK_ID_WIDTH{1'b0}};
        end else if (event_accept_w && (!has_block_reg || input_new_block_w)) begin
            block_id_reg <= input_block_id_w;
        end
    end

    /*
     * block mask 缓存。
     * 同 block 事件只置位对应 bit；切换到新 block 时，旧 mask 同拍发出，
     * 寄存器里开始记录新 block 的第一个 bit。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            block_mask_reg <= {P_BLOCK_SIZE{1'b0}};
        end else if (i_clear) begin
            block_mask_reg <= {P_BLOCK_SIZE{1'b0}};
        end else if (event_accept_w && (!has_block_reg || input_new_block_w)) begin
            block_mask_reg <= ({P_BLOCK_SIZE{1'b0}} | ({{(P_BLOCK_SIZE-1){1'b0}}, 1'b1} << input_bit_idx_w));
        end else if (event_accept_w && input_same_block_w) begin
            block_mask_reg[input_bit_idx_w] <= 1'b1;
        end else if (block_accept_w && !event_accept_w) begin
            block_mask_reg <= {P_BLOCK_SIZE{1'b0}};
        end
    end

    /*
     * 当前是否有尚未输出的 block。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            has_block_reg <= 1'b0;
        end else if (i_clear) begin
            has_block_reg <= 1'b0;
        end else if (event_accept_w) begin
            has_block_reg <= 1'b1;
        end else if (block_accept_w) begin
            has_block_reg <= 1'b0;
        end
    end

    /*
     * frame_done 等待标志。
     * 如果时间步结束时还有 block 没被后级接收，需要等最后一个 block 发完后再输出 done。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_done_pending_reg <= 1'b0;
        end else if (i_clear) begin
            frame_done_pending_reg <= 1'b0;
        end else if (current_state_reg == S_COLLECT && i_event_frame_done && has_block_reg && !i_block_ready) begin
            frame_done_pending_reg <= 1'b1;
        end else if (current_state_reg == S_DONE) begin
            frame_done_pending_reg <= 1'b0;
        end
    end

    /*
     * 输出时间步结束脉冲。
     * 最后一包 block 被接收后，或者空帧直接结束时，拉高一个周期。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_block_frame_done <= 1'b0;
        end else if (i_clear) begin
            o_block_frame_done <= 1'b0;
        end else begin
            o_block_frame_done <= (next_state_reg == S_DONE);
        end
    end

endmodule
