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
     * 当前版本维护 4 个 block 聚合 entry，可以合并短距离重复出现的 block_id。
     */
    localparam LP_BIT_IDX_WIDTH = $clog2(P_BLOCK_SIZE);
    localparam LP_AGG_ENTRIES = 4;
    localparam LP_AGG_IDX_WIDTH = 2;

    localparam [1:0] S_COLLECT = 2'b00;
    localparam [1:0] S_FLUSH = 2'b01;
    localparam [1:0] S_DONE = 2'b10;

    reg [1:0] current_state_reg;
    reg [1:0] next_state_reg;

    reg [P_BLOCK_ID_WIDTH-1:0] entry_block_id_reg [0:LP_AGG_ENTRIES-1];
    reg [P_BLOCK_SIZE-1:0] entry_block_mask_reg [0:LP_AGG_ENTRIES-1];
    reg [LP_AGG_ENTRIES-1:0] entry_valid_reg;
    reg frame_done_pending_reg;

    wire [P_BLOCK_ID_WIDTH-1:0] input_block_id_w;
    wire [LP_BIT_IDX_WIDTH-1:0] input_bit_idx_w;
    wire event_accept_w;
    wire block_accept_w;
    wire flush_need_w;
    wire full_miss_w;
    wire frame_flush_w;

    reg input_hit_comb;
    reg has_free_comb;
    reg has_valid_comb;
    reg flush_last_entry_comb;
    reg [LP_AGG_IDX_WIDTH-1:0] hit_idx_comb;
    reg [LP_AGG_IDX_WIDTH-1:0] free_idx_comb;
    reg [LP_AGG_IDX_WIDTH-1:0] flush_idx_comb;
    integer entry_idx;

    assign input_block_id_w = i_event_addr[P_EVENT_ADDR_WIDTH-1:LP_BIT_IDX_WIDTH];
    assign input_bit_idx_w = i_event_addr[LP_BIT_IDX_WIDTH-1:0];

    always @(*) begin
        input_hit_comb = 1'b0;
        has_free_comb = 1'b0;
        has_valid_comb = 1'b0;
        flush_last_entry_comb = 1'b1;
        hit_idx_comb = {LP_AGG_IDX_WIDTH{1'b0}};
        free_idx_comb = {LP_AGG_IDX_WIDTH{1'b0}};
        flush_idx_comb = {LP_AGG_IDX_WIDTH{1'b0}};

        for (entry_idx = 0; entry_idx < LP_AGG_ENTRIES; entry_idx = entry_idx + 1) begin
            if (entry_valid_reg[entry_idx]) begin
                has_valid_comb = 1'b1;
            end else if (!has_free_comb) begin
                has_free_comb = 1'b1;
                free_idx_comb = entry_idx[LP_AGG_IDX_WIDTH-1:0];
            end

            if (entry_valid_reg[entry_idx] && (entry_block_id_reg[entry_idx] == input_block_id_w)) begin
                input_hit_comb = 1'b1;
                hit_idx_comb = entry_idx[LP_AGG_IDX_WIDTH-1:0];
            end
        end

        for (entry_idx = 0; entry_idx < LP_AGG_ENTRIES; entry_idx = entry_idx + 1) begin
            if (entry_valid_reg[entry_idx] && flush_last_entry_comb) begin
                flush_idx_comb = entry_idx[LP_AGG_IDX_WIDTH-1:0];
                flush_last_entry_comb = 1'b0;
            end
        end

        flush_last_entry_comb = 1'b1;
        for (entry_idx = 0; entry_idx < LP_AGG_ENTRIES; entry_idx = entry_idx + 1) begin
            if (entry_valid_reg[entry_idx] && (entry_idx[LP_AGG_IDX_WIDTH-1:0] != flush_idx_comb)) begin
                flush_last_entry_comb = 1'b0;
            end
        end
    end

    assign full_miss_w = i_event_valid && !input_hit_comb && !has_free_comb;
    assign frame_flush_w = i_event_frame_done && !i_event_valid;

    /*
     * 需要输出当前 block 的两种情况：
     * 1. 聚合 entry 已满且新事件没有命中，需要先发走一个 entry；
     * 2. 当前时间步结束，需要把所有 pending entry 刷出去。
     */
    assign flush_need_w = has_valid_comb &&
                          ((current_state_reg == S_FLUSH) ||
                           frame_flush_w ||
                           full_miss_w);

    assign o_block_valid = flush_need_w;
    assign o_block_id = entry_block_id_reg[flush_idx_comb];
    assign o_block_mask = entry_block_mask_reg[flush_idx_comb];

    assign block_accept_w = o_block_valid && i_block_ready;

    /*
     * 前级事件 ready。
     * 命中已有 entry 或存在空 entry 时直接接收；
     * entry 满且未命中时，只有同拍能 flush 一个 entry 才接收新事件。
     */
    assign o_event_ready = (current_state_reg == S_COLLECT) &&
                           (input_hit_comb || has_free_comb || i_block_ready) &&
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
                    if (has_valid_comb) begin
                        if (i_block_ready && flush_last_entry_comb) begin
                            next_state_reg = S_DONE;
                        end else begin
                            next_state_reg = S_FLUSH;
                        end
                    end else begin
                        next_state_reg = S_DONE;
                    end
                end else if (full_miss_w && !i_block_ready) begin
                    next_state_reg = S_FLUSH;
                end
            end

            S_FLUSH: begin
                if (i_block_ready) begin
                    if (frame_done_pending_reg && flush_last_entry_comb) begin
                        next_state_reg = S_DONE;
                    end else if (frame_done_pending_reg) begin
                        next_state_reg = S_FLUSH;
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
     * 4-entry block 聚合缓存。
     * hit 时合并 mask；miss 且有空 entry 时分配；full miss 且后级 ready 时，
     * 同拍输出一个 entry，并在该位置放入新 block。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            entry_valid_reg <= {LP_AGG_ENTRIES{1'b0}};
            for (entry_idx = 0; entry_idx < LP_AGG_ENTRIES; entry_idx = entry_idx + 1) begin
                entry_block_id_reg[entry_idx] <= {P_BLOCK_ID_WIDTH{1'b0}};
                entry_block_mask_reg[entry_idx] <= {P_BLOCK_SIZE{1'b0}};
            end
        end else if (i_clear) begin
            entry_valid_reg <= {LP_AGG_ENTRIES{1'b0}};
            for (entry_idx = 0; entry_idx < LP_AGG_ENTRIES; entry_idx = entry_idx + 1) begin
                entry_block_id_reg[entry_idx] <= {P_BLOCK_ID_WIDTH{1'b0}};
                entry_block_mask_reg[entry_idx] <= {P_BLOCK_SIZE{1'b0}};
            end
        end else begin
            if (block_accept_w) begin
                entry_valid_reg[flush_idx_comb] <= 1'b0;
                entry_block_mask_reg[flush_idx_comb] <= {P_BLOCK_SIZE{1'b0}};
            end

            if (event_accept_w) begin
                if (input_hit_comb) begin
                    entry_block_mask_reg[hit_idx_comb][input_bit_idx_w] <= 1'b1;
                end else if (has_free_comb) begin
                    entry_valid_reg[free_idx_comb] <= 1'b1;
                    entry_block_id_reg[free_idx_comb] <= input_block_id_w;
                    entry_block_mask_reg[free_idx_comb] <=
                        ({P_BLOCK_SIZE{1'b0}} | ({{(P_BLOCK_SIZE-1){1'b0}}, 1'b1} << input_bit_idx_w));
                end else begin
                    entry_valid_reg[flush_idx_comb] <= 1'b1;
                    entry_block_id_reg[flush_idx_comb] <= input_block_id_w;
                    entry_block_mask_reg[flush_idx_comb] <=
                        ({P_BLOCK_SIZE{1'b0}} | ({{(P_BLOCK_SIZE-1){1'b0}}, 1'b1} << input_bit_idx_w));
                end
            end
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
        end else if (current_state_reg == S_COLLECT &&
                     i_event_frame_done &&
                     has_valid_comb &&
                     !(i_block_ready && flush_last_entry_comb)) begin
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
