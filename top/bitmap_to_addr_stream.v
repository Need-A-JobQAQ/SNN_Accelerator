module bitmap_to_addr_stream #(
    parameter P_BITMAP_WIDTH = 1568,
    parameter P_ADDR_WIDTH = $clog2(P_BITMAP_WIDTH),
    parameter P_WORD_WIDTH = 32                      //每32位为一组，称为一“字”（word），共1568/32=49个字
) (
    input wire clk,
    input wire rst_n,
    input wire i_start,
    input wire [P_BITMAP_WIDTH-1:0] i_active_bitmap,
    input wire i_addr_ready,

    output reg o_addr_valid,
    output reg [P_ADDR_WIDTH-1:0] o_addr,
    output reg o_done,
    output reg o_busy,
    output reg [31:0] o_active_count
);

    /*
     * 活跃位图分段压缩模块。
     * 整帧 bitmap 被划分成若干个小分段；空分段一拍跳过，
     * 非空分段只输出其中为 1 的地址，从而避免逐神经元地址扫描。
     */
    localparam LP_NUM_WORDS = (P_BITMAP_WIDTH + P_WORD_WIDTH - 1) / P_WORD_WIDTH;               //总word的个数，不足一个word的按一个word算，缺少的部分用0填充（pad）。 P_WORD_WIDTH - 1 是保证做除法后余数仍然算一个word
    localparam LP_PADDED_WIDTH = LP_NUM_WORDS * P_WORD_WIDTH;                                   //

    localparam LP_WORD_INDEX_WIDTH = (LP_NUM_WORDS <= 1) ? 1 : $clog2(LP_NUM_WORDS);
    localparam LP_BIT_INDEX_WIDTH = (P_WORD_WIDTH <= 1) ? 1 : $clog2(P_WORD_WIDTH);

    localparam [1:0] S_IDLE = 2'b00;
    localparam [1:0] S_SCAN_WORD = 2'b01;
    localparam [1:0] S_EMIT_ADDR = 2'b10;
    localparam [1:0] S_DONE = 2'b11;

    reg [1:0] current_state_reg;
    reg [1:0] next_state_reg;

    reg [LP_PADDED_WIDTH-1:0] work_bitmap_reg;
    reg [LP_WORD_INDEX_WIDTH-1:0] word_idx_reg;

    wire [LP_PADDED_WIDTH-1:0] padded_input_bitmap_w;
    wire [P_WORD_WIDTH-1:0] current_word_w;
    wire current_word_active_w;
    wire current_word_onehot_w;
    wire addr_fire_w;
    wire last_word_w;
    wire [LP_BIT_INDEX_WIDTH-1:0] first_bit_idx_w;
    wire [P_ADDR_WIDTH-1:0] first_addr_w;

    assign padded_input_bitmap_w = {{(LP_PADDED_WIDTH - P_BITMAP_WIDTH){1'b0}}, i_active_bitmap}; //不足一个word的用0填充
    assign current_word_w = work_bitmap_reg[word_idx_reg * P_WORD_WIDTH +: P_WORD_WIDTH];         //当前分组
    assign current_word_active_w = |current_word_w;                                               //当前分组中是否有有效脉冲
    assign current_word_onehot_w = current_word_active_w &&
                                   ((current_word_w & (current_word_w - 1'b1)) == {P_WORD_WIDTH{1'b0}});
    assign addr_fire_w = o_addr_valid && i_addr_ready;
    assign last_word_w = (word_idx_reg == LP_NUM_WORDS - 1);
    assign first_bit_idx_w = find_first_bit(current_word_w);
    assign first_addr_w = (word_idx_reg * P_WORD_WIDTH) + first_bit_idx_w;                        //基地址 + 偏移地址 = 绝对地址

    /*
     * 在当前分段内寻找地址最小的 active bit。
     */
    function [LP_BIT_INDEX_WIDTH-1:0] find_first_bit;
        input [P_WORD_WIDTH-1:0] word_bits;
        integer bit_idx;
        begin
            find_first_bit = {LP_BIT_INDEX_WIDTH{1'b0}};
            for (bit_idx = P_WORD_WIDTH - 1; bit_idx >= 0; bit_idx = bit_idx - 1) begin            //从高向低遍历，不断覆盖高位idx，最终留下最低位的idx
                if (word_bits[bit_idx]) begin
                    find_first_bit = bit_idx[LP_BIT_INDEX_WIDTH-1:0];
                end
            end
        end
    endfunction

    /*
     * 两段式状态机的组合转移逻辑。
     */
    always @(*) begin
        next_state_reg = current_state_reg;

        case (current_state_reg)
            S_IDLE: begin
                if (i_start) begin
                    if (i_active_bitmap == {P_BITMAP_WIDTH{1'b0}}) begin
                        next_state_reg = S_DONE;
                    end else begin
                        next_state_reg = S_SCAN_WORD;
                    end
                end
            end

            S_SCAN_WORD: begin
                if (current_word_active_w) begin
                    next_state_reg = S_EMIT_ADDR;
                end else if (last_word_w) begin
                    next_state_reg = S_DONE;
                end
            end

            S_EMIT_ADDR: begin
                if (addr_fire_w && current_word_onehot_w && last_word_w) begin
                    next_state_reg = S_DONE;
                end else if (addr_fire_w && current_word_onehot_w) begin
                    next_state_reg = S_SCAN_WORD;
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
     * 输出组合逻辑。
     */
    always @(*) begin
        o_addr_valid = 1'b0;
        o_addr = first_addr_w;
        o_done = 1'b0;
        o_busy = (current_state_reg != S_IDLE);

        if (current_state_reg == S_EMIT_ADDR && current_word_active_w) begin
            o_addr_valid = 1'b1;
        end

        if (current_state_reg == S_DONE) begin
            o_done = 1'b1;
        end
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
     * 工作位图寄存器。
     * start 时锁存整帧位图；地址被接收后，清掉对应 bit。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            work_bitmap_reg <= {LP_PADDED_WIDTH{1'b0}};
        end else if (current_state_reg == S_IDLE && i_start) begin
            work_bitmap_reg <= padded_input_bitmap_w;
        end else if (addr_fire_w) begin
            work_bitmap_reg[first_addr_w] <= 1'b0;
        end else if (current_state_reg == S_DONE) begin
            work_bitmap_reg <= {LP_PADDED_WIDTH{1'b0}};
        end
    end

    /*
     * 分段索引寄存器。
     * 空分段一拍跳过；非空分段被清空后，再进入下一分段。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            word_idx_reg <= {LP_WORD_INDEX_WIDTH{1'b0}};
        end else if (current_state_reg == S_IDLE && i_start) begin
            word_idx_reg <= {LP_WORD_INDEX_WIDTH{1'b0}};
        end else if (current_state_reg == S_SCAN_WORD && !current_word_active_w && !last_word_w) begin
            word_idx_reg <= word_idx_reg + 1'b1;
        end else if (addr_fire_w && current_word_onehot_w && !last_word_w) begin
            word_idx_reg <= word_idx_reg + 1'b1;
        end else if (current_state_reg == S_DONE) begin
            word_idx_reg <= {LP_WORD_INDEX_WIDTH{1'b0}};
        end
    end

    /*
     * 活跃地址计数寄存器。
     * 不做大规模组合 popcount，而是在地址流输出时顺序累计。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_active_count <= 32'd0;
        end else if (current_state_reg == S_IDLE && i_start) begin
            o_active_count <= 32'd0;
        end else if (addr_fire_w) begin
            o_active_count <= o_active_count + 1'b1;
        end
    end

endmodule
