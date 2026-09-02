module block_event_fifo #(
    parameter P_BLOCK_ID_WIDTH = 6,
    parameter P_BLOCK_SIZE = 32,
    parameter P_FIFO_DEPTH = 64
) (
    input wire clk,
    input wire rst_n,
    input wire i_clear,

    input wire i_block_valid,
    input wire [P_BLOCK_ID_WIDTH-1:0] i_block_id,
    input wire [P_BLOCK_SIZE-1:0] i_block_mask,

    input wire i_block_ready,
    output wire o_block_valid,
    output wire [P_BLOCK_ID_WIDTH-1:0] o_block_id,
    output wire [P_BLOCK_SIZE-1:0] o_block_mask,
    output wire o_empty,
    output wire o_full,
    output wire [$clog2(P_FIFO_DEPTH + 1)-1:0] o_count,
    output reg o_overflow
);

    /*
     * Block-Mask AER 事件 FIFO。
     * 普通 AER FIFO 只保存一个 event_addr；
     * 这里保存 block_id 和 block_mask，用于缓存打包后的块事件。
     */
    localparam LP_PTR_WIDTH = $clog2(P_FIFO_DEPTH);
    localparam LP_COUNT_WIDTH = $clog2(P_FIFO_DEPTH + 1);
    localparam [LP_COUNT_WIDTH-1:0] LP_FIFO_DEPTH_VALUE = P_FIFO_DEPTH;

    reg [P_BLOCK_ID_WIDTH-1:0] fifo_block_id_mem [P_FIFO_DEPTH-1:0];
    reg [P_BLOCK_SIZE-1:0] fifo_block_mask_mem [P_FIFO_DEPTH-1:0];
    reg [LP_PTR_WIDTH-1:0] write_ptr_reg;
    reg [LP_PTR_WIDTH-1:0] read_ptr_reg;
    reg [LP_COUNT_WIDTH-1:0] fifo_count_reg;

    wire write_fire_w;
    wire read_fire_w;

    assign o_empty = (fifo_count_reg == {LP_COUNT_WIDTH{1'b0}});
    assign o_full = (fifo_count_reg == LP_FIFO_DEPTH_VALUE);
    assign o_count = fifo_count_reg;
    assign o_block_valid = !o_empty;
    assign o_block_id = fifo_block_id_mem[read_ptr_reg];
    assign o_block_mask = fifo_block_mask_mem[read_ptr_reg];

    assign write_fire_w = i_block_valid && !o_full;
    assign read_fire_w = i_block_ready && !o_empty;

    /*
     * 写端口和写指针。
     * 输入 block 有效且 FIFO 未满时，保存 block_id 和 block_mask。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_ptr_reg <= {LP_PTR_WIDTH{1'b0}};
        end else if (i_clear) begin
            write_ptr_reg <= {LP_PTR_WIDTH{1'b0}};
        end else if (write_fire_w) begin
            fifo_block_id_mem[write_ptr_reg] <= i_block_id;
            fifo_block_mask_mem[write_ptr_reg] <= i_block_mask;
            if (write_ptr_reg == P_FIFO_DEPTH - 1) begin
                write_ptr_reg <= {LP_PTR_WIDTH{1'b0}};
            end else begin
                write_ptr_reg <= write_ptr_reg + 1'b1;
            end
        end
    end

    /*
     * 读指针。
     * 输出端采用组合读；后级真正接收 block 后，读指针前进。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_ptr_reg <= {LP_PTR_WIDTH{1'b0}};
        end else if (i_clear) begin
            read_ptr_reg <= {LP_PTR_WIDTH{1'b0}};
        end else if (read_fire_w) begin
            if (read_ptr_reg == P_FIFO_DEPTH - 1) begin
                read_ptr_reg <= {LP_PTR_WIDTH{1'b0}};
            end else begin
                read_ptr_reg <= read_ptr_reg + 1'b1;
            end
        end
    end

    /*
     * FIFO 计数。
     * 同一拍既写又读时，缓存数量保持不变。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_count_reg <= {LP_COUNT_WIDTH{1'b0}};
        end else if (i_clear) begin
            fifo_count_reg <= {LP_COUNT_WIDTH{1'b0}};
        end else begin
            case ({write_fire_w, read_fire_w})
                2'b10: fifo_count_reg <= fifo_count_reg + 1'b1;
                2'b01: fifo_count_reg <= fifo_count_reg - 1'b1;
                default: fifo_count_reg <= fifo_count_reg;
            endcase
        end
    end

    /*
     * 溢出标志。
     * 当输入 block 有效但 FIFO 已满时置位，直到复位或 clear。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_overflow <= 1'b0;
        end else if (i_clear) begin
            o_overflow <= 1'b0;
        end else if (i_block_valid && o_full) begin
            o_overflow <= 1'b1;
        end
    end

endmodule
