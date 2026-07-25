module aer_event_fifo #(
    parameter P_ADDR_WIDTH = 11,
    parameter P_FIFO_DEPTH = 64
) (
    input wire clk,
    input wire rst_n,
    input wire i_clear,

    input wire i_event_valid,
    input wire [P_ADDR_WIDTH-1:0] i_event_addr,

    input wire i_event_ready,
    output wire o_event_valid,
    output wire [P_ADDR_WIDTH-1:0] o_event_addr,
    output wire o_empty,
    output wire o_full,
    output wire [$clog2(P_FIFO_DEPTH + 1)-1:0] o_count,
    output reg o_overflow
);

    /*
     * 这个 FIFO 只保存 AER 地址，不再保存完整 1568bit 脉冲向量。
     * 卷积 LIF 每产生一个脉冲，就把对应地址写入 FIFO；
     * 后面的 AER 全连接层 ready 时，再从 FIFO 中逐个读出事件。
     */
    localparam LP_PTR_WIDTH = $clog2(P_FIFO_DEPTH);
    localparam LP_COUNT_WIDTH = $clog2(P_FIFO_DEPTH + 1);
    localparam [LP_COUNT_WIDTH-1:0] LP_FIFO_DEPTH_VALUE = P_FIFO_DEPTH;

    reg [P_ADDR_WIDTH-1:0] fifo_mem [P_FIFO_DEPTH-1:0];
    reg [LP_PTR_WIDTH-1:0] write_ptr_reg;
    reg [LP_PTR_WIDTH-1:0] read_ptr_reg;
    reg [LP_COUNT_WIDTH-1:0] fifo_count_reg;

    wire write_fire_w;
    wire read_fire_w;

    assign o_empty = (fifo_count_reg == {LP_COUNT_WIDTH{1'b0}});
    assign o_full = (fifo_count_reg == LP_FIFO_DEPTH_VALUE);
    assign o_count = fifo_count_reg;
    assign o_event_valid = !o_empty;
    assign o_event_addr = fifo_mem[read_ptr_reg];

    assign write_fire_w = i_event_valid && !o_full;
    assign read_fire_w = i_event_ready && !o_empty;

    /*
     * 写端口和写指针。
     * 只有 FIFO 未满且输入事件有效时，才把地址写入当前写指针位置。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_ptr_reg <= {LP_PTR_WIDTH{1'b0}};
        end else if (i_clear) begin
            write_ptr_reg <= {LP_PTR_WIDTH{1'b0}};
        end else if (write_fire_w) begin
            fifo_mem[write_ptr_reg] <= i_event_addr;
            if (write_ptr_reg == P_FIFO_DEPTH - 1) begin
                write_ptr_reg <= {LP_PTR_WIDTH{1'b0}};
            end else begin
                write_ptr_reg <= write_ptr_reg + 1'b1;
            end
        end
    end

    /*
     * 读指针。
     * 输出数据使用 fifo_mem[read_ptr_reg] 组合读出；
     * 后级真正接收事件后，读指针才前进到下一个地址。
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
     * FIFO 当前保存的事件数量。
     * 同一拍既写又读时，数量保持不变。
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
     * 正常情况下 aer_linear_layer 每拍都 ready，FIFO 不应满；
     * 如果输入事件有效但 FIFO 已满，就保持 overflow 为 1，方便仿真定位问题。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_overflow <= 1'b0;
        end else if (i_clear) begin
            o_overflow <= 1'b0;
        end else if (i_event_valid && o_full) begin
            o_overflow <= 1'b1;
        end
    end

endmodule
