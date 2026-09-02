module block_event_arbiter #(
    parameter P_NUM_PORTS = 4,
    parameter P_BLOCK_ID_WIDTH = 6,
    parameter P_BLOCK_SIZE = 32
) (
    input wire clk,
    input wire rst_n,
    input wire i_clear,

    input wire [P_NUM_PORTS-1:0] i_block_valid,
    input wire [P_NUM_PORTS-1:0][P_BLOCK_ID_WIDTH-1:0] i_block_id,
    input wire [P_NUM_PORTS-1:0][P_BLOCK_SIZE-1:0] i_block_mask,
    output reg [P_NUM_PORTS-1:0] o_block_ready,

    output reg o_block_valid,
    output reg [P_BLOCK_ID_WIDTH-1:0] o_block_id,
    output reg [P_BLOCK_SIZE-1:0] o_block_mask,
    input wire i_block_ready
);

    /*
     * 多路 Block-Mask AER 事件仲裁器。
     * 当前只保留轮询策略，避免某个 core 长期占用输出通道。
     */
    localparam LP_PORT_WIDTH = $clog2(P_NUM_PORTS);

    reg [LP_PORT_WIDTH-1:0] rr_ptr_reg;
    reg [LP_PORT_WIDTH-1:0] selected_port_comb;
    reg selected_valid_comb;

    integer search_idx;
    integer port_idx;

    /*
     * 仲裁选择逻辑。
     * 这里只决定当前输出选择哪一路 block 包。
     */
    always @(*) begin
        selected_valid_comb = 1'b0;
        selected_port_comb = {LP_PORT_WIDTH{1'b0}};
        port_idx = 0;

        selected_port_comb = rr_ptr_reg;

        for (search_idx = 0; search_idx < P_NUM_PORTS; search_idx = search_idx + 1) begin
            port_idx = rr_ptr_reg + search_idx;
            if (port_idx >= P_NUM_PORTS) begin
                port_idx = port_idx - P_NUM_PORTS;
            end

            if (!selected_valid_comb && i_block_valid[port_idx]) begin
                selected_valid_comb = 1'b1;
                selected_port_comb = port_idx[LP_PORT_WIDTH-1:0];
            end
        end
    end

    /*
     * 输出 block 包和 ready 分发。
     * 只有被选中的端口能看到后级 ready，其余端口保持等待。
     */
    always @(*) begin
        o_block_valid = selected_valid_comb;
        o_block_id = i_block_id[selected_port_comb];
        o_block_mask = i_block_mask[selected_port_comb];
        o_block_ready = {P_NUM_PORTS{1'b0}};

        if (selected_valid_comb) begin
            o_block_ready[selected_port_comb] = i_block_ready;
        end
    end

    /*
     * 轮询指针只在 block 包完成握手后前进。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr_reg <= {LP_PORT_WIDTH{1'b0}};
        end else if (i_clear) begin
            rr_ptr_reg <= {LP_PORT_WIDTH{1'b0}};
        end else if (o_block_valid && i_block_ready) begin
            if (selected_port_comb == P_NUM_PORTS - 1) begin
                rr_ptr_reg <= {LP_PORT_WIDTH{1'b0}};
            end else begin
                rr_ptr_reg <= selected_port_comb + 1'b1;
            end
        end
    end

endmodule
