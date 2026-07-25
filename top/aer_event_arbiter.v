module aer_event_arbiter #(
    parameter P_NUM_PORTS = 4,
    parameter P_ADDR_WIDTH = 11
) (
    input wire clk,
    input wire rst_n,
    input wire i_clear,

    input wire [P_NUM_PORTS-1:0] i_event_valid,
    input wire [P_NUM_PORTS-1:0][P_ADDR_WIDTH-1:0] i_event_addr,
    output reg [P_NUM_PORTS-1:0] o_event_ready,

    output reg o_event_valid,
    output reg [P_ADDR_WIDTH-1:0] o_event_addr,
    input wire i_event_ready
);

    /*
     * 多路 AER 事件轮询仲裁器。
     * 每次后级真正接收一个事件后，下一次从下一个端口开始查找，
     * 避免固定优先级让低编号 core 长期占用输出通道。
     */
    localparam LP_PORT_WIDTH = $clog2(P_NUM_PORTS);

    reg [LP_PORT_WIDTH-1:0] rr_ptr_reg;
    reg [LP_PORT_WIDTH-1:0] selected_port_comb;
    reg selected_valid_comb;

    integer search_idx;
    integer port_idx;

    always @(*) begin
        selected_valid_comb = 1'b0;
        selected_port_comb = rr_ptr_reg;

        for (search_idx = 0; search_idx < P_NUM_PORTS; search_idx = search_idx + 1) begin
            port_idx = rr_ptr_reg + search_idx;
            if (port_idx >= P_NUM_PORTS) begin
                port_idx = port_idx - P_NUM_PORTS;
            end

            if (!selected_valid_comb && i_event_valid[port_idx]) begin
                selected_valid_comb = 1'b1;
                selected_port_comb = port_idx[LP_PORT_WIDTH-1:0];
            end
        end
    end

    /*
     * 输出选择和 ready 分发。
     */
    always @(*) begin
        o_event_valid = selected_valid_comb;
        o_event_addr = i_event_addr[selected_port_comb];
        o_event_ready = {P_NUM_PORTS{1'b0}};

        if (selected_valid_comb) begin
            o_event_ready[selected_port_comb] = i_event_ready;
        end
    end

    /*
     * 轮询指针只在事件完成握手后前进。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr_reg <= {LP_PORT_WIDTH{1'b0}};
        end else if (i_clear) begin
            rr_ptr_reg <= {LP_PORT_WIDTH{1'b0}};
        end else if (o_event_valid && i_event_ready) begin
            if (selected_port_comb == P_NUM_PORTS - 1) begin
                rr_ptr_reg <= {LP_PORT_WIDTH{1'b0}};
            end else begin
                rr_ptr_reg <= selected_port_comb + 1'b1;
            end
        end
    end

endmodule
