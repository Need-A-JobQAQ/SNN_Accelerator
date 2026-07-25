module aer_event_arbiter #(
    parameter P_NUM_PORTS = 4,
    parameter P_ADDR_WIDTH = 11,
    parameter P_COUNT_WIDTH = 10,
    parameter P_ARB_POLICY = 1
) (
    input wire clk,
    input wire rst_n,
    input wire i_clear,

    input wire [P_NUM_PORTS-1:0] i_event_valid,
    input wire [P_NUM_PORTS-1:0][P_ADDR_WIDTH-1:0] i_event_addr,
    input wire [P_NUM_PORTS-1:0][P_COUNT_WIDTH-1:0] i_event_count,
    output reg [P_NUM_PORTS-1:0] o_event_ready,

    output reg o_event_valid,
    output reg [P_ADDR_WIDTH-1:0] o_event_addr,
    input wire i_event_ready
);

    /*
     * 多路 AER 事件仲裁器。
     * P_ARB_POLICY = 0：固定优先级，低编号端口优先。
     * P_ARB_POLICY = 1：轮询仲裁，避免低编号 core 长期占用输出通道。
     * P_ARB_POLICY = 2：负载感知仲裁，优先服务 FIFO 占用量更高的 core；
     *                  占用量相同时再按轮询顺序打破平局。
     */
    localparam LP_PORT_WIDTH = $clog2(P_NUM_PORTS);

    reg [LP_PORT_WIDTH-1:0] rr_ptr_reg;
    reg [LP_PORT_WIDTH-1:0] selected_port_comb;
    reg selected_valid_comb;
    reg [P_COUNT_WIDTH-1:0] best_count_comb;

    integer search_idx;
    integer port_idx;
    integer fixed_idx;

    /*
     * 仲裁选择逻辑。
     * 这里只决定“选哪一路”，ready 反压和指针更新放到后面的 always 块里。
     */
    always @(*) begin
        selected_valid_comb = 1'b0;
        selected_port_comb = {LP_PORT_WIDTH{1'b0}};
        best_count_comb = {P_COUNT_WIDTH{1'b0}};
        port_idx = 0;

        if (P_ARB_POLICY == 0) begin
            for (fixed_idx = 0; fixed_idx < P_NUM_PORTS; fixed_idx = fixed_idx + 1) begin
                if (!selected_valid_comb && i_event_valid[fixed_idx]) begin
                    selected_valid_comb = 1'b1;
                    selected_port_comb = fixed_idx[LP_PORT_WIDTH-1:0];
                end
            end
        end else begin
            selected_port_comb = rr_ptr_reg;

            for (search_idx = 0; search_idx < P_NUM_PORTS; search_idx = search_idx + 1) begin
                port_idx = rr_ptr_reg + search_idx;
                if (port_idx >= P_NUM_PORTS) begin
                    port_idx = port_idx - P_NUM_PORTS;
                end

                if (i_event_valid[port_idx]) begin
                    if (P_ARB_POLICY == 2) begin
                        if (!selected_valid_comb || i_event_count[port_idx] > best_count_comb) begin
                            selected_valid_comb = 1'b1;
                            selected_port_comb = port_idx[LP_PORT_WIDTH-1:0];
                            best_count_comb = i_event_count[port_idx];
                        end
                    end else if (!selected_valid_comb) begin
                        selected_valid_comb = 1'b1;
                        selected_port_comb = port_idx[LP_PORT_WIDTH-1:0];
                    end
                end
            end
        end
    end

    /*
     * 输出事件和 ready 分发。
     * 只有被选中的端口能看到后级 ready，其余端口继续等待。
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
     * 负载感知模式也更新指针，用它作为同负载情况下的公平性依据。
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
