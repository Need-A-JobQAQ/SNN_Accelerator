module spike_vector_to_aer #(
    parameter P_NUM_SPIKES = 1568,
    parameter P_ADDR_WIDTH = $clog2(P_NUM_SPIKES)
) (
    input wire clk,
    input wire rst_n,
    input wire i_start,
    input wire [P_NUM_SPIKES-1:0] i_spike_vector,
    input wire i_event_ready,

    output reg o_event_valid,
    output reg [P_ADDR_WIDTH-1:0] o_event_addr,
    output reg o_frame_done,
    output reg o_busy
);

    /*
     * 将一个完整的脉冲向量转换为 AER 地址事件流。
     * 工程中约定最高位对应逻辑第 0 个位置，因此本模块从高位向低位扫描。
     */
    localparam [1:0] S_IDLE = 2'b00;
    localparam [1:0] S_SCAN = 2'b01;
    localparam [1:0] S_DONE = 2'b10;

    reg [1:0] current_state_reg;
    reg [1:0] next_state_reg;

    reg [P_NUM_SPIKES-1:0] latched_spike_vector_reg;
    reg [P_ADDR_WIDTH-1:0] scan_idx_reg;

    wire current_spike_w;
    wire event_accepted_w;
    wire scan_last_w;
    wire scan_advance_w;

    assign current_spike_w = latched_spike_vector_reg[P_NUM_SPIKES - 1 - scan_idx_reg];
    assign event_accepted_w = current_spike_w && i_event_ready; //握手机制
    assign scan_last_w = (scan_idx_reg == P_NUM_SPIKES - 1);
    assign scan_advance_w = (!current_spike_w) || event_accepted_w;

    /*
     * 状态转移逻辑。
     * 遇到脉冲且后级未 ready 时，停在当前地址等待事件被接收。
     */
    always @(*) begin
        next_state_reg = current_state_reg;

        case (current_state_reg)
            S_IDLE: begin
                if (i_start) begin
                    next_state_reg = S_SCAN;
                end
            end

            S_SCAN: begin
                if (scan_last_w && scan_advance_w) begin
                    next_state_reg = S_DONE;
                end else begin
                    next_state_reg = S_SCAN;
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
     * o_event_addr 是逻辑地址：0 表示输入向量最高位，P_NUM_SPIKES-1 表示输入向量最低位。
     */
    always @(*) begin
        o_event_valid = 1'b0;
        o_event_addr = scan_idx_reg;
        o_frame_done = 1'b0;
        o_busy = (current_state_reg != S_IDLE);

        if (current_state_reg == S_SCAN && current_spike_w) begin
            o_event_valid = 1'b1;
        end

        if (current_state_reg == S_DONE) begin
            o_frame_done = 1'b1;
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
     * 扫描寄存器。
     * 启动时锁存整帧脉冲，之后按逻辑地址从 0 到 P_NUM_SPIKES-1 扫描。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latched_spike_vector_reg <= {P_NUM_SPIKES{1'b0}};
            scan_idx_reg <= {P_ADDR_WIDTH{1'b0}};
        end else begin
            if (current_state_reg == S_IDLE && next_state_reg == S_SCAN) begin
                latched_spike_vector_reg <= i_spike_vector;
                scan_idx_reg <= {P_ADDR_WIDTH{1'b0}};
            end else if (current_state_reg == S_SCAN && scan_advance_w && !scan_last_w) begin
                scan_idx_reg <= scan_idx_reg + 1'b1;
            end else if (current_state_reg == S_DONE) begin
                scan_idx_reg <= {P_ADDR_WIDTH{1'b0}};
            end
        end
    end

endmodule
