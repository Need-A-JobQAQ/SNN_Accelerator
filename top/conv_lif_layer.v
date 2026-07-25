module conv_lif_layer #(
    parameter P_NUM_NEURONS = 1568,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter P_NEURON_VALUE_FRAC_BITS = 12
) (
    input wire clk,
    input wire rst_n,
    input wire i_enable_layer,
    input wire signed [P_NUM_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] i_all_currents_I,

    output reg [P_NUM_NEURONS-1:0] o_all_spikes_out,
    output reg o_all_spikes_valid,
    output reg o_event_valid,
    output reg [$clog2(P_NUM_NEURONS)-1:0] o_event_addr,
    output reg o_event_frame_done,
    output wire o_layer_ready
);

    /*
     * 该模块用一套 LIF 计算通路分时处理所有卷积输出神经元。
     * 膜电位状态按同步 RAM 形式保存，读出数据天然延迟 1 拍。
     */
    localparam LP_ADDR_WIDTH = $clog2(P_NUM_NEURONS);       //n位二进制数能表示的范围是(2^n - 1),想要表示2^n-1大小的数，需要$clog2(2^n-1 +1)的位数。LP_ADDR_WIDTH可表示到P_NUM_NEURONS-1
    localparam LP_COUNT_WIDTH = $clog2(P_NUM_NEURONS + 1);  //LP_COUNT_WIDTH可表示到P_NUM_NEURONS
    localparam BRAM_READ_LATENCY = 1;

    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] LP_V_THRESHOLD_FIXED = (1'b1 << P_NEURON_VALUE_FRAC_BITS);
    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] LP_V_RESET_FIXED = {P_NEURON_VALUE_TOTAL_BITS{1'b0}};

    localparam [2:0] S_CLEAR = 3'b000;
    localparam [2:0] S_IDLE = 3'b001;
    localparam [2:0] S_PROCESSING = 3'b010;
    localparam [2:0] S_FLUSHING = 3'b011;
    localparam [2:0] S_DONE = 3'b100;

    reg [2:0] current_state_reg;
    reg [2:0] next_state_reg;

    reg signed [P_NUM_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] latched_currents_reg;

    reg [LP_ADDR_WIDTH-1:0] clear_addr_reg;
    reg [LP_COUNT_WIDTH-1:0] issued_read_count_reg;
    reg [LP_COUNT_WIDTH-1:0] processed_count_reg;

    reg [LP_ADDR_WIDTH-1:0] addr_pipeline_reg [BRAM_READ_LATENCY-1:0];
    reg valid_pipeline_reg [BRAM_READ_LATENCY-1:0];

    wire issue_read_en_w;
    wire [LP_ADDR_WIDTH-1:0] issue_addr_w;
    wire [LP_ADDR_WIDTH-1:0] process_addr_w;
    wire clear_membrane_en_w;
    wire membrane_ram_write_en_w;
    wire [LP_ADDR_WIDTH-1:0] membrane_ram_write_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] membrane_ram_write_data_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] membrane_ram_read_data_w;
    wire lif_membrane_write_en_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] lif_membrane_write_data_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_input_current_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_membrane_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_diff_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_delta_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_candidate_w;
    wire process_spike_w;
    wire [LP_ADDR_WIDTH-1:0] process_event_addr_w;

    integer current_reset_idx;
    integer pipe_idx;

    assign o_layer_ready = (current_state_reg == S_IDLE);
    assign issue_read_en_w = (current_state_reg == S_PROCESSING) && (issued_read_count_reg < P_NUM_NEURONS);
    assign issue_addr_w = issued_read_count_reg[LP_ADDR_WIDTH-1:0];
    assign process_addr_w = addr_pipeline_reg[BRAM_READ_LATENCY-1];
    assign clear_membrane_en_w = (current_state_reg == S_CLEAR);
    assign membrane_ram_write_en_w = clear_membrane_en_w || lif_membrane_write_en_w;
    assign membrane_ram_write_addr_w = clear_membrane_en_w ? clear_addr_reg : process_addr_w;
    assign membrane_ram_write_data_w = clear_membrane_en_w ? {P_NEURON_VALUE_TOTAL_BITS{1'b0}} : lif_membrane_write_data_w;
    assign lif_membrane_write_en_w = ((current_state_reg == S_PROCESSING) || (current_state_reg == S_FLUSHING)) &&
                                     valid_pipeline_reg[BRAM_READ_LATENCY-1];
    assign lif_membrane_write_data_w = process_spike_w ? LP_V_RESET_FIXED : process_candidate_w;
    assign process_input_current_w = latched_currents_reg[process_addr_w];
    assign process_membrane_w = membrane_ram_read_data_w;

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_ADDR_WIDTH),
        .P_DEPTH        (P_NUM_NEURONS)
    ) u_membrane_potential_ram (
        .clk            (clk),
        .i_write_en     (membrane_ram_write_en_w),
        .i_write_addr   (membrane_ram_write_addr_w),
        .i_write_data   (membrane_ram_write_data_w),
        .i_read_en      (issue_read_en_w),
        .i_read_addr    (issue_addr_w),
        .o_read_data    (membrane_ram_read_data_w)
    );

    /*
     * 这里的 tau 固定为 2，因此除以 tau 可以用算术右移 1 位实现。
     * 计算公式对应：v = v + (I - v) / tau。
     */
    assign process_diff_w = process_input_current_w - process_membrane_w;
    assign process_delta_w = process_diff_w >>> 1;
    assign process_candidate_w = process_membrane_w + process_delta_w;
    assign process_spike_w = (process_candidate_w >= LP_V_THRESHOLD_FIXED);
    /*
     * 工程约定 AER 逻辑地址 0 对应脉冲向量最高位。
     * 本模块处理地址 process_addr_w 对应实际 bit 下标，因此这里要做一次反向映射。
     */
    assign process_event_addr_w = (P_NUM_NEURONS - 1) - process_addr_w;

    always @(*) begin
        next_state_reg = current_state_reg;

        case (current_state_reg)
            S_CLEAR: begin
                if (clear_addr_reg == P_NUM_NEURONS - 1) begin
                    next_state_reg = S_IDLE;
                end else begin
                    next_state_reg = S_CLEAR;
                end
            end

            S_IDLE: begin
                if (i_enable_layer) begin
                    next_state_reg = S_PROCESSING;
                end
            end

            S_PROCESSING: begin
                if (issued_read_count_reg == P_NUM_NEURONS) begin
                    next_state_reg = S_FLUSHING;
                end else begin
                    next_state_reg = S_PROCESSING;
                end
            end

            S_FLUSHING: begin
                if (processed_count_reg == P_NUM_NEURONS) begin
                    next_state_reg = S_DONE;
                end else begin
                    next_state_reg = S_FLUSHING;
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
     * 状态寄存器单独放置，和上面的组合逻辑一起构成两段式状态机。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state_reg <= S_CLEAR;
        end else begin
            current_state_reg <= next_state_reg;
        end
    end

    /*
     * BRAM 不能像寄存器阵列一样异步全清零。
     * 复位后进入 S_CLEAR 状态，逐地址写 0，完成后才允许开始正常计算。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clear_addr_reg <= {LP_ADDR_WIDTH{1'b0}};
        end else if (current_state_reg == S_CLEAR) begin
            if (clear_addr_reg != P_NUM_NEURONS - 1) begin
                clear_addr_reg <= clear_addr_reg + 1'b1;
            end
        end else begin
            clear_addr_reg <= {LP_ADDR_WIDTH{1'b0}};
        end
    end

    /*
     * 每次启动一帧 LIF 计算时，把卷积层输出电流锁存下来。
     * 后续流水线只读锁存值，避免输入总线在处理中变化造成结果不稳定。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (current_reset_idx = 0; current_reset_idx < P_NUM_NEURONS; current_reset_idx = current_reset_idx + 1) begin
                latched_currents_reg[current_reset_idx] <= {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
            end
        end else if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
            latched_currents_reg <= i_all_currents_I;
        end
    end

    /*
     * 读地址发射流水线。
     * 同步 RAM 读数据本身延迟 1 拍，因此这里只需要把地址和 valid 同样延迟 1 拍。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            issued_read_count_reg <= {LP_COUNT_WIDTH{1'b0}};

            for (pipe_idx = 0; pipe_idx < BRAM_READ_LATENCY; pipe_idx = pipe_idx + 1) begin
                addr_pipeline_reg[pipe_idx] <= {LP_ADDR_WIDTH{1'b0}};
                valid_pipeline_reg[pipe_idx] <= 1'b0;
            end
        end else begin
            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
                issued_read_count_reg <= {LP_COUNT_WIDTH{1'b0}};

                for (pipe_idx = 0; pipe_idx < BRAM_READ_LATENCY; pipe_idx = pipe_idx + 1) begin
                    addr_pipeline_reg[pipe_idx] <= {LP_ADDR_WIDTH{1'b0}};
                    valid_pipeline_reg[pipe_idx] <= 1'b0;
                end
            end else if (current_state_reg == S_PROCESSING || current_state_reg == S_FLUSHING) begin
                for (pipe_idx = BRAM_READ_LATENCY - 1; pipe_idx > 0; pipe_idx = pipe_idx - 1) begin
                    addr_pipeline_reg[pipe_idx] <= addr_pipeline_reg[pipe_idx-1];
                    valid_pipeline_reg[pipe_idx] <= valid_pipeline_reg[pipe_idx-1];
                end

                valid_pipeline_reg[0] <= issue_read_en_w;
                if (issue_read_en_w) begin
                    addr_pipeline_reg[0] <= issue_addr_w;
                    issued_read_count_reg <= issued_read_count_reg + 1'b1;
                end else begin
                    addr_pipeline_reg[0] <= {LP_ADDR_WIDTH{1'b0}};
                end
            end
        end
    end

    /*
     * LIF 结果写回。
     * 流水线尾部 valid 有效时，只更新输出脉冲和完成计数。
     * 膜电位 RAM 的写回由 simple_dual_port_ram 的写端口完成。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            processed_count_reg <= {LP_COUNT_WIDTH{1'b0}};
            o_all_spikes_out <= {P_NUM_NEURONS{1'b0}};
            o_event_valid <= 1'b0;
            o_event_addr <= {LP_ADDR_WIDTH{1'b0}};
        end else begin
            o_event_valid <= 1'b0;

            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
                processed_count_reg <= {LP_COUNT_WIDTH{1'b0}};
                o_all_spikes_out <= {P_NUM_NEURONS{1'b0}};
                o_event_addr <= {LP_ADDR_WIDTH{1'b0}};
            end else if ((current_state_reg == S_PROCESSING || current_state_reg == S_FLUSHING) &&
                         valid_pipeline_reg[BRAM_READ_LATENCY-1]) begin
                if (process_spike_w) begin
                    o_all_spikes_out[process_addr_w] <= 1'b1;
                    o_event_valid <= 1'b1;
                    o_event_addr <= process_event_addr_w;
                end else begin
                    o_all_spikes_out[process_addr_w] <= 1'b0;
                end

                processed_count_reg <= processed_count_reg + 1'b1;
            end
        end
    end

    /*
     * 输出有效信号只在 S_DONE 状态拉高一个周期。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_all_spikes_valid <= 1'b0;
            o_event_frame_done <= 1'b0;
        end else begin
            o_all_spikes_valid <= (current_state_reg == S_DONE);
            o_event_frame_done <= (current_state_reg == S_DONE);
        end
    end

endmodule
