module conv_lif_sparse_core #(
    parameter P_GLOBAL_NUM_NEURONS = 1568,
    parameter P_CORE_START_ADDR = 0,
    parameter P_CORE_NUM_NEURONS = 392,
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_INPUT_HEIGHT = 28,
    parameter P_INPUT_WIDTH = 28,
    parameter P_KERNEL_SIZE = 3,
    parameter P_PADDING = 1,
    parameter P_CORE_ID = 0,
    parameter P_CORE_MAPPING_MODE = 0,
    parameter P_NEURON_VALUE_TOTAL_BITS = 26,
    parameter P_NEURON_VALUE_FRAC_BITS = 12,
    parameter P_SKIP_THRESHOLD_SHIFT = 5
) (
    input wire clk,
    input wire rst_n,
    input wire i_enable_core,
    input wire [P_NUM_INPUT_PIXELS-1:0] i_input_spike_vector,
    input wire signed [P_GLOBAL_NUM_NEURONS-1:0][P_NEURON_VALUE_TOTAL_BITS-1:0] i_all_currents_I,

    output reg [P_CORE_NUM_NEURONS-1:0] o_core_spikes_out,
    output reg o_core_done,
    output reg o_event_valid,
    output reg [$clog2(P_GLOBAL_NUM_NEURONS)-1:0] o_event_addr,
    output wire o_core_ready,
    output reg [31:0] o_skip_count,
    output reg [31:0] o_update_count
);

    /*
     * 一个局部卷积 LIF 神经核心。
     * 每个 core 只处理全局神经元地址空间中的一段连续地址，
     * 并独立维护自己的膜电位 RAM 和 active bitmap。
     */
    localparam LP_LOCAL_ADDR_WIDTH = $clog2(P_CORE_NUM_NEURONS);
    localparam LP_GLOBAL_ADDR_WIDTH = $clog2(P_GLOBAL_NUM_NEURONS);
    localparam LP_COUNT_WIDTH = $clog2(P_CORE_NUM_NEURONS + 1);
    localparam BRAM_READ_LATENCY = 1;

    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] LP_V_THRESHOLD_FIXED =
        (1'b1 << P_NEURON_VALUE_FRAC_BITS);
    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] LP_V_RESET_FIXED =
        {P_NEURON_VALUE_TOTAL_BITS{1'b0}};
    localparam signed [P_NEURON_VALUE_TOTAL_BITS-1:0] LP_SKIP_THRESHOLD_FIXED =
        (P_NEURON_VALUE_FRAC_BITS > P_SKIP_THRESHOLD_SHIFT) ?
        (1'b1 << (P_NEURON_VALUE_FRAC_BITS - P_SKIP_THRESHOLD_SHIFT)) :
        {{(P_NEURON_VALUE_TOTAL_BITS-1){1'b0}}, 1'b1};

    localparam [2:0] S_CLEAR = 3'b000;
    localparam [2:0] S_IDLE = 3'b001;
    localparam [2:0] S_PROCESSING = 3'b010;
    localparam [2:0] S_FLUSHING = 3'b011;
    localparam [2:0] S_DONE = 3'b100;

    reg [2:0] current_state_reg;
    reg [2:0] next_state_reg;

    reg [P_NUM_INPUT_PIXELS-1:0] latched_input_spikes_reg;
    reg [P_CORE_NUM_NEURONS-1:0] active_state_bitmap_reg;
    reg [LP_LOCAL_ADDR_WIDTH-1:0] clear_addr_reg;
    reg [LP_COUNT_WIDTH-1:0] issued_addr_count_reg;
    reg [LP_LOCAL_ADDR_WIDTH-1:0] addr_pipeline_reg [BRAM_READ_LATENCY-1:0];
    reg valid_pipeline_reg [BRAM_READ_LATENCY-1:0];
    reg pipeline_busy_comb;

    wire scan_addr_valid_w;
    wire [LP_LOCAL_ADDR_WIDTH-1:0] scan_local_addr_w;
    wire [LP_GLOBAL_ADDR_WIDTH-1:0] scan_global_addr_w;
    wire scan_rf_active_w;
    wire scan_state_active_w;
    wire scan_need_update_w;
    wire pipeline_busy_w;

    wire clear_membrane_en_w;
    wire membrane_ram_write_en_w;
    wire [LP_LOCAL_ADDR_WIDTH-1:0] membrane_ram_write_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] membrane_ram_write_data_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] membrane_ram_read_data_w;
    wire lif_membrane_write_en_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] lif_membrane_write_data_w;

    wire [LP_LOCAL_ADDR_WIDTH-1:0] process_local_addr_w;
    wire [LP_GLOBAL_ADDR_WIDTH-1:0] process_global_addr_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_input_current_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_membrane_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_diff_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_delta_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_candidate_w;
    wire signed [P_NEURON_VALUE_TOTAL_BITS-1:0] process_abs_candidate_w;
    wire process_spike_w;
    wire process_next_active_w;
    wire [LP_GLOBAL_ADDR_WIDTH-1:0] process_event_addr_w;

    integer reset_idx;
    integer pipe_idx;
    integer busy_idx;

    assign o_core_ready = (current_state_reg == S_IDLE);
    assign scan_addr_valid_w = (current_state_reg == S_PROCESSING) &&
                               (issued_addr_count_reg < P_CORE_NUM_NEURONS);
    assign scan_local_addr_w = issued_addr_count_reg[LP_LOCAL_ADDR_WIDTH-1:0];
    assign scan_global_addr_w = local_to_global_addr(scan_local_addr_w);
    assign scan_rf_active_w = receptive_field_active(latched_input_spikes_reg, scan_global_addr_w);
    assign scan_state_active_w = active_state_bitmap_reg[scan_local_addr_w];
    assign scan_need_update_w = scan_addr_valid_w && (scan_rf_active_w || scan_state_active_w);

    assign pipeline_busy_w = pipeline_busy_comb;
    assign process_local_addr_w = addr_pipeline_reg[BRAM_READ_LATENCY-1];
    assign process_global_addr_w = local_to_global_addr(process_local_addr_w);
    assign clear_membrane_en_w = (current_state_reg == S_CLEAR);
    assign membrane_ram_write_en_w = clear_membrane_en_w || lif_membrane_write_en_w;
    assign membrane_ram_write_addr_w = clear_membrane_en_w ? clear_addr_reg : process_local_addr_w;
    assign membrane_ram_write_data_w = clear_membrane_en_w ? {P_NEURON_VALUE_TOTAL_BITS{1'b0}} :
                                       lif_membrane_write_data_w;
    assign lif_membrane_write_en_w = ((current_state_reg == S_PROCESSING) ||
                                      (current_state_reg == S_FLUSHING)) &&
                                     valid_pipeline_reg[BRAM_READ_LATENCY-1];
    assign lif_membrane_write_data_w = process_spike_w ? LP_V_RESET_FIXED : process_candidate_w;
    assign process_input_current_w = i_all_currents_I[process_global_addr_w];
    assign process_membrane_w = membrane_ram_read_data_w;

    simple_dual_port_ram #(
        .P_DATA_WIDTH   (P_NEURON_VALUE_TOTAL_BITS),
        .P_ADDR_WIDTH   (LP_LOCAL_ADDR_WIDTH),
        .P_DEPTH        (P_CORE_NUM_NEURONS)
    ) u_membrane_potential_ram (
        .clk            (clk),
        .i_write_en     (membrane_ram_write_en_w),
        .i_write_addr   (membrane_ram_write_addr_w),
        .i_write_data   (membrane_ram_write_data_w),
        .i_read_en      (scan_need_update_w),
        .i_read_addr    (scan_local_addr_w),
        .o_read_data    (membrane_ram_read_data_w)
    );

    function get_input_spike;
        input [P_NUM_INPUT_PIXELS-1:0] spikes;
        input integer row;
        input integer col;
        integer flat_idx;
        begin
            if (row < 0 || row >= P_INPUT_HEIGHT || col < 0 || col >= P_INPUT_WIDTH) begin
                get_input_spike = 1'b0;
            end else begin
                flat_idx = (row * P_INPUT_WIDTH) + col;
                get_input_spike = spikes[P_NUM_INPUT_PIXELS - 1 - flat_idx];
            end
        end
    endfunction

    function receptive_field_active;
        input [P_NUM_INPUT_PIXELS-1:0] spikes;
        input [LP_GLOBAL_ADDR_WIDTH-1:0] global_addr;
        integer local_addr;
        integer spatial_idx;
        integer out_row;
        integer out_col;
        integer kernel_row;
        integer kernel_col;
        begin
            receptive_field_active = 1'b0;
            local_addr = global_addr % P_NUM_INPUT_PIXELS;
            spatial_idx = P_NUM_INPUT_PIXELS - 1 - local_addr;
            out_row = spatial_idx / P_INPUT_WIDTH;
            out_col = spatial_idx % P_INPUT_WIDTH;

            for (kernel_row = 0; kernel_row < P_KERNEL_SIZE; kernel_row = kernel_row + 1) begin
                for (kernel_col = 0; kernel_col < P_KERNEL_SIZE; kernel_col = kernel_col + 1) begin
                    if (get_input_spike(spikes,
                                        out_row + kernel_row - P_PADDING,
                                        out_col + kernel_col - P_PADDING)) begin
                        receptive_field_active = 1'b1;
                    end
                end
            end
        end
    endfunction

    /*
     * core 本地地址到全局神经元地址的映射。
     * mode 0：连续地址划分，保持原来的 core0/core1/core2/core3 连续分段方式。
     * mode 1：2x2 空间块交错划分，按(row[0], col[0])把相邻空间位置分散到4个core。
     * mode 2：7x7 tile 空间块交错划分，按(tile_row + tile_col) % 4分配tile。
     */
    function [LP_GLOBAL_ADDR_WIDTH-1:0] local_to_global_addr;
        input [LP_LOCAL_ADDR_WIDTH-1:0] local_addr;
        integer positions_per_core;
        integer tile_size;
        integer tile_pixels;
        integer tile_cols;
        integer local_channel;
        integer local_spatial_idx;
        integer local_tile_idx;
        integer in_tile_idx;
        integer local_row;
        integer local_col;
        integer tile_row;
        integer tile_col;
        integer in_tile_row;
        integer in_tile_col;
        integer global_row;
        integer global_col;
        integer global_spatial_idx;
        integer global_local_addr;
        begin
            if (P_CORE_MAPPING_MODE == 1) begin
                positions_per_core = (P_INPUT_HEIGHT / 2) * (P_INPUT_WIDTH / 2);
                local_channel = local_addr / positions_per_core;
                local_spatial_idx = local_addr % positions_per_core;
                local_row = local_spatial_idx / (P_INPUT_WIDTH / 2);
                local_col = local_spatial_idx % (P_INPUT_WIDTH / 2);
                global_row = (local_row * 2) + (P_CORE_ID / 2);
                global_col = (local_col * 2) + (P_CORE_ID % 2);
                global_spatial_idx = (global_row * P_INPUT_WIDTH) + global_col;
                global_local_addr = P_NUM_INPUT_PIXELS - 1 - global_spatial_idx;
                local_to_global_addr = (local_channel * P_NUM_INPUT_PIXELS) + global_local_addr;
            end else if (P_CORE_MAPPING_MODE == 2) begin
                /*
                 * 7x7 tile映射：
                 * 28x28特征图被切成4x4个tile，每个tile包含49个像素。
                 * local_addr先拆成：通道号、core内部第几个tile、tile内部第几个像素。
                 */
                tile_size = 7;
                tile_pixels = tile_size * tile_size;
                tile_cols = P_INPUT_WIDTH / tile_size;
                positions_per_core = P_CORE_NUM_NEURONS / (P_GLOBAL_NUM_NEURONS / P_NUM_INPUT_PIXELS);
                local_channel = local_addr / positions_per_core;
                local_spatial_idx = local_addr % positions_per_core;
                local_tile_idx = local_spatial_idx / tile_pixels;
                in_tile_idx = local_spatial_idx % tile_pixels;

                /*
                 * 每个core有4个tile。
                 * 采用core_id = (tile_row + tile_col) % 4的反向映射：
                 * local_tile_idx决定tile_row，core_id决定对应的tile_col。
                 */
                tile_row = local_tile_idx;
                tile_col = (P_CORE_ID + tile_cols - tile_row) % tile_cols;
                in_tile_row = in_tile_idx / tile_size;
                in_tile_col = in_tile_idx % tile_size;
                global_row = (tile_row * tile_size) + in_tile_row;
                global_col = (tile_col * tile_size) + in_tile_col;
                global_spatial_idx = (global_row * P_INPUT_WIDTH) + global_col;
                global_local_addr = P_NUM_INPUT_PIXELS - 1 - global_spatial_idx;
                local_to_global_addr = (local_channel * P_NUM_INPUT_PIXELS) + global_local_addr;
            end else begin
                local_to_global_addr = P_CORE_START_ADDR + local_addr;
            end
        end
    endfunction

    assign process_diff_w = process_input_current_w - process_membrane_w;
    assign process_delta_w = process_diff_w >>> 1;
    assign process_candidate_w = process_membrane_w + process_delta_w;
    assign process_spike_w = (process_candidate_w >= LP_V_THRESHOLD_FIXED);
    assign process_abs_candidate_w = process_candidate_w[P_NEURON_VALUE_TOTAL_BITS-1] ?
                                     -process_candidate_w : process_candidate_w;
    assign process_next_active_w = process_spike_w ? 1'b0 :
                                   (process_abs_candidate_w >= LP_SKIP_THRESHOLD_FIXED);
    assign process_event_addr_w = (P_GLOBAL_NUM_NEURONS - 1) - process_global_addr_w;

    always @(*) begin
        pipeline_busy_comb = 1'b0;
        for (busy_idx = 0; busy_idx < BRAM_READ_LATENCY; busy_idx = busy_idx + 1) begin
            pipeline_busy_comb = pipeline_busy_comb || valid_pipeline_reg[busy_idx];
        end
    end

    always @(*) begin
        next_state_reg = current_state_reg;
        case (current_state_reg)
            S_CLEAR: begin
                if (clear_addr_reg == P_CORE_NUM_NEURONS - 1) begin
                    next_state_reg = S_IDLE;
                end
            end
            S_IDLE: begin
                if (i_enable_core) begin
                    next_state_reg = S_PROCESSING;
                end
            end
            S_PROCESSING: begin
                if (issued_addr_count_reg == P_CORE_NUM_NEURONS) begin
                    next_state_reg = S_FLUSHING;
                end
            end
            S_FLUSHING: begin
                if (!pipeline_busy_w) begin
                    next_state_reg = S_DONE;
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

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state_reg <= S_CLEAR;
        end else begin
            current_state_reg <= next_state_reg;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clear_addr_reg <= {LP_LOCAL_ADDR_WIDTH{1'b0}};
        end else if (current_state_reg == S_CLEAR) begin
            if (clear_addr_reg != P_CORE_NUM_NEURONS - 1) begin
                clear_addr_reg <= clear_addr_reg + 1'b1;
            end
        end else begin
            clear_addr_reg <= {LP_LOCAL_ADDR_WIDTH{1'b0}};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latched_input_spikes_reg <= {P_NUM_INPUT_PIXELS{1'b0}};
        end else if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
            latched_input_spikes_reg <= i_input_spike_vector;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            issued_addr_count_reg <= {LP_COUNT_WIDTH{1'b0}};
            for (pipe_idx = 0; pipe_idx < BRAM_READ_LATENCY; pipe_idx = pipe_idx + 1) begin
                addr_pipeline_reg[pipe_idx] <= {LP_LOCAL_ADDR_WIDTH{1'b0}};
                valid_pipeline_reg[pipe_idx] <= 1'b0;
            end
        end else begin
            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
                issued_addr_count_reg <= {LP_COUNT_WIDTH{1'b0}};
                for (pipe_idx = 0; pipe_idx < BRAM_READ_LATENCY; pipe_idx = pipe_idx + 1) begin
                    addr_pipeline_reg[pipe_idx] <= {LP_LOCAL_ADDR_WIDTH{1'b0}};
                    valid_pipeline_reg[pipe_idx] <= 1'b0;
                end
            end else if (current_state_reg == S_PROCESSING || current_state_reg == S_FLUSHING) begin
                for (pipe_idx = BRAM_READ_LATENCY - 1; pipe_idx > 0; pipe_idx = pipe_idx - 1) begin
                    addr_pipeline_reg[pipe_idx] <= addr_pipeline_reg[pipe_idx-1];
                    valid_pipeline_reg[pipe_idx] <= valid_pipeline_reg[pipe_idx-1];
                end

                valid_pipeline_reg[0] <= scan_need_update_w;
                if (scan_need_update_w) begin
                    addr_pipeline_reg[0] <= scan_local_addr_w;
                end else begin
                    addr_pipeline_reg[0] <= {LP_LOCAL_ADDR_WIDTH{1'b0}};
                end

                if (scan_addr_valid_w) begin
                    issued_addr_count_reg <= issued_addr_count_reg + 1'b1;
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_state_bitmap_reg <= {P_CORE_NUM_NEURONS{1'b0}};
        end else if (current_state_reg == S_CLEAR) begin
            active_state_bitmap_reg <= {P_CORE_NUM_NEURONS{1'b0}};
        end else if (lif_membrane_write_en_w) begin
            active_state_bitmap_reg[process_local_addr_w] <= process_next_active_w;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_core_spikes_out <= {P_CORE_NUM_NEURONS{1'b0}};
            o_event_valid <= 1'b0;
            o_event_addr <= {LP_GLOBAL_ADDR_WIDTH{1'b0}};
            o_skip_count <= 32'd0;
            o_update_count <= 32'd0;
        end else begin
            o_event_valid <= 1'b0;
            if (current_state_reg == S_IDLE && next_state_reg == S_PROCESSING) begin
                o_core_spikes_out <= {P_CORE_NUM_NEURONS{1'b0}};
                o_event_addr <= {LP_GLOBAL_ADDR_WIDTH{1'b0}};
                o_skip_count <= 32'd0;
                o_update_count <= 32'd0;
            end else begin
                if (scan_addr_valid_w && !scan_need_update_w) begin
                    o_skip_count <= o_skip_count + 1'b1;
                end

                if (lif_membrane_write_en_w) begin
                    o_update_count <= o_update_count + 1'b1;
                    if (process_spike_w) begin
                        o_core_spikes_out[process_local_addr_w] <= 1'b1;
                        o_event_valid <= 1'b1;
                        o_event_addr <= process_event_addr_w;
                    end else begin
                        o_core_spikes_out[process_local_addr_w] <= 1'b0;
                    end
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_core_done <= 1'b0;
        end else begin
            o_core_done <= (current_state_reg == S_DONE);
        end
    end

endmodule
