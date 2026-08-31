module current_bank_2way_arbiter #(
    parameter P_ADDR_WIDTH = 10,
    parameter P_DATA_WIDTH = 26
) (
    input wire clk,
    input wire rst_n,
    input wire i_clear,

    input wire i_core0_req,
    input wire [P_ADDR_WIDTH-1:0] i_core0_addr,
    output reg o_core0_grant,
    output reg o_core0_data_valid,
    output reg signed [P_DATA_WIDTH-1:0] o_core0_data,

    input wire i_core1_req,
    input wire [P_ADDR_WIDTH-1:0] i_core1_addr,
    output reg o_core1_grant,
    output reg o_core1_data_valid,
    output reg signed [P_DATA_WIDTH-1:0] o_core1_data,

    output reg o_ram_rd_en,
    output reg [P_ADDR_WIDTH-1:0] o_ram_rd_addr,
    input wire i_ram_rd_valid,
    input wire signed [P_DATA_WIDTH-1:0] i_ram_rd_data
);

    /*
     * 两路 current RAM 读请求仲裁器。
     * 本拍选择一个 core 的读请求送入 RAM，下一拍把 RAM 返回数据送回被选中的 core。
     */
    reg rr_ptr_reg;
    reg grant_core0_comb;
    reg grant_core1_comb;
    reg selected_core_dly_reg;
    reg selected_valid_dly_reg;

    /*
     * 组合仲裁逻辑。
     * 两路同时请求时按轮询指针选择；只有一路请求时直接服务该路。
     */
    always @(*) begin
        grant_core0_comb = 1'b0;
        grant_core1_comb = 1'b0;

        if (i_core0_req && i_core1_req) begin
            if (rr_ptr_reg == 1'b0) begin
                grant_core0_comb = 1'b1;
            end else begin
                grant_core1_comb = 1'b1;
            end
        end else if (i_core0_req) begin
            grant_core0_comb = 1'b1;
        end else if (i_core1_req) begin
            grant_core1_comb = 1'b1;
        end
    end

    /*
     * RAM 读口输出。
     * grant 同时表示该 core 的请求在本拍被接受。
     */
    always @(*) begin
        o_core0_grant = grant_core0_comb;
        o_core1_grant = grant_core1_comb;
        o_ram_rd_en = grant_core0_comb || grant_core1_comb;
        o_ram_rd_addr = {P_ADDR_WIDTH{1'b0}};

        if (grant_core0_comb) begin
            o_ram_rd_addr = i_core0_addr;
        end else if (grant_core1_comb) begin
            o_ram_rd_addr = i_core1_addr;
        end
    end

    /*
     * 轮询指针更新。
     * 只有真正向 RAM 发出读请求后才切换优先级。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr_reg <= 1'b0;
        end else if (i_clear) begin
            rr_ptr_reg <= 1'b0;
        end else if (o_ram_rd_en) begin
            if (grant_core0_comb) begin
                rr_ptr_reg <= 1'b1;
            end else begin
                rr_ptr_reg <= 1'b0;
            end
        end
    end

    /*
     * 锁存本拍被服务的 core，用于下一拍数据返回分发。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            selected_core_dly_reg <= 1'b0;
            selected_valid_dly_reg <= 1'b0;
        end else if (i_clear) begin
            selected_core_dly_reg <= 1'b0;
            selected_valid_dly_reg <= 1'b0;
        end else begin
            selected_valid_dly_reg <= o_ram_rd_en;
            if (grant_core1_comb) begin
                selected_core_dly_reg <= 1'b1;
            end else begin
                selected_core_dly_reg <= 1'b0;
            end
        end
    end

    /*
     * RAM 返回数据分发。
     * 选择信息已经打一拍，RAM 返回数据到达时直接组合分发给对应 core。
     * 这样不会额外增加一拍延迟，能和 core 内部膜电位 RAM 的读延迟对齐。
     */
    always @(*) begin
        o_core0_data_valid = 1'b0;
        o_core1_data_valid = 1'b0;
        o_core0_data = {P_DATA_WIDTH{1'b0}};
        o_core1_data = {P_DATA_WIDTH{1'b0}};

        if (selected_valid_dly_reg && i_ram_rd_valid) begin
            if (selected_core_dly_reg) begin
                o_core1_data_valid = 1'b1;
                o_core1_data = i_ram_rd_data;
            end else begin
                o_core0_data_valid = 1'b1;
                o_core0_data = i_ram_rd_data;
            end
        end
    end

endmodule
