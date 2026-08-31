`timescale 1ns / 1ps

module GPT_current_bank_2way_arbiter_tb;

    localparam P_ADDR_WIDTH = 4;
    localparam P_DATA_WIDTH = 16;

    reg clk;
    reg rst_n;
    reg clear;
    reg core0_req;
    reg [P_ADDR_WIDTH-1:0] core0_addr;
    reg core1_req;
    reg [P_ADDR_WIDTH-1:0] core1_addr;
    reg ram_rd_valid;
    reg signed [P_DATA_WIDTH-1:0] ram_rd_data;

    wire core0_grant;
    wire core1_grant;
    wire core0_data_valid;
    wire signed [P_DATA_WIDTH-1:0] core0_data;
    wire core1_data_valid;
    wire signed [P_DATA_WIDTH-1:0] core1_data;
    wire ram_rd_en;
    wire [P_ADDR_WIDTH-1:0] ram_rd_addr;

    current_bank_2way_arbiter #(
        .P_ADDR_WIDTH (P_ADDR_WIDTH),
        .P_DATA_WIDTH (P_DATA_WIDTH)
    ) dut (
        .clk                (clk),
        .rst_n              (rst_n),
        .i_clear            (clear),
        .i_core0_req        (core0_req),
        .i_core0_addr       (core0_addr),
        .o_core0_grant      (core0_grant),
        .o_core0_data_valid (core0_data_valid),
        .o_core0_data       (core0_data),
        .i_core1_req        (core1_req),
        .i_core1_addr       (core1_addr),
        .o_core1_grant      (core1_grant),
        .o_core1_data_valid (core1_data_valid),
        .o_core1_data       (core1_data),
        .o_ram_rd_en        (ram_rd_en),
        .o_ram_rd_addr      (ram_rd_addr),
        .i_ram_rd_valid     (ram_rd_valid),
        .i_ram_rd_data      (ram_rd_data)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    /*
     * 简化 RAM 模型：本拍给读使能和地址，下一拍返回 addr+100。
     */
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_rd_valid <= 1'b0;
            ram_rd_data <= {P_DATA_WIDTH{1'b0}};
        end else begin
            ram_rd_valid <= ram_rd_en;
            ram_rd_data <= 16'sd100 + ram_rd_addr;
        end
    end

    task expect_grant;
        input exp_core0;
        input exp_core1;
        input [P_ADDR_WIDTH-1:0] exp_addr;
        begin
            #1;
            if (core0_grant !== exp_core0 || core1_grant !== exp_core1 || ram_rd_addr !== exp_addr) begin
                $display("SIM_FAIL: grant mismatch exp c0=%b c1=%b addr=%0d got c0=%b c1=%b addr=%0d",
                         exp_core0, exp_core1, exp_addr, core0_grant, core1_grant, ram_rd_addr);
                $finish;
            end
        end
    endtask

    task expect_data_now;
        input exp_core;
        input signed [P_DATA_WIDTH-1:0] exp_data;
        begin
            #1;
            if (exp_core == 1'b0) begin
                if (core0_data_valid !== 1'b1 || core0_data !== exp_data || core1_data_valid !== 1'b0) begin
                    $display("SIM_FAIL: core0 data mismatch exp=%0d c0_valid=%b c0_data=%0d c1_valid=%b",
                             exp_data, core0_data_valid, core0_data, core1_data_valid);
                    $finish;
                end
            end else begin
                if (core1_data_valid !== 1'b1 || core1_data !== exp_data || core0_data_valid !== 1'b0) begin
                    $display("SIM_FAIL: core1 data mismatch exp=%0d c1_valid=%b c1_data=%0d c0_valid=%b",
                             exp_data, core1_data_valid, core1_data, core0_data_valid);
                    $finish;
                end
            end
        end
    endtask

    initial begin
        rst_n = 1'b0;
        clear = 1'b0;
        core0_req = 1'b0;
        core1_req = 1'b0;
        core0_addr = 4'd0;
        core1_addr = 4'd0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        /* core0 单独请求，应该直接获得 RAM 读口。 */
        core0_req = 1'b1;
        core0_addr = 4'd3;
        core1_req = 1'b0;
        expect_grant(1'b1, 1'b0, 4'd3);
        @(posedge clk);
        expect_data_now(1'b0, 16'sd103);
        core0_req = 1'b0;
        @(posedge clk);

        /* 上一次服务 core0 后，双请求时优先服务 core1。 */
        core0_req = 1'b1;
        core1_req = 1'b1;
        core0_addr = 4'd4;
        core1_addr = 4'd9;
        expect_grant(1'b0, 1'b1, 4'd9);
        @(posedge clk);
        expect_data_now(1'b1, 16'sd109);

        /* 两路继续保持请求，轮询指针切回 core0。 */
        expect_grant(1'b1, 1'b0, 4'd4);
        @(posedge clk);
        expect_data_now(1'b0, 16'sd104);
        core0_req = 1'b0;
        core1_req = 1'b0;

        $display("SIM_PASS: current_bank_2way_arbiter basic test passed.");
        $finish;
    end

endmodule
