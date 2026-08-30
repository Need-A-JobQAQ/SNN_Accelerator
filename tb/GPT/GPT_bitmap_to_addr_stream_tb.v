`timescale 1ns / 1ps

module GPT_bitmap_to_addr_stream_tb;

    localparam P_BITMAP_WIDTH = 16;
    localparam P_ADDR_WIDTH = 4;

    reg clk;
    reg rst_n;
    reg i_start;
    reg [P_BITMAP_WIDTH-1:0] i_active_bitmap;
    reg i_addr_ready;

    wire o_addr_valid;
    wire [P_ADDR_WIDTH-1:0] o_addr;
    wire o_done;
    wire o_busy;
    wire [31:0] o_active_count;

    integer recv_count;
    integer error_count;

    bitmap_to_addr_stream #(
        .P_BITMAP_WIDTH(P_BITMAP_WIDTH),
        .P_ADDR_WIDTH(P_ADDR_WIDTH)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .i_start(i_start),
        .i_active_bitmap(i_active_bitmap),
        .i_addr_ready(i_addr_ready),
        .o_addr_valid(o_addr_valid),
        .o_addr(o_addr),
        .o_done(o_done),
        .o_busy(o_busy),
        .o_active_count(o_active_count)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    task pulse_start;
        input [P_BITMAP_WIDTH-1:0] bitmap;
        begin
            @(posedge clk);
            i_active_bitmap <= bitmap;
            i_start <= 1'b1;
            @(posedge clk);
            i_start <= 1'b0;
            i_active_bitmap <= {P_BITMAP_WIDTH{1'b0}};
            #1;
        end
    endtask

    task expect_addr;
        input [P_ADDR_WIDTH-1:0] expected_addr;
        begin
            while (!o_addr_valid) begin
                @(posedge clk);
            end

            if (o_addr !== expected_addr) begin
                $display("ERROR: expected addr=%0d, got addr=%0d", expected_addr, o_addr);
                error_count = error_count + 1;
            end else begin
                $display("INFO: got addr=%0d", o_addr);
            end

            i_addr_ready = 1'b1;
            @(posedge clk);
            #1;
            i_addr_ready = 1'b0;
            recv_count = recv_count + 1;
        end
    endtask

    initial begin
        rst_n = 1'b0;
        i_start = 1'b0;
        i_active_bitmap = {P_BITMAP_WIDTH{1'b0}};
        i_addr_ready = 1'b0;
        recv_count = 0;
        error_count = 0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        // 测试 1：普通稀疏位图，应按地址从小到大输出 0、3、7、15。
        pulse_start(16'b1000_0000_1000_1001);
        expect_addr(4'd0);
        expect_addr(4'd3);
        expect_addr(4'd7);
        expect_addr(4'd15);
        if (o_active_count !== 32'd4) begin
            $display("ERROR: expected active_count=4, got %0d", o_active_count);
            error_count = error_count + 1;
        end
        while (!o_done) @(posedge clk);

        // 测试 2：空位图，应直接 done，不输出地址。
        recv_count = 0;
        pulse_start(16'b0000_0000_0000_0000);
        while (!o_done) @(posedge clk);
        if (o_addr_valid) begin
            $display("ERROR: empty bitmap should not output valid addr");
            error_count = error_count + 1;
        end

        // 测试 3：ready 反压，地址有效但未 ready 时应保持当前地址。
        i_addr_ready = 1'b0;
        pulse_start(16'b0000_0000_0010_0010);
        repeat (3) @(posedge clk);
        if (!o_addr_valid || o_addr !== 4'd1) begin
            $display("ERROR: backpressure hold failed, addr=%0d valid=%0b", o_addr, o_addr_valid);
            error_count = error_count + 1;
        end
        i_addr_ready = 1'b0;
        expect_addr(4'd1);
        expect_addr(4'd5);
        while (!o_done) @(posedge clk);

        if (error_count == 0) begin
            $display("SIM PASS: bitmap_to_addr_stream basic tests passed");
        end else begin
            $display("SIM FAIL: error_count=%0d", error_count);
        end

        $finish;
    end

    initial begin
        #2000;
        $display("TIMEOUT: state=%0d valid=%0b addr=%0d done=%0b busy=%0b bitmap=%b", dut.current_state_reg, o_addr_valid, o_addr, o_done, o_busy, dut.work_bitmap_reg);
        $finish;
    end

endmodule




