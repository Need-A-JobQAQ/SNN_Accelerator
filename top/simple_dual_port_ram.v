module simple_dual_port_ram #(
    parameter P_DATA_WIDTH = 26,
    parameter P_ADDR_WIDTH = 11,
    parameter P_DEPTH = 1568
) (
    input wire clk,

    input wire i_write_en,
    input wire [P_ADDR_WIDTH-1:0] i_write_addr,
    input wire [P_DATA_WIDTH-1:0] i_write_data,

    input wire i_read_en,
    input wire [P_ADDR_WIDTH-1:0] i_read_addr,
    output reg [P_DATA_WIDTH-1:0] o_read_data
);

    /*
     * 标准简单双端口 RAM 模板：
     * 一个同步写端口，一个同步读端口。
     * 不在 RAM 内部写复位清零逻辑，清零由外部逐地址写 0 完成。
     */
    (* ram_style = "block" *)
    reg [P_DATA_WIDTH-1:0] ram_mem [P_DEPTH-1:0];

    always @(posedge clk) begin
        if (i_write_en) begin
            ram_mem[i_write_addr] <= i_write_data;
        end

        if (i_read_en) begin
            o_read_data <= ram_mem[i_read_addr];
        end
    end

endmodule
