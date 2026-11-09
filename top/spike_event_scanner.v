module spike_event_scanner #(
    parameter P_NUM_INPUT_PIXELS = 784,
    parameter P_EVENT_ADDR_WIDTH = $clog2(P_NUM_INPUT_PIXELS),
    parameter P_WORD_WIDTH = 32
) (
    input wire                                      clk,
    input wire                                      rst_n,
    input wire                                      i_start,
    input wire [P_NUM_INPUT_PIXELS-1:0]             i_spike_vector,
    input wire                                      i_event_ready,

    output wire                                     o_event_valid,
    output wire [P_EVENT_ADDR_WIDTH-1:0]            o_event_addr,
    output wire                                     o_done,
    output wire                                     o_busy,
    output wire [31:0]                              o_event_count
);

    /*
     * 脉冲事件扫描器。
     * 工程中 spike_vector[P_NUM_INPUT_PIXELS-1] 对应图像像素 0，
     * spike_vector[0] 对应最后一个图像像素。
     * 这里先把位序翻转成“bit 地址 = 图像像素地址”，再复用
     * bitmap_to_addr_stream 的低地址优先扫描。
     * 因此 o_event_addr 对外表示图像空间地址，输出顺序为左上到右下。
     */
    wire [P_NUM_INPUT_PIXELS-1:0] scan_bitmap_w;

    genvar pixel_idx;
    generate
        for (pixel_idx = 0; pixel_idx < P_NUM_INPUT_PIXELS; pixel_idx = pixel_idx + 1) begin : g_scan_bitmap
            assign scan_bitmap_w[pixel_idx] = i_spike_vector[P_NUM_INPUT_PIXELS - 1 - pixel_idx];
        end
    endgenerate

    bitmap_to_addr_stream #(
        .P_BITMAP_WIDTH  (P_NUM_INPUT_PIXELS),
        .P_ADDR_WIDTH    (P_EVENT_ADDR_WIDTH),
        .P_WORD_WIDTH    (P_WORD_WIDTH)
    ) u_spike_bitmap_compactor (
        .clk             (clk),
        .rst_n           (rst_n),
        .i_start         (i_start),
        .i_active_bitmap (scan_bitmap_w),
        .i_addr_ready    (i_event_ready),
        .o_addr_valid    (o_event_valid),
        .o_addr          (o_event_addr),
        .o_done          (o_done),
        .o_busy          (o_busy),
        .o_active_count  (o_event_count)
    );

endmodule
