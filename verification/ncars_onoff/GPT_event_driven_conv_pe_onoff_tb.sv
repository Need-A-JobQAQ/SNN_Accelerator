`timescale 1ns/1ps

module GPT_event_driven_conv_pe_onoff_tb;
    `include "ncars_onoff_vectors.vh"
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg start = 0;
    reg collecting = 0;
    reg [1023:0] input_on = 0;
    reg [1023:0] input_off = 0;
    reg [1023:0] baseline_union = 0;
    reg [1023:0] on_vectors [0:CASE_COUNT-1];
    reg [1023:0] off_vectors [0:CASE_COUNT-1];
    reg signed [25:0] golden [0:CASE_COUNT*4096-1];
    reg expected_active [0:CASE_COUNT*1024-1];
    reg signed [25:0] actual_fast [0:4095];
    reg signed [25:0] actual_serial [0:4095];
    reg active_fast [0:4095];
    reg active_serial [0:4095];
    reg seen_fast [0:4095];
    reg seen_serial [0:4095];
    reg [3:0] done_fast_seen = 0;
    reg [3:0] done_serial_seen = 0;
    reg [3:0] done_baseline_seen = 0;
    wire [3:0] valid_fast, valid_serial, active_fast_w, active_serial_w;
    wire [3:0] done_fast, done_serial, done_baseline, valid_baseline;
    wire [3:0][9:0] addr_fast, addr_serial;
    wire signed [25:0] data_fast [0:3];
    wire signed [25:0] data_serial [0:3];
    integer cycles_fast [0:3];
    integer cycles_serial [0:3];
    integer cycles_baseline [0:3];
    integer writes_fast [0:3];
    integer writes_baseline [0:3];
    integer process_fast [0:3];
    integer process_serial [0:3];
    integer last_fast [0:3];
    integer last_serial [0:3];
    integer case_id = 0;
    integer pixel, ch, tick, event_count;
    integer total_fast = 0;
    integer total_serial = 0;
    integer total_baseline = 0;
    string data_dir;
    genvar g;

    generate for (g = 0; g < 4; g = g + 1) begin : channels
        event_driven_conv_pe_onoff #(
            .P_KERNEL_ON_WEIGHTS_PACKED(WEIGHTS_ON[575-g*144 -: 144]),
            .P_KERNEL_OFF_WEIGHTS_PACKED(WEIGHTS_OFF[575-g*144 -: 144])
        ) fast (
            .clk(clk), .rst_n(rst_n), .i_start(start),
            .i_input_spike_vector_on(input_on), .i_input_spike_vector_off(input_off),
            .o_current_valid(valid_fast[g]), .o_current_addr(addr_fast[g]),
            .o_current_data(data_fast[g]), .o_current_active(active_fast_w[g]), .o_done(done_fast[g]),
            .o_total_retired_pixels(), .o_total_current_ram_writes(),
            .o_skipped_clean_retirements(), .o_priority_search_count()
        );
        event_driven_conv_pe_onoff_9cycle #(
            .P_KERNEL_ON_WEIGHTS_PACKED(WEIGHTS_ON[575-g*144 -: 144]),
            .P_KERNEL_OFF_WEIGHTS_PACKED(WEIGHTS_OFF[575-g*144 -: 144])
        ) serial_pe (
            .clk(clk), .rst_n(rst_n), .i_start(start),
            .i_input_spike_vector_on(input_on), .i_input_spike_vector_off(input_off),
            .o_current_valid(valid_serial[g]), .o_current_addr(addr_serial[g]),
            .o_current_data(data_serial[g]), .o_current_active(active_serial_w[g]), .o_done(done_serial[g])
        );
        // 原优化版只作相同空间事件流的周期对照，不作为双极性数值黄金模型。
        event_driven_conv_pe_reference32 #(
            .P_INPUT_HEIGHT(32), .P_INPUT_WIDTH(32), .P_NUM_INPUT_PIXELS(1024),
            .P_KERNEL_WEIGHTS_PACKED(WEIGHTS_ON[575-g*144 -: 144])
        ) baseline (
            .clk(clk), .rst_n(rst_n), .i_start(start), .i_input_spike_vector(baseline_union),
            .o_current_valid(valid_baseline[g]), .o_current_addr(), .o_current_data(),
            .o_current_active(), .o_done(done_baseline[g]), .o_total_retired_pixels(),
            .o_total_current_ram_writes(), .o_skipped_clean_retirements(), .o_priority_search_count()
        );

        integer index_fast, index_serial, spatial_fast, spatial_serial;
        always @(posedge clk) begin
            if (collecting) begin
                if (!done_fast_seen[g]) cycles_fast[g] = cycles_fast[g] + 1;
                if (!done_serial_seen[g]) cycles_serial[g] = cycles_serial[g] + 1;
                if (!done_baseline_seen[g]) cycles_baseline[g] = cycles_baseline[g] + 1;
                if (done_fast[g]) done_fast_seen[g] = 1;
                if (done_serial[g]) done_serial_seen[g] = 1;
                if (done_baseline[g]) done_baseline_seen[g] = 1;
                if (valid_baseline[g]) writes_baseline[g] = writes_baseline[g] + 1;
                if (fast.state_reg == fast.S_PROCESS_EVENT) begin
                    process_fast[g] = process_fast[g] + 1;
                    if (fast.event_polarity_reg == 0) $fatal(1, "empty polarity");
                    if (fast.frontier_overflow_comb) $fatal(1, "fast frontier overflow");
                    if ((fast.update_old_valid_0_comb && fast.update_col_0_comb < fast.retire_col_reg) ||
                        (fast.update_old_valid_1_comb && fast.update_col_1_comb < fast.retire_col_reg) ||
                        (fast.update_old_valid_2_comb && fast.update_col_2_comb < fast.retire_col_reg))
                        $fatal(1, "old output modified after retirement");
                end
                if (serial_pe.state_reg == serial_pe.S_PROCESS_EVENT) begin
                    process_serial[g] = process_serial[g] + 1;
                    if (serial_pe.update_valid_comb && serial_pe.update_row_sel_comb == 2 &&
                        serial_pe.frontier_overflow_comb) $fatal(1, "serial frontier overflow");
                end
                if (fast.state_reg == fast.S_FINISH_ROW && !fast.finish_needed_w &&
                    (fast.frontier_valid_reg[0] || fast.frontier_valid_reg[1] || fast.frontier_valid_reg[2]))
                    $fatal(1, "nonempty frontier at row switch");
                if (valid_fast[g]) begin
                    spatial_fast = 1023 - addr_fast[g];
                    index_fast = g*1024 + spatial_fast;
                    if (seen_fast[index_fast] || spatial_fast <= last_fast[g])
                        $fatal(1, "fast duplicate/unordered output case=%0d ch=%0d", case_id, g);
                    seen_fast[index_fast] = 1;
                    last_fast[g] = spatial_fast;
                    actual_fast[index_fast] = data_fast[g];
                    active_fast[index_fast] = active_fast_w[g];
                    writes_fast[g] = writes_fast[g] + 1;
                end
                if (valid_serial[g]) begin
                    spatial_serial = 1023 - addr_serial[g];
                    index_serial = g*1024 + spatial_serial;
                    if (seen_serial[index_serial] || spatial_serial <= last_serial[g])
                        $fatal(1, "serial duplicate/unordered output case=%0d ch=%0d", case_id, g);
                    seen_serial[index_serial] = 1;
                    last_serial[g] = spatial_serial;
                    actual_serial[index_serial] = data_serial[g];
                    active_serial[index_serial] = active_serial_w[g];
                end
            end
        end
    end endgenerate

    initial begin
        if (!$value$plusargs("DATA_DIR=%s", data_dir)) $fatal(1, "missing DATA_DIR");
        $readmemh({data_dir, "/input_on.mem"}, on_vectors);
        $readmemh({data_dir, "/input_off.mem"}, off_vectors);
        $readmemh({data_dir, "/golden.mem"}, golden);
        $readmemh({data_dir, "/active.mem"}, expected_active);
        repeat (4) @(negedge clk);
        rst_n = 1;
        for (case_id = 0; case_id < CASE_COUNT; case_id = case_id + 1) begin
            @(negedge clk);
            for (pixel = 0; pixel < 4096; pixel = pixel + 1) begin
                actual_fast[pixel] = 0; actual_serial[pixel] = 0;
                active_fast[pixel] = 0; active_serial[pixel] = 0;
                seen_fast[pixel] = 0; seen_serial[pixel] = 0;
            end
            for (ch = 0; ch < 4; ch = ch + 1) begin
                cycles_fast[ch] = 0; cycles_serial[ch] = 0; cycles_baseline[ch] = 0;
                writes_fast[ch] = 0; writes_baseline[ch] = 0;
                process_fast[ch] = 0; process_serial[ch] = 0;
                last_fast[ch] = -1; last_serial[ch] = -1;
            end
            done_fast_seen = 0; done_serial_seen = 0; done_baseline_seen = 0;
            input_on = on_vectors[case_id]; input_off = off_vectors[case_id];
            baseline_union = input_on | input_off;
            event_count = $countones(baseline_union);
            collecting = 1;
            start = 1;
            @(negedge clk);
            start = 0;
            // 接收开始后立即改动外部输入，验证整帧快照和极性不会串帧。
            input_on = ~input_on; input_off = ~input_off;
            tick = 0;
            while ((&done_fast_seen) !== 1'b1 || (&done_serial_seen) !== 1'b1 ||
                   (&done_baseline_seen) !== 1'b1) begin
                @(negedge clk);
                tick = tick + 1;
                if (tick > 100000) $fatal(1, "timeout case=%0d", case_id);
            end
            collecting = 0;
            for (ch = 0; ch < 4; ch = ch + 1) begin
                if (process_fast[ch] != event_count || process_serial[ch] != event_count*9)
                    $fatal(1, "incorrect process cycles case=%0d ch=%0d", case_id, ch);
                if (cycles_fast[ch] != cycles_baseline[ch] || writes_fast[ch] != writes_baseline[ch])
                    $fatal(1, "cycle/write regression case=%0d ch=%0d", case_id, ch);
                for (pixel = 0; pixel < 1024; pixel = pixel + 1) begin
                    if (actual_fast[ch*1024+pixel] !== golden[case_id*4096+ch*1024+pixel] ||
                        actual_serial[ch*1024+pixel] !== golden[case_id*4096+ch*1024+pixel] ||
                        active_fast[ch*1024+pixel] !== expected_active[case_id*1024+pixel] ||
                        active_serial[ch*1024+pixel] !== expected_active[case_id*1024+pixel])
                        $fatal(1, "map mismatch case=%0d ch=%0d pixel=%0d golden=%0d fast=%0d serial=%0d",
                               case_id, ch, pixel, golden[case_id*4096+ch*1024+pixel],
                               actual_fast[ch*1024+pixel], actual_serial[ch*1024+pixel]);
                end
            end
            total_fast = total_fast + cycles_fast[0];
            total_serial = total_serial + cycles_serial[0];
            total_baseline = total_baseline + cycles_baseline[0];
            $display("CASE_PASS id=%0d events=%0d fast=%0d serial=%0d baseline=%0d writes=%0d",
                     case_id, event_count, cycles_fast[0], cycles_serial[0], cycles_baseline[0], writes_fast[0]);
            repeat (3) @(negedge clk);
        end
        $display("PASS cases=%0d channels=4 fast_cycles=%0d serial_cycles=%0d baseline_cycles=%0d",
                 CASE_COUNT, total_fast, total_serial, total_baseline);
        $finish;
    end
endmodule
