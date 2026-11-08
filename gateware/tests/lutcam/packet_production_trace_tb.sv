`timescale 1ns/1ps
`default_nettype none

// Observe the existing complete camera fixture without changing its stimulus
// or flow control. The resulting trace contains real production output times,
// including semantic LWC bursts and the explicitly marked zero padding tail.
module packet_production_trace_tb;
    camera_fullchain_tb frame_source();
    integer trace_file;
    integer cycle = 0;
    string trace_path;

    initial begin
        if (!$value$plusargs("TRACE=%s", trace_path))
            $fatal(1, "TRACE=<output path> is required");
        trace_file = $fopen(trace_path, "w");
        if (trace_file == 0)
            $fatal(1, "cannot open output trace %s", trace_path);
    end

    always @(posedge frame_source.clk) begin
        if (frame_source.rst) begin
            cycle = 0;
        end else begin
            cycle = cycle + 1;
            if (frame_source.current_frame >= 0) begin
                if (frame_source.lwc_out_valid)
                    $fwrite(trace_file, "%0d I %0d %04x %0d %0d\n",
                        cycle, frame_source.frame_id, frame_source.lwc_out_data,
                        frame_source.lwc_out_sof, frame_source.lwc_out_padding);
                if (frame_source.inner_fv_valid)
                    $fwrite(trace_file, "%0d C %0d %09x %0d\n",
                        cycle, frame_source.frame_id,
                        frame_source.dut.inner_fv_data, frame_source.hier_inner_end);
                if (frame_source.lftag_result_valid)
                    $fwrite(trace_file, "%0d P %0d %08x %0d\n",
                        cycle, frame_source.frame_id, frame_source.lftag_result_data,
                        frame_source.lftag_result_last);
            end
        end
    end

    final begin
        $fclose(trace_file);
    end
endmodule

`default_nettype wire
