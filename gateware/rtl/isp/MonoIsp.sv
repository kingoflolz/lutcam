`default_nettype none

// Two-pixel-per-clock monochrome image signal processor. Sparse defect
// correction operates in the ten-bit domain before exposure statistics and
// the programmable raw-to-eight-bit tone lookup.
module MonoIsp #(
    parameter integer WIDTH = 1280,
    parameter integer HEIGHT = 800
) (
    input  wire         clk,
    input  wire         rst,

    input  wire         wb_cyc_i,
    input  wire         wb_stb_i,
    input  wire         wb_we_i,
    input  wire  [3:0]  wb_sel_i,
    input  wire  [15:0] wb_adr_i,
    input  wire  [31:0] wb_dat_i,
    output logic [31:0] wb_dat_o,
    output logic        wb_ack_o,
    output logic        wb_err_o,

    input  wire         in_valid,
    output logic        in_ready,
    input  wire  [9:0]  in_pixel0,
    input  wire  [9:0]  in_pixel1,
    input  wire         in_sof,
    input  wire         in_eol,

    output logic        out_valid,
    input  wire         out_ready,
    output logic [7:0]  out_pixel0,
    output logic [7:0]  out_pixel1,
    output logic        out_sof,
    output logic        out_eol,

    input  wire         hist_dump_req,
    output logic        hist_dump_ready,
    output logic        hist_out_valid,
    input  wire         hist_out_ready,
    output logic [31:0] hist_out_data,

    output logic        protocol_error,
    // Expected markers for the next accepted pair, before the output stage.
    // A controller sharing this raster must cancel its frame on a malformed
    // marker or rejected input; these outputs do not change ISP behavior.
    output wire         input_position_sof,
    output wire         input_position_eol,
    output wire         input_position_eof
);
    localparam integer PAIRS_PER_ROW = WIDTH / 2;
    localparam integer FRAME_PAIRS = PAIRS_PER_ROW * HEIGHT;
    localparam integer ROW_PAIR_WIDTH =
        (PAIRS_PER_ROW <= 1) ? 1 : $clog2(PAIRS_PER_ROW);

    localparam integer FRAME_ADDRESS_WIDTH =
        (FRAME_PAIRS <= 1) ? 1 : $clog2(FRAME_PAIRS);
    // Stored defect addresses load only after the full sum is in range.
    // Keep enough sum bits for the entire 18-bit delta even in tiny frames.
    localparam integer DEFECT_SUM_WIDTH =
        ((FRAME_ADDRESS_WIDTH > 18) ? FRAME_ADDRESS_WIDTH : 18) + 1;
    localparam logic [26:0] LAST_FRAME_PAIR = 27'(FRAME_PAIRS - 1);
    localparam logic [27:0] FRAME_PAIRS_EXTENDED = 28'(FRAME_PAIRS);
    localparam logic [ROW_PAIR_WIDTH-1:0] LAST_ROW_PAIR =
        ROW_PAIR_WIDTH'(PAIRS_PER_ROW - 1);

    localparam logic [15:0] REG_ID = 16'h0000;
    localparam logic [15:0] REG_CONTROL = 16'h0004;
    localparam logic [15:0] REG_STATUS = 16'h0008;
    localparam logic [15:0] REG_DEFECT_COUNT = 16'h000c;
    localparam logic [15:0] REG_COMMAND = 16'h0010;
    localparam logic [15:0] TONE_LUT_BASE = 16'h1000;
    localparam logic [15:0] TONE_LUT_LAST = 16'h1ffc;
    localparam logic [15:0] DEFECT_TABLE_BASE = 16'h2000;
    localparam logic [15:0] DEFECT_TABLE_LAST = 16'h27fc;

    localparam logic [31:0] ISP_ID = 32'h4d49_5350;

    (* ram_style = "block", syn_ramstyle = "block_ram" *)
    logic [7:0] tone_lut [0:1023];
    (* ram_style = "block", syn_ramstyle = "block_ram" *)
    logic [19:0] defect_table [0:511];
    (* ram_style = "block", syn_ramstyle = "block_ram" *)
    logic [19:0] histogram [0:511];

    logic run_enabled;
    logic lut_enabled;
    logic histogram_enabled;
    logic histogram_frame_mode;
    logic histogram_collecting;
    logic histogram_frozen;
    wire histogram_sample;
    logic [9:0] defect_word_count;

    logic output_pending;
    logic output_lut_enabled;
    logic [7:0] output_bypass0;
    logic [7:0] output_bypass1;
    logic output_sof_q;
    logic output_eol_q;
    logic [7:0] tone_port0_q;
    logic [7:0] tone_port1_q;

    // The counter wraps at FRAME_PAIRS - 1; keep only its reachable bits.
    logic [FRAME_ADDRESS_WIDTH-1:0] pair_address;
    // Retain the original descriptor-comparison width explicitly.
    wire [26:0] pair_address_extended = 27'(pair_address);
    logic [ROW_PAIR_WIDTH-1:0] row_pair_address;
    logic [9:0] previous_corrected;

    logic defect_restart;
    logic [9:0] defect_issue_index;
    logic defect_read_pending;
    logic [19:0] defect_port_q;
    logic [FRAME_ADDRESS_WIDTH-1:0] defect_accumulated_address;
    // Nonnegative deltas need only remember whether the prefix has advanced
    // past the last real defect; the first real defect may be at address zero.
    logic defect_order_advanced;
    logic defect_descriptor0_valid;
    logic defect_descriptor1_valid;
    logic [FRAME_ADDRESS_WIDTH-1:0] defect_descriptor0_address;
    logic [FRAME_ADDRESS_WIDTH-1:0] defect_descriptor1_address;
    logic [1:0] defect_descriptor0_mask;
    logic [1:0] defect_descriptor1_mask;

    logic histogram_initialized;
    logic histogram_clear_busy;
    logic [8:0] histogram_clear_index;
    logic histogram_read_pending;
    logic [8:0] histogram_read_bin;
    logic [19:0] histogram_port_q;
    logic histogram_forward_valid;
    logic [8:0] histogram_forward_bin;
    logic [19:0] histogram_forward_value;
    logic histogram_dump_valid;
    logic [8:0] histogram_dump_bin;

    logic wb_ram_pending;
    logic wb_ram_is_defect;
    logic wb_request;
    logic wb_stop_request;
    logic wb_run_start_request;
    wire wb_idle_control_accept;
    logic wb_histogram_clear_request;
    logic wb_tone_read_request;
    logic wb_tone_write_request;
    logic wb_defect_read_request;
    logic wb_defect_write_request;

    logic tone_port0_read_enable;
    logic tone_port1_read_enable;
    logic tone_port0_write_enable;
    logic tone_port0_enable;
    logic tone_port1_enable;
    logic [9:0] tone_port0_address;
    logic [9:0] tone_port1_address;
    logic defect_memory_read_enable;
    logic [8:0] defect_memory_address;
    logic histogram_memory_read_enable;
    logic histogram_memory_write_enable;
    logic [8:0] histogram_memory_read_address;
    logic [8:0] histogram_memory_write_address;
    logic [19:0] histogram_memory_write_data;

    logic [1:0] active_defect_mask;
    logic [9:0] corrected_pixel0;
    logic [9:0] corrected_pixel1;
    logic [10:0] histogram_sum;
    logic [8:0] histogram_sample_bin;
    logic [19:0] histogram_increment_base;
    logic [19:0] histogram_increment_value;

    logic [DEFECT_SUM_WIDTH-1:0] defect_response_sum;
    logic defect_response_in_range;
    logic defect_response_in_order;
    logic defect_response_push;
    logic defect_descriptor_pop;
    logic [1:0] defect_queue_count;
    logic [1:0] defect_queue_after_events;
    logic defect_issue_enable;
    logic defect_all_words_read;
    logic defect_ready;
    logic pipeline_busy;
    logic pixel_accept;
    logic histogram_dump_start;
    logic histogram_dump_fire;
    logic [31:0] status_word;

`ifndef SYNTHESIS
    initial begin
        if ((WIDTH <= 0) || (HEIGHT <= 0) || ((WIDTH % 2) != 0))
            $error("MonoIsp requires positive dimensions and an even WIDTH");
        if (FRAME_PAIRS > 134217727)
            $error("MonoIsp frame exceeds the cumulative address width");
    end
`endif

    always_comb begin
        active_defect_mask = 2'b00;
        if (defect_descriptor0_valid
                && (defect_descriptor0_address == pair_address))
            active_defect_mask = defect_descriptor0_mask;

        corrected_pixel0 = in_pixel0;
        corrected_pixel1 = in_pixel1;
        case (active_defect_mask)
            2'b01: begin
                corrected_pixel0 = in_pixel1;
                corrected_pixel1 = in_pixel1;
            end
            2'b10: begin
                corrected_pixel0 = in_pixel0;
                corrected_pixel1 = in_pixel0;
            end
            2'b11: begin
                if (row_pair_address == '0) begin
                    corrected_pixel0 = 10'd0;
                    corrected_pixel1 = 10'd0;
                end else begin
                    corrected_pixel0 = previous_corrected;
                    corrected_pixel1 = previous_corrected;
                end
            end
            default: begin
                corrected_pixel0 = in_pixel0;
                corrected_pixel1 = in_pixel1;
            end
        endcase
    end

    assign histogram_sum = {1'b0, corrected_pixel0}
        + {1'b0, corrected_pixel1};
    assign histogram_sample_bin = 9'((histogram_sum + 11'd1) >> 2);

    assign histogram_increment_base =
        histogram_forward_valid && (histogram_forward_bin == histogram_read_bin)
            ? histogram_forward_value : histogram_port_q;
    assign histogram_increment_value = (&histogram_increment_base)
        ? histogram_increment_base : histogram_increment_base + 20'd1;

    // DSPs implement actual increment/addition operations. Portable simulation
    // uses identical edge timing; wider parameterizations retain fabric adders.
    localparam bit USE_NATIVE_ARITHMETIC = 1'b1
`ifndef SYNTHESIS
        // synthesis translate_off
        && 1'b0
        // synthesis translate_on
`endif
        ;
    generate if (USE_NATIVE_ARITHMETIC) begin : native_raster
        wire [47:0] unused_pair_count, unused_row_count;
        wire [47:0] unused_pair_cascade, unused_row_cascade;
        wire unused_pair_overflow, unused_row_overflow;
        EFX_DSP48 #(.MODE("NORMAL"), .SIGNED(0), .M_SEL("C"), .N_SEL("W"),
            .W_SEL("X"), .W_REG(1), .RST_SYNC(1)) unused_pair_counter (
            .A(19'd0), .B(18'd0), .C(18'd1), .CASCIN(48'd0),
            .OP(2'd0), .SHIFT_ENA(1'b0), .CLK(clk), .CE(pixel_accept),
            .RST(rst || wb_idle_control_accept
                || (pixel_accept && (pair_address_extended == LAST_FRAME_PAIR))),
            .O(unused_pair_count), .CASCOUT(unused_pair_cascade),
            .OVFL(unused_pair_overflow));
        EFX_DSP48 #(.MODE("NORMAL"), .SIGNED(0), .M_SEL("C"), .N_SEL("W"),
            .W_SEL("X"), .W_REG(1), .RST_SYNC(1)) unused_row_counter (
            .A(19'd0), .B(18'd0), .C(18'd1), .CASCIN(48'd0),
            .OP(2'd0), .SHIFT_ENA(1'b0), .CLK(clk), .CE(pixel_accept),
            .RST(rst || wb_idle_control_accept || (pixel_accept
                && ((pair_address_extended == LAST_FRAME_PAIR)
                    || (row_pair_address == LAST_ROW_PAIR)))),
            .O(unused_row_count), .CASCOUT(unused_row_cascade),
            .OVFL(unused_row_overflow));
        assign pair_address = FRAME_ADDRESS_WIDTH'(unused_pair_count);
        assign row_pair_address = ROW_PAIR_WIDTH'(unused_row_count);
    end else begin : portable_raster
        always_ff @(posedge clk) begin
            if (rst || wb_idle_control_accept) begin
                pair_address <= '0;
                row_pair_address <= '0;
            end else if (pixel_accept) begin
                if (pair_address_extended == LAST_FRAME_PAIR) begin
                    pair_address <= '0;
                    row_pair_address <= '0;
                end else begin
                    pair_address <= pair_address + 1'b1;
                    if (row_pair_address == LAST_ROW_PAIR)
                        row_pair_address <= '0;
                    else
                        row_pair_address <= row_pair_address + ROW_PAIR_WIDTH'(1);
                end
            end
        end
    end endgenerate
    generate if (USE_NATIVE_ARITHMETIC && (FRAME_ADDRESS_WIDTH <= 19)) begin : native_defect_sum
        wire [47:0] unused_next_address, unused_address_cascade;
        wire unused_address_overflow;
        // The unregistered DSP output computes the current prefix plus delta;
        // all original register enables and descriptor timing remain intact.
        EFX_DSP48 #(.MODE("NORMAL"), .SIGNED(0), .M_SEL("P"), .N_SEL("C"),
            .W_SEL("X"), .W_REG(0), .O_REG(0)) address_adder (
            .A(19'(defect_accumulated_address)), .B(18'd1),
            .C(defect_port_q[19:2]), .CASCIN(48'd0),
            .OP(2'd0), .SHIFT_ENA(1'b0), .CLK(1'b0), .CE(1'b0), .RST(1'b0),
            .O(unused_next_address), .CASCOUT(unused_address_cascade),
            .OVFL(unused_address_overflow));
        assign defect_response_sum = DEFECT_SUM_WIDTH'(unused_next_address);
    end else begin : portable_defect_sum
        assign defect_response_sum = DEFECT_SUM_WIDTH'(defect_accumulated_address)
            + DEFECT_SUM_WIDTH'(defect_port_q[19:2]);
    end endgenerate
    always_ff @(posedge clk) begin
        if (rst || !run_enabled || defect_restart)
            defect_accumulated_address <= '0;
        else if (!protocol_error && defect_read_pending && defect_response_in_range)
            defect_accumulated_address <= FRAME_ADDRESS_WIDTH'(defect_response_sum);
    end
    assign defect_response_in_range =
        28'(defect_response_sum) < FRAME_PAIRS_EXTENDED;
    assign defect_response_in_order = defect_order_advanced
        || (defect_port_q[19:2] != 18'd0);
    assign defect_response_push = defect_read_pending
        && (defect_port_q[1:0] != 2'b00)
        && defect_response_in_range
        && defect_response_in_order;

    assign defect_descriptor_pop = pixel_accept
        && defect_descriptor0_valid
        && (defect_descriptor0_address == pair_address);

    always_comb begin
        case ({defect_descriptor1_valid, defect_descriptor0_valid})
            2'b00: defect_queue_count = 2'd0;
            2'b01: defect_queue_count = 2'd1;
            default: defect_queue_count = 2'd2;
        endcase

        case ({defect_descriptor_pop, defect_response_push})
            2'b01: defect_queue_after_events = defect_queue_count + 2'd1;
            2'b10: defect_queue_after_events = defect_queue_count - 2'd1;
            default: defect_queue_after_events = defect_queue_count;
        endcase
    end

    assign defect_issue_enable = run_enabled
        && !defect_restart
        && !protocol_error
        && (defect_issue_index < defect_word_count)
        && (defect_queue_after_events < 2'd2);
    assign defect_all_words_read =
        (defect_issue_index == defect_word_count) && !defect_read_pending;
    assign defect_ready = run_enabled
        && !defect_restart
        && (defect_all_words_read
            || defect_descriptor0_valid
            // Every unread delta is nonnegative. With no queued descriptor,
            // the decoded prefix therefore proves all pairs below this
            // frontier are clear. Equality is deliberately excluded: a
            // following descriptor may use delta zero at the frontier.
            || (defect_accumulated_address > pair_address));

    assign wb_request = !wb_ram_pending && wb_cyc_i && wb_stb_i;
    assign wb_stop_request = wb_request
        && wb_we_i
        && (wb_sel_i == 4'b1111)
        && (wb_adr_i == REG_CONTROL)
        && run_enabled
        && (wb_dat_i == 32'd0);
    assign wb_run_start_request = wb_request
        && wb_we_i
        && (wb_sel_i == 4'b1111)
        && (wb_adr_i == REG_CONTROL)
        && !run_enabled
        && (wb_dat_i[31:4] == 28'd0)
        && wb_dat_i[0]
        && !histogram_clear_busy
        && !histogram_dump_valid
        && !pipeline_busy
        && (!wb_dat_i[2] || histogram_initialized);
    // Successful stopped-mode CONTROL writes reset the raster counters,
    // including writes which leave the processor stopped.
    assign wb_idle_control_accept = wb_request && wb_we_i
        && (wb_sel_i == 4'b1111) && (wb_adr_i == REG_CONTROL)
        && !run_enabled && (wb_dat_i[31:4] == 28'd0)
        && (!wb_dat_i[0] || (!histogram_clear_busy && !histogram_dump_valid
            && !pipeline_busy && (!wb_dat_i[2] || histogram_initialized)));

    assign wb_histogram_clear_request = wb_request
        && wb_we_i
        && (wb_sel_i == 4'b1111)
        && (wb_adr_i == REG_COMMAND)
        && (wb_dat_i[31:2] == 30'd0)
        && wb_dat_i[0]
        && !run_enabled
        && !pipeline_busy
        && !histogram_clear_busy
        && !histogram_dump_valid;

    assign wb_tone_read_request = wb_request
        && !wb_we_i
        && (wb_adr_i[1:0] == 2'b00)
        && (wb_adr_i >= TONE_LUT_BASE)
        && (wb_adr_i <= TONE_LUT_LAST)
        && !run_enabled
        && !pipeline_busy;
    assign wb_tone_write_request = wb_request
        && wb_we_i
        && (wb_sel_i == 4'b1111)
        && (wb_adr_i[1:0] == 2'b00)
        && (wb_adr_i >= TONE_LUT_BASE)
        && (wb_adr_i <= TONE_LUT_LAST)
        && !run_enabled
        && !pipeline_busy;
    assign wb_defect_read_request = wb_request
        && !wb_we_i
        && (wb_adr_i[1:0] == 2'b00)
        && (wb_adr_i >= DEFECT_TABLE_BASE)
        && (wb_adr_i <= DEFECT_TABLE_LAST)
        && !run_enabled;
    assign wb_defect_write_request = wb_request
        && wb_we_i
        && (wb_sel_i == 4'b1111)
        && (wb_adr_i[1:0] == 2'b00)
        && (wb_adr_i >= DEFECT_TABLE_BASE)
        && (wb_adr_i <= DEFECT_TABLE_LAST)
        && !run_enabled
        && (wb_dat_i[31:20] == 12'd0);

    assign tone_port0_read_enable = pixel_accept || wb_tone_read_request;
    assign tone_port1_read_enable = pixel_accept;
    assign tone_port0_write_enable = wb_tone_write_request;
    assign tone_port0_enable =
        tone_port0_read_enable || tone_port0_write_enable;
    assign tone_port1_enable = tone_port1_read_enable;
    assign tone_port0_address = pixel_accept
        ? corrected_pixel0 : wb_adr_i[11:2];
    assign tone_port1_address = corrected_pixel1;
    assign defect_memory_read_enable =
        defect_issue_enable || wb_defect_read_request;
    assign defect_memory_address = defect_issue_enable
        ? defect_issue_index[8:0] : wb_adr_i[10:2];

    assign pipeline_busy = output_pending || histogram_read_pending;
    assign in_ready = run_enabled
        && defect_ready
        && !protocol_error
        && !wb_stop_request
        && (!output_pending || out_ready);
    assign input_position_sof = pair_address == '0;
    assign input_position_eol = row_pair_address == LAST_ROW_PAIR;
    assign input_position_eof = pair_address_extended == LAST_FRAME_PAIR;
    assign pixel_accept = in_valid && in_ready;

    assign out_valid = output_pending;
    assign out_pixel0 = output_lut_enabled ? tone_port0_q : output_bypass0;
    assign out_pixel1 = output_lut_enabled ? tone_port1_q : output_bypass1;
    assign out_sof = output_sof_q;
    assign out_eol = output_eol_q;

    // Freeze the existing RAM at EOF; video continues while software reads it.
    // Clearing after consumption re-arms only at the next complete frame.
    assign histogram_sample = pixel_accept && histogram_enabled
        && (!histogram_frame_mode || (histogram_initialized
            && !histogram_clear_busy && !histogram_frozen && !histogram_dump_valid
            && (histogram_collecting || (input_position_sof && in_sof))));
    assign hist_dump_ready = ((histogram_frame_mode && histogram_frozen
            && !histogram_read_pending) || (!run_enabled && !pipeline_busy))
        && !histogram_clear_busy
        && !histogram_dump_valid
        && histogram_initialized
        && !wb_run_start_request
        && !wb_histogram_clear_request;
    assign histogram_dump_start = hist_dump_req && hist_dump_ready;
    assign hist_out_valid = histogram_dump_valid;
    assign histogram_dump_fire = histogram_dump_valid && hist_out_ready;
    assign hist_out_data = {
        histogram_dump_bin == 9'd0,
        histogram_dump_bin == 9'd511,
        1'b0,
        histogram_dump_bin,
        histogram_port_q
    };

    assign status_word = {
        25'd0,
        protocol_error,
        histogram_initialized,
        histogram_dump_valid,
        histogram_clear_busy,
        defect_ready,
        pipeline_busy,
        run_enabled
    };

    always_comb begin
        histogram_memory_read_enable = 1'b0;
        histogram_memory_read_address = 9'd0;
        if (histogram_dump_start) begin
            histogram_memory_read_enable = 1'b1;
            histogram_memory_read_address = 9'd0;
        end else if (histogram_dump_fire
                && (histogram_dump_bin != 9'd511)) begin
            histogram_memory_read_enable = 1'b1;
            histogram_memory_read_address = histogram_dump_bin + 9'd1;
        end else if (histogram_sample) begin
            histogram_memory_read_enable = 1'b1;
            histogram_memory_read_address = histogram_sample_bin;
        end

        histogram_memory_write_enable = 1'b0;
        histogram_memory_write_address = 9'd0;
        histogram_memory_write_data = 20'd0;
        if (histogram_clear_busy) begin
            histogram_memory_write_enable = 1'b1;
            histogram_memory_write_address = histogram_clear_index;
            histogram_memory_write_data = 20'd0;
        end else if (!histogram_dump_valid && histogram_read_pending) begin
            histogram_memory_write_enable = 1'b1;
            histogram_memory_write_address = histogram_read_bin;
            histogram_memory_write_data = histogram_increment_value;
        end
    end

    // The tone table uses the true-dual-port template: port zero switches
    // between software read/write access and the first runtime lookup, while
    // the independently addressed port one performs the second runtime read.
    // Every array retains a single procedural owner to keep lint and inference
    // deterministic.
    always_ff @(posedge clk) begin
        if (tone_port0_enable) begin
            if (tone_port0_write_enable)
                tone_lut[tone_port0_address] <= wb_dat_i[7:0];
            else
                tone_port0_q <= tone_lut[tone_port0_address];
        end

        if (tone_port1_enable)
            tone_port1_q <= tone_lut[tone_port1_address];
    end

    always_ff @(posedge clk) begin
        if (wb_defect_write_request)
            defect_table[wb_adr_i[10:2]] <= wb_dat_i[19:0];
        else if (defect_memory_read_enable)
            defect_port_q <= defect_table[defect_memory_address];
    end

    always_ff @(posedge clk) begin
        if (histogram_memory_write_enable)
            histogram[histogram_memory_write_address]
                <= histogram_memory_write_data;
        if (histogram_memory_read_enable)
            histogram_port_q <= histogram[histogram_memory_read_address];
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            run_enabled <= 1'b0;
            lut_enabled <= 1'b0;
            histogram_enabled <= 1'b0;
            histogram_frame_mode <= 1'b0;
            histogram_collecting <= 1'b0;
            histogram_frozen <= 1'b0;
            defect_word_count <= 10'd0;
            protocol_error <= 1'b0;

            output_pending <= 1'b0;
            output_lut_enabled <= 1'b0;
            output_bypass0 <= 8'd0;
            output_bypass1 <= 8'd0;
            output_sof_q <= 1'b0;
            output_eol_q <= 1'b0;

            previous_corrected <= 10'd0;

            defect_restart <= 1'b0;
            defect_issue_index <= 10'd0;
            defect_read_pending <= 1'b0;
            defect_order_advanced <= 1'b1;
            defect_descriptor0_valid <= 1'b0;
            defect_descriptor1_valid <= 1'b0;
            defect_descriptor0_address <= '0;
            defect_descriptor1_address <= '0;
            defect_descriptor0_mask <= 2'b00;
            defect_descriptor1_mask <= 2'b00;

            histogram_initialized <= 1'b0;
            histogram_clear_busy <= 1'b0;
            histogram_clear_index <= 9'd0;
            histogram_read_pending <= 1'b0;
            histogram_read_bin <= 9'd0;
            histogram_forward_valid <= 1'b0;
            histogram_forward_bin <= 9'd0;
            histogram_forward_value <= 20'd0;
            histogram_dump_valid <= 1'b0;
            histogram_dump_bin <= 9'd0;

            wb_ram_pending <= 1'b0;
            wb_ram_is_defect <= 1'b0;
            wb_dat_o <= 32'd0;
            wb_ack_o <= 1'b0;
            wb_err_o <= 1'b0;
        end else begin
            wb_ack_o <= 1'b0;
            wb_err_o <= 1'b0;

            if (output_pending && out_ready)
                output_pending <= 1'b0;
            if (pixel_accept) begin
                output_pending <= 1'b1;
                output_lut_enabled <= lut_enabled;
                output_bypass0 <= corrected_pixel0[9:2];
                output_bypass1 <= corrected_pixel1[9:2];
                output_sof_q <= in_sof;
                output_eol_q <= in_eol;

                if (in_sof != (pair_address == '0))
                    protocol_error <= 1'b1;
                if (in_eol != (row_pair_address == LAST_ROW_PAIR))
                    protocol_error <= 1'b1;

                if (pair_address_extended == LAST_FRAME_PAIR) begin
                    previous_corrected <= 10'd0;
                    defect_restart <= 1'b1;
                end else begin
                    if (row_pair_address == LAST_ROW_PAIR) begin
                        previous_corrected <= 10'd0;
                    end else begin
                        previous_corrected <= corrected_pixel1;
                    end
                end
            end

            if (!run_enabled || defect_restart) begin
                defect_restart <= 1'b0;
                defect_issue_index <= 10'd0;
                defect_read_pending <= 1'b0;
                defect_order_advanced <= 1'b1;
                defect_descriptor0_valid <= 1'b0;
                defect_descriptor1_valid <= 1'b0;
                defect_descriptor0_address <= '0;
                defect_descriptor1_address <= '0;
                defect_descriptor0_mask <= 2'b00;
                defect_descriptor1_mask <= 2'b00;
            end else if (!protocol_error) begin
                defect_read_pending <= defect_issue_enable;
                if (defect_issue_enable) begin
                    defect_issue_index <= defect_issue_index + 10'd1;
                end

                if (defect_read_pending) begin
                    if (!defect_response_in_range) begin
                        protocol_error <= 1'b1;
                    end else begin
                        if ((defect_port_q[1:0] != 2'b00)
                                && !defect_response_in_order)
                            protocol_error <= 1'b1;
                        if (defect_response_push)
                            defect_order_advanced <= 1'b0;
                        else if (defect_port_q[19:2] != 18'd0)
                            defect_order_advanced <= 1'b1;
                    end
                end

                case ({defect_descriptor_pop, defect_response_push})
                    2'b01: begin
                        if (!defect_descriptor0_valid) begin
                            defect_descriptor0_valid <= 1'b1;
                            defect_descriptor0_address
                                <= FRAME_ADDRESS_WIDTH'(defect_response_sum);
                            defect_descriptor0_mask <= defect_port_q[1:0];
                        end else begin
                            defect_descriptor1_valid <= 1'b1;
                            defect_descriptor1_address
                                <= FRAME_ADDRESS_WIDTH'(defect_response_sum);
                            defect_descriptor1_mask <= defect_port_q[1:0];
                        end
                    end
                    2'b10: begin
                        defect_descriptor0_valid <= defect_descriptor1_valid;
                        defect_descriptor0_address <= defect_descriptor1_address;
                        defect_descriptor0_mask <= defect_descriptor1_mask;
                        defect_descriptor1_valid <= 1'b0;
                    end
                    2'b11: begin
                        if (defect_descriptor1_valid) begin
                            defect_descriptor0_valid <= 1'b1;
                            defect_descriptor0_address
                                <= defect_descriptor1_address;
                            defect_descriptor0_mask <= defect_descriptor1_mask;
                            defect_descriptor1_valid <= 1'b1;
                            defect_descriptor1_address
                                <= FRAME_ADDRESS_WIDTH'(defect_response_sum);
                            defect_descriptor1_mask <= defect_port_q[1:0];
                        end else begin
                            defect_descriptor0_valid <= 1'b1;
                            defect_descriptor0_address
                                <= FRAME_ADDRESS_WIDTH'(defect_response_sum);
                            defect_descriptor0_mask <= defect_port_q[1:0];
                            defect_descriptor1_valid <= 1'b0;
                        end
                    end
                    default: begin
                    end
                endcase

                if (defect_descriptor0_valid
                        && (defect_descriptor0_address < pair_address))
                    protocol_error <= 1'b1;
            end

            if (histogram_frame_mode && histogram_sample) begin
                histogram_collecting <= !input_position_eof;
                if (input_position_eof)
                    histogram_frozen <= 1'b1;
            end
            if (wb_run_start_request && wb_dat_i[3]) begin
                histogram_collecting <= 1'b0;
                histogram_frozen <= 1'b0;
                histogram_clear_busy <= 1'b1;
                histogram_clear_index <= 9'd0;
                histogram_initialized <= 1'b0;
                histogram_forward_valid <= 1'b0;
            end
            if (histogram_clear_busy) begin
                histogram_forward_valid <= 1'b0;
                if (histogram_clear_index == 9'd511) begin
                    histogram_clear_busy <= 1'b0;
                    histogram_initialized <= 1'b1;
                    histogram_clear_index <= 9'd0;
                end else begin
                    histogram_clear_index <= histogram_clear_index + 9'd1;
                end
            end else if (histogram_dump_start) begin
                histogram_dump_valid <= 1'b1;
                histogram_dump_bin <= 9'd0;
            end else if (histogram_dump_fire) begin
                if (histogram_dump_bin == 9'd511) begin
                    histogram_dump_valid <= 1'b0;
                    histogram_dump_bin <= 9'd0;
                    if (histogram_frame_mode) begin
                        histogram_clear_busy <= 1'b1;
                        histogram_clear_index <= 9'd0;
                        histogram_initialized <= 1'b0;
                        histogram_forward_valid <= 1'b0;
                        histogram_collecting <= 1'b0;
                        histogram_frozen <= 1'b0;
                    end
                end else begin
                    histogram_dump_bin <= histogram_dump_bin + 9'd1;
                end
            end else if (!histogram_dump_valid) begin
                if (histogram_read_pending) begin
                    histogram_forward_valid <= 1'b1;
                    histogram_forward_bin <= histogram_read_bin;
                    histogram_forward_value <= histogram_increment_value;
                end
                histogram_read_pending <= histogram_sample;
                if (histogram_sample) begin
                    histogram_read_bin <= histogram_sample_bin;
                end
            end

            if (wb_ram_pending) begin
                wb_ram_pending <= 1'b0;
                if (wb_cyc_i && wb_stb_i) begin
                    wb_ack_o <= 1'b1;
                    if (wb_ram_is_defect)
                        wb_dat_o <= {12'd0, defect_port_q};
                    else
                        wb_dat_o <= {24'd0, tone_port0_q};
                end
            end else if (wb_cyc_i && wb_stb_i) begin
                if (wb_adr_i[1:0] != 2'b00) begin
                    wb_err_o <= 1'b1;
                end else if (wb_we_i) begin
                    if (wb_sel_i != 4'b1111) begin
                        wb_err_o <= 1'b1;
                    end else if (wb_adr_i == REG_CONTROL) begin
                        if (run_enabled) begin
                            if (wb_dat_i == 32'd0) begin
                                wb_ack_o <= 1'b1;
                                run_enabled <= 1'b0;
                                defect_restart <= 1'b1;
                            end else begin
                                wb_err_o <= 1'b1;
                            end
                        end else if (wb_dat_i[31:4] != 28'd0) begin
                            wb_err_o <= 1'b1;
                        end else if (wb_dat_i[0]
                                && (histogram_clear_busy
                                    || histogram_dump_valid
                                    || pipeline_busy
                                    || (wb_dat_i[2]
                                        && !histogram_initialized))) begin
                            wb_err_o <= 1'b1;
                        end else begin
                            wb_ack_o <= 1'b1;
                            run_enabled <= wb_dat_i[0];
                            lut_enabled <= wb_dat_i[1];
                            histogram_enabled <= wb_dat_i[2];
                            histogram_frame_mode <= wb_dat_i[3];
                            previous_corrected <= 10'd0;
                            if (wb_dat_i[0])
                                defect_restart <= 1'b1;
                        end
                    end else if (wb_adr_i == REG_DEFECT_COUNT) begin
                        if (run_enabled || (wb_dat_i > 32'd512)) begin
                            wb_err_o <= 1'b1;
                        end else begin
                            wb_ack_o <= 1'b1;
                            defect_word_count <= wb_dat_i[9:0];
                        end
                    end else if (wb_adr_i == REG_COMMAND) begin
                        if (wb_dat_i[31:2] != 30'd0) begin
                            wb_err_o <= 1'b1;
                        end else if (wb_dat_i[0]
                                && (run_enabled
                                    || pipeline_busy
                                    || histogram_clear_busy
                                    || histogram_dump_valid)) begin
                            wb_err_o <= 1'b1;
                        end else begin
                            wb_ack_o <= 1'b1;
                            if (wb_dat_i[1])
                                protocol_error <= 1'b0;
                            if (wb_dat_i[0]) begin
                                histogram_clear_busy <= 1'b1;
                                histogram_clear_index <= 9'd0;
                                histogram_initialized <= 1'b0;
                                histogram_forward_valid <= 1'b0;
                            end
                        end
                    end else if ((wb_adr_i >= TONE_LUT_BASE)
                            && (wb_adr_i <= TONE_LUT_LAST)) begin
                        if (run_enabled || pipeline_busy) begin
                            wb_err_o <= 1'b1;
                        end else begin
                            wb_ack_o <= 1'b1;
                        end
                    end else if ((wb_adr_i >= DEFECT_TABLE_BASE)
                            && (wb_adr_i <= DEFECT_TABLE_LAST)) begin
                        if (run_enabled || (wb_dat_i[31:20] != 12'd0)) begin
                            wb_err_o <= 1'b1;
                        end else begin
                            wb_ack_o <= 1'b1;
                        end
                    end else begin
                        wb_err_o <= 1'b1;
                    end
                end else begin
                    if (wb_adr_i == REG_ID) begin
                        wb_dat_o <= ISP_ID;
                        wb_ack_o <= 1'b1;
                    end else if (wb_adr_i == REG_CONTROL) begin
                        wb_dat_o <= {
                            28'd0,
                            histogram_frame_mode,
                            histogram_enabled,
                            lut_enabled,
                            run_enabled
                        };
                        wb_ack_o <= 1'b1;
                    end else if (wb_adr_i == REG_STATUS) begin
                        wb_dat_o <= status_word;
                        wb_ack_o <= 1'b1;
                    end else if (wb_adr_i == REG_DEFECT_COUNT) begin
                        wb_dat_o <= {22'd0, defect_word_count};
                        wb_ack_o <= 1'b1;
                    end else if (wb_adr_i == REG_COMMAND) begin
                        wb_dat_o <= 32'd0;
                        wb_ack_o <= 1'b1;
                    end else if ((wb_adr_i >= TONE_LUT_BASE)
                            && (wb_adr_i <= TONE_LUT_LAST)) begin
                        if (run_enabled || pipeline_busy) begin
                            wb_err_o <= 1'b1;
                        end else begin
                            wb_ram_is_defect <= 1'b0;
                            wb_ram_pending <= 1'b1;
                        end
                    end else if ((wb_adr_i >= DEFECT_TABLE_BASE)
                            && (wb_adr_i <= DEFECT_TABLE_LAST)) begin
                        if (run_enabled) begin
                            wb_err_o <= 1'b1;
                        end else begin
                            wb_ram_is_defect <= 1'b1;
                            wb_ram_pending <= 1'b1;
                        end
                    end else begin
                        wb_err_o <= 1'b1;
                    end
                end
            end
        end
    end
endmodule

`default_nettype wire
