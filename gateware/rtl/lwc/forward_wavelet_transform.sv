`default_nettype none

// Complete fixed-point 5H/3V forward Mallat transform. The only output is
// the raw per-band event stream consumed by the scalar frontend.
module forward_wavelet_transform #(
    parameter integer FRAME_WIDTH = 1280,
    parameter integer FRAME_HEIGHT = 800,
    parameter integer DATA_WIDTH = 14,
    parameter integer COORD_WIDTH = 10,
    // Opt in only for complete byte rasters or reset-aborted prefixes.
    parameter bit BOUNDED_BYTE_RASTER = 1'b0
) (
    input  wire                                      clk,
    input  wire                                      rst_n,
    input  wire                                      in_sof,
    input  wire                                      in_valid,
    input  wire [7:0]                                in_px0,
    input  wire [7:0]                                in_px1,

    // Raw per-band coefficient events.  Several bands can assert together;
    // each asserted lane advances that band's own row-major stream by one.
    // A direct group collector completes a group on every sixteenth push and
    // resolves simultaneous completions in ascending band-ID order.
    output logic [11:0]                              out_raw_valid,
    output logic signed [11:0][DATA_WIDTH-1:0]       out_raw_value,
    output logic                                     out_ll2_valid,
    output logic                                     out_ll2_first,
    output logic                                     out_ll2_last,
    output logic [COORD_WIDTH-1:0]                   out_ll2_pair_x,
    output logic [COORD_WIDTH-1:0]                   out_ll2_y,
    output logic signed [1:0][DATA_WIDTH-1:0]        out_ll2_value,
    output logic                                     out_h4_low_valid,
    output logic                                     out_h4_low_first,
    output logic                                     out_h4_low_last,
    output logic [COORD_WIDTH-1:0]                   out_h4_low_pair_x,
    output logic [COORD_WIDTH-1:0]                   out_h4_low_y,
    output logic signed [1:0][DATA_WIDTH-1:0]        out_h4_low_value,
    output logic                                     out_ll4_valid,
    output logic                                     out_ll4_first,
    output logic                                     out_ll4_last,
    output logic [COORD_WIDTH-1:0]                   out_ll4_pair_x,
    output logic [COORD_WIDTH-1:0]                   out_ll4_y,
    output logic signed [1:0][DATA_WIDTH-1:0]        out_ll4_value,
    output logic                                     frame_done,
    output logic                                     protocol_error
);
    // The complete finite 1280x800 byte-raster CDF 5/3 composition is
    // bounded by [-975,976] across every emitted coefficient and intermediate
    // stage. Retain the public signed14 interface and narrow only this proved
    // profile. CameraPipeline admits valid raster prefixes from processing
    // reset, then either drains the frame or resets on a source fault.
    localparam bit NARROW_BYTE_RASTER =
        BOUNDED_BYTE_RASTER && FRAME_WIDTH == 1280 && FRAME_HEIGHT == 800
        && DATA_WIDTH == 14;
    localparam integer INTERNAL_WIDTH = NARROW_BYTE_RASTER ? 11 : DATA_WIDTH;
    wire signed [11:0][INTERNAL_WIDTH-1:0] narrow_out_raw_value;
    wire signed [1:0][INTERNAL_WIDTH-1:0] narrow_out_ll2_value;
    wire signed [1:0][INTERNAL_WIDTH-1:0] narrow_out_h4_low_value;
    wire signed [1:0][INTERNAL_WIDTH-1:0] narrow_out_ll4_value;
    forward_wavelet_transform_core #(
        .FRAME_WIDTH(FRAME_WIDTH),.FRAME_HEIGHT(FRAME_HEIGHT),
        .DATA_WIDTH(INTERNAL_WIDTH),.COORD_WIDTH(COORD_WIDTH),
        .NARROW_BYTE_RASTER(NARROW_BYTE_RASTER)
    ) core (
        .clk(clk),
        .rst_n(rst_n),
        .in_sof(in_sof),
        .in_valid(in_valid),
        .in_px0(in_px0),
        .in_px1(in_px1),
        .out_raw_valid(out_raw_valid),
        .out_raw_value(narrow_out_raw_value),
        .out_ll2_valid(out_ll2_valid),
        .out_ll2_first(out_ll2_first),
        .out_ll2_last(out_ll2_last),
        .out_ll2_pair_x(out_ll2_pair_x),
        .out_ll2_y(out_ll2_y),
        .out_ll2_value(narrow_out_ll2_value),
        .out_h4_low_valid(out_h4_low_valid),
        .out_h4_low_first(out_h4_low_first),
        .out_h4_low_last(out_h4_low_last),
        .out_h4_low_pair_x(out_h4_low_pair_x),
        .out_h4_low_y(out_h4_low_y),
        .out_h4_low_value(narrow_out_h4_low_value),
        .out_ll4_valid(out_ll4_valid),
        .out_ll4_first(out_ll4_first),
        .out_ll4_last(out_ll4_last),
        .out_ll4_pair_x(out_ll4_pair_x),
        .out_ll4_y(out_ll4_y),
        .out_ll4_value(narrow_out_ll4_value),
        .frame_done(frame_done),
        .protocol_error(protocol_error)
    );
    assign out_raw_value[0] = DATA_WIDTH'($signed(narrow_out_raw_value[0]));
    assign out_raw_value[1] = DATA_WIDTH'($signed(narrow_out_raw_value[1]));
    assign out_raw_value[2] = DATA_WIDTH'($signed(narrow_out_raw_value[2]));
    assign out_raw_value[3] = DATA_WIDTH'($signed(narrow_out_raw_value[3]));
    assign out_raw_value[4] = DATA_WIDTH'($signed(narrow_out_raw_value[4]));
    assign out_raw_value[5] = DATA_WIDTH'($signed(narrow_out_raw_value[5]));
    assign out_raw_value[6] = DATA_WIDTH'($signed(narrow_out_raw_value[6]));
    assign out_raw_value[7] = DATA_WIDTH'($signed(narrow_out_raw_value[7]));
    assign out_raw_value[8] = DATA_WIDTH'($signed(narrow_out_raw_value[8]));
    assign out_raw_value[9] = DATA_WIDTH'($signed(narrow_out_raw_value[9]));
    assign out_raw_value[10] = DATA_WIDTH'($signed(narrow_out_raw_value[10]));
    assign out_raw_value[11] = DATA_WIDTH'($signed(narrow_out_raw_value[11]));
    assign out_ll2_value[0] = DATA_WIDTH'($signed(narrow_out_ll2_value[0]));
    assign out_ll2_value[1] = DATA_WIDTH'($signed(narrow_out_ll2_value[1]));
    assign out_h4_low_value[0] = DATA_WIDTH'($signed(narrow_out_h4_low_value[0]));
    assign out_h4_low_value[1] = DATA_WIDTH'($signed(narrow_out_h4_low_value[1]));
    assign out_ll4_value[0] = DATA_WIDTH'($signed(narrow_out_ll4_value[0]));
    assign out_ll4_value[1] = DATA_WIDTH'($signed(narrow_out_ll4_value[1]));
endmodule

`default_nettype none

// Complete fixed-point 5H/3V forward Mallat transform. The only output is
// the raw per-band event stream consumed by the scalar frontend.
module forward_wavelet_transform_core #(
    parameter integer FRAME_WIDTH = 1280,
    parameter integer FRAME_HEIGHT = 800,
    parameter integer DATA_WIDTH = 14,
    parameter integer COORD_WIDTH = 10,
    parameter bit NARROW_BYTE_RASTER = 0
) (
    input  wire                                      clk,
    input  wire                                      rst_n,
    input  wire                                      in_sof,
    input  wire                                      in_valid,
    input  wire [7:0]                                in_px0,
    input  wire [7:0]                                in_px1,

    // Raw per-band coefficient events.  Several bands can assert together;
    // each asserted lane advances that band's own row-major stream by one.
    // A direct group collector completes a group on every sixteenth push and
    // resolves simultaneous completions in ascending band-ID order.
    output logic [11:0]                              out_raw_valid,
    output logic signed [11:0][DATA_WIDTH-1:0]       out_raw_value,
    output logic                                     out_ll2_valid,
    output logic                                     out_ll2_first,
    output logic                                     out_ll2_last,
    output logic [COORD_WIDTH-1:0]                   out_ll2_pair_x,
    output logic [COORD_WIDTH-1:0]                   out_ll2_y,
    output logic signed [1:0][DATA_WIDTH-1:0]        out_ll2_value,
    output logic                                     out_h4_low_valid,
    output logic                                     out_h4_low_first,
    output logic                                     out_h4_low_last,
    output logic [COORD_WIDTH-1:0]                   out_h4_low_pair_x,
    output logic [COORD_WIDTH-1:0]                   out_h4_low_y,
    output logic signed [1:0][DATA_WIDTH-1:0]        out_h4_low_value,
    output logic                                     out_ll4_valid,
    output logic                                     out_ll4_first,
    output logic                                     out_ll4_last,
    output logic [COORD_WIDTH-1:0]                   out_ll4_pair_x,
    output logic [COORD_WIDTH-1:0]                   out_ll4_y,
    output logic signed [1:0][DATA_WIDTH-1:0]        out_ll4_value,
    output logic                                     frame_done,
    output logic                                     protocol_error
);
    localparam integer H1_WIDTH = FRAME_WIDTH / 2;
    localparam integer H2_WIDTH = FRAME_WIDTH / 4;
    localparam integer H3_WIDTH = FRAME_WIDTH / 8;
    localparam integer H4_WIDTH = FRAME_WIDTH / 16;
    localparam integer H5_WIDTH = FRAME_WIDTH / 32;
    localparam integer V1_HEIGHT = FRAME_HEIGHT / 2;
    localparam integer V2_HEIGHT = FRAME_HEIGHT / 4;
    localparam integer V3_HEIGHT = FRAME_HEIGHT / 8;
    localparam integer V1_ROW_WIDTH = $clog2(V1_HEIGHT);
    localparam integer V2_ROW_WIDTH = $clog2(V2_HEIGHT);
    localparam integer LOWER_STATE_DEPTH =
        1 << $clog2(2 * H2_WIDTH + 2 * H3_WIDTH);
    localparam integer V1_ADDRESS_WIDTH = (H1_WIDTH <= 1) ? 1 : $clog2(H1_WIDTH);
    localparam integer LOWER_ADDRESS_WIDTH =
        (LOWER_STATE_DEPTH <= 1) ? 1 : $clog2(LOWER_STATE_DEPTH);
    localparam integer LOWER_USED_COLUMN_WIDTH = $clog2(H2_WIDTH);
    localparam integer LOCAL_ROW_BANK_DEPTH = 1 << $clog2(H3_WIDTH);
    localparam integer LOCAL_ROW_MEMORY_DEPTH = 2 * LOCAL_ROW_BANK_DEPTH;
    localparam integer LOCAL_ROW_ADDRESS_WIDTH = $clog2(LOCAL_ROW_MEMORY_DEPTH);
    // H2 occupies 0..H2_WIDTH/2-1 and H3 occupies the first H3_WIDTH
    // locations of the upper bank.  The production profile therefore leaves
    // 96 upper-bank words free; retain one 40-word H4-low row there for the
    // LL4 adjacent-row average without growing the physical memory.
    localparam integer H4_AVERAGE_ROW_BASE = LOCAL_ROW_BANK_DEPTH + H3_WIDTH;
    localparam logic [1:0] SERVICE_H2 = 2'd0;
    localparam logic [1:0] SERVICE_H3 = 2'd1;
    localparam logic [1:0] SERVICE_H4 = 2'd2;
    localparam logic [1:0] SERVICE_H5 = 2'd3;

    localparam logic [1:0] OP_EVEN = 2'd0;
    localparam logic [1:0] OP_ODD = 2'd1;
    localparam logic [1:0] OP_ADVANCE = 2'd2;
    localparam logic [1:0] OP_TAIL = 2'd3;

    localparam logic [3:0] FLUSH_IDLE = 4'd0;
    localparam logic [3:0] FLUSH_V1_WAIT = 4'd1;
    localparam logic [3:0] FLUSH_V1_TAIL = 4'd2;
    localparam logic [3:0] FLUSH_V2_INPUT_WAIT = 4'd3;
    localparam logic [3:0] FLUSH_V2_TAIL = 4'd4;
    localparam logic [3:0] FLUSH_V3_INPUT_WAIT = 4'd5;
    localparam logic [3:0] FLUSH_V3_TAIL = 4'd6;
    localparam logic [3:0] FLUSH_FINAL_WAIT = 4'd7;

    function automatic integer band_width(input integer band);
        if (band <= 1)
            band_width = H5_WIDTH;
        else if (band == 2)
            band_width = H4_WIDTH;
        else if (band <= 5)
            band_width = H3_WIDTH;
        else if (band <= 8)
            band_width = H2_WIDTH;
        else
            band_width = H1_WIDTH;
    endfunction

    function automatic integer band_scale(input integer band);
        if (band <= 5)
            band_scale = 8;
        else if (band <= 8)
            band_scale = 4;
        else
            band_scale = 2;
    endfunction

    function automatic integer band_fifo_depth(input integer band);
        if (band >= 9)
            // Exact 1280x800 production trace peaks at 726 queued values in
            // each first-level band.  Three 256-deep RAM5Ks per band leave a
            // 42-value margin without paying for a fourth block.
            band_fifo_depth = 768;
        else if (band >= 6)
            band_fifo_depth = 256;
        else if (band >= 2)
            band_fifo_depth = 128;
        else
            band_fifo_depth = 64;
    endfunction

    function automatic logic signed [8:0] center_pixel(
        input logic [7:0] pixel
    );
        begin
            // Subtracting 128 from an unsigned byte is exactly a sign-bit
            // flip followed by sign extension; no arithmetic is required.
            center_pixel = {~pixel[7],~pixel[7],pixel[6:0]};
        end
    endfunction

    function automatic logic signed [DATA_WIDTH-1:0] rounded_average(
        input logic signed [DATA_WIDTH-1:0] even_value,
        input logic signed [DATA_WIDTH-1:0] odd_value
    );
        logic signed [DATA_WIDTH:0] sum;
        begin
            sum = $signed({even_value[DATA_WIDTH-1],even_value})
                + $signed({odd_value[DATA_WIDTH-1],odd_value});
            sum = sum + {{DATA_WIDTH{1'b0}},1'b1};
            rounded_average = DATA_WIDTH'(sum >>> 1);
        end
    endfunction

    // The two 320-column contexts occupy the low 320 addresses of each
    // 512-word half. The two 160-column contexts use otherwise empty codes
    // in those halves. This is an injective 960-state layout in the same
    // physical 1024x40 RAM, with only concatenation and a mux on the address
    // path rather than three constant-offset adders.
    function automatic logic [LOWER_ADDRESS_WIDTH-1:0] lower_state_address(
        input logic [1:0] context_id,
        input logic [LOWER_ADDRESS_WIDTH-2:0] column
    );
        logic [LOWER_ADDRESS_WIDTH-2:0] sparse_column;
        begin
            if (!context_id[1]) begin
                sparse_column = column[LOWER_ADDRESS_WIDTH-2:0];
            end else if (!column[7]) begin
                sparse_column = {2'b11,column[6:0]};
            end else begin
                sparse_column = {4'b1010,column[4:0]};
            end
            lower_state_address = {context_id[0],sparse_column};
        end
    endfunction

    logic frame_active;
    logic [COORD_WIDTH-1:0] input_pair_x;
    logic [COORD_WIDTH-1:0] input_row;
    logic h1_in_valid;
    logic h1_in_first;
    logic h1_in_last;
    logic signed [8:0] h1_in_even;
    logic signed [8:0] h1_in_odd;
    logic h1_out_valid, unused_h1_out_first, h1_out_last;
    logic signed [DATA_WIDTH-1:0] h1_out_low, h1_out_high;
    logic [COORD_WIDTH-1:0] h1_out_x_count;
    logic [COORD_WIDTH-1:0] h1_out_row_count;

    logic v1_in_valid;
    logic [1:0] v1_in_op;
    logic [V1_ADDRESS_WIDTH-1:0] v1_in_address;
    logic [COORD_WIDTH-1:0] v1_in_column;
    logic [V1_ROW_WIDTH-1:0] v1_in_output_y;
    logic v1_in_first, v1_in_last;
    logic signed [1:0][DATA_WIDTH-1:0] v1_in_value;
    logic v1_out_valid;
    logic [1:0] unused_v1_out_context;
    logic [COORD_WIDTH-1:0] v1_out_column;
    logic [V1_ROW_WIDTH-1:0] v1_out_y;
    logic unused_v1_out_first, unused_v1_out_last;
    logic signed [1:0][DATA_WIDTH-1:0] v1_out_low, v1_out_high;

`ifndef SYNTHESIS
    logic h2_pending_valid;
`endif
    logic signed [DATA_WIDTH-1:0] h2_pending_even;
    logic h2_in_valid;
    logic signed [DATA_WIDTH-1:0] h2_in_even, h2_in_odd;
    logic h2_out_valid, h2_out_last;
    logic signed [DATA_WIDTH-1:0] h2_out_low, h2_out_high;
    logic [V1_ROW_WIDTH-1:0] h2_row_tag;
    logic [$clog2(H2_WIDTH)-1:0] h2_out_x_count;

    (* syn_ramstyle = "block_ram" *)
    logic signed [2*DATA_WIDTH-1:0]
        local_row_memory [0:LOCAL_ROW_MEMORY_DEPTH-1];
    logic signed [2*DATA_WIDTH-1:0] local_row_read_q;
    logic signed [DATA_WIDTH-1:0] shared_pair_value_q;
    logic local_row_write_enable;
    logic [LOCAL_ROW_ADDRESS_WIDTH-1:0] local_row_write_address;
    logic signed [2*DATA_WIDTH-1:0] local_row_write_data;
    logic local_row_read_enable;
    logic [LOCAL_ROW_ADDRESS_WIDTH-1:0] local_row_read_address;
    logic regular_local_row_write_enable;
    logic regular_local_row_read_enable;
    logic h4_average_row_write;
    logic h4_average_row_read;
    logic h4_average_pending_valid;
    logic [$clog2(H5_WIDTH)-1:0] h4_average_pending_pair_x;
    logic [$clog2(H2_WIDTH+1)-1:0] v2_high_count;
    logic v2_high_buffer_ready;
    logic v2_high_requests_done;
    // The retained read column already identifies the last request. Keep
    // only whether this row has started instead of a duplicate position
    // counter. The next index is observable only before requests_done.
    logic v2_high_request_started;
    wire [$clog2(H2_WIDTH)-1:0] v2_high_request_index =
        v2_high_request_started ? v2_high_read_column_q + 1'b1 : '0;
    logic v2_high_read_request;
    logic v2_high_read_valid_q;
    logic [$clog2(H2_WIDTH)-1:0] v2_high_read_column_q;
    logic [V1_ROW_WIDTH-1:0] v2_high_buffer_row;
    logic v2_high_consume;

    logic lower_in_valid;
    logic [1:0] lower_in_op;
    logic [LOWER_ADDRESS_WIDTH-1:0] lower_in_address;
    logic [1:0] lower_in_context;
    logic [LOWER_USED_COLUMN_WIDTH-1:0] lower_in_column;
    logic [V2_ROW_WIDTH-1:0] lower_in_output_y;
    logic lower_in_first, lower_in_last;
    logic signed [0:0][DATA_WIDTH-1:0] lower_in_value;
    logic lower_out_valid;
    logic [1:0] lower_out_context;
    logic [LOWER_USED_COLUMN_WIDTH-1:0] lower_out_column;
    logic [V2_ROW_WIDTH-1:0] lower_out_y;
    logic unused_lower_out_first, unused_lower_out_last;
    logic signed [0:0][DATA_WIDTH-1:0] lower_out_low, lower_out_high;

`ifndef SYNTHESIS
    logic h3_pending_valid;
`endif
    logic signed [DATA_WIDTH-1:0] h3_pending_even;
    logic h3_in_valid;
    logic signed [DATA_WIDTH-1:0] h3_in_even, h3_in_odd;
    logic h3_out_valid, h3_out_last;
    logic signed [DATA_WIDTH-1:0] h3_out_low, h3_out_high;
    logic [V2_ROW_WIDTH-1:0] h3_row_tag;
    logic [$clog2(H3_WIDTH)-1:0] h3_out_x_count;

    logic v3_buffer_ready;
    logic [V2_ROW_WIDTH-1:0] v3_buffer_row;
    logic [$clog2(H3_WIDTH)-1:0] v3_drain_column;
    logic [1:0] v3_drain_phase;

`ifndef SYNTHESIS
    logic h4_pending_valid;
`endif
    logic signed [DATA_WIDTH-1:0] h4_pending_even;
    logic h4_pair_valid;
    logic [$clog2(H4_WIDTH)-1:0] h4_pair_index;
    logic [V2_ROW_WIDTH-1:0] h4_pair_row;
    logic service_out_valid, unused_service_out_first, service_out_last;
    logic [1:0] service_out_kind;
    logic signed [DATA_WIDTH-1:0] service_out_low, service_out_high;
    logic [$clog2(H2_WIDTH)-1:0] service_out_x;
    logic [V1_ROW_WIDTH-1:0] service_out_row;
    logic [V1_ROW_WIDTH-1:0] service_h4_low_pair_x;
    logic [V1_ROW_WIDTH-1:0] service_h4_low_y;
    logic service_idle;
    logic service_error;

    logic [3:0] flush_state;
    logic [COORD_WIDTH-1:0] flush_column;

    logic [11:0] raw_push;
    logic signed [11:0][DATA_WIDTH-1:0] raw_value;

    logic frame_clear;
    always_comb begin
        h1_in_valid = in_valid;
        h1_in_first = in_valid && (in_sof || input_pair_x == 0);
        h1_in_last = in_valid && (in_sof
            ? H1_WIDTH == 1
            : input_pair_x + 1'b1 == COORD_WIDTH'(H1_WIDTH));
        h1_in_even = center_pixel(in_px0);
        h1_in_odd = center_pixel(in_px1);
        frame_clear = in_sof && in_valid;
    end

    first_horizontal_lift #(.OUTPUT_WIDTH(NARROW_BYTE_RASTER ? DATA_WIDTH : 14)) h1 (
        .clk, .rst_n,
        .in_valid(h1_in_valid), .in_first(h1_in_first), .in_last(h1_in_last),
        .in_even(h1_in_even), .in_odd(h1_in_odd),
        .out_valid(h1_out_valid), .out_first(unused_h1_out_first),
        .out_last(h1_out_last),
        .out_low(h1_out_low), .out_high(h1_out_high)
    );

    always_comb begin
        v1_in_valid = 1'b0;
        v1_in_op = OP_EVEN;
        v1_in_address = V1_ADDRESS_WIDTH'(h1_out_x_count);
        v1_in_column = h1_out_x_count;
        v1_in_output_y = '0;
        v1_in_first = 1'b0;
        v1_in_last = 1'b0;
        v1_in_value[0] = h1_out_low;
        v1_in_value[1] = h1_out_high;

        if (h1_out_valid) begin
            v1_in_valid = 1'b1;
            if (h1_out_row_count[0]) begin
                v1_in_op = OP_ODD;
            end else if (h1_out_row_count == 0) begin
                v1_in_op = OP_EVEN;
            end else begin
                v1_in_op = OP_ADVANCE;
                v1_in_output_y = h1_out_row_count[COORD_WIDTH-1:1]
                    - V1_ROW_WIDTH'(1);
                v1_in_first = h1_out_row_count == 2;
            end
        end else if (flush_state == FLUSH_V1_TAIL) begin
            v1_in_valid = 1'b1;
            v1_in_op = OP_TAIL;
            v1_in_address = V1_ADDRESS_WIDTH'(flush_column);
            v1_in_column = flush_column;
            v1_in_output_y = V1_ROW_WIDTH'(V1_HEIGHT - 1);
            v1_in_last = 1'b1;
            v1_in_value = '0;
        end
    end

    first_vertical_lift #(.OUTPUT_WIDTH(NARROW_BYTE_RASTER ? DATA_WIDTH : 14)) v1 (
        .clk, .rst_n, .in_valid(v1_in_valid), .in_op(v1_in_op),
        .in_state_address(v1_in_address), .in_context(2'd0),
        .in_column(v1_in_column), .in_output_y(v1_in_output_y),
        .in_output_first(v1_in_first), .in_output_last(v1_in_last),
        .in_value(v1_in_value),
        .out_valid(v1_out_valid), .out_context(unused_v1_out_context),
        .out_column(v1_out_column), .out_y(v1_out_y),
        .out_first(unused_v1_out_first), .out_last(unused_v1_out_last),
        .out_low(v1_out_low), .out_high(v1_out_high)
    );

    always_comb begin
        h2_in_valid = v1_out_valid && v1_out_column[0];
        h2_in_even = h2_pending_even;
        h2_in_odd = v1_out_low[0];
        h2_out_valid = service_out_valid && service_out_kind == SERVICE_H2;
        h2_out_last = service_out_last;
        h2_out_low = service_out_low;
        h2_out_high = service_out_high;
        h2_row_tag = service_out_row;
        h2_out_x_count = service_out_x;
    end

    always_comb begin
        lower_in_valid = 1'b0;
        lower_in_op = OP_EVEN;
        lower_in_address = '0;
        lower_in_context = '0;
        lower_in_column = '0;
        lower_in_output_y = '0;
        lower_in_first = 1'b0;
        lower_in_last = 1'b0;
        lower_in_value = '0;

        if (flush_state == FLUSH_V2_TAIL) begin
            lower_in_valid = 1'b1;
            lower_in_context = flush_column[0] ? 2'd1 : 2'd0;
            lower_in_column = LOWER_USED_COLUMN_WIDTH'(flush_column >> 1);
            lower_in_op = OP_TAIL;
            lower_in_output_y = V2_ROW_WIDTH'(V2_HEIGHT - 1);
            lower_in_last = 1'b1;
        end else if (flush_state == FLUSH_V3_TAIL) begin
            lower_in_valid = 1'b1;
            lower_in_context = flush_column[0] ? 2'd3 : 2'd2;
            lower_in_column = LOWER_USED_COLUMN_WIDTH'(flush_column >> 1);
            lower_in_op = OP_TAIL;
            lower_in_output_y = V2_ROW_WIDTH'(V3_HEIGHT - 1);
            lower_in_last = 1'b1;
        end else if (h2_out_valid) begin
            lower_in_valid = 1'b1;
            lower_in_context = 2'd0;
            lower_in_column = LOWER_USED_COLUMN_WIDTH'(h2_out_x_count);
            lower_in_value[0] = h2_out_low;
            if (h2_row_tag[0]) begin
                lower_in_op = OP_ODD;
            end else if (h2_row_tag == 0) begin
                lower_in_op = OP_EVEN;
            end else begin
                lower_in_op = OP_ADVANCE;
                lower_in_output_y = h2_row_tag[V1_ROW_WIDTH-1:1]
                    - V2_ROW_WIDTH'(1);
                lower_in_first = h2_row_tag == 2;
            end
        end else if (v2_high_read_valid_q) begin
            lower_in_valid = 1'b1;
            lower_in_context = 2'd1;
            lower_in_column = v2_high_read_column_q;
            lower_in_value[0] = v2_high_read_column_q[0]
                ? local_row_read_q[2*DATA_WIDTH-1:DATA_WIDTH]
                : local_row_read_q[DATA_WIDTH-1:0];
            if (v2_high_buffer_row[0]) begin
                lower_in_op = OP_ODD;
            end else if (v2_high_buffer_row == 0) begin
                lower_in_op = OP_EVEN;
            end else begin
                lower_in_op = OP_ADVANCE;
                lower_in_output_y = v2_high_buffer_row[V1_ROW_WIDTH-1:1]
                    - V2_ROW_WIDTH'(1);
                lower_in_first = v2_high_buffer_row == 2;
            end
        end else if (v3_drain_phase == 2'd2 || v3_drain_phase == 2'd3) begin
            lower_in_valid = 1'b1;
            lower_in_context = v3_drain_phase == 2'd2 ? 2'd2 : 2'd3;
            lower_in_column = LOWER_USED_COLUMN_WIDTH'(v3_drain_column);
            lower_in_value[0] = v3_drain_phase == 2'd2
                ? local_row_read_q[DATA_WIDTH-1:0]
                : local_row_read_q[2*DATA_WIDTH-1:DATA_WIDTH];
            if (v3_buffer_row[0]) begin
                lower_in_op = OP_ODD;
            end else if (v3_buffer_row == 0) begin
                lower_in_op = OP_EVEN;
            end else begin
                lower_in_op = OP_ADVANCE;
                lower_in_output_y = V2_ROW_WIDTH'(
                    v3_buffer_row[V2_ROW_WIDTH-1:1]
                ) - V2_ROW_WIDTH'(1);
                lower_in_first = v3_buffer_row == 2;
            end
        end
        lower_in_address = lower_state_address(
            lower_in_context,
            lower_in_column[LOWER_ADDRESS_WIDTH-2:0]
        );
    end

    always_comb begin
        v2_high_consume = lower_in_valid && lower_in_context == 1
            && flush_state != FLUSH_V2_TAIL;
        v2_high_read_request = v2_high_buffer_ready
            && !v2_high_requests_done
            && (!v2_high_read_valid_q || v2_high_consume)
            && flush_state != FLUSH_V2_TAIL && flush_state != FLUSH_V3_TAIL;
    end

    vertical_lift_scheduler #(
        .LANES(1), .DATA_WIDTH(DATA_WIDTH),
        // Every V2/V3 raw input fits signed 13 bits.  V3's previous high
        // retains the full 14-bit datapath bound.
        .RAW_WIDTH(NARROW_BYTE_RASTER ? DATA_WIDTH : 13),
        .HIGH_WIDTH(NARROW_BYTE_RASTER ? DATA_WIDTH : 14),
        .STATE_DEPTH(LOWER_STATE_DEPTH), .ADDRESS_WIDTH(LOWER_ADDRESS_WIDTH),
        .CONTEXT_WIDTH(2), .COLUMN_WIDTH(LOWER_USED_COLUMN_WIDTH),
        .ROW_WIDTH(V2_ROW_WIDTH)
    ) lower_vertical (
        .clk, .rst_n, .in_valid(lower_in_valid), .in_op(lower_in_op),
        .in_state_address(lower_in_address), .in_context(lower_in_context),
        .in_column(lower_in_column), .in_output_y(lower_in_output_y),
        .in_output_first(lower_in_first), .in_output_last(lower_in_last),
        .in_value(lower_in_value),
        .out_valid(lower_out_valid), .out_context(lower_out_context),
        .out_column(lower_out_column),
        .out_y(lower_out_y),
        .out_first(unused_lower_out_first), .out_last(unused_lower_out_last),
        .out_low(lower_out_low), .out_high(lower_out_high)
    );

    always_comb begin
        h3_in_valid = lower_out_valid && lower_out_context == 0
            && lower_out_column[0];
        h3_in_even = h3_pending_even;
        h3_in_odd = lower_out_low[0];
        h3_out_valid = service_out_valid && service_out_kind == SERVICE_H3;
        h3_out_last = service_out_last;
        h3_out_low = service_out_low;
        h3_out_high = service_out_high;
        h3_row_tag = service_out_row[V2_ROW_WIDTH-1:0];
        h3_out_x_count = service_out_x[$clog2(H3_WIDTH)-1:0];
    end

    always_comb begin
        h4_pair_valid = lower_out_valid && lower_out_context == 2
            && lower_out_column[0];
        h4_pair_index = lower_out_column[$clog2(H4_WIDTH):1];
        h4_pair_row = lower_out_y;
    end

    horizontal_lift_row_service #(
        .DATA_WIDTH(DATA_WIDTH), .RAW_WIDTH(NARROW_BYTE_RASTER ? 10 : DATA_WIDTH),
        .COORD_WIDTH(V1_ROW_WIDTH),
        .H2_PAIR_COUNT(H2_WIDTH), .H3_PAIR_COUNT(H3_WIDTH),
        .H4_PAIR_COUNT(H4_WIDTH), .H5_PAIR_COUNT(H5_WIDTH)
    ) lower_horizontal_service (
        .clk, .rst_n, .clear(frame_clear),
        .h2_pair_valid(h2_in_valid),
        .h2_pair_index(v1_out_column[$clog2(H2_WIDTH):1]),
        .h2_pair_even(h2_in_even), .h2_pair_odd(h2_in_odd),
        // The valid odd beat closes the adjacent pair and necessarily carries
        // the same row tag as its retained even predecessor.
        .h2_pair_row(v1_out_y),
        .h2_sink_ready(v2_high_count == 0 && !v2_high_buffer_ready
            && !v2_high_read_valid_q),
        .h3_pair_valid(h3_in_valid),
        .h3_pair_index(lower_out_column[$clog2(H3_WIDTH):1]),
        .h3_pair_even(h3_in_even), .h3_pair_odd(h3_in_odd),
        .h3_pair_row(V1_ROW_WIDTH'(lower_out_y)),
        .h3_sink_ready(!v3_buffer_ready),
        .h4_pair_valid, .h4_pair_index,
        .h4_pair_even(h4_pending_even), .h4_pair_odd(lower_out_low[0]),
        .h4_pair_row(V1_ROW_WIDTH'(h4_pair_row)),
        .out_valid(service_out_valid), .out_kind(service_out_kind),
        .out_first(unused_service_out_first), .out_last(service_out_last),
        .out_low(service_out_low), .out_high(service_out_high),
        .out_x(service_out_x), .out_row(service_out_row),
        .out_h4_low_pair_valid(out_h4_low_valid),
        .out_h4_low_pair_first(out_h4_low_first),
        .out_h4_low_pair_last(out_h4_low_last),
        .out_h4_low_pair_x(service_h4_low_pair_x),
        .out_h4_low_pair_y(service_h4_low_y),
        .out_h4_low_pair_value(out_h4_low_value),
        .idle(service_idle), .protocol_error(service_error)
    );

    always_comb begin
        out_h4_low_pair_x = COORD_WIDTH'(service_h4_low_pair_x);
        out_h4_low_y = COORD_WIDTH'(service_h4_low_y);
    end

    // Pack adjacent H2 highs into one 28-bit word and place the complete H3
    // row in the other half of the same physical memory.  The unused tail of
    // the H3 bank stores one 40-word H4-low row for LL4 averaging. H2, H3 and
    // H4-low writes are mutually exclusive shared-leaf outputs. V2 reads
    // repeat each word address for its low and high halves, preserving one
    // sample per clock. The fixed production schedule leaves the read port
    // idle on every odd-row H4-low event; a sticky guard and simulation
    // assertion below make that required port schedule fail-visible.
    always_comb begin
        h4_average_row_write = out_h4_low_valid && !service_h4_low_y[0];
        h4_average_row_read = out_h4_low_valid && service_h4_low_y[0];

        regular_local_row_write_enable = h2_out_valid && h2_out_x_count[0];
        local_row_write_enable = regular_local_row_write_enable;
        local_row_write_address = LOCAL_ROW_ADDRESS_WIDTH'(
            h2_out_x_count >> 1
        );
        local_row_write_data = {h2_out_high, shared_pair_value_q};
        if (h3_out_valid) begin
            regular_local_row_write_enable = 1'b1;
            local_row_write_enable = 1'b1;
            local_row_write_address = LOCAL_ROW_ADDRESS_WIDTH'(
                LOCAL_ROW_BANK_DEPTH + h3_out_x_count
            );
            local_row_write_data = {h3_out_high, h3_out_low};
        end
        if (h4_average_row_write) begin
            local_row_write_enable = 1'b1;
            local_row_write_address = LOCAL_ROW_ADDRESS_WIDTH'(
                H4_AVERAGE_ROW_BASE + service_h4_low_pair_x
            );
            local_row_write_data = out_h4_low_value;
        end

        regular_local_row_read_enable = v2_high_read_request;
        local_row_read_enable = regular_local_row_read_enable;
        local_row_read_address = LOCAL_ROW_ADDRESS_WIDTH'(
            v2_high_request_index >> 1
        );
        if (!v2_high_read_request && v3_drain_phase == 2'd1
                && !h2_out_valid && v2_high_count == 0
                && !(flush_state == FLUSH_V2_TAIL
                    || flush_state == FLUSH_V3_TAIL)) begin
            regular_local_row_read_enable = 1'b1;
            local_row_read_enable = 1'b1;
            local_row_read_address = LOCAL_ROW_ADDRESS_WIDTH'(
                LOCAL_ROW_BANK_DEPTH + v3_drain_column
            );
        end
        if (h4_average_row_read) begin
            local_row_read_enable = 1'b1;
            local_row_read_address = LOCAL_ROW_ADDRESS_WIDTH'(
                H4_AVERAGE_ROW_BASE + service_h4_low_pair_x
            );
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            shared_pair_value_q <= '0;
        else if (frame_clear)
            shared_pair_value_q <= '0;
        else if (h2_out_valid && !h2_out_x_count[0])
            shared_pair_value_q <= h2_out_high;
        else if (h4_average_row_read)
            // H2 and H4 rows are serialized by the shared horizontal
            // service.  The H2 even-half lifetime ends at its following odd
            // output, while this H4 odd half is consumed by the synchronous
            // average response one cycle later.  These lifetimes therefore
            // cannot overlap, even when either stream contains bubbles.
            shared_pair_value_q <= out_h4_low_value[1];
    end

    always_ff @(posedge clk) begin
        if (local_row_write_enable)
            local_row_memory[local_row_write_address] <= local_row_write_data;
        if (local_row_read_enable)
            local_row_read_q <= local_row_memory[local_row_read_address];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h4_average_pending_valid <= 1'b0;
            h4_average_pending_pair_x <= '0;
        end else if (frame_clear) begin
            h4_average_pending_valid <= 1'b0;
            h4_average_pending_pair_x <= '0;
        end else begin
            h4_average_pending_valid <= h4_average_row_read;
            if (h4_average_row_read) begin
                h4_average_pending_pair_x
                    <= service_h4_low_pair_x[$clog2(H5_WIDTH)-1:0];
            end
        end
    end

    always_comb begin
        out_ll4_valid = h4_average_pending_valid;
        // pair_x is already retained for the synchronous row-memory result;
        // derive its boundary flags instead of delaying duplicate bits.
        out_ll4_first = h4_average_pending_valid
            && h4_average_pending_pair_x == 0;
        out_ll4_last = h4_average_pending_valid
            && h4_average_pending_pair_x + 1'b1
                == $clog2(H5_WIDTH)'(H5_WIDTH);
        out_ll4_pair_x = COORD_WIDTH'(h4_average_pending_pair_x);
        out_ll4_y = COORD_WIDTH'(service_h4_low_y >> 1);
        out_ll4_value[0] = rounded_average(
            $signed(local_row_read_q[DATA_WIDTH-1:0]),
            $signed(out_h4_low_value[0])
        );
        out_ll4_value[1] = rounded_average(
            $signed(local_row_read_q[2*DATA_WIDTH-1:DATA_WIDTH]),
            $signed(shared_pair_value_q)
        );
    end

    always_comb begin
        raw_push = '0;
        // Each value lane has one fixed producer.  Drive the value bus
        // continuously and use raw_push as the sole admission qualifier;
        // zeroing 168 inactive data bits only adds mux fabric and cannot be
        // observed by a valid-gated consumer.
        raw_value[0] = service_out_low;
        raw_value[1] = service_out_high;
        raw_value[2] = service_out_high;
        raw_value[3] = lower_out_high[0];
        raw_value[4] = lower_out_low[0];
        raw_value[5] = lower_out_high[0];
        raw_value[6] = lower_out_high[0];
        raw_value[7] = lower_out_low[0];
        raw_value[8] = lower_out_high[0];
        raw_value[9] = v1_out_high[0];
        raw_value[10] = v1_out_low[1];
        raw_value[11] = v1_out_high[1];

        if (v1_out_valid) begin
            raw_push[9] = 1'b1;
            raw_push[10] = 1'b1;
            raw_push[11] = 1'b1;
        end
        if (lower_out_valid && lower_out_context == 0) begin
            raw_push[6] = 1'b1;
        end
        if (lower_out_valid && lower_out_context == 1) begin
            raw_push[7] = 1'b1;
            raw_push[8] = 1'b1;
        end
        if (lower_out_valid && lower_out_context == 2) begin
            raw_push[3] = 1'b1;
        end
        if (lower_out_valid && lower_out_context == 3) begin
            raw_push[4] = 1'b1;
            raw_push[5] = 1'b1;
        end
        if (service_out_valid && service_out_kind == SERVICE_H4) begin
            raw_push[2] = 1'b1;
        end
        if (service_out_valid && service_out_kind == SERVICE_H5) begin
            raw_push[0] = 1'b1;
            raw_push[1] = 1'b1;
        end

        out_raw_valid = raw_push;
        out_raw_value = raw_value;
    end

    // Reuse the two gearboxes already required by the deeper Mallat levels.
    // pair_x is a pair index: lane zero is scalar column 2*pair_x and lane
    // one is the following scalar column. Both streams are row-major.
    always_comb begin
        out_ll2_valid = h3_in_valid;
        out_ll2_first = h3_in_valid && lower_out_column == 1;
        out_ll2_last = h3_in_valid
            && lower_out_column + 1'b1
                == LOWER_USED_COLUMN_WIDTH'(H2_WIDTH);
        out_ll2_pair_x = COORD_WIDTH'(lower_out_column >> 1);
        out_ll2_y = COORD_WIDTH'(lower_out_y);
        out_ll2_value[0] = h3_pending_even;
        out_ll2_value[1] = lower_out_low[0];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_active <= 1'b0;
            input_pair_x <= '0;
            input_row <= '0;
            h1_out_x_count <= '0;
            h1_out_row_count <= '0;
            h2_pending_even <= '0;
            v2_high_count <= '0;
            v2_high_buffer_ready <= 1'b0;
            v2_high_requests_done <= 1'b0;
            v2_high_request_started <= 1'b0;
            v2_high_read_valid_q <= 1'b0;
            v2_high_read_column_q <= '0;
            v2_high_buffer_row <= '0;
            h3_pending_even <= '0;
            v3_buffer_ready <= 1'b0;
            v3_buffer_row <= '0;
            v3_drain_column <= '0;
            v3_drain_phase <= '0;
            h4_pending_even <= '0;
            flush_state <= FLUSH_IDLE;
            flush_column <= '0;
            frame_done <= 1'b0;
            protocol_error <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            if (frame_clear) begin
                frame_active <= 1'b1;
                input_pair_x <= H1_WIDTH == 1 ? '0 : 1;
                input_row <= '0;
                h1_out_x_count <= '0;
                h1_out_row_count <= '0;
                flush_state <= FLUSH_IDLE;
                v2_high_count <= '0;
                v2_high_buffer_ready <= 1'b0;
                v2_high_requests_done <= 1'b0;
                v2_high_request_started <= 1'b0;
                v2_high_read_valid_q <= 1'b0;
                protocol_error <= 1'b0;
            end else if (in_valid) begin
                if (!frame_active || in_sof)
                    protocol_error <= 1'b1;
                if (input_pair_x + 1'b1 == COORD_WIDTH'(H1_WIDTH)) begin
                    input_pair_x <= '0;
                    if (input_row + 1'b1 == COORD_WIDTH'(FRAME_HEIGHT))
                        frame_active <= 1'b0;
                    else
                        input_row <= input_row + 1'b1;
                end else begin
                    input_pair_x <= input_pair_x + 1'b1;
                end
            end

            if (h1_out_valid) begin
                if (h1_out_last) begin
                    h1_out_x_count <= '0;
                    if (h1_out_row_count + 1'b1
                            == COORD_WIDTH'(FRAME_HEIGHT)) begin
                        flush_state <= FLUSH_V1_WAIT;
                        flush_column <= COORD_WIDTH'(31);
                    end else begin
                        h1_out_row_count <= h1_out_row_count + 1'b1;
                    end
                end else begin
                    h1_out_x_count <= h1_out_x_count + 1'b1;
                end
            end

            if (v1_out_valid) begin
                if (!v1_out_column[0]) begin
                    h2_pending_even <= v1_out_low[0];
                end
            end
            if (h2_out_valid) begin
                if (h2_out_last) begin
                    v2_high_buffer_ready <= 1'b1;
                    v2_high_buffer_row <= h2_row_tag;
                end
            end
            if (v2_high_read_request) begin
                v2_high_request_started <= 1'b1;
                v2_high_read_valid_q <= 1'b1;
                v2_high_read_column_q <= v2_high_request_index;
                if (v2_high_request_index + 1'b1
                        == $clog2(H2_WIDTH)'(H2_WIDTH)) begin
                    v2_high_requests_done <= 1'b1;
                end
            end else if (v2_high_consume) begin
                v2_high_read_valid_q <= 1'b0;
            end
            if (v2_high_consume && v2_high_read_column_q + 1'b1
                    == $clog2(H2_WIDTH)'(H2_WIDTH)) begin
                v2_high_buffer_ready <= 1'b0;
                v2_high_requests_done <= 1'b0;
                v2_high_request_started <= 1'b0;
            end
            case ({h2_out_valid, v2_high_consume})
                2'b10: v2_high_count <= v2_high_count + 1'b1;
                2'b01: v2_high_count <= v2_high_count - 1'b1;
                default: begin
                end
            endcase

            if (lower_out_valid && lower_out_context == 0) begin
                if (!lower_out_column[0]) begin
                    h3_pending_even <= lower_out_low[0];
                end
            end
            if (h3_out_valid) begin
                if (h3_out_last) begin
                    v3_buffer_ready <= 1'b1;
                    v3_buffer_row <= h3_row_tag;
                    v3_drain_column <= '0;
                    v3_drain_phase <= 2'd1;
                end
            end

            if (v3_drain_phase == 2'd1 && !h2_out_valid && v2_high_count == 0
                && !(flush_state == FLUSH_V2_TAIL
                    || flush_state == FLUSH_V3_TAIL)) begin
                v3_drain_phase <= 2'd2;
            end else if (v3_drain_phase == 2'd2 && lower_in_valid
                && lower_in_context == 2'd2) begin
                v3_drain_phase <= 2'd3;
            end else if (v3_drain_phase == 2'd3 && lower_in_valid
                && lower_in_context == 2'd3) begin
                if (v3_drain_column + 1'b1
                        == $clog2(H3_WIDTH)'(H3_WIDTH)) begin
                    v3_drain_phase <= '0;
                    v3_buffer_ready <= 1'b0;
                end else begin
                    v3_drain_column <= v3_drain_column + 1'b1;
                    v3_drain_phase <= 2'd1;
                end
            end

            if (lower_out_valid && lower_out_context == 2) begin
                if (!lower_out_column[0]) begin
                    h4_pending_even <= lower_out_low[0];
                end
            end

            case (flush_state)
                FLUSH_IDLE: begin
                end
                FLUSH_V1_WAIT: begin
                    if (flush_column == 0 && service_idle) begin
                        flush_state <= FLUSH_V1_TAIL;
                        flush_column <= '0;
                    end else if (flush_column != 0) begin
                        flush_column <= flush_column - 1'b1;
                    end
                end
                FLUSH_V1_TAIL: begin
                    if (flush_column + 1'b1
                            == COORD_WIDTH'(H1_WIDTH)) begin
                        flush_state <= FLUSH_V2_INPUT_WAIT;
                        flush_column <= COORD_WIDTH'(767);
                    end else begin
                        flush_column <= flush_column + 1'b1;
                    end
                end
                FLUSH_V2_INPUT_WAIT: begin
                    if (flush_column == 0 && service_idle
                        && v2_high_count == 0 && !v3_buffer_ready) begin
                        flush_state <= FLUSH_V2_TAIL;
                        flush_column <= '0;
                    end else if (flush_column != 0) begin
                        flush_column <= flush_column - 1'b1;
                    end
                end
                FLUSH_V2_TAIL: begin
                    if (flush_column + 1'b1
                            == COORD_WIDTH'(2 * H2_WIDTH)) begin
                        flush_state <= FLUSH_V3_INPUT_WAIT;
                        flush_column <= COORD_WIDTH'(767);
                    end else begin
                        flush_column <= flush_column + 1'b1;
                    end
                end
                FLUSH_V3_INPUT_WAIT: begin
                    if (flush_column == 0 && service_idle
                        && !v3_buffer_ready) begin
                        flush_state <= FLUSH_V3_TAIL;
                        flush_column <= '0;
                    end else if (flush_column != 0) begin
                        flush_column <= flush_column - 1'b1;
                    end
                end
                FLUSH_V3_TAIL: begin
                    if (flush_column + 1'b1
                            == COORD_WIDTH'(2 * H3_WIDTH)) begin
                        flush_state <= FLUSH_FINAL_WAIT;
                        flush_column <= COORD_WIDTH'(511);
                    end else begin
                        flush_column <= flush_column + 1'b1;
                    end
                end
                FLUSH_FINAL_WAIT: begin
                    if (flush_column == 0 && service_idle) begin
                        flush_state <= FLUSH_IDLE;
                        frame_done <= 1'b1;
                    end else if (flush_column != 0) begin
                        flush_column <= flush_column - 1'b1;
                    end
                end
                default: flush_state <= FLUSH_IDLE;
            endcase

            if (service_error) begin
                protocol_error <= 1'b1;
            end
            if ((h4_average_row_write && regular_local_row_write_enable)
                    || (h4_average_row_read
                        && regular_local_row_read_enable)) begin
                protocol_error <= 1'b1;
            end
        end
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h2_pending_valid <= 1'b0;
            h3_pending_valid <= 1'b0;
            h4_pending_valid <= 1'b0;
        end else if (frame_clear) begin
            h2_pending_valid <= 1'b0;
            h3_pending_valid <= 1'b0;
            h4_pending_valid <= 1'b0;
        end else begin
            if (h2_in_valid)
                assert (h2_pending_valid)
                    else $error("forward_wavelet_transform: H2 gearbox underflow");
            if (h3_in_valid)
                assert (h3_pending_valid)
                    else $error("forward_wavelet_transform: H3 gearbox underflow");
            if (h3_out_valid && h3_out_last)
                assert (!v3_buffer_ready)
                    else $error("forward_wavelet_transform: V3 row buffer overflow");
            if (lower_out_valid && lower_out_context == 2 && lower_out_column[0])
                assert (h4_pending_valid)
                    else $error("forward_wavelet_transform: H4 pair buffer underflow");
            assert (!(h4_average_row_write
                    && regular_local_row_write_enable))
                else $error("forward_wavelet_transform: H4 average write collision");
            assert (!(h4_average_row_read && regular_local_row_read_enable))
                else $error("forward_wavelet_transform: H4 average read collision");
            assert (H4_AVERAGE_ROW_BASE + H5_WIDTH
                    <= LOCAL_ROW_MEMORY_DEPTH)
                else $error("forward_wavelet_transform: H4 average row does not fit");

            if (v1_out_valid)
                h2_pending_valid <= !v1_out_column[0];
            if (lower_out_valid && lower_out_context == 0)
                h3_pending_valid <= !lower_out_column[0];
            if (lower_out_valid && lower_out_context == 2)
                h4_pending_valid <= !lower_out_column[0];
        end
    end
`endif
endmodule

`default_nettype wire
