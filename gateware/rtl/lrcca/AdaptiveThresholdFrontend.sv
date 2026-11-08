`default_nettype none

// Production single-frame TagDetect formatter. RAW uses a causal 13x3 LL2
// box plus the local LL2 coefficient; these reads share the existing row RAM.
// With N=39, S=sum(centered LL2), and d=local centered LL2:
//   T = clamp(round((35*(S+128*N)+8*N*(d+128))/(64*N))+20, 40, 128)
// Cap T by half the rounded horizontal three-tap LL2 mean plus 64.
// This limits bright-background contamination without another image buffer.
// LL2 compares against nearest(mean5x3(LL4))-12; LL4 compares against zero.
// The moment ROM weights dark pixels before CCA, while a packed residual row
// resolves diagonal RAW gaps without assigning centroid mass to added pixels.
//
// Nine tagged rows at each scale retain samples until their delayed composite
// display windows. The coarse scales need the ninth row because their final
// producer bursts precede release of row 191's fourth copy and row 41's
// sixteenth copy. The box windows are centered horizontally and causal
// vertically; every edge replicates its nearest scalar sample.
module AdaptiveThresholdFrontend #(
  parameter int WIDTH = 1280,
  parameter int HEIGHT = 800,
  parameter int PIXEL_BITS = 8,
  parameter int COEFFICIENT_BITS = 14,
  parameter int COORD_WIDTH = 10,
  parameter int STAGE2_Y_OFFSET = 27,
  parameter int STAGE4_Y_OFFSET = 127,
  // Minimum start-to-start service epoch. A slower source may insert idle
  // time between composite rows, but a ready source is serviced at this rate.
  parameter int ROW_CYCLES = 896,
  localparam int SOURCE_Y_BITS = (HEIGHT <= 1) ? 1 : $clog2(HEIGHT)
) (
  input  wire clk,
  input  wire rst,
  input  wire in_vref,
  input  wire in_href,
  input  wire [PIXEL_BITS-1:0] in_d0,
  input  wire [PIXEL_BITS-1:0] in_d1,
  input  wire ll2_valid,
  input  wire ll2_first,
  input  wire ll2_last,
  input  wire [COORD_WIDTH-1:0] ll2_pair_x,
  input  wire [COORD_WIDTH-1:0] ll2_y,
  input  wire signed [1:0][COEFFICIENT_BITS-1:0] ll2_value,
  input  wire ll4_valid,
  input  wire ll4_first,
  input  wire ll4_last,
  input  wire [COORD_WIDTH-1:0] ll4_pair_x,
  input  wire [COORD_WIDTH-1:0] ll4_y,
  input  wire signed [1:0][COEFFICIENT_BITS-1:0] ll4_value,
  input  wire out_ready,
  output logic out_vref,
  output logic out_href,
  output logic [PIXEL_BITS-1:0] out_d0,
  output logic [PIXEL_BITS-1:0] out_d1,
  output logic out_t0,
  output logic out_t1,
  output logic out_eol,
  output logic out_row_replicated,
  output logic out_feature_valid_d0,
  output logic out_feature_valid_d1,
  output logic [SOURCE_Y_BITS-1:0] out_source_y,
  output logic busy,
  output logic protocol_error,
  output logic overrun
);
  localparam int RAW_REFERENCE_RADIUS = 6;
  localparam int RAW_THRESHOLD_BIAS = 20;
  localparam int CONNECTIVITY_THRESHOLD_LIMIT = 120;
  localparam int RAW_THRESHOLD_FLOOR = 40;
  localparam int RAW_THRESHOLD_CEILING = 128;

  logic base_out_vref, base_out_href, base_out_ready;
  logic [PIXEL_BITS-1:0] base_out_d0, base_out_d1;
  logic base_out_t0, base_out_t1, base_out_eol, base_out_row_replicated;
  logic base_out_feature_valid_d0, base_out_feature_valid_d1;
  logic [SOURCE_Y_BITS-1:0] base_out_source_y;
  logic [3:0] base_strength0, base_strength1;
  logic base_raw, base_allow_repair, base_first_row;
  logic [$clog2(WIDTH/2)-1:0] base_pair_x;
  wire repair_busy;
    ThresholdConnectivity2PPC #(.RAW_PAIRS(WIDTH/2), .SOURCE_Y_BITS(SOURCE_Y_BITS)) repair (
      .clk(clk), .rst(rst), .in_valid(base_out_vref), .in_ready(base_out_ready),
      .in_d0(base_out_d0), .in_d1(base_out_d1), .in_t0(base_out_t0), .in_t1(base_out_t1),
      .in_fv0(base_out_feature_valid_d0), .in_fv1(base_out_feature_valid_d1),
      .in_eol(base_out_eol), .in_row_replicated(base_out_row_replicated),
      .in_source_y(base_out_source_y), .in_strength0(base_strength0), .in_strength1(base_strength1),
      .in_raw(base_raw), .in_allow_repair(base_allow_repair),
      .in_first_row(base_first_row), .in_pair_x(base_pair_x),
      .out_valid(out_vref), .out_ready(out_ready), .out_d0(out_d0), .out_d1(out_d1),
      .out_t0(out_t0), .out_t1(out_t1), .out_fv0(out_feature_valid_d0), .out_fv1(out_feature_valid_d1),
      .out_eol(out_eol), .out_row_replicated(out_row_replicated), .out_source_y(out_source_y),
      .busy(repair_busy)
    );
    assign out_href = out_vref;
    wire unused_base_href = base_out_href;


  function automatic int max_int(input int a, input int b);
    max_int = (a > b) ? a : b;
  endfunction

  function automatic int min_int(input int a, input int b);
    min_int = (a < b) ? a : b;
  endfunction

  function automatic int covered_rows(
    input int stage_height,
    input int replication,
    input int offset
  );
    int available;
    int rounded;
    begin
      available = HEIGHT - offset;
      if (available < 0) available = 0;
      rounded = (available + replication - 1) / replication;
      covered_rows = min_int(rounded, stage_height);
    end
  endfunction

  function automatic int last_source_y(input int covered, input int tail);
    if (tail > 0) last_source_y = HEIGHT + tail - 1;
    else last_source_y = (covered > 0) ? HEIGHT - 1 : HEIGHT;
  endfunction

  localparam int STAGE0_WIDTH = WIDTH;
  localparam int STAGE2_WIDTH = WIDTH / 4;
  localparam int STAGE4_WIDTH = WIDTH / 16;
  localparam int STAGE0_HEIGHT = HEIGHT;
  localparam int STAGE2_HEIGHT = HEIGHT / 4;
  localparam int STAGE4_HEIGHT = HEIGHT / 16;
  localparam int S0_PAIRS = STAGE0_WIDTH / 2;
  localparam int S2_PAIRS = STAGE2_WIDTH / 2;
  localparam int S4_PAIRS = STAGE4_WIDTH / 2;
  localparam int S0_X_BITS = $clog2(S0_PAIRS);
  localparam int S2_X_BITS = $clog2(S2_PAIRS);
  localparam int S4_X_BITS = $clog2(S4_PAIRS);
  localparam int S0_Y_BITS = $clog2(STAGE0_HEIGHT);
  localparam int S2_Y_BITS = $clog2(STAGE2_HEIGHT);
  localparam int S4_Y_BITS = $clog2(STAGE4_HEIGHT);
  localparam int COARSE_DATA_BITS = 10;
  localparam int RAW_PAIR_BITS = 2 * PIXEL_BITS;
  localparam int COARSE_PAIR_BITS = 2 * COARSE_DATA_BITS;

  localparam int TAG_X_DEPTH = 4;
  localparam int TAG_Y_DEPTH = 3;
  localparam int S0_OUT_PAIRS = S0_PAIRS;
  localparam int S2_OUT_PAIRS = S2_PAIRS + TAG_X_DEPTH;
  localparam int S4_OUT_PAIRS = S4_PAIRS + TAG_X_DEPTH;
  localparam int OUTPUT_PAIRS = S0_OUT_PAIRS + S2_OUT_PAIRS + S4_OUT_PAIRS;
  localparam int OUTPUT_PAIR_BITS = $clog2(OUTPUT_PAIRS);
  localparam int MAX_STAGE_Y_BITS = $clog2(HEIGHT + 1);
  localparam int STAGE2_COVERED = covered_rows(STAGE2_HEIGHT, 4, STAGE2_Y_OFFSET);
  localparam int STAGE4_COVERED = covered_rows(STAGE4_HEIGHT, 16, STAGE4_Y_OFFSET);
  localparam int STAGE2_TAIL = STAGE2_HEIGHT - STAGE2_COVERED;
  localparam int STAGE4_TAIL = STAGE4_HEIGHT - STAGE4_COVERED;
  localparam int OUTPUT_HEIGHT = HEIGHT + max_int(STAGE2_TAIL, STAGE4_TAIL);
  localparam int OUTPUT_Y_BITS = $clog2(OUTPUT_HEIGHT);

  localparam int S0_FIRST_Y = 0;
  localparam int S2_FIRST_Y = STAGE2_Y_OFFSET;
  localparam int S4_FIRST_Y = STAGE4_Y_OFFSET;
  localparam int S0_LAST_Y = HEIGHT - 1;
  localparam int S2_LAST_Y = last_source_y(STAGE2_COVERED, STAGE2_TAIL);
  localparam int S4_LAST_Y = last_source_y(STAGE4_COVERED, STAGE4_TAIL);
  localparam int S0_TOP_PAD = min_int(TAG_Y_DEPTH, S0_FIRST_Y);
  localparam int S2_TOP_PAD = min_int(TAG_Y_DEPTH, S2_FIRST_Y);
  localparam int S4_TOP_PAD = min_int(TAG_Y_DEPTH, S4_FIRST_Y);
  localparam int S0_BOTTOM_PAD = min_int(TAG_Y_DEPTH, OUTPUT_HEIGHT - 1 - S0_LAST_Y);
  localparam int S2_BOTTOM_PAD = min_int(TAG_Y_DEPTH, OUTPUT_HEIGHT - 1 - S2_LAST_Y);
  localparam int S4_BOTTOM_PAD = min_int(TAG_Y_DEPTH, OUTPUT_HEIGHT - 1 - S4_LAST_Y);
  localparam int S0_LOCAL_HEIGHT = S0_TOP_PAD + S0_LAST_Y - S0_FIRST_Y + 1 +
      S0_BOTTOM_PAD;
  localparam int S2_LOCAL_HEIGHT = S2_TOP_PAD + S2_LAST_Y - S2_FIRST_Y + 1 +
      S2_BOTTOM_PAD;
  localparam int S4_LOCAL_HEIGHT = S4_TOP_PAD + S4_LAST_Y - S4_FIRST_Y + 1 +
      S4_BOTTOM_PAD;
  localparam int S0_FRAME_Y = S0_FIRST_Y - S0_TOP_PAD;
  localparam int S2_FRAME_Y = S2_FIRST_Y - S2_TOP_PAD;
  localparam int S4_FRAME_Y = S4_FIRST_Y - S4_TOP_PAD;

  localparam int RAW_ROW_BANKS = 9;
  localparam int LL2_ROW_BANKS = 9;
  localparam int LL4_ROW_BANKS = 9;
  localparam int RAW_BANK_BITS = $clog2(RAW_ROW_BANKS);
  localparam int LL2_BANK_BITS = $clog2(LL2_ROW_BANKS);
  localparam int LL4_BANK_BITS = $clog2(LL4_ROW_BANKS);
  localparam int RAW_DEPTH = RAW_ROW_BANKS * S0_PAIRS;
  localparam int LL2_DEPTH = LL2_ROW_BANKS * S2_PAIRS;
  localparam int LL4_DEPTH = LL4_ROW_BANKS * S4_PAIRS;
  localparam int RAW_ADDR_BITS = $clog2(RAW_DEPTH);
  localparam int LL2_ADDR_BITS = $clog2(LL2_DEPTH);
  localparam int LL4_ADDR_BITS = $clog2(LL4_DEPTH);


  function automatic logic [PIXEL_BITS-1:0] clamp_coarse(
    input logic signed [COARSE_DATA_BITS-1:0] value
  );
    begin
      // Adding the unsigned midpoint only flips the sign bit when the
      // centered sample fits in PIXEL_BITS. Otherwise saturate by its sign.
      if (value[COARSE_DATA_BITS-1:PIXEL_BITS-1] ==
          {(COARSE_DATA_BITS-PIXEL_BITS+1){value[PIXEL_BITS-1]}})
        clamp_coarse = {~value[PIXEL_BITS-1], value[PIXEL_BITS-2:0]};
      else clamp_coarse = {PIXEL_BITS{!value[COARSE_DATA_BITS-1]}};
    end
  endfunction

  function automatic logic [RAW_ADDR_BITS-1:0] raw_address(
    input logic [RAW_BANK_BITS-1:0] bank,
    input logic [S0_X_BITS-1:0] pair_x
  );
    logic [RAW_ADDR_BITS-1:0] base;
    begin
      case (bank)
        4'd0: base = RAW_ADDR_BITS'(0);
        4'd1: base = RAW_ADDR_BITS'(S0_PAIRS);
        4'd2: base = RAW_ADDR_BITS'(2 * S0_PAIRS);
        4'd3: base = RAW_ADDR_BITS'(3 * S0_PAIRS);
        4'd4: base = RAW_ADDR_BITS'(4 * S0_PAIRS);
        4'd5: base = RAW_ADDR_BITS'(5 * S0_PAIRS);
        4'd6: base = RAW_ADDR_BITS'(6 * S0_PAIRS);
        4'd7: base = RAW_ADDR_BITS'(7 * S0_PAIRS);
        default: base = RAW_ADDR_BITS'(8 * S0_PAIRS);
      endcase
      raw_address = base + RAW_ADDR_BITS'(pair_x);
    end
  endfunction

  function automatic logic [LL2_ADDR_BITS-1:0] ll2_address(
    input logic [LL2_BANK_BITS-1:0] bank,
    input logic [S2_X_BITS-1:0] pair_x
  );
    logic [LL2_ADDR_BITS-1:0] base;
    begin
      case (bank)
        4'd0: base = LL2_ADDR_BITS'(0);
        4'd1: base = LL2_ADDR_BITS'(S2_PAIRS);
        4'd2: base = LL2_ADDR_BITS'(2 * S2_PAIRS);
        4'd3: base = LL2_ADDR_BITS'(3 * S2_PAIRS);
        4'd4: base = LL2_ADDR_BITS'(4 * S2_PAIRS);
        4'd5: base = LL2_ADDR_BITS'(5 * S2_PAIRS);
        4'd6: base = LL2_ADDR_BITS'(6 * S2_PAIRS);
        4'd7: base = LL2_ADDR_BITS'(7 * S2_PAIRS);
        default: base = LL2_ADDR_BITS'(8 * S2_PAIRS);
      endcase
      ll2_address = base + LL2_ADDR_BITS'(pair_x);
    end
  endfunction

  function automatic logic [LL4_ADDR_BITS-1:0] ll4_address(
    input logic [LL4_BANK_BITS-1:0] bank,
    input logic [S4_X_BITS-1:0] pair_x
  );
    logic [LL4_ADDR_BITS-1:0] base;
    begin
      case (bank)
        4'd0: base = LL4_ADDR_BITS'(0);
        4'd1: base = LL4_ADDR_BITS'(S4_PAIRS);
        4'd2: base = LL4_ADDR_BITS'(2 * S4_PAIRS);
        4'd3: base = LL4_ADDR_BITS'(3 * S4_PAIRS);
        4'd4: base = LL4_ADDR_BITS'(4 * S4_PAIRS);
        4'd5: base = LL4_ADDR_BITS'(5 * S4_PAIRS);
        4'd6: base = LL4_ADDR_BITS'(6 * S4_PAIRS);
        4'd7: base = LL4_ADDR_BITS'(7 * S4_PAIRS);
        default: base = LL4_ADDR_BITS'(8 * S4_PAIRS);
      endcase
      ll4_address = base + LL4_ADDR_BITS'(pair_x);
    end
  endfunction

  function automatic logic [3:0] next_bank9(input logic [3:0] bank);
    next_bank9 = bank == 4'd8 ? 4'd0 : bank + 1'b1;
  endfunction

  logic [S0_X_BITS-1:0] raw_x;
  logic [S0_Y_BITS-1:0] raw_y;
  wire raw_valid = in_vref && in_href;
  wire raw_first = raw_x == '0;
  wire raw_last = raw_x == S0_X_BITS'(S0_PAIRS - 1);

  logic [S2_X_BITS-1:0] ll2_expected_x;
  logic [S2_Y_BITS-1:0] ll2_expected_y;
  logic signed [COARSE_DATA_BITS-1:0] ll2_d0, ll2_d1;
  logic signed [COARSE_DATA_BITS-1:0] ll4_d0, ll4_d1;
  assign ll2_d0 = $signed(ll2_value[0][COARSE_DATA_BITS-1:0]);
  assign ll2_d1 = $signed(ll2_value[1][COARSE_DATA_BITS-1:0]);
  assign ll4_d0 = $signed(ll4_value[0][COARSE_DATA_BITS-1:0]);
  assign ll4_d1 = $signed(ll4_value[1][COARSE_DATA_BITS-1:0]);

  logic [RAW_ROW_BANKS-1:0] raw_bank_valid;
  logic [LL2_ROW_BANKS-1:0] ll2_bank_valid;
  logic [LL4_ROW_BANKS-1:0] ll4_bank_valid;
  logic [LL4_ROW_BANKS-1:0] mean_bank_valid;
  logic mean_valid, mean_first, mean_last, mean_busy, mean_protocol_error;
  logic [COORD_WIDTH-1:0] mean_pair_x, mean_y;
  wire [2*COORD_WIDTH-S4_X_BITS-S4_Y_BITS-1:0] unused_mean_coordinates =
      {mean_pair_x[COORD_WIDTH-1:S4_X_BITS], mean_y[COORD_WIDTH-1:S4_Y_BITS]};
  logic signed [1:0][COARSE_DATA_BITS-1:0] mean_value;
  logic [LL4_BANK_BITS-1:0] mean_write_bank;
  Ll4CausalMean5x3 #(
    .WIDTH(STAGE4_WIDTH), .HEIGHT(STAGE4_HEIGHT),
    .DATA_BITS(COARSE_DATA_BITS), .COORD_WIDTH(COORD_WIDTH)
  ) ll4_mean (
    .clk(clk), .rst(rst), .in_valid(ll4_valid), .in_first(ll4_first),
    .in_last(ll4_last), .in_pair_x(ll4_pair_x), .in_y(ll4_y),
    .in_value({ll4_d1, ll4_d0}), .out_valid(mean_valid),
    .out_first(mean_first), .out_last(mean_last), .out_pair_x(mean_pair_x),
    .out_y(mean_y), .out_value(mean_value), .busy(mean_busy),
    .protocol_error(mean_protocol_error)
  );
`ifndef SYNTHESIS
  // Sequential producer/consumer bank pointers are the hardware identity.
  // Keep explicit row tags only as simulation proofs of that invariant.
  logic [RAW_ROW_BANKS-1:0][S0_Y_BITS-1:0] raw_bank_y;
  logic [LL2_ROW_BANKS-1:0][S2_Y_BITS-1:0] ll2_bank_y;
  logic [LL4_ROW_BANKS-1:0][S4_Y_BITS-1:0] ll4_bank_y;
  logic [LL4_ROW_BANKS-1:0][S4_Y_BITS-1:0] mean_bank_y;
`endif

  logic [OUTPUT_Y_BITS-1:0] output_y;
  logic [1:0] output_region;
  logic [OUTPUT_PAIR_BITS-1:0] output_region_pair;
  logic stage_active0, stage_active2, stage_active4;
  logic stage_replicated2, stage_replicated4;
  wire [MAX_STAGE_Y_BITS-1:0] stage_y0 = MAX_STAGE_Y_BITS'(output_y);
  logic [S2_Y_BITS-1:0] consumed_rows2;
  logic [S4_Y_BITS-1:0] consumed_rows4;
  wire [MAX_STAGE_Y_BITS-1:0] stage_y2 = MAX_STAGE_Y_BITS'(consumed_rows2);
  wire [MAX_STAGE_Y_BITS-1:0] stage_y4 = MAX_STAGE_Y_BITS'(consumed_rows4);
  logic [MAX_STAGE_Y_BITS-1:0] current_stage_y;
  logic current_active, current_replicated;
  logic [OUTPUT_PAIR_BITS-1:0] current_last_pair;
  logic [OUTPUT_PAIR_BITS-1:0] source_first_pair, source_limit_pair;
  wire [S2_X_BITS-1:0] source_pair =
      S2_X_BITS'(output_region_pair) - S2_X_BITS'(TAG_X_DEPTH/2);
  logic source_active, last_pair, last_region, slot_ready;
  logic output_issue, wait_new_frame;
  logic output_slot_ready, pending_slot_ready, pending_advance;
  logic release_stage0, release_stage2, release_stage4;
  wire [1:0] row_phase2 = output_y[1:0] - 2'(STAGE2_Y_OFFSET);
  wire [3:0] row_phase4 = output_y[3:0] - 4'(STAGE4_Y_OFFSET);
  logic [RAW_BANK_BITS-1:0] raw_read_bank, raw_write_bank;
  logic [LL2_BANK_BITS-1:0] ll2_write_bank;
  logic [LL4_BANK_BITS-1:0] ll4_write_bank;
  logic [RAW_BANK_BITS-1:0] raw_stage_bank;
  logic [LL2_BANK_BITS-1:0] ll2_raw_reference_bank, ll2_stage_bank;
  logic [LL4_BANK_BITS-1:0] ll4_ll2_reference_bank, ll4_stage_bank;
  logic [LL4_BANK_BITS-1:0] mean_upper_bank;
  logic mean_rows_ready, release_mean_row;
  logic raw_reference_started;
  wire raw_reference_ready, raw_reference_read_en;
  wire [3:0] raw_reference_read_bank;
  wire [7:0] raw_reference_read_pair_x;
  localparam int RAW_WINDOW_SAMPLES = 3*(2*RAW_REFERENCE_RADIUS+1);
  wire signed [15:0] raw_reference_sum;
  wire signed [9:0] raw_reference_direct;
  wire signed [11:0] raw_reference_local_three;
  logic [7:0] pending_raw_threshold;
  wire raw_reference_start = !raw_reference_started &&
      !wait_new_frame && output_region == 0 && current_active &&
      raw_bank_valid[raw_stage_bank] && ll2_bank_valid[ll2_raw_reference_bank];

  always_comb begin
    // Select the nearest LL4 sample center, at LL2 row 4*y+1.
    // Lower row ownership advances at the same completed-row boundary.
    mean_upper_bank = ll4_ll2_reference_bank;
    if (stage_y2 != 0 && (stage_y2[1:0] == 2'b00 || stage_y2[1:0] == 2'b11) &&
        stage_y2 < MAX_STAGE_Y_BITS'(4*(STAGE4_HEIGHT-1)+1))
      mean_upper_bank = next_bank9(ll4_ll2_reference_bank);
    mean_rows_ready = mean_bank_valid[mean_upper_bank];
    release_mean_row = release_stage2 &&
        ((stage_y2 != 0 && stage_y2[1:0] == 2'b00) ||
         stage_y2 == MAX_STAGE_Y_BITS'(STAGE2_HEIGHT-1));
  end

  // A source row advances exactly when its final display copy is released.
  // Count that event alongside the bank pointer, avoiding repeated full-width
  // offset/tail arithmetic on every output pair. Only the replication phase
  // depends on the low bits of the composite row coordinate.
  always_comb begin
    stage_active0 = output_y < OUTPUT_Y_BITS'(HEIGHT);
    stage_active2 = output_y >= OUTPUT_Y_BITS'(S2_FIRST_Y) &&
        output_y <= OUTPUT_Y_BITS'(S2_LAST_Y);
    stage_active4 = output_y >= OUTPUT_Y_BITS'(S4_FIRST_Y) &&
        output_y <= OUTPUT_Y_BITS'(S4_LAST_Y);
    stage_replicated2 = stage_active2 && stage_active0 && row_phase2 != 0;
    stage_replicated4 = stage_active4 && stage_active0 && row_phase4 != 0;
  end

  logic [10:0] row_cycle_count;
  logic output_row_seen;
  logic [10:0] unused_in_synthesis_ready_stalls;
  logic [10:0] unused_in_synthesis_slot_stalls;

  always_comb begin
    current_active = stage_active0;
    current_replicated = 1'b0;
    current_stage_y = stage_y0;
    current_last_pair = OUTPUT_PAIR_BITS'(S0_OUT_PAIRS - 1);
    source_first_pair = '0;
    source_limit_pair = OUTPUT_PAIR_BITS'(S0_PAIRS);
    raw_read_bank = raw_stage_bank;
    slot_ready = raw_bank_valid[raw_read_bank] &&
        ll2_bank_valid[ll2_raw_reference_bank] && raw_reference_ready;
    if (output_region == 1) begin
      current_active = stage_active2;
      current_replicated = stage_replicated2;
      current_stage_y = stage_y2;
      current_last_pair = OUTPUT_PAIR_BITS'(S2_OUT_PAIRS - 1);
      source_first_pair = OUTPUT_PAIR_BITS'(TAG_X_DEPTH / 2);
      source_limit_pair = OUTPUT_PAIR_BITS'((TAG_X_DEPTH / 2) + S2_PAIRS);
      slot_ready = ll2_bank_valid[ll2_stage_bank] &&
          mean_rows_ready;
    end else if (output_region == 2) begin
      current_active = stage_active4;
      current_replicated = stage_replicated4;
      current_stage_y = stage_y4;
      current_last_pair = OUTPUT_PAIR_BITS'(S4_OUT_PAIRS - 1);
      source_first_pair = OUTPUT_PAIR_BITS'(TAG_X_DEPTH / 2);
      source_limit_pair = OUTPUT_PAIR_BITS'((TAG_X_DEPTH / 2) + S4_PAIRS);
      slot_ready = ll4_bank_valid[ll4_stage_bank];
    end
    last_pair = output_region_pair == current_last_pair;
    last_region = output_region == 2;
    source_active = current_active &&
        output_region_pair >= source_first_pair &&
        output_region_pair < source_limit_pair;
    output_issue = pending_slot_ready && !wait_new_frame &&
        (output_region != 0 || output_region_pair != '0 || !output_row_seen ||
         row_cycle_count >= 11'(ROW_CYCLES - 1)) &&
        (!source_active || slot_ready);
    release_stage0 = output_issue && last_pair && output_region == 0 && current_active;
    release_stage2 = output_issue && last_pair && output_region == 1 &&
        current_active && (int'(output_y) >= HEIGHT || row_phase2 == 2'b11 ||
        output_y == OUTPUT_Y_BITS'(HEIGHT - 1));
    release_stage4 = output_issue && last_pair && output_region == 2 &&
        current_active && (int'(output_y) >= HEIGHT || row_phase4 == 4'hf ||
        output_y == OUTPUT_Y_BITS'(HEIGHT - 1));
  end

  logic raw_read_en, ll2_read_en, ll4_read_en, mean_read_en;
  logic [RAW_ADDR_BITS-1:0] raw_write_addr, raw_read_addr;
  logic [LL2_ADDR_BITS-1:0] ll2_write_addr, ll2_read_addr;
  logic [LL4_ADDR_BITS-1:0] ll4_write_addr, ll4_read_addr;
  logic [LL4_ADDR_BITS-1:0] mean_write_addr, mean_read_addr;
  logic [LL4_BANK_BITS-1:0] mean_read_bank;
  logic [S4_X_BITS-1:0] mean_read_pair_x;
  logic [RAW_PAIR_BITS-1:0] raw_q;
  logic [COARSE_PAIR_BITS-1:0] ll2_q, ll4_q, mean_q;
  (* syn_ramstyle = "block_ram" *)
  logic [RAW_PAIR_BITS-1:0] raw_mem [0:RAW_DEPTH-1];
  (* syn_ramstyle = "block_ram" *)
  logic [COARSE_PAIR_BITS-1:0] ll2_mem [0:LL2_DEPTH-1];
  (* syn_ramstyle = "block_ram" *)
  logic [COARSE_PAIR_BITS-1:0] ll4_mem [0:LL4_DEPTH-1];
  (* syn_ramstyle = "block_ram" *)
  logic [COARSE_PAIR_BITS-1:0] mean_mem [0:LL4_DEPTH-1];

    Ll2CausalBoxReference raw_reference (
      .clk(clk), .rst(rst), .start(raw_reference_start),
      .current_bank(ll2_raw_reference_bank), .current_y(S2_Y_BITS'(stage_y0 >> 2)),
      .take(output_issue && source_active && output_region == 0),
      .pair_x(output_region_pair[S0_X_BITS-1:0]),
      .ready(raw_reference_ready), .window_sum(raw_reference_sum),
      .direct_value(raw_reference_direct),
      .local_three_sum(raw_reference_local_three),
      .read_en(raw_reference_read_en), .read_bank(raw_reference_read_bank),
      .read_pair_x(raw_reference_read_pair_x), .read_data(ll2_q)
    );

  always_ff @(posedge clk) begin
    if (rst) raw_reference_started <= 1'b0;
    else begin
      if (raw_reference_start) raw_reference_started <= 1'b1;
      if (output_issue && last_pair && output_region == 0)
        raw_reference_started <= 1'b0;
    end
  end

  // Each LL2 pair shares one nearest coarse reference. Read it in parallel
  // with the LL2 pixels; only the packed-word lane selector needs a register.
  wire [S2_X_BITS-1:0] mean_rounded_x =
      {1'b0, source_pair[S2_X_BITS-1:1]} + S2_X_BITS'(source_pair[0]);
  wire [S4_X_BITS:0] mean_source_x = mean_rounded_x >= (S2_X_BITS)'(STAGE4_WIDTH) ?
      (S4_X_BITS+1)'(STAGE4_WIDTH-1) : (S4_X_BITS+1)'(mean_rounded_x);
  logic pending_mean_lane;
  wire signed [COARSE_DATA_BITS-1:0] mean_reference = pending_mean_lane ?
      $signed(mean_q[COARSE_PAIR_BITS-1:COARSE_DATA_BITS]) :
      $signed(mean_q[COARSE_DATA_BITS-1:0]);
  assign mean_read_en = output_issue && source_active && output_region == 1;
  assign mean_read_bank = mean_upper_bank;
  assign mean_read_pair_x = mean_source_x[S4_X_BITS:1];

  always_comb begin
    raw_write_addr = raw_address(raw_write_bank, raw_x);
    ll2_write_addr = ll2_address(ll2_write_bank,
        ll2_pair_x[S2_X_BITS-1:0]);
    ll4_write_addr = ll4_address(ll4_write_bank,
        ll4_pair_x[S4_X_BITS-1:0]);
    mean_write_addr = ll4_address(mean_write_bank,
        mean_pair_x[S4_X_BITS-1:0]);
    mean_read_addr = ll4_address(mean_read_bank, mean_read_pair_x);

    raw_read_en = output_issue && source_active && output_region == 0;
    ll2_read_en = output_region == 0 ? raw_reference_read_en :
        output_issue && source_active && output_region == 1;
    ll4_read_en = output_issue && source_active && output_region == 2;
    raw_read_addr = raw_address(raw_read_bank, S0_X_BITS'(output_region_pair));
    // Select the LL2 request before calculating its address, sharing the row
    // stride addition. LL4 has only its display reader; reference reads use
    // the filtered-mean RAM instead.
    ll2_read_addr = ll2_address(
        output_region == 0 ? raw_reference_read_bank : ll2_stage_bank,
        output_region == 0 ? raw_reference_read_pair_x : source_pair[S2_X_BITS-1:0]);
    ll4_read_addr = ll4_address(ll4_stage_bank, source_pair[S4_X_BITS-1:0]);
  end

  always_ff @(posedge clk) begin
    if (raw_valid) raw_mem[raw_write_addr] <= {in_d1, in_d0};
    if (raw_read_en) raw_q <= raw_mem[raw_read_addr];
    if (ll2_valid) ll2_mem[ll2_write_addr] <= {ll2_d1, ll2_d0};
    if (ll2_read_en) ll2_q <= ll2_mem[ll2_read_addr];
    if (ll4_valid) ll4_mem[ll4_write_addr] <= {ll4_d1, ll4_d0};
    if (ll4_read_en) ll4_q <= ll4_mem[ll4_read_addr];
    if (mean_valid) mean_mem[mean_write_addr] <= mean_value;
    if (mean_read_en) mean_q <= mean_mem[mean_read_addr];
  end

  logic pending_valid, pending_source_active, pending_replicated;
  logic [1:0] pending_region;
  logic [OUTPUT_PAIR_BITS-1:0] pending_region_pair;
  logic pending_first_row, pending_source_pixels, pending_white0, pending_white1;
  logic [MAX_STAGE_Y_BITS-1:0] pending_stage_y;
  logic pending_eol, pending_last_frame;
  logic [PIXEL_BITS-1:0] source_d0, source_d1;
  logic source_t0, source_t1;

  assign output_slot_ready = !base_out_vref || base_out_ready;
  assign pending_slot_ready = !pending_valid || output_slot_ready;
  assign pending_advance = pending_valid && output_slot_ready;

  // The local term and rounding bias are multiples of eight. Divide that
  // common factor out before adding the two terms; discarded gain bits cannot
  // affect the final division by 64. Fold the fixed bias into the local term.
  wire signed [21:0] gain_sum = raw_reference_sum * 22'sd35;
  wire signed [18:0] scaled_gain = gain_sum[21:3];
  wire signed [18:0] biased_local = raw_reference_direct * 19'(RAW_WINDOW_SAMPLES) +
      19'(16*RAW_WINDOW_SAMPLES*43 + 4*RAW_WINDOW_SAMPLES +
          8*RAW_WINDOW_SAMPLES*RAW_THRESHOLD_BIAS);
  wire signed [18:0] blended_numerator = scaled_gain + biased_local;
  wire signed [15:0] blended_reference = blended_numerator[18:3];
  wire [12:0] threshold_dividend =
      blended_reference < 16'(RAW_THRESHOLD_FLOOR*RAW_WINDOW_SAMPLES) ?
          13'(RAW_THRESHOLD_FLOOR*RAW_WINDOW_SAMPLES) :
      blended_reference > 16'(RAW_THRESHOLD_CEILING*RAW_WINDOW_SAMPLES) ?
          13'(RAW_THRESHOLD_CEILING*RAW_WINDOW_SAMPLES) : blended_reference[12:0];
  // floor(n/39) = (n*6722)>>18 over this clipped domain.
  localparam logic [12:0] RAW_RECIPROCAL = 13'd6722;
  wire [25:0] threshold_product = threshold_dividend * RAW_RECIPROCAL;
  // floor((sum(centered three taps)+772)/6) is exactly
  // round(round(mean(unsigned three taps))/2)+64 for integer samples.
  // After clipping, division by six is floor(n/2)*171 >> 9. It is exact
  // for 0 <= floor(n/2) <= 511.
  wire signed [12:0] local_cap_numerator = 13'(raw_reference_local_three) + 13'sd772;
  wire [9:0] local_cap_dividend = local_cap_numerator < 13'(6*RAW_THRESHOLD_FLOOR) ?
      10'(6*RAW_THRESHOLD_FLOOR) : local_cap_numerator > 13'(6*RAW_THRESHOLD_CEILING) ?
      10'(6*RAW_THRESHOLD_CEILING) : local_cap_numerator[9:0];
  wire [8:0] local_cap_half = local_cap_dividend[9:1];
  wire [16:0] local_cap_product = local_cap_half * 8'd171;
  wire [7:0] raw_local_cap_dn = local_cap_product[16:9];
  wire [12:0] unused_local_cap_fraction = {local_cap_numerator[12:10], local_cap_dividend[0],
      local_cap_product[8:0]};
  wire [7:0] next_raw_threshold = raw_local_cap_dn < threshold_product[25:18] ?
      raw_local_cap_dn : threshold_product[25:18];
  wire [7:0] raw_threshold_dn = pending_raw_threshold;
  wire [23:0] unused_threshold_fraction =
      {gain_sum[2:0], blended_numerator[2:0], threshold_product[17:0]};
  function automatic logic [3:0] residual_strength(input logic [7:0] pixel, threshold);
    logic signed [8:0] difference;
    logic [4:0] units, rounded;
    logic round_up;
    logic [2:0] magnitude;
    begin
      difference = $signed({1'b0,pixel}) - $signed({1'b0,threshold});
      // Round abs(difference)/16 using one subtraction and a five-bit
      // correction. For a negative difference, complement its floor quotient
      // and round upward exactly when the low nibble is at most eight.
      units = difference[8] ? ~difference[8:4] : difference[8:4];
      round_up = difference[8] ? difference[3:0] <= 4'd8 : difference[3];
      rounded = units + 5'(round_up);
      magnitude = rounded >= 5'd7 ? 3'd7 : rounded[2:0];
      residual_strength = {!difference[8], magnitude};
    end
  endfunction
  wire [3:0] source_strength0 = residual_strength(raw_q[PIXEL_BITS-1:0], raw_threshold_dn);
  wire [3:0] source_strength1 = residual_strength(raw_q[RAW_PAIR_BITS-1:PIXEL_BITS], raw_threshold_dn);
  always_comb begin
    logic signed [COARSE_DATA_BITS-1:0] ll2_sample0, ll2_sample1;
    logic signed [COARSE_DATA_BITS-1:0] ll4_sample0, ll4_sample1;
    ll2_sample0 = $signed(ll2_q[COARSE_DATA_BITS-1:0]);
    ll2_sample1 = $signed(ll2_q[COARSE_PAIR_BITS-1:COARSE_DATA_BITS]);
    ll4_sample0 = $signed(ll4_q[COARSE_DATA_BITS-1:0]);
    ll4_sample1 = $signed(ll4_q[COARSE_PAIR_BITS-1:COARSE_DATA_BITS]);
    source_d0 = '0;
    source_d1 = '0;
    source_t0 = 1'b0;
    source_t1 = 1'b0;
    case (pending_region)
      2'd0: begin
        source_d0 = raw_q[PIXEL_BITS-1:0];
        source_d1 = raw_q[RAW_PAIR_BITS-1:PIXEL_BITS];
        source_t0 = source_strength0[3];
        source_t1 = source_strength1[3];
      end
      2'd1: begin
        source_d0 = clamp_coarse(ll2_sample0);
        source_d1 = clamp_coarse(ll2_sample1);
        source_t0 = ll2_sample0 >= mean_reference;
        source_t1 = ll2_sample1 >= mean_reference;
      end
      default: begin
        source_d0 = clamp_coarse(ll4_sample0);
        source_d1 = clamp_coarse(ll4_sample1);
        source_t0 = ll4_sample0 >= 0;
        source_t1 = ll4_sample1 >= 0;
      end
    endcase
  end

  logic frame_inner_row, frame_fence_row;
  wire first_pair = output_region_pair == 0;
  wire source_pixels = source_active && (output_region == 0 ?
      (int'(output_y) >= TAG_Y_DEPTH && output_region_pair >= 2 &&
       int'(output_region_pair) < S0_OUT_PAIRS-2) :
      output_region == 1 ? int'(output_y) < S2_FRAME_Y + S2_LOCAL_HEIGHT - TAG_Y_DEPTH :
                          int'(output_y) < S4_FRAME_Y + S4_LOCAL_HEIGHT - TAG_Y_DEPTH);
  wire frame_white0 = frame_inner_row && !first_pair && (frame_fence_row || last_pair);
  wire frame_white1 = frame_inner_row && !last_pair && (frame_fence_row || first_pair);

  // Coarse source_active already excludes framing columns and rows. RAW has
  // no extra horizontal padding, so its first/last two pairs and top three
  // rows are excluded here. Coarse tail rows also stop before the bottom fence.
  // Capture only source/white flags across the RAM
  // read; there is no need to register Y and reconstruct the framing later.
  always_comb begin
    case (output_region)
      2'd1: begin
        frame_inner_row = int'(output_y) > S2_FRAME_Y &&
            int'(output_y) < S2_FRAME_Y + S2_LOCAL_HEIGHT - 1;
        frame_fence_row = int'(output_y) == S2_FRAME_Y + 1 ||
            int'(output_y) == S2_FRAME_Y + S2_LOCAL_HEIGHT - 2;
      end
      2'd2: begin
        frame_inner_row = int'(output_y) > S4_FRAME_Y &&
            int'(output_y) < S4_FRAME_Y + S4_LOCAL_HEIGHT - 1;
        frame_fence_row = int'(output_y) == S4_FRAME_Y + 1 ||
            int'(output_y) == S4_FRAME_Y + S4_LOCAL_HEIGHT - 2;
      end
      default: begin
        frame_inner_row = int'(output_y) > S0_FRAME_Y &&
            int'(output_y) < S0_FRAME_Y + S0_LOCAL_HEIGHT - 1;
        frame_fence_row = int'(output_y) == S0_FRAME_Y + 1 ||
            int'(output_y) == S0_FRAME_Y + S0_LOCAL_HEIGHT - 2;
      end
    endcase
  end
  wire [PIXEL_BITS-1:0] emit_d0 = pending_source_pixels ? source_d0 : '0;
  wire [PIXEL_BITS-1:0] emit_d1 = pending_source_pixels ? source_d1 : '0;
  wire emit_t0 = pending_white0 || (pending_source_pixels && source_t0);
  wire emit_t1 = pending_white1 || (pending_source_pixels && source_t1);
  wire emit_fv0 = pending_source_pixels && !pending_replicated;
  wire emit_fv1 = emit_fv0;
  // All stream samples use the same weight representation; feature_valid
  // excludes framing, replicated rows and repaired pixels from moments.
  ThresholdMomentWeightRom weights (
    .clk(clk), .ce(output_slot_ready),
    .address0({pending_region != 0, emit_d0}),
    .address1({pending_region != 0, emit_d1}),
    .q0(base_out_d0), .q1(base_out_d1)
  );

  always_ff @(posedge clk) begin
    if (rst) begin
      raw_x <= '0;
      raw_y <= '0;
      ll2_expected_x <= '0;
      ll2_expected_y <= '0;
      raw_bank_valid <= '0;
      ll2_bank_valid <= '0;
      ll4_bank_valid <= '0;
      mean_bank_valid <= '0;
`ifndef SYNTHESIS
      raw_bank_y <= '0;
      ll2_bank_y <= '0;
      ll4_bank_y <= '0;
      mean_bank_y <= '0;
`endif
      raw_write_bank <= '0;
      ll2_write_bank <= '0;
      ll4_write_bank <= '0;
      mean_write_bank <= '0;
      raw_stage_bank <= '0;
      ll2_raw_reference_bank <= '0;
      ll2_stage_bank <= '0;
      ll4_ll2_reference_bank <= '0;
      ll4_stage_bank <= '0;
      output_y <= '0;
      consumed_rows2 <= '0;
      consumed_rows4 <= '0;
      output_region <= '0;
      output_region_pair <= '0;
      wait_new_frame <= 1'b0;
      pending_valid <= 1'b0;
      pending_source_active <= 1'b0;
      pending_replicated <= 1'b0;
      pending_region <= '0;
      pending_region_pair <= '0;
      pending_raw_threshold <= '0;
      pending_first_row <= 1'b0;
      pending_source_pixels <= 1'b0;
      pending_white0 <= 1'b0;
      pending_white1 <= 1'b0;
      pending_stage_y <= '0;
      pending_eol <= 1'b0;
      pending_last_frame <= 1'b0;
      pending_mean_lane <= 1'b0;
      base_out_vref <= 1'b0;
      base_out_href <= 1'b0;
      base_out_t0 <= 1'b0;
      base_out_t1 <= 1'b0;
      base_out_eol <= 1'b0;
      base_out_row_replicated <= 1'b0;
      base_out_feature_valid_d0 <= 1'b0;
      base_out_feature_valid_d1 <= 1'b0;
      base_out_source_y <= '0;
      base_strength0 <= 4'h8;
      base_strength1 <= 4'h8;
      base_raw <= 0;
      base_allow_repair <= 0;
      base_first_row <= 0;
      base_pair_x <= '0;
      protocol_error <= 1'b0;
      overrun <= 1'b0;
      row_cycle_count <= '0;
      output_row_seen <= 1'b0;
      unused_in_synthesis_ready_stalls <= '0;
      unused_in_synthesis_slot_stalls <= '0;
    end else begin
      if (mean_protocol_error) protocol_error <= 1'b1;
      if (output_slot_ready) begin
        base_raw <= pending_valid && pending_region == 0 && pending_source_active;
        base_allow_repair <= int'(raw_threshold_dn) <= CONNECTIVITY_THRESHOLD_LIMIT;
        base_first_row <= pending_first_row;
        base_pair_x <= pending_region_pair[S0_X_BITS-1:0];
        base_strength0 <= pending_region == 0 && pending_source_active ? source_strength0 : 4'h8;
        base_strength1 <= pending_region == 0 && pending_source_active ? source_strength1 : 4'h8;
        base_out_vref <= pending_valid;
        base_out_href <= pending_valid;
        base_out_t0 <= pending_valid ? emit_t0 : 1'b0;
        base_out_t1 <= pending_valid ? emit_t1 : 1'b0;
        base_out_eol <= pending_valid && pending_eol;
        base_out_row_replicated <= pending_valid && pending_replicated;
        base_out_feature_valid_d0 <= pending_valid && emit_fv0;
        base_out_feature_valid_d1 <= pending_valid && emit_fv1;
        base_out_source_y <= pending_valid && (emit_fv0 || emit_fv1) ?
            SOURCE_Y_BITS'(pending_stage_y) : '0;
      end

      if (release_stage0) begin
        raw_bank_valid[raw_read_bank] <= 1'b0;
        raw_stage_bank <= next_bank9(raw_stage_bank);
        if (stage_y0[1:0] == 2'b11)
          ll2_raw_reference_bank <= next_bank9(ll2_raw_reference_bank);
      end
      if (release_stage2) begin
        consumed_rows2 <= consumed_rows2 + 1'b1;
        ll2_bank_valid[ll2_stage_bank] <= 1'b0;
        ll2_stage_bank <= next_bank9(ll2_stage_bank);
        if (release_mean_row) begin
          mean_bank_valid[ll4_ll2_reference_bank] <= 1'b0;
          ll4_ll2_reference_bank <= next_bank9(ll4_ll2_reference_bank);
        end
      end
      if (release_stage4) begin
        consumed_rows4 <= consumed_rows4 + 1'b1;
        ll4_bank_valid[ll4_stage_bank] <= 1'b0;
        ll4_stage_bank <= next_bank9(ll4_stage_bank);
      end

      if (raw_valid) begin
        if (raw_first && raw_bank_valid[raw_write_bank] &&
            !(release_stage0 && raw_read_bank == raw_write_bank)) begin
          overrun <= 1'b1;
        end
        if (raw_last) begin
          raw_bank_valid[raw_write_bank] <= 1'b1;
`ifndef SYNTHESIS
          raw_bank_y[raw_write_bank] <= raw_y;
`endif
          raw_write_bank <= raw_y == S0_Y_BITS'(STAGE0_HEIGHT - 1) ?
              '0 : next_bank9(raw_write_bank);
          raw_x <= '0;
          raw_y <= raw_y == S0_Y_BITS'(STAGE0_HEIGHT - 1) ? '0 : raw_y + 1'b1;
        end else raw_x <= raw_x + 1'b1;
      end

      if (ll2_valid) begin
        if (ll2_pair_x[S2_X_BITS-1:0] != ll2_expected_x ||
            ll2_y[S2_Y_BITS-1:0] != ll2_expected_y ||
            ((ll2_pair_x[S2_X_BITS-1:0] == '0) != ll2_first) ||
            ((ll2_pair_x[S2_X_BITS-1:0] == S2_X_BITS'(S2_PAIRS - 1)) != ll2_last) ||
            |(ll2_pair_x >> S2_X_BITS) || |(ll2_y >> S2_Y_BITS))
          protocol_error <= 1'b1;
        if (ll2_first && ll2_bank_valid[ll2_write_bank] &&
            !(release_stage2 && ll2_stage_bank == ll2_write_bank)) begin
          overrun <= 1'b1;
        end
        if (ll2_last) begin
          ll2_bank_valid[ll2_write_bank] <= 1'b1;
`ifndef SYNTHESIS
          ll2_bank_y[ll2_write_bank] <= ll2_y[S2_Y_BITS-1:0];
`endif
          ll2_write_bank <= ll2_y[S2_Y_BITS-1:0] ==
              S2_Y_BITS'(STAGE2_HEIGHT - 1) ? '0 : next_bank9(ll2_write_bank);
          ll2_expected_x <= '0;
          ll2_expected_y <= ll2_expected_y == S2_Y_BITS'(STAGE2_HEIGHT - 1) ?
              '0 : ll2_expected_y + 1'b1;
        end else ll2_expected_x <= ll2_expected_x + 1'b1;
      end

      if (ll4_valid) begin
        // ll4_mean checks the complete LL4 coordinate/first/last sequence;
        // mean_protocol_error feeds the same sticky diagnostic above.
        if (ll4_first && ll4_bank_valid[ll4_write_bank] &&
            !(release_stage4 && ll4_stage_bank == ll4_write_bank)) begin
          overrun <= 1'b1;
        end
        if (ll4_last) begin
          ll4_bank_valid[ll4_write_bank] <= 1'b1;
`ifndef SYNTHESIS
          ll4_bank_y[ll4_write_bank] <= ll4_y[S4_Y_BITS-1:0];
`endif
          ll4_write_bank <= ll4_y[S4_Y_BITS-1:0] ==
              S4_Y_BITS'(STAGE4_HEIGHT - 1) ? '0 : next_bank9(ll4_write_bank);
        end
      end

      if (mean_valid) begin
        if (mean_first && mean_bank_valid[mean_write_bank] &&
            !(release_mean_row &&
              ll4_ll2_reference_bank == mean_write_bank))
          overrun <= 1'b1;
        if (mean_last) begin
          mean_bank_valid[mean_write_bank] <= 1'b1;
`ifndef SYNTHESIS
          mean_bank_y[mean_write_bank] <= mean_y[S4_Y_BITS-1:0];
`endif
          mean_write_bank <= mean_y[S4_Y_BITS-1:0] ==
              S4_Y_BITS'(STAGE4_HEIGHT-1) ? '0 : next_bank9(mean_write_bank);
        end
      end

      if (pending_slot_ready) begin
        pending_valid <= output_issue;
        if (output_issue) begin
          pending_source_active <= source_active;
          pending_replicated <= current_replicated;
          pending_region <= output_region;
          pending_region_pair <= output_region_pair;
          pending_raw_threshold <= next_raw_threshold;
          pending_first_row <= output_y == 0;
          pending_source_pixels <= source_pixels;
          pending_white0 <= frame_white0;
          pending_white1 <= frame_white1;
          pending_stage_y <= current_stage_y;
          pending_eol <= last_pair && last_region;
          pending_last_frame <= last_pair && last_region &&
              output_y == OUTPUT_Y_BITS'(OUTPUT_HEIGHT - 1);
          pending_mean_lane <= mean_source_x[0];
        end
      end
      if (output_issue) begin
        if (last_pair) begin
          output_region_pair <= '0;
          if (last_region) begin
            output_region <= '0;
            if (output_y == OUTPUT_Y_BITS'(OUTPUT_HEIGHT - 1))
              wait_new_frame <= 1'b1;
            else output_y <= output_y + 1'b1;
          end else output_region <= output_region + 1'b1;
        end else output_region_pair <= output_region_pair + 1'b1;
      end

      if (pending_advance && pending_last_frame) begin
        output_y <= '0;
        consumed_rows2 <= '0;
        consumed_rows4 <= '0;
        output_region <= '0;
        output_region_pair <= '0;
        wait_new_frame <= 1'b0;
        raw_stage_bank <= '0;
        ll2_raw_reference_bank <= '0;
        ll2_stage_bank <= '0;
        ll4_ll2_reference_bank <= '0;
        ll4_stage_bank <= '0;
        row_cycle_count <= '0;
        output_row_seen <= 1'b0;
        unused_in_synthesis_ready_stalls <= '0;
        unused_in_synthesis_slot_stalls <= '0;
      end else begin
        // Saturation preserves the minimum-epoch qualification across an
        // arbitrarily long input gap. Wrapping here could defer a ready row by
        // another complete counter interval at low source frame rates.
        if (row_cycle_count < 11'(ROW_CYCLES - 1))
          row_cycle_count <= row_cycle_count + 1'b1;
        if (output_row_seen && !output_issue && !wait_new_frame) begin
          if (!base_out_ready)
            unused_in_synthesis_ready_stalls <=
                unused_in_synthesis_ready_stalls + 1'b1;
          else if (source_active && !slot_ready)
            unused_in_synthesis_slot_stalls <=
                unused_in_synthesis_slot_stalls + 1'b1;
        end
        if (output_issue && output_region == 0 && output_region_pair == 0) begin
`ifndef SYNTHESIS
          if (output_row_seen)
            assert (row_cycle_count == 11'(ROW_CYCLES - 1))
              else $error("composite row started before the ROW_CYCLES minimum epoch: y=%0d cycles=%0d ready_stalls=%0d slot_stalls=%0d raw=%b ll2=%b ll4=%b",
                  output_y, row_cycle_count, unused_in_synthesis_ready_stalls,
                  unused_in_synthesis_slot_stalls, raw_bank_valid,
                  ll2_bank_valid, ll4_bank_valid);
`endif
          row_cycle_count <= '0;
          output_row_seen <= 1'b1;
          unused_in_synthesis_ready_stalls <= '0;
          unused_in_synthesis_slot_stalls <= '0;
        end
      end
    end
  end

  assign busy = repair_busy || base_out_vref || pending_valid || raw_bank_valid != 0 ||
      ll2_bank_valid != 0 || ll4_bank_valid != 0 ||
      mean_bank_valid != 0 || mean_busy;

`ifndef SYNTHESIS
  localparam int OUTPUT_PAYLOAD_BITS = 2 + (2 * PIXEL_BITS) + 6 + SOURCE_Y_BITS;
  localparam logic signed [COEFFICIENT_BITS-1:0] LL2_CERTIFIED_MIN =
      COEFFICIENT_BITS'(-343);
  localparam logic signed [COEFFICIENT_BITS-1:0] LL2_CERTIFIED_MAX =
      COEFFICIENT_BITS'(342);
  localparam logic signed [COEFFICIENT_BITS-1:0] LL4_CERTIFIED_MIN =
      COEFFICIENT_BITS'(-315);
  localparam logic signed [COEFFICIENT_BITS-1:0] LL4_CERTIFIED_MAX =
      COEFFICIENT_BITS'(314);
  logic stalled_output_last;
  logic [OUTPUT_PAYLOAD_BITS-1:0] stalled_output_payload;

  initial begin
    assert (WIDTH == 1280 && HEIGHT == 800)
      else $error("wavelet-guided threshold frontend is production-fixed at 1280x800");
    assert (PIXEL_BITS == 8 && COEFFICIENT_BITS == 14)
      else $error("wavelet-guided threshold frontend requires 8-bit raw/14-bit taps");
    assert (OUTPUT_PAIRS == 848)
      else $error("1280-pixel composite must contain 848 pairs");
    assert (RAW_DEPTH == 5760 && LL2_DEPTH == 1440 && LL4_DEPTH == 360)
      else $error("production circular row-memory geometry changed");
  end

  always_ff @(posedge clk) if (!rst) begin
    if (output_issue && source_active) begin
      case (output_region)
        2'd0: begin
          assert (raw_bank_valid[raw_read_bank] &&
                  raw_bank_y[raw_read_bank] == S0_Y_BITS'(stage_y0))
            else $error("raw circular-buffer row identity mismatch on source read");
          assert (ll2_bank_valid[ll2_raw_reference_bank] &&
                  ll2_bank_y[ll2_raw_reference_bank] == S2_Y_BITS'(stage_y0 >> 2))
            else $error("LL2 circular-buffer identity mismatch on raw comparison");
        end
        2'd1: begin
          assert (ll2_bank_valid[ll2_stage_bank] &&
                  ll2_bank_y[ll2_stage_bank] == S2_Y_BITS'(stage_y2))
            else $error("LL2 circular-buffer row identity mismatch on source read");
          assert (mean_bank_valid[mean_read_bank] &&
                  mean_bank_y[mean_read_bank] ==
                    S4_Y_BITS'(((int'(stage_y2)+1)/4 < STAGE4_HEIGHT) ?
                      (int'(stage_y2)+1)/4 : STAGE4_HEIGHT-1))
            else $error("LL4 mean row identity mismatch on LL2 comparison");
        end
        default: begin
          assert (ll4_bank_valid[ll4_stage_bank] &&
                  ll4_bank_y[ll4_stage_bank] == S4_Y_BITS'(stage_y4))
            else $error("LL4 circular-buffer row identity mismatch on source read");
        end
      endcase
    end
    if (release_stage2)
      assert (ll2_bank_valid[ll2_stage_bank] &&
              ll2_bank_y[ll2_stage_bank] == S2_Y_BITS'(stage_y2))
        else $error("LL2 producer phase failed at final replicated row");
    if (release_stage4)
      assert (ll4_bank_valid[ll4_stage_bank] &&
              ll4_bank_y[ll4_stage_bank] == S4_Y_BITS'(stage_y4))
        else $error("LL4 producer phase failed at final replicated row");
    assert (!(raw_valid && raw_read_en && raw_write_addr == raw_read_addr))
      else $error("raw circular RAM same-address read/write collision");
    assert (!(ll2_valid && ll2_read_en && ll2_write_addr == ll2_read_addr))
      else $error("LL2 circular RAM same-address read/write collision");
    assert (!(ll4_valid && ll4_read_en && ll4_write_addr == ll4_read_addr))
      else $error("LL4 circular RAM same-address read/write collision");
    assert (!(mean_valid && mean_read_en && mean_write_addr == mean_read_addr))
      else $error("LL4 mean circular RAM same-address read/write collision");
    if (ll2_valid)
      assert ($signed(ll2_value[0]) >= LL2_CERTIFIED_MIN &&
              $signed(ll2_value[0]) <= LL2_CERTIFIED_MAX &&
              $signed(ll2_value[1]) >= LL2_CERTIFIED_MIN &&
              $signed(ll2_value[1]) <= LL2_CERTIFIED_MAX)
        else $error("LL2 tap exceeded its certified signed-10-bit range");
    if (ll4_valid)
      assert ($signed(ll4_value[0]) >= LL4_CERTIFIED_MIN &&
              $signed(ll4_value[0]) <= LL4_CERTIFIED_MAX &&
              $signed(ll4_value[1]) >= LL4_CERTIFIED_MIN &&
              $signed(ll4_value[1]) <= LL4_CERTIFIED_MAX)
        else $error("LL4 tap exceeded its certified signed-10-bit range");
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      stalled_output_last <= 1'b0;
      stalled_output_payload <= '0;
    end else begin
      if (stalled_output_last)
        assert ({base_out_vref, base_out_href, base_out_d0, base_out_d1, base_out_t0, base_out_t1,
                 base_out_eol, base_out_row_replicated, base_out_feature_valid_d0,
                 base_out_feature_valid_d1, base_out_source_y} == stalled_output_payload)
          else $error("threshold output payload changed while stalled");
      stalled_output_last <= base_out_vref && base_out_href && !base_out_ready;
      stalled_output_payload <= {base_out_vref, base_out_href, base_out_d0, base_out_d1,
          base_out_t0, base_out_t1, base_out_eol, base_out_row_replicated,
          base_out_feature_valid_d0, base_out_feature_valid_d1, base_out_source_y};
    end
  end
`endif
endmodule

`default_nettype wire
