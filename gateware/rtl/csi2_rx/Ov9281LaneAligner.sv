// Fixed x8 D-PHY byte aligner and bounded lane-skew queue for OV9281 CSI-2.

module Ov9281LaneAligner (
  input  logic       byte_clk,
  input  logic       reset_n,
  input  logic       burst_start,
  input  logic       discard,
  input  logic [7:0] raw_word,
  input  logic       raw_valid,
  input  logic       pair_pop,
  output logic       byte_valid,
  output logic [7:0] byte_data,
  output logic       sync_found,
  output logic       sync_error,
  output logic       queue_overflow
);
  localparam logic [7:0] DPHY_SYNC_BYTE = 8'hb8;
  localparam logic [5:0] SEARCH_LAST_CYCLE = 6'd63;
  localparam logic [3:0] MAX_QUEUE_COUNT = 4'd8;

  logic [7:0] raw_history;
  logic       history_valid;
  logic       search_active;
  logic [5:0] search_cycle;
  logic       phase_locked;
  logic [2:0] locked_offset;

  logic [15:0] search_window;
  logic [7:0]  sync_matches;
  logic        unique_sync_match;
  logic        ambiguous_sync_match;
  logic [2:0]  sync_match_offset;
  logic [7:0]  aligned_byte;

  // Eight parallel eight-bit shift lanes infer compact SRL8 storage. Only
  // accepted pushes move the history; the occupancy selects the oldest byte.
  // Data does not need reset because queue_count defines its validity.
  logic [2:0] queue_read_address;
  logic [7:0] queue_read_data;
  logic [7:0] queue_forward_data;
  logic       queue_forward_valid;
  logic [3:0] queue_count;
  logic       queue_pop_stored;
  logic       queue_bypass_pop;
  logic       queue_push_request;
  logic       queue_store_request;
  logic       queue_push;
  logic       queue_overflow_attempt;

  // Bit zero of each raw word is earlier on the wire. The old raw word is
  // therefore the low half of the sliding stream window.
  assign search_window = {raw_word, raw_history};

  generate
    for (genvar offset = 0; offset < 8; offset = offset + 1) begin : sync_match_bits
      assign sync_matches[offset] =
        search_window[offset +: 8] == DPHY_SYNC_BYTE;
    end
  endgenerate

  // B8 has no proper prefix equal to its suffix, so two matches cannot
  // coexist within these eight overlapping windows. The match vector is
  // always onehot0; the offset encoder therefore needs no priority logic.
  assign unique_sync_match = |sync_matches;
  assign sync_match_offset = {
    |(sync_matches & 8'hf0),
    |(sync_matches & 8'hcc),
    |(sync_matches & 8'haa)
  };
  assign ambiguous_sync_match = 1'b0;
  assign aligned_byte = search_window[{1'b0, locked_offset} +: 8];

  // A newly aligned byte bypasses the storage array when the queue is empty.
  // This removes the otherwise unavoidable extra cycle between the later lane
  // locking and paired consumption, preserving the seven-byte skew bound.
  assign byte_valid = (queue_count != 4'd0) || queue_push_request;
  assign byte_data = queue_count != 4'd0
    ? (queue_forward_valid ? queue_forward_data : queue_read_data)
    : aligned_byte;
  assign queue_pop_stored = pair_pop && (queue_count != 4'd0);
  assign queue_bypass_pop = pair_pop && (queue_count == 4'd0) &&
    queue_push_request;

  // All eight physical entries are usable; empty-queue bypass avoids
  // consuming one merely for the parser handoff. The specified conservative
  // bound remains seven complete aligned bytes of inter-lane skew.
  assign queue_push_request =
    phase_locked && history_valid && raw_valid && !discard;
  assign queue_store_request = queue_push_request && !queue_bypass_pop;
  assign queue_push =
    queue_store_request &&
    ((queue_count < MAX_QUEUE_COUNT) || queue_pop_stored);
  assign queue_overflow_attempt =
    queue_store_request && (queue_count == MAX_QUEUE_COUNT) &&
    !queue_pop_stored;

  // Read the next head from the pre-shift history. A push shifts this byte
  // one position while also increasing the occupancy, so no push adjustment
  // is needed. A stored pop selects the next-youngest byte. Modulo-eight
  // addressing also represents a full queue (count 8 -> oldest index 7).
  assign queue_read_address = queue_count[2:0] - 3'd1
    - {2'b0, queue_pop_stored};

  for (genvar bit_lane = 0; bit_lane < 8; bit_lane = bit_lane + 1) begin : skew_storage
    logic [7:0] history;
    always_ff @(posedge byte_clk) begin
      if (queue_push)
        history <= {history[6:0], aligned_byte[bit_lane]};
      queue_read_data[bit_lane] <= history[queue_read_address];
    end
  end

  // First push and simultaneous single-entry replacement need the new byte,
  // which is not yet in the pre-shift history sampled by the read register.
  always_ff @(posedge byte_clk) begin
    queue_forward_valid <= queue_push && ((queue_count == 4'd0)
      || (queue_pop_stored && (queue_count == 4'd1)));
    queue_forward_data <= aligned_byte;
  end

  always_ff @(posedge byte_clk or negedge reset_n) begin
    if (!reset_n) begin
      raw_history <= 8'b0;
      history_valid <= 1'b0;
      search_active <= 1'b0;
      search_cycle <= 6'b0;
      phase_locked <= 1'b0;
      locked_offset <= 3'b0;
      queue_count <= 4'b0;
      sync_found <= 1'b0;
      sync_error <= 1'b0;
      queue_overflow <= 1'b0;
    end else begin
      sync_found <= 1'b0;
      sync_error <= 1'b0;
      queue_overflow <= 1'b0;

      if (burst_start) begin
        // A new burst always wins over the discard level left by the previous
        // packet. This lets the parent clear discard on the same byte edge as
        // the synchronized burst-start pulse without losing the first word.
        raw_history <= raw_valid ? raw_word : 8'b0;
        history_valid <= raw_valid;
        search_active <= 1'b1;
        search_cycle <= 6'b0;
        phase_locked <= 1'b0;
        locked_offset <= 3'b0;
        queue_count <= 4'b0;
      end else if (discard) begin
        raw_history <= 8'b0;
        history_valid <= 1'b0;
        search_active <= 1'b0;
        search_cycle <= 6'b0;
        phase_locked <= 1'b0;
        locked_offset <= 3'b0;
        queue_count <= 4'b0;
      end else begin
        if (raw_valid && (search_active || phase_locked)) begin
          raw_history <= raw_word;
          history_valid <= 1'b1;
        end

        if (search_active) begin
          // A valid unique match on the final allowed clock wins over the
          // wall-clock timeout. B8 is framing and is never enqueued.
          if (raw_valid && history_valid && unique_sync_match) begin
            search_active <= 1'b0;
            phase_locked <= 1'b1;
            locked_offset <= sync_match_offset;
            sync_found <= 1'b1;
          end else if (raw_valid && history_valid && ambiguous_sync_match) begin
            search_active <= 1'b0;
            phase_locked <= 1'b0;
            sync_error <= 1'b1;
          end else if (search_cycle == SEARCH_LAST_CYCLE) begin
            search_active <= 1'b0;
            phase_locked <= 1'b0;
            sync_error <= 1'b1;
          end else begin
            search_cycle <= search_cycle + 6'd1;
          end
        end

        unique case ({queue_push, queue_pop_stored})
          2'b10: queue_count <= queue_count + 4'd1;
          2'b01: queue_count <= queue_count - 4'd1;
          default: queue_count <= queue_count;
        endcase

        if (queue_overflow_attempt) begin
          queue_overflow <= 1'b1;
        end
      end
    end
  end

`ifndef SYNTHESIS
  always_ff @(posedge byte_clk or negedge reset_n) begin
    if (!reset_n) begin
      // Assertion state follows the same asynchronous reset as the design.
    end else begin
      assert (!(queue_push && queue_overflow_attempt));
    end
  end
`endif
endmodule
