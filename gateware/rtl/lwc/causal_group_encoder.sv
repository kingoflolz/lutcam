`default_nettype none

// Single-pass causal scalar group encoder.
//
// A group is accepted as eight consecutive two-coefficient transfers. The
// causal decision uses only masks and exact candidate lengths. Once selected,
// a header and eight bounded pair tokens are appended to one chronological
// output reservoir. No complete packet or payload bitmap is materialized.
module causal_group_encoder #(
    // The reusable leaf defaults to frame-budget-only allocation. Production
    // pixel wrappers enable the elapsed-clock burst budget explicitly.
    parameter bit BURST_LIMIT_ENABLE = 1'b0
) (
    input  wire                         clk,
    input  wire                         rst,
    input  wire                         clear_frame,

    input  wire                         pair_valid,
    output logic                        pair_ready,
    input  wire                         pair_group_start,
    input  wire                         pair_group_last,
    input  wire [3:0]                   pair_band,
    input  wire signed [7:0]            pair_q0,
    input  wire signed [7:0]            pair_q1,
    input  wire [1:0]                   pair_correction,
    input  wire [1:0]                   pair_residual_class,

    output logic                        out_sof,
    output logic                        out_valid,
    input  wire                         out_ready,
    output logic [15:0]                 out_data,
    output logic                        frame_done,

    output logic                        packets_done,
    output logic [20:0]                 packet_bits,
    output logic [15:0]                 groups_committed,
    output logic [15:0]                 groups_serialized,
    output logic signed [20:0]          rate_debt,
    output logic [19:0]                 extra_room,
    output logic                        protocol_error,
    // A transport may represent these pure-zero words as a run. The final
    // semantic word, including any sub-word zero fill, is never marked.
    output wire                         out_padding
);
    import codec_profile_pkg::*;

    localparam logic [19:0] INITIAL_EXTRA_ROOM = 20'd960_000;
    localparam logic [15:0] FRAME_GROUPS = 16'd64_000;
    localparam logic [15:0] FRAME_WORDS_MINUS_ONE = 16'd63_999;
    localparam logic [4:0] MAX_PAIR_TOKEN_BITS = 5'd17;
    localparam int OUTPUT_RESERVOIR_BITS = 46;
    localparam logic [5:0] OUTPUT_RESERVOIR_COUNT_MAX = 6'd46;
    localparam logic [1:0] FINALIZER_IDLE = 2'd0;
    localparam logic [1:0] FINALIZER_COARSE = 2'd1;
    localparam logic [1:0] FINALIZER_FINE_LOW = 2'd2;
    localparam logic [1:0] FINALIZER_READ_HIGH = 2'd3;
    // cutoff <= 8 makes 134*12 = 1608 the largest possible selected-packet
    // admission requirement. The 147-bit coarse raw candidate is always
    // dominated by the 135-bit fine raw candidate. Incremental energy is at
    // most 16*24 = 384, so clipping
    // direct energy at 1608+384 preserves the D-I coarse-admission test and
    // the nested incremental fine-admission test.
    localparam logic [10:0] DIRECT_ENERGY_LIMIT = 11'd1992;

    // Fine q and its coarse-rounding correction share one native 16x18
    // dual-port RAM10. Address bit three selects the independently
    // collected/emitted context; low bits are the pair index.
    logic [17:0] pair_ram [0:15];
    // One RAM10 stores the complete two-nibble canonical table.  Its single
    // eighteen-bit read port services the low and high mask bytes on
    // consecutive finalizer clocks.  Avoiding simultaneous reads lets the
    // table use the native 512x20 organization instead of two DPRAM10 slices.
    logic [17:0] huffman_pair_rom [0:255];
    // A compact two-read lookup replaces the serializer's two variable
    // unary-code shifters.  Five 129-entry magnitude tables fit in one
    // 1024-deep address space; sign remains a two-bit pipeline sideband.
    (* ram_style = "block", syn_ramstyle = "block_ram" *)
    logic [20:0] rice_token_rom [0:1023];
    logic [20:0] rice_token_q0;
    logic [20:0] rice_token_q1;
    logic [9:0] rice_token_address0;
    logic [9:0] rice_token_address1;
    logic rice_token_read_enable;
    logic rice_token_pending;
    logic rice_sign0_q;
    logic rice_sign1_q;
    logic [17:0] huffman_low_pair;
    logic [17:0] huffman_pair_q;
    logic [17:0] huffman_current_pair;
    logic huffman_pair_read_enable;
    logic [7:0] huffman_pair_read_symbols;
    logic fill_context;
    logic [3:0] pair_ram_write_address;
    logic [17:0] pair_ram_write_data;
    logic pair_ram_read_enable;
    logic [3:0] pair_ram_read_address;
    logic [17:0] pair_ram_read_data;
    logic pair_read_pending;

    logic fill_active;
    logic [2:0] fill_pair;
    logic [3:0] fill_band;
    logic [13:0] fill_fine_mask;
    logic [13:0] fill_coarse_mask;
    logic [4:0] fill_fine_huffman_bits;
    logic [4:0] fill_coarse_huffman_bits;
    logic [10:0] fill_direct_energy;
    logic [8:0] fill_incremental_energy;
    logic [7:0] fill_fine_prefix;
    logic [7:0] fill_coarse_prefix;
    logic [2:0] fill_coarse_previous_magnitude;
    logic fill_fine_pair_overflow;
    logic fill_coarse_pair_overflow;

    logic [7:0] pair_coarse0;
    logic [7:0] pair_coarse1;
    // Both coefficient-energy lookups share one native dual-read RAM. The
    // collector accepts the current pair while the previous pair updates its
    // sum. On the existing COARSE finalizer cycle, that sum covers all but the
    // final pair, whose lookup has just completed; the decision edge does not
    // move. energy_start also preserves malformed one-pair-group semantics.
    (* syn_ramstyle = "block_ram" *) logic [10:0] direct_energy_rom [0:127];
    logic energy_pending;
    logic energy_start;
    logic [10:0] final_direct_gain;
    logic [10:0] pair_direct_energy0;
    logic [10:0] pair_direct_energy1;
    logic [4:0] pair_incremental_energy0;
    logic [4:0] pair_incremental_energy1;
    logic [5:0] pair_incremental_energy_sum;

    logic [15:0] complete_fine_mask;
    logic [15:0] complete_coarse_mask;
    logic [4:0] complete_fine_huffman_bits;
    logic [4:0] complete_coarse_huffman_bits;
    logic [10:0] complete_direct_energy;
    logic [8:0] complete_incremental_energy;
    logic [8:0] complete_incremental_gain;
    logic [7:0] complete_fine_bits;
    logic [7:0] complete_coarse_bits;
    logic complete_fine_pair_overflow;
    logic complete_coarse_pair_overflow;

    logic [1:0] finalizer_state;
    logic [15:0] final_fine_mask;
    logic [15:0] final_coarse_mask;
    logic [4:0] final_fine_huffman_bits;
    logic [4:0] final_coarse_huffman_bits;
    logic [7:0] final_fine_bits;
    logic [7:0] final_coarse_bits;
    logic [8:0] final_incremental_gain;
    logic [10:0] final_coarse_gain;
    logic [3:0] final_band;
    logic [7:0] final_nominal_bits;
    logic [3:0] final_energy_shift_profile;
    logic [3:0] final_coarse_cutoff;
    logic [3:0] final_incremental_cutoff;
    logic final_coarse_pass;
    logic final_incremental_pass;
    logic decision_gain_pass;
    logic final_coarse_fits;
    logic signed [21:0] final_pre_debt;
    logic signed [20:0] final_post_debt;
    logic [1:0] final_selected_state;
    logic [7:0] final_selected_bits;
    logic [15:0] final_selected_mask;
    logic final_selected_huffman;

    logic pair_fire;
    logic group_finalize;

    logic [7:0] pair_fine_prefix_next;
    logic [7:0] pair_coarse_prefix_next;
    logic [8:0] pair_fine_token_length;
    logic [8:0] pair_coarse_token_length;

    logic start_build;
    logic header_service;
    logic selection_commit;
    logic pending_valid;
    logic pending_fine;
    logic pending_huffman;
    logic [1:0] pending_header_phase;
    logic build_active;
    logic build_context;
    logic build_fine;
    logic [3:0] build_band;
    logic build_raw;
`ifndef SYNTHESIS
    // Retain the original diagnostic payload in simulation only.
    logic [7:0] build_length;
`endif
    logic [3:0] build_pairs_issued;
    logic [2:0] build_pairs_consumed;
    wire [3:0] build_service_clocks = {1'b0,build_pairs_consumed} + 4'd1;
    logic [2:0] build_previous_magnitude;
    logic signed [7:0] build_value0;
    logic signed [7:0] build_value1;
    logic [16:0] build_pair_token;
    logic [4:0] build_pair_token_length;
    logic build_pair_fire;
    logic build_complete_now;
    logic pair_advance;
    logic header_pair_prefetch_start;
    logic header_rice_issue;

    logic token_valid;
    logic [16:0] token_bits;
    logic [4:0] token_count;
    logic token_ready;
    logic token_fire;
    logic packet_complete;
    logic zero_absorb;
    logic guaranteed_zero_credit;
    logic run_flush_pending;
    logic run_token_fire;
    logic zero_run_valid;
    logic [7:0] zero_run_count;
    logic [8:0] serialized_advance;
    logic [7:0] charged_bits;
    logic [19:0] room_charge;

    logic [OUTPUT_RESERVOIR_BITS-1:0] output_stage;
    logic [OUTPUT_RESERVOIR_BITS-1:0] output_stage_after;
    logic [OUTPUT_RESERVOIR_BITS-1:0] output_stage_next;
    logic [5:0] output_count;
    logic [5:0] output_count_after;
    logic [5:0] output_count_next;
    logic [15:0] output_words;
    logic output_fire;
    logic padding_mode;
    logic [15:0] burst_credit;
    logic [15:0] burst_available;
    logic [15:0] burst_decision_available;
    logic [2:0] saved_burst_cutoff;
    logic [3:0] burst_cutoff;

    assign out_padding = out_valid && padding_mode && output_count == 0;

    // Additional admission room in the existing causal decision, not an
    // output throttle. Two bits of idle/active clock time replenish each
    // clock, up to 45,056 bits (5.5 KiB). Clearing a frame discards credit; it never
    // grants a free burst and makes the profile independent of interframe idle.
    assign burst_available = burst_credit >= 16'd45_054
        ? 16'd45_056 : burst_credit + 16'd2;
    // A finalized group advances through COARSE and FINE_LOW on consecutive
    // clocks, without an intervening selection charge. Predict the FINE_LOW
    // admission's three replenishments here, then hold its quality cutoff
    // across both decisions. This separates credit arithmetic from the shared
    // gain comparator without changing decision cycles or the live hard fit.
    assign burst_decision_available = burst_credit >= 16'd45_050
        ? 16'd45_056 : burst_credit + 16'd6;
    // The largest deficit gives cutoff five, so no saturation comparator is
    // needed. Hard admission below still prevents spending unavailable credit.
    assign burst_cutoff = {1'b0,saved_burst_cutoff};
    always_ff @(posedge clk) begin
        if (rst || clear_frame) begin
            burst_credit <= 0;
            saved_burst_cutoff <= 0;
        end else if (BURST_LIMIT_ENABLE) begin
            if (group_finalize)
                saved_burst_cutoff <=
                    3'((16'd45_056 - burst_decision_available) >> 13);
            if (selection_commit)
                // Charge a full bit for each selected zero, even when the
                // frame allocator refunds bits saved by run coding. This
                // deliberately overestimates eventual semantic output.
                burst_credit <= burst_available - {8'd0,final_selected_bits};
            else
                burst_credit <= burst_available;
        end
    end


    // The analyzer consumes only zero tests, Rice lengths and clipped
    // magnitude context. Preserve magnitude directly instead of negating
    // it to a signed coarse value and immediately taking absolute value.
    function automatic logic [7:0] corrected_coarse_q(
        input logic signed [7:0] fine,
        input logic correction
    );
        corrected_coarse_q = fine == 0 ? 0 :
            (magnitude8(fine) >> 1) + correction;
    endfunction

    // The serializer does not have the analyzer's magnitude-energy cone to
    // share.  Form the same signed coarse value directly from an arithmetic
    // half and its one-bit rounding adjustment.
    function automatic logic signed [7:0] serialized_coarse_q(
        input logic signed [7:0] fine,
        input logic correction
    );
        logic signed [7:0] floor_half;
        logic signed [1:0] adjustment;
        begin
            floor_half = fine >>> 1;
            adjustment = 0;
            if (fine[7]) begin
                case ({fine[0],correction})
                    2'b01: adjustment = -2'sd1;
                    2'b10: adjustment = 2'sd1;
                    default: adjustment = 0;
                endcase
            end else if (fine != 0 && correction) begin
                adjustment = 2'sd1;
            end
            serialized_coarse_q = floor_half
                + {{6{adjustment[1]}},adjustment};
        end
    endfunction

    function automatic logic [2:0] fixed_fine_rice_k(
        input logic [3:0] band
    );
        case (band)
            0,1: fixed_fine_rice_k = 4;
            2: fixed_fine_rice_k = 2;
            3,4: fixed_fine_rice_k = 1;
            default: fixed_fine_rice_k = 0;
        endcase
    endfunction

    // Only coarse coding uses packet-local context. Spell out the exact
    // band/magnitude table directly so analyzer and serializer do not build
    // an eleven-bit profile mux followed by a generic threshold selector.
    function automatic logic [2:0] contextual_coarse_rice_k(
        input logic [3:0] band,
        input logic [2:0] previous_magnitude
    );
        begin
            case (band)
                0: begin
                    case (previous_magnitude)
                        0: contextual_coarse_rice_k = 1;
                        1: contextual_coarse_rice_k = 3;
                        default: contextual_coarse_rice_k = 2;
                    endcase
                end
                1: begin
                    case (previous_magnitude)
                        1: contextual_coarse_rice_k = 1;
                        default: contextual_coarse_rice_k = 2;
                    endcase
                end
                2: contextual_coarse_rice_k = 2;
                3,4: begin
                    case (previous_magnitude)
                        0,1,2,3,4: contextual_coarse_rice_k = 1;
                        default: contextual_coarse_rice_k = 2;
                    endcase
                end
                5: begin
                    case (previous_magnitude)
                        0,1: contextual_coarse_rice_k = 0;
                        default: contextual_coarse_rice_k = 1;
                    endcase
                end
                6,7: begin
                    case (previous_magnitude)
                        0,1,2: contextual_coarse_rice_k = 0;
                        default: contextual_coarse_rice_k = 1;
                    endcase
                end
                default: contextual_coarse_rice_k = 0;
            endcase
        end
    endfunction

    function automatic logic [9:0] rice_token_address_for(
        input logic [7:0] magnitude,
        input logic [2:0] k
    );
        logic [9:0] base;
        begin
            case (k)
                0: base = 0;
                1: base = 129;
                2: base = 258;
                3: base = 387;
                default: base = 516;
            endcase
            rice_token_address_for = base + {2'd0,magnitude};
        end
    endfunction

    function automatic logic [2:0] rice_context_magnitude(
        input logic signed [7:0] value
    );
        logic [7:0] magnitude;
        begin
            magnitude = magnitude8(value);
            rice_context_magnitude = |magnitude[7:3]
                ? 3'd7 : magnitude[2:0];
        end
    endfunction

    function automatic logic [10:0] saturated_direct_sum(
        input logic [10:0] base,
        input logic [10:0] value0,
        input logic [10:0] value1
    );
        logic [12:0] sum;
        begin
            sum = {2'd0,base} + {2'd0,value0} + {2'd0,value1};
            saturated_direct_sum = sum >= {2'd0,DIRECT_ENERGY_LIMIT}
                ? DIRECT_ENERGY_LIMIT : sum[10:0];
        end
    endfunction

    function automatic logic [10:0] direct_energy(
        input logic signed [7:0] value,
        input logic correction,
        input logic residual_class
    );
        logic [7:0] magnitude;
        logic [2:0] coefficient;
        logic assists;
        logic [7:0] factor;
        logic [12:0] product;
        begin
            magnitude = magnitude8(value);
            if (magnitude >= 17) begin
                direct_energy = DIRECT_ENERGY_LIMIT;
            end else if (magnitude == 0) begin
                direct_energy = 0;
            end else begin
                coefficient = magnitude[0]
                    ? (residual_class ? 3'd6 : 3'd2) : 3'd4;
                assists = magnitude[0] ? correction : residual_class;
                factor = {magnitude[4:0],3'b000};
                factor = assists ? factor + {5'd0,coefficient}
                    : factor - {5'd0,coefficient};
                product = magnitude[4:0] * factor;
                direct_energy = product >= {2'd0,DIRECT_ENERGY_LIMIT}
                    ? DIRECT_ENERGY_LIMIT : product[10:0];
            end
        end
    endfunction

    function automatic logic [4:0] incremental_energy(
        input logic fine_is_odd,
        input logic correction,
        input logic residual_class
    );
        if (fine_is_odd)
            incremental_energy = residual_class ? 2 : 6;
        else if (correction)
            incremental_energy = 24;
        else
            incremental_energy = 0;
    endfunction

    function automatic logic gain_passes_cutoff(
        input logic [10:0] energy,
        input logic [7:0] extra_bits,
        input logic [3:0] cutoff,
        input logic [1:0] metric_shift
    );
        logic [11:0] required_gain;
        logic [3:0] factor;
        logic [13:0] shifted_energy;
        begin
            // Map the small cutoff table into a hard multiplier. This
            // replaces the parallel constant-product adder/mux tree while
            // retaining the complete 8-bit extra_bits input range.
            case (cutoff)
                1: factor=1;
                2,3: factor=2;
                4: factor=3;
                5: factor=4;
                6: factor=6;
                7: factor=8;
                default: factor=12;
            endcase
            required_gain = extra_bits * factor;
            // For nonnegative integers, E >= ceil(R / 2**s) is exactly
            // equivalent to (E << s) >= R. Keep all fourteen shifted energy
            // bits so this removes rounding hardware without saturation.
            shifted_energy = {3'd0,energy} << metric_shift;
            if (cutoff == 0) begin
                gain_passes_cutoff = 1'b1;
            end else if (extra_bits == 0) begin
                gain_passes_cutoff = energy != 0;
            end else begin
                gain_passes_cutoff = shifted_energy >= {2'd0,required_gain};
            end
        end
    endfunction

    function automatic logic [7:0] exact_packet_bits(
        input logic [15:0] mask,
        input logic [7:0] literal_bits,
        input logic [4:0] huffman_bits,
        input logic fine
    );
        logic [4:0] mask_bits;
        logic [7:0] encoded_bits;
        begin
            mask_bits = huffman_bits;
            if (mask_bits > 16)
                mask_bits = 16;
            // The literal accumulator is saturated at 135 and the mask at
            // sixteen, so the complete non-raw packet length fits in eight
            // bits.  Fold the two prefix bits and the final entropy/raw flag
            // into one +3 length calculation.
            encoded_bits = literal_bits + {3'd0,mask_bits} + 8'd3;
            if (mask == 0)
                exact_packet_bits = 1;
            else if (encoded_bits >= 8'd135)
                exact_packet_bits = fine ? 8'd135 : 8'd147;
            else
                exact_packet_bits = encoded_bits;
        end
    endfunction

    function automatic logic [17:0] huffman_pair_entry(
        input logic [7:0] symbols
    );
        logic [6:0] first_code;
        logic [6:0] second_code;
        logic [3:0] first_length;
        logic [3:0] second_length;
        logic [3:0] pair_length;
        logic [13:0] shifted_first;
        logic [13:0] right_aligned_code;
        logic [13:0] left_aligned_code;
        begin
            first_code = nibble_huffman_code(symbols[3:0]);
            second_code = nibble_huffman_code(symbols[7:4]);
            first_length = nibble_huffman_length(symbols[3:0]);
            second_length = nibble_huffman_length(symbols[7:4]);
            case (second_length)
                1: shifted_first = {6'd0,first_code,1'd0};
                3: shifted_first = {4'd0,first_code,3'd0};
                4: shifted_first = {3'd0,first_code,4'd0};
                5: shifted_first = {2'd0,first_code,5'd0};
                6: shifted_first = {1'd0,first_code,6'd0};
                default: shifted_first = {first_code,7'd0};
            endcase
            pair_length = first_length + second_length;
            right_aligned_code = shifted_first | {7'd0,second_code};
            left_aligned_code = right_aligned_code
                << (4'd14 - pair_length);
            huffman_pair_entry = {pair_length,left_aligned_code};
        end
    endfunction

    integer huffman_rom_index;
    initial begin
        for (huffman_rom_index = 0; huffman_rom_index < 256;
                huffman_rom_index = huffman_rom_index + 1)
            huffman_pair_rom[huffman_rom_index]
                = huffman_pair_entry(huffman_rom_index[7:0]);
    end

    function automatic logic [7:0] rice_zeros_for(
        input logic signed [7:0] value,
        input logic [2:0] k
    );
        logic [7:0] magnitude;
        begin
            magnitude = magnitude8(value);
            rice_zeros_for = magnitude == 0
                ? 0 : (magnitude - 1'b1) >> k;
        end
    endfunction

    function automatic logic [3:0] rice_low_for(
        input logic signed [7:0] value,
        input logic [2:0] k
    );
        logic [7:0] magnitude;
        logic [3:0] remainder_value;
        begin
            magnitude = magnitude8(value);
            remainder_value = magnitude == 0
                ? 0 : magnitude[3:0] - 1'b1;
            case (k)
                0: rice_low_for = 0;
                1: rice_low_for = {3'd0,remainder_value[0]};
                2: rice_low_for = {2'd0,remainder_value[1:0]};
                3: rice_low_for = {1'd0,remainder_value[2:0]};
                default: rice_low_for = remainder_value[3:0];
            endcase
        end
    endfunction

    function automatic logic [7:0] rice_length_for(
        input logic signed [7:0] value,
        input logic [2:0] k
    );
        rice_length_for = value == 0
            ? 0 : 8'd2 + {5'd0,k} + rice_zeros_for(value,k);
    endfunction

    // The analyzer only needs lengths. Spell out its four legal Rice shifts
    // so the four concurrent coefficient paths do not infer generic
    // eight-bit variable shifters.
    function automatic logic [7:0] fixed_rice_length(
        input logic [7:0] magnitude,
        input logic [2:0] k
    );
        logic [7:0] decremented;
        begin
            decremented = magnitude - 1'b1;
            if (magnitude == 0) begin
                fixed_rice_length = 0;
            end else begin
                case (k)
                    0: fixed_rice_length = 8'd2 + decremented;
                    1: fixed_rice_length = 8'd3
                        + {1'b0,decremented[7:1]};
                    2: fixed_rice_length = 8'd4
                        + {2'd0,decremented[7:2]};
                    3: fixed_rice_length = 8'd5
                        + {3'd0,decremented[7:3]};
                    default: fixed_rice_length = 8'd6
                        + {4'd0,decremented[7:4]};
                endcase
            end
        end
    endfunction

    function automatic logic [8:0] fine_pair_length(
        input logic signed [7:0] value0,
        input logic signed [7:0] value1,
        input logic [3:0] band
    );
        logic [7:0] magnitude0;
        logic [7:0] magnitude1;
        logic [2:0] k;
        begin
            magnitude0 = magnitude8(value0);
            magnitude1 = magnitude8(value1);
            k = fixed_fine_rice_k(band);
            fine_pair_length = {1'b0,fixed_rice_length(magnitude0,k)}
                + {1'b0,fixed_rice_length(magnitude1,k)};
        end
    endfunction

    function automatic logic [8:0] coarse_pair_length(
        input logic [7:0] value0,
        input logic [7:0] value1,
        input logic [3:0] band,
        input logic [2:0] previous_magnitude
    );
        logic [7:0] magnitude0;
        logic [7:0] magnitude1;
        logic [2:0] context0;
        logic [2:0] k0;
        logic [2:0] k1;
        begin
            magnitude0 = value0;
            magnitude1 = value1;
            context0 = |magnitude0[7:3] ? 3'd7 : magnitude0[2:0];
            k0 = contextual_coarse_rice_k(band,previous_magnitude);
            k1 = contextual_coarse_rice_k(band,context0);
            coarse_pair_length = {1'b0,fixed_rice_length(magnitude0,k0)}
                + {1'b0,fixed_rice_length(magnitude1,k1)};
        end
    endfunction

    // A bounded Rice literal is left aligned so two coefficient literals can
    // be concatenated with one small shift. The causal length calculation
    // forces the whole group to raw if a selected pair would exceed seventeen
    // bits, so no normal token is truncated by this representation.
    function automatic logic [16:0] rice_token_for(
        input logic signed [7:0] value,
        input logic [2:0] k
    );
        logic [16:0] tail;
        logic [3:0] low;
        logic [7:0] zeros;
        begin
            tail = 0;
            low = rice_low_for(value,k);
            zeros = rice_zeros_for(value,k);
            case (k)
                0: tail = 17'h10000;
                1: tail = {1'b1,low[0],15'd0};
                2: tail = {1'b1,low[1:0],14'd0};
                3: tail = {1'b1,low[2:0],13'd0};
                default: tail = {1'b1,low,12'd0};
            endcase
            if (value == 0) begin
                rice_token_for = 0;
            end else begin
                rice_token_for = (value < 0 ? 17'h10000 : 17'd0)
                    | (tail >> (zeros + 1'b1));
            end
        end
    endfunction

    function automatic logic [20:0] rice_token_rom_entry(
        input logic [7:0] magnitude,
        input logic [2:0] k
    );
        logic signed [7:0] representative;
        logic [15:0] token;
        begin
            // Magnitude 128 has only the signed -128 representative; its
            // low sixteen code bits are nevertheless the common magnitude
            // suffix stored by this signless table.
            representative = magnitude == 8'd128
                ? -8'sd128 : $signed(magnitude);
            token = 16'(rice_token_for(representative,k));
            rice_token_rom_entry = {
                5'(rice_length_for(representative,k)),token};
        end
    endfunction

    integer rice_k_index;
    integer rice_magnitude_index;
    initial begin : rice_token_rom_init
        for (rice_magnitude_index = 0; rice_magnitude_index < 1024;
                rice_magnitude_index = rice_magnitude_index + 1)
            rice_token_rom[rice_magnitude_index] = 0;
        for (rice_k_index = 0; rice_k_index < 5;
                rice_k_index = rice_k_index + 1)
            for (rice_magnitude_index = 0; rice_magnitude_index <= 128;
                    rice_magnitude_index = rice_magnitude_index + 1)
                rice_token_rom[rice_k_index * 129 + rice_magnitude_index]
                    = rice_token_rom_entry(rice_magnitude_index[7:0],
                        rice_k_index[2:0]);
    end

    function automatic logic [7:0] saturated_payload_add(
        input logic [7:0] base,
        input logic [8:0] increment
    );
        logic [9:0] sum;
        begin
            sum = {2'd0,base} + {1'b0,increment};
            saturated_payload_add = sum >= 10'd135
                ? 8'd135 : sum[7:0];
        end
    endfunction


    function automatic logic [6:0] direct_energy_address(
        input logic signed [7:0] value,
        input logic correction,
        input logic residual_class
    );
        logic [7:0] magnitude;
        begin
            magnitude = magnitude8(value);
            direct_energy_address = {residual_class, correction,
                (|magnitude[7:5]) ? 5'd31 : magnitude[4:0]};
        end
    endfunction

    initial begin
        for (integer index=0; index<128; index=index+1)
            direct_energy_rom[index] = direct_energy(
                $signed({3'd0, 5'(index)}), 1'(index >> 5), 1'(index >> 6));
    end

    always_ff @(posedge clk) begin
        if (pair_fire) begin
            pair_direct_energy0 <= direct_energy_rom[
                direct_energy_address(pair_q0, pair_correction[0], pair_residual_class[0])];
            pair_direct_energy1 <= direct_energy_rom[
                direct_energy_address(pair_q1, pair_correction[1], pair_residual_class[1])];
        end
        if (rst || clear_frame) begin
            energy_pending <= 1'b0;
            energy_start <= 1'b0;
        end else begin
            energy_pending <= pair_fire;
            if (pair_fire) energy_start <= !fill_active;
        end
    end

    always_comb begin : pair_analysis
        pair_coarse0 = corrected_coarse_q(pair_q0,pair_correction[0]);
        pair_coarse1 = corrected_coarse_q(pair_q1,pair_correction[1]);
        pair_incremental_energy0 = incremental_energy(
            pair_q0[0],pair_correction[0],pair_residual_class[0]);
        pair_incremental_energy1 = incremental_energy(
            pair_q1[0],pair_correction[1],pair_residual_class[1]);
        pair_incremental_energy_sum = {1'b0,pair_incremental_energy0}
            + {1'b0,pair_incremental_energy1};
    end

    always_comb begin : pair_length_accumulator
        logic [7:0] fine_base;
        logic [7:0] coarse_base;
        logic [2:0] coarse_previous_magnitude;

        coarse_previous_magnitude = fill_active
            ? fill_coarse_previous_magnitude : 0;
        fine_base = fill_active ? fill_fine_prefix : 0;
        coarse_base = fill_active ? fill_coarse_prefix : 0;
        pair_fine_token_length = fine_pair_length(pair_q0,pair_q1,
            pair_band);
        pair_coarse_token_length = coarse_pair_length(pair_coarse0,
            pair_coarse1,pair_band,coarse_previous_magnitude);
        pair_fine_prefix_next = saturated_payload_add(fine_base,
            pair_fine_token_length);
        pair_coarse_prefix_next = saturated_payload_add(coarse_base,
            pair_coarse_token_length);
    end

    always_comb begin : group_finalizer

        pair_ready = !rst;
        if (fill_active) begin
            // A completed candidate occupies the shared final registers until
            // its third header slice transfers.  The first seven pairs of the
            // following group may overlap that wait; hold only its final pair.
            if (fill_pair == 7 && pending_valid)
                pair_ready = 1'b0;
        end else if (finalizer_state != FINALIZER_IDLE || build_active
                || (zero_run_valid && zero_run_count == 0)) begin
            // Do not start writing the context still being drained by a
            // backpressured packet builder.
            pair_ready = 1'b0;
        end
        pair_fire = pair_valid && pair_ready;
        group_finalize = pair_fire && pair_group_last;

        complete_fine_mask = {pair_q1 != 0,pair_q0 != 0,
            fill_active ? fill_fine_mask : 14'd0};
        complete_coarse_mask = {pair_coarse1 != 0,pair_coarse0 != 0,
            fill_active ? fill_coarse_mask : 14'd0};
        complete_fine_huffman_bits = fill_active
            ? fill_fine_huffman_bits : 0;
        complete_coarse_huffman_bits = fill_active
            ? fill_coarse_huffman_bits : 0;
        if (fill_pair[0]) begin
            complete_fine_huffman_bits
                = 5'(complete_fine_huffman_bits
                    + {1'd0,nibble_huffman_length(
                        complete_fine_mask[15:12])});
            complete_coarse_huffman_bits
                = 5'(complete_coarse_huffman_bits
                    + {1'd0,nibble_huffman_length(
                        complete_coarse_mask[15:12])});
        end
        complete_direct_energy = saturated_direct_sum(energy_start
                ? 11'd0 : fill_direct_energy,
            pair_direct_energy0,pair_direct_energy1);
        complete_incremental_energy = (fill_active
                ? fill_incremental_energy : 9'd0)
            + {3'd0,pair_incremental_energy_sum};
        complete_incremental_gain = complete_incremental_energy;
        final_direct_gain = complete_direct_energy > {2'd0,final_incremental_gain}
            ? complete_direct_energy - {2'd0,final_incremental_gain} : 0;
        complete_fine_pair_overflow = (fill_active
            ? fill_fine_pair_overflow : 1'b0)
            || pair_fine_token_length > {4'd0,MAX_PAIR_TOKEN_BITS};
        complete_coarse_pair_overflow = (fill_active
            ? fill_coarse_pair_overflow : 1'b0)
            || pair_coarse_token_length > {4'd0,MAX_PAIR_TOKEN_BITS};
        complete_fine_bits = complete_fine_mask != 0
                && complete_fine_pair_overflow
            ? 8'd135 : exact_packet_bits(complete_fine_mask,
                pair_fine_prefix_next,complete_fine_huffman_bits,1'b1);
        complete_coarse_bits = complete_coarse_mask != 0
                && complete_coarse_pair_overflow
            ? 8'd147 : exact_packet_bits(complete_coarse_mask,
                pair_coarse_prefix_next,complete_coarse_huffman_bits,1'b0);
    end

    always_comb begin : causal_decision
        logic [7:0] nominal;
        logic [7:0] coarse_extra;
        logic [7:0] fine_extra;
        logic [7:0] incremental_extra;
        logic coarse_fits;
        logic fine_fits;
        logic coarse_burst_fits;
        logic fine_burst_fits;
        logic fine_dominates_coarse;
        logic [10:0] decision_energy;
        logic [7:0] decision_extra;
        logic [3:0] decision_cutoff;
        logic [1:0] decision_shift;

        nominal = final_nominal_bits;
        final_pre_debt = $signed({rate_debt[20],rate_debt})
            - $signed({14'd0,nominal});
        {final_incremental_cutoff,final_coarse_cutoff}
            = rate_cutoffs(final_pre_debt);
        // The stronger of frame pressure and elapsed-time burst pressure
        // raises the existing quality threshold before hard admission fills.
        // This preserves credit for high-gain groups instead of waiting for a
        // full bucket deficit to force whole groups to zero.
        if (BURST_LIMIT_ENABLE) begin
            if (burst_cutoff > final_coarse_cutoff)
                final_coarse_cutoff = burst_cutoff;
            if (burst_cutoff > final_incremental_cutoff)
                final_incremental_cutoff = burst_cutoff;
        end

        coarse_extra = final_coarse_bits - 1'b1;
        fine_extra = final_fine_bits - 1'b1;
        incremental_extra = final_fine_bits > final_coarse_bits
            ? final_fine_bits - final_coarse_bits : 0;
        coarse_fits = {12'd0,coarse_extra} <= extra_room;
        fine_fits = {12'd0,fine_extra} <= extra_room;
        coarse_burst_fits = !BURST_LIMIT_ENABLE
            || {8'd0,final_coarse_bits} <= burst_available;
        fine_burst_fits = !BURST_LIMIT_ENABLE
            || {8'd0,final_fine_bits} <= burst_available;
        fine_dominates_coarse = final_fine_bits <= final_coarse_bits;

        final_coarse_fits = coarse_fits;
        decision_energy = finalizer_state == FINALIZER_COARSE
            ? final_direct_gain : {2'd0,final_incremental_gain};
        decision_extra = finalizer_state == FINALIZER_COARSE
            ? coarse_extra : incremental_extra;
        decision_cutoff = finalizer_state == FINALIZER_COARSE
            ? final_coarse_cutoff : final_incremental_cutoff;
        decision_shift = finalizer_state == FINALIZER_COARSE
            ? final_energy_shift_profile[1:0]
            : final_energy_shift_profile[3:2];
        decision_gain_pass = gain_passes_cutoff(decision_energy,
            decision_extra,decision_cutoff,decision_shift);
        final_incremental_pass = decision_gain_pass;

        final_selected_state = 0;
        final_selected_bits = 1;
        final_selected_mask = 0;
        final_selected_huffman = 1'b0;
        if (finalizer_state == FINALIZER_FINE_LOW
                && final_coarse_pass) begin
            if ((fine_dominates_coarse || final_incremental_pass)
                    && fine_fits && fine_burst_fits) begin
                final_selected_state = 2;
                final_selected_bits = final_fine_bits;
                final_selected_mask = final_fine_mask;
                final_selected_huffman = final_fine_huffman_bits <= 16;
            end else if (coarse_burst_fits) begin
                final_selected_state = 1;
                final_selected_bits = final_coarse_bits;
                final_selected_mask = final_coarse_mask;
                final_selected_huffman = final_coarse_huffman_bits <= 16;
            end
        end
    end

    assign selection_commit = finalizer_state == FINALIZER_FINE_LOW;
    assign zero_absorb = selection_commit && final_selected_state == 0;
    assign guaranteed_zero_credit = zero_absorb && zero_run_valid
        && zero_run_count >= 8'd15;
    assign charged_bits = guaranteed_zero_credit
        ? 8'd0 : final_selected_bits;
    assign room_charge = guaranteed_zero_credit
        ? 20'hf_ffff : {12'd0,final_selected_bits - 1'b1};
    assign final_post_debt = $signed(final_pre_debt[20:0])
        + $signed({13'd0,charged_bits});

    assign run_flush_pending = zero_run_valid
        && (zero_run_count == 0
            || pending_valid
            || groups_committed == FRAME_GROUPS);
    assign serialized_advance = run_token_fire
        ? (zero_run_count == 0
            ? 9'd256 : {1'b0,zero_run_count})
        : 9'd1;

    always_comb begin : huffman_pair_rom_control
        huffman_pair_read_enable = 1'b0;
        huffman_pair_read_symbols = 0;
        if (finalizer_state == FINALIZER_FINE_LOW
                && final_selected_state != 0
                && final_selected_huffman) begin
            huffman_pair_read_enable = 1'b1;
            huffman_pair_read_symbols = final_selected_mask[7:0];
        end else if (finalizer_state == FINALIZER_READ_HIGH
                && pending_valid && pending_huffman) begin
            huffman_pair_read_enable = 1'b1;
            huffman_pair_read_symbols = final_fine_mask[15:8];
        end
    end

    assign huffman_current_pair = huffman_pair_q;

    // Once the candidate decision has completed, its two Huffman-length
    // registers are dead until the next group finalizes.  Reuse those bits
    // for the one-entry header descriptor instead of adding seven flops.
    // A candidate may replace the descriptor on the same clock its third
    // header slice starts the old build, so candidate states explicitly mask
    // the overloaded valid bit.
    assign pending_valid = final_coarse_huffman_bits[2]
        && (finalizer_state == FINALIZER_READ_HIGH
            || finalizer_state == FINALIZER_IDLE);
    assign pending_header_phase = final_coarse_huffman_bits[1:0];
    assign pending_fine = final_fine_huffman_bits[1];
    assign pending_huffman = final_fine_huffman_bits[0];
    assign final_coarse_pass = final_coarse_gain[0];

    assign header_service = pending_valid
        && finalizer_state == FINALIZER_IDLE
        && !build_active && !run_flush_pending;

    // One synchronous RAM result becomes one complete chronological token.
    // Normal pairs are bounded to seventeen bits by the finalizer. Raw pairs
    // remain the two literal bytes and are top-aligned in the same token.
    always_comb begin : selected_pair_token
        logic [2:0] rice_k0;
        logic [2:0] rice_k1;
        logic [2:0] magnitude0;
        logic serializer_fine;
        logic [3:0] serializer_band;
        logic [2:0] serializer_previous_magnitude;
        logic [4:0] length0;
        logic [4:0] length1;
        logic [16:0] token0;
        logic [16:0] token1;

        serializer_fine = build_active ? build_fine : pending_fine;
        serializer_band = build_active ? build_band : final_band;
        serializer_previous_magnitude = build_active
            ? build_previous_magnitude : 0;
        build_value0 = serializer_fine
            ? $signed(pair_ram_read_data[7:0])
            : serialized_coarse_q($signed(pair_ram_read_data[7:0]),
                pair_ram_read_data[8]);
        build_value1 = serializer_fine
            ? $signed(pair_ram_read_data[16:9])
            : serialized_coarse_q($signed(pair_ram_read_data[16:9]),
                pair_ram_read_data[17]);
        magnitude0 = rice_context_magnitude(build_value0);
        if (serializer_fine) begin
            rice_k0 = fixed_fine_rice_k(serializer_band);
            rice_k1 = rice_k0;
        end else begin
            rice_k0 = contextual_coarse_rice_k(serializer_band,
                serializer_previous_magnitude);
            rice_k1 = contextual_coarse_rice_k(serializer_band,
                magnitude0);
        end
        rice_token_address0 = rice_token_address_for(
            magnitude8(build_value0),rice_k0);
        rice_token_address1 = rice_token_address_for(
            magnitude8(build_value1),rice_k1);
        length0 = rice_token_q0[20:16];
        length1 = rice_token_q1[20:16];
        token0 = {rice_sign0_q,rice_token_q0[15:0]};
        token1 = {rice_sign1_q,rice_token_q1[15:0]};

        if (build_raw) begin
            build_pair_token = {build_value0,build_value1,1'd0};
            build_pair_token_length = 16;
        end else begin
            build_pair_token = token0 | (token1 >> length0);
            build_pair_token_length = length0 + length1;
        end
    end

    // Every selected header is serviced as three bounded tokens: prefix,
    // low-mask half, and high-mask half.  For a literal mask the halves are
    // emitted high byte then low byte to preserve the exact chronological
    // bitstream.  Raw and one-bit packets use zero-length padding tokens so
    // all selected packets retain the same three-clock header schedule.
    always_comb begin : packet_token_source
        token_valid = (build_active
                && ((build_raw)
                    ? pair_read_pending : rice_token_pending))
            || (!build_active && run_flush_pending)
            || header_service;
        token_bits = 0;
        token_count = 0;
        if (build_active
                && ((build_raw)
                    ? pair_read_pending : rice_token_pending)) begin
            token_bits = build_pair_token;
            token_count = build_pair_token_length;
        end else if (run_flush_pending) begin
            if (zero_run_count != 0 && zero_run_count < 8'd15) begin
                token_bits = 0;
                token_count = zero_run_count[4:0];
            end else begin
                token_bits = {7'b1000000,zero_run_count,2'd0};
                token_count = 5'd15;
            end
        end else if (header_service) begin
            case (pending_header_phase)
                0: begin
                    if (final_fine_bits == 1) begin
                        token_bits = 0;
                        token_count = 1;
                    end else if (final_fine_bits == 135) begin
                        token_bits = {1'b1,pending_fine,5'd0,
                            10'd0};
                        token_count = 7;
                    end else if (final_fine_bits == 147) begin
                        token_bits = {3'b101,14'd0};
                        token_count = 3;
                    end else begin
                        token_bits = {1'b1,pending_fine,
                            !pending_huffman,14'd0};
                        token_count = 3;
                    end
                end
                1: begin
                    if (final_fine_bits == 147
                            || (final_fine_bits != 1
                                && final_fine_bits != 135
                                && !pending_huffman)) begin
                        token_bits = {final_fine_mask[15:8],9'd0};
                        token_count = 8;
                        if (final_fine_bits == 147)
                            token_bits = 0;
                    end else if (final_fine_bits != 1
                            && final_fine_bits != 135) begin
                        token_bits = {huffman_low_pair[13:0],3'd0};
                        token_count = {1'b0,huffman_low_pair[17:14]};
                    end
                end
                default: begin
                    if (final_fine_bits == 147
                            || (final_fine_bits != 1
                                && final_fine_bits != 135
                                && !pending_huffman)) begin
                        token_bits = {final_fine_mask[7:0],9'd0};
                        token_count = 8;
                        if (final_fine_bits == 147)
                            token_bits = 0;
                    end else if (final_fine_bits != 1
                            && final_fine_bits != 135) begin
                        token_bits = {huffman_current_pair[13:0],3'd0};
                        token_count = {1'b0,huffman_current_pair[17:14]};
                    end
                end
            endcase
        end
    end

    always_comb begin : packet_token_handshake
        token_fire = token_valid && token_ready;
        start_build = header_service && token_fire
            && pending_header_phase == 2;
        build_pair_fire = build_active && pair_read_pending
            && token_ready;
        if (build_active && !build_raw)
            build_pair_fire = rice_token_pending && token_ready;
        header_pair_prefetch_start = header_service && token_fire
            && pending_header_phase == 0
            && final_fine_bits != 1 && final_fine_bits != 135
            && final_fine_bits != 147;
        header_rice_issue = header_service && token_fire
            && pending_header_phase == 1 && pair_read_pending
            && final_fine_bits != 1 && final_fine_bits != 135
            && final_fine_bits != 147;
        rice_token_read_enable = header_rice_issue
            || (build_active && pair_read_pending
                && !build_raw
                && (!rice_token_pending || build_pair_fire));
        pair_advance = header_rice_issue
            || (build_active
                && ((build_raw)
                    ? build_pair_fire : rice_token_read_enable));
        run_token_fire = !build_active && run_flush_pending
            && token_fire;
        build_complete_now = build_pair_fire
            && build_pairs_consumed == 7;
        packet_complete = (start_build && final_fine_bits == 1)
            || build_complete_now;
    end

    always_comb begin : pair_ram_control
        pair_ram_write_address = {fill_context,fill_pair};
        pair_ram_write_data = {pair_correction[1],pair_q1,
            pair_correction[0],pair_q0};
        pair_ram_read_enable = 1'b0;
        pair_ram_read_address = 0;
        if (header_pair_prefetch_start) begin
            pair_ram_read_enable = 1'b1;
            pair_ram_read_address = {~fill_context,3'd0};
        end else if (header_rice_issue) begin
            pair_ram_read_enable = 1'b1;
            pair_ram_read_address = {~fill_context,3'd1};
        end else if (start_build
                && (final_fine_bits == 135 || final_fine_bits == 147)) begin
            pair_ram_read_enable = 1'b1;
            pair_ram_read_address = {~fill_context,3'd0};
        end else if (pair_advance && build_pairs_issued < 8) begin
            pair_ram_read_enable = 1'b1;
            pair_ram_read_address = {
                build_context,build_pairs_issued[2:0]};
        end
    end

    // A power-of-two product aligns the token within a sixteen-bit window.
    wire [15:0] append_scale = 16'b1 << ~output_count_after[3:0];
    wire [47:0] append_product;
`ifdef SYNTHESIS
    EFX_DSP48 #(.MODE("NORMAL"), .SIGNED(1'b0), .M_SEL("P"),
        .N_SEL("C"), .W_SEL("X"), .W_REG(1'b0)) append_multiply (
        .A({2'd0, token_bits}), .B({2'd0, append_scale}), .C(18'd0),
        .CASCIN(48'd0), .OP(2'd0), .SHIFT_ENA(1'b0),
        .CLK(clk), .CE(1'b1), .RST(1'b0), .O(append_product));
`else
    assign append_product = 48'(token_bits) * 48'(append_scale);
`endif
    logic [OUTPUT_RESERVOIR_BITS-1:0] append_bits;
    always_comb begin
        case (output_count_after[5:4])
            2'd0: append_bits = {append_product[31:0],14'd0};
            2'd1: append_bits = {16'd0,append_product[31:2]};
            2'd2: append_bits = {32'd0,append_product[31:18]};
            default: append_bits = '0;
        endcase
    end

    // Packet tokens and frame words share one chronological reservoir. A word
    // is removed before each append. Forty-six bits cover the exact repeated
    // packet arrival bound for legal Rice pairs ({0,2..17} bits), a bounded
    // header slice, and at most 135 total packet bits.
    always_comb begin : fused_output
        // Reserve the final physical word until packet 64,000 has completed.
        // An exactly full semantic stream may otherwise reach sixteen queued
        // bits before the final pair-service clocks have retired.
        out_valid = (output_count >= 16
                && output_words != FRAME_WORDS_MINUS_ONE)
            || padding_mode;
        out_data = output_stage[OUTPUT_RESERVOIR_BITS-1 -: 16];
        out_sof = out_valid && output_words == 0;
        output_fire = out_valid && out_ready;

        output_stage_after = output_stage;
        output_count_after = output_count;
        if (output_fire) begin
            if (output_count >= 16) begin
                output_stage_after = output_stage << 16;
                output_count_after = output_count - 16;
            end else begin
                output_stage_after = 0;
                output_count_after = 0;
            end
        end

        token_ready = !padding_mode
            && {1'b0,output_count_after} + {2'd0,token_count}
                <= {1'b0,OUTPUT_RESERVOIR_COUNT_MAX};
        output_stage_next = output_stage_after;
        output_count_next = output_count_after;
        if (token_fire && token_count != 0) begin
            output_stage_next = output_stage_after
                | append_bits;
            output_count_next = output_count_after + token_count;
        end
    end

    // All four counters retain their original update and reset edges.
    LwcEncoderAccumulator #(.WIDTH(16)) committed_accumulator (
        .clk, .reset(rst || clear_frame), .enable(selection_commit),
        .increment(18'sd1), .value(groups_committed));
    LwcEncoderAccumulator #(.WIDTH(16)) serialized_accumulator (
        .clk, .reset(rst || clear_frame), .enable(packet_complete || run_token_fire),
        .increment($signed({9'd0, serialized_advance})), .value(groups_serialized));
    LwcEncoderAccumulator #(.WIDTH(21)) packet_accumulator (
        .clk, .reset(rst || clear_frame), .enable(selection_commit),
        .increment($signed({10'd0, charged_bits})), .value(packet_bits));
    LwcEncoderAccumulator #(.WIDTH(16)) output_accumulator (
        .clk, .reset(rst || clear_frame || (output_fire && output_words == FRAME_WORDS_MINUS_ONE)),
        .enable(output_fire), .increment(18'sd1), .value(output_words));

    always_ff @(posedge clk) begin : main_state
        if (rst || clear_frame) begin
            fill_context <= 1'b0;
            fill_active <= 1'b0;
            fill_pair <= 0;
            fill_band <= 0;
            fill_fine_mask <= 0;
            fill_coarse_mask <= 0;
            fill_fine_huffman_bits <= 0;
            fill_coarse_huffman_bits <= 0;
            fill_direct_energy <= 0;
            fill_incremental_energy <= 0;
            fill_fine_prefix <= 0;
            fill_coarse_prefix <= 0;
            fill_coarse_previous_magnitude <= 0;
            fill_fine_pair_overflow <= 1'b0;
            fill_coarse_pair_overflow <= 1'b0;
            finalizer_state <= FINALIZER_IDLE;
            final_fine_huffman_bits <= 0;
            final_coarse_huffman_bits <= 0;
            final_coarse_gain <= 0;
            final_nominal_bits <= 0;
            final_energy_shift_profile <= 0;
            build_active <= 1'b0;
            build_context <= 1'b0;
            build_fine <= 1'b0;
            build_band <= 0;
            build_raw <= 1'b0;
`ifndef SYNTHESIS
            build_length <= 0;
`endif
            build_pairs_issued <= 0;
            build_pairs_consumed <= 0;
            build_previous_magnitude <= 0;
            pair_read_pending <= 1'b0;
            rice_token_pending <= 1'b0;
            output_stage <= 0;
            output_count <= 0;
            padding_mode <= 1'b0;
            frame_done <= 1'b0;
            packets_done <= 1'b0;
            rate_debt <= 0;
            extra_room <= INITIAL_EXTRA_ROOM;
            zero_run_valid <= 1'b0;
            zero_run_count <= 0;
            protocol_error <= 1'b0;
        end else begin
            packets_done <= 1'b0;
            frame_done <= 1'b0;
            output_stage <= output_stage_next;
            output_count <= output_count_next;

            if (pair_ram_read_enable)
                pair_read_pending <= 1'b1;
            else if (pair_advance)
                pair_read_pending <= 1'b0;

            if (rice_token_read_enable)
                rice_token_pending <= 1'b1;
            else if (build_pair_fire)
                rice_token_pending <= 1'b0;

            if (energy_pending)
                fill_direct_energy <= complete_direct_energy;

            if (pair_fire) begin
                fill_fine_mask <= complete_fine_mask[15:2];
                fill_coarse_mask <= complete_coarse_mask[15:2];
                fill_fine_huffman_bits <= complete_fine_huffman_bits;
                fill_coarse_huffman_bits <= complete_coarse_huffman_bits;
                fill_incremental_energy <= complete_incremental_energy;
                fill_fine_prefix <= pair_fine_prefix_next;
                fill_coarse_prefix <= pair_coarse_prefix_next;
                fill_coarse_previous_magnitude
                    <= (|pair_coarse1[7:3] ? 3'd7 : pair_coarse1[2:0]);
                fill_fine_pair_overflow
                    <= complete_fine_pair_overflow;
                fill_coarse_pair_overflow
                    <= complete_coarse_pair_overflow;

                if (!fill_active) begin
                    fill_active <= 1'b1;
                    fill_pair <= 1;
                    fill_band <= pair_band;
                    if (!pair_group_start)
                        protocol_error <= 1'b1;
                end else begin
                    fill_pair <= fill_pair + 1'b1;
                    if (pair_group_start || pair_band != fill_band)
                        protocol_error <= 1'b1;
                end

                if (pair_group_last != (fill_pair == 7))
                    protocol_error <= 1'b1;

                if (group_finalize) begin
                    final_fine_mask <= complete_fine_mask;
                    final_coarse_mask <= complete_coarse_mask;
                    final_fine_huffman_bits
                        <= complete_fine_huffman_bits;
                    final_coarse_huffman_bits
                        <= complete_coarse_huffman_bits;
                    final_fine_bits <= complete_fine_bits;
                    final_coarse_bits <= complete_coarse_bits;
                    final_incremental_gain <= complete_incremental_gain;
                    final_band <= fill_band;
                    final_nominal_bits <= nominal_group_bits(fill_band);
                    final_energy_shift_profile
                        <= energy_shift_profile(fill_band);
                    finalizer_state <= FINALIZER_COARSE;
                    fill_active <= 1'b0;
                    fill_pair <= 0;
                    if (groups_committed >= FRAME_GROUPS)
                        protocol_error <= 1'b1;
                end
            end

            if (pair_ram_read_enable && !start_build) begin
                build_pairs_issued <= build_pairs_issued + 1'b1;
            end

            if (finalizer_state == FINALIZER_COARSE) begin
                // Retain the coarse pass bit for the following incremental
                // decision. The delayed energy lookup is consumed here.
                final_coarse_gain <= {10'd0,
                    decision_gain_pass && final_coarse_fits};
                finalizer_state <= FINALIZER_FINE_LOW;
            end else if (finalizer_state == FINALIZER_FINE_LOW) begin
                // The low-byte ROM request is issued from the combinational
                // selection on this clock.  Preserve the selected descriptor
                // in otherwise-idle final-candidate registers while its
                // header waits behind an earlier packet or zero run.
                if (final_selected_state != 0) begin
                    if (pending_valid)
                        protocol_error <= 1'b1;
                    final_coarse_huffman_bits <= 5'b0_0100;
                    final_fine_huffman_bits <= {3'd0,
                        final_selected_state == 2,
                        final_selected_huffman};
                    final_fine_mask <= final_selected_mask;
                    final_fine_bits <= final_selected_bits;
                    fill_context <= ~fill_context;
                end else begin
                    final_coarse_huffman_bits <= 0;
                    final_fine_huffman_bits <= 0;
                end
                finalizer_state <= FINALIZER_READ_HIGH;
            end else if (finalizer_state == FINALIZER_READ_HIGH) begin
                if (pending_valid && pending_huffman)
                    huffman_low_pair <= huffman_current_pair;
                finalizer_state <= FINALIZER_IDLE;
            end

            if (header_service && token_fire) begin
                if (pending_header_phase == 2) begin
                    if (!group_finalize)
                        final_coarse_huffman_bits <= 0;
                end else begin
                    final_coarse_huffman_bits[1:0]
                        <= pending_header_phase + 1'b1;
                end
            end

            if (build_pair_fire) begin
                build_pairs_consumed <= build_pairs_consumed + 1'b1;
                if (!build_raw
                        && build_pair_token_length > MAX_PAIR_TOKEN_BITS)
                    protocol_error <= 1'b1;
                if (!build_complete_now && build_service_clocks == 9)
                    protocol_error <= 1'b1;
            end

            if (pair_advance)
                build_previous_magnitude
                    <= rice_context_magnitude(build_value1);

            if (build_complete_now) begin
                build_active <= 1'b0;
            end

            if (packet_complete || run_token_fire) begin
                if ((run_token_fire && groups_committed == FRAME_GROUPS
                            && !pending_valid)
                        || (packet_complete
                            && groups_serialized == FRAME_GROUPS-1'b1)) begin
                    packets_done <= 1'b1;
                    padding_mode <= 1'b1;
                end else if (groups_serialized >= FRAME_GROUPS) begin
                    protocol_error <= 1'b1;
                end
            end

            if (run_token_fire) begin
                zero_run_valid <= 1'b0;
                zero_run_count <= 0;
            end

            if (start_build) begin
                build_active <= final_fine_bits != 1;
                build_context <= ~fill_context;
                build_fine <= pending_fine;
                build_band <= final_band;
                build_raw <= final_fine_bits == 135 || final_fine_bits == 147;
`ifndef SYNTHESIS
                build_length <= final_fine_bits;
`endif
                build_pairs_issued <= final_fine_bits == 1 ? 0
                    : ((final_fine_bits == 135
                        || final_fine_bits == 147) ? 1 : 2);
                build_pairs_consumed <= 0;
                if (pending_fine || final_fine_bits == 135
                        || final_fine_bits == 147)
                    build_previous_magnitude <= 0;
            end

            if (zero_absorb) begin
                zero_run_valid <= 1'b1;
                zero_run_count <= zero_run_valid
                    ? zero_run_count + 1'b1 : 8'd1;
            end

            if (selection_commit) begin
                extra_room <= extra_room - room_charge;
                rate_debt <= final_post_debt;
            end

            if (output_fire) begin
                if (output_words == FRAME_WORDS_MINUS_ONE) begin
                    if (!padding_mode)
                        protocol_error <= 1'b1;
                    padding_mode <= 1'b0;
                    output_stage <= 0;
                    output_count <= 0;
                    frame_done <= 1'b1;
                end
            end
        end
    end

    // The finalizer serializes the low and high mask bytes across two clocks
    // so one physical RAM10 read port services the complete table.
    always_ff @(posedge clk) begin : huffman_pair_rom_read
        if (!rst && !clear_frame && huffman_pair_read_enable)
            huffman_pair_q <= huffman_pair_rom[
                huffman_pair_read_symbols];
    end

    // Both coefficient suffixes are fetched together.  A held response is
    // replaced only when the current token transfers, so arbitrary output
    // stalls cannot overwrite the one-entry response slot.
    always_ff @(posedge clk) begin : rice_token_rom_read
        if (!rst && !clear_frame && rice_token_read_enable) begin
            rice_token_q0 <= rice_token_rom[rice_token_address0];
            rice_token_q1 <= rice_token_rom[rice_token_address1];
            rice_sign0_q <= build_value0[7];
            rice_sign1_q <= build_value1[7];
        end
    end

    // The two ports are independent: the collector writes one pair while the
    // serializer issues a synchronous read from the other context.
    always_ff @(posedge clk) begin : pair_context_ram
        if (!rst && !clear_frame && pair_fire)
            pair_ram[pair_ram_write_address] <= pair_ram_write_data;
        if (pair_ram_read_enable)
            pair_ram_read_data <= pair_ram[pair_ram_read_address];
    end

`ifndef SYNTHESIS
    // Independent live-credit oracle for the saved prediction. No decision
    // may consume stale credit after a reset, a gap, or output backpressure.
    logic [15:0] reference_cutoff_available;
    assign reference_cutoff_available = finalizer_state == FINALIZER_COARSE
        ? (burst_available >= 16'd45_054
            ? 16'd45_056 : burst_available + 16'd2)
        : burst_available;
    // The maximum backlog of a two-bit/clock ideal server equals the maximum
    // excess over that rate over every contiguous output interval. Keep this
    // monitor across clear_frame: clearing frame credit must not grant a burst.
    // The 5,696-byte allowance includes queued descriptors and reservoir bits.
    logic [16:0] burst_semantic_excess;
    always_ff @(posedge clk) begin
        if (rst) begin
            burst_semantic_excess <= 0;
        end else if (BURST_LIMIT_ENABLE) begin
            assert (burst_semantic_excess <= 17'd45_568);
            if (output_fire && !out_padding)
                burst_semantic_excess <= burst_semantic_excess + 17'd14;
            else if (burst_semantic_excess > 17'd2)
                burst_semantic_excess <= burst_semantic_excess - 17'd2;
            else
                burst_semantic_excess <= 0;
        end
    end

    always_ff @(posedge clk) begin
        if (!rst && !clear_frame) begin
            if (BURST_LIMIT_ENABLE) begin
                assert (burst_credit <= 16'd45_056);
                if (group_finalize)
                    assert (finalizer_state == FINALIZER_IDLE);
                if (finalizer_state == FINALIZER_COARSE || selection_commit)
                    assert (burst_cutoff ==
                        4'((16'd45_056 - reference_cutoff_available) >> 13));
                if (selection_commit)
                    assert ({8'd0,final_selected_bits} <= burst_available);
            end
            assert (extra_room <= 20'd1_024_000);
            assert (groups_serialized <= groups_committed);
            assert (groups_committed - groups_serialized <= 16'd257);
            assert (zero_run_valid || zero_run_count == 0);
            assert (output_count <= OUTPUT_RESERVOIR_COUNT_MAX);
            if (build_active) begin
                assert (build_length >= 2 && build_length <= 147);
                assert (build_service_clocks <= 9);
            end
            if (token_fire) begin
                assert (token_count <= 17);
                assert ((token_bits & (17'h1ffff >> token_count)) == 0);
            end
            if (rice_token_read_enable) begin
                assert (rice_token_address0 <= 10'd644);
                assert (rice_token_address1 <= 10'd644);
                assert (!rice_token_pending || build_pair_fire);
            end
            if (rice_token_pending && !build_pair_fire)
                assert (!rice_token_read_enable);
            if (build_complete_now)
                assert (build_service_clocks == 8);
            if (start_build)
                assert (!(!pending_fine
                    && final_fine_bits == 147));
            if (finalizer_state == FINALIZER_READ_HIGH) begin
                if (pending_valid && pending_huffman) begin
                    assert (huffman_pair_read_enable);
                    assert (huffman_pair_read_symbols
                        == final_fine_mask[15:8]);
                    assert (huffman_current_pair
                        == huffman_pair_entry(final_fine_mask[7:0]));
                end
            end
            if (header_service && pending_huffman) begin
                assert (huffman_low_pair
                    == huffman_pair_entry(final_fine_mask[7:0]));
                assert (huffman_current_pair
                    == huffman_pair_entry(final_fine_mask[15:8]));
            end
            if (start_build && pending_huffman)
                assert (header_service);
            if (packets_done) begin
                assert (groups_committed == FRAME_GROUPS);
                assert (groups_serialized == FRAME_GROUPS);
            end
            if (output_fire && output_words == FRAME_WORDS_MINUS_ONE)
                assert (padding_mode);
        end
    end
`endif
endmodule

`default_nettype wire
