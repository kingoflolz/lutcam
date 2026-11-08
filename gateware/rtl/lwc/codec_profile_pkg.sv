package codec_profile_pkg;
    localparam logic signed [22:0] RATE_COARSE_CUTOFF_BASE = 23'sd5;
    localparam logic signed [22:0] RATE_INCREMENTAL_CUTOFF_BASE = 23'sd6;
    localparam logic [3:0] RATE_INCREMENTAL_CUTOFF_OFFSET
        = RATE_INCREMENTAL_CUTOFF_BASE[3:0]
        - RATE_COARSE_CUTOFF_BASE[3:0];
    localparam int RATE_DEBT_SHIFT = 14;

    function automatic logic [3:0] fine_shift(input logic [3:0] band);
        case (band)
            0,1: fine_shift = 1;
            2,3,4: fine_shift = 2;
            5,6,7: fine_shift = 3;
            default: fine_shift = 4;
        endcase
    endfunction

    function automatic logic [3:0] coarse_shift(input logic [3:0] band);
        case (band)
            0,1: coarse_shift = 2;
            2,3,4: coarse_shift = 3;
            5,6,7: coarse_shift = 4;
            default: coarse_shift = 5;
        endcase
    endfunction

    function automatic logic [4:0] fine_bias(input logic [3:0] band);
        case (band)
            0,1,2,3: fine_bias = 0;
            4: fine_bias = 1;
            5,7: fine_bias = 3;
            6: fine_bias = 2;
            default: fine_bias = 6;
        endcase
    endfunction

    function automatic logic [4:0] coarse_bias(input logic [3:0] band);
        case (band)
            0,1: coarse_bias = 1;
            2,3,4: coarse_bias = 3;
            5,6,7: coarse_bias = 7;
            10: coarse_bias = 17;
            default: coarse_bias = 15;
        endcase
    endfunction

    // One-sided signed-domain adjustment used by the physical fine lanes.
    // Encoding the already-resolved adjustment keeps quantization to one
    // 15-bit adder even though the magnitude-domain bias varies by band.
    function automatic logic [4:0] fine_signed_adjustment(
        input logic [3:0] band,
        input logic negative
    );
        case (band)
            0,1: fine_signed_adjustment = negative ? 1 : 0;
            2,3: fine_signed_adjustment = negative ? 3 : 0;
            4: fine_signed_adjustment = negative ? 2 : 1;
            5,7: fine_signed_adjustment = negative ? 4 : 3;
            6: fine_signed_adjustment = negative ? 5 : 2;
            default: fine_signed_adjustment = negative ? 9 : 6;
        endcase
    endfunction

    function automatic logic [2:0] fine_rice_k(input logic [3:0] band);
        case (band)
            0,1: fine_rice_k = 4;
            2: fine_rice_k = 2;
            3,4: fine_rice_k = 1;
            default: fine_rice_k = 0;
        endcase
    endfunction

    function automatic logic [7:0] nominal_group_bits(
        input logic [3:0] band
    );
        case (band)
            0: nominal_group_bits = 139;
            1: nominal_group_bits = 135;
            2: nominal_group_bits = 102;
            3: nominal_group_bits = 58;
            4: nominal_group_bits = 59;
            5: nominal_group_bits = 43;
            6: nominal_group_bits = 27;
            7: nominal_group_bits = 30;
            8: nominal_group_bits = 21;
            9: nominal_group_bits = 10;
            10: nominal_group_bits = 13;
            default: nominal_group_bits = 5;
        endcase
    endfunction

    // Packed as {incremental,coarse} so synthesis shares the band decode.
    function automatic logic [3:0] energy_shift_profile(
        input logic [3:0] band
    );
        case (band)
            0: energy_shift_profile = 4'hc;
            1,9: energy_shift_profile = 4'h6;
            2: energy_shift_profile = 4'ha;
            3,4,5: energy_shift_profile = 4'h4;
            6,7,8,10: energy_shift_profile = 4'h5;
            default: energy_shift_profile = 0;
        endcase
    endfunction

    // Returns {incremental,coarse}. Both causal thresholds use the same
    // signed debt octave but have independently specified transition bases.
    function automatic logic [7:0] rate_cutoffs(
        input logic signed [21:0] pre_debt
    );
        logic signed [21:0] debt_octaves;
        logic signed [22:0] coarse_wide;
        logic [3:0] coarse_cutoff;
        logic [3:0] incremental_cutoff;
        begin
            debt_octaves = pre_debt >>> RATE_DEBT_SHIFT;
            coarse_wide = RATE_COARSE_CUTOFF_BASE
                + $signed({debt_octaves[21],debt_octaves});
            if (coarse_wide < 0) begin
                coarse_cutoff = 0;
                incremental_cutoff = 0;
            end else if (coarse_wide >= 8) begin
                coarse_cutoff = 8;
                incremental_cutoff = 8;
            end else begin
                coarse_cutoff = coarse_wide[3:0];
                incremental_cutoff = coarse_wide[3:0]
                    + RATE_INCREMENTAL_CUTOFF_OFFSET;
            end
            rate_cutoffs = {incremental_cutoff,coarse_cutoff};
        end
    endfunction

    function automatic logic [3:0] nibble_huffman_length(
        input logic [3:0] nibble
    );
        case (nibble)
            0: nibble_huffman_length = 1;
            15: nibble_huffman_length = 3;
            1,8: nibble_huffman_length = 4;
            3,4,7,12,14: nibble_huffman_length = 5;
            5,10: nibble_huffman_length = 7;
            default: nibble_huffman_length = 6;
        endcase
    endfunction

    // Canonical code is right aligned; code_length says how many low bits to
    // append, MSB first.
    function automatic logic [6:0] nibble_huffman_code(
        input logic [3:0] nibble
    );
        case (nibble)
            0: nibble_huffman_code = 0;
            1: nibble_huffman_code = 10;
            2: nibble_huffman_code = 58;
            3: nibble_huffman_code = 24;
            4: nibble_huffman_code = 25;
            5: nibble_huffman_code = 126;
            6: nibble_huffman_code = 59;
            7: nibble_huffman_code = 26;
            8: nibble_huffman_code = 11;
            9: nibble_huffman_code = 60;
            10: nibble_huffman_code = 127;
            11: nibble_huffman_code = 61;
            12: nibble_huffman_code = 27;
            13: nibble_huffman_code = 62;
            14: nibble_huffman_code = 28;
            default: nibble_huffman_code = 4;
        endcase
    endfunction

    function automatic logic [5:0] mask_huffman_length(
        input logic [15:0] mask
    );
        mask_huffman_length = 6'(nibble_huffman_length(mask[3:0]))
            + 6'(nibble_huffman_length(mask[7:4]))
            + 6'(nibble_huffman_length(mask[11:8]))
            + 6'(nibble_huffman_length(mask[15:12]));
    endfunction

    // Bit eight reports true signed-eight saturation. Values exactly at -128
    // and +127 are representable and therefore do not set the clip bit.
    function automatic logic [8:0] quantize_signed8_with_clip(
        input logic signed [13:0] value,
        input logic [3:0] shift,
        input logic [4:0] signed_adjustment
    );
        logic signed [14:0] adjusted;
        logic signed [14:0] shifted;
        begin
            // Magnitude-plus-bias followed by sign restoration. The caller
            // resolves the sign-dependent constant so this remains one adder.
            adjusted = $signed({value[13], value})
                + $signed({10'd0,signed_adjustment});
            shifted = adjusted >>> shift;
            if (shifted < -15'sd128)
                quantize_signed8_with_clip = {1'b1,8'h80};
            else if (shifted > 15'sd127)
                quantize_signed8_with_clip = {1'b1,8'h7f};
            else
                quantize_signed8_with_clip = {1'b0,shifted[7:0]};
        end
    endfunction

    function automatic logic [7:0] magnitude8(
        input logic signed [7:0] value
    );
        magnitude8 = value[7] ? ((~value) + 8'd1) : value;
    endfunction

    function automatic logic [9:0] rice_literal_bits(
        input logic signed [7:0] value,
        input logic [2:0] k
    );
        logic [7:0] magnitude;
        logic [7:0] remainder_value;
        begin
            magnitude = magnitude8(value);
            if (magnitude == 0)
                rice_literal_bits = 0;
            else begin
                remainder_value = magnitude - 1'b1;
                rice_literal_bits = 10'(2 + k)
                    + 10'(remainder_value >> k);
            end
        end
    endfunction

endpackage
