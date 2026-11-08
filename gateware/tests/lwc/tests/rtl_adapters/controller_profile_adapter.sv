`default_nettype none

// Exposes the production package's causal-controller tables and signed debt
// transition function through Marlin-sized scalar ports.
module controller_profile_adapter (
    input  wire [3:0]               band,
    input  wire signed [21:0]       pre_debt,
    input  wire [3:0]               nibble,
    output wire [7:0]               nominal_bits,
    output wire [1:0]               coarse_energy_shift,
    output wire [1:0]               incremental_energy_shift,
    output wire [3:0]               coarse_cutoff,
    output wire [3:0]               incremental_cutoff,
    output wire [3:0]               huffman_length,
    output wire [6:0]               huffman_code
);
    wire [7:0] cutoffs;

    assign nominal_bits = codec_profile_pkg::nominal_group_bits(band);
    assign {incremental_energy_shift,coarse_energy_shift}
        = codec_profile_pkg::energy_shift_profile(band);
    assign cutoffs = codec_profile_pkg::rate_cutoffs(pre_debt);
    assign {incremental_cutoff,coarse_cutoff} = cutoffs;
    assign huffman_length = codec_profile_pkg::nibble_huffman_length(nibble);
    assign huffman_code = codec_profile_pkg::nibble_huffman_code(nibble);
endmodule

`default_nettype wire
