// One-RAM reciprocal-square-root service for compact LFTag pose scale.
//
// The Q32 radicand is normalized two bits per clock into [2^34, 2^36).
// Two 256-entry ROM banks cover both leading-bit parities.  The ROM returns
// lambda with eight guard bits; serial exponent removal produces the
// signed-Q10 multiplier used with common-scale Q16 homography coefficients.
// Small norms return the same signed18 mantissa plus a binary left shift;
// the pose leaf applies that exponent to products instead of overflowing its
// multiplier operand. Zero and bit36 are invalid; six signed18 column squares
// divided by two can never reach 2^36.
module LfTagReciprocalSqrt512 (
    input  logic         clk,
    input  logic         rst,
    input  logic         in_valid,
    output logic         in_ready,
    input  logic [36:0]  in_radicand,
    output logic         out_valid,
    input  logic         out_ready,
    output logic [18:0]  out_lambda_q10,
    output logic [3:0]   out_lambda_left_shift
);
    typedef enum logic [2:0] {
        S_IDLE,
        S_NORMALIZE,
        S_ROM_WAIT_0,
        S_ROM_WAIT_1,
        S_SCALE,
        S_OUTPUT
    } state_t;

    state_t state;
    logic [36:0] radicand_work;
    logic signed [5:0] right_shift_count;
    logic [8:0] rom_address;
    logic [17:0] rom_data;

    (* syn_romstyle = "block_ram" *) logic [17:0] reciprocal_sqrt_rom [0:511];
    initial begin
`include "lftag_rsqrt_512x18_init.svh"
    end

    assign in_ready = state == S_IDLE;
    assign out_valid = state == S_OUTPUT;

    always_ff @(posedge clk) begin : reciprocal_sqrt_rom_read
        rom_data <= reciprocal_sqrt_rom[rom_address];
    end

    always_ff @(posedge clk) begin : reciprocal_sqrt_sequencer
        if (rst) begin
            state <= S_IDLE;
            radicand_work <= 37'b0;
            right_shift_count <= 6'sb0;
            rom_address <= 9'b0;
            out_lambda_q10 <= 19'b0;
            out_lambda_left_shift <= 4'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (in_valid) begin
                        radicand_work <= in_radicand;
                        right_shift_count <= 6'sd8;
                        out_lambda_q10 <= 19'b0;
                        out_lambda_left_shift <= 4'b0;
                        state <= S_NORMALIZE;
                    end
                end

                S_NORMALIZE: begin
                    if ((radicand_work == 0) || radicand_work[36]) begin
                        out_lambda_q10 <= 19'b0;
                        state <= S_OUTPUT;
                    end else if (|radicand_work[35:34]) begin
                        rom_address <= {radicand_work[35], radicand_work[35]
                            ? radicand_work[34:27] : radicand_work[33:26]};
                        state <= S_ROM_WAIT_0;
                    end else begin
                        radicand_work <= radicand_work << 2;
                        right_shift_count <= right_shift_count - 1'b1;
                    end
                end

                S_ROM_WAIT_0: state <= S_ROM_WAIT_1;

                S_ROM_WAIT_1: begin
                    // The output is unobservable until out_valid, so use it
                    // as the serial scale work register instead of keeping a
                    // duplicate 18-bit lambda register.
                    out_lambda_q10 <= {1'b0, rom_data};
                    state <= S_SCALE;
                end

                S_SCALE: begin
                    if (right_shift_count <= 0) begin
                        out_lambda_left_shift <= 4'(-right_shift_count);
                        state <= S_OUTPUT;
                    end else begin
                        out_lambda_q10 <= out_lambda_q10 >> 1;
                        right_shift_count <= right_shift_count - 1'b1;
                    end
                end

                S_OUTPUT: begin
                    if (out_valid && out_ready)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
