`default_nettype none

// Cheap raw-event fast path.  The measured 5/3 transform cadence has a
// five-color conflict graph, so only five scalar lanes are required even
// though events are exposed as twelve named bands.  Entropy lengths and the
// R-D proxy are intentionally not computed here; the causal group encoder
// folds them while accepting the queued coefficient pairs.
module coefficient_quantizer (
    input  wire                              clk,
    input  wire                              rst,
    input  wire                              clear_frame,
    input  wire [11:0]                       in_valid,
    input  wire signed [11:0][13:0]          in_value,

    output logic [11:0]                      out_valid,
    output logic signed [11:0][7:0]          out_q,
    output logic [11:0]                      out_correction,
    output logic [11:0]                      out_residual_class,
    output logic                             protocol_error
);
    import codec_profile_pkg::*;

    // Exact raw-event coloring over the complete 1280x800 cadence:
    //   lane 0: bands 6,7,9       lane 3: bands 0,2,3,4
    //   lane 1: bands 8,10        lane 4: bands 1,5
    //   lane 2: band 11
    logic [3:0] lane_band [0:4];
    logic signed [13:0] lane_source [0:4];
    logic [8:0] lane_quantized [0:4];
    wire signed [12:0] lane_rounded [0:4];
    logic [4:0] lane_clipped;
    logic signed [7:0] lane_fine [0:4];
    logic [4:0] lane_correction;
    logic [4:0] lane_residual_class;
    logic [4:0] lane_conflict;

    // Both metadata decisions reduce to short low-bit intervals once the
    // common power-of-two quantizer step is removed.  These exact predicates
    // avoid a second quantizer and reconstructed-coefficient subtractor in
    // every physical lane.
    function automatic logic periodic_correction(
        input logic [3:0] band,
        input logic negative,
        input logic [4:0] residue
    );
        begin
            case (band)
                0,1: periodic_correction = negative
                    ? residue[1:0] == 2'd1 : residue[1:0] == 2'd3;
                2,3: periodic_correction = negative
                    ? residue[2:0] >= 3'd1 && residue[2:0] <= 3'd3
                    : residue[2:0] >= 3'd5;
                4: periodic_correction = negative
                    ? residue[2:0] >= 3'd2 && residue[2:0] <= 3'd3
                    : residue[2:0] >= 3'd5 && residue[2:0] <= 3'd6;
                5,7: periodic_correction = negative
                    ? residue[3:0] >= 4'd4 && residue[3:0] <= 4'd7
                    : residue[3:0] >= 4'd9 && residue[3:0] <= 4'd12;
                6: periodic_correction = negative
                    ? residue[3:0] >= 4'd3 && residue[3:0] <= 4'd7
                    : residue[3:0] >= 4'd9 && residue[3:0] <= 4'd13;
                10: periodic_correction = negative
                    ? residue >= 5'd7 && residue <= 5'd17
                    : residue >= 5'd15 && residue <= 5'd25;
                default: periodic_correction = negative
                    ? residue >= 5'd7 && residue <= 5'd15
                    : residue >= 5'd17 && residue <= 5'd25;
            endcase
        end
    endfunction

    function automatic logic correction_saturation(
        input logic [3:0] band,
        input logic signed [13:0] source
    );
        begin
            case (band)
                0,1: correction_saturation = source >= 14'sd255
                    || source <= -14'sd259;
                2,3,4: correction_saturation = source >= 14'sd509
                    || source <= -14'sd517;
                5,6,7: correction_saturation = source >= 14'sd1017
                    || source <= -14'sd1033;
                10: correction_saturation = source >= 14'sd2031
                    || source <= -14'sd2063;
                default: correction_saturation = source >= 14'sd2033
                    || source <= -14'sd2065;
            endcase
        end
    endfunction

    function automatic logic periodic_residual_class(
        input logic [3:0] band,
        input logic negative,
        input logic [4:0] residue
    );
        begin
            case (band)
                0,1: periodic_residual_class = residue[1:0] != 2'd2;
                2,3: periodic_residual_class = residue[2:0] != 3'd4;
                4: periodic_residual_class = negative
                    ? residue[2:0] != 3'd1 && residue[2:0] != 3'd4
                    : residue[2:0] != 3'd4 && residue[2:0] != 3'd7;
                5,7: periodic_residual_class = negative
                    ? residue[3:0] == 0
                        || (residue[3:0] >= 4'd4 && residue[3:0] <= 4'd6)
                        || residue[3:0] >= 4'd10
                    : residue[3:0] <= 4'd6
                        || (residue[3:0] >= 4'd10 && residue[3:0] <= 4'd12);
                6: periodic_residual_class = negative
                    ? residue[3:0] == 0
                        || (residue[3:0] >= 4'd3 && residue[3:0] <= 4'd6)
                        || residue[3:0] >= 4'd10
                    : residue[3:0] <= 4'd6
                        || (residue[3:0] >= 4'd10 && residue[3:0] <= 4'd13);
                default: periodic_residual_class = negative
                    ? residue == 0
                        || (residue >= 5'd7 && residue <= 5'd12)
                        || residue >= 5'd20
                    : residue <= 5'd12
                        || (residue >= 5'd20 && residue <= 5'd25);
            endcase
        end
    endfunction

    always_comb begin : lane_select
        integer lane;
        lane_conflict = '0;
        for (lane = 0; lane < 5; lane = lane + 1) begin
            lane_band[lane] = '0;
            lane_source[lane] = '0;
        end

        lane_conflict[0] = (in_valid[6] + in_valid[7] + in_valid[9]) > 1;
        if (in_valid[6]) begin
            lane_band[0] = 4'd6;
            lane_source[0] = in_value[6];
        end else if (in_valid[7]) begin
            lane_band[0] = 4'd7;
            lane_source[0] = in_value[7];
        end else begin
            lane_band[0] = 4'd9;
            lane_source[0] = in_value[9];
        end

        lane_conflict[1] = in_valid[8] && in_valid[10];
        if (in_valid[8]) begin
            lane_band[1] = 4'd8;
            lane_source[1] = in_value[8];
        end else begin
            lane_band[1] = 4'd10;
            lane_source[1] = in_value[10];
        end

        lane_band[2] = 4'd11;
        lane_source[2] = in_value[11];

        lane_conflict[3] = (in_valid[0] + in_valid[2]
            + in_valid[3] + in_valid[4]) > 1;
        if (in_valid[0]) begin
            lane_band[3] = 4'd0;
            lane_source[3] = in_value[0];
        end else if (in_valid[2]) begin
            lane_band[3] = 4'd2;
            lane_source[3] = in_value[2];
        end else if (in_valid[3]) begin
            lane_band[3] = 4'd3;
            lane_source[3] = in_value[3];
        end else begin
            lane_band[3] = 4'd4;
            lane_source[3] = in_value[4];
        end

        lane_conflict[4] = in_valid[1] && in_valid[5];
        if (in_valid[1]) begin
            lane_band[4] = 4'd1;
            lane_source[4] = in_value[1];
        end else begin
            lane_band[4] = 4'd5;
            lane_source[4] = in_value[5];
        end
    end

    // Efinity does not predefine SYNTHESIS. Translation directives select
    // native arithmetic for hardware and portable arithmetic for simulation.
    // Defining SYNTHESIS explicitly enables the vendor simulation model.
    localparam bit USE_NATIVE_DSP = 1'b1
`ifndef SYNTHESIS
        // synthesis translate_off
        && 1'b0
        // synthesis translate_on
`endif
        ;

    // Multiply by 16 / step and add the equally scaled rounding bias before
    // a fixed arithmetic shift. This computes the original quantizer exactly,
    // while moving its variable rounding and shift arithmetic into five DSPs.
    // The complete signed 14-bit domain fits in product[16:0]. All DSP
    // registers are disabled: the public combinational latency is unchanged.
    for (genvar physical_lane=0; physical_lane<5; physical_lane=physical_lane+1) begin : rounding
        wire [3:0] band=lane_band[physical_lane];
        wire signed [13:0] value=lane_source[physical_lane];
        wire signed [17:0] scale=18'sd16 >>> fine_shift(band);
        wire [4:0] adjustment=fine_signed_adjustment(band,value[13]);
        wire signed [17:0] scaled_adjustment=18'(adjustment) << (4-fine_shift(band));
        wire [47:0] product;
        wire unused_product_bits = ^{product[47:17], product[3:0]};
        if (USE_NATIVE_DSP) begin : native
        wire [47:0] unused_cascout;
        wire unused_overflow;
        EFX_DSP48 #(
            .MODE("NORMAL"), .SIGNED(1'b1), .M_SEL("P"), .N_SEL("C"),
            .P_EXT("ALIGN_RIGHT"), .C_EXT("ALIGN_RIGHT"), .W_SEL("X"),
            .A_REG(0), .B_REG(0), .C_REG(0), .P_REG(0), .OP_REG(0),
            .W_REG(0), .O_REG(0), .SHIFTER(0)) round_multiply_add(
            .A({{5{value[13]}},value}),.B(scale),.C(scaled_adjustment),
            .CASCIN(48'd0),.OP(2'd0),.SHIFT_ENA(1'b0),
            .CLK(1'b0),.CE(1'b0),.RST(1'b0),.O(product),
            .CASCOUT(unused_cascout),.OVFL(unused_overflow));
        end else begin : portable
        assign product=48'($signed(value)) * 48'(scale) + 48'(scaled_adjustment);
        end
        assign lane_rounded[physical_lane]=$signed(product[16:4]);
    end

    always_comb begin : scalar_lanes
        integer lane;
        out_valid = in_valid;
        for (lane = 0; lane < 5; lane = lane + 1) begin
            lane_clipped[lane] = lane_rounded[lane] < -13'sd128 || lane_rounded[lane] > 13'sd127;
            lane_quantized[lane] = {lane_clipped[lane], lane_clipped[lane]
                ? (lane_source[lane][13] ? 8'h80 : 8'h7f) : lane_rounded[lane][7:0]};
            lane_fine[lane] = $signed(lane_quantized[lane][7:0]);
            lane_correction[lane] = correction_saturation(
                lane_band[lane],lane_source[lane])
                || periodic_correction(lane_band[lane],
                    lane_source[lane][13],lane_source[lane][4:0]);
            lane_residual_class[lane] = lane_quantized[lane][8]
                || periodic_residual_class(
                    lane_band[lane],lane_source[lane][13],
                    lane_source[lane][4:0]);
        end

        // Output data is qualified exclusively by out_valid.  Statically
        // fan each physical lane back to its fixed color set so the queue's
        // write enables, rather than a duplicate twelve-way value decoder,
        // provide the validity gating.
        out_q[0] = lane_fine[3];
        out_q[1] = lane_fine[4];
        out_q[2] = lane_fine[3];
        out_q[3] = lane_fine[3];
        out_q[4] = lane_fine[3];
        out_q[5] = lane_fine[4];
        out_q[6] = lane_fine[0];
        out_q[7] = lane_fine[0];
        out_q[8] = lane_fine[1];
        out_q[9] = lane_fine[0];
        out_q[10] = lane_fine[1];
        out_q[11] = lane_fine[2];
        out_correction = {lane_correction[2],lane_correction[1],
            lane_correction[0],lane_correction[1],lane_correction[0],
            lane_correction[0],lane_correction[4],lane_correction[3],
            lane_correction[3],lane_correction[3],lane_correction[4],
            lane_correction[3]};
        out_residual_class = {lane_residual_class[2],
            lane_residual_class[1],lane_residual_class[0],
            lane_residual_class[1],lane_residual_class[0],
            lane_residual_class[0],lane_residual_class[4],
            lane_residual_class[3],lane_residual_class[3],
            lane_residual_class[3],lane_residual_class[4],
            lane_residual_class[3]};
    end

    always_ff @(posedge clk) begin
        if (rst || clear_frame)
            protocol_error <= 1'b0;
        else if (lane_conflict != 0)
            protocol_error <= 1'b1;
    end

`ifndef SYNTHESIS
`ifndef LUTCAM_TEST_ALLOW_QUANTIZER_CONFLICTS
    always_ff @(posedge clk) begin
        if (!rst && !clear_frame)
            assert (lane_conflict == 0)
                else $error("coefficient_quantizer: invalid raw-event coloring");
    end
`endif
`endif
endmodule

`default_nettype wire
