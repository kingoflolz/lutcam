// Area-oriented LFTag nine-point geometry preflight.
//
// ChildArena supplies nine descriptors in ascending mass order.  Slots 7/8
// are the fixed top anchors.  The seven data points occupy three canonical
// rows: after orienting the top baseline, the deepest three are the bottom
// row.  Their minimum/maximum baseline projections select the two bottom
// corners.  Internal coordinate widths match the 15-bit X / 14-bit Y FIFO;
// descriptors outside that active-plane contract are rejected explicitly.
module LfTagCornerSelect9 #(
    parameter bit USE_DSP_SUBTRACT = 1'b0
) (
    input  logic                 clk,
    input  logic                 rst,
    input  logic                 in_valid,
    output logic                 in_ready,
    input  logic                 in_points_valid,
    input  logic [15:0]          in_min_baseline_q4,
    output logic                 desc_req_valid,
    output logic [3:0]           desc_req_index,
    input  logic                 desc_rsp_valid,
    input  logic signed [17:0]   desc_rsp_x_q4,
    input  logic signed [17:0]   desc_rsp_y_q4,
    input  logic [7:0]           desc_rsp_mass_q5_3,
    input  logic                 desc_rsp_centroid_valid,
    output logic                 out_valid,
    input  logic                 out_ready,
    output logic [2:0]           out_status,
    output logic                 out_input_valid,
    output logic                 out_baseline_valid,
    output logic                 out_dominant_valid,
    output logic                 out_corners_valid,
    output logic                 out_quad_valid,
    output logic [15:0]          out_corner_index_flat,
    output logic [19:0]          out_remaining_index_flat,
    output logic [71:0]          out_corner_x_q4_flat,
    output logic [71:0]          out_corner_y_q4_flat,
    output logic [7:0]           out_cycles
);
    localparam logic [2:0] STATUS_VALID = 3'd0;
    localparam logic [2:0] STATUS_INVALID_INPUT = 3'd1;
    localparam logic [2:0] STATUS_BASELINE_SHORT = 3'd2;
    localparam logic [2:0] STATUS_DOMINANT_SIDE = 3'd3;
    localparam logic [2:0] STATUS_CORNER_SELECTION = 3'd4;
    localparam logic [2:0] STATUS_INVALID_QUAD = 3'd5;

    typedef enum logic [3:0] {
        S_IDLE,
        S_ANCHOR_SCAN,
        S_BASE_SETUP,
        S_BASE_LENGTH,
        S_BASE_MINIMUM,
        S_SIDE_SCAN,
        S_DEPTH_SCAN,
        S_PICK_SCAN,
        S_CORNER_LEFT_CAPTURE,
        S_CORNER_RIGHT_CAPTURE,
        S_QUAD_CHECK,
        S_FINISH
    } state_t;

    state_t state;
    logic [15:0] minimum_baseline_q4;
    logic [3:0] scan_index;
    logic [1:0] depth_pass;
    logic pick_right;

    logic [14:0] anchor_a_x;
    logic [13:0] anchor_a_y;
    logic [14:0] anchor_b_x;
    logic [13:0] anchor_b_y;
    logic signed [15:0] base_x;
    logic signed [14:0] base_y;
    logic orientation_flip;

    // Before the depth scan value0 holds the baseline norm.  It then holds
    // one maximum depth or one projection extreme.  Three
    // short FIFO passes find the three deepest indices without a rank bank.
    logic [31:0] value0;
    logic [2:0] depth_index0;
    logic [2:0] depth_index1;
    logic [2:0] depth_index2;

    logic [2:0] positive_count;
    logic [2:0] negative_count;
    logic [2:0] bottom_left;
    logic [2:0] bottom_right;
    logic [14:0] bottom_left_x;
    logic [13:0] bottom_left_y;
    logic [14:0] bottom_right_x;
    logic [13:0] bottom_right_y;
    logic [7:0] cycle_counter;
    logic [7:0] anchor_first_mass;
    logic anchor_mass_order_valid;
    logic anchor_centroids_valid;

    logic signed [15:0] descriptor_dx;
    logic signed [14:0] descriptor_dy;

    logic signed [17:0] multiply_a0;
    logic signed [17:0] multiply_b0;
    logic signed [17:0] multiply_a1;
    logic signed [17:0] multiply_b1;
    (* syn_use_dsp = "yes" *) logic signed [31:0] metric_q;
    logic product_launch;
    logic product_valid_q;
    logic product_is_baseline_min_q;
    logic [2:0] product_index_q;
    logic [1:0] product_pass_q;
    logic signed [30:0] cross_value;
    logic signed [31:0] projection_value;
    logic [30:0] norm_value;
    logic [31:0] metric_value;
    logic coordinate_range_valid;
    logic [31:0] compare_left;
    logic [31:0] compare_right;
    logic compare_greater;
    logic [2:0] side_positive_next;
    logic [2:0] side_negative_next;
    logic depth_candidate_wins;
    logic [2:0] product_point_index;


    assign in_ready = state == S_IDLE;
    assign out_valid = (state == S_FINISH);
    assign desc_req_valid = ((state == S_IDLE) && in_valid
            && in_points_valid)
        || ((state == S_ANCHOR_SCAN) && desc_rsp_valid
            && (scan_index == 7))
        || (state == S_BASE_MINIMUM)
        || ((state == S_SIDE_SCAN) && (
            (desc_rsp_valid && (scan_index < 6))
            || (product_valid_q && !product_is_baseline_min_q
                && (product_index_q == 6))))
        || ((state == S_DEPTH_SCAN) && (
            (desc_rsp_valid
                && ((scan_index < 6) || (depth_pass < 2)))
            || (product_valid_q && (product_pass_q == 2)
                && (product_index_q == 6))))
        || ((state == S_PICK_SCAN) && (
            (desc_rsp_valid && (scan_index < 2))
            || (product_valid_q && (product_index_q == 2))))
        || ((state == S_CORNER_LEFT_CAPTURE) && desc_rsp_valid);

    always_comb begin
        case (scan_index[1:0])
            2'd0: desc_req_index = {1'b0, depth_index0};
            2'd1: desc_req_index = {1'b0, depth_index1};
            default: desc_req_index = {1'b0, depth_index2};
        endcase
        if (state == S_IDLE)
            desc_req_index = 4'd7;
        else if (state == S_ANCHOR_SCAN)
            desc_req_index = 4'd8;
        else if (state == S_BASE_MINIMUM)
            desc_req_index = 4'd0;
        else if (state == S_SIDE_SCAN) begin
            if (product_valid_q && !product_is_baseline_min_q
                    && (product_index_q == 6))
                desc_req_index = 4'd0;
            else
                desc_req_index = scan_index + 1'b1;
        end
        else if (state == S_DEPTH_SCAN) begin
            if (product_valid_q && (product_pass_q == 2)
                    && (product_index_q == 6))
                desc_req_index = {1'b0, depth_index0};
            else if (scan_index == 6)
                desc_req_index = 4'd0;
            else
                desc_req_index = scan_index + 1'b1;
        end else if (state == S_PICK_SCAN) begin
            if (product_valid_q && (product_index_q == 2)) begin
                if (pick_right)
                    desc_req_index = {1'b0, bottom_left};
                else
                    desc_req_index = {1'b0, depth_index0};
            end else begin
                case (scan_index[1:0])
                    2'd0: desc_req_index = {1'b0, depth_index1};
                    default: desc_req_index = {1'b0, depth_index2};
                endcase
            end
        end else if (state == S_CORNER_LEFT_CAPTURE)
            desc_req_index = {1'b0, bottom_right};
    end

    logic signed [15:0] base_delta_x;
    logic signed [14:0] base_delta_y;
    logic signed [17:0] quad_delta_x_right, quad_delta_y_left;
    logic signed [17:0] quad_delta_y_right, quad_delta_x_left;
    LfTagCornerSubtract #(.WIDTH(16), .USE_DSP(USE_DSP_SUBTRACT)) descriptor_x (
        .a({1'b0, desc_rsp_x_q4[14:0]}), .b({1'b0, anchor_a_x}), .result(descriptor_dx));
    LfTagCornerSubtract #(.WIDTH(15), .USE_DSP(USE_DSP_SUBTRACT)) descriptor_y (
        .a({1'b0, desc_rsp_y_q4[13:0]}), .b({1'b0, anchor_a_y}), .result(descriptor_dy));
    LfTagCornerSubtract #(.WIDTH(16), .USE_DSP(USE_DSP_SUBTRACT)) baseline_x (
        .a({1'b0, anchor_b_x}), .b({1'b0, anchor_a_x}), .result(base_delta_x));
    LfTagCornerSubtract #(.WIDTH(15), .USE_DSP(USE_DSP_SUBTRACT)) baseline_y (
        .a({1'b0, anchor_b_y}), .b({1'b0, anchor_a_y}), .result(base_delta_y));
    LfTagCornerSubtract #(.WIDTH(18), .USE_DSP(USE_DSP_SUBTRACT)) quad_x_right (
        .a({3'b0, bottom_right_x}), .b({3'b0, anchor_b_x}), .result(quad_delta_x_right));
    LfTagCornerSubtract #(.WIDTH(18), .USE_DSP(USE_DSP_SUBTRACT)) quad_y_left (
        .a({4'b0, bottom_left_y}), .b({4'b0, anchor_b_y}), .result(quad_delta_y_left));
    LfTagCornerSubtract #(.WIDTH(18), .USE_DSP(USE_DSP_SUBTRACT)) quad_y_right (
        .a({4'b0, anchor_b_y}), .b({4'b0, bottom_right_y}), .result(quad_delta_y_right));
    LfTagCornerSubtract #(.WIDTH(18), .USE_DSP(USE_DSP_SUBTRACT)) quad_x_left (
        .a({3'b0, bottom_left_x}), .b({3'b0, anchor_b_x}), .result(quad_delta_x_left));
    assign coordinate_range_valid = !(|desc_rsp_x_q4[17:15])
        && !(|desc_rsp_y_q4[17:14]);

    assign product_launch = (state == S_BASE_LENGTH)
        || (state == S_BASE_MINIMUM)
        || (((state == S_SIDE_SCAN) || (state == S_DEPTH_SCAN)
                || (state == S_PICK_SCAN)) && desc_rsp_valid)
        || ((state == S_QUAD_CHECK) && !product_valid_q);
    assign metric_value = metric_q;
    assign cross_value = metric_value[30:0];
    assign projection_value = metric_value;
    assign norm_value = metric_value[30:0];
    assign side_positive_next = positive_count
        + ((cross_value > 0) ? 3'd1 : 3'd0);
    assign side_negative_next = negative_count
        + ((cross_value < 0) ? 3'd1 : 3'd0);
    assign depth_candidate_wins = (cross_value > 0)
        && ((product_pass_q == 0)
            || (product_index_q != depth_index0))
        && ((product_pass_q < 2)
            || (product_index_q != depth_index1))
        && compare_greater;

    always_comb begin
        case (product_index_q)
            3'd0: product_point_index = depth_index0;
            3'd1: product_point_index = depth_index1;
            default: product_point_index = depth_index2;
        endcase
    end

    logic [32:0] compare_difference;
    // Keep the metric and saved extreme on fixed subtractor inputs. Borrow
    // selects strict minimum; a nonzero non-borrow difference selects strict
    // maximum. Flipping only the sign bits gives signed projection ordering.
    always_comb begin
        compare_left = {1'b0, metric_value[30:0]};
        compare_right = value0;
        if (state == S_PICK_SCAN) begin
            compare_left[31] = ~metric_value[31];
            compare_right[31] = ~value0[31];
        end else if ((state == S_SIDE_SCAN) && product_is_baseline_min_q)
            compare_left[31] = metric_value[31];
        compare_difference = {1'b0, compare_left} - {1'b0, compare_right};
        compare_greater = ((state == S_PICK_SCAN) && !pick_right)
            ? compare_difference[32]
            : (!compare_difference[32] && (|compare_difference[31:0]));
    end

    always_comb begin
        multiply_a0 = {{2{base_x[15]}}, base_x};
        multiply_b0 = {{2{descriptor_dx[15]}}, descriptor_dx};
        multiply_a1 = {{3{base_y[14]}}, base_y};
        multiply_b1 = {{3{descriptor_dy[14]}}, descriptor_dy};
        case (state)
            S_BASE_LENGTH: begin
                multiply_a0 = {{2{base_x[15]}}, base_x};
                multiply_b0 = {{2{base_x[15]}}, base_x};
                multiply_a1 = {{3{base_y[14]}}, base_y};
                multiply_b1 = {{3{base_y[14]}}, base_y};
            end
            S_BASE_MINIMUM: begin
                multiply_a0 = $signed({2'b00, minimum_baseline_q4});
                multiply_b0 = $signed({2'b00, minimum_baseline_q4});
                multiply_a1 = 18'sb0;
            end
            S_SIDE_SCAN, S_DEPTH_SCAN: begin
                multiply_a0 = {{2{base_x[15]}}, base_x};
                multiply_b0 = {{3{descriptor_dy[14]}}, descriptor_dy};
                multiply_a1 = -$signed({{3{base_y[14]}}, base_y});
                multiply_b1 = {{2{descriptor_dx[15]}}, descriptor_dx};
            end
            S_PICK_SCAN: begin
                multiply_a0 = {{2{base_x[15]}}, base_x};
                multiply_b0 = {{2{descriptor_dx[15]}}, descriptor_dx};
                multiply_a1 = {{3{base_y[14]}}, base_y};
                multiply_b1 = {{3{descriptor_dy[14]}}, descriptor_dy};
            end
            S_QUAD_CHECK: begin
                multiply_a0 = quad_delta_x_right;
                multiply_b0 = quad_delta_y_left;
                multiply_a1 = quad_delta_y_right;
                multiply_b1 = quad_delta_x_left;
            end
            default: begin
            end
        endcase
    end

    assign out_corner_index_flat = {
        1'b0, bottom_left, 1'b0, bottom_right,
        (orientation_flip ? 4'd8 : 4'd7),
        (orientation_flip ? 4'd7 : 4'd8)
    };
    assign out_corner_x_q4_flat = {
        3'b000, bottom_left_x, 3'b000, bottom_right_x,
        3'b000, anchor_b_x, 3'b000, anchor_a_x
    };
    assign out_corner_y_q4_flat = {
        4'b0000, bottom_left_y, 4'b0000, bottom_right_y,
        4'b0000, anchor_b_y, 4'b0000, anchor_a_y
    };
    assign out_cycles = cycle_counter;
    // Each slot skips the two exclusions without sorting them or building
    // a dynamic increment chain. This also preserves equal-index payloads.
    for (genvar slot = 0; slot < 5; slot = slot + 1) begin : g_remaining_order
        localparam logic [2:0] SLOT = 3'(slot);
        localparam logic [2:0] NEXT_SLOT = 3'(slot + 1);
        wire hit = (bottom_left <= SLOT) || (bottom_right <= SLOT);
        wire skip_two = ((bottom_left <= SLOT) && (bottom_right <= NEXT_SLOT))
            || ((bottom_right <= SLOT) && (bottom_left <= NEXT_SLOT));
        assign out_remaining_index_flat[4*slot +: 4]
            = skip_two ? 4'(slot + 2) : hit ? 4'(slot + 1) : 4'(slot);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            out_status <= STATUS_INVALID_INPUT;
            out_input_valid <= 1'b0;
            out_baseline_valid <= 1'b0;
            out_dominant_valid <= 1'b0;
            out_corners_valid <= 1'b0;
            out_quad_valid <= 1'b0;
            cycle_counter <= 0;
            scan_index <= 0;
            depth_pass <= 0;
            pick_right <= 1'b0;
            orientation_flip <= 1'b0;
            anchor_first_mass <= 0;
            anchor_mass_order_valid <= 1'b0;
            anchor_centroids_valid <= 1'b0;
            product_valid_q <= 1'b0;
        end else begin
            if ((state != S_IDLE) && (state != S_FINISH))
                cycle_counter <= cycle_counter + 1'b1;

            // The sum register maps to the existing two-product DSP cascade.
            // Subtracting jobs negate one bounded operand before multiplication,
            // so the fabric does not need a separate 32-bit metric add/subtract.
            // Metadata advances beside the result; only valid needs reset.
            if (product_launch) begin
                metric_q <= 32'($signed(multiply_a0) * $signed(multiply_b0)
                    + $signed(multiply_a1) * $signed(multiply_b1));
                product_valid_q <= 1'b1;
                product_is_baseline_min_q <= state == S_BASE_MINIMUM;
                product_index_q <= scan_index[2:0];
                product_pass_q <= depth_pass;
            end else begin
                product_valid_q <= 1'b0;
            end

            case (state)
                S_IDLE: begin
                    if (in_valid) begin
                        minimum_baseline_q4 <= in_min_baseline_q4;
                        out_status <= STATUS_VALID;
                        out_input_valid <= in_points_valid;
                        out_baseline_valid <= 1'b0;
                        out_dominant_valid <= 1'b0;
                        out_corners_valid <= 1'b0;
                        out_quad_valid <= 1'b0;
                        cycle_counter <= 0;
                        orientation_flip <= 1'b0;
                        anchor_centroids_valid <= 1'b1;
                        anchor_mass_order_valid <= 1'b1;
                        product_valid_q <= 1'b0;
                        scan_index <= 7;
                        if (in_points_valid)
                            state <= S_ANCHOR_SCAN;
                        else begin
                            out_status <= STATUS_INVALID_INPUT;
                            cycle_counter <= 1;
                            state <= S_FINISH;
                        end
                    end
                end

                S_ANCHOR_SCAN: if (desc_rsp_valid) begin
                    if (!coordinate_range_valid)
                        out_input_valid <= 1'b0;
                    if (!desc_rsp_centroid_valid)
                        anchor_centroids_valid <= 1'b0;
                    if (scan_index == 7) begin
                        anchor_b_x <= desc_rsp_x_q4[14:0];
                        anchor_b_y <= desc_rsp_y_q4[13:0];
                        anchor_first_mass <= desc_rsp_mass_q5_3;
                        scan_index <= 8;
                    end else begin
                        anchor_a_x <= desc_rsp_x_q4[14:0];
                        anchor_a_y <= desc_rsp_y_q4[13:0];
                        state <= S_BASE_SETUP;
                    end
                end

                S_BASE_SETUP: begin
                    base_x <= base_delta_x;
                    base_y <= base_delta_y;
                    state <= S_BASE_LENGTH;
                end

                // The norm launches here.  The next state consumes it while
                // launching the minimum-baseline square in the same DSP lane.
                S_BASE_LENGTH: state <= S_BASE_MINIMUM;
                S_BASE_MINIMUM: begin
                    if (product_valid_q)
                        value0 <= {1'b0, norm_value};
                    positive_count <= 0;
                    negative_count <= 0;
                    scan_index <= 0;
                    state <= S_SIDE_SCAN;
                end

                S_SIDE_SCAN: begin
                    if (desc_rsp_valid) begin
                        if (!desc_rsp_centroid_valid
                                || !coordinate_range_valid)
                            out_input_valid <= 1'b0;
                        if (scan_index == 6)
                            anchor_mass_order_valid <=
                                desc_rsp_mass_q5_3 < anchor_first_mass;
                        if (scan_index < 6)
                            scan_index <= scan_index + 1'b1;
                    end

                    if (product_valid_q) begin
                        if (product_is_baseline_min_q) begin
                            if (!out_input_valid) begin
                                out_status <= STATUS_INVALID_INPUT;
                                state <= S_FINISH;
                            end else if (compare_greater) begin
                                out_status <= STATUS_BASELINE_SHORT;
                                state <= S_FINISH;
                            end else begin
                                out_baseline_valid <= 1'b1;
                            end
                        end else begin
                            positive_count <= side_positive_next;
                            negative_count <= side_negative_next;
                            if (product_index_q == 6) begin
                                if (!anchor_centroids_valid) begin
                                    out_input_valid <= 1'b0;
                                    out_status <= STATUS_INVALID_INPUT;
                                    state <= S_FINISH;
                                end else if (!out_input_valid) begin
                                    out_status <= STATUS_INVALID_INPUT;
                                    state <= S_FINISH;
                                end else if (!anchor_mass_order_valid) begin
                                    out_status <= STATUS_CORNER_SELECTION;
                                    state <= S_FINISH;
                                end else if ((side_positive_next < 6)
                                        && (side_negative_next < 6)) begin
                                    out_status <= STATUS_DOMINANT_SIDE;
                                    state <= S_FINISH;
                                end else begin
                                    out_dominant_valid <= 1'b1;
                                    orientation_flip <= side_negative_next
                                        > side_positive_next;
                                    if (side_negative_next
                                            > side_positive_next) begin
                                        anchor_a_x <= anchor_b_x;
                                        anchor_a_y <= anchor_b_y;
                                        anchor_b_x <= anchor_a_x;
                                        anchor_b_y <= anchor_a_y;
                                        base_x <= -base_x;
                                        base_y <= -base_y;
                                    end
                                    value0 <= 0;
                                    depth_index0 <= 0;
                                    depth_index1 <= 0;
                                    depth_index2 <= 0;
                                    scan_index <= 0;
                                    depth_pass <= 0;
                                    state <= S_DEPTH_SCAN;
                                end
                            end
                        end
                    end
                end

                S_DEPTH_SCAN: begin
                    if (desc_rsp_valid) begin
                        if (scan_index == 6) begin
                            scan_index <= 0;
                            if (depth_pass < 2)
                                depth_pass <= depth_pass + 1'b1;
                        end else begin
                            scan_index <= scan_index + 1'b1;
                        end
                    end

                    if (product_valid_q) begin
                        if (depth_candidate_wins) begin
                            value0 <= {1'b0, $unsigned(cross_value)};
                            case (product_pass_q)
                                2'd0: depth_index0 <= product_index_q;
                                2'd1: depth_index1 <= product_index_q;
                                default: depth_index2 <= product_index_q;
                            endcase
                        end

                        if (product_index_q == 6) begin
                            value0 <= 0;
                            if (product_pass_q == 2) begin
                                scan_index <= 0;
                                pick_right <= 1'b0;
                                state <= S_PICK_SCAN;
                            end
                        end
                    end
                end

                S_PICK_SCAN: begin
                    if (desc_rsp_valid && (scan_index < 2))
                        scan_index <= scan_index + 1'b1;

                    if (product_valid_q) begin
                        if (product_index_q == 0) begin
                            value0 <= projection_value;
                            if (pick_right)
                                bottom_right <= product_point_index;
                            else
                                bottom_left <= product_point_index;
                        end else if (compare_greater) begin
                            value0 <= projection_value;
                            if (pick_right)
                                bottom_right <= product_point_index;
                            else
                                bottom_left <= product_point_index;
                        end

                        if (product_index_q == 2) begin
                            if (pick_right) begin
                                state <= S_CORNER_LEFT_CAPTURE;
                            end else begin
                                pick_right <= 1'b1;
                                scan_index <= 0;
                                value0 <= 0;
                            end
                        end
                    end
                end

                S_CORNER_LEFT_CAPTURE: if (desc_rsp_valid) begin
                    bottom_left_x <= desc_rsp_x_q4[14:0];
                    bottom_left_y <= desc_rsp_y_q4[13:0];
                    state <= S_CORNER_RIGHT_CAPTURE;
                end

                S_CORNER_RIGHT_CAPTURE: if (desc_rsp_valid) begin
                    bottom_right_x <= desc_rsp_x_q4[14:0];
                    bottom_right_y <= desc_rsp_y_q4[13:0];
                    out_corners_valid <= 1'b1;
                    state <= S_QUAD_CHECK;
                end

                S_QUAD_CHECK: if (product_valid_q) begin
                    if ((bottom_left == bottom_right)
                            || (cross_value <= 0)) begin
                        out_status <= (bottom_left == bottom_right)
                            ? STATUS_CORNER_SELECTION : STATUS_INVALID_QUAD;
                        state <= S_FINISH;
                    end else begin
                        out_quad_valid <= 1'b1;
                        out_status <= STATUS_VALID;
                        state <= S_FINISH;
                    end
                end

                S_FINISH: if (out_ready) state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

// Exact modulo-width coordinate subtraction. The multiplier passes A and
// the arithmetic unit subtracts B; this primitive contains no registers.
/* verilator lint_off DECLFILENAME */
module LfTagCornerSubtract #(parameter integer WIDTH=18, parameter bit USE_DSP=1)(

    input wire signed [WIDTH-1:0] a,b,
    output wire signed [WIDTH-1:0] result
);
    localparam bit USE_NATIVE = USE_DSP
`ifndef SYNTHESIS
        // synthesis translate_off
        && 1'b0
        // synthesis translate_on
`endif
        ;
    generate if (USE_NATIVE) begin : native
        /* verilator lint_off UNUSEDSIGNAL */
        wire [47:0] difference;
        /* verilator lint_on UNUSEDSIGNAL */
        wire [47:0] unused_cascade;
        wire unused_overflow;
        EFX_DSP48 #(
            .MODE("NORMAL"),.SIGNED(1),.M_SEL("P"),.N_SEL("C"),
            .P_EXT("ALIGN_RIGHT"),.C_EXT("ALIGN_RIGHT"),
            .W_SEL("X"),.CASCOUT_SEL("W"),
            .A_REG(0),.B_REG(0),.C_REG(0),.P_REG(0),.OP_REG(0),
            .W_REG(0),.O_REG(0),.SHIFTER(0)
        ) arithmetic (
            .A({{(19-WIDTH){a[WIDTH-1]}},a}),.B(18'd1),
            .C({{(18-WIDTH){b[WIDTH-1]}},b}),.CASCIN(48'd0),
            .OP(2'b01),.SHIFT_ENA(1'b0),.CLK(1'b0),.CE(1'b0),.RST(1'b0),
            .O(difference),.CASCOUT(unused_cascade),.OVFL(unused_overflow)
        );
        assign result=difference[WIDTH-1:0];
    end else begin : portable
        assign result=a-b;
    end endgenerate
endmodule
/* verilator lint_on DECLFILENAME */
