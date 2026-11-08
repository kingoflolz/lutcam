// Retained fabric lifting reference for native DSP arithmetic/latency regression.
// Original production SHA256: 242acc0b5056fa82f01fc9be7fa87bb3ff5f011df8d5dbbdc45b5630cea83f7f
`default_nettype none

// One complete, full-rate horizontal reversible CDF 5/3 analysis stage.
//
// Input pairs are {even,odd} samples in row order. The predict step needs the
// following even sample, so one pair is retained as lookahead state. The last
// pair duplicates its even sample at the right boundary. The update step uses
// the preceding predicted high value and duplicates the first high at the
// left boundary. Valid bubbles are allowed and output initiation interval is
// one clock for a gap-free row.
module horizontal_lift_stage_baseline #(
    parameter integer DATA_WIDTH = 14
) (
    input  wire                           clk,
    input  wire                           rst_n,

    input  wire                           in_valid,
    input  wire                           in_first,
    input  wire                           in_last,
    input  wire signed [DATA_WIDTH-1:0]   in_even,
    input  wire signed [DATA_WIDTH-1:0]   in_odd,

    output logic                          out_valid,
    output logic                          out_first,
    output logic                          out_last,
    output logic signed [DATA_WIDTH-1:0]  out_low,
    output logic signed [DATA_WIDTH-1:0]  out_high
);
    localparam integer SUM_WIDTH = DATA_WIDTH + 1;
    localparam integer UPDATE_WIDTH = DATA_WIDTH + 2;

    logic pending_valid;
    logic pending_first;
    logic pending_last;
    logic signed [DATA_WIDTH-1:0] pending_even;
    logic signed [DATA_WIDTH-1:0] pending_odd;

    logic launch;
    logic signed [DATA_WIDTH-1:0] right_even;
    logic signed [SUM_WIDTH-1:0] predict_sum;
    logic signed [SUM_WIDTH-1:0] predicted_half;
    logic signed [DATA_WIDTH-1:0] high_value;
    logic predict_valid_q;
    logic predict_first_q;
    logic predict_last_q;
    logic signed [DATA_WIDTH-1:0] predict_even_q;
    logic signed [DATA_WIDTH-1:0] predict_high_q;
    logic signed [DATA_WIDTH-1:0] update_left_high;
    logic signed [UPDATE_WIDTH-1:0] update_sum;
    // The quartered sum always fits DATA_WIDTH. The low coefficient is
    // defined modulo DATA_WIDTH, so retaining a wider final carry only burns
    // fabric without changing any emitted bit.
    logic signed [DATA_WIDTH-1:0] update_quarter;
    logic signed [DATA_WIDTH-1:0] low_value;
`ifndef SYNTHESIS
    logic signed [SUM_WIDTH:0] high_wide_check;
    logic signed [UPDATE_WIDTH-1:0] low_wide_check;
`endif

    always_comb begin
        launch = pending_valid && (pending_last || in_valid);
        right_even = pending_last ? pending_even : in_even;
        predict_sum =
            $signed({pending_even[DATA_WIDTH-1], pending_even})
            + $signed({right_even[DATA_WIDTH-1], right_even});
        predicted_half = predict_sum >>> 1;
        high_value = DATA_WIDTH'(
            $signed({pending_odd[DATA_WIDTH-1], pending_odd})
                - $signed({predicted_half[SUM_WIDTH-1], predicted_half})
        );
        // out_high is the most recently emitted high coefficient.  Since
        // every launched prediction is emitted in order, it is also the
        // left neighbour needed by the following update.  This remains true
        // across valid bubbles because out_high holds its value; the first
        // coefficient of each row duplicates its own high instead.
        update_left_high = predict_first_q ? predict_high_q : out_high;
        update_sum =
            $signed({{2{update_left_high[DATA_WIDTH-1]}}, update_left_high})
            + $signed({{2{predict_high_q[DATA_WIDTH-1]}}, predict_high_q})
            + $signed(UPDATE_WIDTH'(2));
        update_quarter = DATA_WIDTH'(update_sum >>> 2);
        low_value = $signed(predict_even_q) + $signed(update_quarter);
`ifndef SYNTHESIS
        high_wide_check =
            $signed({pending_odd[DATA_WIDTH-1], pending_odd})
            - $signed({predicted_half[SUM_WIDTH-1], predicted_half});
        low_wide_check =
            $signed({{2{predict_even_q[DATA_WIDTH-1]}}, predict_even_q})
            + $signed({{2{update_quarter[DATA_WIDTH-1]}}, update_quarter});
`endif
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending_valid <= 1'b0;
            pending_first <= 1'b0;
            pending_last <= 1'b0;
            pending_even <= '0;
            pending_odd <= '0;
            predict_valid_q <= 1'b0;
            predict_first_q <= 1'b0;
            predict_last_q <= 1'b0;
            predict_even_q <= '0;
            predict_high_q <= '0;
            out_valid <= 1'b0;
            out_first <= 1'b0;
            out_last <= 1'b0;
            out_low <= '0;
            out_high <= '0;
        end else begin
            predict_valid_q <= launch;
            if (launch) begin
                predict_first_q <= pending_first;
                predict_last_q <= pending_last;
                predict_even_q <= pending_even;
                predict_high_q <= high_value;
            end

            out_valid <= predict_valid_q;
            if (predict_valid_q) begin
                out_first <= predict_first_q;
                out_last <= predict_last_q;
                out_low <= low_value;
                out_high <= predict_high_q;
            end

            if (in_valid) begin
                pending_valid <= 1'b1;
                pending_first <= in_first;
                pending_last <= in_last;
                pending_even <= in_even;
                pending_odd <= in_odd;
            end else if (launch) begin
                pending_valid <= 1'b0;
            end
        end
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
        end else begin
            if (in_valid && pending_valid) begin
                assert (pending_last == in_first)
                    else $error("horizontal_lift_stage: missing or unexpected row boundary");
            end
            if (launch) begin
                assert (high_wide_check[SUM_WIDTH]
                        == high_wide_check[SUM_WIDTH-1])
                    else $error("horizontal_lift_stage: predict overflow");
            end
            if (predict_valid_q) begin
                assert (low_wide_check[UPDATE_WIDTH-1:DATA_WIDTH]
                    == {(UPDATE_WIDTH-DATA_WIDTH){
                        low_wide_check[DATA_WIDTH-1]}})
                    else $error("horizontal_lift_stage: update overflow");
            end
        end
    end
`endif
endmodule

`default_nettype wire
