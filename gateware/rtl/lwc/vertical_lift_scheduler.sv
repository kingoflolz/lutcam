`default_nettype none

// Bounded, one-job-per-clock vertical reversible CDF 5/3 scheduler.
//
// A column context retains only the current even sample, current odd sample,
// and preceding predicted high value. OP_ADVANCE supplies the next even sample
// and emits the completed low/high pair. OP_TAIL duplicates the final even
// sample at the bottom boundary and emits the last pair. The state RAM has one
// synchronous read and one independent write; predict and update are separated
// by registers so the accepted-job initiation interval remains one clock.
module vertical_lift_scheduler #(
    parameter integer LANES = 1,
    parameter integer DATA_WIDTH = 14,
    parameter integer RAW_WIDTH = DATA_WIDTH,
    parameter integer HIGH_WIDTH = DATA_WIDTH,
    parameter integer STATE_DEPTH = 960,
    parameter integer ADDRESS_WIDTH = $clog2(STATE_DEPTH),
    parameter integer CONTEXT_WIDTH = 2,
    parameter integer COLUMN_WIDTH = 10,
    parameter integer ROW_WIDTH = 10,
    parameter integer SPLIT_STATE_FIELDS = 0
) (
    input  wire                                           clk,
    input  wire                                           rst_n,

    input  wire                                           in_valid,
    input  wire [1:0]                                     in_op,
    input  wire [ADDRESS_WIDTH-1:0]                       in_state_address,
    input  wire [CONTEXT_WIDTH-1:0]                       in_context,
    input  wire [COLUMN_WIDTH-1:0]                        in_column,
    input  wire [ROW_WIDTH-1:0]                           in_output_y,
    input  wire                                           in_output_first,
    input  wire                                           in_output_last,
    input  wire signed [LANES-1:0][DATA_WIDTH-1:0]        in_value,

    output logic                                          out_valid,
    output logic [CONTEXT_WIDTH-1:0]                      out_context,
    output logic [COLUMN_WIDTH-1:0]                       out_column,
    output logic [ROW_WIDTH-1:0]                          out_y,
    output logic                                          out_first,
    output logic                                          out_last,
    output logic signed [LANES-1:0][DATA_WIDTH-1:0]       out_low,
    output logic signed [LANES-1:0][DATA_WIDTH-1:0]       out_high
);
    localparam logic [1:0] OP_EVEN = 2'd0;
    localparam logic [1:0] OP_ODD = 2'd1;
    localparam logic [1:0] OP_ADVANCE = 2'd2;
    localparam logic [1:0] OP_TAIL = 2'd3;

    localparam integer SUM_WIDTH = DATA_WIDTH + 1;
    localparam integer WIDE_WIDTH = DATA_WIDTH + 2;
    localparam integer LANE_STATE_WIDTH = 2 * RAW_WIDTH + HIGH_WIDTH;
    localparam integer STATE_WIDTH = LANES * LANE_STATE_WIDTH;
    localparam integer EVEN_OFFSET = 0;
    localparam integer ODD_OFFSET = RAW_WIDTH;
    localparam integer HIGH_OFFSET = 2 * RAW_WIDTH;

    function automatic logic signed [DATA_WIDTH-1:0] extend_raw(
        input logic signed [RAW_WIDTH-1:0] value
    );
        extend_raw = {{(DATA_WIDTH-RAW_WIDTH){value[RAW_WIDTH-1]}}, value};
    endfunction

    function automatic logic signed [DATA_WIDTH-1:0] extend_high(
        input logic signed [HIGH_WIDTH-1:0] value
    );
        extend_high = {{(DATA_WIDTH-HIGH_WIDTH){value[HIGH_WIDTH-1]}}, value};
    endfunction

    logic [STATE_WIDTH-1:0] state_read_q;

    // Metadata paired with the synchronous state-memory result.
    logic read_valid_q;
    logic [1:0] read_op_q;
    logic [ADDRESS_WIDTH-1:0] read_address_q;
    logic [CONTEXT_WIDTH-1:0] read_context_q;
    logic [COLUMN_WIDTH-1:0] read_column_q;
    logic [ROW_WIDTH-1:0] read_output_y_q;
    logic read_output_first_q;
    logic read_output_last_q;
    logic signed [LANES-1:0][DATA_WIDTH-1:0] read_value_q;

    logic signed [LANES-1:0][DATA_WIDTH-1:0] state_even;
    logic signed [LANES-1:0][DATA_WIDTH-1:0] state_odd;
    logic signed [LANES-1:0][DATA_WIDTH-1:0] state_previous_high;
    logic signed [LANES-1:0][DATA_WIDTH-1:0] predict_right_even;
    logic signed [LANES-1:0][SUM_WIDTH-1:0] predict_sum;
    logic signed [LANES-1:0][SUM_WIDTH-1:0] predict_half;

    // Registered predict result and the state/metadata needed by update.
    logic predict_valid_q;
    logic [1:0] predict_op_q;
    logic [ADDRESS_WIDTH-1:0] predict_address_q;
    logic [CONTEXT_WIDTH-1:0] predict_context_q;
    logic [COLUMN_WIDTH-1:0] predict_column_q;
    logic [ROW_WIDTH-1:0] predict_output_y_q;
    logic predict_output_first_q;
    logic predict_output_last_q;
    logic [STATE_WIDTH-1:0] predict_record_q;
    logic signed [LANES-1:0][DATA_WIDTH-1:0] predict_input_q;
    logic signed [LANES-1:0][DATA_WIDTH-1:0] predict_even_q;
    logic signed [LANES-1:0][DATA_WIDTH-1:0] predict_previous_high_q;
    logic signed [LANES-1:0][DATA_WIDTH-1:0] predict_high_q;

    logic signed [LANES-1:0][DATA_WIDTH-1:0] update_left_high;
    logic signed [LANES-1:0][WIDE_WIDTH-1:0] update_sum;
    // The quartered update fits DATA_WIDTH and the emitted low coefficient is
    // its exact DATA_WIDTH sum with the saved even sample. Do not retain an
    // unobservable carry bit in every arithmetic lane.
    logic signed [LANES-1:0][DATA_WIDTH-1:0] update_quarter;
    logic [LANES-1:0][LANE_STATE_WIDTH-1:0] write_lane_record;
    // These packed-state aliases are intentionally unused when split field
    // memories are selected; that branch updates each field independently.
    logic [STATE_WIDTH-1:0] unused_packed_write_record;
    logic unused_packed_write_valid;

    genvar lane;
    generate
        for (lane = 0; lane < LANES; lane = lane + 1) begin : lanes
            localparam integer LANE_BASE = lane * LANE_STATE_WIDTH;
            LwcRegisteredAddSub #(.WIDTH(DATA_WIDTH),.SUBTRACT(1)) predict_register (
                .clk,.rst_n,.enable(read_valid_q),.a(state_odd[lane]),
                .b(predict_half[lane][DATA_WIDTH-1:0]),.result(predict_high_q[lane])
            );
            LwcRegisteredAddSub #(.WIDTH(DATA_WIDTH)) update_register (
                .clk,.rst_n,
                .enable(predict_valid_q && (predict_op_q == OP_ADVANCE || predict_op_q == OP_TAIL)),
                .a(predict_even_q[lane]),.b(update_quarter[lane]),.result(out_low[lane])
            );

            always_comb begin
                state_even[lane] = extend_raw(state_read_q[
                    LANE_BASE + EVEN_OFFSET +: RAW_WIDTH
                ]);
                state_odd[lane] = extend_raw(state_read_q[
                    LANE_BASE + ODD_OFFSET +: RAW_WIDTH
                ]);
                state_previous_high[lane] = extend_high(state_read_q[
                    LANE_BASE + HIGH_OFFSET +: HIGH_WIDTH
                ]);
                predict_right_even[lane] = read_op_q == OP_TAIL
                    ? state_even[lane] : read_value_q[lane];
                predict_sum[lane] =
                    $signed({state_even[lane][DATA_WIDTH-1], state_even[lane]})
                    + $signed({predict_right_even[lane][DATA_WIDTH-1],
                        predict_right_even[lane]});
                predict_half[lane] = predict_sum[lane] >>> 1;

                update_left_high[lane] = predict_output_first_q
                    ? predict_high_q[lane] : predict_previous_high_q[lane];
                update_sum[lane] =
                    $signed({{2{update_left_high[lane][DATA_WIDTH-1]}},
                        update_left_high[lane]})
                    + $signed({{2{predict_high_q[lane][DATA_WIDTH-1]}},
                        predict_high_q[lane]})
                    + $signed(WIDE_WIDTH'(2));
                update_quarter[lane] = DATA_WIDTH'(update_sum[lane] >>> 2);

                write_lane_record[lane] = predict_record_q[
                    LANE_BASE +: LANE_STATE_WIDTH
                ];
                case (predict_op_q)
                    OP_EVEN: begin
                        write_lane_record[lane][EVEN_OFFSET +: RAW_WIDTH]
                            = predict_input_q[lane][RAW_WIDTH-1:0];
                    end
                    OP_ODD: begin
                        write_lane_record[lane][ODD_OFFSET +: RAW_WIDTH]
                            = predict_input_q[lane][RAW_WIDTH-1:0];
                    end
                    OP_ADVANCE: begin
                        write_lane_record[lane][EVEN_OFFSET +: RAW_WIDTH]
                            = predict_input_q[lane][RAW_WIDTH-1:0];
                        write_lane_record[lane][HIGH_OFFSET +: HIGH_WIDTH]
                            = predict_high_q[lane][HIGH_WIDTH-1:0];
                    end
                    OP_TAIL: begin
                        // No later job consumes this context in the frame.
                    end
                    default: begin
                    end
                endcase
            end

`ifndef SYNTHESIS
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    // The diagnostic process has no state to reset.
                end else begin
                    if (read_valid_q && read_op_q != OP_TAIL) begin
                        assert ($signed(read_value_q[lane])
                            == $signed({{(DATA_WIDTH-RAW_WIDTH){
                                read_value_q[lane][RAW_WIDTH-1]}},
                                read_value_q[lane][RAW_WIDTH-1:0]}))
                            else $error("vertical_lift_scheduler: raw state truncation");
                    end
                    if (predict_valid_q && predict_op_q == OP_ADVANCE) begin
                        assert ($signed(predict_high_q[lane])
                            == $signed({{(DATA_WIDTH-HIGH_WIDTH){
                                predict_high_q[lane][HIGH_WIDTH-1]}},
                                predict_high_q[lane][HIGH_WIDTH-1:0]}))
                            else $error("vertical_lift_scheduler: high state truncation");
                    end
                end
            end
`endif
        end
    endgenerate

    always_comb begin
        unused_packed_write_record = write_lane_record;
        unused_packed_write_valid = predict_valid_q;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_valid_q <= 1'b0;
            read_op_q <= OP_EVEN;
            read_address_q <= '0;
            read_context_q <= '0;
            read_column_q <= '0;
            read_output_y_q <= '0;
            read_output_first_q <= 1'b0;
            read_output_last_q <= 1'b0;
            read_value_q <= '0;
            predict_valid_q <= 1'b0;
            predict_op_q <= OP_EVEN;
            predict_address_q <= '0;
            predict_context_q <= '0;
            predict_column_q <= '0;
            predict_output_y_q <= '0;
            predict_output_first_q <= 1'b0;
            predict_output_last_q <= 1'b0;
            predict_record_q <= '0;
            predict_input_q <= '0;
            predict_even_q <= '0;
            predict_previous_high_q <= '0;
            out_valid <= 1'b0;
            out_context <= '0;
            out_column <= '0;
            out_y <= '0;
            out_first <= 1'b0;
            out_last <= 1'b0;
            out_high <= '0;
        end else begin
            read_valid_q <= in_valid;
            if (in_valid) begin
                read_op_q <= in_op;
                read_address_q <= in_state_address;
                read_context_q <= in_context;
                read_column_q <= in_column;
                read_output_y_q <= in_output_y;
                read_output_first_q <= in_output_first;
                read_output_last_q <= in_output_last;
                read_value_q <= in_value;
            end

            predict_valid_q <= read_valid_q;
            if (read_valid_q) begin
                predict_op_q <= read_op_q;
                predict_address_q <= read_address_q;
                predict_context_q <= read_context_q;
                predict_column_q <= read_column_q;
                predict_output_y_q <= read_output_y_q;
                predict_output_first_q <= read_output_first_q;
                predict_output_last_q <= read_output_last_q;
                predict_record_q <= state_read_q;
                predict_input_q <= read_value_q;
                predict_even_q <= state_even;
                predict_previous_high_q <= state_previous_high;
            end

            out_valid <= predict_valid_q
                && (predict_op_q == OP_ADVANCE || predict_op_q == OP_TAIL);
            if (predict_valid_q
                    && (predict_op_q == OP_ADVANCE || predict_op_q == OP_TAIL)) begin
                out_context <= predict_context_q;
                out_column <= predict_column_q;
                out_y <= predict_output_y_q;
                out_first <= predict_output_first_q;
                out_last <= predict_output_last_q;
                out_high <= predict_high_q;
            end
        end
    end

    generate
        if (SPLIT_STATE_FIELDS != 0) begin : split_state_fields
            localparam integer RAW_STATE_WIDTH = LANES * RAW_WIDTH;
            localparam integer HIGH_STATE_WIDTH = LANES * HIGH_WIDTH;

            (* syn_ramstyle = "block_ram" *)
            logic [RAW_STATE_WIDTH-1:0]
                even_state_memory [0:STATE_DEPTH-1];
            (* syn_ramstyle = "block_ram" *)
            logic [RAW_STATE_WIDTH-1:0]
                odd_state_memory [0:STATE_DEPTH-1];
            (* syn_ramstyle = "block_ram" *)
            logic [HIGH_STATE_WIDTH-1:0]
                high_state_memory [0:STATE_DEPTH-1];
            logic [RAW_STATE_WIDTH-1:0] even_state_read_q;
            logic [RAW_STATE_WIDTH-1:0] odd_state_read_q;
            logic [HIGH_STATE_WIDTH-1:0] high_state_read_q;
            logic [RAW_STATE_WIDTH-1:0] raw_write_data;
            logic [HIGH_STATE_WIDTH-1:0] high_write_data;

            always_comb begin : assemble_split_state
                state_read_q = '0;
                raw_write_data = '0;
                high_write_data = '0;
                for (integer state_lane = 0;
                        state_lane < LANES; state_lane = state_lane + 1) begin
                    state_read_q[state_lane*LANE_STATE_WIDTH
                            + EVEN_OFFSET +: RAW_WIDTH]
                        = even_state_read_q[state_lane*RAW_WIDTH
                            +: RAW_WIDTH];
                    state_read_q[state_lane*LANE_STATE_WIDTH
                            + ODD_OFFSET +: RAW_WIDTH]
                        = odd_state_read_q[state_lane*RAW_WIDTH
                            +: RAW_WIDTH];
                    state_read_q[state_lane*LANE_STATE_WIDTH
                            + HIGH_OFFSET +: HIGH_WIDTH]
                        = high_state_read_q[state_lane*HIGH_WIDTH
                            +: HIGH_WIDTH];
                    raw_write_data[state_lane*RAW_WIDTH +: RAW_WIDTH]
                        = predict_input_q[state_lane][RAW_WIDTH-1:0];
                    high_write_data[state_lane*HIGH_WIDTH +: HIGH_WIDTH]
                        = predict_high_q[state_lane][HIGH_WIDTH-1:0];
                end
            end

            always_ff @(posedge clk) begin
                if (in_valid)
                    even_state_read_q
                        <= even_state_memory[in_state_address];
                if (predict_valid_q
                        && (predict_op_q == OP_EVEN
                            || predict_op_q == OP_ADVANCE))
                    even_state_memory[predict_address_q] <= raw_write_data;
            end

            always_ff @(posedge clk) begin
                if (in_valid)
                    odd_state_read_q <= odd_state_memory[in_state_address];
                if (predict_valid_q && predict_op_q == OP_ODD)
                    odd_state_memory[predict_address_q] <= raw_write_data;
            end

            always_ff @(posedge clk) begin
                if (in_valid)
                    high_state_read_q
                        <= high_state_memory[in_state_address];
                if (predict_valid_q && predict_op_q == OP_ADVANCE)
                    high_state_memory[predict_address_q] <= high_write_data;
            end
        end else begin : packed_state_fields
            (* syn_ramstyle = "block_ram" *)
            logic [STATE_WIDTH-1:0] state_memory [0:STATE_DEPTH-1];

            always_ff @(posedge clk) begin
                if (in_valid)
                    state_read_q <= state_memory[in_state_address];
                if (unused_packed_write_valid)
                    state_memory[predict_address_q] <= unused_packed_write_record;
            end
        end
    endgenerate

`ifndef SYNTHESIS
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // The diagnostic process has no state to reset.
        end else if (in_valid) begin
            assert (int'({1'b0, in_state_address}) < STATE_DEPTH)
                else $error("vertical_lift_scheduler: state address out of range");
        end
    end
`endif
endmodule

`default_nettype wire
