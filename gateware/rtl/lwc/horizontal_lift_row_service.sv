`default_nettype none

// One reversible CDF 5/3 horizontal lifting datapath shared by H2 through H5.
//
// H2, H3, and H4 rows are completed into pair RAM before they are queued.
// H2 uses alternating row banks because its next row begins while the prior
// row is being read.  H3 and H4 have enough vertical-row separation to use a
// single construction bank; explicit busy checks make that scheduling
// assumption fail closed.  H5 is generated from the low output of H4 and is
// always serviced immediately after its parent H4 row.
module horizontal_lift_row_service #(
    parameter integer DATA_WIDTH = 14,
    parameter integer COORD_WIDTH = 10,
    parameter integer H2_PAIR_COUNT = 320,
    parameter integer H3_PAIR_COUNT = 160,
    parameter integer H4_PAIR_COUNT = 80,
    parameter integer H5_PAIR_COUNT = 40,
    // Opt in only when every stored input, including the H4 low output fed
    // into H5, fits RAW_WIDTH. Arithmetic and public outputs retain DATA_WIDTH.
    parameter integer RAW_WIDTH = DATA_WIDTH
) (
    input  wire                                     clk,
    input  wire                                     rst_n,
    input  wire                                     clear,

    input  wire                                     h2_pair_valid,
    input  wire [$clog2(H2_PAIR_COUNT)-1:0]         h2_pair_index,
    input  wire signed [DATA_WIDTH-1:0]             h2_pair_even,
    input  wire signed [DATA_WIDTH-1:0]             h2_pair_odd,
    input  wire [COORD_WIDTH-1:0]                   h2_pair_row,
    input  wire                                     h2_sink_ready,

    input  wire                                     h3_pair_valid,
    input  wire [$clog2(H3_PAIR_COUNT)-1:0]         h3_pair_index,
    input  wire signed [DATA_WIDTH-1:0]             h3_pair_even,
    input  wire signed [DATA_WIDTH-1:0]             h3_pair_odd,
    input  wire [COORD_WIDTH-1:0]                   h3_pair_row,
    input  wire                                     h3_sink_ready,

    input  wire                                     h4_pair_valid,
    input  wire [$clog2(H4_PAIR_COUNT)-1:0]         h4_pair_index,
    input  wire signed [DATA_WIDTH-1:0]             h4_pair_even,
    input  wire signed [DATA_WIDTH-1:0]             h4_pair_odd,
    input  wire [COORD_WIDTH-1:0]                   h4_pair_row,

    output logic                                    out_valid,
    output logic [1:0]                              out_kind,
    output logic                                    out_first,
    output logic                                    out_last,
    output logic signed [DATA_WIDTH-1:0]            out_low,
    output logic signed [DATA_WIDTH-1:0]            out_high,
    output logic [COORD_WIDTH-1:0]                  out_x,
    output logic [COORD_WIDTH-1:0]                  out_row,
    output logic                                    out_h4_low_pair_valid,
    output logic                                    out_h4_low_pair_first,
    output logic                                    out_h4_low_pair_last,
    output logic [COORD_WIDTH-1:0]                  out_h4_low_pair_x,
    output logic [COORD_WIDTH-1:0]                  out_h4_low_pair_y,
    output logic signed [1:0][DATA_WIDTH-1:0]       out_h4_low_pair_value,
    output logic                                    idle,
    output logic                                    protocol_error
);
    localparam logic [1:0] KIND_H2 = 2'd0;
    localparam logic [1:0] KIND_H3 = 2'd1;
    localparam logic [1:0] KIND_H4 = 2'd2;
    localparam logic [1:0] KIND_H5 = 2'd3;

    localparam logic [1:0] SERVICE_IDLE = 2'd0;
    localparam logic [1:0] SERVICE_FEED = 2'd1;
    localparam logic [1:0] SERVICE_DRAIN = 2'd2;

    localparam integer H2_BANK_COUNT = 2 * H2_PAIR_COUNT;
    localparam integer UPPER_PAIR_COUNT = H2_BANK_COUNT + H3_PAIR_COUNT;
    localparam integer LOWER_PAIR_COUNT =
        1 << $clog2(H4_PAIR_COUNT + H5_PAIR_COUNT);
    // Each raw pair uses 2*RAW_WIDTH bits of the RAM10 row store.
    // Expose that physical depth so each logical region can occupy sparse,
    // power-of-two address codes without constant-offset adders.
    localparam integer PAIR_MEMORY_COUNT =
        1 << $clog2(UPPER_PAIR_COUNT + LOWER_PAIR_COUNT);
    localparam integer UPPER_ADDRESS_WIDTH = $clog2(UPPER_PAIR_COUNT);
    localparam integer LOWER_ADDRESS_WIDTH = $clog2(LOWER_PAIR_COUNT);
    localparam integer PAIR_ADDRESS_WIDTH = $clog2(PAIR_MEMORY_COUNT);
    localparam integer MAX_PAIR_COUNT = H2_PAIR_COUNT;
    localparam integer FEED_INDEX_WIDTH = $clog2(MAX_PAIR_COUNT);
    localparam integer H2_INDEX_WIDTH = $clog2(H2_PAIR_COUNT);
    localparam integer H3_INDEX_WIDTH = $clog2(H3_PAIR_COUNT);
    localparam integer H4_INDEX_WIDTH = $clog2(H4_PAIR_COUNT);

    // The fixed 1280x800 cadence has a proven peak occupancy of two jobs.
`ifndef SYNTHESIS
    localparam integer JOB_COUNT_WIDTH = 2;
`endif

    (* syn_ramstyle = "block_ram" *)
    logic signed [2*RAW_WIDTH-1:0] pair_memory [0:PAIR_MEMORY_COUNT-1];

    logic upper_write_enable;
    logic [UPPER_ADDRESS_WIDTH-1:0] upper_write_address;
    logic signed [2*RAW_WIDTH-1:0] upper_write_data;
    logic lower_write_enable;
    logic [LOWER_ADDRESS_WIDTH-1:0] lower_write_address;
    logic signed [2*RAW_WIDTH-1:0] lower_write_data;

    logic [UPPER_ADDRESS_WIDTH-1:0] upper_read_address;
    logic [LOWER_ADDRESS_WIDTH-1:0] lower_read_address;
    logic pair_write_enable;
    logic [PAIR_ADDRESS_WIDTH-1:0] pair_write_address;
    logic signed [2*RAW_WIDTH-1:0] pair_write_data;
    logic pair_read_enable;
    logic [PAIR_ADDRESS_WIDTH-1:0] pair_read_address;
    logic signed [2*RAW_WIDTH-1:0] pair_read_q;
    logic pending_write_valid;
    logic [PAIR_ADDRESS_WIDTH-1:0] pending_write_address;
    logic signed [2*RAW_WIDTH-1:0] pending_write_data;
    logic pending_write_valid_next;
    logic [PAIR_ADDRESS_WIDTH-1:0] pending_write_address_next;
    logic signed [2*RAW_WIDTH-1:0] pending_write_data_next;

    // The proven queue occupancy is two. Keep explicit head/tail registers so
    // synthesis does not build three separate dynamically indexed memories
    // for kind, H2 bank, and row metadata. Upper jobs precede H4 when both
    // complete together, matching the original circular queue ordering.
    logic [1:0] job_head_kind;
    logic job_head_h2_bank;
    logic [COORD_WIDTH-1:0] job_head_row;
    logic [1:0] job_tail_kind;
    logic job_tail_h2_bank;
    logic [COORD_WIDTH-1:0] job_tail_row;
    logic [1:0] job_count;
    logic [1:0] job_push_count;
    logic job_overflow;
    logic job_push_upper;
    logic job_push_h4;
    logic [1:0] job_push_upper_kind;
    logic job_push_upper_bank;
    logic [COORD_WIDTH-1:0] job_push_upper_row;
    logic [1:0] job_push_first_kind;
    logic job_push_first_bank;
    logic [COORD_WIDTH-1:0] job_push_first_row;
    logic [1:0] job_push_second_kind;
    logic job_push_second_bank;
    logic [COORD_WIDTH-1:0] job_push_second_row;
    logic job_pop;

    logic h2_write_bank;
    logic [1:0] h2_bank_busy;
    logic h3_bank_busy;
    logic h4_bank_busy;

    logic [1:0] service_state;
    logic [1:0] service_kind;
    logic service_h2_bank;
    logic [COORD_WIDTH-1:0] service_row;
    logic [FEED_INDEX_WIDTH-1:0] service_feed_index;
    logic service_read_request;
    logic service_read_valid_q;
    logic service_read_first_q;
    logic service_read_last_q;
    logic signed [2*RAW_WIDTH-1:0] service_read_data;
    logic service_in_valid;
    logic service_in_first;
    logic service_in_last;
    logic signed [DATA_WIDTH-1:0] service_in_even;
    logic signed [DATA_WIDTH-1:0] service_in_odd;
    logic service_out_valid;
    logic service_out_first;
    logic service_out_last;
    logic signed [DATA_WIDTH-1:0] service_out_low;
    logic signed [DATA_WIDTH-1:0] service_out_high;
    logic [COORD_WIDTH-1:0] service_out_x;
`ifndef SYNTHESIS
    logic h5_pending_valid;
`endif
    logic signed [DATA_WIDTH-1:0] h5_pending_even;
    logic h5_pair_write;

    function automatic logic [FEED_INDEX_WIDTH-1:0] pair_count_for_kind(
        input logic [1:0] kind
    );
        case (kind)
            KIND_H2: pair_count_for_kind = FEED_INDEX_WIDTH'(
                H2_PAIR_COUNT
            );
            KIND_H3: pair_count_for_kind = FEED_INDEX_WIDTH'(
                H3_PAIR_COUNT
            );
            KIND_H4: pair_count_for_kind = FEED_INDEX_WIDTH'(
                H4_PAIR_COUNT
            );
            default: pair_count_for_kind = FEED_INDEX_WIDTH'(
                H5_PAIR_COUNT
            );
        endcase
    endfunction

    // H4 uses addresses 0..79. Map H5's 40 entries into the otherwise empty
    // 80..127 range using concatenation, avoiding a seven-bit base adder.
    function automatic logic [LOWER_ADDRESS_WIDTH-1:0] lower_pair_address(
        input logic is_h5,
        input logic [LOWER_ADDRESS_WIDTH-1:0] index
    );
        if (!is_h5)
            lower_pair_address = index;
        else if (!index[5])
            lower_pair_address = {2'b11,index[4:0]};
        else
            lower_pair_address = {4'b1010,index[2:0]};
    endfunction

    // H4/H5 use 120 of the 128 lower logical codes.  Prefixing those codes
    // with 3'b111 places them at 896..1023, disjoint from both H2 banks and
    // H3, with no base-address addition.
    function automatic logic [PAIR_ADDRESS_WIDTH-1:0] physical_lower_address(
        input logic [LOWER_ADDRESS_WIDTH-1:0] address
    );
        physical_lower_address = {3'b111,address};
    endfunction

    // Bank zero H2 occupies 0..319 and bank one H2 occupies 512..831.
    // Put H3 in unused bank-zero codes 320..511.  The valid H3 indices are
    // 0..159, so this permutation is injective and uses only wiring/mux bits.
    function automatic logic [UPPER_ADDRESS_WIDTH-1:0] upper_pair_address(
        input logic is_h3,
        input logic h2_bank,
        input logic [FEED_INDEX_WIDTH-1:0] index
    );
        if (!is_h3)
            upper_pair_address = {h2_bank,index[H2_INDEX_WIDTH-1:0]};
        else if (!index[7])
            upper_pair_address = {1'b0,2'b11,index[6:0]};
        else
            upper_pair_address = {1'b0,4'b1010,index[4:0]};
    endfunction

    always_comb begin
        job_push_upper = (h2_pair_valid
                && h2_pair_index + 1'b1
                    == H2_INDEX_WIDTH'(H2_PAIR_COUNT))
            || (h3_pair_valid
                && h3_pair_index + 1'b1
                    == H3_INDEX_WIDTH'(H3_PAIR_COUNT));
        job_push_upper_kind = h2_pair_valid ? KIND_H2 : KIND_H3;
        job_push_upper_bank = h2_write_bank;
        job_push_upper_row = h2_pair_valid ? h2_pair_row : h3_pair_row;
        job_push_h4 = h4_pair_valid
            && h4_pair_index + 1'b1 == H4_INDEX_WIDTH'(H4_PAIR_COUNT);
        job_push_count = {1'b0, job_push_upper} + {1'b0, job_push_h4};
        job_pop = service_state == SERVICE_IDLE && job_count != 0
            && (job_head_kind != KIND_H2 || h2_sink_ready)
            && (job_head_kind != KIND_H3 || h3_sink_ready);
        job_push_first_kind = job_push_upper ? job_push_upper_kind : KIND_H4;
        job_push_first_bank = job_push_upper ? job_push_upper_bank : 1'b0;
        job_push_first_row = job_push_upper ? job_push_upper_row : h4_pair_row;
        job_push_second_kind = KIND_H4;
        job_push_second_bank = 1'b0;
        job_push_second_row = h4_pair_row;
        case ({job_pop,job_push_count})
            3'b000: job_overflow = job_count > 2;
            3'b001: job_overflow = job_count > 1;
            3'b010: job_overflow = job_count != 0;
            3'b100: job_overflow = 1'b0;
            3'b101: job_overflow = job_count > 2;
            3'b110: job_overflow = job_count > 1;
            default: job_overflow = 1'b1;
        endcase

        upper_write_enable = h2_pair_valid || h3_pair_valid;
        if (h2_pair_valid) begin
            upper_write_address = upper_pair_address(
                1'b0, h2_write_bank, FEED_INDEX_WIDTH'(h2_pair_index)
            );
            upper_write_data = {h2_pair_odd[RAW_WIDTH-1:0], h2_pair_even[RAW_WIDTH-1:0]};
        end else begin
            upper_write_address = upper_pair_address(
                1'b1, 1'b0, FEED_INDEX_WIDTH'(h3_pair_index)
            );
            upper_write_data = {h3_pair_odd[RAW_WIDTH-1:0], h3_pair_even[RAW_WIDTH-1:0]};
        end

        h5_pair_write = service_out_valid && service_kind == KIND_H4
            && service_out_x[0];
        lower_write_enable = h4_pair_valid || h5_pair_write;
        if (h5_pair_write) begin
            lower_write_address = lower_pair_address(
                1'b1,service_out_x[LOWER_ADDRESS_WIDTH:1]
            );
            lower_write_data = {service_out_low[RAW_WIDTH-1:0], h5_pending_even[RAW_WIDTH-1:0]};
        end else begin
            lower_write_address = LOWER_ADDRESS_WIDTH'(h4_pair_index);
            lower_write_data = {h4_pair_odd[RAW_WIDTH-1:0], h4_pair_even[RAW_WIDTH-1:0]};
        end

        service_read_request = service_state == SERVICE_FEED;
        upper_read_address = upper_pair_address(
            service_kind == KIND_H3,
            service_h2_bank,
            service_feed_index
        );
        lower_read_address = lower_pair_address(
            service_kind == KIND_H5,
            service_feed_index[LOWER_ADDRESS_WIDTH-1:0]
        );

        pair_read_enable = service_read_request;
        if (service_kind == KIND_H2 || service_kind == KIND_H3)
            pair_read_address = PAIR_ADDRESS_WIDTH'(upper_read_address);
        else
            pair_read_address = physical_lower_address(lower_read_address);
        service_read_data = pair_read_q;

        // The Ti60 RAM10 has one independent write port.  The upper and lower
        // row builders occasionally finish work on the same clock, so retain
        // one request while the other uses the physical port.  A complete
        // 1280x800 schedule never accumulates more than one retained write.
        pair_write_enable = 1'b0;
        pair_write_address = '0;
        pair_write_data = '0;
        pending_write_valid_next = pending_write_valid;
        pending_write_address_next = pending_write_address;
        pending_write_data_next = pending_write_data;
        if (pending_write_valid) begin
            pair_write_enable = 1'b1;
            pair_write_address = pending_write_address;
            pair_write_data = pending_write_data;
            if (lower_write_enable) begin
                pending_write_valid_next = 1'b1;
                pending_write_address_next =
                    physical_lower_address(lower_write_address);
                pending_write_data_next = lower_write_data;
            end else if (upper_write_enable) begin
                pending_write_valid_next = 1'b1;
                pending_write_address_next = PAIR_ADDRESS_WIDTH'(
                    upper_write_address
                );
                pending_write_data_next = upper_write_data;
            end else begin
                pending_write_valid_next = 1'b0;
            end
        end else if (lower_write_enable) begin
            pair_write_enable = 1'b1;
            pair_write_address = physical_lower_address(lower_write_address);
            pair_write_data = lower_write_data;
            if (upper_write_enable) begin
                pending_write_valid_next = 1'b1;
                pending_write_address_next = PAIR_ADDRESS_WIDTH'(
                    upper_write_address
                );
                pending_write_data_next = upper_write_data;
            end
        end else if (upper_write_enable) begin
            pair_write_enable = 1'b1;
            pair_write_address = PAIR_ADDRESS_WIDTH'(upper_write_address);
            pair_write_data = upper_write_data;
        end
        service_in_valid = service_read_valid_q;
        service_in_first = service_read_first_q;
        service_in_last = service_read_last_q;
        service_in_even = DATA_WIDTH'($signed(service_read_data[RAW_WIDTH-1:0]));
        service_in_odd = DATA_WIDTH'($signed(service_read_data[2*RAW_WIDTH-1:RAW_WIDTH]));

        out_valid = service_out_valid;
        out_kind = service_kind;
        out_first = service_out_first;
        out_last = service_out_last;
        out_low = service_out_low;
        out_high = service_out_high;
        out_x = service_out_x;
        out_row = service_row;
        out_h4_low_pair_valid = h5_pair_write;
        out_h4_low_pair_first = h5_pair_write && service_out_x == 1;
        out_h4_low_pair_last = h5_pair_write
            && service_out_x + 1'b1
                == COORD_WIDTH'(H4_PAIR_COUNT);
        out_h4_low_pair_x = COORD_WIDTH'(service_out_x >> 1);
        out_h4_low_pair_y = service_row;
        out_h4_low_pair_value[0] = h5_pending_even;
        out_h4_low_pair_value[1] = service_out_low;
        idle = service_state == SERVICE_IDLE && job_count == 0;
    end

    always_ff @(posedge clk) begin
        if (pair_write_enable)
            pair_memory[pair_write_address] <= pair_write_data;
        if (pair_read_enable)
            pair_read_q <= pair_memory[pair_read_address];
    end

    horizontal_lift_stage #(.DATA_WIDTH(DATA_WIDTH)) shared_hlift (
        .clk, .rst_n,
        .in_valid(service_in_valid), .in_first(service_in_first),
        .in_last(service_in_last), .in_even(service_in_even),
        .in_odd(service_in_odd), .out_valid(service_out_valid),
        .out_first(service_out_first), .out_last(service_out_last),
        .out_low(service_out_low), .out_high(service_out_high)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            job_count <= '0;
            h2_write_bank <= 1'b0;
            h2_bank_busy <= '0;
            h3_bank_busy <= 1'b0;
            h4_bank_busy <= 1'b0;
            service_state <= SERVICE_IDLE;
            service_kind <= KIND_H2;
            service_h2_bank <= 1'b0;
            service_row <= '0;
            service_feed_index <= '0;
            service_read_valid_q <= 1'b0;
            service_read_first_q <= 1'b0;
            service_read_last_q <= 1'b0;
            service_out_x <= '0;
            h5_pending_even <= '0;
            pending_write_valid <= 1'b0;
            pending_write_address <= '0;
            pending_write_data <= '0;
            protocol_error <= 1'b0;
        end else if (clear) begin
            job_count <= '0;
            h2_write_bank <= 1'b0;
            h2_bank_busy <= '0;
            h3_bank_busy <= 1'b0;
            h4_bank_busy <= 1'b0;
            service_state <= SERVICE_IDLE;
            service_feed_index <= '0;
            service_read_valid_q <= 1'b0;
            service_out_x <= '0;
            pending_write_valid <= 1'b0;
            protocol_error <= 1'b0;
        end else begin
            pending_write_valid <= pending_write_valid_next;
            pending_write_address <= pending_write_address_next;
            pending_write_data <= pending_write_data_next;
            service_read_valid_q <= service_read_request;
            if (service_read_request) begin
                service_read_first_q <= service_feed_index == 0;
                service_read_last_q <= service_feed_index + 1'b1
                    == pair_count_for_kind(service_kind);
            end

            if (h2_pair_valid && h2_pair_index == 0 && h2_bank_busy[h2_write_bank])
                protocol_error <= 1'b1;
            if (h3_pair_valid && h3_pair_index == 0 && h3_bank_busy)
                protocol_error <= 1'b1;
            if (h4_pair_valid && h4_pair_index == 0 && h4_bank_busy)
                protocol_error <= 1'b1;
            if (h2_pair_valid && h3_pair_valid)
                protocol_error <= 1'b1;
            if (h4_pair_valid && h5_pair_write)
                protocol_error <= 1'b1;
            if (pending_write_valid && upper_write_enable && lower_write_enable)
                protocol_error <= 1'b1;
            if (job_overflow)
                protocol_error <= 1'b1;

            case ({job_pop,job_push_count})
                3'b001: begin
                    if (job_count == 0) begin
                        job_head_kind <= job_push_first_kind;
                        job_head_h2_bank <= job_push_first_bank;
                        job_head_row <= job_push_first_row;
                    end else begin
                        job_tail_kind <= job_push_first_kind;
                        job_tail_h2_bank <= job_push_first_bank;
                        job_tail_row <= job_push_first_row;
                    end
                    job_count <= job_count + 1'b1;
                end
                3'b010: begin
                    job_head_kind <= job_push_first_kind;
                    job_head_h2_bank <= job_push_first_bank;
                    job_head_row <= job_push_first_row;
                    job_tail_kind <= job_push_second_kind;
                    job_tail_h2_bank <= job_push_second_bank;
                    job_tail_row <= job_push_second_row;
                    job_count <= job_count + 2'd2;
                end
                3'b100: begin
                    if (job_count == 2) begin
                        job_head_kind <= job_tail_kind;
                        job_head_h2_bank <= job_tail_h2_bank;
                        job_head_row <= job_tail_row;
                    end
                    job_count <= job_count - 1'b1;
                end
                3'b101: begin
                    if (job_count == 1) begin
                        job_head_kind <= job_push_first_kind;
                        job_head_h2_bank <= job_push_first_bank;
                        job_head_row <= job_push_first_row;
                    end else begin
                        job_head_kind <= job_tail_kind;
                        job_head_h2_bank <= job_tail_h2_bank;
                        job_head_row <= job_tail_row;
                        job_tail_kind <= job_push_first_kind;
                        job_tail_h2_bank <= job_push_first_bank;
                        job_tail_row <= job_push_first_row;
                    end
                end
                3'b110: begin
                    job_head_kind <= job_push_first_kind;
                    job_head_h2_bank <= job_push_first_bank;
                    job_head_row <= job_push_first_row;
                    job_tail_kind <= job_push_second_kind;
                    job_tail_h2_bank <= job_push_second_bank;
                    job_tail_row <= job_push_second_row;
                    job_count <= job_count + 1'b1;
                end
                default: begin
                end
            endcase

            if (h2_pair_valid
                    && h2_pair_index + 1'b1
                        == H2_INDEX_WIDTH'(H2_PAIR_COUNT)) begin
                h2_bank_busy[h2_write_bank] <= 1'b1;
                h2_write_bank <= ~h2_write_bank;
            end
            if (h3_pair_valid
                    && h3_pair_index + 1'b1
                        == H3_INDEX_WIDTH'(H3_PAIR_COUNT))
                h3_bank_busy <= 1'b1;
            if (h4_pair_valid
                    && h4_pair_index + 1'b1
                        == H4_INDEX_WIDTH'(H4_PAIR_COUNT))
                h4_bank_busy <= 1'b1;

            if (service_state == SERVICE_IDLE) begin
                if (job_pop) begin
                    service_kind <= job_head_kind;
                    service_h2_bank <= job_head_h2_bank;
                    service_row <= job_head_row;
                    service_feed_index <= '0;
                    service_out_x <= '0;
                    service_state <= SERVICE_FEED;
                end
            end else if (service_state == SERVICE_FEED) begin
                if (service_read_request) begin
                    if (service_feed_index + 1'b1
                            == pair_count_for_kind(service_kind)) begin
                        service_feed_index <= '0;
                        service_state <= SERVICE_DRAIN;
                        case (service_kind)
                            KIND_H2: h2_bank_busy[service_h2_bank] <= 1'b0;
                            KIND_H3: h3_bank_busy <= 1'b0;
                            KIND_H4: h4_bank_busy <= 1'b0;
                            default: begin
                            end
                        endcase
                    end else begin
                        service_feed_index <= service_feed_index + 1'b1;
                    end
                end
            end

            if (service_out_valid) begin
                if (service_kind == KIND_H4) begin
                    if (!service_out_x[0]) begin
                        h5_pending_even <= service_out_low;
                    end
                end

                if (service_out_last) begin
                    service_out_x <= '0;
                    if (service_kind == KIND_H4) begin
                        service_kind <= KIND_H5;
                        service_feed_index <= '0;
                        service_state <= SERVICE_FEED;
                    end else begin
                        service_state <= SERVICE_IDLE;
                    end
                end else begin
                    service_out_x <= service_out_x + 1'b1;
                end
            end
        end
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h5_pending_valid <= 1'b0;
        end else if (clear) begin
            h5_pending_valid <= 1'b0;
        end else begin
            assert (!(h2_pair_valid && h3_pair_valid))
                else $error("horizontal_lift_row_service: upper RAM write collision");
            assert (!(h4_pair_valid && h5_pair_write))
                else $error("horizontal_lift_row_service: lower RAM write collision");
            assert (!(pending_write_valid && upper_write_enable
                    && lower_write_enable))
                else $error("horizontal_lift_row_service: merged RAM write overflow");
            if (h5_pair_write)
                assert (h5_pending_valid)
                    else $error("horizontal_lift_row_service: H5 gearbox underflow");
            assert (job_count <= JOB_COUNT_WIDTH'(2))
                else $error("horizontal_lift_row_service: job FIFO overflow");

            if (service_out_valid && service_kind == KIND_H4)
                h5_pending_valid <= !service_out_x[0];
        end
    end
`endif
endmodule

`default_nettype wire
