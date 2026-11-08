`default_nettype none

// Each queue writes one 10-bit coefficient at a time and reads two adjacent
// coefficients as one 20-bit pair. The asymmetric 10-bit-write/20-bit-read
// shape maps directly to native RAM10 widths without a first-half register.
// Band zero stores 256 pairs, bands one through eight store 512 pairs, and
// bands nine through eleven store 1024 pairs. A compact family schedule makes
// replay independent of the wall-clock cycle on which each band completes.
module coefficient_pair_queues #(
    parameter string GROUP_FAMILY_SCHEDULE_BIN =
        "../../../gateware/rtl/lwc/raw_group_family_schedule.bin",
    // The default keeps the cadence-independent public queue profile.  The
    // production encoder selects the compact profile, whose five payload
    // banks are proved for the fixed 640-active/256-blank transform trace.
    parameter bit COMPACT_PRODUCTION_PROFILE = 1'b0
) (
    input  wire                              clk,
    input  wire                              rst,
    input  wire                              clear_frame,
    input  wire [11:0]                       in_valid,
    input  wire signed [11:0][7:0]           in_q,
    input  wire [11:0]                       in_correction,
    input  wire [11:0]                       in_residual_class,

    output logic                             pair_valid,
    input  wire                              pair_ready,
    output logic                             pair_group_start,
    output logic                             pair_group_last,
    output logic [3:0]                       pair_band,
    output logic signed [7:0]                pair_q0,
    output logic signed [7:0]                pair_q1,
    output logic [1:0]                       pair_correction,
    output logic [1:0]                       pair_residual_class,
    output logic                             protocol_error
);
    logic [9:0] ram0 [0:511];
    logic [9:0] ram1 [0:1023];
    logic [9:0] ram2 [0:1023];
    logic [9:0] ram3 [0:1023];
    logic [9:0] ram4 [0:1023];
    logic [9:0] ram5 [0:1023];
    logic [9:0] ram6 [0:1023];
    logic [9:0] ram7 [0:1023];
    logic [9:0] ram8 [0:1023];
    logic [9:0] ram9 [0:2047];
    logic [9:0] ram10 [0:2047];
    logic [9:0] ram11 [0:2047];
    // Compact mode shares five native 1024x10 stores.  Like the general
    // queues, each store has a ten-bit coefficient write and an adjacent
    // twenty-bit pair read.  The fixed transform's raw-valid color graph
    // guarantees at most one coefficient write per physical bank and clock.
    logic [9:0] compact_ram0 [0:1023];
    logic [9:0] compact_ram1 [0:1023];
    logic [9:0] compact_ram2 [0:1023];
    logic [9:0] compact_ram3 [0:1023];
    logic [9:0] compact_ram4 [0:1023];
    // The canonical order has a startup block, one steady block repeated 48
    // times, and a tail block. Collapsing the seven simultaneous-band families
    // reduces those templates to 1,605 three-bit tokens. Family tokenization
    // crosses the first block boundary, so their expanded lengths are 1,281,
    // 1,280 and 1,279 groups: exactly 64,000 in total. The tokens fit in the
    // RAM10 formerly used for descriptors.
    (* syn_romstyle = "block_rom" *)
    logic [2:0] schedule_rom [0:1604];
    logic [2:0] schedule_rom_q;

    logic half_full [0:11];
    logic [9:0] write_pointer [0:11];
    // Replay consumes complete eight-pair groups.  Store only the group base
    // per band; the active group's low three address bits are already held in
    // issue_pair_index.
    logic [6:0] read_group_pointer [0:11];
    logic [11:0] write_wrap;
    logic [11:0] read_wrap;
`ifndef SYNTHESIS
    logic [9:0] read_pair_pointer [0:11];
    logic [11:0] pair_full;
    logic [11:0] compact_exact_pair_full;
    logic [11:0] compact_exact_pair_empty;
    logic [11:0] pair_write;
`endif
    logic [11:0] pair_empty;
    logic [9:0] coefficient_write_word [0:11];
    logic [11:0] compact_capacity_guard_mask;
    logic compact_capacity_guard;
    logic [11:0] compact_pair_grant;
    logic compact_write_conflict;
    logic [4:0] compact_ram_write_enable;
    logic [4:0] compact_ram_read_enable;
    logic [4:0][9:0] compact_ram_write_address;
    logic [4:0][8:0] compact_ram_read_address;
    logic [4:0][9:0] compact_ram_write_data;
    logic [4:0][19:0] compact_ram_read_data;

    logic [5:0] schedule_block;
    logic [10:0] schedule_token_address;
    logic schedule_rom_read_enable;
    logic schedule_rom_read_pending;
    wire schedule_family_valid;
    logic [2:0] schedule_family;
    logic [1:0] schedule_family_phase;
    logic [3:0] schedule_band;
    logic schedule_family_last_phase;
    logic schedule_advance;
    logic schedule_done;

    logic [3:0] issue_band;
    logic [2:0] issue_pair_index;
    logic pair_read_attempt;
    logic pair_read_issue;
    logic pair_read_start_group;
    logic [3:0] pair_read_band;
    logic [2:0] pair_read_index;
    logic pair_read_pending;
    logic [3:0] pair_pending_band;
    logic [2:0] pair_pending_index;
    logic [19:0] pair_response_word;
    logic output_stage_ready;
    logic pair_fire;

    logic [19:0] ram0_read_data;
    logic [19:0] ram1_read_data;
    logic [19:0] ram2_read_data;
    logic [19:0] ram3_read_data;
    logic [19:0] ram4_read_data;
    logic [19:0] ram5_read_data;
    logic [19:0] ram6_read_data;
    logic [19:0] ram7_read_data;
    logic [19:0] ram8_read_data;
    logic [19:0] ram9_read_data;
    logic [19:0] ram10_read_data;
    logic [19:0] ram11_read_data;
    logic ram0_read_enable;
    logic [7:0] ram0_read_address;

    // The hardware guard compares only the wrap phase and upper two address
    // bits. It becomes sticky in the final quarter of each queue, before an
    // exact full condition, while exact pair-pointer checks remain in
    // simulation.
    logic [11:0] capacity_guard_mask;
    logic capacity_guard;

    localparam logic [10:0] STARTUP_TOKEN_LAST = 11'd500;
    localparam logic [10:0] STEADY_TOKEN_BASE = 11'd501;
    localparam logic [10:0] STEADY_TOKEN_LAST = 11'd1035;
    localparam logic [10:0] TAIL_TOKEN_BASE = 11'd1036;
    localparam logic [10:0] TAIL_TOKEN_LAST = 11'd1604;

    initial begin
        $readmemb(GROUP_FAMILY_SCHEDULE_BIN, schedule_rom);
    end

    assign schedule_family_valid = schedule_family_phase != 3;

    function automatic logic [6:0] next_read_group_pointer(
        input logic [3:0] band,
        input logic [6:0] pointer
    );
        if (band == 0 && pointer == 7'd31)
            next_read_group_pointer = 0;
        else if (band < 9 && pointer == 7'd63)
            next_read_group_pointer = 0;
        else if (pointer == 7'd127)
            next_read_group_pointer = 0;
        else
            next_read_group_pointer = pointer + 1'b1;
    endfunction

    // Every region is group-aligned.  The deliberately rounded capacities
    // leave at least sixteen pair positions above the conservative fixed-trace
    // peak and make each bank no larger than 512 logical pairs.
    function automatic logic [9:0] compact_pair_capacity(
        input logic [3:0] band
    );
        case (band)
            0, 1, 2: compact_pair_capacity = 10'd64;
            3, 4, 5: compact_pair_capacity = 10'd192;
            6, 7, 8: compact_pair_capacity = 10'd176;
            default: compact_pair_capacity = 10'd336;
        endcase
    endfunction

    function automatic logic [6:0] compact_group_capacity(
        input logic [3:0] band
    );
        case (band)
            0, 1, 2: compact_group_capacity = 7'd8;
            3, 4, 5: compact_group_capacity = 7'd24;
            6, 7, 8: compact_group_capacity = 7'd22;
            default: compact_group_capacity = 7'd42;
        endcase
    endfunction

    function automatic logic [2:0] compact_bank_for(
        input logic [3:0] band
    );
        case (band)
            6, 9: compact_bank_for = 0;
            8, 10: compact_bank_for = 1;
            7, 11: compact_bank_for = 2;
            0, 2, 3, 4: compact_bank_for = 3;
            default: compact_bank_for = 4; // bands 1 and 5
        endcase
    endfunction

    function automatic logic [8:0] compact_base_for(
        input logic [3:0] band
    );
        case (band)
            9, 10, 11: compact_base_for = 9'd176;
            2: compact_base_for = 9'd64;
            3: compact_base_for = 9'd128;
            4: compact_base_for = 9'd320;
            5: compact_base_for = 9'd64;
            default: compact_base_for = 0;
        endcase
    endfunction

    always_comb begin : input_pairing
        integer band;
`ifndef SYNTHESIS
        pair_write = '0;
        pair_full = '0;
        compact_exact_pair_full = '0;
        compact_exact_pair_empty = '0;
`endif
        for (band = 0; band < 12; band = band + 1) begin
`ifndef SYNTHESIS
            read_pair_pointer[band]
                = {read_group_pointer[band],3'b000};
            if (issue_pair_index != 0 && issue_band == 4'(band))
                read_pair_pointer[band][2:0] = issue_pair_index;
            compact_exact_pair_full[band]
                = write_pointer[band] == read_pair_pointer[band]
                    && write_wrap[band] != read_wrap[band];
            compact_exact_pair_empty[band]
                = write_pointer[band] == read_pair_pointer[band]
                    && write_wrap[band] == read_wrap[band];
            pair_write[band] = in_valid[band] && half_full[band];
`endif
            coefficient_write_word[band] = {in_residual_class[band],
                in_correction[band],in_q[band]};
            // The write pointer's group field advances only after the eighth
            // pair has been stored. Ignore its intra-group bits so a partial
            // group cannot become visible to the deterministic reader.
            if (COMPACT_PRODUCTION_PROFILE)
                pair_empty[band]
                    = write_pointer[band][9:3]
                        == read_group_pointer[band]
                    && write_wrap[band] == read_wrap[band];
            else if (band == 0)
                pair_empty[band]
                    = write_pointer[band][7:3]
                        == read_group_pointer[band][4:0]
                    && write_wrap[band] == read_wrap[band];
            else if (band < 9)
                pair_empty[band]
                    = write_pointer[band][8:3]
                        == read_group_pointer[band][5:0]
                    && write_wrap[band] == read_wrap[band];
            else
                pair_empty[band]
                    = write_pointer[band][9:3]
                        == read_group_pointer[band]
                    && write_wrap[band] == read_wrap[band];
`ifndef SYNTHESIS
            if (!COMPACT_PRODUCTION_PROFILE && band == 0)
                pair_full[band]
                    = write_pointer[band][7:0]
                        == read_pair_pointer[band][7:0]
                    && write_wrap[band] != read_wrap[band];
            else if (!COMPACT_PRODUCTION_PROFILE && band < 9)
                pair_full[band]
                    = write_pointer[band][8:0]
                        == read_pair_pointer[band][8:0]
                    && write_wrap[band] != read_wrap[band];
            else if (!COMPACT_PRODUCTION_PROFILE)
                pair_full[band]
                    = write_pointer[band] == read_pair_pointer[band]
                    && write_wrap[band] != read_wrap[band];
`endif
        end
    end

    always_comb begin : compact_capacity_monitor
        integer band;
        compact_capacity_guard_mask = '0;
        for (band = 0; band < 12; band = band + 1)
            compact_capacity_guard_mask[band]
                = write_pointer[band][9:3]
                    == read_group_pointer[band]
                && write_wrap[band] != read_wrap[band];
        compact_capacity_guard = |compact_capacity_guard_mask;
    end

    // The fixed transform raw-valid graph permits at most one coefficient
    // write to each bank.  Priority grants make even an out-of-contract input
    // physically safe: conflicting or full writes are dropped and diagnosed
    // before a RAM port is enabled.
    always_comb begin : compact_write_control
        logic [11:0] request;
        request = in_valid & ~compact_capacity_guard_mask;
        compact_pair_grant = '0;

        compact_pair_grant[9] = request[9];
        compact_pair_grant[6] = request[6] && !request[9];
        compact_pair_grant[10] = request[10];
        compact_pair_grant[8] = request[8] && !request[10];
        compact_pair_grant[11] = request[11];
        compact_pair_grant[7] = request[7] && !request[11];
        compact_pair_grant[4] = request[4];
        compact_pair_grant[3] = request[3] && !request[4];
        compact_pair_grant[2] = request[2]
            && !(request[4] || request[3]);
        compact_pair_grant[0] = request[0]
            && !(request[4] || request[3] || request[2]);
        compact_pair_grant[5] = request[5];
        compact_pair_grant[1] = request[1] && !request[5];

        compact_write_conflict = (in_valid[9] && in_valid[6])
            || (in_valid[10] && in_valid[8])
            || (in_valid[11] && in_valid[7])
            || (in_valid[4]
                && (in_valid[3] || in_valid[2] || in_valid[0]))
            || (in_valid[3] && (in_valid[2] || in_valid[0]))
            || (in_valid[2] && in_valid[0])
            || (in_valid[5] && in_valid[1]);
    end

    always_comb begin : capacity_monitor
        integer band;
        capacity_guard_mask = '0;
        if (!COMPACT_PRODUCTION_PROFILE) begin
            capacity_guard_mask[0] = write_wrap[0] != read_wrap[0]
                && write_pointer[0][7:6] == read_group_pointer[0][4:3];
            for (band = 1; band < 9; band = band + 1)
                capacity_guard_mask[band]
                    = write_wrap[band] != read_wrap[band]
                        && write_pointer[band][8:7]
                            == read_group_pointer[band][5:4];
            for (band = 9; band < 12; band = band + 1)
                capacity_guard_mask[band]
                    = write_wrap[band] != read_wrap[band]
                        && write_pointer[band][9:8]
                            == read_group_pointer[band][6:5];
        end
        capacity_guard = |capacity_guard_mask;
    end

    always_comb begin : schedule_control
        schedule_rom_read_enable = !schedule_family_valid
            && !schedule_rom_read_pending && !schedule_done;
        schedule_band = 0;
        schedule_family_last_phase = 1'b1;
        case (schedule_family)
            3'd0: begin
                // Bands 9, 10 and 11 are simultaneous first-level outputs.
                schedule_band = 4'd9 + {2'd0,schedule_family_phase};
                schedule_family_last_phase = schedule_family_phase == 2;
            end
            3'd1: schedule_band = 4'd6;
            3'd2: begin
                schedule_band = 4'd7 + {2'd0,schedule_family_phase};
                schedule_family_last_phase = schedule_family_phase == 1;
            end
            3'd3: schedule_band = 4'd3;
            3'd4: begin
                schedule_band = 4'd4 + {2'd0,schedule_family_phase};
                schedule_family_last_phase = schedule_family_phase == 1;
            end
            3'd5: schedule_band = 4'd2;
            default: begin
                schedule_band = {2'd0,schedule_family_phase};
                schedule_family_last_phase = schedule_family_phase == 1;
            end
        endcase
    end

    assign schedule_advance = pair_read_start_group;

    always_comb begin : replay_control
        output_stage_ready = !pair_valid || pair_ready;
        pair_fire = pair_valid && pair_ready;
        pair_read_attempt = 1'b0;
        pair_read_start_group = 1'b0;
        pair_read_band = issue_band;
        pair_read_index = issue_pair_index;
        if ((!pair_read_pending || output_stage_ready)
                && issue_pair_index != 0) begin
            pair_read_attempt = 1'b1;
        end else if ((!pair_read_pending || output_stage_ready)
                && schedule_family_valid && !pair_empty[schedule_band]) begin
            pair_read_attempt = 1'b1;
            pair_read_start_group = 1'b1;
            pair_read_band = schedule_band;
            pair_read_index = 0;
        end
        pair_read_issue = pair_read_attempt;
        if (!pair_read_issue)
            pair_read_start_group = 1'b0;

        ram0_read_enable = pair_read_issue && pair_read_band == 0;
        ram0_read_address = {read_group_pointer[0][4:0],pair_read_index};

        if (COMPACT_PRODUCTION_PROFILE) begin
            // Decode the band directly instead of constructing a binary bank
            // number and decoding it again for the wide response mux.
            case (pair_pending_band)
                6, 9: pair_response_word = compact_ram_read_data[0];
                8, 10: pair_response_word = compact_ram_read_data[1];
                7, 11: pair_response_word = compact_ram_read_data[2];
                0, 2, 3, 4: pair_response_word = compact_ram_read_data[3];
                default: pair_response_word = compact_ram_read_data[4];
            endcase
        end else begin
            case (pair_pending_band)
                0: pair_response_word = ram0_read_data;
                1: pair_response_word = ram1_read_data;
                2: pair_response_word = ram2_read_data;
                3: pair_response_word = ram3_read_data;
                4: pair_response_word = ram4_read_data;
                5: pair_response_word = ram5_read_data;
                6: pair_response_word = ram6_read_data;
                7: pair_response_word = ram7_read_data;
                8: pair_response_word = ram8_read_data;
                9: pair_response_word = ram9_read_data;
                10: pair_response_word = ram10_read_data;
                default: pair_response_word = ram11_read_data;
            endcase
        end
    end

    always_comb begin : compact_memory_control
        logic [8:0] pair_address;
        compact_ram_write_enable = '0;
        compact_ram_write_address = '0;
        compact_ram_write_data = '0;
        compact_ram_read_enable = '0;
        // All banks see the same address; only the selected bank's read enable
        // captures it. Clearing each inactive address repeats the bank-select
        // predicate across nine bits without changing any memory read.
        for (integer bank = 0; bank < 5; bank = bank + 1)
            compact_ram_read_address[bank] = compact_base_for(pair_read_band)
                + {read_group_pointer[pair_read_band][5:0],pair_read_index};

        // Each expression is a priority mux only for malformed input.  Under
        // the fixed transform contract exactly zero or one term is true.
        compact_ram_write_enable[0]
            = compact_pair_grant[9] || compact_pair_grant[6];
        if (compact_pair_grant[9]) begin
            pair_address = compact_base_for(9) + write_pointer[9][8:0];
            compact_ram_write_address[0]
                = {pair_address,half_full[9]};
            compact_ram_write_data[0] = coefficient_write_word[9];
        end else begin
            pair_address = compact_base_for(6) + write_pointer[6][8:0];
            compact_ram_write_address[0]
                = {pair_address,half_full[6]};
            compact_ram_write_data[0] = coefficient_write_word[6];
        end

        compact_ram_write_enable[1]
            = compact_pair_grant[10] || compact_pair_grant[8];
        if (compact_pair_grant[10]) begin
            pair_address = compact_base_for(10) + write_pointer[10][8:0];
            compact_ram_write_address[1]
                = {pair_address,half_full[10]};
            compact_ram_write_data[1] = coefficient_write_word[10];
        end else begin
            pair_address = compact_base_for(8) + write_pointer[8][8:0];
            compact_ram_write_address[1]
                = {pair_address,half_full[8]};
            compact_ram_write_data[1] = coefficient_write_word[8];
        end

        compact_ram_write_enable[2]
            = compact_pair_grant[11] || compact_pair_grant[7];
        if (compact_pair_grant[11]) begin
            pair_address = compact_base_for(11) + write_pointer[11][8:0];
            compact_ram_write_address[2]
                = {pair_address,half_full[11]};
            compact_ram_write_data[2] = coefficient_write_word[11];
        end else begin
            pair_address = compact_base_for(7) + write_pointer[7][8:0];
            compact_ram_write_address[2]
                = {pair_address,half_full[7]};
            compact_ram_write_data[2] = coefficient_write_word[7];
        end

        compact_ram_write_enable[3] = |(compact_pair_grant
            & ((12'd1 << 4) | (12'd1 << 3)
                | (12'd1 << 2) | (12'd1 << 0)));
        if (compact_pair_grant[4]) begin
            pair_address = compact_base_for(4) + write_pointer[4][8:0];
            compact_ram_write_address[3]
                = {pair_address,half_full[4]};
            compact_ram_write_data[3] = coefficient_write_word[4];
        end else if (compact_pair_grant[3]) begin
            pair_address = compact_base_for(3) + write_pointer[3][8:0];
            compact_ram_write_address[3]
                = {pair_address,half_full[3]};
            compact_ram_write_data[3] = coefficient_write_word[3];
        end else if (compact_pair_grant[2]) begin
            pair_address = compact_base_for(2) + write_pointer[2][8:0];
            compact_ram_write_address[3]
                = {pair_address,half_full[2]};
            compact_ram_write_data[3] = coefficient_write_word[2];
        end else begin
            pair_address = compact_base_for(0) + write_pointer[0][8:0];
            compact_ram_write_address[3]
                = {pair_address,half_full[0]};
            compact_ram_write_data[3] = coefficient_write_word[0];
        end

        compact_ram_write_enable[4]
            = compact_pair_grant[5] || compact_pair_grant[1];
        if (compact_pair_grant[5]) begin
            pair_address = compact_base_for(5) + write_pointer[5][8:0];
            compact_ram_write_address[4]
                = {pair_address,half_full[5]};
            compact_ram_write_data[4] = coefficient_write_word[5];
        end else begin
            pair_address = compact_base_for(1) + write_pointer[1][8:0];
            compact_ram_write_address[4]
                = {pair_address,half_full[1]};
            compact_ram_write_data[4] = coefficient_write_word[1];
        end

        if (COMPACT_PRODUCTION_PROFILE && pair_read_issue) begin
            case (compact_bank_for(pair_read_band))
                0: begin
                    compact_ram_read_enable[0] = 1'b1;
                end
                1: begin
                    compact_ram_read_enable[1] = 1'b1;
                end
                2: begin
                    compact_ram_read_enable[2] = 1'b1;
                end
                3: begin
                    compact_ram_read_enable[3] = 1'b1;
                end
                default: begin
                    compact_ram_read_enable[4] = 1'b1;
                end
            endcase
        end
    end

    // Keep the one shared increment and terminal comparator. Each band's
    // register only stores its needed compact modulo width; preserving these
    // common results prevents synthesis from duplicating the arithmetic at
    // every truncated destination.
    (* syn_keep = 1 *) wire [5:0] compact_read_increment
        = read_group_pointer[pair_read_band][5:0] + 1'b1;
    (* syn_keep = 1 *) wire compact_read_terminal
        = read_group_pointer[pair_read_band]
            == compact_group_capacity(pair_read_band) - 1'b1;
    always_ff @(posedge clk) begin : queue_state
        integer band;
        if (rst || clear_frame) begin
            pair_valid <= 1'b0;
            pair_group_start <= 1'b0;
            pair_group_last <= 1'b0;
            pair_band <= '0;
            pair_q0 <= '0;
            pair_q1 <= '0;
            pair_correction <= '0;
            pair_residual_class <= '0;
            protocol_error <= 1'b0;
            write_wrap <= '0;
            read_wrap <= '0;
            schedule_block <= '0;
            schedule_token_address <= '0;
            schedule_rom_read_pending <= 1'b0;
            schedule_family <= '0;
            schedule_family_phase <= 3;
            schedule_done <= 1'b0;
            issue_band <= '0;
            issue_pair_index <= '0;
            pair_read_pending <= 1'b0;
            pair_pending_band <= '0;
            pair_pending_index <= '0;
            for (band = 0; band < 12; band = band + 1) begin
                half_full[band] <= 1'b0;
                write_pointer[band] <= '0;
                read_group_pointer[band] <= '0;
            end
        end else begin
            if ((!COMPACT_PRODUCTION_PROFILE && capacity_guard)
                    || (COMPACT_PRODUCTION_PROFILE
                        && (compact_write_conflict
                            || compact_capacity_guard))
`ifndef SYNTHESIS
                    || (!COMPACT_PRODUCTION_PROFILE
                        && (|(pair_write & pair_full)
                            || (pair_read_attempt
                                && pair_empty[pair_read_band])))
`endif
                    )
                protocol_error <= 1'b1;

            schedule_rom_read_pending <= schedule_rom_read_enable;
            if (schedule_rom_read_pending) begin
                schedule_family <= schedule_rom_q;
                schedule_family_phase <= '0;
            end
            if (schedule_advance) begin
                if (!schedule_family_last_phase) begin
                    schedule_family_phase <= schedule_family_phase + 1'b1;
                end else begin
                    schedule_family_phase <= 3;
                    if (schedule_token_address == STARTUP_TOKEN_LAST) begin
                        schedule_token_address <= STEADY_TOKEN_BASE;
                    end else if (schedule_token_address
                            == STEADY_TOKEN_LAST) begin
                        if (schedule_block == 47) begin
                            schedule_token_address <= TAIL_TOKEN_BASE;
                        end else begin
                            schedule_block <= schedule_block + 1'b1;
                            schedule_token_address <= STEADY_TOKEN_BASE;
                        end
                    end else if (schedule_token_address
                            == TAIL_TOKEN_LAST) begin
                        schedule_done <= 1'b1;
                    end else begin
                        schedule_token_address
                            <= schedule_token_address + 1'b1;
                    end
                end
            end

            if (pair_read_pending && output_stage_ready) begin
                pair_valid <= 1'b1;
                pair_group_start <= pair_pending_index == 0;
                pair_group_last <= pair_pending_index == 7;
                pair_band <= pair_pending_band;
                pair_q0 <= pair_response_word[7:0];
                pair_correction[0] <= pair_response_word[8];
                pair_residual_class[0] <= pair_response_word[9];
                pair_q1 <= pair_response_word[17:10];
                pair_correction[1] <= pair_response_word[18];
                pair_residual_class[1] <= pair_response_word[19];
            end else if (pair_fire) begin
                pair_valid <= 1'b0;
            end
            pair_read_pending <= (pair_read_pending && !output_stage_ready)
                || pair_read_issue;
            if (pair_read_issue) begin
                pair_pending_band <= pair_read_band;
                pair_pending_index <= pair_read_index;
                if (pair_read_index == 7) begin
                    if (COMPACT_PRODUCTION_PROFILE) begin
                        // Capacities are 8, 24, 22, and 42 groups. The upper
                        // bits remain at reset zero and can be removed.
                        for (band = 0; band < 12; band = band + 1) begin
                            if (pair_read_band == 4'(band)) begin
                                if (compact_read_terminal)
                                    read_wrap[band] <= !read_wrap[band];
                                if (band < 3)
                                    read_group_pointer[band][2:0]
                                        <= compact_read_terminal ? 3'd0
                                            : compact_read_increment[2:0];
                                else if (band < 9)
                                    read_group_pointer[band][4:0]
                                        <= compact_read_terminal ? 5'd0
                                            : compact_read_increment[4:0];
                                else
                                    read_group_pointer[band][5:0]
                                        <= compact_read_terminal ? 6'd0
                                            : compact_read_increment[5:0];
                            end
                        end
                    end else begin
                        read_group_pointer[pair_read_band]
                            <= next_read_group_pointer(pair_read_band,
                                read_group_pointer[pair_read_band]);
                    end
                    if (!COMPACT_PRODUCTION_PROFILE
                            && ((pair_read_band == 0
                            && read_group_pointer[pair_read_band] == 7'd31)
                            || (pair_read_band > 0 && pair_read_band < 9
                                && read_group_pointer[pair_read_band]
                                    == 7'd63)
                            || (pair_read_band >= 9
                                && read_group_pointer[pair_read_band]
                                    == 7'd127)))
                        read_wrap[pair_read_band]
                            <= !read_wrap[pair_read_band];
                end
            end

            if (pair_read_start_group) begin
                issue_band <= pair_read_band;
                issue_pair_index <= 1;
            end else if (pair_read_issue && issue_pair_index != 0) begin
                issue_pair_index <= issue_pair_index + 1'b1;
            end

            for (band = 0; band < 12; band = band + 1) begin
                if (in_valid[band]
                        && (!COMPACT_PRODUCTION_PROFILE
                            || compact_pair_grant[band])) begin
                    if (!half_full[band]) begin
                        half_full[band] <= 1'b1;
                    end else begin
                        half_full[band] <= 1'b0;
                        if (COMPACT_PRODUCTION_PROFILE) begin
                            // Spell out the four fixed modulo classes so
                            // synthesis does not widen every band to the
                            // largest 336-pair region.  Upper bits are held
                            // at their reset value and therefore disappear.
                            if (band < 3) begin
                                if (write_pointer[band][5:0] == 6'd63) begin
                                    write_pointer[band][5:0] <= '0;
                                    write_wrap[band] <= !write_wrap[band];
                                end else begin
                                    write_pointer[band][5:0]
                                        <= write_pointer[band][5:0] + 1'b1;
                                end
                            end else if (band < 6) begin
                                if (write_pointer[band][7:0] == 8'd191) begin
                                    write_pointer[band][7:0] <= '0;
                                    write_wrap[band] <= !write_wrap[band];
                                end else begin
                                    write_pointer[band][7:0]
                                        <= write_pointer[band][7:0] + 1'b1;
                                end
                            end else if (band < 9) begin
                                if (write_pointer[band][7:0] == 8'd175) begin
                                    write_pointer[band][7:0] <= '0;
                                    write_wrap[band] <= !write_wrap[band];
                                end else begin
                                    write_pointer[band][7:0]
                                        <= write_pointer[band][7:0] + 1'b1;
                                end
                            end else begin
                                if (write_pointer[band][8:0] == 9'd335) begin
                                    write_pointer[band][8:0] <= '0;
                                    write_wrap[band] <= !write_wrap[band];
                                end else begin
                                    write_pointer[band][8:0]
                                        <= write_pointer[band][8:0] + 1'b1;
                                end
                            end
                        end else if (band == 0) begin
                            write_pointer[band][7:0]
                                <= write_pointer[band][7:0] + 1'b1;
                        end else if (band < 9) begin
                            write_pointer[band][8:0]
                                <= write_pointer[band][8:0] + 1'b1;
                        end else begin
                            write_pointer[band]
                                <= write_pointer[band] + 1'b1;
                        end
                        if (!COMPACT_PRODUCTION_PROFILE
                                && ((band == 0
                                    && write_pointer[band] == 10'd255)
                                || (band > 0 && band < 9
                                    && write_pointer[band] == 10'd511)
                                || (band >= 9
                                    && write_pointer[band] == 10'd1023)))
                            write_wrap[band] <= !write_wrap[band];
                    end
                end
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[0])
            ram0[{write_pointer[0][7:0],half_full[0]}]
                <= coefficient_write_word[0];
        if (!COMPACT_PRODUCTION_PROFILE && ram0_read_enable)
            ram0_read_data <= {ram0[{ram0_read_address,1'b1}],
                ram0[{ram0_read_address,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (schedule_rom_read_enable)
            schedule_rom_q <= schedule_rom[schedule_token_address];
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[1])
            ram1[{write_pointer[1][8:0],half_full[1]}]
                <= coefficient_write_word[1];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 1)
            ram1_read_data <= {ram1[{read_group_pointer[1][5:0],
                    pair_read_index,1'b1}],
                ram1[{read_group_pointer[1][5:0],pair_read_index,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[2])
            ram2[{write_pointer[2][8:0],half_full[2]}]
                <= coefficient_write_word[2];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 2)
            ram2_read_data <= {ram2[{read_group_pointer[2][5:0],
                    pair_read_index,1'b1}],
                ram2[{read_group_pointer[2][5:0],pair_read_index,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[3])
            ram3[{write_pointer[3][8:0],half_full[3]}]
                <= coefficient_write_word[3];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 3)
            ram3_read_data <= {ram3[{read_group_pointer[3][5:0],
                    pair_read_index,1'b1}],
                ram3[{read_group_pointer[3][5:0],pair_read_index,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[4])
            ram4[{write_pointer[4][8:0],half_full[4]}]
                <= coefficient_write_word[4];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 4)
            ram4_read_data <= {ram4[{read_group_pointer[4][5:0],
                    pair_read_index,1'b1}],
                ram4[{read_group_pointer[4][5:0],pair_read_index,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[5])
            ram5[{write_pointer[5][8:0],half_full[5]}]
                <= coefficient_write_word[5];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 5)
            ram5_read_data <= {ram5[{read_group_pointer[5][5:0],
                    pair_read_index,1'b1}],
                ram5[{read_group_pointer[5][5:0],pair_read_index,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[6])
            ram6[{write_pointer[6][8:0],half_full[6]}]
                <= coefficient_write_word[6];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 6)
            ram6_read_data <= {ram6[{read_group_pointer[6][5:0],
                    pair_read_index,1'b1}],
                ram6[{read_group_pointer[6][5:0],pair_read_index,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[7])
            ram7[{write_pointer[7][8:0],half_full[7]}]
                <= coefficient_write_word[7];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 7)
            ram7_read_data <= {ram7[{read_group_pointer[7][5:0],
                    pair_read_index,1'b1}],
                ram7[{read_group_pointer[7][5:0],pair_read_index,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[8])
            ram8[{write_pointer[8][8:0],half_full[8]}]
                <= coefficient_write_word[8];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 8)
            ram8_read_data <= {ram8[{read_group_pointer[8][5:0],
                    pair_read_index,1'b1}],
                ram8[{read_group_pointer[8][5:0],pair_read_index,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[9])
            ram9[{write_pointer[9],half_full[9]}]
                <= coefficient_write_word[9];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 9)
            ram9_read_data <= {ram9[{read_group_pointer[9],
                    pair_read_index,1'b1}],
                ram9[{read_group_pointer[9],pair_read_index,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[10])
            ram10[{write_pointer[10],half_full[10]}]
                <= coefficient_write_word[10];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 10)
            ram10_read_data <= {ram10[{read_group_pointer[10],
                    pair_read_index,1'b1}],
                ram10[{read_group_pointer[10],pair_read_index,1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (!COMPACT_PRODUCTION_PROFILE && in_valid[11])
            ram11[{write_pointer[11],half_full[11]}]
                <= coefficient_write_word[11];
        if (!COMPACT_PRODUCTION_PROFILE
                && pair_read_issue && pair_read_band == 11)
            ram11_read_data <= {ram11[{read_group_pointer[11],
                    pair_read_index,1'b1}],
                ram11[{read_group_pointer[11],pair_read_index,1'b0}]};
    end

    always_ff @(posedge clk) begin
        if (COMPACT_PRODUCTION_PROFILE && compact_ram_write_enable[0])
            compact_ram0[compact_ram_write_address[0]]
                <= compact_ram_write_data[0];
        if (COMPACT_PRODUCTION_PROFILE && compact_ram_read_enable[0])
            compact_ram_read_data[0]
                <= {compact_ram0[{compact_ram_read_address[0],1'b1}],
                    compact_ram0[{compact_ram_read_address[0],1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (COMPACT_PRODUCTION_PROFILE && compact_ram_write_enable[1])
            compact_ram1[compact_ram_write_address[1]]
                <= compact_ram_write_data[1];
        if (COMPACT_PRODUCTION_PROFILE && compact_ram_read_enable[1])
            compact_ram_read_data[1]
                <= {compact_ram1[{compact_ram_read_address[1],1'b1}],
                    compact_ram1[{compact_ram_read_address[1],1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (COMPACT_PRODUCTION_PROFILE && compact_ram_write_enable[2])
            compact_ram2[compact_ram_write_address[2]]
                <= compact_ram_write_data[2];
        if (COMPACT_PRODUCTION_PROFILE && compact_ram_read_enable[2])
            compact_ram_read_data[2]
                <= {compact_ram2[{compact_ram_read_address[2],1'b1}],
                    compact_ram2[{compact_ram_read_address[2],1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (COMPACT_PRODUCTION_PROFILE && compact_ram_write_enable[3])
            compact_ram3[compact_ram_write_address[3]]
                <= compact_ram_write_data[3];
        if (COMPACT_PRODUCTION_PROFILE && compact_ram_read_enable[3])
            compact_ram_read_data[3]
                <= {compact_ram3[{compact_ram_read_address[3],1'b1}],
                    compact_ram3[{compact_ram_read_address[3],1'b0}]};
    end
    always_ff @(posedge clk) begin
        if (COMPACT_PRODUCTION_PROFILE && compact_ram_write_enable[4])
            compact_ram4[compact_ram_write_address[4]]
                <= compact_ram_write_data[4];
        if (COMPACT_PRODUCTION_PROFILE && compact_ram_read_enable[4])
            compact_ram_read_data[4]
                <= {compact_ram4[{compact_ram_read_address[4],1'b1}],
                    compact_ram4[{compact_ram_read_address[4],1'b0}]};
    end

`ifndef SYNTHESIS
    always_ff @(posedge clk) begin : simulation_protocol_checks
        if (!rst && !clear_frame) begin
            if (!COMPACT_PRODUCTION_PROFILE) begin
                assert (!(|(pair_write & pair_full)))
                    else $error("pair queue band data RAM full mask=%h",
                        pair_write & pair_full);
                assert (!(pair_read_attempt && pair_empty[pair_read_band]))
                    else $error("pair queue empty read band=%0d",
                        pair_read_band);
            end else begin
                assert (!(pair_read_issue
                    && compact_exact_pair_empty[pair_read_band]))
                    else $error("compact pair queue exact empty read band=%0d",
                        pair_read_band);
                assert ((compact_exact_pair_full
                        & ~compact_capacity_guard_mask) == 0)
                    else $error("compact early guard missed exact full mask=%h",
                        compact_exact_pair_full
                            & ~compact_capacity_guard_mask);
            end
            if (schedule_family_valid)
                assert (schedule_family <= 6)
                    else $error("invalid canonical group-family token");
        end
    end
`endif
endmodule

`default_nettype wire
