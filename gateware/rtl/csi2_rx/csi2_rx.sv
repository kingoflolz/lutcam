`default_nettype none

// Fixed-function two-lane OV9281 CSI-2 receiver.  The byte side connects
// directly to Titanium x8 MIPI receive-lane FIFOs.  pixel_clk may be any
// continuously running clock at or above the fixed output demand; the Ti60
// integration uses the existing 120 MHz image-processing clock directly.
module csi2_rx #(
    parameter integer INIT_GUARD_CYCLES = 10_000,
    parameter integer LPX_CYCLES = 5,
    parameter integer SOT_SETTLE_CYCLES = 9,
    parameter integer CLOCK_TIMEOUT_CYCLES = 64,
    parameter integer PIXEL_FIFO_DEPTH = 512,
    parameter bit PACK_MSB8_MARKERS = 1'b0,
    parameter bit USE_DSP_COUNTERS = 1'b0
) (
    input  wire         reset_n,
    input  wire         ctrl_clk,
    input  wire         byte_clk,
    input  wire         pixel_clk,
    input  wire         status_clear,

    input  wire         clk_lp_p,
    input  wire         clk_lp_n,
    output logic        clk_hs_term,
    output logic        clk_hs_ena,

    input  wire  [1:0]  data_lp_p,
    input  wire  [1:0]  data_lp_n,
    input  wire  [7:0]  data0_hs_in,
    input  wire  [7:0]  data1_hs_in,
    input  wire  [1:0]  data_fifo_empty,
    output wire  [1:0]  data_fifo_rd,
    output wire  [1:0]  data_rst,
    output logic [1:0]  data_hs_term,
    output logic [1:0]  data_hs_ena,

    output logic        init_done,
    output logic [15:0] status,

    output logic [19:0] pixel_data,
    output logic        pixel_valid,
    output logic        pixel_sof,
    output logic        pixel_sol,
    output logic        pixel_eol,
    output logic        pixel_eof
);
    localparam integer IMAGE_WIDTH = 1280;
    localparam integer IMAGE_HEIGHT = 800;
    localparam integer LINE_PAYLOAD_BYTES = 1600;
    localparam integer PAYLOAD_PAIRS = LINE_PAYLOAD_BYTES / 2;
    localparam integer PAYLOAD_SCHEDULES = PAYLOAD_PAIRS / 5;
    localparam integer BEATS_PER_LINE = IMAGE_WIDTH / 2;
    localparam integer INIT_COUNT_WIDTH = $clog2(INIT_GUARD_CYCLES + 1);
    localparam integer INIT_LP11_COUNT_WIDTH = $clog2(LPX_CYCLES + 1);
    localparam integer CLOCK_TIMEOUT_COUNT_WIDTH =
        CLOCK_TIMEOUT_CYCLES <= 1 ? 1 : $clog2(CLOCK_TIMEOUT_CYCLES);
    localparam integer DATA_SETTLE_COUNT_WIDTH =
        SOT_SETTLE_CYCLES <= 1 ? 1 : $clog2(SOT_SETTLE_CYCLES);
    localparam integer DATA_STOP_COUNT_WIDTH =
        LPX_CYCLES <= 1 ? 1 : $clog2(LPX_CYCLES);
    localparam integer DATA_PHASE_COUNT_WIDTH =
        DATA_SETTLE_COUNT_WIDTH > DATA_STOP_COUNT_WIDTH ?
            DATA_SETTLE_COUNT_WIDTH : DATA_STOP_COUNT_WIDTH;

    localparam logic [5:0] DT_FRAME_START = 6'h00;
    localparam logic [5:0] DT_FRAME_END = 6'h01;
    localparam logic [5:0] DT_RAW10 = 6'h2b;

    localparam integer STATUS_HEADER_ECC_CORRECTED = 0;
    localparam integer STATUS_HEADER_ECC_UNCORRECTABLE = 1;
    localparam integer STATUS_PAYLOAD_CRC_ERROR = 2;
    localparam integer STATUS_SOT_SYNC_ERROR = 3;
    localparam integer STATUS_LANE_ALIGNMENT_ERROR = 4;
    localparam integer STATUS_LP_SEQUENCE_ERROR = 5;
    localparam integer STATUS_UNEXPECTED_PACKET = 6;
    localparam integer STATUS_WORD_COUNT_ERROR = 7;
    localparam integer STATUS_PACKET_LENGTH_ERROR = 8;
    localparam integer STATUS_FRAME_SEQUENCE_ERROR = 9;
    localparam integer STATUS_LINE_COUNT_ERROR = 10;
    localparam integer STATUS_PIXEL_FIFO_OVERFLOW = 11;
    localparam integer STATUS_CLOCK_STOP_ERROR = 12;
    localparam integer STATUS_INITIALIZATION_ERROR = 13;

    function automatic logic [5:0] header_ecc(input logic [23:0] data);
        logic [5:0] value;
        begin
            value[0] = data[0] ^ data[1] ^ data[2] ^ data[4] ^
                data[5] ^ data[7] ^ data[10] ^ data[11] ^ data[13] ^
                data[16] ^ data[20] ^ data[21] ^ data[22] ^ data[23];
            value[1] = data[0] ^ data[1] ^ data[3] ^ data[4] ^
                data[6] ^ data[8] ^ data[10] ^ data[12] ^ data[14] ^
                data[17] ^ data[20] ^ data[21] ^ data[22] ^ data[23];
            value[2] = data[0] ^ data[2] ^ data[3] ^ data[5] ^
                data[6] ^ data[9] ^ data[11] ^ data[12] ^ data[15] ^
                data[18] ^ data[20] ^ data[21] ^ data[22];
            value[3] = data[1] ^ data[2] ^ data[3] ^ data[7] ^
                data[8] ^ data[9] ^ data[13] ^ data[14] ^ data[15] ^
                data[19] ^ data[20] ^ data[21] ^ data[23];
            value[4] = data[4] ^ data[5] ^ data[6] ^ data[7] ^
                data[8] ^ data[9] ^ data[16] ^ data[17] ^ data[18] ^
                data[19] ^ data[20] ^ data[22] ^ data[23];
            value[5] = data[10] ^ data[11] ^ data[12] ^ data[13] ^
                data[14] ^ data[15] ^ data[16] ^ data[17] ^ data[18] ^
                data[19] ^ data[21] ^ data[22] ^ data[23];
            header_ecc = value;
        end
    endfunction

    function automatic logic [23:0] header_data_error_mask(
        input logic [5:0] syndrome
    );
        begin
            header_data_error_mask = 24'h000000;
            case (syndrome)
                6'd7:  header_data_error_mask[0] = 1'b1;
                6'd11: header_data_error_mask[1] = 1'b1;
                6'd13: header_data_error_mask[2] = 1'b1;
                6'd14: header_data_error_mask[3] = 1'b1;
                6'd19: header_data_error_mask[4] = 1'b1;
                6'd21: header_data_error_mask[5] = 1'b1;
                6'd22: header_data_error_mask[6] = 1'b1;
                6'd25: header_data_error_mask[7] = 1'b1;
                6'd26: header_data_error_mask[8] = 1'b1;
                6'd28: header_data_error_mask[9] = 1'b1;
                6'd35: header_data_error_mask[10] = 1'b1;
                6'd37: header_data_error_mask[11] = 1'b1;
                6'd38: header_data_error_mask[12] = 1'b1;
                6'd41: header_data_error_mask[13] = 1'b1;
                6'd42: header_data_error_mask[14] = 1'b1;
                6'd44: header_data_error_mask[15] = 1'b1;
                6'd49: header_data_error_mask[16] = 1'b1;
                6'd50: header_data_error_mask[17] = 1'b1;
                6'd52: header_data_error_mask[18] = 1'b1;
                6'd56: header_data_error_mask[19] = 1'b1;
                6'd31: header_data_error_mask[20] = 1'b1;
                6'd47: header_data_error_mask[21] = 1'b1;
                6'd55: header_data_error_mask[22] = 1'b1;
                6'd59: header_data_error_mask[23] = 1'b1;
                default: begin
                end
            endcase
        end
    endfunction

    function automatic logic header_ecc_error(input logic [5:0] syndrome);
        begin
            case (syndrome)
                6'd1, 6'd2, 6'd4, 6'd8, 6'd16, 6'd32:
                    header_ecc_error = 1'b1;
                default:
                    header_ecc_error = 1'b0;
            endcase
        end
    endfunction

    function automatic logic [15:0] crc16_pair(
        input logic [15:0] previous,
        input logic [15:0] data
    );
        logic [15:0] mixed;
        begin
            mixed = previous ^ data;
            crc16_pair[0] = mixed[4] ^ mixed[5] ^ mixed[8] ^ mixed[12];
            crc16_pair[1] = mixed[5] ^ mixed[6] ^ mixed[9] ^ mixed[13];
            crc16_pair[2] = mixed[6] ^ mixed[7] ^ mixed[10] ^ mixed[14];
            crc16_pair[3] = mixed[0] ^ mixed[7] ^ mixed[8] ^ mixed[11] ^
                mixed[15];
            crc16_pair[4] = mixed[0] ^ mixed[1] ^ mixed[4] ^ mixed[5] ^
                mixed[9];
            crc16_pair[5] = mixed[1] ^ mixed[2] ^ mixed[5] ^ mixed[6] ^
                mixed[10];
            crc16_pair[6] = mixed[0] ^ mixed[2] ^ mixed[3] ^ mixed[6] ^
                mixed[7] ^ mixed[11];
            crc16_pair[7] = mixed[0] ^ mixed[1] ^ mixed[3] ^ mixed[4] ^
                mixed[7] ^ mixed[8] ^ mixed[12];
            crc16_pair[8] = mixed[0] ^ mixed[1] ^ mixed[2] ^ mixed[4] ^
                mixed[5] ^ mixed[8] ^ mixed[9] ^ mixed[13];
            crc16_pair[9] = mixed[1] ^ mixed[2] ^ mixed[3] ^ mixed[5] ^
                mixed[6] ^ mixed[9] ^ mixed[10] ^ mixed[14];
            crc16_pair[10] = mixed[2] ^ mixed[3] ^ mixed[4] ^ mixed[6] ^
                mixed[7] ^ mixed[10] ^ mixed[11] ^ mixed[15];
            crc16_pair[11] = mixed[0] ^ mixed[3] ^ mixed[7] ^ mixed[11];
            crc16_pair[12] = mixed[0] ^ mixed[1] ^ mixed[4] ^ mixed[8] ^
                mixed[12];
            crc16_pair[13] = mixed[1] ^ mixed[2] ^ mixed[5] ^ mixed[9] ^
                mixed[13];
            crc16_pair[14] = mixed[2] ^ mixed[3] ^ mixed[6] ^ mixed[10] ^
                mixed[14];
            crc16_pair[15] = mixed[3] ^ mixed[4] ^ mixed[7] ^ mixed[11] ^
                mixed[15];
        end
    endfunction

    // Asynchronous assertion and local synchronous reset release.
    logic [1:0] ctrl_reset_pipe;
    logic [1:0] byte_global_reset_pipe;
    logic [1:0] pixel_reset_pipe;
    wire ctrl_reset_n = ctrl_reset_pipe[1];
    wire byte_global_reset_n = byte_global_reset_pipe[1];
    wire pixel_reset_n = pixel_reset_pipe[1];

    always_ff @(posedge ctrl_clk or negedge reset_n) begin
        if (!reset_n)
            ctrl_reset_pipe <= 2'b00;
        else
            ctrl_reset_pipe <= {ctrl_reset_pipe[0], 1'b1};
    end

    always_ff @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n)
            byte_global_reset_pipe <= 2'b00;
        else
            byte_global_reset_pipe <= {byte_global_reset_pipe[0], 1'b1};
    end

    always_ff @(posedge pixel_clk or negedge reset_n) begin
        if (!reset_n)
            pixel_reset_pipe <= 2'b00;
        else
            pixel_reset_pipe <= {pixel_reset_pipe[0], 1'b1};
    end

    // Recovered-clock heartbeat.  This counter is deliberately outside the
    // receive-session reset so ctrl_clk can detect both startup and loss.
    logic [3:0] heartbeat_binary;
    wire [3:0] heartbeat_gray = heartbeat_binary ^ (heartbeat_binary >> 1);
    logic [3:0] heartbeat_sync0;
    logic [3:0] heartbeat_sync1;

    always_ff @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n)
            heartbeat_binary <= 4'h0;
        else if (byte_global_reset_n)
            heartbeat_binary <= heartbeat_binary + 1'b1;
    end

    always_ff @(posedge ctrl_clk or negedge reset_n) begin
        if (!reset_n) begin
            heartbeat_sync0 <= 4'h0;
            heartbeat_sync1 <= 4'h0;
        end else begin
            heartbeat_sync0 <= heartbeat_gray;
            heartbeat_sync1 <= heartbeat_sync0;
        end
    end

    // LP inputs are asynchronous to ctrl_clk.
    logic [5:0] lp_sync0;
    logic [5:0] lp_sync1;
    wire [1:0] clk_lp = lp_sync1[1:0];
    wire [1:0] data0_lp = lp_sync1[3:2];
    wire [1:0] data1_lp = lp_sync1[5:4];
    wire [1:0] data_lane_lp [0:1];
    assign data_lane_lp[0] = data0_lp;
    assign data_lane_lp[1] = data1_lp;

    always_ff @(posedge ctrl_clk or negedge reset_n) begin
        if (!reset_n) begin
            lp_sync0 <= 6'h00;
            lp_sync1 <= 6'h00;
        end else begin
            lp_sync0 <= {data_lp_p[1], data_lp_n[1],
                data_lp_p[0], data_lp_n[0], clk_lp_p, clk_lp_n};
            lp_sync1 <= lp_sync0;
        end
    end

    // The LP vector stores {P,N}; LP11 is 2'b11, LP01 is 2'b01.
    wire all_lp11 = clk_lp == 2'b11 && data0_lp == 2'b11 &&
        data1_lp == 2'b11;

    logic [INIT_COUNT_WIDTH-1:0] init_count;
    logic [INIT_LP11_COUNT_WIDTH-1:0] init_lp11_count;

    // Operational clears remain synchronous; only reset_n asserts asynchronously.
    Ov9281Counter #(.WIDTH(INIT_COUNT_WIDTH), .USE_DSP(USE_DSP_COUNTERS))
    init_timer_counter (
        .clk(ctrl_clk), .reset_n, .clear(!ctrl_reset_n),
        .enable(!init_done && init_count < INIT_COUNT_WIDTH'(INIT_GUARD_CYCLES)),
        .value(init_count)
    );

    always_ff @(posedge ctrl_clk or negedge reset_n) begin
        if (!reset_n) begin
            init_lp11_count <= '0;
            init_done <= 1'b0;
        end else if (!ctrl_reset_n) begin
            init_lp11_count <= '0;
            init_done <= 1'b0;
        end else if (!init_done) begin
            if (all_lp11) begin
                if (init_lp11_count <
                        INIT_LP11_COUNT_WIDTH'(LPX_CYCLES))
                    init_lp11_count <= init_lp11_count + 1'b1;
            end else begin
                init_lp11_count <= '0;
            end
            if (init_count >= INIT_COUNT_WIDTH'(INIT_GUARD_CYCLES - 1) &&
                    init_lp11_count >=
                        INIT_LP11_COUNT_WIDTH'(LPX_CYCLES - 1))
                init_done <= 1'b1;
        end
    end

    typedef enum logic [2:0] {
        CLOCK_WAIT_INIT,
        CLOCK_WAIT_REQUEST,
        CLOCK_WAIT_ZERO,
        CLOCK_WAIT_HEARTBEAT,
        CLOCK_WAIT_RELEASE,
        CLOCK_ACTIVE,
        CLOCK_WAIT_STOP
    } clock_state_t;

    clock_state_t clock_state;
    logic session_enable_ctrl;
    logic [CLOCK_TIMEOUT_COUNT_WIDTH-1:0] clock_timeout_count;
    logic [3:0] heartbeat_last;
    logic clock_lp_error_pulse;
    logic data_lp_error_pulse;
    logic ctrl_clock_stop_pulse;
    logic ctrl_init_error_pulse;

    wire byte_session_async_n = reset_n && session_enable_ctrl;
    logic [1:0] byte_session_reset_pipe;
    wire byte_session_reset_n = byte_session_reset_pipe[1];
    logic byte_session_up_sync0;
    logic byte_session_up_sync1;

    // Preserve heartbeat/release priority over timeout at the terminal count.
    wire clock_timer_clear =
        (clock_state == CLOCK_WAIT_ZERO && clk_lp == 2'b00) ||
        (clock_state == CLOCK_WAIT_HEARTBEAT && heartbeat_sync1 != heartbeat_last) ||
        (clock_state == CLOCK_WAIT_RELEASE && byte_session_up_sync1) ||
        (clock_state == CLOCK_ACTIVE && clk_lp != 2'b11 &&
            heartbeat_sync1 != heartbeat_last);
    wire clock_timer_enable =
        clock_timeout_count < CLOCK_TIMEOUT_COUNT_WIDTH'(CLOCK_TIMEOUT_CYCLES - 1) && (
        (clock_state == CLOCK_WAIT_HEARTBEAT && heartbeat_sync1 == heartbeat_last) ||
        (clock_state == CLOCK_WAIT_RELEASE && !byte_session_up_sync1) ||
        (clock_state == CLOCK_ACTIVE && clk_lp != 2'b11 &&
            heartbeat_sync1 == heartbeat_last));
    Ov9281Counter #(.WIDTH(CLOCK_TIMEOUT_COUNT_WIDTH), .USE_DSP(USE_DSP_COUNTERS))
    clock_timer_counter (
        .clk(ctrl_clk), .reset_n, .clear(clock_timer_clear),
        .enable(clock_timer_enable), .value(clock_timeout_count)
    );

    always_ff @(posedge byte_clk or negedge byte_session_async_n) begin
        if (!byte_session_async_n)
            byte_session_reset_pipe <= 2'b00;
        else
            byte_session_reset_pipe <= {byte_session_reset_pipe[0], 1'b1};
    end

    always_ff @(posedge ctrl_clk or negedge reset_n) begin
        if (!reset_n) begin
            byte_session_up_sync0 <= 1'b0;
            byte_session_up_sync1 <= 1'b0;
        end else begin
            byte_session_up_sync0 <= byte_session_reset_n;
            byte_session_up_sync1 <= byte_session_up_sync0;
        end
    end

    assign data_rst = {2{!byte_session_reset_n}};
    assign data_fifo_rd = byte_session_reset_n ? ~data_fifo_empty : 2'b00;

    always_ff @(posedge ctrl_clk or negedge reset_n) begin
        if (!reset_n) begin
            clock_state <= CLOCK_WAIT_INIT;
            clk_hs_term <= 1'b0;
            clk_hs_ena <= 1'b0;
            session_enable_ctrl <= 1'b0;
            heartbeat_last <= '0;
            clock_lp_error_pulse <= 1'b0;
            ctrl_clock_stop_pulse <= 1'b0;
            ctrl_init_error_pulse <= 1'b0;
        end else begin
            clock_lp_error_pulse <= 1'b0;
            ctrl_clock_stop_pulse <= 1'b0;
            ctrl_init_error_pulse <= 1'b0;
            case (clock_state)
                CLOCK_WAIT_INIT: begin
                    clk_hs_term <= 1'b0;
                    clk_hs_ena <= 1'b0;
                    session_enable_ctrl <= 1'b0;
                    if (init_done)
                        clock_state <= CLOCK_WAIT_REQUEST;
                end
                CLOCK_WAIT_REQUEST: begin
                    if (clk_lp == 2'b01) begin
                        clk_hs_term <= 1'b1;
                        clock_state <= CLOCK_WAIT_ZERO;
                    end else if (clk_lp == 2'b00 || clk_lp == 2'b10) begin
                        clock_lp_error_pulse <= 1'b1;
                        clock_state <= CLOCK_WAIT_STOP;
                    end
                end
                CLOCK_WAIT_ZERO: begin
                    if (clk_lp == 2'b00) begin
                        clk_hs_ena <= 1'b1;
                        heartbeat_last <= heartbeat_sync1;
                        clock_state <= CLOCK_WAIT_HEARTBEAT;
                    end else if (clk_lp == 2'b10 || clk_lp == 2'b11) begin
                        clock_lp_error_pulse <= 1'b1;
                        clk_hs_term <= 1'b0;
                        clock_state <= CLOCK_WAIT_STOP;
                    end
                end
                CLOCK_WAIT_HEARTBEAT: begin
                    if (heartbeat_sync1 != heartbeat_last) begin
                        session_enable_ctrl <= 1'b1;
                        clock_state <= CLOCK_WAIT_RELEASE;
                    end else if (clock_timeout_count >=
                            CLOCK_TIMEOUT_COUNT_WIDTH'(
                                CLOCK_TIMEOUT_CYCLES - 1)) begin
                        ctrl_init_error_pulse <= 1'b1;
                        clk_hs_term <= 1'b0;
                        clk_hs_ena <= 1'b0;
                        clock_state <= CLOCK_WAIT_STOP;
                    end
                end
                CLOCK_WAIT_RELEASE: begin
                    if (byte_session_up_sync1) begin
                        heartbeat_last <= heartbeat_sync1;
                        clock_state <= CLOCK_ACTIVE;
                    end else if (clock_timeout_count >=
                            CLOCK_TIMEOUT_COUNT_WIDTH'(
                                CLOCK_TIMEOUT_CYCLES - 1)) begin
                        ctrl_init_error_pulse <= 1'b1;
                        session_enable_ctrl <= 1'b0;
                        clk_hs_term <= 1'b0;
                        clk_hs_ena <= 1'b0;
                        clock_state <= CLOCK_WAIT_STOP;
                    end
                end
                CLOCK_ACTIVE: begin
                    if (clk_lp == 2'b11) begin
                        session_enable_ctrl <= 1'b0;
                        clk_hs_term <= 1'b0;
                        clk_hs_ena <= 1'b0;
                        clock_state <= CLOCK_WAIT_STOP;
                    end else if (heartbeat_sync1 != heartbeat_last) begin
                        heartbeat_last <= heartbeat_sync1;
                    end else if (clock_timeout_count >=
                            CLOCK_TIMEOUT_COUNT_WIDTH'(
                                CLOCK_TIMEOUT_CYCLES - 1)) begin
                        ctrl_clock_stop_pulse <= 1'b1;
                        session_enable_ctrl <= 1'b0;
                        clk_hs_term <= 1'b0;
                        clk_hs_ena <= 1'b0;
                        clock_state <= CLOCK_WAIT_STOP;
                    end
                end
                default: begin
                    session_enable_ctrl <= 1'b0;
                    clk_hs_term <= 1'b0;
                    clk_hs_ena <= 1'b0;
                    if (clk_lp == 2'b11)
                        clock_state <= CLOCK_WAIT_REQUEST;
                end
            endcase
        end
    end

    typedef enum logic [2:0] {
        DATA_WAIT_STOP,
        DATA_WAIT_REQUEST,
        DATA_WAIT_ZERO,
        DATA_SETTLE,
        DATA_ACTIVE
    } data_state_t;

    data_state_t data_state [0:1];
    logic [1:0] burst_start_toggle_ctrl;
    logic [1:0] burst_end_toggle_ctrl;
    logic [1:0][DATA_PHASE_COUNT_WIDTH-1:0] data_phase_count;

    // Independently synchronized P/N inputs can briefly report LP10 while
    // returning from HS to LP11. Qualify only this ACTIVE-state diagnostic;
    // keep LP11 burst termination and every entry/settle check immediate.
    logic [1:0] active_lp10_previous;
    always_ff @(posedge ctrl_clk or negedge reset_n) begin
        if (!reset_n) active_lp10_previous <= 2'b00;
        else for (integer lane = 0; lane < 2; lane++)
            active_lp10_previous[lane] <= session_enable_ctrl && byte_session_up_sync1
                && (data_state[lane] == DATA_ACTIVE) && (data_lane_lp[lane] == 2'b10);
    end

    always_ff @(posedge ctrl_clk or negedge reset_n) begin
        if (!reset_n) begin
            data_hs_term <= 2'b00;
            data_hs_ena <= 2'b00;
            burst_start_toggle_ctrl <= 2'b00;
            burst_end_toggle_ctrl <= 2'b00;
            data_phase_count <= '0;
            data_lp_error_pulse <= 1'b0;
            for (integer lane = 0; lane < 2; lane++)
                data_state[lane] <= DATA_WAIT_STOP;
        end else begin
            data_lp_error_pulse <= 1'b0;
            for (integer lane = 0; lane < 2; lane++) begin
                if (!session_enable_ctrl || !byte_session_up_sync1) begin
                    data_hs_term[lane] <= 1'b0;
                    data_hs_ena[lane] <= 1'b0;
                    data_phase_count[lane] <= '0;
                    data_state[lane] <= DATA_WAIT_STOP;
                end else begin
                    if ((data_state[lane] == DATA_WAIT_STOP &&
                            data_lane_lp[lane] == 2'b11 &&
                            data_phase_count[lane] <
                                DATA_PHASE_COUNT_WIDTH'(LPX_CYCLES - 1)) ||
                            (data_state[lane] == DATA_SETTLE &&
                            data_lane_lp[lane] != 2'b11 &&
                            data_phase_count[lane] <
                                DATA_PHASE_COUNT_WIDTH'(
                                    SOT_SETTLE_CYCLES - 1))) begin
                        data_phase_count[lane] <=
                            data_phase_count[lane] + 1'b1;
                    end
                    case (data_state[lane])
                        DATA_WAIT_STOP: begin
                            if (data_lane_lp[lane] == 2'b11) begin
                                if (data_phase_count[lane] >=
                                        DATA_PHASE_COUNT_WIDTH'(
                                            LPX_CYCLES - 1)) begin
                                    data_phase_count[lane] <= '0;
                                    data_state[lane] <= DATA_WAIT_REQUEST;
                                end
                            end else begin
                                data_phase_count[lane] <= '0;
                            end
                        end
                        DATA_WAIT_REQUEST: begin
                            if (data_lane_lp[lane] == 2'b01) begin
                                data_hs_term[lane] <= 1'b1;
                                data_state[lane] <= DATA_WAIT_ZERO;
                            end else if (data_lane_lp[lane] == 2'b00 ||
                                    data_lane_lp[lane] == 2'b10) begin
                                data_lp_error_pulse <= 1'b1;
                                data_state[lane] <= DATA_WAIT_STOP;
                            end
                        end
                        DATA_WAIT_ZERO: begin
                            if (data_lane_lp[lane] == 2'b00) begin
                                data_hs_ena[lane] <= 1'b1;
                                data_phase_count[lane] <= '0;
                                data_state[lane] <= DATA_SETTLE;
                            end else if (data_lane_lp[lane] == 2'b10 ||
                                    data_lane_lp[lane] == 2'b11) begin
                                data_lp_error_pulse <= 1'b1;
                                data_hs_term[lane] <= 1'b0;
                                data_phase_count[lane] <= '0;
                                data_state[lane] <= DATA_WAIT_STOP;
                            end
                        end
                        DATA_SETTLE: begin
                            if (data_lane_lp[lane] == 2'b11) begin
                                data_lp_error_pulse <= 1'b1;
                                data_hs_term[lane] <= 1'b0;
                                data_hs_ena[lane] <= 1'b0;
                                data_phase_count[lane] <= '0;
                                burst_end_toggle_ctrl[lane] <=
                                    ~burst_end_toggle_ctrl[lane];
                                data_state[lane] <= DATA_WAIT_STOP;
                            end else if (data_phase_count[lane] >=
                                    DATA_PHASE_COUNT_WIDTH'(
                                        SOT_SETTLE_CYCLES - 1)) begin
                                burst_start_toggle_ctrl[lane] <=
                                    ~burst_start_toggle_ctrl[lane];
                                data_state[lane] <= DATA_ACTIVE;
                            end
                        end
                        default: begin
                            if (data_lane_lp[lane] == 2'b11) begin
                                data_hs_term[lane] <= 1'b0;
                                data_hs_ena[lane] <= 1'b0;
                                data_phase_count[lane] <= '0;
                                burst_end_toggle_ctrl[lane] <=
                                    ~burst_end_toggle_ctrl[lane];
                                data_state[lane] <= DATA_WAIT_STOP;
                            end else if (data_lane_lp[lane] == 2'b10 && active_lp10_previous[lane]) begin
                                data_lp_error_pulse <= 1'b1;
                            end
                        end
                    endcase
                end
            end
        end
    end

    // Cross per-lane packet boundary events into the recovered-byte domain.
    logic [1:0] burst_start_sync0;
    logic [1:0] burst_start_sync1;
    logic [1:0] burst_start_seen;
    logic [1:0] burst_end_sync0;
    logic [1:0] burst_end_sync1;
    logic [1:0] burst_end_seen;
    wire [1:0] burst_start_pulse = burst_start_sync1 ^ burst_start_seen;
    wire [1:0] burst_end_pulse = burst_end_sync1 ^ burst_end_seen;

    always_ff @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            // Source toggles use the same constant reset. A live-data async
            // load cannot map to ordinary synchronizer flops and infers LUT
            // feedback; only session rebasing below samples the live toggles.
            burst_start_sync0 <= '0;
            burst_start_sync1 <= '0;
            burst_start_seen <= '0;
            burst_end_sync0 <= '0;
            burst_end_sync1 <= '0;
            burst_end_seen <= '0;
        end else if (!byte_session_reset_n) begin
            burst_start_sync0 <= burst_start_toggle_ctrl;
            burst_start_sync1 <= burst_start_toggle_ctrl;
            burst_start_seen <= burst_start_toggle_ctrl;
            burst_end_sync0 <= burst_end_toggle_ctrl;
            burst_end_sync1 <= burst_end_toggle_ctrl;
            burst_end_seen <= burst_end_toggle_ctrl;
        end else begin
            burst_start_sync0 <= burst_start_toggle_ctrl;
            burst_start_sync1 <= burst_start_sync0;
            burst_start_seen <= burst_start_sync1;
            burst_end_sync0 <= burst_end_toggle_ctrl;
            burst_end_sync1 <= burst_end_sync0;
            burst_end_seen <= burst_end_sync1;
        end
    end

    logic [1:0] lane_discard;
    logic [1:0] lane_pop;
    wire [1:0] lane_byte_valid;
    wire [7:0] lane0_byte;
    wire [7:0] lane1_byte;
    wire [1:0] lane_sync_found;
    wire [1:0] lane_sync_error;
    wire [1:0] lane_queue_overflow;

    Ov9281LaneAligner lane0_aligner (
        .byte_clk,
        .reset_n,
        .burst_start(burst_start_pulse[0]),
        .discard(lane_discard[0]),
        .raw_word(data0_hs_in),
        .raw_valid(data_fifo_rd[0]),
        .pair_pop(lane_pop[0]),
        .byte_valid(lane_byte_valid[0]),
        .byte_data(lane0_byte),
        .sync_found(lane_sync_found[0]),
        .sync_error(lane_sync_error[0]),
        .queue_overflow(lane_queue_overflow[0])
    );

    Ov9281LaneAligner lane1_aligner (
        .byte_clk,
        .reset_n,
        .burst_start(burst_start_pulse[1]),
        .discard(lane_discard[1]),
        .raw_word(data1_hs_in),
        .raw_valid(data_fifo_rd[1]),
        .pair_pop(lane_pop[1]),
        .byte_valid(lane_byte_valid[1]),
        .byte_data(lane1_byte),
        .sync_found(lane_sync_found[1]),
        .sync_error(lane_sync_error[1]),
        .queue_overflow(lane_queue_overflow[1])
    );

    logic [1:0] lane_synchronized;
    wire pair_available = &lane_byte_valid;

    typedef enum logic [2:0] {
        PARSER_WAIT_SYNC = 3'b000,
        PARSER_HEADER_SECOND = 3'b001,
        PARSER_PAYLOAD = 3'b010,
        PARSER_FOOTER = 3'b011,
        PARSER_HEADER_CHECK = 3'b100,
        PARSER_DISCARD = 3'b101
    } parser_state_t;

    parser_state_t parser_state;
    wire parser_needs_pair = !parser_state[2];
    wire parser_pair_pop = parser_needs_pair && pair_available &&
        ((parser_state != PARSER_WAIT_SYNC) || (&lane_synchronized));
    assign lane_pop = {2{parser_pair_pop}};

    logic [31:0] packet_buffer;
    wire [23:0] header_data = {
        packet_buffer[15:8], packet_buffer[23:16], packet_buffer[31:24]
    };
    wire [7:0] header_ecc_received = packet_buffer[7:0];
    logic [23:0] corrected_header;
    logic header_was_corrected;
    logic header_is_uncorrectable;
    logic [5:0] header_syndrome;
    logic [23:0] header_correction_mask;

    always_comb begin
        header_syndrome = header_ecc_received[5:0] ^ header_ecc(header_data);
        header_correction_mask = header_data_error_mask(header_syndrome);
        corrected_header = header_data ^ header_correction_mask;
        header_is_uncorrectable = header_ecc_received[7:6] != 2'b00;
        header_was_corrected = header_syndrome != 0 &&
            (|header_correction_mask || header_ecc_error(header_syndrome));
        if (!header_is_uncorrectable && header_syndrome != 0 &&
                !header_was_corrected)
            header_is_uncorrectable = 1'b1;
    end

    wire [7:0] corrected_data_id = corrected_header[7:0];
    wire [5:0] corrected_data_type = corrected_data_id[5:0];
    wire [1:0] corrected_virtual_channel = corrected_data_id[7:6];
    wire [15:0] corrected_data_field = corrected_header[23:8];

    logic [7:0] payload_schedule_count;
    logic [2:0] gearbox_phase;
    logic [15:0] payload_crc;
    wire [15:0] payload_crc_pair = crc16_pair(
        payload_crc, {lane1_byte, lane0_byte}
    );
    wire [9:0] parser_count_increment =
        (parser_state == PARSER_PAYLOAD ?
            {2'b00, payload_schedule_count} : byte_line_count) + 1'b1;

    // The previous four payload bytes are sufficient to produce both the
    // immediate and delayed RAW10 pixel pairs.  Keeping one rolling history
    // avoids separate A/B byte banks and a materialized pending pixel pair.
    wire [31:0] payload_history = packet_buffer;

    logic gearbox_write_valid;
    logic [19:0] gearbox_write_data;
    always_comb begin
        gearbox_write_valid = 1'b0;
        gearbox_write_data = 20'h0;
        if (parser_pair_pop && parser_state == PARSER_PAYLOAD) begin
            case (gearbox_phase)
                3'd0: begin
                    gearbox_write_valid = payload_schedule_count != 0;
                    gearbox_write_data[9:0] = {
                        payload_history[23:16], payload_history[5:4]
                    };
                    gearbox_write_data[19:10] = {
                        payload_history[15:8], payload_history[7:6]
                    };
                end
                3'd2: begin
                    gearbox_write_valid = 1'b1;
                    gearbox_write_data[9:0] = {
                        payload_history[31:24], lane0_byte[1:0]
                    };
                    gearbox_write_data[19:10] = {
                        payload_history[23:16], lane0_byte[3:2]
                    };
                end
                3'd3: begin
                    gearbox_write_valid = 1'b1;
                    gearbox_write_data[9:0] = {
                        payload_history[31:24], payload_history[13:12]
                    };
                    gearbox_write_data[19:10] = {
                        payload_history[23:16], payload_history[15:14]
                    };
                end
                3'd4: begin
                    gearbox_write_valid = 1'b1;
                    gearbox_write_data[9:0] = {
                        payload_history[23:16], lane1_byte[1:0]
                    };
                    gearbox_write_data[19:10] = {
                        payload_history[15:8], lane1_byte[3:2]
                    };
                end
                default: begin
                    gearbox_write_valid = 1'b0;
                end
            endcase
        end else if (parser_pair_pop && parser_state == PARSER_FOOTER) begin
            gearbox_write_valid = 1'b1;
            gearbox_write_data[9:0] = {
                payload_history[23:16], payload_history[5:4]
            };
            gearbox_write_data[19:10] = {
                payload_history[15:8], payload_history[7:6]
            };
        end
    end

    logic byte_frame_open;
    logic [15:0] byte_frame_id;
    logic [9:0] byte_line_count;
    logic frame_start_event_byte;
    logic frame_abort_event_byte;
    logic frame_start_toggle_byte;
    logic frame_abort_toggle_byte;
    logic pixel_idle_ack_sync0;
    logic pixel_idle_ack_sync1;
    logic pixel_idle_ack_seen;
    logic pixel_epoch_busy;
    logic packet_finished;
    logic end_guard_active;
    logic [2:0] end_guard_count;
    logic [15:0] parser_error_events;

    logic pixel_fifo_write_enable;
    logic [19:0] pixel_fifo_write_data;
    wire pixel_fifo_full;
    wire pixel_fifo_overflow;
    wire pixel_fifo_empty;
    wire pixel_fifo_read_valid;
    wire [19:0] pixel_fifo_read_data;
    logic pixel_fifo_read_ready;

    wire gearbox_first_beat = parser_state == PARSER_PAYLOAD &&
        gearbox_phase == 3'd2 && payload_schedule_count == 0;
    wire gearbox_last_beat = parser_state == PARSER_FOOTER;
    wire [3:0] gearbox_markers = {
        gearbox_first_beat && byte_line_count == 0,
        gearbox_first_beat,
        gearbox_last_beat,
        gearbox_last_beat && byte_line_count == 10'(IMAGE_HEIGHT - 1)
    };

    assign pixel_fifo_write_enable = gearbox_write_valid && byte_frame_open;
    assign pixel_fifo_write_data = PACK_MSB8_MARKERS ? {
        gearbox_write_data[19:12],
        gearbox_write_data[9:2],
        gearbox_markers
    } : gearbox_write_data;

    Ov9281PixelAsyncFifo #(
        .DEPTH(PIXEL_FIFO_DEPTH),
        .DATA_WIDTH(20)
    ) pixel_fifo (
        .wr_clk(byte_clk),
        .wr_reset_n(reset_n),
        .wr_en(pixel_fifo_write_enable),
        .wr_data(pixel_fifo_write_data),
        .wr_full(pixel_fifo_full),
        .wr_overflow(pixel_fifo_overflow),
        .rd_clk(pixel_clk),
        .rd_reset_n(reset_n),
        .rd_ready(pixel_fifo_read_ready),
        .rd_valid(pixel_fifo_read_valid),
        .rd_data(pixel_fifo_read_data),
        .rd_empty(pixel_fifo_empty)
    );

    // Pixel idle acknowledgement crosses back to the byte domain.
    logic pixel_idle_ack_toggle;
    always_ff @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            pixel_idle_ack_sync0 <= 1'b0;
            pixel_idle_ack_sync1 <= 1'b0;
            pixel_idle_ack_seen <= 1'b0;
        end else begin
            pixel_idle_ack_sync0 <= pixel_idle_ack_toggle;
            pixel_idle_ack_sync1 <= pixel_idle_ack_sync0;
            if (pixel_idle_ack_sync1 != pixel_idle_ack_seen)
                pixel_idle_ack_seen <= pixel_idle_ack_sync1;
        end
    end

    wire pixel_idle_ack_pulse = pixel_idle_ack_sync1 ^ pixel_idle_ack_seen;

    // Keep CDC toggles and the FIFO epoch outside the receive-session reset.
    // Otherwise a stopped/restarted recovered clock could look like a new
    // frame event in the pixel domain or admit a new frame before stale FIFO
    // contents have been drained.
    always_ff @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            frame_start_toggle_byte <= 1'b0;
            frame_abort_toggle_byte <= 1'b0;
            pixel_epoch_busy <= 1'b0;
        end else begin
            if (frame_start_event_byte) begin
                frame_start_toggle_byte <= ~frame_start_toggle_byte;
                pixel_epoch_busy <= 1'b1;
            end
            if (frame_abort_event_byte)
                frame_abort_toggle_byte <= ~frame_abort_toggle_byte;
            if (pixel_idle_ack_pulse && !frame_start_event_byte)
                pixel_epoch_busy <= 1'b0;
        end
    end

    always_ff @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n || !byte_session_reset_n) begin
            parser_state <= PARSER_WAIT_SYNC;
            lane_discard <= 2'b11;
            lane_synchronized <= 2'b00;
            payload_schedule_count <= '0;
            gearbox_phase <= '0;
            payload_crc <= 16'hffff;
            byte_frame_open <= 1'b0;
            byte_frame_id <= '0;
            byte_line_count <= '0;
            frame_start_event_byte <= 1'b0;
            frame_abort_event_byte <= 1'b0;
            packet_finished <= 1'b0;
            end_guard_active <= 1'b0;
            end_guard_count <= '0;
            parser_error_events <= '0;
        end else begin
            parser_error_events <= '0;
            frame_start_event_byte <= 1'b0;
            frame_abort_event_byte <= 1'b0;

            for (integer lane = 0; lane < 2; lane++) begin
                if (burst_start_pulse[lane]) begin
                    lane_discard[lane] <= 1'b0;
                    lane_synchronized[lane] <= 1'b0;
                    packet_finished <= 1'b0;
                    parser_state <= PARSER_WAIT_SYNC;
                    end_guard_active <= 1'b0;
                end
                if (lane_sync_found[lane])
                    lane_synchronized[lane] <= 1'b1;
            end

            if (|burst_end_pulse) begin
                end_guard_active <= 1'b1;
                end_guard_count <= '0;
            end else if (end_guard_active) begin
                if (end_guard_count == 3'd7) begin
                    end_guard_active <= 1'b0;
                    lane_discard <= 2'b11;
                    lane_synchronized <= 2'b00;
                    if (!packet_finished) begin
                        parser_error_events[STATUS_PACKET_LENGTH_ERROR] <= 1'b1;
                        if (byte_frame_open) begin
                            byte_frame_open <= 1'b0;
                            frame_abort_event_byte <= 1'b1;
                        end
                    end
                end else begin
                    end_guard_count <= end_guard_count + 1'b1;
                end
            end

            if (|lane_sync_error) begin
                parser_error_events[STATUS_SOT_SYNC_ERROR] <= 1'b1;
                lane_discard <= 2'b11;
                parser_state <= PARSER_DISCARD;
                if (byte_frame_open) begin
                    byte_frame_open <= 1'b0;
                    frame_abort_event_byte <= 1'b1;
                end
            end
            if (|lane_queue_overflow) begin
                parser_error_events[STATUS_LANE_ALIGNMENT_ERROR] <= 1'b1;
                lane_discard <= 2'b11;
                parser_state <= PARSER_DISCARD;
                if (byte_frame_open) begin
                    byte_frame_open <= 1'b0;
                    frame_abort_event_byte <= 1'b1;
                end
            end

            if (gearbox_write_valid && byte_frame_open &&
                    pixel_fifo_full) begin
                parser_error_events[STATUS_PIXEL_FIFO_OVERFLOW] <= 1'b1;
                byte_frame_open <= 1'b0;
                frame_abort_event_byte <= 1'b1;
            end

            if (parser_pair_pop) begin
                case (parser_state)
                    PARSER_WAIT_SYNC: begin
                        parser_state <= PARSER_HEADER_SECOND;
                    end
                    PARSER_HEADER_SECOND: begin
                        parser_state <= PARSER_HEADER_CHECK;
                    end
                    PARSER_PAYLOAD: begin
                        payload_crc <= payload_crc_pair;
                        if (gearbox_phase == 3'd4) begin
                            gearbox_phase <= 3'd0;
                            if (payload_schedule_count ==
                                    8'(PAYLOAD_SCHEDULES - 1)) begin
                                parser_state <= PARSER_FOOTER;
                                payload_schedule_count <= '0;
                            end else begin
                                payload_schedule_count <=
                                    parser_count_increment[7:0];
                            end
                        end else begin
                            gearbox_phase <= gearbox_phase + 1'b1;
                        end
                    end
                    default: begin
                        if ({lane1_byte, lane0_byte} != payload_crc)
                            parser_error_events[STATUS_PAYLOAD_CRC_ERROR] <= 1'b1;
                        byte_line_count <= parser_count_increment;
                        packet_finished <= 1'b1;
                        lane_discard <= 2'b11;
                        parser_state <= PARSER_DISCARD;
                    end
                endcase
            end

            if (parser_state == PARSER_HEADER_CHECK) begin
                if (header_is_uncorrectable) begin
                    parser_error_events[STATUS_HEADER_ECC_UNCORRECTABLE] <= 1'b1;
                    if (byte_frame_open) begin
                        byte_frame_open <= 1'b0;
                        frame_abort_event_byte <= 1'b1;
                    end
                    packet_finished <= 1'b1;
                    lane_discard <= 2'b11;
                    parser_state <= PARSER_DISCARD;
                end else begin
                    if (header_was_corrected)
                        parser_error_events[STATUS_HEADER_ECC_CORRECTED] <= 1'b1;
                    if (corrected_virtual_channel != 0) begin
                        parser_error_events[STATUS_UNEXPECTED_PACKET] <= 1'b1;
                        packet_finished <= 1'b1;
                        lane_discard <= 2'b11;
                        parser_state <= PARSER_DISCARD;
                    end else if (corrected_data_type == DT_FRAME_START) begin
                        if (byte_frame_open || pixel_epoch_busy) begin
                            parser_error_events[STATUS_FRAME_SEQUENCE_ERROR] <= 1'b1;
                            if (byte_frame_open) begin
                                byte_frame_open <= 1'b0;
                                frame_abort_event_byte <= 1'b1;
                            end
                        end else begin
                            byte_frame_open <= 1'b1;
                            byte_frame_id <= corrected_data_field;
                            byte_line_count <= '0;
                            frame_start_event_byte <= 1'b1;
                        end
                        packet_finished <= 1'b1;
                        lane_discard <= 2'b11;
                        parser_state <= PARSER_DISCARD;
                    end else if (corrected_data_type == DT_FRAME_END) begin
                        if (!byte_frame_open ||
                                corrected_data_field != byte_frame_id) begin
                            parser_error_events[STATUS_FRAME_SEQUENCE_ERROR] <= 1'b1;
                        end
                        if (byte_line_count != 10'(IMAGE_HEIGHT)) begin
                            parser_error_events[STATUS_LINE_COUNT_ERROR] <= 1'b1;
                        end
                        if (byte_frame_open &&
                                (corrected_data_field != byte_frame_id ||
                                byte_line_count != 10'(IMAGE_HEIGHT))) begin
                            frame_abort_event_byte <= 1'b1;
                        end
                        byte_frame_open <= 1'b0;
                        packet_finished <= 1'b1;
                        lane_discard <= 2'b11;
                        parser_state <= PARSER_DISCARD;
                    end else if (corrected_data_type == DT_RAW10) begin
                        if (corrected_data_field !=
                                16'(LINE_PAYLOAD_BYTES)) begin
                            parser_error_events[STATUS_WORD_COUNT_ERROR] <= 1'b1;
                            if (byte_frame_open) begin
                                byte_frame_open <= 1'b0;
                                frame_abort_event_byte <= 1'b1;
                            end
                            packet_finished <= 1'b1;
                            lane_discard <= 2'b11;
                            parser_state <= PARSER_DISCARD;
                        end else if (!byte_frame_open) begin
                            parser_error_events[STATUS_FRAME_SEQUENCE_ERROR] <= 1'b1;
                            packet_finished <= 1'b1;
                            lane_discard <= 2'b11;
                            parser_state <= PARSER_DISCARD;
                        end else if (byte_line_count >= 10'(IMAGE_HEIGHT)) begin
                            parser_error_events[STATUS_LINE_COUNT_ERROR] <= 1'b1;
                            parser_error_events[STATUS_FRAME_SEQUENCE_ERROR] <= 1'b1;
                            byte_frame_open <= 1'b0;
                            frame_abort_event_byte <= 1'b1;
                            packet_finished <= 1'b1;
                            lane_discard <= 2'b11;
                            parser_state <= PARSER_DISCARD;
                        end else begin
                            payload_schedule_count <= '0;
                            gearbox_phase <= '0;
                            payload_crc <= 16'hffff;
                            parser_state <= PARSER_PAYLOAD;
                        end
                    end else begin
                        parser_error_events[STATUS_UNEXPECTED_PACKET] <= 1'b1;
                        packet_finished <= 1'b1;
                        lane_discard <= 2'b11;
                        parser_state <= PARSER_DISCARD;
                    end
                end
            end
        end
    end

    // Every consumed pair advances one common two-pair history. Header
    // inspection follows two writes; RAW10 first output follows two payload
    // writes. Byte-order conversion for the header is wiring only.
    always_ff @(posedge byte_clk) begin
        if (parser_pair_pop)
            packet_buffer <= {packet_buffer[15:0], lane0_byte, lane1_byte};
    end

    // Sticky byte-domain status and clear-toggle handshake.
    logic status_clear_toggle_ctrl;
    logic status_clear_pending_ctrl;
    logic status_clear_ack_toggle_byte;
    logic status_clear_ack_sync0;
    logic status_clear_ack_sync1;
    logic status_clear_sync0;
    logic status_clear_sync1;
    logic status_clear_seen;
    logic [15:0] byte_status_sticky;
    logic [15:0] lane_error_events;
    logic [15:0] byte_error_events;
    always_comb begin
        lane_error_events = 16'h0000;
        if (|lane_sync_error)
            lane_error_events[STATUS_SOT_SYNC_ERROR] = 1'b1;
        if (|lane_queue_overflow)
            lane_error_events[STATUS_LANE_ALIGNMENT_ERROR] = 1'b1;
        byte_error_events = parser_error_events | lane_error_events;
        if (pixel_fifo_overflow)
            byte_error_events[STATUS_PIXEL_FIFO_OVERFLOW] = 1'b1;
    end

    wire status_clear_accept = status_clear && !status_clear_pending_ctrl;

    always_ff @(posedge ctrl_clk or negedge reset_n) begin
        if (!reset_n) begin
            status_clear_toggle_ctrl <= 1'b0;
            status_clear_pending_ctrl <= 1'b0;
            status_clear_ack_sync0 <= 1'b0;
            status_clear_ack_sync1 <= 1'b0;
        end else begin
            status_clear_ack_sync0 <= status_clear_ack_toggle_byte;
            status_clear_ack_sync1 <= status_clear_ack_sync0;
            if (status_clear_accept) begin
                status_clear_toggle_ctrl <= ~status_clear_toggle_ctrl;
                status_clear_pending_ctrl <= 1'b1;
            end else if (status_clear_pending_ctrl &&
                    status_clear_ack_sync1 == status_clear_toggle_ctrl) begin
                status_clear_pending_ctrl <= 1'b0;
            end
        end
    end

    always_ff @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            status_clear_sync0 <= 1'b0;
            status_clear_sync1 <= 1'b0;
            status_clear_seen <= 1'b0;
            status_clear_ack_toggle_byte <= 1'b0;
            byte_status_sticky <= 16'h0000;
        end else begin
            status_clear_sync0 <= status_clear_toggle_ctrl;
            status_clear_sync1 <= status_clear_sync0;
            if (status_clear_sync1 != status_clear_seen) begin
                status_clear_seen <= status_clear_sync1;
                status_clear_ack_toggle_byte <= status_clear_sync1;
                byte_status_sticky <= byte_error_events;
            end else begin
                byte_status_sticky <= byte_status_sticky | byte_error_events;
            end
        end
    end

    logic [15:0] ctrl_status_sticky;
    logic [15:0] byte_status_sync0;
    logic [15:0] byte_status_sync1;
    logic [15:0] ctrl_error_events;
    logic [15:0] ctrl_status_next;
    always_comb begin
        ctrl_error_events = 16'h0000;
        if (clock_lp_error_pulse || data_lp_error_pulse)
            ctrl_error_events[STATUS_LP_SEQUENCE_ERROR] = 1'b1;
        if (ctrl_clock_stop_pulse)
            ctrl_error_events[STATUS_CLOCK_STOP_ERROR] = 1'b1;
        if (ctrl_init_error_pulse)
            ctrl_error_events[STATUS_INITIALIZATION_ERROR] = 1'b1;
        ctrl_status_next = status_clear_accept ? 16'h0000 : ctrl_status_sticky;
        ctrl_status_next = ctrl_status_next | ctrl_error_events;
    end

    always_ff @(posedge ctrl_clk or negedge reset_n) begin
        if (!reset_n) begin
            ctrl_status_sticky <= 16'h0000;
            byte_status_sync0 <= 16'h0000;
            byte_status_sync1 <= 16'h0000;
            status <= 16'h0000;
        end else begin
            byte_status_sync0 <= byte_status_sticky;
            byte_status_sync1 <= byte_status_sync0;
            ctrl_status_sticky <= ctrl_status_next;
            status <= (ctrl_status_next | byte_status_sync1) & 16'h3fff;
        end
    end

    // Frame event/abort toggles cross into the public pixel/core domain.
    logic frame_start_sync0;
    logic frame_start_sync1;
    logic frame_start_seen;
    logic frame_abort_sync0;
    logic frame_abort_sync1;
    logic frame_abort_seen;
    logic pixel_session_sync0;
    logic pixel_session_sync1;
    wire pixel_frame_start_pulse = frame_start_sync1 ^ frame_start_seen;
    wire pixel_frame_abort_pulse = frame_abort_sync1 ^ frame_abort_seen;

    logic pixel_frame_armed;
    logic pixel_abort_draining;
    logic pixel_normal_idle_wait;
    logic [2:0] pixel_idle_quiet;
    logic [9:0] pixel_x;
    logic [9:0] pixel_y;

    wire pixel_counter_reset = !pixel_reset_n || pixel_frame_abort_pulse ||
        (pixel_frame_armed && !pixel_session_sync1);
    wire pixel_counter_start = pixel_frame_start_pulse &&
        !pixel_abort_draining && !pixel_normal_idle_wait;
    wire pixel_counter_beat = pixel_frame_armed && pixel_fifo_read_valid &&
        !PACK_MSB8_MARKERS;
    wire pixel_counter_x_last = pixel_x == 10'(BEATS_PER_LINE - 1);
    wire pixel_counter_y_step = pixel_counter_beat && pixel_counter_x_last &&
        pixel_y != 10'(IMAGE_HEIGHT - 1);

    // In the original clocked block, an accepted beat follows frame-start
    // initialization. Its increment wins over a simultaneous start clear;
    // global/local reset and frame abort still win over both.
    Ov9281Counter #(.WIDTH(10), .USE_DSP(USE_DSP_COUNTERS)) pixel_x_counter (
        .clk(pixel_clk), .reset_n,
        .clear(pixel_counter_reset || (pixel_counter_beat && pixel_counter_x_last) ||
            (pixel_counter_start && !pixel_counter_beat)),
        .enable(pixel_counter_beat && !pixel_counter_x_last), .value(pixel_x)
    );
    Ov9281Counter #(.WIDTH(10), .USE_DSP(USE_DSP_COUNTERS)) pixel_y_counter (
        .clk(pixel_clk), .reset_n,
        .clear(pixel_counter_reset || (pixel_counter_start && !pixel_counter_y_step)),
        .enable(pixel_counter_y_step), .value(pixel_y)
    );

    assign pixel_fifo_read_ready = pixel_frame_armed || pixel_abort_draining ||
        pixel_normal_idle_wait;

    always_ff @(posedge pixel_clk or negedge reset_n) begin
        if (!reset_n) begin
            frame_start_sync0 <= 1'b0;
            frame_start_sync1 <= 1'b0;
            frame_start_seen <= 1'b0;
            frame_abort_sync0 <= 1'b0;
            frame_abort_sync1 <= 1'b0;
            frame_abort_seen <= 1'b0;
            pixel_session_sync0 <= 1'b0;
            pixel_session_sync1 <= 1'b0;
            pixel_frame_armed <= 1'b0;
            pixel_abort_draining <= 1'b0;
            pixel_normal_idle_wait <= 1'b0;
            pixel_idle_quiet <= '0;
            pixel_idle_ack_toggle <= 1'b0;
            pixel_data <= '0;
            pixel_valid <= 1'b0;
            pixel_sof <= 1'b0;
            pixel_sol <= 1'b0;
            pixel_eol <= 1'b0;
            pixel_eof <= 1'b0;
        end else begin
            frame_start_sync0 <= frame_start_toggle_byte;
            frame_start_sync1 <= frame_start_sync0;
            frame_start_seen <= frame_start_sync1;
            frame_abort_sync0 <= frame_abort_toggle_byte;
            frame_abort_sync1 <= frame_abort_sync0;
            frame_abort_seen <= frame_abort_sync1;
            pixel_session_sync0 <= session_enable_ctrl;
            pixel_session_sync1 <= pixel_session_sync0;

            pixel_valid <= 1'b0;
            pixel_sof <= 1'b0;
            pixel_sol <= 1'b0;
            pixel_eol <= 1'b0;
            pixel_eof <= 1'b0;

            if (!pixel_reset_n) begin
                pixel_frame_armed <= 1'b0;
                pixel_abort_draining <= 1'b0;
                pixel_normal_idle_wait <= 1'b0;
                pixel_idle_quiet <= '0;
            end else if (pixel_frame_abort_pulse ||
                    (pixel_frame_armed && !pixel_session_sync1)) begin
                pixel_frame_armed <= 1'b0;
                pixel_abort_draining <= 1'b1;
                pixel_normal_idle_wait <= 1'b0;
                pixel_idle_quiet <= '0;
            end else begin
                if (pixel_frame_start_pulse && !pixel_abort_draining &&
                        !pixel_normal_idle_wait) begin
                    pixel_frame_armed <= 1'b1;
                end

                if (pixel_frame_armed && pixel_fifo_read_valid) begin
                    pixel_valid <= 1'b1;
                    if (PACK_MSB8_MARKERS) begin
                        pixel_data <= {
                            pixel_fifo_read_data[19:12], 2'b00,
                            pixel_fifo_read_data[11:4], 2'b00
                        };
                        pixel_sof <= pixel_fifo_read_data[3];
                        pixel_sol <= pixel_fifo_read_data[2];
                        pixel_eol <= pixel_fifo_read_data[1];
                        pixel_eof <= pixel_fifo_read_data[0];
                        if (pixel_fifo_read_data[0]) begin
                            pixel_frame_armed <= 1'b0;
                            pixel_normal_idle_wait <= 1'b1;
                            pixel_idle_quiet <= '0;
                        end
                    end else begin
                        pixel_data <= pixel_fifo_read_data;
                        pixel_sof <= pixel_x == 0 && pixel_y == 0;
                        pixel_sol <= pixel_x == 0;
                        pixel_eol <= pixel_x == 10'(BEATS_PER_LINE - 1);
                        pixel_eof <= pixel_x == 10'(BEATS_PER_LINE - 1) &&
                            pixel_y == 10'(IMAGE_HEIGHT - 1);
                        if (pixel_x == 10'(BEATS_PER_LINE - 1) &&
                                pixel_y == 10'(IMAGE_HEIGHT - 1)) begin
                            pixel_frame_armed <= 1'b0;
                            pixel_normal_idle_wait <= 1'b1;
                            pixel_idle_quiet <= '0;
                        end
                    end
                end

                if (pixel_abort_draining || pixel_normal_idle_wait) begin
                    if (pixel_fifo_empty && !pixel_fifo_read_valid) begin
                        if (pixel_idle_quiet == 3'd4) begin
                            pixel_idle_ack_toggle <= ~pixel_idle_ack_toggle;
                            pixel_abort_draining <= 1'b0;
                            pixel_normal_idle_wait <= 1'b0;
                            pixel_idle_quiet <= '0;
                        end else begin
                            pixel_idle_quiet <= pixel_idle_quiet + 1'b1;
                        end
                    end else begin
                        pixel_idle_quiet <= '0;
                    end
                end
            end
        end
    end

`ifndef SYNTHESIS
    initial begin
        assert (INIT_GUARD_CYCLES > 0);
        assert (LPX_CYCLES > 0);
        assert (SOT_SETTLE_CYCLES > 0);
        assert (CLOCK_TIMEOUT_CYCLES > 0);
        assert (PIXEL_FIFO_DEPTH >= 64)
            else $error("OV9281 supported profiles require at least 64 pixel FIFO entries");
    end

    always_ff @(posedge byte_clk) begin
        if (byte_session_reset_n) begin
            assert ((data_fifo_rd & data_fifo_empty) == 0)
                else $error("hard MIPI FIFO read while empty");
            assert (!(parser_pair_pop && !pair_available));
        end
    end
`endif
endmodule

`default_nettype wire
