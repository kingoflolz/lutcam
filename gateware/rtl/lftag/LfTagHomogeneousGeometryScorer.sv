// Integrated divide-free LFTag geometry and sixteen-hypothesis scorer.
//
// The four selected corners define projective barycentric cross products
// T=(B-A)x(D-A), R=(C-A)x(D-A), and S=(B-A)x(C-A).  For each non-key point,
// A_p=(P-A)x(D-A) and B_p=(B-A)x(P-A).  A candidate-wide Q15 normalization
// keeps every cross product on one common scale.  The unit-square homogeneous
// coordinates are then
//
//   U = -(T-R-S) S A_p
//   V = -(T-R-S) R B_p
//   Z = R S T + S(S-T)A_p + R(R-T)B_p.
//
// Applying a canonical hypothesis to (U,V,Z) needs no reciprocal.  Cell and
// symbol decisions are signed cross-comparisons against Z.  The per-point
// squared residual is accumulated on that common candidate scale for both
// hypothesis ranking and publication. The winning unsaturated residual and
// sum(W*W) are also retained for scale-independent acceptance in the caller.
// The saturating ranking score preserves deterministic first-minimum ordering.
//
// Nine DSP blocks perform signed arithmetic: one cross product, two shared
// homogeneous products, three shared projection products, one residual
// square-accumulator, and the two-block signed W/12 residual-offset MAC.
// Projection W overlaps point retirement, and Y overlaps the X response;
// priming W once per hypothesis adds sixteen clocks per complete search.
module LfTagHomogeneousGeometryScorer (
    input  logic                 clk,
    input  logic                 rst,
    input  logic                 clear,

    input  logic                 start_valid,
    output logic                 start_ready,
    input  logic [1:0]           in_scale,
    input  logic [71:0]          corner_x_flat,
    input  logic [71:0]          corner_y_flat,
    input  logic [19:0]          point_input_index_flat,

    output logic                 descriptor_req_valid,
    input  logic                 descriptor_req_ready,
    output logic [3:0]           descriptor_req_index,
    input  logic                 descriptor_rsp_valid,
    input  logic signed [17:0]   descriptor_rsp_x,
    input  logic signed [17:0]   descriptor_rsp_y,

    output logic                 geometry_done_valid,
    output logic                 geometry_done_invalid,
    output logic signed [17:0]   map_cden,
    output logic signed [17:0]   map_cg,
    output logic signed [17:0]   map_ch,

    output logic                 out_valid,
    input  logic                 out_ready,
    output logic                 out_best_valid,
    output logic                 out_input_invalid,
    output logic [3:0]           out_best_hypothesis,
    output logic [13:0]          out_best_id,
    output logic [14:0]          out_best_cell_map,
    output logic [63:0]          out_best_score_q32,
    output logic [37:0]          out_best_residual_sum,
    output logic [38:0]          out_best_weight_sum,
    output logic [9:0]           out_cycles
);
    logic [14:0] hypothesis_cell_map;
    typedef enum logic [5:0] {
        S_IDLE,
        S_DELTA_B,
        S_DELTA_C,
        S_DELTA_D,
        S_CROSS_READ_0,
        S_CROSS_MULTIPLY_0,
        S_CROSS_MULTIPLY_1,
        S_CROSS_RESULT,
        S_PREPARE_D0,
        S_PREPARE_D_READ,
        S_PREPARE_D1,
        S_PREPARE_G,
        S_PREPARE_H,
        S_MAP_PRELOAD,
        S_MAP_PARTIAL,
        S_MAP_PREPARE,
        S_MAP_DEN,
        S_MAP_G,
        S_MAP_H,
        S_NORMALIZE_CROSS,
        S_CONSTANT_RS_KU,
        S_CONSTANT_K0_KV,
        S_CONSTANT_KA_KB,
        S_DESCRIPTOR_REQ,
        S_DESCRIPTOR_WAIT,
        S_POINT_COEFFICIENT_READ,
        S_POINT_SCALE,
        S_POINT_UV,
        S_POINT_TERMS,
        S_POINT_WRITE_0,
        S_POINT_WRITE_1,
        S_WAIT_MAP,
        S_COEFFICIENT_0,
        S_COEFFICIENT_1,
        S_FETCH_0,
        S_FETCH_1,
        S_PROJECT_X,
        S_PROJECT_Y,
        S_PROJECT_W,
        S_RESPONSE_X,
        S_RESPONSE_Y,
        S_RESPONSE_SCORE,
        S_HYPOTHESIS_FINISH,
        S_OUTPUT,
        S_INVALID
    } state_t;

    state_t state;

    typedef enum logic [2:0] {
        CROSS_T,
        CROSS_R,
        CROSS_S,
        CROSS_A,
        CROSS_B
    } cross_job_t;

    cross_job_t cross_job;

    wire signed [17:0] corner_x [0:3];
    wire signed [17:0] corner_y [0:3];
    generate
        for (genvar corner = 0; corner < 4; corner = corner + 1) begin : unpack_corners
            assign corner_x[corner] = corner_x_flat[18*corner +: 18];
            assign corner_y[corner] = corner_y_flat[18*corner +: 18];
        end
    endgenerate

    // A single true-dual-port store holds geometry scratch values, the two
    // point words, and both hypothesis-coefficient words.  These lifetimes
    // are disjoint, so the otherwise-unused top twelve addresses replace a
    // separate scratch RAM without adding ports.  The 128x20 organization
    // maps to two RAM10s.
    (* ram_style = "block", syn_ramstyle = "block_ram" *)
    logic [19:0] scorer_memory [0:127];
    logic [6:0] memory_address_0;
    logic [6:0] memory_address_1;
    logic [19:0] memory_q_0;
    logic [19:0] memory_q_1;
    logic point_write_enable;
    logic [3:0] point_write_address;
    logic [19:0] point_write_data_0;
    logic [19:0] point_write_data_1;
    logic memory_write_enable_0;
    logic memory_write_enable_1;
    logic [19:0] memory_write_data_0;
    logic [19:0] memory_write_data_1;

    localparam logic [6:0] G_B_DX = 7'd96;
    localparam logic [6:0] G_B_DY = 7'd97;
    localparam logic [6:0] G_C_DX = 7'd98;
    localparam logic [6:0] G_C_DY = 7'd99;
    localparam logic [6:0] G_D_DX = 7'd100;
    localparam logic [6:0] G_D_DY = 7'd101;
    localparam logic [6:0] G_P_DX = 7'd102;
    localparam logic [6:0] G_P_DY = 7'd103;
    localparam logic [6:0] G_KU = 7'd104;
    localparam logic [6:0] G_KV = 7'd105;
    localparam logic [6:0] G_KA = 7'd106;
    localparam logic [6:0] G_KB = 7'd107;
    localparam logic [6:0] G_MAP_TMP = 7'd108;
    localparam logic [6:0] G_MAP_G = 7'd109;
    localparam logic [6:0] G_MAP_H = 7'd110;
    localparam logic [6:0] G_MAP_D = 7'd111;

    initial begin : coefficient_init
        scorer_memory[32] = 20'h40158; scorer_memory[64] = 20'h3d600;
        scorer_memory[33] = 20'h00000; scorer_memory[65] = 20'h00000;
        scorer_memory[34] = 20'h42d50; scorer_memory[66] = 20'h42d50;
        scorer_memory[35] = 20'h58100; scorer_memory[67] = 20'h00000;
        scorer_memory[36] = 20'h40000; scorer_memory[68] = 20'h40000;
        scorer_memory[37] = 20'h01700; scorer_memory[69] = 20'h00000;
        scorer_memory[38] = 20'h431f8; scorer_memory[70] = 20'h46140;
        scorer_memory[39] = 20'h61800; scorer_memory[71] = 20'h00000;
        scorer_memory[40] = 20'h3d960; scorer_memory[72] = 20'h3d4a0;
        scorer_memory[41] = 20'hb3f00; scorer_memory[73] = 20'h00000;
        scorer_memory[42] = 20'h40158; scorer_memory[74] = 20'h42a00;
        scorer_memory[43] = 20'h00000; scorer_memory[75] = 20'h00000;
        scorer_memory[44] = 20'h3d408; scorer_memory[76] = 20'h3feb0;
        scorer_memory[45] = 20'ha9600; scorer_memory[77] = 20'h00000;
        scorer_memory[46] = 20'h40000; scorer_memory[78] = 20'h45e00;
        scorer_memory[47] = 20'h01700; scorer_memory[79] = 20'h00000;
        scorer_memory[48] = 20'h40000; scorer_memory[80] = 20'h3b200;
        scorer_memory[49] = 20'h02c00; scorer_memory[81] = 20'h00000;
        scorer_memory[50] = 20'h42808; scorer_memory[82] = 20'h3ff60;
        scorer_memory[51] = 20'h52c00; scorer_memory[83] = 20'h00000;
        scorer_memory[52] = 20'h400a8; scorer_memory[84] = 20'h3d600;
        scorer_memory[53] = 20'h00000; scorer_memory[85] = 20'h00000;
        scorer_memory[54] = 20'h42cb0; scorer_memory[86] = 20'h42950;
        scorer_memory[55] = 20'h5bf00; scorer_memory[87] = 20'h00000;
        scorer_memory[56] = 20'h3dbf8; scorer_memory[88] = 20'h3b298;
        scorer_memory[57] = 20'hb6d00; scorer_memory[89] = 20'h00000;
        scorer_memory[58] = 20'h40000; scorer_memory[90] = 20'h40000;
        scorer_memory[59] = 20'h02c00; scorer_memory[91] = 20'h00000;
        scorer_memory[60] = 20'h3d6a8; scorer_memory[92] = 20'h3d6a8;
        scorer_memory[61] = 20'hac100; scorer_memory[93] = 20'h00000;
        scorer_memory[62] = 20'h400a8; scorer_memory[94] = 20'h42a00;
        scorer_memory[63] = 20'h00000; scorer_memory[95] = 20'h00000;
    end

    (* syn_use_dsp = "yes" *) logic signed [39:0] cross_product_accumulator;
    logic signed [18:0] cross_multiply_a;
    logic signed [17:0] cross_multiply_b;
    logic signed [17:0] homogeneous_multiply_a [0:1];
    logic signed [17:0] homogeneous_multiply_b [0:1];
    (* syn_use_dsp = "yes" *) logic signed [35:0] homogeneous_multiply_p [0:1];
    logic signed [35:0] q_product_0;
    logic signed [35:0] q_product_1;
    logic q_product_0_fits;
    logic q_product_1_fits;
    logic signed [10:0] projection_xy_coefficient_u;
    logic signed [10:0] projection_xy_coefficient_v;
    logic projection_is_w;
    // Ten signed-18 residual squares need 38 unsigned bits in the worst case.
    // The extra sign bit keeps the DSP feedback accumulator non-negative.
    (* syn_use_dsp = "yes" *) logic signed [38:0] residual_score_accumulator;
    logic signed [22:0] axis_offset_numerator;

    logic signed [17:0] delta_source_x;
    logic signed [17:0] delta_source_y;
    logic signed [17:0] delta_result_x;
    logic signed [17:0] delta_result_y;
    logic signed [18:0] setup_lhs;
    logic signed [18:0] setup_rhs;
    logic signed [18:0] setup_difference;
    logic map_half_partial;
    logic map_half;
    logic map_negative;
    logic map_den_nonzero;
    logic signed [18:0] map_sign_value;
    logic signed [18:0] map_scaled_value;
    // The high bit is an intentional guard bit; map outputs use the exact
    // low-18 two's-complement result after the range preparation above.
    /* verilator lint_off UNUSEDSIGNAL */
    logic signed [18:0] map_sign_result;
    /* verilator lint_on UNUSEDSIGNAL */

    logic signed [17:0] q_t;
    logic signed [17:0] q_r;
    logic signed [17:0] q_s;
    logic signed [5:0] cross_shift;
    logic signed [17:0] q_d;
    logic signed [17:0] q_rs;
    logic signed [17:0] q_k0;

    logic [2:0] point_index;
    logic signed [5:0] point_shift_remaining;
    logic signed [17:0] point_u;
    logic signed [17:0] point_v;
    logic signed [19:0] point_z_sum;

    logic [3:0] score_hypothesis;
    logic [2:0] score_point;
    logic signed [17:0] score_u;
    logic signed [17:0] score_v;
    logic signed [10:0] coefficient_0;
    logic signed [5:0] coefficient_1;
    logic signed [5:0] coefficient_3;
    logic signed [10:0] coefficient_4;
    logic signed [5:0] coefficient_6;
    logic signed [5:0] coefficient_7;
    logic signed [19:0] projected_axis;
    logic signed [18:0] projected_w;

    logic [4:0] hypothesis_occupancy;
    logic [1:0] hypothesis_symbol_0;
    logic [1:0] hypothesis_symbol_1;
    logic [1:0] hypothesis_symbol_2;
    logic [1:0] hypothesis_symbol_3;
    logic [1:0] hypothesis_symbol_5;
    logic hypothesis_valid;
    logic [33:0] hypothesis_public_score;
    logic [33:0] best_public_score;

    logic response_legal;
    logic [1:0] response_region_x;
    logic [2:0] response_cell;
    logic [1:0] response_symbol;
    logic decoded_axis_legal;
    logic [1:0] decoded_axis_region;
    logic decoded_axis_symbol;
    logic signed [17:0] decoded_axis_error;
    logic response_conflict;
    logic decoded_legal;
    logic [2:0] decoded_cell;

    logic map_normalize_active;
    logic map_ready;
    logic invalid_accumulator;
    logic [1:0] work_scale;
    logic [9:0] cycle_counter;

    function automatic logic [3:0] input_index_for_point(input logic [2:0] index);
        case (index)
            3'd0: input_index_for_point = point_input_index_flat[3:0];
            3'd1: input_index_for_point = point_input_index_flat[7:4];
            3'd2: input_index_for_point = point_input_index_flat[11:8];
            3'd3: input_index_for_point = point_input_index_flat[15:12];
            default: input_index_for_point = point_input_index_flat[19:16];
        endcase
    endfunction

    always_comb begin : memory_addresses
        logic [2:0] read_point;
        logic read_group;
        logic [3:0] point_address;
        logic [4:0] coefficient_address;

        read_point = score_point;
        read_group = (state != S_FETCH_0);
        if ((state == S_RESPONSE_X) || (state == S_RESPONSE_Y)
                || (state == S_RESPONSE_SCORE)) begin
            read_point = (score_point == 3'd4)
                ? 3'd0 : score_point + 1'b1;
            read_group = (state != S_RESPONSE_X);
        end
        point_address = {read_point, read_group};
        coefficient_address = {score_hypothesis,
            (state == S_COEFFICIENT_1)};

        point_write_enable = (state == S_POINT_WRITE_0)
            || (state == S_POINT_WRITE_1);
        point_write_address = {point_index, (state == S_POINT_WRITE_1)};
        if (state == S_POINT_WRITE_1) begin
            point_write_data_0 = {2'b0, q_t};
        end else begin
            point_write_data_0 = {2'b0, point_u};
        end
        point_write_data_1 = {2'b0, point_v};
        memory_write_enable_0 = point_write_enable;
        // The odd word on port one formerly retained an input-index map that
        // no direct-pose consumer used. Only the even V word is live.
        memory_write_enable_1 = (state == S_POINT_WRITE_0);
        memory_write_data_0 = point_write_data_0;
        memory_write_data_1 = point_write_data_1;

        memory_address_0 = {3'b000, point_address};
        memory_address_1 = {2'b00, 1'b1, point_address};
        if ((state == S_COEFFICIENT_0)
                || (state == S_COEFFICIENT_1)) begin
            memory_address_0 = {2'b01, coefficient_address};
            memory_address_1 = {2'b10, coefficient_address};
        end else if (point_write_enable) begin
            memory_address_0 = {3'b000, point_write_address};
            memory_address_1 = {2'b00, 1'b1, point_write_address};
        end

        case (state)
            S_DELTA_B: begin
                memory_address_0 = G_B_DX;
                memory_address_1 = G_B_DY;
                memory_write_enable_0 = 1'b1;
                memory_write_enable_1 = 1'b1;
                memory_write_data_0 = {2'b0, delta_result_x};
                memory_write_data_1 = {2'b0, delta_result_y};
            end
            S_DELTA_C: begin
                memory_address_0 = G_C_DX;
                memory_address_1 = G_C_DY;
                memory_write_enable_0 = 1'b1;
                memory_write_enable_1 = 1'b1;
                memory_write_data_0 = {2'b0, delta_result_x};
                memory_write_data_1 = {2'b0, delta_result_y};
            end
            S_DELTA_D: begin
                memory_address_0 = G_D_DX;
                memory_address_1 = G_D_DY;
                memory_write_enable_0 = 1'b1;
                memory_write_enable_1 = 1'b1;
                memory_write_data_0 = {2'b0, delta_result_x};
                memory_write_data_1 = {2'b0, delta_result_y};
            end
            S_DESCRIPTOR_WAIT: begin
                memory_address_0 = G_P_DX;
                memory_address_1 = G_P_DY;
                memory_write_enable_0 = descriptor_rsp_valid;
                memory_write_enable_1 = descriptor_rsp_valid;
                memory_write_data_0 = {2'b0, delta_result_x};
                memory_write_data_1 = {2'b0, delta_result_y};
            end
            S_CROSS_READ_0: begin
                case (cross_job)
                    CROSS_T: begin
                        memory_address_0 = G_B_DX;
                        memory_address_1 = G_D_DY;
                    end
                    CROSS_R: begin
                        memory_address_0 = G_C_DX;
                        memory_address_1 = G_D_DY;
                    end
                    CROSS_S: begin
                        memory_address_0 = G_B_DX;
                        memory_address_1 = G_C_DY;
                    end
                    CROSS_A: begin
                        memory_address_0 = G_P_DX;
                        memory_address_1 = G_D_DY;
                    end
                    default: begin
                        memory_address_0 = G_B_DX;
                        memory_address_1 = G_P_DY;
                    end
                endcase
            end
            S_CROSS_MULTIPLY_0: begin
                case (cross_job)
                    CROSS_T: begin
                        memory_address_0 = G_B_DY;
                        memory_address_1 = G_D_DX;
                    end
                    CROSS_R: begin
                        memory_address_0 = G_C_DY;
                        memory_address_1 = G_D_DX;
                    end
                    CROSS_S: begin
                        memory_address_0 = G_B_DY;
                        memory_address_1 = G_C_DX;
                    end
                    CROSS_A: begin
                        memory_address_0 = G_P_DY;
                        memory_address_1 = G_D_DX;
                    end
                    default: begin
                        memory_address_0 = G_B_DY;
                        memory_address_1 = G_P_DX;
                    end
                endcase
            end
            S_PREPARE_D0: begin
                memory_address_0 = G_MAP_TMP;
                memory_write_enable_0 = 1'b1;
                memory_write_data_0 = {setup_difference[18],
                    setup_difference};
            end
            S_PREPARE_D_READ: begin
                memory_address_0 = G_MAP_TMP;
            end
            S_PREPARE_D1: begin
                memory_address_0 = G_MAP_D;
                memory_write_enable_0 = 1'b1;
                memory_write_data_0 = {setup_difference[18],
                    setup_difference};
            end
            S_PREPARE_G: begin
                memory_address_0 = G_MAP_G;
                memory_write_enable_0 = 1'b1;
                memory_write_data_0 = {setup_difference[18],
                    setup_difference};
            end
            S_PREPARE_H: begin
                memory_address_0 = G_MAP_H;
                memory_write_enable_0 = 1'b1;
                memory_write_data_0 = {setup_difference[18],
                    setup_difference};
            end
            S_MAP_PRELOAD: begin
                memory_address_0 = G_MAP_D;
                memory_address_1 = G_MAP_G;
            end
            S_MAP_PARTIAL,
            S_MAP_PREPARE: begin
                memory_address_0 = G_MAP_D;
                memory_address_1 = G_MAP_H;
            end
            S_MAP_DEN,
            S_MAP_G,
            S_MAP_H: begin
                memory_address_0 = G_MAP_G;
                memory_address_1 = G_MAP_H;
            end
            S_CONSTANT_RS_KU: begin
                memory_address_0 = G_KU;
                memory_write_enable_0 = 1'b1;
                memory_write_data_0 = {2'b0, q_product_1[17:0]};
            end
            S_CONSTANT_K0_KV: begin
                memory_address_0 = G_KV;
                memory_write_enable_0 = 1'b1;
                memory_write_data_0 = {2'b0, q_product_1[17:0]};
            end
            S_CONSTANT_KA_KB: begin
                memory_address_0 = G_KA;
                memory_address_1 = G_KB;
                memory_write_enable_0 = 1'b1;
                memory_write_enable_1 = 1'b1;
                memory_write_data_0 = {2'b0, q_product_0[17:0]};
                memory_write_data_1 = {2'b0, q_product_1[17:0]};
            end
            S_POINT_COEFFICIENT_READ: begin
                memory_address_0 = G_KU;
                memory_address_1 = G_KV;
            end
            S_POINT_UV: begin
                memory_address_0 = G_KA;
                memory_address_1 = G_KB;
            end
            default: begin
            end
        endcase
    end

    always_ff @(posedge clk) begin : memories
        memory_q_0 <= scorer_memory[memory_address_0];
        memory_q_1 <= scorer_memory[memory_address_1];
        if (memory_write_enable_0)
            scorer_memory[memory_address_0] <= memory_write_data_0;
        if (memory_write_enable_1)
            scorer_memory[memory_address_1] <= memory_write_data_1;
    end

    always_comb begin : shared_coordinate_delta
        delta_source_x = descriptor_rsp_x;
        delta_source_y = descriptor_rsp_y;
        case (state)
            S_DELTA_B: begin
                delta_source_x = corner_x[1];
                delta_source_y = corner_y[1];
            end
            S_DELTA_C: begin
                delta_source_x = corner_x[2];
                delta_source_y = corner_y[2];
            end
            S_DELTA_D: begin
                delta_source_x = corner_x[3];
                delta_source_y = corner_y[3];
            end
            default: begin
            end
        endcase
        delta_result_x = delta_source_x - corner_x[0];
        delta_result_y = delta_source_y - corner_y[0];
    end

    always_comb begin : shared_setup_arithmetic
        // These operands are consumed only by the four PREPARE states. Keep
        // their common selection outside those states to avoid zero-gating
        // an otherwise unused combinational subtractor.
        setup_lhs = {q_s[17], q_s};
        setup_rhs = {q_t[17], q_t};
        case (state)
            S_PREPARE_D0: begin
                setup_lhs = {q_t[17], q_t};
                setup_rhs = {q_r[17], q_r};
            end
            S_PREPARE_D1: begin
                setup_lhs = memory_q_0[18:0];
                setup_rhs = {q_s[17], q_s};
            end
            S_PREPARE_G: begin
                setup_lhs = {q_s[17], q_s};
                setup_rhs = {q_t[17], q_t};
            end
            S_PREPARE_H: begin
                setup_lhs = {q_r[17], q_r};
                setup_rhs = {q_t[17], q_t};
            end
            default: begin
            end
        endcase
        setup_difference = setup_lhs - setup_rhs;

        case (state)
            S_MAP_G: map_sign_value = memory_q_0[18:0];
            S_MAP_H: map_sign_value = memory_q_1[18:0];
            default: map_sign_value = memory_q_0[18:0];
        endcase
        map_scaled_value = map_half
            ? (map_sign_value >>> 1) : map_sign_value;
        map_sign_result = map_negative
            ? -map_scaled_value : map_scaled_value;
    end

    always_comb begin : homogeneous_multiplier_operands
        // The multipliers are sampled only in CONSTANT and POINT states.
        // Their common POINT selection needs no zero-gating while idle.
        homogeneous_multiply_a[0] = memory_q_0[17:0];
        homogeneous_multiply_a[1] = memory_q_1[17:0];
        homogeneous_multiply_b[0] = q_t;
        homogeneous_multiply_b[1] = q_r;
        case (state)
            S_CONSTANT_RS_KU: begin
                homogeneous_multiply_a[0] = q_r;
                homogeneous_multiply_b[0] = q_s;
                homogeneous_multiply_a[1] = -q_d;
                homogeneous_multiply_b[1] = q_s;
            end
            S_CONSTANT_K0_KV: begin
                homogeneous_multiply_a[0] = q_rs;
                homogeneous_multiply_b[0] = q_t;
                homogeneous_multiply_a[1] = -q_d;
                homogeneous_multiply_b[1] = q_r;
            end
            S_CONSTANT_KA_KB: begin
                homogeneous_multiply_a[0] = q_s;
                homogeneous_multiply_b[0] = q_s - q_t;
                homogeneous_multiply_a[1] = q_r;
                homogeneous_multiply_b[1] = q_r - q_t;
            end
            S_POINT_UV: begin
                homogeneous_multiply_a[0] = memory_q_0[17:0];
                homogeneous_multiply_b[0] = q_t;
                homogeneous_multiply_a[1] = memory_q_1[17:0];
                homogeneous_multiply_b[1] = q_r;
            end
            S_POINT_TERMS: begin
                homogeneous_multiply_a[0] = memory_q_0[17:0];
                homogeneous_multiply_b[0] = q_t;
                homogeneous_multiply_a[1] = memory_q_1[17:0];
                homogeneous_multiply_b[1] = q_r;
            end
            default: begin
            end
        endcase
    end

    // W for the following point is evaluated while RESPONSE_SCORE retires
    // this point. Its U/V and Z were prefetched in RESPONSE_X/RESPONSE_Y.
    // The first point warms up in PROJECT_W. PROJECT_X retains W while the
    // native result register becomes X, then RESPONSE_X replaces X with Y.
    assign projection_is_w = (state == S_PROJECT_W) || (state == S_RESPONSE_SCORE);
    assign projection_xy_coefficient_u = projection_is_w
        ? {{5{coefficient_6[5]}}, coefficient_6}
        : ((state == S_RESPONSE_X) ? {{5{coefficient_3[5]}}, coefficient_3} : coefficient_0);
    assign projection_xy_coefficient_v = projection_is_w
        ? {{5{coefficient_7[5]}}, coefficient_7}
        : ((state == S_RESPONSE_X) ? coefficient_4 : {{5{coefficient_1[5]}}, coefficient_1});
    wire [47:0] projection_xy_product_native;
    wire [47:0] projection_xy_sum_native;
    wire [47:0] projection_xy_result_native;
    // Ordinary simulators use the same exact 48-bit arithmetic without
    // requiring the vendor library. Efinity retains the native branch;
    // defining SYNTHESIS explicitly enables vendor-model regressions.
    localparam bit USE_NATIVE_PROJECTION = 1'b1
`ifndef SYNTHESIS
        // synthesis translate_off
        && 1'b0
        // synthesis translate_on
`endif
        ;
    generate if (USE_NATIVE_PROJECTION) begin : native_projection
    EFX_DSP48 #(.MODE("NORMAL"), .SIGNED(1), .M_SEL("P"),
        .N_SEL("CONST0"), .W_SEL("X"), .W_REG(0),
        .CASCOUT_SEL("P"), .W_REG_USE_RST(0)) projection_xy_dsp0 (
        .A(19'($signed(projection_xy_coefficient_u))), .B(18'($signed(score_u))), .C(18'b0),
        .CASCIN(48'd0), .OP(2'd0), .SHIFT_ENA(1'b0),
        .CLK(1'b0), .CE(1'b0), .RST(1'b0),
        .O(), .CASCOUT(projection_xy_product_native), .OVFL());
    EFX_DSP48 #(.MODE("NORMAL"), .SIGNED(1), .M_SEL("P"),
        .N_SEL("CASCIN"), .W_SEL("X"), .W_REG(0),
        .CASCOUT_SEL("W"), .W_REG_USE_RST(0)) projection_xy_dsp1 (
        .A(19'($signed(projection_xy_coefficient_v))), .B(18'($signed(score_v))), .C(18'b0),
        .CASCIN(projection_xy_product_native), .OP(2'd0), .SHIFT_ENA(1'b0),
        .CLK(1'b0), .CE(1'b0), .RST(1'b0),
        .O(), .CASCOUT(projection_xy_sum_native), .OVFL());
    EFX_DSP48 #(.MODE("NORMAL"), .SIGNED(1), .M_SEL("P"),
        .N_SEL("CASCIN"), .W_SEL("X"), .W_REG(1),
        .CASCOUT_SEL("W"), .W_REG_USE_RST(0)) projection_xy_dsp2 (
        .A(19'($signed(memory_q_0[17:0]))), .B(18'($signed(18'sd256))), .C(18'b0),
        .CASCIN(projection_xy_sum_native), .OP(projection_is_w ? 2'd0 : 2'd2), .SHIFT_ENA(1'b0),
        .CLK(clk), .CE(((state == S_PROJECT_X) || (state == S_RESPONSE_X) || projection_is_w) && !rst), .RST(1'b0),
        .O(projection_xy_result_native), .CASCOUT(), .OVFL());
    end else begin : portable_projection
        wire signed [47:0] product_u = $signed(projection_xy_coefficient_u)
            * $signed(score_u);
        wire signed [47:0] product_v = $signed(projection_xy_coefficient_v)
            * $signed(score_v);
        wire signed [47:0] product_z = $signed(memory_q_0[17:0]) * 48'sd256;
        logic signed [47:0] result;
        always_ff @(posedge clk) begin
            if (!rst && ((state == S_PROJECT_X) || (state == S_RESPONSE_X)
                    || projection_is_w))
                result <= product_u + product_v
                    + (projection_is_w ? product_z : -product_z);
        end
        assign projection_xy_result_native = result;
    end endgenerate
    assign projected_axis = projection_xy_result_native[8 +: 20];

    generate
        for (genvar lane = 0; lane < 2; lane = lane + 1) begin : homogeneous_dsp_lanes
            assign homogeneous_multiply_p[lane] = $signed(homogeneous_multiply_a[lane])
                * $signed(homogeneous_multiply_b[lane]);
        end
    endgenerate

    assign q_product_0 = homogeneous_multiply_p[0] >>> 15;
    assign q_product_1 = homogeneous_multiply_p[1] >>> 15;
    assign q_product_0_fits = q_product_0[35:17]
        == {19{q_product_0[17]}};
    assign q_product_1_fits = q_product_1[35:17]
        == {19{q_product_1[17]}};

    // A/B are consumed by the two products before this sum is registered.
    // Their shared q_t/q_r registers then hold Z[17:0] and Z[19:18]
    // respectively until the second point-memory write.
    assign point_z_sum =
        $signed({{2{q_k0[17]}},q_k0})
        + $signed({{2{q_product_0[17]}},q_product_0[17:0]})
        + $signed({{2{q_product_1[17]}},q_product_1[17:0]});

    always_comb begin : cross_multiplier_operand
        cross_multiply_a = $signed({memory_q_0[17], memory_q_0[17:0]});
        cross_multiply_b = $signed(memory_q_1[17:0]);
        if (state == S_CROSS_MULTIPLY_1)
            cross_multiply_a = -cross_multiply_a;
        else if (state == S_RESPONSE_X) begin
            cross_multiply_a = projected_w;
            cross_multiply_b = $signed({1'b0, projected_w[16:0]});
        end else if (state == S_RESPONSE_Y) begin
            cross_multiply_a = -projected_w;
            cross_multiply_b = projected_w[17] ? -18'sd131072 : 18'sd0;
        end
    end

    // Geometry and scoring are disjoint. Reuse the cross-product MAC for
    // sum(W*W), splitting positive signed-19 W into two signed-18 B operands.
    // Five full-range squares fit 39 unsigned bits plus the accumulator sign.
    always_ff @(posedge clk) begin : cross_product_mac
        if (rst || clear || (state == S_CROSS_READ_0)
                || (state == S_COEFFICIENT_0)) begin
            cross_product_accumulator <= 40'sb0;
        end else if ((state == S_CROSS_MULTIPLY_0)
                || (state == S_CROSS_MULTIPLY_1)
                || (state == S_RESPONSE_X) || (state == S_RESPONSE_Y)) begin
            cross_product_accumulator <= cross_product_accumulator
                + cross_multiply_a * cross_multiply_b;
        end
    end

    // X and Y residuals are produced on consecutive cycles.  Accumulating the
    // square immediately lets Efinity use the DSP's W feedback instead of two
    // square DSPs followed by a 35-bit fabric score adder.  Saturating only at
    // hypothesis completion is exact because every square is non-negative.
    always_ff @(posedge clk) begin : residual_score_mac
        if (rst || clear || (state == S_COEFFICIENT_0)) begin
            residual_score_accumulator <= 39'sb0;
        end else if ((state == S_RESPONSE_X) || (state == S_RESPONSE_Y)) begin
            residual_score_accumulator <= residual_score_accumulator
                + $signed(decoded_axis_error) * $signed(decoded_axis_error);
        end
    end

    always_comb begin : published_hypothesis_score
        hypothesis_public_score = (|residual_score_accumulator[38:34])
            ? 34'h3_ffff_ffff : residual_score_accumulator[33:0];
    end

    // W is genuinely signed-19; use that width at the native multiplier
    // instead of asking inference to multiply a sign-extended 23-bit input.
    // The low-23 truncation precedes the signed Q8 shift below, preserving
    // the original modular numerator even at the signed-19 extremes.
    // Reuse the existing native/portable selection for both DSP structures.
    generate if (USE_NATIVE_PROJECTION) begin : native_axis_offset
        wire [47:0] numerator;
        EFX_DSP48 #(.MODE("NORMAL"), .SIGNED(1),
            .A_REG(0), .B_REG(0), .C_REG(0), .P_REG(0),
            .OP_REG(0), .W_REG(0), .O_REG(0), .SHIFTER(0),
            .P_EXT("ALIGN_RIGHT"), .C_EXT("ALIGN_RIGHT"),
            .M_SEL("P"), .N_SEL("C"), .W_SEL("X")) offset_dsp (
            .A(projected_w), .B(18'sd21), .C(18'sd128),
            .CASCIN(48'b0), .OP(2'd0), .SHIFT_ENA(1'b0),
            .CLK(1'b0), .CE(1'b0), .RST(1'b0),
            .O(numerator), .CASCOUT(), .OVFL());
        assign axis_offset_numerator = numerator[22:0];
    end else begin : portable_axis_offset
        wire signed [22:0] axis_w_wide = {{4{projected_w[18]}}, projected_w};
        assign axis_offset_numerator =
            23'($signed(axis_w_wide) * 6'sd21 + 29'sd128);
    end endgenerate

    always_comb begin : response_axis_decode
        logic signed [20:0] axis_value;
        logic signed [20:0] w;
        logic signed [20:0] triple_w;
        logic signed [20:0] center;
        logic signed [20:0] expected;
        logic signed [20:0] error_wide;
        // Guard bits make the shift/add approximation explicit.  Proven
        // projection bounds keep the retained 21-bit quotient in range.
        /* verilator lint_off UNUSEDSIGNAL */
        logic signed [22:0] offset_wide;
        /* verilator lint_on UNUSEDSIGNAL */
        logic [20:0] offset;
        logic error_fits;

        axis_value = {{1{projected_axis[19]}}, projected_axis};
        w = {{2{projected_w[18]}}, projected_w};
        triple_w = w + (w <<< 1);

        decoded_axis_legal = (w > 0)
            && ((axis_value <<< 1) >= -triple_w)
            && ((axis_value <<< 1) <= triple_w);
        decoded_axis_region = 2'd1;
        center = 21'sb0;
        if ((axis_value <<< 1) < -w) begin
            decoded_axis_region = 2'd0;
            center = -w;
        end else if ((axis_value <<< 1) > w) begin
            decoded_axis_region = 2'd2;
            center = w;
        end

        decoded_axis_symbol = (axis_value >= center);
        // Q8 approximation of one twelfth.  It is shared by X and Y and
        // implemented as shifts/adds, not as a selector DSP.
        offset_wide = axis_offset_numerator >>> 8;
        offset = 21'(offset_wide);
        expected = center + (decoded_axis_symbol
            ? $signed(offset) : -$signed(offset));
        error_wide = axis_value - expected;
        error_fits = error_wide[20:17] == {4{error_wide[17]}};
        decoded_axis_error = error_wide[17:0];
        decoded_axis_legal = decoded_axis_legal && error_fits;
    end

    always_comb begin : response_cell_decode
        logic cell_pattern_legal;

        decoded_cell = 3'b0;
        cell_pattern_legal = 1'b1;
        case ({response_region_x, decoded_axis_region})
            4'b0100: decoded_cell = 3'd0;
            4'b0001: decoded_cell = 3'd1;
            4'b0101: decoded_cell = 3'd2;
            4'b1001: decoded_cell = 3'd3;
            4'b0010: decoded_cell = 3'd4;
            4'b0110: decoded_cell = 3'd5;
            4'b1010: decoded_cell = 3'd6;
            default: cell_pattern_legal = 1'b0;
        endcase

        response_conflict = 1'b0;
        case (decoded_cell)
            3'd0: response_conflict = hypothesis_occupancy[0];
            3'd1: response_conflict = hypothesis_occupancy[1];
            3'd2: response_conflict = hypothesis_occupancy[2];
            3'd3: response_conflict = hypothesis_occupancy[3];
            3'd5: response_conflict = hypothesis_occupancy[4];
            default: response_conflict = 1'b0;
        endcase
        decoded_legal = response_legal && decoded_axis_legal
            && cell_pattern_legal
            && (decoded_cell != 3'd4) && (decoded_cell != 3'd6)
            && !response_conflict;
    end

    assign start_ready = (state == S_IDLE);
    assign descriptor_req_valid = (state == S_DESCRIPTOR_REQ);
    assign descriptor_req_index = input_index_for_point(point_index);
    assign out_valid = (state == S_OUTPUT);
    // The saved unsaturated winner already contains the public score. Derive
    // its saturation rather than retaining a second copy of its low 34 bits.
    assign best_public_score = !out_best_valid || (|out_best_residual_sum[37:34])
        ? 34'h3_ffff_ffff : out_best_residual_sum[33:0];
    assign out_best_score_q32 = {30'b0, best_public_score};

    always_ff @(posedge clk) begin : state_machine
        logic signed [36:0] cross_scaled;
        logic [4:0] occupancy_total;

        case (work_scale)
            2'd0: cross_scaled = 37'(cross_product_accumulator >>> 8);
            2'd1: cross_scaled = 37'(cross_product_accumulator >>> 4);
            default: cross_scaled = cross_product_accumulator[36:0];
        endcase
        occupancy_total = hypothesis_occupancy;
        case (response_cell)
            3'd0: occupancy_total[0] = 1'b1;
            3'd1: occupancy_total[1] = 1'b1;
            3'd2: occupancy_total[2] = 1'b1;
            3'd3: occupancy_total[3] = 1'b1;
            3'd5: occupancy_total[4] = 1'b1;
            default: begin end
        endcase

        if (rst) begin
            state <= S_IDLE;
            geometry_done_valid <= 1'b0;
            geometry_done_invalid <= 1'b0;
            map_cden <= 18'sb0;
            map_cg <= 18'sb0;
            map_ch <= 18'sb0;
            map_normalize_active <= 1'b0;
            map_ready <= 1'b0;
            invalid_accumulator <= 1'b0;
            work_scale <= 2'b0;
            out_best_valid <= 1'b0;
            out_input_invalid <= 1'b0;
            out_best_hypothesis <= 4'b0;
            out_best_id <= 14'b0;
            out_best_cell_map <= 0;
            hypothesis_cell_map <= 0;
            out_best_residual_sum <= 38'b0;
            out_best_weight_sum <= 39'b0;
            point_index <= 3'b0;
            score_hypothesis <= 4'b0;
            score_point <= 3'b0;
            cycle_counter <= 10'b0;
            out_cycles <= 10'b0;
            hypothesis_occupancy <= 5'b0;
            hypothesis_valid <= 1'b0;
            hypothesis_symbol_0 <= 2'b0;
            hypothesis_symbol_1 <= 2'b0;
            hypothesis_symbol_2 <= 2'b0;
            hypothesis_symbol_3 <= 2'b0;
            hypothesis_symbol_5 <= 2'b0;
            cross_job <= CROSS_T;
        end else begin
            if ((state != S_IDLE) && (state != S_OUTPUT)
                    && (state != S_INVALID))
                cycle_counter <= cycle_counter + 1'b1;

            if (map_normalize_active) begin
                if (map_cden[16] || map_cden[15]) begin
                    map_normalize_active <= 1'b0;
                    map_ready <= 1'b1;
                end else if (map_cden[17]
                        || (map_cg[17] != map_cg[16])
                        || (map_ch[17] != map_ch[16])) begin
                    invalid_accumulator <= 1'b1;
                    map_normalize_active <= 1'b0;
                end else begin
                    map_cden <= map_cden <<< 1;
                    map_cg <= map_cg <<< 1;
                    map_ch <= map_ch <<< 1;
                end
            end

            case (state)
                S_IDLE: begin
                    if (start_valid) begin
                        geometry_done_valid <= 1'b0;
                        geometry_done_invalid <= 1'b0;
                        map_normalize_active <= 1'b0;
                        map_ready <= 1'b0;
                        invalid_accumulator <= 1'b0;
                        work_scale <= in_scale;
                        out_best_valid <= 1'b0;
                        out_input_invalid <= 1'b0;
                        out_best_hypothesis <= 4'b0;
                        out_best_id <= 14'b0;
            out_best_cell_map <= 0;
            hypothesis_cell_map <= 0;
                        out_best_residual_sum <= 38'b0;
                        out_best_weight_sum <= 39'b0;
                        point_index <= 3'b0;
                        cycle_counter <= 10'b0;
                        state <= S_DELTA_B;
                    end
                end
                S_DELTA_B: begin
                    state <= S_DELTA_C;
                end
                S_DELTA_C: begin
                    state <= S_DELTA_D;
                end
                S_DELTA_D: begin
                    cross_job <= CROSS_T;
                    state <= S_CROSS_READ_0;
                end
                S_CROSS_READ_0: begin
                    state <= S_CROSS_MULTIPLY_0;
                end
                S_CROSS_MULTIPLY_0: begin
                    state <= S_CROSS_MULTIPLY_1;
                end
                S_CROSS_MULTIPLY_1: begin
                    state <= S_CROSS_RESULT;
                end
                S_CROSS_RESULT: begin
                    if (cross_scaled[36:17] != {20{cross_scaled[17]}}) begin
                        invalid_accumulator <= 1'b1;
                        state <= S_INVALID;
                    end else begin
                        case (cross_job)
                            CROSS_T: begin
                                q_t <= cross_scaled[17:0];
                                cross_job <= CROSS_R;
                                state <= S_CROSS_READ_0;
                            end
                            CROSS_R: begin
                                q_r <= cross_scaled[17:0];
                                cross_job <= CROSS_S;
                                state <= S_CROSS_READ_0;
                            end
                            CROSS_S: begin
                                q_s <= cross_scaled[17:0];
                                state <= S_PREPARE_D0;
                            end
                            CROSS_A: begin
                                // q_t is dead after the constant products;
                                // reuse it as the per-point A work register.
                                q_t <= cross_scaled[17:0];
                                cross_job <= CROSS_B;
                                state <= S_CROSS_READ_0;
                            end
                            default: begin
                                // q_r has the same disjoint lifetime for B.
                                q_r <= cross_scaled[17:0];
                                point_shift_remaining <= cross_shift;
                                state <= S_POINT_SCALE;
                            end
                        endcase
                    end
                end
                S_PREPARE_D0: begin
                    cross_shift <= 0;
                    state <= S_PREPARE_D_READ;
                end
                S_PREPARE_D_READ: begin
                    state <= S_PREPARE_D1;
                end
                S_PREPARE_D1: begin
                    state <= S_PREPARE_G;
                end
                S_PREPARE_G: begin
                    state <= S_PREPARE_H;
                end
                S_PREPARE_H: begin
                    state <= S_MAP_PRELOAD;
                end
                S_MAP_PRELOAD: begin
                    state <= S_MAP_PARTIAL;
                end
                S_MAP_PARTIAL: begin
                    map_half_partial <=
                        (memory_q_0[18] != memory_q_0[17])
                        || (memory_q_0[18]
                            && (($signed(memory_q_0[18:0])
                                    == -19'sd131072)
                                || ($signed(memory_q_1[18:0])
                                    == -19'sd131072)));
                    map_negative <= memory_q_0[18];
                    map_den_nonzero <= memory_q_0[18:0] != 0;
                    state <= S_MAP_PREPARE;
                end
                S_MAP_PREPARE: begin
                    map_half <= map_half_partial
                        || (map_negative
                            && ($signed(memory_q_1[18:0])
                                == -19'sd131072));
                    invalid_accumulator <= invalid_accumulator
                        | !map_den_nonzero;
                    state <= S_MAP_DEN;
                end
                S_MAP_DEN: begin
                    map_cden <= 18'(map_sign_result);
                    state <= S_MAP_G;
                end
                S_MAP_G: begin
                    map_cg <= 18'(map_sign_result);
                    state <= S_MAP_H;
                end
                S_MAP_H: begin
                    map_ch <= 18'(map_sign_result);
                    map_normalize_active <= map_den_nonzero;
                    state <= S_NORMALIZE_CROSS;
                end
                S_NORMALIZE_CROSS: begin
                    if ((q_t <= 0) || (q_r <= 0) || (q_s <= 0)) begin
                        invalid_accumulator <= 1'b1;
                        state <= S_INVALID;
                    end else if ((|q_t[17:15]) || (|q_r[17:15])
                            || (|q_s[17:15])) begin
                        q_t <= q_t >>> 1;
                        q_r <= q_r >>> 1;
                        q_s <= q_s >>> 1;
                        cross_shift <= cross_shift - 1'b1;
                    end else if (!q_t[14] && !q_r[14] && !q_s[14]) begin
                        q_t <= q_t <<< 1;
                        q_r <= q_r <<< 1;
                        q_s <= q_s <<< 1;
                        cross_shift <= cross_shift + 1'b1;
                    end else begin
                        q_d <= q_t - q_r - q_s;
                        state <= S_CONSTANT_RS_KU;
                    end
                end
                S_CONSTANT_RS_KU: begin
                    if (!q_product_0_fits || !q_product_1_fits) begin
                        invalid_accumulator <= 1'b1;
                        state <= S_INVALID;
                    end else begin
                        q_rs <= q_product_0[17:0];
                        state <= S_CONSTANT_K0_KV;
                    end
                end
                S_CONSTANT_K0_KV: begin
                    if (!q_product_0_fits || !q_product_1_fits) begin
                        invalid_accumulator <= 1'b1;
                        state <= S_INVALID;
                    end else begin
                        q_k0 <= q_product_0[17:0];
                        state <= S_CONSTANT_KA_KB;
                    end
                end
                S_CONSTANT_KA_KB: begin
                    if (!q_product_0_fits || !q_product_1_fits) begin
                        invalid_accumulator <= 1'b1;
                        state <= S_INVALID;
                    end else if (q_k0 <= 0) begin
                        invalid_accumulator <= 1'b1;
                        state <= S_INVALID;
                    end else begin
                        point_index <= 0;
                        state <= S_DESCRIPTOR_REQ;
                    end
                end
                S_DESCRIPTOR_REQ: begin
                    if (descriptor_req_ready)
                        state <= S_DESCRIPTOR_WAIT;
                end
                S_DESCRIPTOR_WAIT: begin
                    if (descriptor_rsp_valid) begin
                        cross_job <= CROSS_A;
                        state <= S_CROSS_READ_0;
                    end
                end
                S_POINT_SCALE: begin
                    if (point_shift_remaining > 0) begin
                        if ((q_t[17] != q_t[16])
                                || (q_r[17] != q_r[16])) begin
                            invalid_accumulator <= 1'b1;
                            state <= S_INVALID;
                        end else begin
                            q_t <= q_t <<< 1;
                            q_r <= q_r <<< 1;
                            point_shift_remaining <= point_shift_remaining - 1'b1;
                        end
                    end else if (point_shift_remaining < 0) begin
                        q_t <= q_t >>> 1;
                        q_r <= q_r >>> 1;
                        point_shift_remaining <= point_shift_remaining + 1'b1;
                    end else begin
                        state <= S_POINT_COEFFICIENT_READ;
                    end
                end
                S_POINT_COEFFICIENT_READ: begin
                    state <= S_POINT_UV;
                end
                S_POINT_UV: begin
                    if (!q_product_0_fits || !q_product_1_fits) begin
                        invalid_accumulator <= 1'b1;
                        state <= S_INVALID;
                    end else begin
                        point_u <= q_product_0[17:0];
                        point_v <= q_product_1[17:0];
                        state <= S_POINT_TERMS;
                    end
                end
                S_POINT_TERMS: begin
                    if (!q_product_0_fits || !q_product_1_fits) begin
                        invalid_accumulator <= 1'b1;
                        state <= S_INVALID;
                    end else begin
                        q_t <= point_z_sum[17:0];
                        q_r <= {16'b0,point_z_sum[19:18]};
                        state <= S_POINT_WRITE_0;
                    end
                end
                S_POINT_WRITE_0: begin
                    if ({q_r[1:0],q_t[17]} != {3{q_t[17]}}) begin
                        invalid_accumulator <= 1'b1;
                        state <= S_INVALID;
                    end else begin
                        state <= S_POINT_WRITE_1;
                    end
                end
                S_POINT_WRITE_1: begin
                    if (point_index == 3'd4) begin
                        if (map_ready) begin
                            geometry_done_valid <= 1'b1;
                            geometry_done_invalid <= invalid_accumulator;
                            score_hypothesis <= 0;
                            score_point <= 0;
                            hypothesis_occupancy <= 0;
                            hypothesis_valid <= 1'b1;
                            state <= S_COEFFICIENT_0;
                        end else begin
                            state <= S_WAIT_MAP;
                        end
                    end else begin
                        point_index <= point_index + 1'b1;
                        state <= S_DESCRIPTOR_REQ;
                    end
                end
                S_WAIT_MAP: begin
                    if (invalid_accumulator) begin
                        state <= S_INVALID;
                    end else if (map_ready) begin
                        geometry_done_valid <= 1'b1;
                        geometry_done_invalid <= 1'b0;
                        score_hypothesis <= 0;
                        score_point <= 0;
                        hypothesis_occupancy <= 0;
                        hypothesis_valid <= 1'b1;
                        state <= S_COEFFICIENT_0;
                    end
                end
                S_COEFFICIENT_0: begin
                    state <= S_COEFFICIENT_1;
                end
                S_COEFFICIENT_1: begin
                    coefficient_0 <= memory_q_0[19:9];
                    coefficient_1 <= memory_q_0[8:3];
                    coefficient_4 <= memory_q_1[19:9];
                    coefficient_3 <= memory_q_1[8:3];
                    state <= S_FETCH_0;
                end
                S_FETCH_0: begin
                    coefficient_6 <= memory_q_0[19:14];
                    coefficient_7 <= memory_q_0[13:8];
                    state <= S_FETCH_1;
                end
                S_FETCH_1: begin
                    score_u <= memory_q_0[17:0];
                    score_v <= memory_q_1[17:0];
                    state <= S_PROJECT_W;
                end
                S_PROJECT_W: begin
                    state <= S_PROJECT_X;
                end
                S_PROJECT_X: begin
                    projected_w <= projected_axis[18:0];
                    state <= S_RESPONSE_X;
                end
                S_RESPONSE_X: begin
                    response_legal <= decoded_axis_legal;
                    response_region_x <= decoded_axis_region;
                    response_symbol[1] <= decoded_axis_symbol;
                    state <= S_RESPONSE_Y;
                end
                S_RESPONSE_Y: begin
                    response_legal <= decoded_legal;
                    response_cell <= decoded_cell;
                    response_symbol[0] <= decoded_axis_symbol;
                    score_u <= memory_q_0[17:0];
                    score_v <= memory_q_1[17:0];
                    state <= S_RESPONSE_SCORE;
                end
                S_RESPONSE_SCORE: begin
                    hypothesis_cell_map <= {response_cell,hypothesis_cell_map[14:3]};
                    if (!response_legal)
                        hypothesis_valid <= 1'b0;
                    else begin
                        hypothesis_occupancy <= occupancy_total;
                        case (response_cell)
                            3'd0: begin
                                hypothesis_symbol_0 <= response_symbol;
                            end
                            3'd1: begin
                                hypothesis_symbol_1 <= response_symbol;
                            end
                            3'd2: begin
                                hypothesis_symbol_2 <= response_symbol;
                            end
                            3'd3: begin
                                hypothesis_symbol_3 <= response_symbol;
                            end
                            default: begin
                                hypothesis_symbol_5 <= response_symbol;
                            end
                        endcase
                    end

                    if (score_point == 3'd4) begin
                        state <= S_HYPOTHESIS_FINISH;
                    end else begin
                        score_point <= score_point + 1'b1;
                        state <= S_PROJECT_X;
                    end
                end
                S_HYPOTHESIS_FINISH: begin
                    if (hypothesis_valid && (&hypothesis_occupancy)) begin
                        if (!out_best_valid || (hypothesis_public_score
                                < best_public_score)) begin
                            out_best_valid <= 1'b1;
                            out_best_hypothesis <= score_hypothesis;
                            out_best_cell_map <= hypothesis_cell_map;
                            out_best_id <= {
                                score_hypothesis[3:2],
                                hypothesis_symbol_5,
                                score_hypothesis[1:0],
                                hypothesis_symbol_3,
                                hypothesis_symbol_2,
                                hypothesis_symbol_1,
                                hypothesis_symbol_0
                            };
                            out_best_residual_sum <= residual_score_accumulator[37:0];
                            out_best_weight_sum <= cross_product_accumulator[38:0];
                        end
                    end

                    score_point <= 0;
                    hypothesis_occupancy <= 0;
                    hypothesis_valid <= 1'b1;
                    hypothesis_symbol_0 <= 0;
                    hypothesis_symbol_1 <= 0;
                    hypothesis_symbol_2 <= 0;
                    hypothesis_symbol_3 <= 0;
                    hypothesis_symbol_5 <= 0;
                    if (score_hypothesis == 4'd15) begin
                        out_cycles <= cycle_counter + 1'b1;
                        state <= S_OUTPUT;
                    end else begin
                        score_hypothesis <= score_hypothesis + 1'b1;
                        state <= S_COEFFICIENT_0;
                    end
                end
                S_OUTPUT: begin
                    if (out_ready)
                        state <= S_IDLE;
                end
                S_INVALID: begin
                    geometry_done_valid <= 1'b1;
                    geometry_done_invalid <= 1'b1;
                    out_input_invalid <= 1'b1;
                    out_best_valid <= 1'b0;
                    out_cycles <= cycle_counter + 1'b1;
                    state <= S_OUTPUT;
                end
                default: state <= S_IDLE;
            endcase

            if (clear) begin
                state <= S_IDLE;
                geometry_done_valid <= 1'b0;
                geometry_done_invalid <= 1'b0;
                out_best_valid <= 1'b0;
                out_input_invalid <= 1'b0;
                map_normalize_active <= 1'b0;
                map_ready <= 1'b0;
            end
        end
    end
endmodule
