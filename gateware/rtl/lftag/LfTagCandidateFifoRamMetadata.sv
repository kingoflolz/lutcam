// Production three-RAM LFTag candidate queue.
//
// The descriptor payload uses two native 20-bit RAM10 memories. Candidate
// metadata uses one synchronous block RAM instead of 76 SRL8 cells. The
// otherwise-unused descriptor bit 37 stores per-centroid validity. This is
// the internal production descriptor format; bits 36:0 contain
// {mass, y, x}, and bits 39:38 remain reserved.
module LfTagCandidateFifoRamMetadata #(
    parameter bit RETAIN_DIAGNOSTICS = 1'b0
) (
    input  logic        clk,
    input  logic        rst,

    input  logic        in_valid,
    input  logic        in_group_end,
    input  logic [14:0] in_x_q4,
    input  logic [13:0] in_y_q4,
    input  logic [7:0]  in_log_mass_q5_3,
    input  logic        in_centroid_valid,
    input  logic [1:0]  in_scale,
    input  logic [7:0]  in_frame_seq,

    // Metadata is read synchronously.  The queue prefetches the current head,
    // so these fields are valid whenever head_valid is asserted, including on
    // the cycle immediately following a commit or pop.
    output logic        head_valid,
    output logic [1:0]  head_scale,
    output logic [7:0]  head_frame_seq,
    output logic        head_protocol,
    output logic [7:0]  head_mass_spread,
    input  logic        head_pop,

    input  logic        desc_read_valid,
    input  logic [3:0]  desc_read_index,
    output logic        desc_read_data_valid,
    output logic [39:0] desc_read_data,
    output logic        desc_read_centroid_valid,

    output logic [5:0]  fifo_level,
    output logic        fifo_full,
    input  logic        clear_diagnostics,
    output logic        fifo_drop_sticky,
    output logic        protocol_error_sticky,

    // Shared-camera processing reset must not erase the first fault evidence.
    // Default standalone use resets these words together with the queue.
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic        diagnostic_reset,
    /* verilator lint_on UNUSEDSIGNAL */
    // {core_idle, output_ready, output_valid, owner, owner_active, peer_level}
    input  logic [10:0] diagnostic_context,
    output logic [31:0] debug_status,
    output logic [31:0] debug_protocol,
    output logic [31:0] debug_drop
);
    localparam int GROUP_COUNT = 32;
    localparam int METADATA_BITS = 19;
    // Each RAM10 has room for a power-of-two physical stride.  Leaving seven
    // addresses unused per group removes both group-pointer multiplies from
    // the descriptor ports without consuming another memory block.
    localparam int DESCRIPTOR_DEPTH = 16 * GROUP_COUNT;

    (* ram_style = "block", syn_ramstyle = "block_ram" *)
    logic [19:0] descriptor_lo [0:DESCRIPTOR_DEPTH-1];
    (* ram_style = "block", syn_ramstyle = "block_ram" *)
    logic [19:0] descriptor_hi [0:DESCRIPTOR_DEPTH-1];
    // Unoccupied metadata entries are never observed, so only the pointers
    // and read bypass require reset; the memory itself does not.
    (* ram_style = "block", syn_ramstyle = "block_ram" *)
    logic [METADATA_BITS-1:0] metadata_memory [0:GROUP_COUNT-1];

    logic [4:0] write_pointer;
    logic [4:0] read_pointer;
    logic [5:0] committed_count;

    logic [3:0] stage_count;
    logic       stage_slot_reserved;
    logic [1:0] stage_scale;
    logic [7:0] stage_frame_seq;
    logic       stage_protocol;
    logic [7:0] stage_min_mass;
    logic [7:0] stage_max_mass;

    logic       pop_fire;
    logic       tuple_slot_reserved;
    logic       commit_fire;
    logic       descriptor_write_enable;
    logic [8:0] descriptor_write_address;
    logic [8:0] descriptor_read_address;
    logic [19:0] descriptor_read_lo;
    logic [19:0] descriptor_read_hi;

    logic [1:0] tuple_scale;
    logic [7:0] tuple_frame_seq;
    logic       tuple_protocol;
    logic [7:0] tuple_min_mass;
    logic [7:0] tuple_max_mass;
    logic [METADATA_BITS-1:0] metadata_write_data;
    logic [4:0] metadata_read_address;
    logic [METADATA_BITS-1:0] metadata_ram_read_data;
    logic [METADATA_BITS-1:0] metadata_bypass_data;
    logic [METADATA_BITS-1:0] metadata_head_data;
    logic        metadata_head_bypass;
    logic        metadata_bypass_valid;

    logic [5:0] diagnostic_high_water;
    logic diagnostic_protocol_valid;
    logic diagnostic_drop_valid;
    logic diagnostic_protocol_seen;
    logic diagnostic_drop_seen;
    logic [4:0] protocol_reason;

    function automatic logic [8:0] descriptor_address(
        input logic [4:0] group_pointer,
        input logic [3:0] tuple_index
    );
        begin
            descriptor_address = {group_pointer, tuple_index};
        end
    endfunction

    assign head_valid = committed_count != 0;
    assign fifo_level = committed_count;
    assign fifo_full = committed_count == 6'd32;
    assign pop_fire = head_pop && head_valid;

    // A pop at a full boundary makes write_pointer available on the same edge.
    assign tuple_slot_reserved = (stage_count == 0)
        ? (!fifo_full || pop_fire) : stage_slot_reserved;
    assign commit_fire = in_valid && in_group_end && tuple_slot_reserved;
    assign descriptor_write_enable = in_valid && tuple_slot_reserved
        && (stage_count < 4'd9);
    assign descriptor_write_address = descriptor_address(
        write_pointer, stage_count);
    assign descriptor_read_address = descriptor_address(
        read_pointer, desc_read_index);

    // Reason bits: count, illegal scale, changing scale/frame, reserved zero.
    // Capture the first offending tuple, rather than waiting for group_end;
    // the group may already have accumulated an error by its last tuple.
    assign protocol_reason[0] = (stage_count >= 4'd9)
        || (in_group_end && (stage_count != 4'd8));
    assign protocol_reason[1] = in_scale == 2'd3;
    assign protocol_reason[2] = (stage_count != 0) && (in_scale != stage_scale);
    assign protocol_reason[3] = (stage_count != 0) && (in_frame_seq != stage_frame_seq);
    assign protocol_reason[4] = 1'b0;

    // Diagnostic fields: [5:0] high-water; [6] protocol context valid; [7]
    // denied admission context valid; [8:9] malformed/dropped group seen. Contexts
    // can precede group_end, hence the separate group-completion flags.
    assign debug_status = {22'b0, diagnostic_drop_seen,
        diagnostic_protocol_seen, diagnostic_drop_valid,
        diagnostic_protocol_valid, diagnostic_high_water};

    always_ff @(posedge clk) begin : retained_diagnostics
        // Clear wins even if a new tuple arrives on the same edge. Clearing
        // diagnostics never changes partial-group staging or queue contents.
        if ((RETAIN_DIAGNOSTICS ? diagnostic_reset : rst) || clear_diagnostics) begin
            diagnostic_high_water <= 6'b0;
            diagnostic_protocol_valid <= 1'b0;
            diagnostic_drop_valid <= 1'b0;
            diagnostic_protocol_seen <= 1'b0;
            diagnostic_drop_seen <= 1'b0;
            debug_protocol <= 32'b0;
            debug_drop <= 32'b0;
        end else if (!rst) begin
            // Include a commit's new occupancy so an immediately following
            // processing reset cannot conceal a one-cycle peak.
            if (commit_fire && !pop_fire) begin
                if (committed_count >= diagnostic_high_water)
                    diagnostic_high_water <= committed_count + 1'b1;
            end else if (committed_count > diagnostic_high_water) begin
                diagnostic_high_water <= committed_count;
            end

            if (in_valid && (|protocol_reason) && !diagnostic_protocol_valid) begin
                diagnostic_protocol_valid <= 1'b1;
                // [4:0] reason, [8:5] tuples already staged, [9] group_end,
                // [11:10]/[13:12] first/current scale, [21:14]/[29:22]
                // first/current frame, [31:30] reserved zero.
                debug_protocol <= {2'b0,
                    in_frame_seq, tuple_frame_seq, in_scale, tuple_scale,
                    in_group_end, stage_count, protocol_reason};
            end
            if (in_valid && (stage_count == 0) && !tuple_slot_reserved
                    && !diagnostic_drop_valid) begin
                diagnostic_drop_valid <= 1'b1;
                // Diagnostic fields: [5:0] own level, [11:6] peer level, [12] pop,
                // [13] owner active, [14] owner, [15]/[16] output valid/ready,
                // [17] core idle, [19:18] scale, [27:20] frame, [28] reserved zero,
                // [29] group_end. This is the denied FIRST tuple's context.
                debug_drop <= {2'b0, in_group_end, 1'b0,
                    in_frame_seq, in_scale, diagnostic_context[10:6],
                    pop_fire, diagnostic_context[5:0], committed_count};
            end
            if (in_valid && in_group_end) begin
                if (tuple_protocol)
                    diagnostic_protocol_seen <= 1'b1;
                if (!tuple_slot_reserved)
                    diagnostic_drop_seen <= 1'b1;
            end
        end
    end

    always_comb begin
        tuple_scale = (stage_count == 0) ? in_scale : stage_scale;
        tuple_frame_seq = (stage_count == 0)
            ? in_frame_seq : stage_frame_seq;
        tuple_protocol = (stage_count == 0)
            ? (in_scale > 2) : stage_protocol;
        tuple_min_mass = (stage_count == 0)
            ? in_log_mass_q5_3 : stage_min_mass;
        tuple_max_mass = (stage_count == 0)
            ? in_log_mass_q5_3 : stage_max_mass;

        if (stage_count != 0) begin
            tuple_protocol = tuple_protocol
                | (in_scale != stage_scale)
                | (in_frame_seq != stage_frame_seq);
            if (in_log_mass_q5_3 < tuple_min_mass)
                tuple_min_mass = in_log_mass_q5_3;
            if (in_log_mass_q5_3 > tuple_max_mass)
                tuple_max_mass = in_log_mass_q5_3;
        end
        if (stage_count >= 4'd9)
            tuple_protocol = 1'b1;
        if (in_group_end)
            tuple_protocol = tuple_protocol | (stage_count != 4'd8);

        metadata_write_data = {
            tuple_max_mass - tuple_min_mass,
            tuple_protocol,
            tuple_frame_seq,
            tuple_scale
        };
    end

    // Prefetch the next circular head on a pop. The one-cycle bypass below
    // covers empty-to-nonempty and simultaneous last-pop/new-commit cases,
    // so behavior never depends on the RAM's read-during-write mode.
    assign metadata_read_address = read_pointer + pop_fire;
    assign metadata_head_bypass = commit_fire
        && ((committed_count == 0)
            || (pop_fire && (committed_count == 1)));

    // On an empty-to-nonempty transition the staging registers are no longer
    // needed by ingress, so reuse them as the one-cycle RAM write-through
    // value instead of allocating a second 19-bit metadata register.
    assign metadata_bypass_data = {
        stage_min_mass,
        stage_protocol,
        stage_frame_seq,
        stage_scale
    };
    assign metadata_head_data = metadata_bypass_valid
        ? metadata_bypass_data : metadata_ram_read_data;
    assign head_scale = head_valid ? metadata_head_data[1:0] : 2'b0;
    assign head_frame_seq = head_valid ? metadata_head_data[9:2] : 8'b0;
    assign head_protocol = head_valid ? metadata_head_data[10] : 1'b0;
    assign head_mass_spread = head_valid
        ? metadata_head_data[18:11] : 8'b0;

    // The compact consumer may use either the named response or descriptor
    // bit 37.  The remaining descriptor fields are byte-for-byte unchanged.
    assign desc_read_data = {descriptor_read_hi, descriptor_read_lo};
    assign desc_read_centroid_valid = desc_read_data_valid
        && descriptor_read_hi[17];

    always_ff @(posedge clk) begin
        if (descriptor_write_enable) begin
            descriptor_lo[descriptor_write_address] <= {
                in_y_q4[4:0], in_x_q4
            };
            descriptor_hi[descriptor_write_address] <= {
                2'b0, in_centroid_valid,
                in_log_mass_q5_3, in_y_q4[13:5]
            };
        end

        if (desc_read_valid && head_valid && (desc_read_index < 4'd9)) begin
            descriptor_read_lo <= descriptor_lo[descriptor_read_address];
            descriptor_read_hi <= descriptor_hi[descriptor_read_address];
        end

        if (rst)
            desc_read_data_valid <= 1'b0;
        else
            desc_read_data_valid <= desc_read_valid && head_valid
                && (desc_read_index < 4'd9);
    end

    always_ff @(posedge clk) begin
        if (commit_fire)
            metadata_memory[write_pointer] <= metadata_write_data;
        metadata_ram_read_data <= metadata_memory[metadata_read_address];
    end

    always_ff @(posedge clk) begin
        if (rst)
            metadata_bypass_valid <= 1'b0;
        else
            metadata_bypass_valid <= metadata_head_bypass;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            write_pointer <= 0;
            read_pointer <= 0;
            committed_count <= 0;
            stage_count <= 0;
            stage_slot_reserved <= 0;
            stage_scale <= 0;
            stage_frame_seq <= 0;
            stage_protocol <= 0;
            stage_min_mass <= 0;
            stage_max_mass <= 0;
            fifo_drop_sticky <= 0;
            protocol_error_sticky <= 0;
        end else begin
            if (clear_diagnostics) begin
                fifo_drop_sticky <= 1'b0;
                protocol_error_sticky <= 1'b0;
            end

            case ({commit_fire, pop_fire})
                2'b10: committed_count <= committed_count + 1'b1;
                2'b01: committed_count <= committed_count - 1'b1;
                default: committed_count <= committed_count;
            endcase
            if (pop_fire)
                read_pointer <= read_pointer + 1'b1;

            if (in_valid) begin
                if (stage_count == 0) begin
                    stage_slot_reserved <= tuple_slot_reserved;
                    stage_scale <= in_scale;
                    stage_frame_seq <= in_frame_seq;
                end

                if (in_group_end) begin
                    if (tuple_protocol)
                        protocol_error_sticky <= 1'b1;

                    if (tuple_slot_reserved) begin
                        write_pointer <= write_pointer + 1'b1;
                    end else begin
                        fifo_drop_sticky <= 1'b1;
                    end

                    stage_count <= 0;
                    stage_slot_reserved <= 0;
                    // Retain the completed tuple metadata for one-cycle
                    // write-through when this commit becomes the FIFO head.
                    stage_protocol <= tuple_protocol;
                    stage_min_mass <= tuple_max_mass - tuple_min_mass;
                end else begin
                    if (stage_count < 4'd15)
                        stage_count <= stage_count + 1'b1;
                    stage_protocol <= tuple_protocol;
                    stage_min_mass <= tuple_min_mass;
                    stage_max_mass <= tuple_max_mass;
                end
            end
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (!rst) begin
            assert (committed_count <= 6'd32);
            assert (!(head_pop && !head_valid));
            assert (!(desc_read_valid && (!head_valid
                || (desc_read_index >= 4'd9))));
        end
    end
`endif
endmodule
