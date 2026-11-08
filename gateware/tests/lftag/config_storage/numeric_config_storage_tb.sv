`timescale 1ns/1ps
module numeric_config_storage_tb;
    logic clk=0;
    always #5 clk=~clk;
    logic rst=1, config_response_valid=0, config_wait=0, config_word=0;
    logic [3:0] state_bits=0;
    logic [2:0] config_index=0;
    logic [9:0] cycle_counter=0;
    logic [31:0] config_read_data=0;
    wire [31:0] controller_banked, controller_direct, focal_banked, focal_direct;
    ControllerStorage c1(.*, .value(controller_banked));
    ControllerStorage #(.EXTERNAL_CONFIG(0)) c0(.*, .value(controller_direct));
    FocalStorage f1(.*, .value(focal_banked));
    FocalStorage #(.EXTERNAL_CONFIG(0)) f0(.*, .value(focal_direct));

    // Static pre-change register contract. Keep the original nested priority:
    // reset, watchdog, state dispatch, response, complete 3-bit index case.
    // These state numbers are checked against the actual enums by test setup.
    localparam logic [3:0] IDLE=0, PACKET=11, WAIT_PACKET=12,
        CENTROIDS=13, CONFIG_RESPONSE=15, CONFIG_FETCH=15;
    logic [31:0] expected_controller=0, expected_focal=0;
    logic focal_known=0;
    integer cycles=0, controller_writes=0, focal_writes=0;
    integer watchdog_suppressed=0, reset_edges=0, hold_edges=0;
    logic [31:0] rng=32'h971ac35e;

    always @(posedge clk) begin
        if (rst) begin
            expected_controller <= 0;
            reset_edges <= reset_edges+1;
        end else if (state_bits != IDLE && state_bits != PACKET
                && state_bits != WAIT_PACKET && state_bits != CENTROIDS
                && cycle_counter >= 999) begin
            if (state_bits == CONFIG_RESPONSE && config_response_valid
                    && (config_index == 0 || config_index == 7))
                watchdog_suppressed <= watchdog_suppressed+1;
        end else begin
            case (state_bits)
                CONFIG_RESPONSE: if (config_response_valid) begin
                    case (config_index)
                        7: begin expected_controller <= config_read_data;
                            controller_writes <= controller_writes+1; end
                        1,2,3,4,5,6: ;
                        default: begin expected_controller <= config_read_data;
                            controller_writes <= controller_writes+1; end
                    endcase
                end
                default: ;
            endcase
        end
        // Baseline focal payload has NO reset assignment. A reset suppresses
        // a new capture but preserves a previously captured arbitrary value.
        if (!rst) begin
            case (state_bits)
                CONFIG_FETCH: begin
                    if (!config_wait) begin end
                    else if (config_response_valid) begin
                        if (!config_word) begin
                            expected_focal <= config_read_data;
                            focal_known <= 1;
                            focal_writes <= focal_writes+1;
                        end
                    end
                end
                default: ;
            endcase
        end
    end

    task automatic tick;
        @(posedge clk); #1;
        cycles=cycles+1;
        assert(controller_banked === expected_controller && controller_direct === expected_controller)
            else $fatal(1,"controller cycle=%0d rst=%0d state=%0d idx=%0d count=%0d rsp=%0d got=%h/%h expected=%h",
                cycles,rst,state_bits,config_index,cycle_counter,config_response_valid,
                controller_banked,controller_direct,expected_controller);
        if (focal_known)
            assert(focal_banked === expected_focal && focal_direct === expected_focal)
                else $fatal(1,"focal cycle=%0d rst=%0d state=%0d wait=%0d word=%0d rsp=%0d got=%h/%h expected=%h",
                    cycles,rst,state_bits,config_wait,config_word,config_response_valid,
                    focal_banked,focal_direct,expected_focal);
    endtask

    task automatic next_input;
        @(negedge clk);
        rng=rng^(rng<<13); rng=rng^(rng>>17); rng=rng^(rng<<5);
        config_read_data=rng;
    endtask

    task automatic capture(input logic [31:0] value);
        next_input(); rst=0; state_bits=15; cycle_counter=0; config_index=0;
        config_wait=1; config_word=0; config_response_valid=1; config_read_data=value;
        tick();
    endtask

    initial begin
        tick();
        capture(32'hdeadbeef);
        // All 32 payload bits are exercised both alone and complemented,
        // including the high sign bit and every capture/hold boundary.
        for (integer bit_index=0; bit_index<32; bit_index++) begin
            capture(32'b1<<bit_index);
            capture(~(32'b1<<bit_index));
        end
        capture(32'hffffffff); capture(0); capture(32'h0007ffff);
        capture(32'h00080000); capture(32'h80000000); capture(32'hfff80000);
        // Reset must clear controller even when its CE is false, and must
        // retain focal even when every non-reset capture condition is true.
        for(integer reset_index=0;reset_index<16;reset_index++) begin
            capture(32'h87654321 ^ (32'b1<<reset_index));
            next_input(); rst=1; state_bits=4'(reset_index);
            config_wait=1; config_word=0; config_response_valid=1; cycle_counter=0;
            tick(); tick();
        end
        // Complete controller guard truth table, including every ten-bit
        // watchdog value. Focal phase values are swept separately below.
        for(integer reset_value=0; reset_value<2; reset_value++)
        for(integer state_value=0; state_value<16; state_value++)
        for(integer index_value=0; index_value<8; index_value++)
        for(integer count_value=0; count_value<1024; count_value++)
        for(integer response_value=0; response_value<2; response_value++) begin
            next_input(); rst=1'(reset_value); state_bits=4'(state_value);
            config_index=3'(index_value); cycle_counter=10'(count_value);
            config_response_valid=1'(response_value); config_wait=rng[4]; config_word=rng[5];
            tick();
        end
        // Complete focal handshake truth table repeated after arbitrary32-bit
        // captures. Spurious responses before request acceptance must hold.
        for(integer reset_value=0;reset_value<2;reset_value++)
        for(integer state_value=0;state_value<16;state_value++)
        for(integer wait_value=0;wait_value<2;wait_value++)
        for(integer word_value=0;word_value<2;word_value++)
        for(integer response_value=0;response_value<2;response_value++) begin
            capture(rng);
            next_input(); rst=1'(reset_value); state_bits=4'(state_value);
            config_wait=1'(wait_value); config_word=1'(word_value);
            config_response_valid=1'(response_value);
            tick();
        end
        // Long request/response gaps, global reset pulse, and watchdog entry
        // after an accepted response. Data must hold while the bus changes.
        capture(32'ha5ff0f91);
        for(integer stall=0;stall<1100;stall++) begin
            next_input(); rst=(stall==550); state_bits=15;
            config_wait=1; config_word=1; config_response_valid=0;
            cycle_counter=10'(stall);
            tick(); hold_edges=hold_edges+1;
        end
        capture(32'h8007ffff);
        next_input(); rst=0; state_bits=15; config_index=7;
        cycle_counter=998; config_response_valid=1; tick();
        next_input(); cycle_counter=999; tick();
        next_input(); cycle_counter=1000; tick();
        assert(controller_writes>2000 && focal_writes>1000 && watchdog_suppressed==52)
            else $fatal(1,"coverage writes=%0d/%0d watchdog=%0d",controller_writes,focal_writes,watchdog_suppressed);
        $display("PASS numeric config storage: %0d checked cycles, controller writes=%0d focal writes=%0d watchdog suppression=%0d resets=%0d holds=%0d",
            cycles,controller_writes,focal_writes,watchdog_suppressed,reset_edges,hold_edges);
        $finish;
    end
endmodule
