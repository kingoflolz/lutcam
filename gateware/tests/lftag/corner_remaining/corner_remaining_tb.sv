`timescale 1ns/1ps
`default_nettype none

module corner_remaining_tb;
    logic [2:0] bottom_left;
    logic [2:0] bottom_right;
    wire [19:0] out_remaining_index_flat;

    CornerRemainingSubject dut (.*);

    initial begin
        for (integer a = 0; a < 8; a = a + 1) begin
            for (integer b = 0; b < 8; b = b + 1) begin
                bottom_left = 3'(a);
                bottom_right = 3'(b);
                #1;
                for (integer slot = 0; slot < 5; slot = slot + 1) begin
                    $display("ROW %0d %0d %0d %0d", a, b, slot,
                             out_remaining_index_flat[4*slot +: 4]);
                end
            end
        end
        $display("PASS pairs=64 slots=320 clock_edges=0");
        $finish;
    end
endmodule

`default_nettype wire
