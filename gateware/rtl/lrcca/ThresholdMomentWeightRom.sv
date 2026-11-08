`default_nettype none

// Two synchronous reads of one 512x8 moment-weight ROM. Address bit 8
// selects the coarse CDF plane; the lower byte is the unweighted pixel.
// Entries store 255-weight, matching the formatter/CCA interface.
// Quantize the following curves to five bits: min(31, (weight+4)>>3).
// RAW weight = round(0.85 * round(255 * (min(255,279-pixel)/255)^5)).
// Coarse weight = round(255 * ((255-pixel)/255)^2.5).
// Packing the complete table here makes the RTL source hash identify it.
module ThresholdMomentWeightRom (
  input wire clk,
  input wire ce,
  input wire [8:0] address0,
  input wire [8:0] address1,
  output logic [7:0] q0,
  output logic [7:0] q1
);
  localparam logic [4095:0] CONTENTS = 4096'hfffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffefefefefefefefefefefefefefefefefefefefefefefefefefefefefdfdfdfdfdfdfdfdfdfdfdfdfdfdfdfdfdfdfcfcfcfcfcfcfcfcfcfcfcfcfcfbfbfbfbfbfbfbfbfbfbfbfafafafafafafafafafaf9f9f9f9f9f9f9f9f9f8f8f8f8f8f8f8f8f7f7f7f7f7f7f7f6f6f6f6f6f6f6f5f5f5f5f5f5f5f4f4f4f4f4f4f3f3f3f3f3f3f2f2f2f2f2f1f1f1f1f1f0f0f0f0f0efefefefefeeeeeeeeeeededededecececececebebebebeaeaeaeae9e9e9e9e8e8e8e8e7e7e7e7e6e6e6e5e5e5e5e4e4e4e4e3e3e3e2e2e2e2e1e1e1e0e0e0e0e0fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffefefefefefefefefefefefefefefefefefefefefefefefefefefefefdfdfdfdfdfdfdfdfdfdfdfdfdfdfdfcfcfcfcfcfcfcfcfcfcfcfcfbfbfbfbfbfbfbfbfbfafafafafafafafaf9f9f9f9f9f9f8f8f8f8f8f7f7f7f7f7f6f6f6f6f6f5f5f5f5f4f4f4f4f3f3f3f3f2f2f2f1f1f1f0f0f0efefefeeeeeeedededececebebebeaeae9e9e8e8e7e7e6e6e5e5e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4e4;
  (* syn_romstyle = "block_rom", syn_ramstyle = "block_ram" *)
  logic [7:0] memory [0:511];
  initial begin
    for (int index = 0; index < 512; index++)
      memory[index] = CONTENTS[8*index +: 8];
  end
  always_ff @(posedge clk) begin
    if (ce) begin
      q0 <= memory[address0];
      q1 <= memory[address1];
    end
  end
endmodule

`default_nettype wire
