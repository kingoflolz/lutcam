"""Real byte-stream protocol and out-of-band abort checks for record UARTBone."""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fpga" / "litex"))

from migen import Module, Signal
from migen.fhdl import verilog
from migen.sim import passive, run_simulation
from litex.soc.interconnect import stream

from lutcam.control import WishboneGuard
from lutcam.record_bus import RecordUARTBone


def send_bytes(sink, values):
    for value in values:
        yield sink.data.eq(value)
        yield sink.valid.eq(1)
        yield
        for _ in range(20000):
            if (yield sink.ready):
                break
            yield
        else:
            raise AssertionError("byte input did not become ready")
        yield sink.valid.eq(0)
        yield


def command(opcode, address, words=(), *, count=None):
    values = [opcode, len(words) if count is None else count]
    values += list(address.to_bytes(4, "big"))
    for word in words:
        values += list(word.to_bytes(4, "big"))
    return values


class UARTFixture(Module):
    def __init__(self, *, timeout_cycles=20000, guarded=False):
        self.submodules.host = self.host = RecordUARTBone(
            clk_freq=120e6, timeout_cycles=timeout_cycles)
        self.ready = Signal(reset=1)
        self.error = Signal()
        self.inject_fault = Signal()
        if guarded:
            self.submodules.guard = self.guard = WishboneGuard(timeout_cycles=4)
            self.comb += self.host.wishbone.connect(self.guard.bus)
            self.comb += self.host.guard_fault.eq(self.guard.fault_event)
            bus = self.guard.master
        else:
            bus = self.host.wishbone
            self.comb += self.host.guard_fault.eq(self.inject_fault)
        self.comb += [bus.ack.eq(bus.cyc & bus.stb & self.ready & ~self.error),
                      bus.err.eq(bus.cyc & bus.stb & self.error),
                      bus.dat_r.eq(bus.adr ^ 0xabcddcba)]


class RecordUARTBoneTest(unittest.TestCase):
    def capture(self, dut, bench, *, read_ready=True):
        words, responses, aborts = [], [], []
        host, bus = dut.host, dut.host.wishbone

        @passive
        def monitor():
            held = None
            while True:
                active = (yield bus.cyc) and (yield bus.stb)
                if active:
                    fields = ((yield bus.adr), (yield bus.dat_w), (yield bus.we),
                              (yield host.record_first), (yield host.record_last),
                              (yield host.record_count), (yield host.record_source))
                    if held is not None:
                        self.assertEqual(fields, held, "held word metadata changed")
                    held = fields
                    if (yield bus.ack):
                        words.append(fields + ((yield host.record_abort),))
                        held = None
                else:
                    held = None
                if (yield host.source.valid) and (yield host.source.ready):
                    responses.append(((yield host.source.data), (yield host.source.last)))
                if (yield host.record_abort):
                    aborts.append((active, (yield host.record_source)))
                yield

        def wrapped():
            yield host.source.ready.eq(read_ready)
            yield from bench(host, words, responses, aborts)

        run_simulation(dut, [wrapped(), monitor()])
        return words, responses, aborts

    def test_incrementing_and_fixed_writes_keep_command_boundaries(self):
        for opcode in (1, 3):
            with self.subTest(opcode=opcode):
                dut = UARTFixture()
                data = [0x10203040, 0x55667788, 0xdeadbeef, 7]

                def bench(host, words, responses, aborts):
                    yield from send_bytes(host.sink, command(opcode, 0x1234, data))
                    for _ in range(10):
                        yield

                words, responses, aborts = self.capture(dut, bench)
                self.assertEqual(len(words), 4)
                for i, word in enumerate(words):
                    self.assertEqual(word, (0x1234 + (i if opcode == 1 else 0),
                                           data[i], 1, i == 0, i == 3, 4, 1, 0))
                self.assertEqual(responses, [])
                self.assertEqual(aborts, [])

    def test_incrementing_and_fixed_reads_preserve_byte_order_and_last(self):
        for opcode in (2, 4):
            with self.subTest(opcode=opcode):
                dut = UARTFixture()

                def bench(host, words, responses, aborts):
                    yield from send_bytes(host.sink, command(opcode, 0x4567, count=3))
                    for _ in range(90):
                        yield

                words, responses, aborts = self.capture(dut, bench)
                self.assertEqual(len(words), 3)
                addresses = [0x4567 + (i if opcode == 2 else 0) for i in range(3)]
                self.assertEqual([word[0] for word in words], addresses)
                self.assertEqual([word[2:] for word in words],
                                 [(0, 1, 0, 3, 1, 0), (0, 0, 0, 3, 1, 0),
                                  (0, 0, 1, 3, 1, 0)])
                expected = b"".join((address ^ 0xabcddcba).to_bytes(4, "big")
                                    for address in addresses)
                self.assertEqual(bytes(value for value, _ in responses), expected)
                self.assertEqual([last for _, last in responses], [0] * 11 + [1])
                self.assertEqual(aborts, [])

    def test_mailbox_maximum_and_full_uart_counts(self):
        for count in (1, 4, 68, 255):
            with self.subTest(count=count):
                dut = UARTFixture()

                def bench(host, words, responses, aborts):
                    yield from send_bytes(host.sink, command(1, 0x30025400 // 4,
                                                            list(range(count))))
                    for _ in range(5):
                        yield

                words, _, aborts = self.capture(dut, bench)
                self.assertEqual(len(words), count)
                self.assertEqual([w[3] for w in words], [1] + [0] * (count - 1))
                self.assertEqual([w[4] for w in words], [0] * (count - 1) + [1])
                self.assertTrue(all(w[5] == count and w[7] == 0 for w in words))
                self.assertEqual(aborts, [])

    def test_guard_failure_poison_persists_across_cyc_gaps_and_clears_next_command(self):
        dut = UARTFixture(guarded=True)

        def bench(host, words, responses, aborts):
            yield dut.error.eq(1)
            yield from send_bytes(host.sink, command(1, 0x100, [10, 11, 12]))
            yield dut.error.eq(0)
            for _ in range(10):
                yield
            yield from send_bytes(host.sink, command(1, 0x200, [99]))
            for _ in range(10):
                yield

        words, _, aborts = self.capture(dut, bench)
        self.assertEqual([w[7] for w in words], [1, 1, 1, 0])
        self.assertTrue(any(not active and source == 1 for active, source in aborts))

    def test_guard_timeout_aborts_on_ack_and_final_word_fault_wins(self):
        dut = UARTFixture(guarded=True)

        def bench(host, words, responses, aborts):
            yield dut.ready.eq(0)
            yield from send_bytes(host.sink, command(1, 0x100, [10]))
            for _ in range(12):
                yield

        words, _, _ = self.capture(dut, bench)
        self.assertEqual(len(words), 1)
        self.assertEqual(words[0][3:], (1, 1, 1, 1, 1))

    def test_incomplete_command_times_out_between_words_and_next_command_recovers(self):
        dut = UARTFixture(timeout_cycles=100)

        def bench(host, words, responses, aborts):
            yield from send_bytes(host.sink, command(1, 0x100, [10], count=4))
            for _ in range(120):
                yield
            self.assertEqual((yield host.wishbone.cyc), 0)
            self.assertTrue(any(not active for active, _ in aborts))
            yield from send_bytes(host.sink, command(1, 0x200, [20]))
            for _ in range(5):
                yield

        words, _, _ = self.capture(dut, bench)
        self.assertEqual([w[0] for w in words], [0x100, 0x200])
        self.assertEqual([w[3:6] for w in words], [(1, 0, 4), (1, 1, 1)])

    def test_command_reset_aborts_with_no_wishbone_word(self):
        dut = UARTFixture()

        def bench(host, words, responses, aborts):
            yield from send_bytes(host.sink, command(1, 0x100, [10], count=3))
            yield host.command_reset.eq(1)
            yield
            self.assertEqual((yield host.record_abort), 1)
            self.assertEqual((yield host.record_source), 1)
            self.assertEqual((yield host.wishbone.cyc), 0)
            self.assertEqual((yield host.sink.ready), 0)
            yield host.command_reset.eq(0)
            yield
            yield from send_bytes(host.sink, command(1, 0x200, [20]))
            for _ in range(5):
                yield

        words, _, aborts = self.capture(dut, bench)
        self.assertEqual([w[0] for w in words], [0x100, 0x200])
        self.assertTrue(aborts)

    def test_zero_length_byte_preserves_256_word_protocol(self):
        dut = UARTFixture()

        def bench(host, words, responses, aborts):
            yield from send_bytes(host.sink, command(2, 0x100, count=0))
            for _ in range(1800):
                yield
            yield from send_bytes(host.sink, command(1, 0x500, [20]))
            for _ in range(5):
                yield

        words, responses, aborts = self.capture(dut, bench)
        self.assertEqual([w[0] for w in words], list(range(0x100, 0x200)) + [0x500])
        self.assertTrue(all(w[5] == 256 for w in words[:256]))
        self.assertEqual([w[3] for w in words[:256]], [1] + [0] * 255)
        self.assertEqual([w[4] for w in words[:256]], [0] * 255 + [1])
        self.assertEqual(len(responses), 1024)
        self.assertEqual([last for _, last in responses], [0] * 1023 + [1])
        self.assertEqual(aborts, [])

    def test_bus_and_response_backpressure_keeps_metadata_and_command_timer(self):
        dut = UARTFixture(timeout_cycles=80)

        def bench(host, words, responses, aborts):
            yield dut.ready.eq(0)
            yield from send_bytes(host.sink, command(2, 0x123, count=2))
            for _ in range(10):
                yield
            yield dut.ready.eq(1)
            # The first word is ACKed but no response byte is accepted. The
            # whole-command timer must expire despite the bus having finished.
            for _ in range(100):
                yield
            self.assertEqual((yield host.source.valid), 0)
            self.assertEqual((yield host.wishbone.cyc), 0)

        words, responses, aborts = self.capture(dut, bench, read_ready=False)
        self.assertEqual(len(words), 1)
        self.assertEqual(responses, [])
        self.assertTrue(any(not active for active, _ in aborts))

    def test_timeout_or_explicit_reset_masks_active_request_and_byte_handshakes(self):
        for explicit in (False, True):
            with self.subTest(explicit=explicit):
                dut = UARTFixture(timeout_cycles=60)

                def bench(host, words, responses, aborts):
                    yield dut.ready.eq(0)
                    yield from send_bytes(host.sink, command(1, 0x100, [10]))
                    self.assertEqual((yield host.wishbone.cyc), 1)
                    if explicit:
                        yield host.command_reset.eq(1)
                        yield
                    else:
                        for _ in range(100):
                            if (yield host.record_abort):
                                break
                            yield
                        else:
                            self.fail("active request did not time out")
                    self.assertEqual((yield host.record_abort), 1)
                    self.assertEqual((yield host.wishbone.cyc), 0)
                    self.assertEqual((yield host.wishbone.stb), 0)
                    self.assertEqual((yield host.sink.ready), 0)
                    self.assertEqual((yield host.source.valid), 0)
                    yield host.command_reset.eq(0)
                    for _ in range(5):
                        yield

                words, _, aborts = self.capture(dut, bench)
                self.assertEqual(words, [])
                self.assertTrue(aborts)

    def test_existing_usb_endpoint_objects_are_reused(self):
        phy = Module()
        phy.sink = stream.Endpoint([("data", 8)])
        phy.source = stream.Endpoint([("data", 8)])
        host = RecordUARTBone(phy=phy, clk_freq=120e6)
        self.assertIs(host.sink, phy.source)
        self.assertIs(host.source, phy.sink)
        self.assertEqual(len(host.record_count), 9)
        self.assertEqual(len(host.record_source), 2)

    @unittest.skipUnless(shutil.which("iverilog") and shutil.which("vvp"),
                         "generated RTL regression requires Icarus Verilog")
    def test_generated_rtl_zero_length_byte_has_exactly_256_words(self):
        host = RecordUARTBone(clk_freq=120e6, timeout_cycles=5000)
        ios = {host.sink.data, host.sink.valid, host.sink.ready,
               host.source.data, host.source.valid, host.source.ready, host.source.last,
               host.wishbone.adr, host.wishbone.ack, host.wishbone.dat_r,
               host.wishbone.cyc, host.wishbone.stb, host.record_first,
               host.record_last, host.record_count, host.record_abort}
        generated = str(verilog.convert(host, ios=ios, name="record_uart"))
        bench = r"""
module tb;
reg clk=0, rst=1, valid=0;
reg [7:0] data=0;
wire ready, cyc, stb, first, last, abort_record, out_valid, out_last;
wire [8:0] count;
wire [29:0] address;
wire [7:0] out_data;
integer words=0, bytes=0, endings=0;
always #5 clk=~clk;
record_uart dut(.sys_clk(clk), .sys_rst(rst),
  .sink_valid(valid), .sink_ready(ready), .sink_payload_data(data),
  .source_ready(1'b1), .source_valid(out_valid), .source_last(out_last),
  .source_payload_data(out_data), .wishbone_adr(address),
  .wishbone_dat_r({2'b00,address}), .wishbone_ack(cyc & stb),
  .wishbone_cyc(cyc), .wishbone_stb(stb), .record_first(first),
  .record_last(last), .record_count(count), .record_abort(abort_record));
task send_byte(input [7:0] value);
  begin
    @(negedge clk); data=value; valid=1;
    @(posedge clk); while(!ready) @(posedge clk);
    @(negedge clk); valid=0;
  end
endtask
always @(posedge clk) if(!rst) begin
  if(abort_record) $fatal(1,"unexpected command abort");
  if(cyc && stb) begin
    if(count != 256 || first != (words==0) || last != (words==255))
      $fatal(1,"bad generated metadata at word %0d", words);
    if(address != 30'h100 + words) $fatal(1,"bad address");
    words=words+1;
  end
  if(out_valid) begin bytes=bytes+1; if(out_last) endings=endings+1; end
end
initial begin
  repeat(3) @(negedge clk); rst=0;
  send_byte(2); send_byte(0);
  send_byte(0); send_byte(0); send_byte(1); send_byte(0);
  repeat(1800) @(negedge clk);
  if(words != 256 || bytes != 1024 || endings != 1 || !ready)
    $fatal(1,"wrong completion words=%0d bytes=%0d endings=%0d",words,bytes,endings);
  $display("PASS generated 256-word record"); $finish;
end
initial #100000 $fatal(1,"timeout");
endmodule
"""
        with tempfile.TemporaryDirectory(prefix="lutcam-record-uart-") as temp:
            path = Path(temp)
            (path / "dut.v").write_text(generated)
            (path / "tb.sv").write_text(bench)
            built = subprocess.run(["iverilog", "-g2012", "-s", "tb", "-o",
                                    str(path / "sim"), str(path / "dut.v"),
                                    str(path / "tb.sv")], capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stderr)
            result = subprocess.run(["vvp", str(path / "sim")],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("PASS generated 256-word record", result.stdout)


if __name__ == "__main__":
    unittest.main()
