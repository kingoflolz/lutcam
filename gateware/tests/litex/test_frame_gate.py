"""Whole-frame validation and replay through the Ethernet RX frame gate."""

import unittest

from migen.sim import passive, run_simulation

from liteeth.mac.frame_gate import ValidatedEthernetFrameGate


def idle(cycles):
    for _ in range(cycles):
        yield


def send(dut, words, *, last=True, last_be=2, error_at=None,
         first=True, nonfinal_be=0, gaps=0):
    """Drive unbackpressured 16-bit PHY beats, optionally leaving a frame open."""
    for index, word in enumerate(words):
        ending = last and index == len(words) - 1
        yield dut.sink.valid.eq(1)
        yield dut.sink.first.eq(first and index == 0)
        yield dut.sink.data.eq(word)
        yield dut.sink.last.eq(ending)
        yield dut.sink.last_be.eq(last_be if ending else nonfinal_be)
        yield dut.sink.error.eq((error_at or {}).get(index, 0))
        yield
        if gaps:
            yield dut.sink.valid.eq(0)
            yield from idle(gaps)
    yield dut.sink.valid.eq(0)
    yield dut.sink.first.eq(0)
    yield dut.sink.last.eq(0)
    yield dut.sink.last_be.eq(0)
    yield dut.sink.error.eq(0)


def payload(words, last_be=2):
    result = b"".join(word.to_bytes(2, "little") for word in words)
    return result[:-1] if last_be == 1 else result


def ethernet_ipv4_udp(data=b"frame gate payload", *, ip_length=None,
                      udp_length=None, padding=b"", protocol=17):
    """Build bytes at the gate's post-FCS Ethernet interface."""
    udp = bytearray.fromhex("1234567800000000")
    udp[4:6] = (8 + len(data) if udp_length is None else udp_length).to_bytes(2, "big")
    ip = bytearray.fromhex("450000000000400040110000c0000201c0000202")
    ip[2:4] = (20 + len(udp) + len(data) if ip_length is None else ip_length).to_bytes(2, "big")
    ip[9] = protocol
    checksum = sum(int.from_bytes(ip[index:index + 2], "big") for index in range(0, 20, 2))
    while checksum >> 16:
        checksum = (checksum & 0xFFFF) + (checksum >> 16)
    ip[10:12] = (checksum ^ 0xFFFF).to_bytes(2, "big")
    ethernet = bytes.fromhex("102030405060a0b0c0d0e0f00800")
    return ethernet + ip + udp + data + padding


def send_bytes(dut, data, **kwargs):
    words = [int.from_bytes(data[index:index + 2], "little") for index in range(0, len(data), 2)]
    yield from send(dut, words, last_be=1 if len(data) & 1 else 2, **kwargs)


class Observe:
    def __init__(self, dut, ready=lambda cycle: True):
        self.dut = dut
        self.ready = ready
        self.frames = []
        self.beats = []
        self.pending = bytearray()
        self.events = {name: 0 for name in ("frame_dropped", "frame_error", "frame_overflow")}
        self.stalled = None

    @passive
    def run(self):
        cycle = 0
        while True:
            dut = self.dut
            assert (yield dut.sink.ready), "PHY input must never be backpressured"
            for name in self.events:
                self.events[name] += (yield getattr(dut, name))
            if (yield dut.source.valid):
                beat = ((yield dut.source.data), (yield dut.source.last),
                        (yield dut.source.last_be), (yield dut.source.error),
                        (yield dut.source.first))
                if self.stalled is not None:
                    assert beat == self.stalled, "replay beat changed while stalled"
                assert beat[3] == 0, "accepted frame replayed an error flag"
                if (yield dut.source.ready):
                    self.beats.append(beat)
                    data = beat[0].to_bytes(2, "little")
                    if beat[1]:
                        assert beat[2] in (1, 2), "illegal final-byte mask reached output"
                        data = data[:beat[2].bit_length()]
                    else:
                        assert beat[2] == 0, "non-final replay word carried final-byte metadata"
                    self.pending.extend(data)
                    if beat[1]:
                        self.frames.append(bytes(self.pending))
                        self.pending.clear()
                    self.stalled = None
                else:
                    self.stalled = beat
            else:
                assert self.stalled is None, "replay valid withdrawn while stalled"
            yield dut.source.ready.eq(bool(self.ready(cycle)))
            cycle += 1
            yield


class FrameGateTests(unittest.TestCase):
    def test_only_complete_frame_is_visible_and_odd_final_byte_is_preserved(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=8)
        observed = Observe(dut)

        def bench():
            yield from send(dut, [0x2211, 0x4433], last=False, gaps=2)
            yield from idle(10)
            self.assertEqual(observed.frames, [])
            self.assertEqual(observed.beats, [])
            self.assertEqual((yield dut.source.valid), 0)
            yield from send(dut, [0xAA55], last_be=1, first=False)
            yield from idle(20)
            self.assertEqual(observed.frames, [b"\x11\x22\x33\x44\x55"])
            self.assertEqual([beat[2] for beat in observed.beats], [0, 0, 1])
            self.assertEqual(observed.events, dict.fromkeys(observed.events, 0))

        run_simulation(dut, [bench(), observed.run()])

    def test_replay_remains_stable_under_backpressure(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=16)
        draining = {"enabled": False}
        observed = Observe(dut, ready=lambda cycle: draining["enabled"] and cycle % 7 in (0, 3, 4))
        words = [0x1000 + index * 0x102 for index in range(12)]

        def bench():
            yield from send(dut, words)
            yield from idle(10)
            self.assertEqual((yield dut.busy), 1)
            self.assertEqual((yield dut.source.valid), 1)
            self.assertEqual(observed.frames, [])
            draining["enabled"] = True
            yield from idle(100)
            self.assertEqual(observed.frames, [payload(words)])
            self.assertEqual(len(observed.beats), len(words))
            self.assertEqual((yield dut.busy), 0)
            self.assertEqual(observed.events, dict.fromkeys(observed.events, 0))

        run_simulation(dut, [bench(), observed.run()])

    def test_frame_starting_while_busy_is_dropped_even_after_output_frees(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=8)
        draining = {"enabled": False}
        observed = Observe(dut, ready=lambda cycle: draining["enabled"])
        original, recovery = [0x3210, 0x7654], [0xBA98, 0xFEDC]

        def bench():
            yield from send(dut, original)
            yield from idle(6)
            self.assertEqual((yield dut.busy), 1)
            yield from send(dut, [0xDEAD], last=False)
            yield from idle(4)
            self.assertEqual(observed.events["frame_dropped"], 0)
            draining["enabled"] = True
            yield from idle(12)
            self.assertEqual(observed.frames, [payload(original)])
            self.assertEqual((yield dut.busy), 0)
            yield from send(dut, [0xBEEF, 0xCAFE], first=False)
            yield from idle(12)
            self.assertEqual(observed.frames, [payload(original)])
            self.assertEqual(observed.events["frame_dropped"], 1)
            self.assertEqual(observed.events["frame_error"], 0)
            self.assertEqual(observed.events["frame_overflow"], 0)
            yield from send(dut, recovery)
            yield from idle(20)
            self.assertEqual(observed.frames, [payload(original), payload(recovery)])
            self.assertEqual(observed.events["frame_dropped"], 1)

        run_simulation(dut, [bench(), observed.run()])

    def test_late_and_earlier_error_bits_drop_every_word_and_recover(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=8)
        observed = Observe(dut)

        def bench():
            yield from send(dut, [0x1234, 0x5678], last=False)
            yield from idle(4)
            self.assertEqual(observed.beats, [])
            yield from send(dut, [0xABCD], error_at={0: 2}, first=False)
            yield from idle(12)
            self.assertEqual(observed.beats, [])
            self.assertEqual(observed.events["frame_dropped"], 1)
            self.assertEqual(observed.events["frame_error"], 1)
            yield from send(dut, [0xA123], last=False, error_at={0: 1})
            yield from idle(5)
            self.assertEqual(observed.events["frame_error"], 1)
            yield from send(dut, [0xB456, 0xC789], first=False)
            yield from idle(12)
            self.assertEqual(observed.beats, [])
            self.assertEqual(observed.events["frame_dropped"], 2)
            self.assertEqual(observed.events["frame_error"], 2)
            yield from send(dut, [0x5A25])
            yield from idle(12)
            self.assertEqual(observed.frames, [b"\x25\x5a"])
            self.assertEqual(observed.events["frame_overflow"], 0)

        run_simulation(dut, [bench(), observed.run()])

    def test_final_byte_mask_validation_and_nonfinal_metadata_is_ignored(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=8)
        observed = Observe(dut)

        def bench():
            for ending in (0, 3):
                yield from send(dut, [0x1234, 0xABCD], last_be=ending)
                yield from idle(10)
            self.assertEqual(observed.frames, [])
            self.assertEqual(observed.events["frame_dropped"], 2)
            self.assertEqual(observed.events["frame_error"], 2)
            yield from send(dut, [0x3210, 0x7654, 0xBA98], nonfinal_be=3)
            yield from idle(20)
            self.assertEqual(observed.frames, [b"\x10\x32\x54\x76\x98\xba"])
            self.assertEqual(observed.events["frame_dropped"], 2)

        run_simulation(dut, [bench(), observed.run()])

    def test_exact_depth_is_accepted_but_one_more_beat_drops_the_whole_frame(self):
        for depth in (2, 8, 1024):
            with self.subTest(depth=depth):
                dut = ValidatedEthernetFrameGate(dw=16, depth=depth)
                observed = Observe(dut)
                exact = [(index * 0x113 + 7) & 0xFFFF for index in range(depth)]
                oversize = [(index * 0x207 + 3) & 0xFFFF for index in range(depth + 1)]

                def bench():
                    yield from send(dut, exact, last_be=1)
                    yield from idle(4 * depth + 16)
                    self.assertEqual(observed.frames, [payload(exact, 1)])
                    yield from send(dut, oversize)
                    yield from idle(4 * depth + 16)
                    self.assertEqual(observed.frames, [payload(exact, 1)])
                    self.assertEqual(observed.events["frame_dropped"], 1)
                    self.assertEqual(observed.events["frame_overflow"], 1)
                    yield from send(dut, [0x6745])
                    yield from idle(12)
                    self.assertEqual(observed.frames, [payload(exact, 1), b"\x45\x67"])

                run_simulation(dut, [bench(), observed.run()])

    def test_missing_last_never_publishes_and_overflow_drains_until_real_last(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=4)
        observed = Observe(dut)

        def bench():
            yield from send(dut, [1, 2, 3, 4], last=False)
            yield from idle(20)
            self.assertEqual(observed.beats, [])
            self.assertEqual(observed.events, dict.fromkeys(observed.events, 0))
            yield from send(dut, list(range(5, 21)), last=False, first=False)
            yield from idle(20)
            self.assertEqual(observed.beats, [])
            self.assertEqual(observed.events, dict.fromkeys(observed.events, 0))
            yield from send(dut, [21], first=False)
            yield from idle(12)
            self.assertEqual(observed.frames, [])
            self.assertEqual(observed.events["frame_dropped"], 1)
            self.assertEqual(observed.events["frame_overflow"], 1)
            yield from send(dut, [0xF013, 0xCAFE])
            yield from idle(16)
            self.assertEqual(observed.frames, [b"\x13\xf0\xfe\xca"])
            self.assertEqual(observed.events["frame_dropped"], 1)

        run_simulation(dut, [bench(), observed.run()])

    def test_ipv4_declared_length_cannot_exceed_complete_physical_frame(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=64)
        observed = Observe(dut)
        frame = ethernet_ipv4_udp(bytes(range(20)))
        truncated = [frame[:length] for length in (14, 15, 16, 17, 18, 24, 34, 38, 40, 41, 42, len(frame) - 1)]
        truncated.append(ethernet_ipv4_udp(b"short", ip_length=65535, udp_length=65515))

        def bench():
            for count, bad_frame in enumerate(truncated, 1):
                yield from send_bytes(dut, bad_frame)
                yield from idle(100)
                self.assertEqual(observed.beats, [], f"published short frame {len(bad_frame)} bytes")
                self.assertEqual(observed.events["frame_dropped"], count)
                self.assertEqual(observed.events["frame_error"], count)
                self.assertEqual(observed.events["frame_overflow"], 0)
            yield from send_bytes(dut, frame)
            yield from idle(100)
            self.assertEqual(observed.frames, [frame])
            self.assertEqual((yield dut.frame_length), len(frame))

        run_simulation(dut, [bench(), observed.run()])

    def test_udp_length_must_exactly_consume_ipv4_payload(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=64)
        observed = Observe(dut)
        body = bytes(range(20))
        malformed = [ethernet_ipv4_udp(body, udp_length=length) for length in (0, 7, 8, 27, 29, 65535)]
        malformed.extend(ethernet_ipv4_udp(body, ip_length=length) for length in (0, 19, 20, 27))
        good = ethernet_ipv4_udp(body)

        def bench():
            for count, frame in enumerate(malformed, 1):
                yield from send_bytes(dut, frame)
                yield from idle(100)
                self.assertEqual(observed.beats, [])
                self.assertEqual(observed.events["frame_dropped"], count)
                self.assertEqual(observed.events["frame_error"], count)
            yield from send_bytes(dut, good)
            yield from idle(100)
            self.assertEqual(observed.frames, [good])
            self.assertEqual(observed.events["frame_overflow"], 0)

        run_simulation(dut, [bench(), observed.run()])

    def test_length_high_bits_cannot_alias_an_exactly_full_buffer(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=32)
        observed = Observe(dut)
        body = bytes(range(22))
        good = ethernet_ipv4_udp(body)
        self.assertEqual(len(good), 64)
        malformed = [ethernet_ipv4_udp(body, ip_length=ip, udp_length=udp)
                     for ip, udp in ((114, 94), (114, 30), (50, 94),
                                     (65535, 65515), (50, 65535))]

        def bench():
            for count, frame in enumerate(malformed, 1):
                yield from send_bytes(dut, frame)
                yield from idle(100)
                self.assertEqual(observed.frames, [])
                self.assertEqual(observed.events['frame_dropped'], count)
                self.assertEqual(observed.events['frame_error'], count)
                self.assertEqual(observed.events['frame_overflow'], 0)
            yield from send_bytes(dut, good)
            yield from idle(100)
            self.assertEqual(observed.frames, [good])
            self.assertEqual((yield dut.frame_length), 64)

        run_simulation(dut, [bench(), observed.run()])

    def test_legal_ethernet_padding_and_odd_udp_payload_are_preserved(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=64)
        observed = Observe(dut, ready=lambda cycle: cycle % 5 in (0, 2, 3))
        frames = [
            ethernet_ipv4_udp(b"", padding=bytes(18)),
            ethernet_ipv4_udp(bytes(range(23))),
            ethernet_ipv4_udp(bytes(range(5)), padding=bytes(range(13))),
        ]

        def bench():
            for count, frame in enumerate(frames, 1):
                yield from send_bytes(dut, frame, gaps=1)
                yield from idle(150)
                self.assertEqual(observed.frames, frames[:count])
                self.assertEqual((yield dut.frame_length), len(frame))
                self.assertEqual(observed.events, dict.fromkeys(observed.events, 0))

        run_simulation(dut, [bench(), observed.run()])

    def test_length_taps_do_not_apply_udp_rules_to_other_traffic(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=64)
        observed = Observe(dut)
        non_udp = ethernet_ipv4_udp(bytes(range(8)), udp_length=0, protocol=1)
        arp = bytearray(ethernet_ipv4_udp(bytes(range(8)), ip_length=65535, udp_length=0))
        arp[12:14] = bytes.fromhex("0806")
        frames = [ethernet_ipv4_udp(b"valid predecessor"), non_udp, bytes(arp), b"\x01\x02\x03"]

        def bench():
            for count, frame in enumerate(frames, 1):
                yield from send_bytes(dut, frame)
                yield from idle(100)
                self.assertEqual(observed.frames, frames[:count])
                self.assertEqual(observed.events, dict.fromkeys(observed.events, 0))

        run_simulation(dut, [bench(), observed.run()])

    def test_ipv4_version_and_ihl_must_match_fixed_header_parser(self):
        dut = ValidatedEthernetFrameGate(dw=16, depth=64)
        observed = Observe(dut)
        frames = []
        for version_ihl in (0x05, 0x44, 0x46, 0x65):
            frame = bytearray(ethernet_ipv4_udp(b"header variant"))
            frame[14] = version_ihl
            frames.append(frame)
        minimum_ip = ethernet_ipv4_udp(b"", ip_length=20, protocol=1)[:34]

        def bench():
            for count, frame in enumerate(frames, 1):
                yield from send_bytes(dut, frame)
                yield from idle(100)
                self.assertEqual(observed.beats, [])
                self.assertEqual(observed.events["frame_dropped"], count)
                self.assertEqual(observed.events["frame_error"], count)
            yield from send_bytes(dut, minimum_ip)
            yield from idle(100)
            self.assertEqual(observed.frames, [minimum_ip])

        run_simulation(dut, [bench(), observed.run()])


if __name__ == "__main__":
    unittest.main()
