"""Actual localizer output through production pose queues and UDP serialization.

The input packet is generated from the production RTL on every test run. The
localizer emits an unversioned status; the packetizer must forward arbitrary
upper-nibble bits unchanged and enforce only the remaining packet contract.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from migen.sim import run_simulation

from fpga.litex.lutcam.packet import PosePacketizer
from tools.lutcam.receive import CHILD, POSE, parse_datagram
from test_packet import Capture, configure, header, idle, pose_result, word_bytes
from test_shared_packet import SharedHarness, child_group, configure as configure_shared, send_pose


ROOT = Path(__file__).resolve().parents[2]


@unittest.skipUnless(shutil.which("verilator"), "Verilator is required")
class LocalizerPoseTransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory(prefix="lutcam-localizer-transport-")
        cls.addClassCleanup(cls.build.cleanup)
        directory = Path(cls.build.name)
        sources = (ROOT / "rtl/lftag/sources.f").read_text().splitlines()
        result = subprocess.run(
            ["verilator", "--binary", "--timing", "--assert", "-Wno-fatal",
             "-j", "2", "--top-module", "retained_nine_tb",
             "--Mdir", str(directory), f"-I{ROOT / 'rtl/lftag'}",
             *(str(ROOT / source) for source in sources if source.strip()),
             str(ROOT / "tests/lftag/retained_nine_tb.sv")],
            cwd=ROOT, capture_output=True, text=True, timeout=240,
        )
        if result.returncode:
            raise RuntimeError(f"Localizer compilation failed:\n{result.stdout}\n{result.stderr}")
        packet_path = directory / "accepted-pose.hex"
        result = subprocess.run(
            [str(directory / "Vretained_nine_tb"), "+CASE=0", f"+PACKET_OUTPUT={packet_path}"],
            cwd=ROOT, capture_output=True, text=True, timeout=30,
        )
        if result.returncode:
            raise RuntimeError(f"Localizer did not produce an accepted pose:\n{result.stdout}\n{result.stderr}")
        cls.actual_pose = tuple(int(word, 16) for word in packet_path.read_text().split())
        if len(cls.actual_pose) != 22 or cls.actual_pose[8] >> 28 != 0:
            raise AssertionError("production localizer fixture did not emit an unversioned pose")

    def test_actual_localizer_and_every_opaque_upper_nibble_preserve_all_status_flags(self):
        dut = PosePacketizer(camera_id=0, slots=2)
        capture = Capture(dut.source, ready=lambda cycle: cycle > 100 and cycle % 7 not in (1, 2, 3))
        packets = [self.actual_pose]
        for upper_nibble in range(16):
            for flags in range(1, 16, 2):
                words = list(self.actual_pose)
                words[8] = upper_nibble << 28 | flags
                packets.append(tuple(words))
        observed = {}

        def stimulus():
            yield from configure(dut)
            for index, words in enumerate(packets):
                yield from pose_result(dut, words, 0x1234005A + index)
            yield from idle(1000)
            observed["faults"] = (yield dut.faults.status)
            observed["committed"] = (yield dut.packets_committed.status)
            observed["sent"] = (yield dut.packets_sent.status)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(observed, {"faults": 0, "committed": len(packets), "sent": len(packets)})
        self.assertEqual(len(capture.packets), len(packets))
        for index, (raw, words) in enumerate(zip(capture.packets, packets)):
            self.assertEqual(raw, header(0, POSE, 0x1234005A + index, index, 0, 88)
                             + word_bytes(words, 4))
        # The production unversioned output also remains readable by the host.
        self.assertEqual(parse_datagram(capture.packets[0]).pose_words(), self.actual_pose)

    def test_production_shared_ingress_batches_actual_unversioned_poses_for_both_cameras(self):
        dut = SharedHarness(
            child_slots=8, frame_words=64000, external_destination=True,
            external_context_held=True, compact_padding=True, batched_packets=True,
            accepted_centroids=True, shared_solver_ingress=True, held_frame_id=True,
        )
        transport = dut.transport
        readiness = {"enabled": False}
        capture = Capture(transport.source,
                          ready=lambda cycle: readiness["enabled"] and cycle % 7 not in (1, 2, 3))
        frames = (0x1234005A, 0x2234005A)
        flagged = list(self.actual_pose)
        flagged[8] ^= 4
        poses = (self.actual_pose, tuple(flagged))
        groups = tuple(tuple(1 << 32 | camera << 24 | record << 16 | index + 1
                             for index in range(9))
                       for camera in range(2) for record in range(2))
        observed = {}

        def stimulus():
            yield from configure_shared(transport)
            for index in range(6):
                yield dut.lookup_ips[index].eq(0xC0A80102)
                yield dut.lookup_ports[index].eq(6000 + index)
            for camera, frame in enumerate(frames):
                for channel in transport.channels[3 * camera:3 * camera + 3]:
                    yield channel.sink.frame_id.eq(frame)
                for record, words in enumerate(poses):
                    # Production single-owner contract: pose, then its accepted
                    # centroid group, before another camera may acquire ingress.
                    yield from send_pose(transport.channels[3 * camera + 2], words, frame)
                    yield from child_group(transport.channels[3 * camera + 1],
                                           groups[2 * camera + record], frame)
            readiness["enabled"] = True
            for _ in range(3000):
                if (yield transport.drained):
                    break
                yield
            else:
                raise AssertionError("production shared transport did not drain")
            yield from idle(4)
            observed["faults"] = []
            observed["protocol_errors"] = []
            for index, channel in enumerate(transport.channels):
                observed["faults"].append((yield channel.faults))
                observed["protocol_errors"].append((yield dut.protocol_errors[index]))
            observed["drained"] = (yield transport.drained)

        run_simulation(dut, [stimulus(), capture.run()])
        self.assertEqual(observed, {"faults": [0] * 6, "protocol_errors": [0] * 6, "drained": 1})
        parsed = {}
        for packet in map(parse_datagram, capture.packets):
            parsed.setdefault((packet.camera, packet.stream), []).append(packet)
        self.assertEqual(set(parsed), {(0, CHILD), (0, POSE), (1, CHILD), (1, POSE)})
        self.assertTrue(any(len(packet.poses()) == 2 for camera in range(2)
                            for packet in parsed[camera, POSE]))
        for camera, frame in enumerate(frames):
            for stream in (CHILD, POSE):
                for sequence, packet in enumerate(parsed[camera, stream]):
                    self.assertEqual((packet.version, packet.frame_id, packet.sequence),
                                     (0, frame, sequence))
            self.assertEqual(tuple(words for packet in parsed[camera, POSE]
                                   for words in packet.poses()), poses)
            self.assertEqual(b"".join(packet.payload for packet in parsed[camera, CHILD]),
                             b"".join(word_bytes(group, 8)
                                      for group in groups[2 * camera:2 * camera + 2]))


if __name__ == "__main__":
    unittest.main()
