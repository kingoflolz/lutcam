"""Small executable checks for the event-driven packet occupancy model."""

from pathlib import Path
import tempfile
import unittest

from packet_service_model import Descriptor, Word, assemble, read_trace, replay


class PacketServiceModelTests(unittest.TestCase):
    def test_wire_schedule_includes_one_bounded_etherbone_response(self):
        result = replay([Descriptor("C", 1, 0, 0, 72)], implementation_gap=0)
        # ceil(170*24/25) data clocks + ceil(146*24/25) EB clocks = 305.
        self.assertEqual(result["totals"]["last_service_cycle"], 610)
        self.assertEqual(result["totals"]["data_datagrams"], 2)
        self.assertEqual(result["totals"]["wire_bytes_including_etherbone"], 632)
        for camera in range(2):
            self.assertEqual(result["streams"][f"camera{camera}.children"]["peak_slots"], 1)

    def test_same_cycle_allocation_precedes_slot_release(self):
        result = replay([Descriptor("C", 1, 0, 0, 72), Descriptor("C", 1, 305, 305, 72)],
                        implementation_gap=0, capacities=(1, 2))
        for camera in range(2):
            stream = result["streams"][f"camera{camera}.children"]
            self.assertEqual(stream["peak_slots"], 2)
            self.assertEqual(stream["first_overflow_cycles"], {"1": 305, "2": None})

    def test_virtual_tail_holds_one_slot_across_many_datagrams(self):
        result = replay([Descriptor("I", 1, 0, 7, 16, True)], fragment_words=2,
                        implementation_gap=0)
        for camera in range(2):
            stream = result["streams"][f"camera{camera}.image"]
            self.assertEqual(stream["peak_slots"], 1)
            self.assertEqual(stream["datagrams"], 4)
            self.assertEqual(stream["udp_bytes"], 4 * 36)
        self.assertEqual(result["totals"]["last_service_cycle"], 7 + 8 * (98 + 141))

    def test_following_frame_allocates_behind_virtual_tail(self):
        descriptors = [Descriptor("I", 1, 0, 7, 16, True),
                       Descriptor("I", 2, 20, 21, 4), Descriptor("I", 2, 22, 23, 4)]
        result = replay(descriptors, fragment_words=2, implementation_gap=0)
        for camera in range(2):
            stream = result["streams"][f"camera{camera}.image"]
            self.assertEqual(stream["peak_slots"], 3)
            self.assertEqual(stream["first_overflow_cycles"]["2"], 22)
            self.assertEqual(stream["datagrams"], 6)

    def test_assembly_measures_padding_groups_and_accepted_poses(self):
        words = [Word(cycle, "I", 7, cycle + 1 if cycle < 3 else 0,
                      sof=cycle == 0, padding=cycle >= 3) for cycle in range(8)]
        words += [Word(10 + index, "C", 7, 0x100000000 + index, last=index == 2)
                  for index in range(3)]
        words += [Word(20 + index, "P", 7, index, last=index == 7) for index in range(8)]
        native_pose = [0x80000000 + index for index in range(22)]
        native_pose[8] = 1
        words += [Word(30 + index, "P", 7, data, last=index == 21)
                  for index, data in enumerate(native_pose)]
        descriptors, frames = assemble(words, frame_words=8, fragment_words=4)
        self.assertEqual(descriptors, [Descriptor("I", 7, 0, 7, 16, True),
                                      Descriptor("C", 7, 10, 12, 24),
                                      Descriptor("P", 7, 30, 53, 88)])
        self.assertEqual(frames["7"]["lwc_semantic_words"], 3)
        self.assertEqual(frames["7"]["lwc_padding_words"], 5)
        self.assertEqual(frames["7"]["image_datagrams"], 2)
        self.assertEqual(frames["7"]["children_datagrams"], 1)
        self.assertEqual(frames["7"]["pose_datagrams"], 1)
        self.assertEqual(frames["7"]["solver_rejections"], 1)

    def test_pose_status_validity_and_flags(self):
        for status in (1, 15, 0, 17):
            with self.subTest(status=hex(status)):
                data = [0] * 22
                data[1], data[8] = 0x80000000, status
                words = [Word(index, "P", 7, value, last=index == 21)
                         for index, value in enumerate(data)]
                if status in (1, 15):
                    descriptors, frames = assemble(words)
                    self.assertEqual(descriptors, [Descriptor("P", 7, 0, 23, 88)])
                    self.assertEqual(frames["7"]["accepted_poses"], 1)
                else:
                    with self.assertRaisesRegex(ValueError, "malformed native"):
                        assemble(words)

    def test_optional_early_padding_commits_known_tail_at_first_marker(self):
        words = [Word(cycle, "I", 7, cycle + 1 if cycle < 3 else 0,
                      sof=cycle == 0, padding=cycle >= 3) for cycle in range(8)]
        late, late_stats = assemble(words, frame_words=8, fragment_words=4)
        early, early_stats = assemble(words, frame_words=8, fragment_words=4, early_padding=True)
        self.assertEqual(late, [Descriptor("I", 7, 0, 7, 16, True)])
        self.assertEqual(early, [Descriptor("I", 7, 0, 3, 16, True)])
        self.assertEqual(early_stats, late_stats)
        self.assertEqual(replay(late, fragment_words=4)["totals"]["last_service_cycle"]
                         - replay(early, fragment_words=4)["totals"]["last_service_cycle"], 4)

    def test_ordinary_fragment_assembly_and_short_last_fragment(self):
        words = [Word(index, "I", 5, index, sof=index == 0) for index in range(10)]
        descriptors, frames = assemble(words, frame_words=10, fragment_words=6)
        self.assertEqual(descriptors, [Descriptor("I", 5, 0, 5, 12), Descriptor("I", 5, 6, 9, 8)])
        self.assertEqual(frames["5"]["image_datagrams"], 2)
        self.assertEqual(frames["5"]["lwc_semantic_words"], 10)

    def test_reader_accepts_actual_trace_format(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "trace.txt"
            path.write_text("# camera trace\n10 I 1 12ab 1 0\n11 C 1 100000001 1\n12 P 1 80000000 0\n")
            self.assertEqual(read_trace(path), [Word(10, "I", 1, 0x12AB, sof=True),
                                               Word(11, "C", 1, 0x100000001, last=True),
                                               Word(12, "P", 1, 0x80000000)])

    def test_incomplete_or_invalid_trace_cannot_silently_pass(self):
        with self.assertRaisesRegex(ValueError, "trace ends"):
            assemble([Word(0, "I", 0, 1, sof=True)], frame_words=8, fragment_words=4)
        with self.assertRaisesRegex(ValueError, "nonzero pure-padding"):
            assemble([Word(0, "I", 0, 1, sof=True, padding=True)], frame_words=8, fragment_words=4)
        malformed = [Word(index, "P", 0, 0x80000000, last=index == 7) for index in range(8)]
        with self.assertRaisesRegex(ValueError, "malformed native"):
            assemble(malformed)


if __name__ == "__main__":
    unittest.main()
