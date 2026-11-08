"""Host-side Etherbone checks: no sockets or attached hardware are used."""

import socket
import unittest

from tools.lutcam.etherbone_check import check_connection


def fixture():
    names = ("board_status", "camera0_run", "camera0_status", "camera1_run",
             "camera1_status", "camera0_cfg_cx_q16_16", "etherbone_diagnostics_status",
             "etherbone_diagnostics_address", "etherbone_diagnostics_count")
    registers = {name: (4 * i, 1, "rw") for i, name in enumerate(names)}
    mapping = {"registers": registers, "identifier_base": 0x1000,
               "identifier": b"LUTCAM test build\0"}
    values = {address: 0 for address, _, _ in registers.values()}
    for name, value in (("board_status", 3), ("camera0_status", 2),
                        ("camera1_status", 2), ("camera0_cfg_cx_q16_16", 0x9876)):
        values[registers[name][0]] = value
    values.update({0x1000 + 4 * i: value for i, value in enumerate(mapping["identifier"])})
    return mapping, values


class FakeTransport:
    def __init__(self, values):
        self.values = values
        self.read_counter = 0
        self.writes = []
        self.read_sizes = []
        self.fail_after_write = False
        self.retry_next = False
        self.fail_restore = False

    def read(self, address, length=None):
        self.read_counter += 2 if self.retry_next else 1
        self.retry_next = False
        self.read_sizes.append(length)
        if self.fail_after_write and len(self.writes) == 1:
            self.fail_after_write = False
            self.read_counter += 9
            raise socket.timeout("injected timeout")
        if length is None:
            return self.values[address]
        return [self.values[address + 4 * i] for i in range(length)]

    def write(self, address, value):
        assert isinstance(value, int), "writes must be scalar"
        self.writes.append((address, value))
        if self.fail_restore and value == 0x9876:
            raise OSError("injected restoration error")
        self.values[address] = value


class EtherboneCheckTests(unittest.TestCase):
    def setUp(self):
        self.mapping, values = fixture()
        self.comm = FakeTransport(values)
        self.scratch = self.mapping["registers"]["camera0_cfg_cx_q16_16"][0]

    def test_scalar_patterns_bursts_and_original_restored(self):
        result = check_connection(self.comm, self.mapping, iterations=8)
        self.assertTrue(result["ok"], result["errors"])
        self.assertEqual(result["write_test"]["patterns_verified"], 8)
        self.assertEqual(self.comm.values[self.scratch], 0x9876)
        self.assertEqual(self.comm.writes[-1], (self.scratch, 0x9876))
        self.assertTrue(result["write_test"]["restored"])
        self.assertTrue({1, 2, 4, 8}.issubset(self.comm.read_sizes))
        self.assertTrue(all(size is None or size <= 8 for size in self.comm.read_sizes))

    def test_failed_read_restores_and_counts_exhausted_retries(self):
        self.comm.fail_after_write = True
        result = check_connection(self.comm, self.mapping, iterations=4)
        self.assertFalse(result["ok"])
        self.assertEqual(result["read_retries"], 9)
        self.assertEqual(result["read_failures"], 1)
        self.assertTrue(result["write_test"]["restored"])
        self.assertEqual(self.comm.values[self.scratch], 0x9876)

    def test_restoration_failure_is_explicit(self):
        self.comm.fail_after_write = True
        self.comm.fail_restore = True
        result = check_connection(self.comm, self.mapping, iterations=4)
        self.assertFalse(result["write_test"]["restored"])
        self.assertTrue(any("RESTORATION FAILED" in error for error in result["errors"]))

    def test_busy_camera_never_written(self):
        self.comm.values[self.mapping["registers"]["camera0_run"][0]] = 1
        result = check_connection(self.comm, self.mapping, iterations=4)
        self.assertFalse(result["ok"])
        self.assertEqual(self.comm.writes, [])

    def test_read_only_performs_no_writes_even_if_running(self):
        self.comm.values[self.mapping["registers"]["camera0_run"][0]] = 1
        result = check_connection(self.comm, self.mapping, iterations=4, read_only=True)
        self.assertTrue(result["ok"], result["errors"])
        self.assertEqual(self.comm.writes, [])

    def test_identifier_mismatch_prevents_writes(self):
        self.comm.values[0x1000] = ord("X")
        result = check_connection(self.comm, self.mapping, iterations=4)
        self.assertFalse(result["ok"])
        self.assertEqual(self.comm.writes, [])

    def test_successful_hidden_retry_fails_loss_check(self):
        self.comm.retry_next = True
        result = check_connection(self.comm, self.mapping, iterations=4, read_only=True)
        self.assertFalse(result["ok"])
        self.assertEqual(result["read_retries"], 1)
        self.assertEqual(result["read_attempts"], result["read_calls"] + 1)

    def test_diagnostics_fault_is_reported_without_clearing(self):
        address = self.mapping["registers"]["etherbone_diagnostics_status"][0]
        self.comm.values[address] = 5
        result = check_connection(self.comm, self.mapping, iterations=4, read_only=True)
        self.assertFalse(result["ok"])
        self.assertEqual(result["diagnostics_after"]["etherbone"]["status"], 5)
        self.assertEqual(self.comm.writes, [])


if __name__ == "__main__":
    unittest.main()
