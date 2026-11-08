"""Golden OTA maintenance checks against a separate full-slot flash model."""

from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

from tools.lutcam.flash import FlashClient, FlashError, JEDEC_ID, SLOT_SIZE, IMAGE_CAPACITY
from tools.lutcam.golden_maintenance import (
    ARTIFACTS, GoldenFlashClient, GoldenMaintenance, MaintenancePlan, load_plan, main,
)
from tools.lutcam.ota_bundle import COMMIT_OFFSET, make_record, parse_record
from tests.litex.test_ota import Clock
from tests.litex.test_ota_bundle import remote_image, synthetic_image


OLD_ID, MAINT_ID = "1" * 32, "2" * 32


def fixture_artifacts():
    old_a = remote_image(synthetic_image(11))
    record = bytearray(make_record(old_a, 1, OLD_ID))
    old_a_slot = old_a + b"\xff" * (IMAGE_CAPACITY - len(old_a)) + record + b"\xff" * (4096 - 64)
    maintenance = remote_image(synthetic_image(22))
    return {
        "old_golden": remote_image(synthetic_image(33)).ljust(SLOT_SIZE, b"\xff"),
        "old_a": bytes(old_a_slot), "maintenance_b": maintenance,
        "new_golden": remote_image(synthetic_image(44)).ljust(SLOT_SIZE, b"\xff"),
        "maintenance_build": json.dumps({"build_id": MAINT_ID, "golden": False,
            "golden_maintenance": True, "device": "Ti60F225", "flash_interface": "page-record"}).encode(),
        "maintenance_bundle": json.dumps({"golden_maintenance": True, "slots": [{
            "name": "B", "selector": 2, "image_slot": 2, "address": 2 * SLOT_SIZE,
            "build_id": MAINT_ID, "length": len(maintenance),
            "sha256": hashlib.sha256(maintenance).hexdigest()}]}).encode(),
    }


def hashes(artifacts):
    return {name: hashlib.sha256(data).hexdigest() for name, data in artifacts.items()}


class PlanTests(unittest.TestCase):

    def test_load_binds_all_six_artifacts_before_board_access(self):
        data = fixture_artifacts()
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            spec = {}
            for name, value in data.items():
                (directory / name).write_bytes(value)
                spec[name] = {"path": name, "sha256": hashlib.sha256(value).hexdigest()}
            path = directory / "plan.json"
            path.write_text(json.dumps(spec))
            plan = load_plan(path)
            self.assertEqual(plan.old_a_build_id, OLD_ID)
            self.assertEqual(plan.maintenance_build_id, MAINT_ID)
            self.assertEqual(len(plan.maintenance_slot), SLOT_SIZE)
            (directory / "old_a").write_bytes(b"bad")
            with self.assertRaisesRegex(ValueError, "old_a SHA256"):
                load_plan(path)

    def test_any_artifact_hash_mismatch_is_rejected(self):
        data = fixture_artifacts()
        for name in ARTIFACTS:
            with self.subTest(name=name):
                bad = hashes(data)
                bad[name] = "0" * 64
                with self.assertRaisesRegex(ValueError, "SHA256"):
                    MaintenancePlan.validated(data, bad)

    def test_mode_marker_and_bundle_binding_are_required_even_with_matching_hashes(self):
        for artifact, key, value in (("maintenance_build", "golden_maintenance", False),
                                     ("maintenance_build", "golden", True),
                                     ("maintenance_bundle", "golden_maintenance", False)):
            with self.subTest(artifact=artifact, key=key):
                data = fixture_artifacts()
                metadata = json.loads(data[artifact]); metadata[key] = value
                data[artifact] = json.dumps(metadata).encode()
                with self.assertRaises(ValueError):
                    MaintenancePlan.validated(data, hashes(data))
        for key, value in (("build_id", OLD_ID), ("sha256", "0" * 64), ("address", SLOT_SIZE), ("image_slot", 1)):
            with self.subTest(key=key):
                data = fixture_artifacts()
                metadata = json.loads(data["maintenance_bundle"]); metadata["slots"][0][key] = value
                data["maintenance_bundle"] = json.dumps(metadata).encode()
                with self.assertRaisesRegex(ValueError, "identity/hash/layout"):
                    MaintenancePlan.validated(data, hashes(data))

    def test_unwritable_or_existing_report_fails_before_transport_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            existing = Path(directory) / "existing.json"
            existing.write_text("keep")
            for report in (Path(directory) / "missing" / "report.json", existing):
                with self.subTest(report=report), patch("tools.lutcam.golden_maintenance.create_transport") as connect:
                    with redirect_stdout(io.StringIO()):
                        result = main(["--plan", "unused.json", "--old-csr-csv", "old.csv",
                                       "--maintenance-csr-csv", "maintenance.csv", "--json-output", str(report)])
                    self.assertEqual(result, 1)
                    connect.assert_not_called()
            self.assertEqual(existing.read_text(), "keep")



class Board:
    def __init__(self, plan):
        self.plan = plan
        self.memory = bytearray(plan.old_golden + plan.old_a + b"\xff" * SLOT_SIZE)
        self.slot, self.build_id, self.status = 1, OLD_ID, 0
        self.jedec, self.calls, self.connections = JEDEC_ID, [], []
        self.fail, self.boot_wrong, self.health_bad = None, False, False
        self.change_identity_after_b_read = False

    def connect(self):
        board = self
        class Connection:
            def __init__(self): self.board, self.closed = board, False
            def close(self): self.closed = True
        result = Connection(); self.connections.append(result)
        return result

    def health(self, bus, slot):
        self.calls.append(("health", slot))
        if self.health_bad:
            raise FlashError("camera not idle")
        return {"healthy": True}

    def maintenance(self):
        self.memory[2 * SLOT_SIZE:3 * SLOT_SIZE] = self.plan.maintenance_slot
        self.slot, self.build_id = 2, MAINT_ID


class Boot:
    def __init__(self, bus): self.board = bus.board
    def identity(self):
        return {"image_slot": self.board.slot, "build_id": self.board.build_id, "status": self.board.status}
    def request(self, selector, *, acknowledge=True):
        self.board.calls.append(("boot", selector))
        if selector == 2:
            self.board.slot, self.board.build_id = 2, MAINT_ID
        elif selector == 0:
            self.board.slot, self.board.build_id = (0, "0" * 32) if self.board.boot_wrong else (1, OLD_ID)
        return {"acknowledged": acknowledge}


class MemoryFlash:
    def __init__(self, bus):
        self.board, self.image_slot = bus.board, bus.board.slot
        self.version, self.packet_commands = 0, True
    def jedec(self): return self.board.jedec
    def _range(self, address, length, *, mutate=False):
        FlashClient._range(self, address, length, mutate=mutate)
    def verify(self, address, data, **unused):
        self._range(address, len(data))
        self.board.calls.append(("verify", address, len(data)))
        if self.board.fail == ("verify", address):
            raise FlashError("injected readback failure")
        if self.board.memory[address:address + len(data)] != data:
            raise FlashError("readback mismatch")
        if address == 2 * SLOT_SIZE and self.board.change_identity_after_b_read:
            self.board.build_id = "f" * 32
    def erase(self, address, length):
        self._range(address, length, mutate=True)
        self.board.calls.append(("erase", address, length))
        if self.board.fail == ("erase", address):
            self.board.memory[address:address + length // 2] = b"\xff" * (length // 2)
            raise FlashError("injected erase failure")
        self.board.memory[address:address + length] = b"\xff" * length
    def program(self, address, data, **kwargs):
        self._range(address, len(data), mutate=True)
        self.board.calls.append(("program", address, bytes(data)))
        if self.board.fail == ("program", address):
            raise FlashError("injected program failure")
        for i, value in enumerate(data): self.board.memory[address + i] &= value
        if kwargs.get("verify", True): self.verify(address, data)


class MemoryGolden(GoldenFlashClient):
    # Keep the real authorization and range logic while replacing only the bus
    # backend: production FlashClient serialization has separate protocol tests.
    jedec, verify, erase, program = MemoryFlash.jedec, MemoryFlash.verify, MemoryFlash.erase, MemoryFlash.program


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        data = fixture_artifacts()
        self.plan = MaintenancePlan.validated(data, hashes(data))
        self.board, self.clock = Board(self.plan), Clock()
        self.backend = patch.object(FlashClient, "__init__", MemoryFlash.__init__)
        self.backend.start(); self.addCleanup(self.backend.stop)
    def worker(self):
        return GoldenMaintenance(self.plan, self.board.connect, self.board.health,
            flash_factory=MemoryFlash, golden_factory=MemoryGolden, boot_factory=Boot,
            reboot_timeout=.1, clock=self.clock, sleep=self.clock.sleep)
    def assert_no_mutation(self):
        self.assertFalse(any(call[0] in ("erase", "program", "boot") for call in self.board.calls))
    def test_success_commit_last_golden_only_mutation_then_boot_saved_a(self):
        report = self.worker().run()
        self.assertTrue(report["ok"])
        self.assertEqual(report["final_identity"]["build_id"], OLD_ID)
        self.assertEqual(self.board.memory, self.plan.new_golden + self.plan.old_a + self.plan.maintenance_slot)
        self.assertEqual([call for call in self.board.calls if call[0] == "boot"], [("boot", 2), ("boot", 0)])
        b_boot = self.board.calls.index(("boot", 2))
        programs = [call for call in self.board.calls[:b_boot] if call[0] == "program"]
        self.assertEqual(programs[-1][1], 3 * SLOT_SIZE - 4096 + COMMIT_OFFSET)
        self.assertEqual(programs[-1][2], b"TMOC")
        gold_writes = [call for call in self.board.calls[b_boot:] if call[0] in ("erase", "program")]
        self.assertTrue(gold_writes)
        self.assertTrue(all(call[1] == 0 for call in gold_writes))
        zero_boot = self.board.calls.index(("boot", 0))
        for address in (0, SLOT_SIZE, 2 * SLOT_SIZE):
            self.assertIn(("verify", address, SLOT_SIZE), self.board.calls[b_boot:zero_boot])
        self.assertTrue(all(bus.closed for bus in self.board.connections))
    def test_wrong_initial_identity_or_backup_does_not_mutate(self):
        for kind in ("identity", "golden", "a", "health", "jedec"):
            with self.subTest(kind=kind):
                self.board = Board(self.plan)
                if kind == "identity": self.board.build_id = "f" * 32
                if kind == "golden": self.board.memory[0] ^= 1
                if kind == "a": self.board.memory[2 * SLOT_SIZE - 1] ^= 1
                if kind == "health": self.board.health_bad = True
                if kind == "jedec": self.board.jedec = 0
                with self.assertRaises(FlashError): self.worker().run()
                self.assert_no_mutation()
    def test_authorization_rejects_wrong_b_or_identity_without_mutation(self):
        for kind in ("slot", "build", "busy", "jedec", "payload", "padding", "race"):
            with self.subTest(kind=kind):
                self.board = Board(self.plan); self.board.maintenance()
                if kind == "slot": self.board.slot = 1
                if kind == "build": self.board.build_id = OLD_ID
                if kind == "busy": self.board.status = 1
                if kind == "jedec": self.board.jedec = 0
                if kind == "payload": self.board.memory[2 * SLOT_SIZE + 500] ^= 1
                if kind == "padding": self.board.memory[3 * SLOT_SIZE - 1] ^= 1
                if kind == "race": self.board.change_identity_after_b_read = True
                with self.assertRaises(FlashError):
                    MemoryGolden(self.board.connect(), self.plan, boot_factory=Boot)
                self.assert_no_mutation()
    def test_specialized_client_rejects_all_cross_boundary_non_golden_writes(self):
        self.board.maintenance()
        client = MemoryGolden(self.board.connect(), self.plan, boot_factory=Boot)
        self.board.calls.clear()
        for address, length in ((-1, 1), (0, 0), (SLOT_SIZE - 1, 2), (SLOT_SIZE, 1), (2 * SLOT_SIZE, 1)):
            with self.subTest(address=address, length=length):
                with self.assertRaises(FlashError): client._range(address, length, mutate=True)
        client._range(0, SLOT_SIZE, mutate=True)
        client._range(SLOT_SIZE, SLOT_SIZE)  # Readback of protected A remains legal.
        client._golden_authorized = False
        with self.assertRaises(FlashError): client._range(0, 1, mutate=True)
        self.assert_no_mutation()
    def test_golden_failures_never_reboot_and_can_resume_without_touching_a_b(self):
        for operation in ("erase", "program", "verify"):
            with self.subTest(operation=operation):
                self.board = Board(self.plan); self.board.maintenance()
                self.board.fail = (operation, 0)
                worker = self.worker()
                with self.assertRaises(FlashError): worker.run(resume_maintenance=True)
                self.assertFalse(any(call[0] == "boot" for call in self.board.calls))
                self.assertTrue(worker.report["leave_power_on"])
                self.assertEqual(self.board.slot, 2)
                self.assertEqual(self.board.memory[SLOT_SIZE:], self.plan.old_a + self.plan.maintenance_slot)
                self.board.fail = None
                self.assertTrue(self.worker().run(resume_maintenance=True)["ok"])
    def test_rerun_resets_old_success_before_new_erase_failure(self):
        worker = self.worker(); worker.run()
        self.board.maintenance(); self.board.fail = ("erase", 0)
        with self.assertRaises(FlashError): worker.run(resume_maintenance=True)
        self.assertFalse(worker.report["ok"])
        self.assertFalse(worker.report["golden_verified"])
        self.assertTrue(worker.report["leave_power_on"])
    def test_wrong_final_boot_fails_after_verification_without_another_mutation(self):
        self.board.maintenance(); self.board.boot_wrong = True
        worker = self.worker()
        with self.assertRaisesRegex(FlashError, "did not reconnect"):
            worker.run(resume_maintenance=True)
        self.assertTrue(worker.report["golden_verified"])
        self.assertFalse(worker.report["ok"])
        self.assertEqual([call for call in self.board.calls if call[0] == "boot"], [("boot", 0)])


if __name__ == "__main__": unittest.main()
