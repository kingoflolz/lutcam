"""Replace golden through an explicitly qualified temporary maintenance B image.

This separate workflow deliberately permits writes to the recovery slot. Power
must remain stable from its first erase through verified completion; after a
failure, leave maintenance B running and resume with --resume-maintenance.
Normal FlashClient and application OTA permissions are unchanged. Finish the
ordinary application OTA afterwards to replace the privileged maintenance B.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import struct
import time
import zlib

from .flash import (BootClient, FlashClient, FlashError, JEDEC_ID, SLOT_SIZE,
                    IMAGE_CAPACITY, add_connection_arguments, create_transport)
from .ota_bundle import COMMIT_OFFSET, make_record, parse_image, parse_record
from .update import check_health, load_health_map


ARTIFACTS = ("old_golden", "old_a", "maintenance_b", "new_golden", "maintenance_build", "maintenance_bundle")


@dataclass(frozen=True)
class MaintenancePlan:
    old_golden: bytes
    old_a: bytes
    maintenance_b: bytes
    new_golden: bytes
    maintenance_build_id: str
    old_a_build_id: str
    maintenance_slot: bytes

    @classmethod
    def validated(cls, artifacts, hashes):
        if set(artifacts) != set(ARTIFACTS) or set(hashes) != set(ARTIFACTS):
            raise ValueError("maintenance plan requires all six artifacts and SHA256 values")
        for name in ARTIFACTS:
            if not isinstance(artifacts[name], bytes) or hashlib.sha256(artifacts[name]).hexdigest() != hashes[name]:
                raise ValueError(f"{name} SHA256 mismatch")
        for name in ("old_golden", "old_a", "new_golden"):
            if len(artifacts[name]) != SLOT_SIZE:
                raise ValueError(f"{name} must contain its complete 2 MiB slot")
        image = artifacts["maintenance_b"]
        if not 0 < len(image) <= IMAGE_CAPACITY:
            raise ValueError("maintenance B overlaps its metadata sector")
        metadata = json.loads(artifacts["maintenance_build"])
        build_id = metadata.get("build_id", "")
        if (not isinstance(build_id, str) or re.fullmatch("[0-9a-f]{32}", build_id) is None
                or metadata.get("golden_maintenance") is not True or metadata.get("golden") is not False
                or metadata.get("device") != "Ti60F225" or metadata.get("flash_interface") != "page-record"):
            raise ValueError("build metadata does not identify an audited maintenance application")
        bundle = json.loads(artifacts["maintenance_bundle"])
        entries = [entry for entry in bundle.get("slots", []) if entry.get("name") == "B"]
        if bundle.get("golden_maintenance") is not True or len(entries) != 1:
            raise ValueError("bundle is not explicitly marked for golden maintenance")
        entry = entries[0]
        if (entry.get("selector") != 2 or entry.get("image_slot") != 2
                or entry.get("address") != 2 * SLOT_SIZE or entry.get("build_id") != build_id
                or entry.get("length") != len(image) or entry.get("sha256") != hashes["maintenance_b"]):
            raise ValueError("maintenance bundle B identity/hash/layout does not match the plan")
        old_a = artifacts["old_a"]
        old_record = old_a[IMAGE_CAPACITY:IMAGE_CAPACITY + 64]
        length = int.from_bytes(old_record[16:20], "little")
        old_identity = parse_record(old_record, 1, old_a[:length])
        for data in (artifacts["old_golden"], old_a[:length], image, artifacts["new_golden"]):
            parse_image(data, require_remote=True)
        record = make_record(image, 2, build_id)
        parse_record(record, 2, image)
        slot = image + b"\xff" * (IMAGE_CAPACITY - len(image)) + record + b"\xff" * (4096 - 64)
        return cls(artifacts["old_golden"], old_a, image, artifacts["new_golden"],
                   build_id, old_identity["build_id"], slot)


def load_plan(path):
    """Read the reviewed plan, then hash and validate every byte before connecting."""
    path = Path(path)
    specification = json.loads(path.read_text(encoding="utf-8"))
    if set(specification) != set(ARTIFACTS):
        raise ValueError("unexpected or missing maintenance plan artifacts")
    artifacts, hashes = {}, {}
    for name in ARTIFACTS:
        entry = specification[name]
        source = path.parent / entry["path"]
        limit = 65536 if name in ("maintenance_build", "maintenance_bundle") else SLOT_SIZE
        if source.stat().st_size > limit:
            raise ValueError(f"{name} exceeds its bounded artifact size")
        artifacts[name], hashes[name] = source.read_bytes(), entry["sha256"]
    return MaintenancePlan.validated(artifacts, hashes)


class GoldenFlashClient(FlashClient):
    """Authorize only golden mutation after exact maintenance-B proof on this bus."""

    def __init__(self, bus, plan, *, boot_factory=BootClient, **kwargs):
        self._golden_authorized = False
        super().__init__(bus, **kwargs)
        boot = boot_factory(bus)
        identity = boot.identity()
        if (identity["image_slot"] != 2 or self.image_slot != 2
                or identity["build_id"] != plan.maintenance_build_id or identity["status"] != 0
                or not self.packet_commands):
            raise FlashError("golden mutation requires the exact idle maintenance B identity")
        if self.jedec() != JEDEC_ID:
            raise FlashError("unexpected maintenance flash JEDEC ID")
        self.verify(2 * SLOT_SIZE, plan.maintenance_slot)
        if boot.identity() != identity:
            raise FlashError("maintenance identity changed during authorization readback")
        self._golden_authorized = True

    def _range(self, address, length, *, mutate=False):
        # Preserve all normal bounds for reads. Mutation gets a smaller aperture
        # than the ordinary client, with explicit proof before its first use.
        super()._range(address, length, mutate=False)
        if mutate and (not self._golden_authorized or self.image_slot != 2
                       or address >= SLOT_SIZE or address + length > SLOT_SIZE):
            raise FlashError("maintenance writes are confined to the verified golden slot")


class GoldenMaintenance:
    def __init__(self, plan, connect, health, *, reboot_timeout=90.0, progress=None,
                 flash_factory=FlashClient, golden_factory=GoldenFlashClient,
                 boot_factory=BootClient, clock=time.monotonic, sleep=time.sleep):
        if reboot_timeout <= 0:
            raise ValueError("reboot timeout must be positive")
        self.plan, self.connect, self.health = plan, connect, health
        self.flash_factory, self.golden_factory, self.boot_factory = flash_factory, golden_factory, boot_factory
        self.clock, self.sleep, self.reboot_timeout = clock, sleep, reboot_timeout
        self.progress = progress or (lambda event: None)
        self.bus = None
        self.report = {"ok": False, "golden_erase_started": False, "golden_verified": False,
                       "maintenance_b_left_installed": True, "events": []}

    def _event(self, phase, **detail):
        event = {"phase": phase, **detail}
        self.report["events"].append(event)
        self.progress(event)

    def _close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    def _identity(self, slot, build_id):
        identity = self.boot_factory(self.bus).identity()
        if identity != {"image_slot": slot, "build_id": build_id, "status": 0}:
            raise FlashError(f"expected idle slot {slot} build {build_id}, got {identity}")
        return identity

    def _normal_client(self, slot, build_id):
        self._identity(slot, build_id)
        client = self.flash_factory(self.bus)
        if client.image_slot != slot or client.jedec() != JEDEC_ID:
            raise FlashError("flash identity/JEDEC does not match maintenance preflight")
        self.health(self.bus, slot)
        return client

    def _stage_b(self, flash):
        self._event("stage_maintenance_b", bytes=len(self.plan.maintenance_b))
        base, metadata = 2 * SLOT_SIZE, 3 * SLOT_SIZE - 4096
        flash.erase(metadata, 4096)  # Invalidate the old commit before payload.
        flash.erase(base, SLOT_SIZE)
        flash.program(base, self.plan.maintenance_b, check_erased=False, verify=False)
        flash.verify(base, self.plan.maintenance_b)
        record = self.plan.maintenance_slot[IMAGE_CAPACITY:IMAGE_CAPACITY + 64]
        for start, end in ((0, COMMIT_OFFSET), (COMMIT_OFFSET + 4, 64)):
            flash.program(metadata + start, record[start:end], check_erased=False, verify=True)
        flash.program(metadata + COMMIT_OFFSET, record[COMMIT_OFFSET:COMMIT_OFFSET + 4],
                      check_erased=False, verify=True)
        flash.verify(base, self.plan.maintenance_slot)
        flash.verify(0, self.plan.old_golden)
        flash.verify(SLOT_SIZE, self.plan.old_a)
        self._event("maintenance_b_staged")

    def _reboot(self, selector, expected_slot, expected_build):
        self._event("reboot", selector=selector, expected_slot=expected_slot)
        deadline = self.clock() + self.reboot_timeout
        prepare = getattr(self.bus, "prepare_reboot", None)
        transition = prepare() if prepare is not None else None
        if self.clock() >= deadline:
            raise TimeoutError("reboot deadline expired before request")
        self.boot_factory(self.bus).request(selector, acknowledge=transition is None)
        if transition is not None:
            transition(deadline, clock=self.clock, sleep=self.sleep)
        self._close()
        last_error = None
        while self.clock() < deadline:
            try:
                self.bus = self.connect()
                identity = self.boot_factory(self.bus).identity()
                if (identity["image_slot"] == expected_slot and identity["build_id"] == expected_build
                        and identity["status"] == 0 and self.clock() < deadline):
                    self._event("reconnected", identity=identity)
                    return
                last_error = FlashError(f"unexpected boot identity {identity}")
            except (OSError, TimeoutError, FlashError) as error:
                last_error = error
            self._close()
            remaining = deadline - self.clock()
            if remaining > 0:
                self.sleep(min(.05, remaining))
        raise FlashError(f"expected image did not reconnect: {last_error}")

    def run(self, *, resume_maintenance=False):
        self.report.update(ok=False, golden_erase_started=False, golden_verified=False, events=[])
        for key in ("error", "leave_power_on", "final_identity"):
            self.report.pop(key, None)
        try:
            self.bus = self.connect()
            if not resume_maintenance:
                flash = self._normal_client(1, self.plan.old_a_build_id)
                self._event("verify_backups")
                flash.verify(0, self.plan.old_golden)
                flash.verify(SLOT_SIZE, self.plan.old_a)
                self._stage_b(flash)
                self._reboot(2, 2, self.plan.maintenance_build_id)
            self._normal_client(2, self.plan.maintenance_build_id)
            # The specialized client independently checks identity, JEDEC and
            # the whole B slot before granting its golden-only write aperture.
            golden = self.golden_factory(self.bus, self.plan, boot_factory=self.boot_factory)
            golden.verify(SLOT_SIZE, self.plan.old_a)
            self._identity(2, self.plan.maintenance_build_id)
            self.health(self.bus, 2)
            self.report["golden_erase_started"] = True
            self._event("erase_golden", bytes=SLOT_SIZE)
            golden.erase(0, SLOT_SIZE)
            self._event("program_golden", bytes=SLOT_SIZE)
            golden.program(0, self.plan.new_golden, check_erased=False, verify=False)
            golden.verify(0, self.plan.new_golden)
            self.report["golden_verified"] = True
            self._event("golden_verified", sha256=hashlib.sha256(self.plan.new_golden).hexdigest())
            golden.verify(SLOT_SIZE, self.plan.old_a)
            golden.verify(2 * SLOT_SIZE, self.plan.maintenance_slot)
            self._identity(2, self.plan.maintenance_build_id)
            self._reboot(0, 1, self.plan.old_a_build_id)
            flash = self._normal_client(1, self.plan.old_a_build_id)
            for address, expected in ((0, self.plan.new_golden), (SLOT_SIZE, self.plan.old_a),
                                      (2 * SLOT_SIZE, self.plan.maintenance_slot)):
                flash.verify(address, expected)
            self.report["ok"] = True
            self.report["final_identity"] = self._identity(1, self.plan.old_a_build_id)
            self._event("complete", running_slot=1, maintenance_b_left_installed=True)
            return self.report
        except (Exception, KeyboardInterrupt) as error:
            self.report["error"] = f"{type(error).__name__}: {error}"
            self.report["leave_power_on"] = bool(self.report["golden_erase_started"])
            self._event("failed", error=self.report["error"], leave_power_on=self.report["leave_power_on"])
            raise
        finally:
            self._close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--old-csr-csv", required=True, type=Path)
    parser.add_argument("--maintenance-csr-csv", required=True, type=Path)
    parser.add_argument("--json-output", required=True, type=Path)
    parser.add_argument("--reboot-timeout", type=float, default=90.0)
    parser.add_argument("--resume-maintenance", action="store_true",
                        help="resume only from the exact, verified maintenance B; golden may be partial")
    args = parser.parse_args(argv)
    worker = None
    report_file = None
    try:
        # Reserve a writable, new report before connecting or mutating flash.
        # Keep it open so a late path/permission error cannot lose the result.
        report_file = args.json_output.open("x", encoding="utf-8")
        report_file.write(json.dumps({"ok": False, "state": "preparing", "plan": str(args.plan)}) + "\n")
        report_file.flush()
        plan = load_plan(args.plan)
        maps = {1: load_health_map(args.old_csr_csv), 2: load_health_map(args.maintenance_csr_csv)}
        worker = GoldenMaintenance(plan, lambda: create_transport(args).open(),
            lambda bus, slot: check_health(bus, slot, maps[slot]), reboot_timeout=args.reboot_timeout,
            progress=lambda event: print(json.dumps(event), flush=True))
        report = worker.run(resume_maintenance=args.resume_maintenance)
    except (Exception, KeyboardInterrupt) as error:
        report = worker.report if worker else {"ok": False, "error": f"{type(error).__name__}: {error}"}
    if report_file is not None:
        try:
            report_file.seek(0)
            report_file.write(json.dumps(report, indent=2) + "\n")
            report_file.truncate()
            report_file.flush()
        finally:
            report_file.close()
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
