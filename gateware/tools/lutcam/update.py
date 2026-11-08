"""Install a verified LUTCAM OTA bundle through application B, then application A.

Golden is never modified. Preflight reads back the bundled golden image before
any erase, including when an application is running. A remains intact until B
has booted, exposed the OTA interfaces, passed health checks, and read its own
programmed image back. The last step normally boots A and checks it too. Mirroring the new release to A
consumes the old-release rollback copy; golden remains the recovery path.
"""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

from .flash import (BootClient, FlashClient, FlashError, JEDEC_ID,
                    SLOT_SIZE, IMAGE_CAPACITY, add_connection_arguments, create_transport)


def load_health_map(path):
    """Load only application health registers, without pinning the timestamp ID."""
    registers = {}
    with Path(path).open(encoding="utf-8", newline="") as source:
        for row in csv.reader(line for line in source if line.strip() and not line.startswith("#")):
            if len(row) == 5 and row[0] == "csr_register":
                _, name, address, size, mode = row
                if int(size) == 1 and mode in ("ro", "rw"):
                    registers[name] = int(address, 0)
    required = ["board_status", "camera0_run", "camera0_status", "camera1_run", "camera1_status",
                "etherbone_diagnostics_status", "etherbone_diagnostics_count"]
    missing = [name for name in required if name not in registers]
    if missing:
        raise ValueError(f"CSR CSV is missing required application health registers: {missing}")
    return registers


def check_health(bus, image_slot, registers):
    """Read health without stopping cameras or clearing evidence of bus faults."""
    if image_slot == 0:
        return {"recovery": True}  # Minimal golden deliberately has no camera CSR bank.
    result = {name: int(bus.read(address)) for name, address in registers.items()
              if name in ("board_status", "camera0_run", "camera0_status", "camera1_run", "camera1_status",
                          "etherbone_diagnostics_status", "etherbone_diagnostics_count",
                          "usb_diagnostics_status", "usb_diagnostics_count")}
    if result["board_status"] & 3 != 3:
        raise FlashError("application does not report PLL lock and PHY reset release")
    for camera in (0, 1):
        if result[f"camera{camera}_run"] or not result[f"camera{camera}_status"] & 2:
            raise FlashError(f"camera {camera} must be stopped and idle before OTA")
    for name, value in result.items():
        if "diagnostics" in name and value:
            raise FlashError(f"application health check failed: {name}={value}")
    if result["board_status"] & 16:
        from .supervisor import CameraSupervisor
        supervisor = CameraSupervisor.discover(bus)
        supervisor.wait()
        result["supervisor"] = supervisor.snapshot()
    return result


class Updater:
    """One update transaction; injectable transports allow fault simulation.

    Construct from a fully validated bundle returned by ota_bundle.load_bundle.
    The constructor additionally checks the exact bytes consumed by this
    transaction, before it opens a socket or erases a sector.
    """

    def __init__(self, manifest, images, connect, health, *, reboot_timeout=90.0,
                 final_a=True, progress=None, clock=time.monotonic, sleep=time.sleep,
                 flash_factory=FlashClient, boot_factory=BootClient):
        from .ota_bundle import parse_record
        if manifest.get("golden_maintenance", False):
            raise ValueError("maintenance-only bundle requires the specialized golden maintenance workflow")
        if reboot_timeout <= 0:
            raise ValueError("reboot timeout must be positive")
        self.slots = {entry["name"]: dict(entry) for entry in manifest["slots"]}
        if set(self.slots) != {"golden", "A", "B"} or set(images) != set(self.slots):
            raise ValueError("update needs golden, A and B bundle entries")
        self.images = {name: bytes(data) for name, data in images.items()}
        if self.slots["A"]["build_id"] != self.slots["B"]["build_id"]:
            raise ValueError("A and B must be built from the same release ID")
        self.records = {}
        for name, slot in self.slots.items():
            image = self.images[name]
            selector = {"golden": 0, "A": 1, "B": 2}[name]
            if (slot["selector"] != selector or slot["image_slot"] != selector or
                    slot["address"] != selector * SLOT_SIZE or slot["length"] != len(image) or
                    slot["sha256"] != hashlib.sha256(image).hexdigest()):
                raise ValueError(f"invalid {name} image layout, length or hash")
            if name == "golden" and not 0 < len(image) <= SLOT_SIZE:
                raise ValueError("golden image must be nonempty and fit its protected slot")
            if name != "golden":
                if not 0 < len(image) <= IMAGE_CAPACITY or slot["metadata_address"] != (selector + 1)*SLOT_SIZE - 4096:
                    raise ValueError(f"{name} overlaps its validity record")
                record = bytes.fromhex(slot["metadata_hex"])
                parsed = parse_record(record, expected_slot=selector, image=image)
                if parsed["build_id"] != slot["build_id"]:
                    raise ValueError(f"{name} validity record build ID mismatch")
                self.records[name] = record
        self.connect, self.health = connect, health
        self.reboot_timeout, self.final_a = reboot_timeout, final_a
        self.progress = progress or (lambda event: None)
        self.clock, self.sleep = clock, sleep
        self.flash_factory, self.boot_factory = flash_factory, boot_factory
        self.bus = None
        self._golden_verified = False
        self._verify_golden_required = any(record[8:12] != b"\x01\x00\x00\x00"
                                           for record in self.records.values())
        self.report = {"ok": False, "started_utc": datetime.now(timezone.utc).isoformat(), "events": [],
                       "golden_modified": False, "a_erase_started": False, "final_a_requested": final_a,
                       "identity_source": "compiled image_slot tag plus 128-bit build ID; not physical boot-address readback"}

    def _event(self, phase, **detail):
        event = {"phase": phase, **detail}
        self.report["events"].append(event)
        self.progress(event)

    def _byte_progress(self, phase, name):
        last = 0
        def progress(completed, total):
            nonlocal last
            if completed == total or completed - last >= 65536:
                last = completed
                self._event(phase, slot=name, completed=completed, total=total)
        return progress

    def _close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    def _identity_matches(self, identity, name):
        expected = self.slots[name]
        return identity["image_slot"] == expected["image_slot"] and identity["build_id"] == expected["build_id"]

    def _probe_identity(self):
        # The optional transport hook changes discovery reads only. Normal
        # timeout/retry settings are restored before preflight or mutation.
        probe = getattr(self.bus, "discovery_probe", None)
        with probe() if probe is not None else nullcontext():
            return self.boot_factory(self.bus).identity()

    def _stable_connection(self):
        deadline = self.clock() + self.reboot_timeout
        last_error = None
        while self.clock() < deadline:
            try:
                self.bus = self.connect()
                identity = self._probe_identity()
                if self.clock() >= deadline:
                    raise TimeoutError("initial identity arrived after the readiness deadline")
                # Golden may still be scanning records, or about to boot an
                # application. Never compete with its flash-controller owner.
                if identity["status"] & 1 or (identity["image_slot"] == 0 and identity["status"] & 4):
                    last_error = FlashError("golden startup scan/reconfiguration is still in progress")
                else:
                    return identity
            except (OSError, TimeoutError, FlashError) as error:
                last_error = error
            self._close()
            remaining = deadline - self.clock()
            if remaining > 0:
                self.sleep(min(0.01, remaining))
        raise FlashError(f"board did not become ready before timeout: {last_error}")

    def _preflight(self, expected=None, *, verify_image=False):
        boot = self.boot_factory(self.bus)
        identity = boot.identity()
        if expected is not None and not self._identity_matches(identity, expected):
            raise FlashError(f"expected {expected} slot/build identity, got {identity}")
        if identity["status"] & (1 | 4):
            raise FlashError("board is preparing a reboot or scanning flash")
        flash = self.flash_factory(self.bus)
        if flash.image_slot != identity["image_slot"]:
            raise FlashError("flash and boot interfaces disagree about running image identity")
        jedec = flash.jedec()
        if jedec != JEDEC_ID:
            raise FlashError(f"wrong flash JEDEC ID: 0x{jedec:06x}")
        health = self.health(self.bus, identity["image_slot"])
        if self._verify_golden_required and not self._golden_verified:
            self._verify_golden(flash)
        if verify_image:
            if expected not in ("A", "B"):
                raise ValueError("full application verification requires A or B")
            self._verify_slot(flash, expected)
            if boot.identity() != identity:
                raise FlashError("running identity changed during verification")
            health = self.health(self.bus, identity["image_slot"])
        self._event("preflight", identity=identity, jedec=f"0x{jedec:06x}", health=health,
                    flash_image_verified=verify_image)
        return flash, identity

    def _verify_golden(self, flash):
        # Application registers cannot identify the dormant recovery scanner.
        # Match the already hash-validated bundle bytes before publishing any
        # new validity format. Golden is protected throughout all later boots,
        # so this readback is required once per run, not after each reboot.
        image = self.images["golden"]
        self._event("verify_golden", bytes=len(image))
        started = self.clock()
        try:
            flash.verify(0, image, progress=self._byte_progress("verify_progress", "golden"))
        except FlashError as error:
            raise FlashError("installed golden image does not match the bundle; install and verify "
                             "the bundled golden scanner before updating applications") from error
        self._golden_verified = True
        detail = {"bytes": len(image), "sha256": self.slots["golden"]["sha256"],
                  "seconds": self.clock() - started}
        self.report["golden_verification"] = detail
        self._event("golden_verified", **detail)

    def _verify_slot(self, flash, name):
        slot = self.slots[name]
        self._event("verify_image", slot=name, bytes=len(self.images[name]))
        flash.verify(slot["address"], self.images[name], progress=self._byte_progress("verify_progress", name))
        flash.verify(slot["metadata_address"], self.records[name])

    def _install(self, flash, name):
        from .ota_bundle import COMMIT_OFFSET
        slot, data, record = self.slots[name], self.images[name], self.records[name]
        if flash.image_slot == slot["image_slot"]:
            raise FlashError(f"refusing to replace running application {name}")
        erase_length = ((len(data) + 65535) // 65536) * 65536
        if not 0 < erase_length <= SLOT_SIZE:
            raise FlashError(f"{name} payload erase exceeds its slot")
        self._event("erase", slot=name, region="metadata", address=slot["metadata_address"], bytes=4096)
        if name == "A":
            self.report["a_erase_started"] = True
        # Erase the validity sector first. Interrupted installs must not leave
        # a previous commit marker attached to partially replaced image data.
        flash.erase(slot["metadata_address"], 4096)
        # Only the declared image length participates in boot validation. Leave
        # the unused tail untouched; rounding to whole blocks avoids slow sector
        # erases. A full-slot span safely erases the metadata sector again here.
        self._event("erase", slot=name, region="payload", address=slot["address"], bytes=erase_length)
        flash.erase(slot["address"], erase_length)
        self._event("program", slot=name, bytes=len(data))
        flash.program(slot["address"], data, check_erased=False, verify=False,
                      progress=self._byte_progress("program_progress", name))
        self._event("verify_image", slot=name, bytes=len(data))
        flash.verify(slot["address"], data, progress=self._byte_progress("verify_progress", name))
        # Publish validity only after a byte-for-byte image readback. Write
        # and verify the body before the commit word, which is programmed last.
        self._event("commit_record", slot=name)
        for start, end in ((0, COMMIT_OFFSET), (COMMIT_OFFSET + 4, len(record))):
            if end > start:
                flash.program(slot["metadata_address"] + start, record[start:end],
                              check_erased=False, verify=True)
        flash.program(slot["metadata_address"] + COMMIT_OFFSET, record[COMMIT_OFFSET:COMMIT_OFFSET+4],
                      check_erased=False, verify=True)
        flash.verify(slot["metadata_address"], record)
        self._event("installed", slot=name, sha256=slot["sha256"])

    def _reboot(self, name):
        expected = self.slots[name]
        previous = self.boot_factory(self.bus).identity()
        self._event("reboot", slot=name)
        deadline = self.clock() + self.reboot_timeout
        prepare = getattr(self.bus, "prepare_reboot", None)
        transition = prepare() if prepare is not None else None
        if self.clock() >= deadline:
            raise FlashError("reboot deadline expired before the request")
        boot = self.boot_factory(self.bus)
        request = (boot.request(expected["selector"], acknowledge=False) if transition is not None
                   else boot.request(expected["selector"]))
        saw_disconnect = False
        if transition is not None:
            request["usb_transition"] = transition(deadline, clock=self.clock, sleep=self.sleep)
            saw_disconnect = True
        self._close()  # Drop every outstanding response before the new boot.
        observed = None
        last_error = None
        while self.clock() < deadline:
            observed = None
            keep_connection = False
            try:
                self.bus = self.connect()  # New source port and response nonce.
                observed = self._probe_identity()
                if self.clock() >= deadline:
                    raise TimeoutError("identity arrived after the reboot deadline")
                if self._identity_matches(observed, name) and not observed["status"] & 1:
                    # If the short outage was missed, a different slot tag plus
                    # expected build ID on a fresh socket establishes the boot.
                    if saw_disconnect or observed["image_slot"] != previous["image_slot"]:
                        self._event("reconnected", slot=name, identity=observed,
                                    observed_disconnect=saw_disconnect, request=request)
                        keep_connection = True
                        return
                if observed["image_slot"] == 0 and not observed["status"] & (1 | 4) and observed["status"] & (2 | 8):
                    raise FlashError(f"{name} did not boot; board returned to golden recovery: {observed}")
                if observed["image_slot"] not in (previous["image_slot"], expected["image_slot"], 0):
                    raise FlashError(f"unexpected application after requesting {name}: {observed}")
                if observed["image_slot"] == expected["image_slot"] and observed["build_id"] != expected["build_id"]:
                    raise FlashError(f"{name} booted with the wrong build ID: {observed}")
            except (OSError, TimeoutError) as error:
                saw_disconnect = True
                last_error = error
            finally:
                # Keep the successful connection; close all other candidates.
                if not keep_connection:
                    self._close()
            remaining = deadline - self.clock()
            if remaining > 0:
                self.sleep(min(0.01, remaining))
        raise FlashError(f"{name} did not reconnect before timeout; last identity={observed}, transport={last_error}. "
                         "No subsequent slot erase was attempted.")

    def run(self):
        self._golden_verified = False
        self.report.update(ok=False, a_erase_started=False)
        self.report.pop("golden_verification", None)
        try:
            initial = self._stable_connection()
            if initial["image_slot"] not in (0, 1, 2):
                raise FlashError("updater supports running golden, A or B only")
            self.report["initial_identity"] = initial
            if initial["image_slot"] == 2:
                # Restart after a prior interrupted mirror: never erase the
                # running B. It must be the exact requested, verified release.
                flash, _ = self._preflight("B", verify_image=True)
                self._event("resume_from_B")
            else:
                flash, _ = self._preflight()
                self._install(flash, "B")
                self._reboot("B")
                # Includes freshly constructed flash/boot ABI checks, JEDEC,
                # stopped-camera health, full B image and record readback.
                flash, _ = self._preflight("B", verify_image=True)
            self._install(flash, "A")
            if self.final_a:
                self._reboot("A")
                self._preflight("A", verify_image=True)
            self.report["final_identity"] = self.boot_factory(self.bus).identity()
            self.report["ok"] = True
            self._event("complete", running_slot=self.report["final_identity"]["image_slot"])
            return self.report
        except (Exception, KeyboardInterrupt) as error:
            self.report["error"] = f"{type(error).__name__}: {error}"
            self._event("failed", error=self.report["error"], a_erase_started=self.report["a_erase_started"])
            raise
        finally:
            self._close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    parser.add_argument("--bundle", required=True, type=Path, help="validated OTA bundle manifest.json")
    parser.add_argument("--csr-csv", required=True, type=Path, help="application CSR CSV for stopped-camera and fault checks")
    parser.add_argument("--reboot-timeout", default=90.0, type=float)
    parser.add_argument("--discover-mac", help="reconnect by this MAC when DHCP changes the Ethernet address")
    parser.add_argument("--broadcast", default="255.255.255.255", help="IPv4 broadcast address for DHCP rediscovery")
    parser.add_argument("--stay-in-b", action="store_true", help="verify A in flash but leave B running")
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args(argv)
    updater = None
    try:
        from .ota_bundle import load_bundle
        # Every local artifact, header, record and digest is checked before any
        # socket is opened. A malformed bundle must not erase a valid release.
        manifest, images = load_bundle(args.bundle)
        registers = load_health_map(args.csr_csv)
        connect = lambda: create_transport(args).open()
        if args.discover_mac:
            if args.transport != "etherbone":
                raise ValueError("--discover-mac requires Etherbone")
            from .discover import DHCPReconnect
            connect = DHCPReconnect(args.target, args.discover_mac, port=args.port,
                                    timeout=args.timeout, broadcast=args.broadcast)
        updater = Updater(manifest, images,
                          connect,
                          lambda bus, slot: check_health(bus, slot, registers),
                          reboot_timeout=args.reboot_timeout, final_a=not args.stay_in_b,
                          progress=lambda event: print(json.dumps(event), flush=True))
        report = updater.run()
    except (Exception, KeyboardInterrupt) as error:
        report = updater.report if updater else {"ok": False, "error": f"{type(error).__name__}: {error}",
                                                "a_erase_started": False, "golden_modified": False}
    if args.json_output:
        args.json_output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
