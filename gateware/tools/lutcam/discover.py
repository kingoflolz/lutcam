"""Read-only Etherbone discovery for reconnecting after a DHCP address change."""

import argparse
import ipaddress
import json
import secrets
import socket
import time

from .flash import BOOT_BASE, BOOT_MAGIC, BootClient, EtherboneUDP, FlashError


def discover(*, broadcast="255.255.255.255", bind="0.0.0.0", port=1234, timeout=0.25):
    """Broadcast only boot-identity reads; correlate responses on a fresh socket."""
    from litex.tools.remote.etherbone import EtherbonePacket, EtherboneRecord, EtherboneReads

    sequence = secrets.randbits(32)
    record = EtherboneRecord(addr_size=4)
    record.reads = EtherboneReads(addr_size=4, addrs=[BOOT_BASE + 4*i for i in range(8)])
    record.reads.base_ret_addr = sequence
    record.rcount = 8
    packet = EtherbonePacket(32)
    packet.records = [record]
    packet.encode()
    found = {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        probe.bind((str(ipaddress.IPv4Address(bind)), 0))
        probe.sendto(packet.bytes, (str(ipaddress.IPv4Address(broadcast)), port))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            probe.settimeout(max(0.001, deadline - time.monotonic()))
            try:
                raw, peer = probe.recvfrom(8192)
            except socket.timeout:
                break
            if peer[1] != port or ipaddress.IPv4Address(peer[0]).is_unspecified:
                continue
            try:
                reply = EtherbonePacket(32, raw)
                reply.decode()
                if len(reply.records) != 1:
                    continue
                words = reply.records[0].writes
                if words is None or words.base_addr != sequence:
                    continue
                values = words.get_datas()
                if len(values) != 8 or values[0] != BOOT_MAGIC or values[2] not in (0, 1, 2):
                    continue
            except (ValueError, AssertionError, IndexError, TypeError, AttributeError):
                continue
            found[peer[0]] = dict(address=peer[0], image_slot=values[2], status=values[3],
                                 build_id="".join(f"{v:08x}" for v in values[4:]))
    return list(found.values())


class DHCPReconnect:
    """Pin unicast connections to the requested MAC after discovery.

    Older applications and golden lack the MAC aperture and remain reachable
    at the explicitly supplied initial address. No mutation uses broadcast.
    """

    def __init__(self, target, mac, *, port=1234, timeout=0.2,
                 broadcast="255.255.255.255"):
        self.initial_target = self.target = str(ipaddress.IPv4Address(target))
        self.mac = int(mac.replace(":", ""), 16)
        if not 0 < self.mac < 1 << 48:
            raise ValueError("expected a nonzero 48-bit MAC address")
        self.port, self.timeout, self.broadcast = port, timeout, broadcast

    def _open(self, target):
        bus = EtherboneUDP(target, self.port, timeout=self.timeout).open()
        try:
            with bus.discovery_probe():
                BootClient(bus).identity()
                if bus.read(4) & 32:
                    words = bus.read(0x30029034, length=2)
                    mac = int.from_bytes(b"".join(w.to_bytes(4, "little") for w in words)[:6], "big")
                    if mac != self.mac:
                        raise FlashError("discovered board has a different MAC address")
                elif target != self.initial_target:
                    raise FlashError("discovered board lacks a MAC identity aperture")
            return bus
        except Exception:
            bus.close()
            raise

    def __call__(self):
        try:
            return self._open(self.target)
        except (OSError, FlashError):
            pass
        for candidate in discover(broadcast=self.broadcast, port=self.port):
            try:
                bus = self._open(candidate["address"])
            except (OSError, FlashError):
                continue
            self.target = candidate["address"]
            return bus
        raise socket.timeout("no board with the requested MAC has acquired an address yet")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broadcast", default="255.255.255.255")
    parser.add_argument("--bind", default="0.0.0.0", help="local IPv4 address for discovery")
    parser.add_argument("--port", type=int, default=1234)
    parser.add_argument("--timeout", type=float, default=0.5)
    print(json.dumps(discover(**vars(parser.parse_args())), indent=2))
