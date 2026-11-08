"""Discovery accepts correlated identities and checks MAC before reconnecting."""
from contextlib import nullcontext
import socket
import unittest
from unittest.mock import Mock, patch

from litex.tools.remote.etherbone import EtherbonePacket, EtherboneRecord, EtherboneWrites
from tools.lutcam.discover import DHCPReconnect, discover
from tools.lutcam.flash import BOOT_BASE, BOOT_MAGIC, FlashError


def response(tag, words):
    record = EtherboneRecord(addr_size=4)
    record.writes = EtherboneWrites(addr_size=4, base_addr=tag, datas=iter(words))
    record.wcount = len(words)
    packet = EtherbonePacket(32)
    packet.records = [record]
    packet.encode()
    return packet.bytes


class DiscoveryTests(unittest.TestCase):
    def test_only_correlated_boot_identity_responses_are_returned(self):
        probe = Mock()
        probe.__enter__ = Mock(return_value=probe)
        probe.__exit__ = Mock(return_value=False)
        words = [BOOT_MAGIC, 0, 2, 0, 1, 2, 3, 4]
        good = response(123, words)
        probe.recvfrom.side_effect = [
            (b"bad", ("192.0.2.9", 1234)),
            (response(124, words), ("192.0.2.9", 1234)),
            (response(123, [0, *words[1:]]), ("192.0.2.9", 1234)),
            (response(123, words[:-1]), ("192.0.2.9", 1234)),
            (good, ("192.0.2.9", 1235)),
            (good, ("0.0.0.0", 1234)),
            (good, ("192.0.2.50", 1234)),
            (good, ("192.0.2.50", 1234)),
            socket.timeout(),
        ]
        with patch("tools.lutcam.discover.socket.socket", return_value=probe), \
             patch("tools.lutcam.discover.secrets.randbits", return_value=123):
            found = discover(bind="192.0.2.1")
        probe.bind.assert_called_once_with(("192.0.2.1", 0))
        self.assertEqual(found, [dict(address="192.0.2.50", image_slot=2, status=0,
                                     build_id="00000001000000020000000300000004")])
        raw, destination = probe.sendto.call_args.args
        sent = EtherbonePacket(32, raw)
        sent.decode()
        self.assertEqual(destination, ("255.255.255.255", 1234))
        self.assertIsNone(sent.records[0].writes)
        self.assertEqual(sent.records[0].reads.get_addrs(), [BOOT_BASE + 4*i for i in range(8)])

    def test_connection_pins_mac_and_closes_rejected_socket(self):
        reconnect = DHCPReconnect("192.0.2.50", "02:4c:55:54:00:01")
        for mac, valid in ((0x024c55540001, True), (0x024c55540002, False)):
            with self.subTest(mac=mac):
                bus = Mock()
                bus.open.return_value = bus
                bus.discovery_probe.return_value = nullcontext()
                packed = mac.to_bytes(6, "big") + b"\0\0"
                bus.read.side_effect = [32, [int.from_bytes(packed[i:i+4], "little") for i in (0, 4)]]
                with patch("tools.lutcam.discover.EtherboneUDP", return_value=bus), \
                     patch("tools.lutcam.discover.BootClient"):
                    if valid:
                        self.assertIs(reconnect._open("192.0.2.51"), bus)
                        bus.close.assert_not_called()
                    else:
                        with self.assertRaises(FlashError):
                            reconnect._open("192.0.2.51")
                        bus.close.assert_called_once()
                bus.write.assert_not_called()

    def test_changed_address_is_reused_on_next_connection(self):
        reconnect = DHCPReconnect("192.0.2.50", "02:4c:55:54:00:01")
        bus = Mock()
        reconnect._open = Mock(side_effect=[socket.timeout(), bus, bus])
        with patch("tools.lutcam.discover.discover", return_value=[{"address": "192.0.2.51"}]) as probe:
            self.assertIs(reconnect(), bus)
            self.assertIs(reconnect(), bus)
        self.assertEqual(reconnect.target, "192.0.2.51")
        self.assertEqual([call.args[0] for call in reconnect._open.call_args_list],
                         ["192.0.2.50", "192.0.2.51", "192.0.2.51"])
        probe.assert_called_once()


if __name__ == "__main__":
    unittest.main()
