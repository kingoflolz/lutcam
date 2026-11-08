"""Run the actual small DHCP client against independent BOOTP/TLV packets."""
import ctypes
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
IP, SERVER, MASK = 0xc0a80142, 0xc0a801fe, 0xffffff00
MAC = bytes.fromhex("024c55540001")


def option(code, value, size=4):
    return bytes((code, size)) + value.to_bytes(size, "big")


class DhcpFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="lutcam-dhcp-")
        cls.addClassCleanup(cls.temp.cleanup)
        d = Path(cls.temp.name)
        (d / "test.c").write_text(r'''
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#define CLOCK_HZ 120000000u
static uint32_t mmio_read(uint32_t);
static void mmio_write(uint32_t,uint32_t);
#define RD(o) mmio_read(0x30028000u+(o))
static union {
    uint32_t packet[67];
    struct { uint32_t options[8],word,index,input; } reader;
} work;
#include "network.h"
static uint32_t registers[32], ticks, busy, rx_length, rx_position;
static uint8_t rx[1476];
uint8_t sim_tx[268];
uint32_t sim_tx_count, sim_revoked, sim_bad_read;
static uint32_t mmio_read(uint32_t a) {
    if (a==0x30028040) return ticks;
    assert(a>=NET && a<NET+128);
    uint32_t offset=a-NET;
    if (offset==4) return (!!rx_length && registers[0x50/4]) | (busy<<1);
    if (offset==0x14) return rx_length;
    if (offset==0x1c) {
        uint32_t v=0;
        if (rx_position>=rx_length) { ++sim_bad_read; return 0; }
        memcpy(&v,rx+rx_position,4); rx_position+=4; return v;
    }
    return registers[offset/4];
}
static void mmio_write(uint32_t a,uint32_t v) {
    assert(a>=NET && a<NET+128);
    uint32_t offset=a-NET;
    if (offset==0x4c) { ++sim_revoked; return; }
    if (offset==0x20) { rx_length=rx_position=0; return; }
    registers[offset/4]=v;
    if (offset==0x30) {
        assert(!busy && registers[0x28/4]==sizeof(sim_tx));
        memcpy(sim_tx,work.packet,sizeof(sim_tx)); ++sim_tx_count; busy=1;
    }
}
void sim_reset(void) {
    memset(&work,0,sizeof(work)); memset(registers,0,sizeof(registers));
    memset(rx,0,sizeof(rx)); memset(sim_tx,0,sizeof(sim_tx));
    ticks=busy=rx_length=rx_position=sim_tx_count=sim_revoked=sim_bad_read=0;
    net_state=net_xid=net_clock=net_seconds=net_start=net_due=0;
    net_ip=net_server=net_lease=net_t1=net_epoch=0;
    registers[0x34/4]=0x54554c02; registers[0x38/4]=0x100;
    registers[0x3c/4]=0xc0a80132;
}
void sim_poll(void) { network_poll(); }
void sim_second(void) { ticks+=CLOCK_HZ; network_poll(); }
void sim_tx_done(void) { busy=0; }
void sim_feed(const uint8_t *p,uint32_t n) {
    assert(!rx_length && n>=244 && n<=1472);
    memset(rx,0,sizeof(rx)); memcpy(rx,p,n); rx_length=n; rx_position=0;
}
uint32_t sim_register(uint32_t offset) { return registers[offset/4]; }
uint32_t sim_pending(void) { return rx_length; }
''')
        subprocess.run(["clang", "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-I", str(ROOT / "firmware"), str(d / "test.c"), "-o", str(d / "test.so")], check=True)
        cls.lib = ctypes.CDLL(str(d / "test.so"))
        cls.lib.sim_feed.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32]
        cls.lib.sim_register.argtypes = [ctypes.c_uint32]
        cls.lib.sim_register.restype = ctypes.c_uint32

    def setUp(self):
        self.lib.sim_reset()
        self.lib.sim_poll()

    def tearDown(self):
        self.assertEqual(self.counter("sim_bad_read"), 0)

    def counter(self, name):
        return ctypes.c_uint32.in_dll(self.lib, name).value

    def tx(self):
        return bytes((ctypes.c_uint8 * 268).in_dll(self.lib, "sim_tx"))

    def options(self, packet):
        result, i = {}, 240
        while i < len(packet):
            code = packet[i]; i += 1
            if code == 255:
                return result
            if code == 0:
                continue
            n = packet[i]; i += 1
            self.assertLessEqual(i+n, len(packet))
            result[code] = packet[i:i+n]; i += n
        self.fail("missing DHCP END")

    def reply(self, kind, *, ip=IP, server=SERVER, extra=b"", fields=None):
        p = bytearray(240)
        p[:3] = b"\x02\x01\x06"
        p[4:8] = self.tx()[4:8]
        p[16:20] = ip.to_bytes(4, "big")
        p[28:34] = MAC
        p[236:240] = bytes.fromhex("63825363")
        if fields:
            for offset, value in fields.items():
                p[offset:offset+len(value)] = value
        p += option(53, kind, 1) + option(54, server) + extra + b"\xff"
        return p

    def feed(self, packet):
        self.lib.sim_feed((ctypes.c_uint8 * len(packet)).from_buffer_copy(packet), len(packet))
        self.lib.sim_poll()

    def advance(self, seconds):
        for _ in range(seconds):
            self.lib.sim_tx_done()
            self.lib.sim_second()

    def bind(self, lease=120, t1=None):
        self.lib.sim_tx_done()
        self.feed(self.reply(2))
        self.lib.sim_tx_done()
        extra = option(51, lease) + option(1, MASK) + option(3, SERVER)
        if t1 is not None:
            extra += option(58, t1)
        self.feed(self.reply(5, extra=extra))
        self.assertEqual(self.lib.sim_register(8), IP)
        self.assertEqual(self.lib.sim_register(0x40), 3)

    def test_discover_request_ack_and_route(self):
        p = self.tx()
        self.assertEqual(p[:3], b"\x01\x01\x06")
        self.assertEqual(p[28:34], MAC)
        self.assertEqual(struct.unpack_from("!H", p, 10)[0], 0x8000)
        self.assertEqual(self.options(p)[53], b"\x01")
        self.lib.sim_tx_done()
        self.feed(self.reply(2))
        options = self.options(self.tx())
        self.assertEqual(options[53], b"\x03")
        self.assertEqual(options[50], IP.to_bytes(4, "big"))
        self.assertEqual(options[54], SERVER.to_bytes(4, "big"))
        self.lib.sim_tx_done()
        self.feed(self.reply(5, extra=option(51, 120)+option(1, MASK)+option(3, SERVER)))
        self.assertEqual([self.lib.sim_register(o) for o in (8,12,16)], [IP,MASK,SERVER])

    def test_renewal_keeps_capture_and_uses_new_transaction(self):
        self.bind(120, t1=30)
        xid, revoked = self.tx()[4:8], self.counter("sim_revoked")
        self.advance(30)
        p = self.tx()
        self.assertNotEqual(p[4:8], xid)
        self.assertEqual(p[12:16], IP.to_bytes(4, "big"))
        self.assertEqual(p[10:12], b"\0\0")
        self.assertEqual(self.lib.sim_register(0x2c), SERVER)
        self.assertFalse({50,54} & self.options(p).keys())
        self.lib.sim_tx_done()
        self.feed(self.reply(5, extra=option(51, 240)))
        self.assertEqual(self.lib.sim_register(0x40), 3)
        self.assertEqual(self.counter("sim_revoked"), revoked)
        self.assertEqual(self.lib.sim_register(0x44), 240)

    def test_failed_renewal_restarts_discovery_and_revokes_capture(self):
        self.bind(120)
        self.advance(60)
        self.assertEqual(self.lib.sim_register(0x40), 4)
        self.advance(16)
        self.assertEqual(self.lib.sim_register(8), 0)
        self.assertEqual(self.lib.sim_register(0x40), 1)
        self.assertEqual(self.options(self.tx())[53], b"\x01")

    def test_short_lease_expires_before_transaction_timeout(self):
        self.bind(8)
        self.advance(8)
        self.assertEqual(self.lib.sim_register(8), 0)
        self.assertEqual(self.lib.sim_register(0x40), 1)

    def test_one_second_lease_does_not_renew_in_a_tight_loop(self):
        self.bind(1)
        sent = self.counter("sim_tx_count")
        for _ in range(10):
            self.lib.sim_poll()
        self.assertEqual(self.counter("sim_tx_count"), sent)
        self.advance(1)
        self.assertEqual(self.lib.sim_register(8), 0)

    def test_already_expired_ack_cannot_install_an_address(self):
        self.lib.sim_tx_done(); self.feed(self.reply(2)); self.lib.sim_tx_done()
        self.advance(2)
        self.feed(self.reply(5, extra=option(51,1)+option(1,MASK)))
        self.assertEqual(self.lib.sim_register(8), 0)

    def test_nak_and_unanswered_discovery_restart(self):
        self.bind()
        self.advance(60)
        self.lib.sim_tx_done()
        self.feed(self.reply(6, ip=0))
        self.assertEqual(self.lib.sim_register(8), 0)
        xid = self.tx()[4:8]
        self.advance(16)
        self.assertNotEqual(self.tx()[4:8], xid)
        self.assertEqual(self.lib.sim_register(0x40), 1)

    def test_infinite_lease_and_timer_wrap(self):
        self.bind(0xffffffff)
        sent = self.counter("sim_tx_count")
        self.advance(3600)
        self.assertEqual(self.counter("sim_tx_count"), sent)
        self.assertEqual(self.lib.sim_register(8), IP)

    def test_rx_waits_for_tx_dma_buffer(self):
        original = self.tx()
        self.feed(self.reply(2))
        self.assertNotEqual(self.lib.sim_pending(), 0)
        self.assertEqual(self.tx(), original)
        self.assertEqual(self.counter("sim_tx_count"), 1)
        self.lib.sim_tx_done(); self.lib.sim_poll()
        self.assertEqual(self.counter("sim_tx_count"), 2)

    def test_rejects_wrong_identity_and_truncated_options(self):
        for offset in (0,1,2,4,28,32,236):
            with self.subTest(offset=offset):
                self.lib.sim_reset(); self.lib.sim_poll(); self.lib.sim_tx_done()
                p = self.reply(2); p[offset] ^= 0x40
                self.feed(p)
                self.assertEqual(self.lib.sim_register(0x40), 1)
        self.lib.sim_reset(); self.lib.sim_poll(); self.lib.sim_tx_done()
        p = self.reply(2)[:-1] + bytes((1, 20, 1, 2))
        self.feed(p)
        self.assertEqual(self.lib.sim_register(0x40), 1)

    def test_overloaded_options_and_unknown_padding(self):
        self.lib.sim_tx_done(); self.feed(self.reply(2)); self.lib.sim_tx_done()
        file = option(51,120) + b"\xff"
        sname = option(1,MASK) + option(3,SERVER) + b"\xff"
        self.feed(self.reply(5, extra=b"\0"+option(52,3,1)+bytes((200,0)),
                            fields={108:file,44:sname}))
        self.assertEqual([self.lib.sim_register(o) for o in (8,12,16)], [IP,MASK,SERVER])

    def test_large_options_stream_and_invalid_renewal_server(self):
        self.lib.sim_tx_done()
        padding = bytes((200,255)) + bytes(255)
        self.feed(self.reply(2, extra=padding*4))
        self.assertEqual(self.lib.sim_register(0x40), 2)
        self.lib.sim_tx_done()
        self.feed(self.reply(5, server=SERVER-1, extra=option(51,120)+option(1,MASK)))
        self.assertEqual(self.lib.sim_register(8), 0)


if __name__ == "__main__":
    unittest.main()
