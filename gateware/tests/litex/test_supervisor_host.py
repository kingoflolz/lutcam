"""Firmware ownership, mailbox ordering and atomic gateware/firmware bundles."""
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib
from tools.lutcam.supervisor import BASE, MAGIC, CameraSupervisor, SupervisorError
from tools.lutcam.ota_bundle import FIRMWARE_OFFSET, load_bundle, write_bundle, validate_firmware
from tests.litex.test_ota_bundle import COMBINED, SOURCES, BUILD_IDS, IMAGES


class Bus:
    def __init__(self):
        self.words={4:19, BASE:MAGIC, BASE+4:1, BASE+8:1, BASE+0x10:0, BASE+0x20:0}
        self.writes=[]
    def read(self,address,length=None):
        if length is not None: return [self.read(address+4*i) for i in range(length)]
        return self.words.get(address,0)
    def write(self,address,values):
        if not isinstance(values,list): values=[values]
        self.writes.append((address,values))
        self.words.update({address+4*i:v for i,v in enumerate(values)})
        if address==BASE+0x10: self.words[BASE+0x20]=values[0]


class SupervisorHostTests(unittest.TestCase):

    def test_arguments_precede_sequence_commit(self):
        bus=Bus();client=CameraSupervisor.discover(bus)
        bus.words[BASE+0x2c]=192
        client.set_auto_exposure(True,-3)
        self.assertEqual(bus.writes,[(BASE+0x14,[6,1,189]),(BASE+0x10,[1])])

    def test_ev_snapshot_and_compensation_bounds(self):
        bus=Bus();client=CameraSupervisor(bus)
        for quarters in range(-8,9):
            bus.words[BASE+0x2c]=192+quarters
            bus.words[BASE+0x30]=180;bus.words[BASE+0x34]=190
            status=client.snapshot()
            self.assertEqual(status["metering"],"percentile_ev")
            self.assertEqual(status["ev_compensation"],quarters/4)
            self.assertEqual(status["p90"],[180,190])
        for value in (-9,9,1.5,True):
            with self.subTest(value=value),self.assertRaises(ValueError):
                client.set_auto_exposure(True,value)

    def test_hardware_and_firmware_failures_are_visible(self):
        for hardware,status in ((0x601,0),(5,1),(9,1),(1,4)):
            with self.subTest(hardware=hardware,status=status):
                bus=Bus();bus.words[BASE+4]=hardware;bus.words[BASE+8]=status
                with self.assertRaises(SupervisorError): CameraSupervisor(bus).wait()

    def test_firmware_is_inside_each_committed_application_and_golden_is_unchanged(self):
        code=b"\x01\x00\x01\x00"*100
        image=struct.pack("<4I",0x3256524c,len(code),zlib.crc32(code),4096)+code
        with tempfile.TemporaryDirectory() as directory:
            path=write_bundle(Path(directory)/"bundle",COMBINED,SOURCES,BUILD_IDS,{},firmware=image)
            manifest,images=load_bundle(path)
            self.assertEqual(images["golden"],IMAGES["golden"])
            for name in ("A","B"):
                self.assertEqual(images[name][FIRMWARE_OFFSET:],image)
                slot=next(s for s in manifest["slots"] if s["name"]==name)
                self.assertEqual(slot["crc32"],zlib.crc32(images[name]))
                self.assertEqual(slot["firmware"],validate_firmware(image))
            damaged=bytearray(image);damaged[-1]^=1
            with self.assertRaises(ValueError): validate_firmware(damaged)

    def test_unaligned_oversized_and_truncated_code_is_rejected(self):
        for length in (0,3,3972,0x10000):
            code=b"\0"*length
            with self.subTest(length=length),self.assertRaises(ValueError):
                validate_firmware(struct.pack("<4I",0x3256524c,length,zlib.crc32(code),4096)+code)

    def test_previous_register_file_abi_is_rejected(self):
        code=b"\0"*400
        with self.assertRaises(ValueError):
            validate_firmware(struct.pack("<4I",0x3156524c,len(code),zlib.crc32(code),4096)+code)


if __name__ == "__main__": unittest.main()
