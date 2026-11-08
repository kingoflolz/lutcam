"""Dual-camera LUTCAM with remote hosts and a flash-loaded camera supervisor."""

from pathlib import Path

from migen import Cat, ClockSignal, Constant, If, ResetInserter, Signal
from migen.genlib.cdc import MultiReg
from litex.gen import LiteXModule
from litex_luna import LunaCDCACM
from litex.soc.integration.soc import SoCCSRRegion, SoCRegion
from litex.soc.integration.export import MockCSR
from litex.soc.integration.soc_core import SoCCore
from litex.soc.interconnect.csr import CSR, CSRStatus, CSRStorage
from liteeth.core import LiteEthUDPIPCore
from liteeth.common import convert_ip
from liteeth.frontend.etherbone import LiteEthEtherbone

from .camera import Camera, add_camera_sources, shared_detector_configuration
from .calibration import CalibrationBank
from .shared_lftag import SharedLfTag
from .control import WishboneGuard
from .readback import SharedReadback
from .i2c import SharedCameraI2C
from .crg import LutcamCRG, LutcamRGMII
from .record_bus import EtherboneRecordMetadata, RecordUARTBone
from .identifier import use_logic_identifier
from .packet import SharedPacketTransport
from .pair import PairedCaptureControl
from .counters import SharedPacketCounters
from .registers import CONTROL_BASE, IntegrationRegisters
from .platform import Platform, request_camera_hsio
from .boot import BOOT_BASE, BootControl, FlashAccessMux, request_remote_update
from .flash import FLASH_BASE
from .flash_packet import PacketFlashController
from .usb import USBConnectDelay
from .usb_diagnostics import USB_LINK_BASE, USBLinkDiagnostics
from .golden import GoldenBootScanner
from .supervisor import CameraSupervisor, SUPERVISOR_BASE, RAM_BYTES
from .network import NetworkControl, NETWORK_BASE
from .boot_reader import FirmwareLoader


class HostDiagnostics(LiteXModule):
    def get_csrs(self, *args, **kwargs):
        return [] if self._direct_control else super().get_csrs(*args, **kwargs)

    def __init__(self, guard, *, direct_control=False):
        self._direct_control = direct_control
        self.external_count = guard.external_count
        self.fault_event = guard.fault_event
        self.status = CSRStatus(4, name="status")
        self.address = CSRStatus(32, name="address")
        self.count = CSRStatus(32, name="count")
        self.clear = CSR(name="clear")
        self.comb += [self.status.status.eq(Cat(guard.fault, guard.fault_status)),
                      self.address.status.eq(guard.fault_address),
                      self.count.status.eq(guard.fault_count),
                      guard.clear_fault.eq(self.clear.re)]


class BoardControl(LiteXModule):
    def __init__(self):
        self.revoke_capture = Signal()
        self.network_ready = ResetInserter()(CSRStorage(1, name="network_ready",
            description="Software asserts after verifying 1Gb link and destination ARP reachability."))
        self.comb += self.network_ready.reset.eq(self.revoke_capture)
        self.status = CSRStatus(6, name="status")
        self.tx_unreachable = CSRStatus(1, name="tx_unreachable")
        self.clear = CSR(name="clear")


class LutcamSoC(SoCCore):
    def __init__(self, *, ip_address="192.168.1.50", mac_address=None,
                 with_usb=True, platform=None, timing_model="C3", golden=False,
                 build_id="0" * 32, golden_maintenance=False):
        if golden and golden_maintenance:
            raise ValueError("golden maintenance must execute as an application image")
        if timing_model != "C3":
            raise ValueError("Lutcam builds require C3 timing")
        if platform is None:
            platform = Platform()
        elif platform.timing_model != "C3":
            raise ValueError("Lutcam builds require a platform with C3 timing")
        root = Path(__file__).resolve().parents[3]
        self.golden = golden
        self.crg = LutcamCRG(platform)
        SoCCore.__init__(self, platform, clk_freq=120_000_000,
            cpu_type=None, integrated_rom_size=0, integrated_sram_size=0,
            with_uart=False, with_timer=False, with_ctrl=False,
            csr_data_width=32, ident="LUTCAM dual RAW10 ISP camera", ident_version=False)
        # All three main-image masters use the same bounded request lifetime.
        # Golden and USB-disabled builds retain independent peripheral responses.
        self.shared_readback = SharedReadback() if not golden and with_usb else None
        use_logic_identifier(self.identifier)
        self.board = BoardControl()
        self.csr.add("board")
        self.ethphy = LutcamRGMII(platform, platform.request("eth_clocks"),
                                  platform.request("eth"), self.crg)
        self.csr.add("ethphy")
        # UDP/IP uses 16 bits at 120 MHz, with standard converters retaining
        # 32-bit application/Etherbone interfaces. MAC framing stays at the
        # PHY's native 125 MHz. The narrower 64-word CDCs save two RAM blocks
        # while supplying Gigabit line rate without another clock crossing.
        if golden:
            self.identity_reader = boot_reader = FirmwareLoader(with_firmware=False)
            self.comb += boot_reader.identity_ready.eq(1)
        else:
            self.supervisor = supervisor = CameraSupervisor(platform, shared_readback=self.shared_readback)
            boot_reader = supervisor.loader
        network_mac = boot_reader.mac_address if mac_address is None else mac_address
        network_ip = ip_address if golden else Signal(32)
        network_mask = None if golden else Signal(32)
        network_gateway = None if golden else Signal(32)
        self.ethcore = LiteEthUDPIPCore(self.ethphy, network_mac, network_ip,
            self.clk_freq, dw=16, with_sys_datapath=False,
            with_icmp=False, arp_entries=2, tx_cdc_depth=64, rx_cdc_depth=64,
            netmask=network_mask, gateway_ip=network_gateway)
        # Preserve the pre-resolved capture destination. LiteEth otherwise
        # expires its whole ARP cache each second, introducing an unbounded
        # resolution pause even on a healthy link. Disarming restores expiry.
        self.comb += self.ethcore.arp.table.cache.clear_enable.eq(
            ~self.board.network_ready.storage)
        # The vendored MAC validates a full frame before Etherbone executes it.
        # Leave room for a base address and 128 payload/address words.
        self.etherbone = LiteEthEtherbone(self.ethcore.udp, 1234, buffer_depth=256)
        self.etherbone_record = EtherboneRecordMetadata(self.etherbone.wishbone,
                                                        source_id=0)
        self.etherbone_guard = WishboneGuard(external_count=True)
        self._host_guards = {"etherbone": self.etherbone_guard}
        self.comb += self.etherbone.wishbone.bus.connect(self.etherbone_guard.bus)
        self.bus.add_master(name="etherbone", master=self.etherbone_guard.master)
        self.etherbone_diagnostics = HostDiagnostics(self.etherbone_guard, direct_control=True)
        diagnostics = [("etherbone_diagnostics", self.etherbone_diagnostics)]

        usb_vbus = Signal()
        usb_reboot_pending = Signal()
        if with_usb:
            usb_pads = platform.request("usb")
            self.usb = LunaCDCACM(platform, usb_pads)
            self.usb_link = USBLinkDiagnostics.from_luna(self.usb,
                pll_locked=self.crg.sys_pll.locked, vbus_valid=usb_pads.vbus_valid,
                use_dsp_counters=not golden, shared_readback=self.shared_readback)
            self.bus.add_slave("usb_link", self.usb_link.bus,
                SoCRegion(origin=USB_LINK_BASE, size=0x1000, cached=False))
            self.usb_connect_delay = USBConnectDelay(
                usb_pads.vbus_valid & self.crg.sys_pll.locked & ~usb_reboot_pending)
            self.comb += self.usb.connect.eq(self.usb_connect_delay.connect)
            self.usb_host = RecordUARTBone(phy=self.usb, clk_freq=self.clk_freq,
                source_id=1, compact_datapath=not golden, use_dsp_timeout=not golden)
            self.usb_guard = WishboneGuard(external_count=True)
            self._host_guards["usb"] = self.usb_guard
            self.comb += self.usb_host.wishbone.connect(self.usb_guard.bus)
            self.bus.add_master(name="usb", master=self.usb_guard.master)
            self.usb_diagnostics = HostDiagnostics(self.usb_guard, direct_control=True)
            diagnostics.append(("usb_diagnostics", self.usb_diagnostics))
            self.specials += MultiReg(usb_pads.vbus_valid, usb_vbus)

        self.sync += [
            If(self.board.clear.re, self.board.tx_unreachable.status.eq(0)),
            # A DHCP server failing to answer a renewal ARP must not revoke
            # an otherwise valid lease or fault an ongoing camera stream.
            If(self.ethcore.ip.tx.target_unreachable & (self.ethcore.udp.tx.sink.src_port != 68),
                self.board.tx_unreachable.status.eq(1)),
        ]
        self.comb += self.board.status.status.eq(Cat(self.crg.locked,
            ~self.ethphy.reset, usb_vbus, self.board.network_ready.storage,
            Constant(not golden, 1), Constant(not golden, 1)))
        if golden:
            identity = None
            image_slot = 0
        else:
            from .image_identity import ImageIdentity
            self.image_identity = identity = ImageIdentity(
                platform, build_id=build_id, sequential_build_id=True)
            image_slot = identity.image_slot
        self.flash = PacketFlashController(platform.request("spiflash"), boot_slot=image_slot,
                                           use_dsp_poll=not golden, use_dsp_storage=False,
                                           share_spi_address=not golden,
                                           golden_maintenance=golden_maintenance, boot_client=boot_reader.spi)
        platform.spi_flash_half_period = self.flash.spi_half_period
        self.boot = BootControl(request_remote_update(platform, self.crg.cd_cam_ref.clk),
                                build_id=build_id, identity=identity,
                                use_dsp_delay=not golden, shared_readback=self.shared_readback)
        # Disconnect while the USB clocks and protocol logic still run. The
        # boot controller then waits 100 ms before starting reconfiguration;
        # the new image also observes its complete startup disconnect delay.
        self.comb += usb_reboot_pending.eq(self.boot.pending)
        self.comb += [self.flash.inhibit.eq(boot_reader.busy | self.boot.pending |
                                            (0 if golden else ~identity.valid))]
        self.bus.add_slave("boot_control", self.boot.bus,
                           SoCRegion(origin=BOOT_BASE, size=0x1000, cached=False))
        if golden:
            self.golden_scanner = scanner = GoldenBootScanner(startup_cycles=12_000_000)
            self.flash_access = FlashAccessMux(self.flash.bus, scanner)
            self.comb += [
                self.boot.flash_busy.eq(self.flash.busy | (self.flash.error != 0) | scanner.busy | boot_reader.busy),
                scanner.ru_error.eq(self.boot.ru_error),
                self.boot.auto_request.eq(scanner.request_valid),
                self.boot.auto_slot.eq(scanner.request_slot),
                scanner.request_ready.eq(self.boot.auto_ready),
                self.boot.golden_status.eq(Cat(scanner.busy, scanner.done,
                    scanner.valid_a, scanner.valid_b, Constant(0, 4), scanner.error)),
            ]
            flash_bus = self.flash_access.bus
        else:
            self.supervisor_guard = WishboneGuard()
            self._host_guards["supervisor"] = self.supervisor_guard
            self.comb += [supervisor.master.connect(self.supervisor_guard.bus),
                          supervisor.granted.eq(self.supervisor_guard.granted),
                          supervisor.loader.slot.eq(identity.image_slot),
                          supervisor.loader.identity_ready.eq(identity.ready)]
            self.bus.add_master(name="supervisor", master=self.supervisor_guard.master)
            self.bus.add_slave("supervisor", supervisor.bus,
                SoCRegion(origin=SUPERVISOR_BASE, size=0x1000, cached=False))
            dhcp_port = self.ethcore.udp.crossbar.get_port(68, dw=32, cd="sys")
            self.network = network = NetworkControl(dhcp_port, supervisor.packet_dma,
                ram_bytes=RAM_BYTES, mac_address=network_mac, preferred_ip=convert_ip(ip_address),
                shared_readback=self.shared_readback)
            self.comb += [network.granted.eq(supervisor.granted),
                          self.board.revoke_capture.eq(network.revoke_capture),
                          network.inhibit.eq(~supervisor.loader.done | supervisor.trap | supervisor.bus_fault),
                          network_ip.eq(network.ip_address), network_mask.eq(network.netmask),
                          network_gateway.eq(network.gateway)]
            self.bus.add_slave("network", network.bus,
                SoCRegion(origin=NETWORK_BASE, size=0x1000, cached=False))
            flash_bus = self.flash.bus
            self.comb += self.boot.flash_busy.eq(self.flash.busy | (self.flash.error != 0)
                                                | supervisor.loader.busy)
        # A record's boundaries and owner survive the guards' per-word CYC
        # gaps. Select metadata with the same grant as its Wishbone signals.
        record_hosts = [("etherbone", self.etherbone_record)]
        if with_usb:
            record_hosts.append(("usb", self.usb_host))
        if not golden:
            record_hosts.append(("supervisor", supervisor))
        record_fields = ("record_first", "record_last", "record_count", "record_source")
        for name, record in record_hosts:
            guard = self._host_guards[name]
            self.comb += [record.guard_fault.eq(guard.fault_event),
                If(guard.granted, *[
                    getattr(self.flash, field).eq(getattr(record, field))
                    for field in record_fields])]
        self.comb += self.flash.record_abort_sources.eq(Cat(
            self.etherbone_record.record_abort,
            self.usb_host.record_abort if with_usb else Constant(0, 1),
            scanner.record_abort if golden else Constant(0, 1),
            Constant(0, 1) if golden else supervisor.record_abort))
        if golden:
            self.comb += If(scanner.busy, *[
                getattr(self.flash, field).eq(getattr(scanner, field))
                for field in record_fields])
        self.bus.add_slave("flash_control", flash_bus,
                           SoCRegion(origin=FLASH_BASE, size=0x1000, cached=False))
        if golden:
            # Recovery contains clocks, Ethernet/USB hosts, flash, CRC and
            # reboot, without the camera pipelines or packet transport.
            for name, diagnostic in diagnostics:
                diagnostic._direct_control = False
                self.csr.add(name)
            return
        add_camera_sources(platform, root)
        # CameraPipeline holds each ID through image/padding and solver output.
        # Paired CLEAR/RESET changes IDs and resets unfinished parsers together;
        # committed packet metadata remains independently stored in the queues.
        self.packet_transport = SharedPacketTransport(child_slots=8, external_destination=True,
            external_context_held=True, compact_padding=True, batched_packets=True,
            accepted_centroids=True, shared_solver_ingress=True, use_dsp_metadata=False, held_frame_id=True)
        self.packet_counters = SharedPacketCounters(packet_totals=True, with_destinations=True,
            fixed_packet_layout=True, with_host_counters=True, compact_image_packets=True,
            batched_packets=True, serialized_datagrams=True, serialized_event_descriptor=True)
        self.capture_pair = pair = PairedCaptureControl(use_dsp_counters=True)
        self.comb += supervisor.timer.eq(pair.now[:32])
        camera_pads = [platform.request("camera", index) for index in range(2)]
        self.camera_i2c = SharedCameraI2C(camera_pads, clk_freq=self.clk_freq,
                                          single_aperture=True, compact_record=True,
                                          use_dsp_state=False, use_dsp_watchdog=True)
        self.bus.add_slave("camera_i2c", self.camera_i2c.bus,
            SoCRegion(origin=0x30020000, size=0x2000, cached=False))
        for name, record in record_hosts:
            guard = self._host_guards[name]
            self.comb += If(guard.granted, *[
                getattr(self.camera_i2c, field).eq(getattr(record, field))
                for field in record_fields])
        self.comb += self.camera_i2c.record_abort_sources.eq(self.flash.record_abort_sources)
        self.cameras = []
        detector_configuration = shared_detector_configuration()
        for index in range(2):
            pads = camera_pads[index]
            hsio = request_camera_hsio(platform, index)
            platform.add_false_path_constraint(hsio.clock.clk, self.crg.cd_sys.clk)
            platform.add_false_path_constraint(self.crg.cd_sys.clk, hsio.clock.clk)
            camera = Camera(platform, hsio, pads, root, direct_control=True,
                            paired=True, shared_lftag=True, accepted_centroids=True,
                            detector_configuration=detector_configuration,
                            shared_readback=self.shared_readback)
            setattr(self.submodules, f"camera{index}", camera)
            self.cameras.append(camera)
            self.bus.add_slave(f"camera{index}_isp", camera.isp_bus,
                SoCRegion(origin=0x30000000 + index * 0x10000, size=0x10000, cached=False))
            self.bus.add_slave(f"camera{index}_histogram", camera.histogram.bus,
                SoCRegion(origin=0x30022000 + index * 0x1000, size=0x1000, cached=False))

            camera_packets = self.packet_transport.channels[index * 3:index * 3 + 3]
            for product, packet in zip(("image", "children", "pose"), camera_packets):
                self.comb += getattr(camera, product).connect(packet.sink)
            packet_fault = camera_packets[0].fault_sticky | camera_packets[1].fault_sticky | camera_packets[2].fault_sticky
            self.comb += [
                camera.capture_permit.eq(self.crg.locked & ~self.ethphy.reset
                    & self.board.network_ready.storage & ~self.board.tx_unreachable.status),
                camera.external_fault.eq(packet_fault),
                camera.run.storage.eq(pair.busy),
                camera.admission_request.eq(pair.grants[index]),
                camera.pending_frame_id.eq(pair.pending_frame_id[index]),
                camera.pair_abort.eq(pair.abort),
                pair.sof[index].eq(camera.frame_sof),
                pair.admitted[index].eq(camera.frame_admitted),
                pair.retired[index].eq(camera.frame_retired),
                pair.active[index].eq(camera.active),
                pair.pipeline_ready[index].eq(camera.admission_ready),
                pair.local_fault[index].eq(camera.local_fault),
                pair.receiver_fault[index].eq((camera.receiver_status.status & 0x3ffe) != 0),
                pair.packet_fault[index].eq(packet_fault),
            ]
        self.calibration_bank = CalibrationBank()
        self.shared_lftag = SharedLfTag(self.cameras, accepted_centroids=True,
                                      configuration_bank=self.calibration_bank)
        self.comb += [
            # Both sensors are FSIN slaves; the FPGA owns the two phased
            # outputs from reset, before any sensor software configuration.
            self.cameras[0].fsin_output.eq(pair.sync_outputs[0]),
            self.cameras[1].fsin_output.eq(pair.sync_outputs[1]),
            pair.health_good.eq(self.crg.locked & ~self.ethphy.reset
                & self.board.network_ready.storage & ~self.board.tx_unreachable.status),
            pair.transport_drained.eq(self.packet_transport.drained),
        ]
        self.camera_controls = IntegrationRegisters(self.cameras, self.packet_transport,
            self.packet_counters, banked_destinations=True, diagnostics=diagnostics, pair=pair,
            configuration_bank=self.calibration_bank, shared_readback=self.shared_readback,
            held_request=self.shared_readback is not None)
        self.bus.add_slave("camera_controls", self.camera_controls.bus,
            SoCRegion(origin=CONTROL_BASE, size=0x1000, cached=False))
        data_port = self.ethcore.udp.crossbar.get_port(6000, dw=32, cd="sys")
        self.comb += [self.packet_transport.source.connect(data_port.sink),
                      data_port.source.ready.eq(1)]

    def _finalize_bus(self):
        super()._finalize_bus()
        if self.shared_readback is not None:
            self.shared_readback.validate_topology(
                self.csr_bridge, self.bus._interconnect, self._host_guards.values(),
                decoder_registered=self.bus.interconnect_register,
                interconnect_timeout_cycles=self.bus.timeout)

    def do_finalize(self):
        super().do_finalize()
        # Export ordinary LiteX names for the compact Wishbone implementation.
        # These are metadata only: no duplicated CSR storage or readback banks.
        regions = dict(self.boot.regions)
        if hasattr(self, "usb_link"):
            regions.update(self.usb_link.regions)
        flash_registers = (
            ("magic", 0x00, "ro"), ("reserved0", 0x04, "ro"),
            ("status", 0x08, "ro"), ("size", 0x0c, "ro"),
            ("address", 0x10, "ro"), ("length", 0x14, "ro"),
            ("opcode", 0x18, "ro"), ("command_id", 0x1c, "ro"),
            ("done_id", 0x20, "ro"), ("error", 0x24, "ro"),
            ("result", 0x28, "ro"), ("image_slot", 0x2c, "ro"),
            ("spi_hz", 0x30, "ro"), ("command_crc32", 0x34, "ro"),
            ("buffer_size", 0x38, "ro"), ("max_transfer_words", 0x3c, "ro"),
            ("bulk_buffer_offset", 0x40, "ro"),
        )
        regions["flash"] = SoCCSRRegion(FLASH_BASE, 32, [
            MockCSR(name, 1, mode, origin=FLASH_BASE + offset)
            for name, offset, mode in flash_registers] + [
            MockCSR("buffer", 64, "ro", origin=FLASH_BASE + 0x100),
            MockCSR("command_packet", 68, "rw", origin=FLASH_BASE + 0x400)])
        if not self.golden:
            regions.update(self.camera_controls.regions)
            regions.update(self.camera_i2c.regions)
            regions["network"] = SoCCSRRegion(NETWORK_BASE, 32, [
                MockCSR(name, 1, mode, origin=NETWORK_BASE + offset)
                for name,offset,mode in (
                    ("magic",0,"ro"),("status",4,"ro"),("ip_address",8,"ro"),
                    ("netmask",12,"ro"),("gateway",16,"ro"),("rx_length",20,"ro"),
                    ("rx_source",24,"ro"),("rx_data",28,"ro"),("rx_release",32,"wo"),
                    ("tx_address",36,"ro"),("tx_length",40,"ro"),("tx_destination",44,"ro"),
                    ("tx_start",48,"wo"),("mac_word0",52,"ro"),("mac_word1",56,"ro"),
                    ("preferred_ip",60,"ro"),("state",64,"ro"),("lease_seconds",68,"ro"),
                    ("server",72,"ro"),("revoke_capture",76,"wo"),("rx_enable",80,"ro"))])
            regions["supervisor"] = SoCCSRRegion(SUPERVISOR_BASE, 32, [
                MockCSR(name, 1, mode, origin=SUPERVISOR_BASE + offset)
                for name, offset, mode in (
                    ("magic",0,"ro"), ("hardware",4,"ro"), ("status",8,"ro"),
                    ("error",12,"ro"), ("sequence",16,"rw"), ("command",20,"rw"),
                    ("arg0",24,"rw"), ("arg1",28,"rw"), ("completed",32,"ro"),
                    ("exposure_lines",36,"ro"), ("gain",40,"ro"), ("ev_compensation_code",44,"ro"),
                    ("p90_0",48,"ro"), ("p90_1",52,"ro"), ("auto",56,"ro"),
                    ("heartbeat",60,"ro"), ("timer",64,"ro"), ("record_words",68,"ro"),
                    ("image_length",72,"ro"), ("image_crc",76,"ro"))])
        for name, region in regions.items():
            if name in self.csr.regions:
                raise ValueError(f"duplicate exported control region: {name}")
            # add_region enforces paging of the separate CSR bridge; this
            # aperture is decoded directly on Wishbone and uses byte origins.
            self.csr.regions[name] = region
        # SoC.finalize has already built the shared fabric. Timeouts belong to
        # the granted host; waiting behind a timed-out peer is not a bus fault.
        interconnect = self.bus._interconnect
        names = list(self.bus.masters)
        if hasattr(interconnect, "arbiter"):
            grant = interconnect.arbiter.rr.grant
            for name, guard in self._host_guards.items():
                self.comb += guard.granted.eq(grant == names.index(name))
        elif len(names) == 1:
            for guard in self._host_guards.values():
                self.comb += guard.granted.eq(1)
        else:
            raise ValueError("Wishbone guards require the shared fabric's arbiter grant")
        self.add_constant("LUTCAM_CAMERAS", 0 if self.golden else 2)
        self.add_constant("LUTCAM_OTA_FLASH_BASE", FLASH_BASE)
        self.add_constant("LUTCAM_OTA_BOOT_BASE", BOOT_BASE)
        self.add_constant("LUTCAM_GOLDEN", int(self.golden))
        self.add_constant("LUTCAM_REQUIRED_LINK_MBPS", 1000)
