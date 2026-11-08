"""Wishbone/CSR camera frontend, with private or shared LFTag processing."""

from pathlib import Path

from migen import Cat, ClockSignal, Constant, If, Instance, ResetInserter, ResetSignal, Signal
from migen.genlib.cdc import MultiReg
from litex.build.io import DDROutput
from litex.gen import LiteXModule
from litex.soc.interconnect import stream, wishbone
from litex.soc.interconnect.csr import CSR, CSRStatus, CSRStorage

from .control import HistogramReader


CAMERA_ROM_FILES = {
    "GROUP_FAMILY_SCHEDULE_BIN": "rtl/lwc/raw_group_family_schedule.bin",
    "CENTROID_RECIPROCAL_SEGMENT_BIN": "rtl/lrcca/centroid_reciprocal_segments_q17.bin",
    "CENTROID_RECIPROCAL_MIDPOINT_REFLECTION_BIN":
        "rtl/lrcca/centroid_reciprocal_midpoint_reflection_q17.bin",
}


CALIBRATION = (
    ("inv_fx_q2_30", 32, 0), ("inv_fy_q2_30", 32, 0),
    ("cx_q16_16", 32, 0), ("cy_q16_16", 32, 0),
    ("printed_width_q16_16", 32, 0),
    ("mean_residual_limit_q16", 16, 1024),
    ("max_mass_spread", 8, 19),
    ("min_baseline_q4", 16, 16),
)

# Explicit offsets preserve every unrelated register when a setting shrinks.
CALIBRATION_OFFSETS = {
    "inv_fx_q2_30": 0x80, "inv_fy_q2_30": 0x84,
    "cx_q16_16": 0x88, "cy_q16_16": 0x8c,
    "printed_width_q16_16": 0x90, "mean_residual_limit_q16": 0x94,
    "max_mass_spread": 0xac,
    "min_baseline_q4": 0xb0,
}
CALIBRATION_RESERVED = (0x98, 0x9c, 0xa0, 0xa4, 0xa8, 0xb4)


# Optics remain camera-specific. Tag geometry and acceptance limits are common
# to the synchronized pair; both camera CSR banks address the same storage.
SHARED_CALIBRATION = frozenset((
    "printed_width_q16_16", "mean_residual_limit_q16",
    "max_mass_spread", "min_baseline_q4",
))


def shared_detector_configuration():
    return {name: Signal(width, reset=default, name="shared_" + name)
            for name, width, default in CALIBRATION if name in SHARED_CALIBRATION}


# Native centroid tuples cannot be backpressured. Each camera feeds its own
# candidate FIFO before the numeric decoder is shared.
LFTAG_CANDIDATE_FIELDS = (
    ("valid", 1), ("group_end", 1), ("x_q4", 15), ("y_q4", 14),
    ("log_mass_q5_3", 8), ("centroid_valid", 1), ("scale", 2),
    ("frame_seq", 8),
)
LFTAG_RETURN_FIELDS = (
    ("result_valid", 1), ("result_data", 32), ("result_last", 1),
    ("busy", 1), ("fifo_level", 6), ("candidate_start", 1),
    ("candidate_done", 1), ("candidate_cycles", 10),
    ("fifo_drop_sticky", 1), ("protocol_error_sticky", 1),
)
LFTAG_DEBUG_FIELDS = ("status", "protocol", "drop")
LFTAG_DEBUG_MAGIC = 0x4c464400


class Camera(LiteXModule):
    def get_csrs(self, *args, **kwargs):
        if self._direct_control:
            return []
        return super().get_csrs(*args, **kwargs)

    def __init__(self, platform, hsio, pads, root, *, direct_control=False, paired=False,
                 shared_lftag=False, detector_configuration=None, accepted_centroids=False,
                 shared_readback=None):
        if detector_configuration is not None:
            if not direct_control or not paired:
                raise ValueError("shared detector configuration requires paired direct control")
            if set(detector_configuration) != SHARED_CALIBRATION:
                raise ValueError("shared detector configuration must contain exactly the common fields")
            for name, width, _default in CALIBRATION:
                if name in SHARED_CALIBRATION and len(detector_configuration[name]) != width:
                    raise ValueError("incorrect shared detector field width: " + name)
        if accepted_centroids and not shared_lftag:
            raise ValueError("accepted centroid export requires the shared solver")
        self.accepted_centroids = accepted_centroids
        self._direct_control = direct_control
        self.shared_lftag = shared_lftag
        self.lftag_candidate = {
            name: Signal(width, name="lftag_in_" + name)
            for name, width in LFTAG_CANDIDATE_FIELDS
        }
        self.lftag_return = {
            name: Signal(width, name="external_lftag_" + name)
            for name, width in LFTAG_RETURN_FIELDS
        }
        self.lftag_processing_reset = Signal()
        if not shared_lftag:
            self.comb += [signal.eq(0) for signal in self.lftag_return.values()]
        self.isp_bus = bus = wishbone.Interface(data_width=32, addressing="word")
        # A camera reset cancels any partially consumed ISP histogram. Reset
        # its bus adapter too, so stale busy state cannot block the next dump.
        self.histogram = ResetInserter()(HistogramReader(shared_readback=shared_readback))
        # Histogram has its own Wishbone aperture, not recursive CSR banks.
        self.autocsr_exclude = {"histogram"}
        self.image = stream.Endpoint([("data", 16), ("sof", 1),
                                      ("padding", 1), ("frame_id", 32)])
        self.children = stream.Endpoint([("data", 33), ("end", 1), ("frame_id", 32)])
        self.pose = stream.Endpoint([("data", 32), ("frame_id", 32)])
        self.capture_permit = Signal()
        self.external_fault = Signal()
        self.active = Signal()
        self.idle = Signal()
        self.frame_id = Signal(32)
        self.pending_frame_id = Signal(32)
        self.fault = Signal()
        self.admission_request = Signal()
        self.admission_ready = Signal()
        self.frame_sof = Signal()
        self.frame_admitted = Signal()
        self.frame_retired = Signal()
        self.local_fault = Signal()
        self.pair_abort = Signal()
        self.fsin_output = Signal()

        self.run = CSRStorage(1, name="run", description="Admit frames after ISP/network configuration.")
        self.clear = CSR(name="clear")
        self.soft_reset = CSR(name="soft_reset")
        self.comb += self.histogram.reset.eq(self.soft_reset.re | self.fault)
        self.sensor_control = CSRStorage(3, name="sensor_control",
            description="Bit0 release shutdown; bit1 FSIN (reserved when paired); bit2 enable 25MHz XVCLK.")
        self.sensor_status = CSRStatus(1, name="sensor_status", description="Synchronized sensor strobe.")
        self.status = CSRStatus(8, name="status")
        self.receiver_status = CSRStatus(16, name="receiver_status")
        self.current_frame = CSRStatus(32, name="current_frame")
        self.frames_started = CSRStatus(32, name="frames_started")
        self.frames_completed = CSRStatus(32, name="frames_completed")
        self.ingress_high_water = CSRStatus(16, name="ingress_high_water")
        self.solver_fifo_level = CSRStatus(6, name="solver_fifo_level")
        self.solver_cycles = CSRStatus(10, name="solver_cycles")
        self.calibration_commit = CSR(name="calibration_commit")
        self.calibration_valid = CSRStatus(1, name="calibration_valid")
        self.configuration_error = CSRStatus(1, name="configuration_error")
        self.configuration_write_error = Signal()
        self.lftag_debug = {}
        for name in LFTAG_DEBUG_FIELDS:
            register = CSRStatus(32, name="solver_debug_" + name)
            setattr(self, "solver_debug_" + name, register)
            self.lftag_debug[name] = register.status
        self.solver_debug_magic = CSRStatus(32, name="solver_debug_magic")
        self.comb += self.solver_debug_magic.status.eq(LFTAG_DEBUG_MAGIC if shared_lftag else 0)
        if not shared_lftag:
            self.comb += [signal.eq(0) for signal in self.lftag_debug.values()]

        config = {}
        self.configuration = {}
        for name, width, default in CALIBRATION:
            if detector_configuration is not None and name in SHARED_CALIBRATION:
                active = detector_configuration[name]
            else:
                active = Signal(width, reset=default, name="active_" + name)
            self.configuration[name] = active
            config["i_lftag_cfg_" + name] = active
            if not direct_control:
                shadow = CSRStorage(width, reset=default, name="cfg_" + name)
                setattr(self, "cfg_" + name, shadow)
                self.sync += If(self.calibration_commit.re & self.idle & ~self.run.storage,
                                active.eq(shadow.storage))
                for offset in ({"mean_residual_limit_q16":
                                (0x98, 0x9c, 0xa0, 0xa4, 0xa8)}.get(name, ())):
                    reserved = CSRStatus(32, name=f"cfg_reserved_{offset:02x}",
                        description="Reserved calibration word; reads zero, writes ignored.")
                    setattr(self, f"cfg_reserved_{offset:02x}", reserved)
                    self.comb += reserved.status.eq(0)
        if not direct_control:
            # The final CSR word reads zero. This reserved
            # address is not a decoder setting and does not enter the bank.
            self.cfg_reserved = CSRStatus(1, name="cfg_reserved",
                description="Reserved calibration word; reads zero, writes ignored.")
            self.comb += self.cfg_reserved.status.eq(0)
        # Direct mode has one active bank. Its external Wishbone decoder owns
        # byte enables and permits writes only while idle and disarmed. The
        # standalone mode uses a shadow/commit interface.
        self.sync += [
            If(self.clear.re, self.configuration_error.status.eq(0)),
            If(self.calibration_commit.re,
                If(self.idle & ~self.run.storage,
                    self.calibration_valid.status.eq(1),
                ).Else(self.configuration_error.status.eq(1))),
        ]
        if direct_control:
            self.sync += If(self.configuration_write_error,
                            self.configuration_error.status.eq(1))

        clock_enable = Signal()
        self.specials += [
            MultiReg(self.sensor_control.storage[2], clock_enable, odomain="cam_ref"),
            MultiReg(pads.strobe, self.sensor_status.status),
            DDROutput(clock_enable, Constant(0), pads.xvclk, ClockSignal("cam_ref")),
        ]
        self.comb += pads.shutdown.eq(self.sensor_control.storage[0])
        if paired:
            self.comb += pads.fsin.eq(self.fsin_output)
        else:
            self.comb += [pads.fsin.eq(self.sensor_control.storage[1]),
                          self.admission_request.eq(self.run.storage)]

        reset = Signal()
        init_done = Signal()
        pixel_data = Signal(20)
        pixel_valid, pixel_sof, pixel_sol, pixel_eol, pixel_eof = [Signal() for _ in range(5)]
        solver_busy, solver_drop, solver_error, codec_error = [Signal() for _ in range(4)]
        candidate_start, candidate_done = Signal(), Signal()
        self.comb += [
            reset.eq(ResetSignal() | self.soft_reset.re),
            self.current_frame.status.eq(self.frame_id),
            self.image.frame_id.eq(self.frame_id), self.children.frame_id.eq(self.frame_id),
            self.pose.frame_id.eq(self.frame_id),
            self.frame_sof.eq(pixel_valid & pixel_sof),
            self.status.status.eq(Cat(self.active, self.idle, self.fault, init_done,
                                      solver_busy, solver_drop, solver_error, codec_error)),
        ]
        self.specials += Instance("csi2_rx",
            p_INIT_GUARD_CYCLES=12000, p_LPX_CYCLES=6, p_SOT_SETTLE_CYCLES=11,
            p_CLOCK_TIMEOUT_CYCLES=77, p_PIXEL_FIFO_DEPTH=64, p_PACK_MSB8_MARKERS=0,
            p_USE_DSP_COUNTERS=1,
            i_reset_n=~reset, i_ctrl_clk=ClockSignal(), i_byte_clk=hsio.clock.clk,
            i_pixel_clk=ClockSignal(), i_status_clear=self.clear.re,
            i_clk_lp_p=hsio.clock.lp_p, i_clk_lp_n=hsio.clock.lp_n,
            o_clk_hs_term=hsio.clock.hs_term, o_clk_hs_ena=hsio.clock.hs_ena,
            i_data_lp_p=Cat(*(lane.lp_p for lane in hsio.data)),
            i_data_lp_n=Cat(*(lane.lp_n for lane in hsio.data)),
            i_data0_hs_in=hsio.data[0].hs_in, i_data1_hs_in=hsio.data[1].hs_in,
            i_data_fifo_empty=Cat(*(lane.fifo_empty for lane in hsio.data)),
            o_data_fifo_rd=Cat(*(lane.fifo_rd for lane in hsio.data)),
            o_data_rst=Cat(*(lane.rst for lane in hsio.data)),
            o_data_hs_term=Cat(*(lane.hs_term for lane in hsio.data)),
            o_data_hs_ena=Cat(*(lane.hs_ena for lane in hsio.data)),
            o_init_done=init_done, o_status=self.receiver_status.status,
            o_pixel_data=pixel_data, o_pixel_valid=pixel_valid, o_pixel_sof=pixel_sof,
            o_pixel_sol=pixel_sol, o_pixel_eol=pixel_eol, o_pixel_eof=pixel_eof)

        # The platform emits these ROMs beside the generated HDL. $readmemb
        # resolves a relative filename from the synthesis/simulation cwd.
        memory_parameters = {"p_" + name: Path(path).name
                             for name, path in CAMERA_ROM_FILES.items()}
        hist = self.histogram
        external_lftag_ports = {
            "o_lftag_in_" + name: signal
            for name, signal in self.lftag_candidate.items()
        }
        external_lftag_ports.update({
            "i_external_lftag_" + name: signal
            for name, signal in self.lftag_return.items()
        })
        self.specials += Instance("CameraPipeline",
            p_PAIRED_MODE=int(paired), p_EXTERNAL_LFTAG=int(shared_lftag),
            p_USE_DSP_COUNTERS=1, p_USE_DSP_HIGH_ZOM=1, p_USE_DSP_HIGH_FOM=1,
            o_lftag_processing_reset=self.lftag_processing_reset,
            i_clk=ClockSignal(), i_rst=reset,
            i_pixel_data=pixel_data, i_pixel_valid=pixel_valid, i_pixel_sof=pixel_sof,
            i_pixel_eol=pixel_eol, i_pixel_eof=pixel_eof,
            i_receiver_fault=((self.receiver_status.status & 0x3fba) != 0) | self.external_fault,
            i_run_request=self.admission_request,
            i_pair_abort=self.pair_abort,
            i_capture_frame_id=self.pending_frame_id,
            i_capture_permit=self.capture_permit & init_done & self.calibration_valid.status,
            i_clear_diagnostics=self.clear.re,
            o_active=self.active, o_idle=self.idle, o_fault_sticky=self.fault,
            o_frame_id=self.frame_id, o_frames_started=self.frames_started.status,
            o_frames_completed=self.frames_completed.status,
            o_admission_ready=self.admission_ready,
            o_frame_admitted=self.frame_admitted, o_frame_retired=self.frame_retired,
            o_local_fault=self.local_fault,
            o_ingress_high_water=self.ingress_high_water.status,
            i_wb_cyc_i=bus.cyc, i_wb_stb_i=bus.stb, i_wb_we_i=bus.we,
            i_wb_sel_i=bus.sel, i_wb_adr_i=Cat(Constant(0, 2), bus.adr[:14]),
            i_wb_dat_i=bus.dat_w, o_wb_dat_o=bus.dat_r, o_wb_ack_o=bus.ack, o_wb_err_o=bus.err,
            i_hist_dump_req=hist.hist_dump_req, o_hist_dump_ready=hist.hist_dump_ready,
            o_hist_out_valid=hist.hist_out_valid, i_hist_out_ready=hist.hist_out_ready,
            o_hist_out_data=hist.hist_out_data,
            o_inner_fv_valid=Signal() if accepted_centroids else self.children.valid,
            o_inner_fv_data=Signal(33) if accepted_centroids else self.children.data,
            o_hier_inner_end=Signal() if accepted_centroids else self.children.end,
            o_lwc_out_sof=self.image.sof, o_lwc_out_valid=self.image.valid,
            o_lwc_out_data=self.image.data, o_lwc_out_padding=self.image.padding,
            o_lwc_protocol_error=codec_error,
            i_lftag_result_ready=self.pose.ready, i_lftag_clear_diagnostics=self.clear.re,
            o_lftag_result_valid=self.pose.valid, o_lftag_result_data=self.pose.data,
            o_lftag_result_last=self.pose.last, o_lftag_busy=solver_busy,
            o_lftag_fifo_level=self.solver_fifo_level.status,
            o_lftag_candidate_start=candidate_start, o_lftag_candidate_done=candidate_done,
            o_lftag_candidate_cycles=self.solver_cycles.status,
            o_lftag_fifo_drop_sticky=solver_drop, o_lftag_protocol_error_sticky=solver_error,
            **memory_parameters, **external_lftag_ports, **config)

def add_camera_sources(platform, root):
    """Use the production manifests, keeping package/dependency order stable."""
    root = Path(root)
    for relative in CAMERA_ROM_FILES.values():
        platform.add_rom_file(root / relative)
    seen = set()
    manifests = ("rtl/tagdetect_lwc/sources.f", "rtl/csi2_rx/sources.f",
                 "rtl/isp/sources.f")
    for manifest in manifests:
        for line in (root / manifest).read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                source = root / line
                if not source.is_file():
                    source = root / Path(manifest).parent / line
                if not source.is_file():
                    raise FileNotFoundError(f"Missing RTL source in {manifest}: {line}")
                if source not in seen:
                    seen.add(source)
                    platform.add_source(str(source), language="systemverilog")
    platform.add_source(str(root / "rtl/lutcam/CameraPipeline.sv"), language="systemverilog")
    for folder in ("rtl/lwc", "rtl/lrcca", "rtl/lftag"):
        platform.add_verilog_include_path(str(root / folder))
