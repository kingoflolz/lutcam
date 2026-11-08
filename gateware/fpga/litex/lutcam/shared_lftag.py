"""Connect two camera frontends to one candidate-arbitrated LFTag solver."""

from migen import Cat, ClockSignal, Constant, Instance, ResetSignal, Signal
from litex.gen import LiteXModule

from .camera import CALIBRATION, LFTAG_CANDIDATE_FIELDS, LFTAG_RETURN_FIELDS, LFTAG_DEBUG_FIELDS


class SharedLfTag(LiteXModule):
    def __init__(self, cameras, accepted_centroids=False, configuration_bank=None):
        if len(cameras) != 2 or not all(camera.shared_lftag for camera in cameras):
            raise ValueError("Shared LFTag requires exactly two external-solver cameras")

        # Packed camera zero is always the low slice, for tuples, calibration
        # and results alike. CameraPipeline retains frame retirement, header
        # frame-ID replacement and sticky fault handling around this engine.
        ports = {
            "i_clk": ClockSignal(), "i_rst": ResetSignal(),
            "i_camera_reset": Cat(*(c.lftag_processing_reset for c in cameras)),
            "i_clear_diagnostics": Cat(*(c.clear.re for c in cameras)),
            "i_out_ready": Cat(*(c.pose.ready for c in cameras)),
        }
        if accepted_centroids:
            if not all(camera.accepted_centroids for camera in cameras):
                raise ValueError("accepted centroid replay must own both camera child streams")
            centroid_data = Signal(33)
            ports.update({
                "p_EXPORT_ACCEPTED_CENTROIDS": 1,
                "i_centroid_ready": Cat(*(c.children.ready for c in cameras)),
                "o_centroid_valid": Cat(*(c.children.valid for c in cameras)),
                "o_centroid_last": Cat(*(c.children.end for c in cameras)),
                "o_centroid_data": centroid_data,
            })
            for camera in cameras:
                self.comb += camera.children.data.eq(centroid_data)
        for name, _ in LFTAG_CANDIDATE_FIELDS:
            ports["i_in_" + name] = Cat(*(c.lftag_candidate[name] for c in cameras))
        for name, width, _ in CALIBRATION:
            ports["i_cfg_" + name] = (Cat(*(c.configuration[name] for c in cameras))
                if configuration_bank is None else Constant(0, 2 * width))
        if configuration_bank is not None:
            ports.update({
                "p_EXTERNAL_CONFIG": 1,
                "o_config_read_valid": configuration_bank.cfg_req_valid,
                "o_config_read_address": configuration_bank.cfg_req_addr,
                "i_config_read_ready": configuration_bank.cfg_req_ready,
                "i_config_response_valid": configuration_bank.cfg_rsp_valid,
                "i_config_read_data": configuration_bank.cfg_rsp_data,
            })

        result_data = Signal(32)
        ports["o_out_data"] = result_data
        for camera in cameras:
            self.comb += camera.lftag_return["result_data"].eq(result_data)
        for name, _ in LFTAG_RETURN_FIELDS:
            if name == "result_data":
                continue
            port = {"result_valid": "out_valid", "result_last": "out_last"}.get(name, name)
            ports["o_" + port] = Cat(*(c.lftag_return[name] for c in cameras))
        for name in LFTAG_DEBUG_FIELDS:
            ports["o_debug_" + name] = Cat(*(c.lftag_debug[name] for c in cameras))
        self.specials += Instance("SharedLfTagDecoder", **ports)
