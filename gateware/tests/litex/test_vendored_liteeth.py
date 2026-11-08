"""Exercise the default integration, with no post-construction patch calls."""
from types import SimpleNamespace
import pytest
from migen import Module, Signal
from liteeth.common import eth_phy_description
from liteeth.core import LiteEthUDPIPCore
from liteeth.core.arp import LiteEthARPRX
from liteeth.core.arp_rx import CompactARPRX
from liteeth.core.checksum import IterativeIPV4Checksum
from liteeth.core.ip_rx import StreamingIPRX
from liteeth.frontend.etherbone import LiteEthEtherbone, LiteEthEtherboneRecordReceiver
from liteeth.frontend.etherbone_receiver import StreamingEtherboneRecordReceiver
from liteeth.mac.crc_rom import ByteCrcRomInserter
from liteeth.packet import IndexedPacketizer
from litex.soc.interconnect import stream


class Phy(Module):
    dw = 8
    def __init__(self):
        self.sink = stream.Endpoint(eth_phy_description(8))
        self.source = stream.Endpoint(eth_phy_description(8))


@pytest.mark.parametrize("dw", [8, 16, 32])
@pytest.mark.parametrize("dynamic", [False, True])
def test_default_network_pipeline(dw, dynamic):
    address = Signal(32) if dynamic else 0xc0a80132
    core = LiteEthUDPIPCore(Phy(), 0x024c55540001, address, 120_000_000,
                           dw=dw, arp_entries=2, with_icmp=False)
    etherbone = LiteEthEtherbone(core.udp, 1234, buffer_depth=256)
    assert isinstance(core.arp.rx, LiteEthARPRX if dynamic else CompactARPRX)
    assert isinstance(core.ip.tx.checksum, IterativeIPV4Checksum)
    assert isinstance(core.ip.tx.packetizer, IndexedPacketizer)
    assert any(isinstance(stage, ByteCrcRomInserter) for stage in core.mac.core.tx_datapath.pipeline)
    assert etherbone.probe.fifo.param_fifo.depth == 1
    if dw == 16:
        assert isinstance(core.ip.rx, StreamingIPRX)
        assert core.mac.checked_frame_gate.memory.depth == 1024
        assert core.udp.rx_frame_validated
        assert isinstance(etherbone.record.receiver, StreamingEtherboneRecordReceiver)
        assert not hasattr(etherbone.record.receiver, "fifo")
    else:
        assert not core.udp.rx_frame_validated
        assert isinstance(etherbone.record.receiver, LiteEthEtherboneRecordReceiver)
