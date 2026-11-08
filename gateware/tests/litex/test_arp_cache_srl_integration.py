"""Native two-entry storage is the default; larger tables retain RAM."""
import pytest
from migen import Instance, Memory
from liteeth.core.arp import LiteEthARPCache


@pytest.mark.parametrize("entries", [1, 2, 4, 8])
def test_cache_storage_matches_capacity(entries):
    cache = LiteEthARPCache(entries=entries, clk_freq=120_000_000)
    native = [item for item in cache._fragment.specials
              if isinstance(item, Instance) and item.of == "EFX_SRL8"]
    memories = [item for item in cache._fragment.specials if isinstance(item, Memory)]
    if entries <= 2:
        assert len(native) == 81
        assert not memories
    else:
        assert not native
        assert len(memories) == 1
        assert memories[0].depth == entries
