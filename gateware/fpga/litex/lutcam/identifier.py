"""Keep the small, immutable LiteX build identifier out of block RAM."""


# Allow at most 48 identifier bytes plus LiteX's NUL terminator. Increasing
# this allowance requires revisiting the explicit bit-blast budget.
IDENTIFIER_ROM_LOGIC_BITS = 49 * 8


def use_logic_identifier(identifier):
    """Change only ROM mapping; retain LiteX's bytes, CSR export and latency.

    Efinity documents ``syn_romstyle="logic"`` for an individual constant ROM.
    Its project-wide ``max-bit-blast-mem-size`` allowance must be at least
    IDENTIFIER_ROM_LOGIC_BITS; the automatic small-memory threshold stays 256.
    """
    memory = identifier.mem
    if (memory.width != 8 or memory.init is None or
            len(memory.init) != memory.depth or
            memory.width * memory.depth > IDENTIFIER_ROM_LOGIC_BITS):
        raise ValueError("Identifier exceeds the validated 392-bit logic-ROM budget")
    memory.attr = set(getattr(memory, "attr", ())) | {("syn_romstyle", "logic")}
