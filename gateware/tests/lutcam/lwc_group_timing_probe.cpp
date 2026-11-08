#include "Vlwc_group_timing_tb.h"
#include "verilated.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

static std::vector<std::uint32_t> capture(bool flat) {
    Vlwc_group_timing_tb model;
    model.clk = 0;
    model.rst = 1;
    model.in_valid = model.in_sof = 0;
    model.in_px0 = model.in_px1 = 0;
    for (unsigned i = 0; i < 8; ++i) {
        model.clk = 0;
        model.eval();
        model.clk = 1;
        model.eval();
    }
    model.rst = 0;

    std::uint64_t state = 0x243f6a8885a308d3ULL;
    auto pixel = [&]() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return flat ? 128 : (state & 1 ? 0 : 255);
    };
    std::vector<std::uint32_t> cycles;
    for (std::uint32_t cycle = 0; cycle < 1000000; ++cycle) {
        model.clk = 0;
        model.in_valid = cycle / 896 < 800 && cycle % 896 < 640;
        model.in_sof = cycle == 0;
        model.in_px0 = model.in_valid ? pixel() : 0;
        model.in_px1 = model.in_valid ? pixel() : 0;
        model.eval();
        const unsigned before = model.groups_committed;
        model.clk = 1;
        model.eval();
        if (model.protocol_error) {
            std::fprintf(stderr, "encoder protocol error at cycle %u\n", cycle);
            std::exit(2);
        }
        if (model.groups_committed != before) {
            if (model.groups_committed != before + 1) {
                std::fprintf(stderr, "non-unit commit increment at cycle %u\n", cycle);
                std::exit(2);
            }
            cycles.push_back(cycle);
        }
    }
    return cycles;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: probe output-deltas.bin\n");
        return 1;
    }
    Verilated::commandArgs(argc, argv);
    const auto binary = capture(false), flat = capture(true);
    if (binary.size() != 64000 || flat.size() != 64000) {
        std::fprintf(stderr, "wrong commit counts: %zu, %zu\n", binary.size(), flat.size());
        return 3;
    }
    if (binary != flat) {
        unsigned i = 0;
        while (binary[i] == flat[i])
            ++i;
        std::fprintf(stderr, "data-dependent timing at group %u: %u, %u\n",
                     i, binary[i], flat[i]);
        return 4;
    }
    FILE* output = std::fopen(argv[1], "wb");
    if (!output) {
        std::perror("opening timing output");
        return 1;
    }
    std::uint32_t previous = 0;
    for (const auto cycle : binary) {
        const std::uint32_t delta = cycle - previous;
        previous = cycle;
        for (unsigned byte = 0; byte < 4; ++byte)
            std::fputc((delta >> (8 * byte)) & 255, output);
    }
    std::fclose(output);
    std::printf("identical binary/flat decision traces: groups=%zu first=%u last=%u\n",
                binary.size(), binary.front(), binary.back());
}
