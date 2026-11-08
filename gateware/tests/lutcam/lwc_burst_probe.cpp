#include "Vlwc_burst_tb.h"
#include "verilated.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>

// One copy suffices: two synchronized identical encoders have identical output
// strobes, so each observed word offers four bytes to their shared link.
int main(int argc, char** argv) {
    if (argc < 3 || argc > 5 || (std::strcmp(argv[1], "binary") != 0
                      && std::strcmp(argv[1], "flat") != 0
                      && std::strcmp(argv[1], "late") != 0
                      && std::strcmp(argv[1], "mixed") != 0)) {
        std::fprintf(stderr, "usage: probe binary|flat|late|mixed output.bin [trace.csv [frames]]\n");
        return 1;
    }
    Verilated::commandArgs(argc, argv);
    Vlwc_burst_tb model;
    model.clk = 0;
    model.rst = 1;
    model.in_valid = 0;
    model.in_sof = 0;
    model.in_px0 = 0;
    model.in_px1 = 0;
    for (int i = 0; i < 8; ++i) {
        model.clk = 0;
        model.eval();
        model.clk = 1;
        model.eval();
    }
    model.rst = 0;

    const bool flat = std::strcmp(argv[1], "flat") == 0;
    const bool late = std::strcmp(argv[1], "late") == 0;
    const bool mixed = std::strcmp(argv[1], "mixed") == 0;
    const unsigned frames = argc >= 5 ? std::strtoul(argv[4], nullptr, 10) : 1;
    if (frames == 0 || frames > 10)
        return 1;
    std::uint64_t state = 0x243f6a8885a308d3ULL;
    unsigned pixel_index = 0, frame_index = 0;
    auto next_pixel = [&]() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        const bool flat_now = flat || (mixed && frame_index % 3 == 0)
            || ((late || (mixed && frame_index % 3 == 2)) && pixel_index < 560 * 1280);
        ++pixel_index;
        return flat_now ? 128 : ((state & 1) ? 0 : 255);
    };
    FILE* output = std::fopen(argv[2], "wb");
    if (output == nullptr) {
        std::perror("opening output");
        return 1;
    }
    FILE* trace = argc >= 4 ? std::fopen(argv[3], "w") : nullptr;
    if (argc >= 4 && trace == nullptr)
        return 1;
    if (trace)
        std::fprintf(trace, "cycle,words,semantic_words,groups_serialized,packet_bits,padding,data,sof,frame_index\n");

    std::int64_t queue24 = 0, peak_queue24 = 0;
    std::int64_t cumulative24 = 0, peak_cumulative24 = 0;
    std::int64_t semantic_queue_bits = 0, peak_semantic_queue_bits = 0;
    unsigned words = 0, semantic_words = 0, sof_count = 0;
    unsigned input_pairs = 0;
    int first_cycle = -1, packets_done_cycle = -1, peak_cycle = -1;
    unsigned peak_semantic_words = 0, peak_groups = 0;
    for (unsigned cycle = 0; cycle < frames * 1000000; ++cycle) {
        // Production cadence: 640 adjacent pixel pairs followed by 256 blank
        // clocks per row, 800 rows, and a 1,000,000-clock frame period.
        model.clk = 0;
        const unsigned frame_cycle = cycle % 1000000;
        frame_index = cycle / 1000000;
        if (frame_cycle == 0) {
            state = 0x243f6a8885a308d3ULL;
            pixel_index = 0;
        }
        model.in_valid = frame_cycle / 896 < 800 && frame_cycle % 896 < 640;
        model.in_sof = frame_cycle == 0;
        if (model.in_valid) {
            model.in_px0 = next_pixel();
            model.in_px1 = next_pixel();
            ++input_pairs;
        } else {
            model.in_px0 = 0;
            model.in_px1 = 0;
        }
        model.eval();

        // Sample the values transferred at the upcoming rising edge.
        if (model.protocol_error) {
            std::fprintf(stderr, "encoder protocol error at cycle %d\n", cycle);
            return 2;
        }
        if (model.out_sof && (!model.out_valid || words % 64000 != 0)) {
            std::fprintf(stderr, "invalid SOF at cycle %d\n", cycle);
            return 2;
        }
        if (model.packets_done && packets_done_cycle < 0)
            packets_done_cycle = cycle;
        const bool offered = model.out_valid && !model.out_padding;
        if (model.out_valid) {
            if (first_cycle < 0)
                first_cycle = cycle;
            ++words;
            if (model.out_sof)
                ++sof_count;
            if (offered)
                ++semantic_words;
            if (model.out_padding && model.out_data != 0) {
                std::fprintf(stderr, "nonzero padding at cycle %d\n", cycle);
                return 2;
            }
            std::fputc(model.out_data >> 8, output);
            std::fputc(model.out_data & 255, output);
            if (trace)
                std::fprintf(trace, "%u,%u,%u,%u,%u,%u,%u,%u,%u\n", cycle,
                    words, semantic_words, model.groups_serialized, model.packet_bits,
                    model.out_padding, model.out_data, model.out_sof, frame_index);
        }
        // Maximum backlog of a two-bit/clock ideal server equals the maximum
        // excess over that rate over EVERY contiguous interval, including
        // intervals spanning frame boundaries. The promised burst is 5696 bytes.
        semantic_queue_bits = std::max(std::int64_t(0),
            semantic_queue_bits + (offered ? 16 : 0) - 2);
        peak_semantic_queue_bits = std::max(peak_semantic_queue_bits, semantic_queue_bits);

        // At 120 MHz, ideal gigabit service is exactly 25/24 bytes per
        // clock. Ignore Ethernet overhead, competing streams, packet commit
        // delays, and padding; each omission makes this a conservative bound.
        // Integers in units of 1/24 byte avoid cumulative rounding errors.
        queue24 = std::max(std::int64_t(0), queue24 + (offered ? 96 : 0) - 25);
        cumulative24 += (offered ? 96 : 0) - 25;
        if (queue24 > peak_queue24) {
            peak_queue24 = queue24;
            peak_cycle = cycle;
            peak_semantic_words = semantic_words;
            peak_groups = model.groups_serialized;
        }
        peak_cumulative24 = std::max(peak_cumulative24, cumulative24);
        model.clk = 1;
        model.eval();
    }
    std::fclose(output);
    if (trace)
        std::fclose(trace);
    std::printf(
        "{\"words\":%u,\"semantic_words\":%u,\"sof_count\":%u,"
        "\"input_pairs\":%u,\"first_cycle\":%d,\"packets_done_cycle\":%d,"
        "\"peak_queue24\":%lld,\"peak_cumulative24\":%lld,"
        "\"peak_cycle\":%d,\"peak_semantic_words\":%u,"
        "\"peak_groups\":%u,\"packet_bits\":%u,\"peak_semantic_queue_bits\":%lld}\n",
        words, semantic_words, sof_count, input_pairs, first_cycle,
        packets_done_cycle, static_cast<long long>(peak_queue24),
        static_cast<long long>(peak_cumulative24), peak_cycle,
        peak_semantic_words, peak_groups, model.packet_bits,
        static_cast<long long>(peak_semantic_queue_bits));
    return words == frames * 64000 && input_pairs == frames * 512000 && sof_count == frames
        && packets_done_cycle >= 0 ? 0 : 3;
}
