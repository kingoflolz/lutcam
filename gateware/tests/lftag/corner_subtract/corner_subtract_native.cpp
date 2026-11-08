#include "VCornerSubtractProbe.h"
#include "verilated.h"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <set>
#include <vector>

// Independent arithmetic oracle: reinterpret each bit pattern as a signed
// mathematical integer, subtract in int64, then calculate its positive residue.
// No RTL subtraction expression or copied portable implementation is evaluated.
static int64_t signed_value(uint32_t bits, unsigned width) {
    const int64_t modulus = int64_t{1} << width;
    return bits < modulus / 2 ? int64_t(bits) : int64_t(bits) - modulus;
}

struct Coverage {
    unsigned width;
    uint64_t pairs = 0, directed_pairs = 0, exhaustive_a_values = 0;
    uint64_t exhaustive_b_values = 0, random_pairs = 0;
    uint64_t positive_wraps = 0, negative_wraps = 0, sign_quadrants[4] = {};
};

static void check(VCornerSubtractProbe& dut, Coverage& c, uint32_t a, uint32_t b) {
    uint64_t actual, full;
    switch (c.width) {
    case 15: dut.a15 = a; dut.b15 = b; break;
    case 16: dut.a16 = a; dut.b16 = b; break;
    case 18: dut.a18 = a; dut.b18 = b; break;
    default: std::abort();
    }
    // CLK is a literal zero in the actual helper. Changing operands and calling
    // eval must produce the answer immediately, with no time or clock advance.
    dut.eval();
    switch (c.width) {
    case 15: actual = dut.r15; full = dut.full15; break;
    case 16: actual = dut.r16; full = dut.full16; break;
    case 18: actual = dut.r18; full = dut.full18; break;
    default: std::abort();
    }
    const int64_t sa = signed_value(a, c.width), sb = signed_value(b, c.width);
    const int64_t mathematical = sa - sb;
    const int64_t modulus = int64_t{1} << c.width;
    int64_t residue = mathematical % modulus;
    if (residue < 0) residue += modulus;
    const uint64_t full_expected = mathematical < 0
        ? uint64_t(mathematical + (int64_t{1} << 48)) : uint64_t(mathematical);
    if (actual != uint64_t(residue) || full != full_expected) {
        std::cerr << "FAIL width=" << c.width << " a=" << sa << " b=" << sb
                  << " full=" << full << " expected_full=" << full_expected
                  << " result=" << actual << " expected_residue=" << residue << '\n';
        std::exit(1);
    }
    ++c.pairs;
    c.positive_wraps += mathematical >= modulus / 2;
    c.negative_wraps += mathematical < -modulus / 2;
    ++c.sign_quadrants[(sa < 0 ? 2 : 0) | (sb < 0 ? 1 : 0)];
}

static uint64_t random64(uint64_t& state) {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return state;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    VCornerSubtractProbe dut;
    dut.a15 = dut.b15 = dut.a16 = dut.b16 = dut.a18 = dut.b18 = 0;
    for (const unsigned width : {15u, 16u, 18u}) {
        Coverage c{width};
        const uint32_t domain = uint32_t{1} << width, mask = domain - 1;
        const uint32_t half = domain / 2;
        const std::set<uint32_t> anchors = {0, 1, 2, half - 2, half - 1,
            half, half + 1, mask - 1, mask, 0x15555u & mask, 0x2aaaau & mask};
        std::set<uint32_t> directed = anchors;
        // Every single bit, inverted bit, and carry/borrow neighbor in both
        // signed domains. Cross products include all extrema combinations.
        for (unsigned bit = 0; bit < width; ++bit) {
            uint32_t power = uint32_t{1} << bit;
            for (uint32_t value : {power, mask ^ power, (power - 1) & mask,
                    (power + 1) & mask, (half + power) & mask,
                    (half - power) & mask}) directed.insert(value);
        }
        for (uint32_t a : directed) for (uint32_t b : directed) {
            check(dut, c, a, b);
            ++c.directed_pairs;
        }
        // Every possible bit pattern appears as each operand against a fixed
        // independent set of extremes/alternating bits; this reaches every
        // input value even at width 18 rather than testing only reduced values.
        for (uint32_t value = 0; value < domain; ++value) {
            for (uint32_t anchor : anchors) {
                check(dut, c, value, anchor);
                check(dut, c, anchor, value);
            }
            ++c.exhaustive_a_values;
            ++c.exhaustive_b_values;
        }
        uint64_t rng = 0x709de348ab6c12f5ULL ^ width;
        for (uint64_t i = 0; i < 750000; ++i) {
            const uint32_t a = random64(rng) & mask;
            const uint32_t b = random64(rng) & mask;
            check(dut, c, a, b);
            ++c.random_pairs;
        }
        std::cout << "RESULT {\"width\":" << width << ",\"pairs\":" << c.pairs
                  << ",\"directed_pairs\":" << c.directed_pairs
                  << ",\"exhaustive_a_values\":" << c.exhaustive_a_values
                  << ",\"exhaustive_b_values\":" << c.exhaustive_b_values
                  << ",\"random_pairs\":" << c.random_pairs
                  << ",\"positive_wraps\":" << c.positive_wraps
                  << ",\"negative_wraps\":" << c.negative_wraps
                  << ",\"clock_edges\":0,\"sign_quadrants\":["
                  << c.sign_quadrants[0] << ',' << c.sign_quadrants[1] << ','
                  << c.sign_quadrants[2] << ',' << c.sign_quadrants[3] << "]}\n";
    }
    dut.final();
    return 0;
}
