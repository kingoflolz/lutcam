//! Reproducible host-decoder corpus. Encoding and verification are outside the
//! timed loop so the ignored benchmark measures the live viewer's operation.

use std::hint::black_box;
use std::time::Instant;

use lwc::{Codec, FRAME_HEIGHT, FRAME_PIXELS, FRAME_WIDTH};

fn corpus() -> Vec<(&'static str, Vec<u8>)> {
    let dense_tags = (0..FRAME_PIXELS)
        .map(|index| {
            let x = index % FRAME_WIDTH;
            let y = index / FRAME_WIDTH;
            let (tx, ty) = (x % 64, y % 64);
            let cell = (x / 64) + 20 * (y / 64);
            let ink = (8..56).contains(&tx)
                && (8..56).contains(&ty)
                && (!(14..50).contains(&tx)
                    || !(14..50).contains(&ty)
                    || ((tx / 6 + ty / 6 * 7 + cell * 13) & 3) == 0);
            if ink { 19 } else { 226 }
        })
        .collect();
    let texture = (0..FRAME_PIXELS)
        .map(|index| {
            let x = index % FRAME_WIDTH;
            let y = index / FRAME_WIDTH;
            ((17 * x + 29 * y + 3 * x * y + (x ^ y)) & 255) as u8
        })
        .collect();
    vec![
        ("dense-tags", dense_tags),
        ("texture", texture),
        ("flat", vec![128; FRAME_PIXELS]),
    ]
}

fn compact(pixels: &[u8]) -> Vec<u8> {
    let frame = Codec::encode(pixels, FRAME_WIDTH, FRAME_HEIGHT).unwrap();
    let end = *Codec::packet_end_bits(&frame).unwrap().last().unwrap();
    frame.bytes[..end.div_ceil(16) * 2].to_vec()
}

fn fingerprint(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325, |hash, &byte| {
        (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
    })
}

#[test]
fn compact_decode_corpus_has_frozen_reconstructions() {
    // Frozen from the original scalar decoder before the host optimization.
    let expected = [
        (128_000, 0xc2397e4bd449e429),
        (128_000, 0x07557ba3f14c2032),
        (7_502, 0x6076239abccaa325),
    ];
    for ((name, pixels), (encoded_len, decoded_hash)) in corpus().into_iter().zip(expected) {
        let encoded = compact(&pixels);
        let decoded = Codec::decode_compact(&encoded, FRAME_WIDTH, FRAME_HEIGHT).unwrap();
        assert_eq!(encoded.len(), encoded_len, "{name} encoded extent");
        assert_eq!(fingerprint(&decoded), decoded_hash, "{name} reconstruction");
        assert_eq!(decoded.len(), FRAME_PIXELS);
    }
}

#[test]
#[ignore = "manual release-mode host-decoder timing; no platform-dependent speed assertion"]
fn benchmark_compact_decode() {
    let iterations = std::env::var("LWC_BENCH_ITERATIONS")
        .ok()
        .map(|value| value.parse::<usize>().unwrap())
        .unwrap_or(120);
    assert!(iterations > 0);
    for (name, pixels) in corpus() {
        let encoded = compact(&pixels);
        let reference = Codec::decode_compact(&encoded, FRAME_WIDTH, FRAME_HEIGHT).unwrap();
        for _ in 0..4 {
            assert_eq!(
                Codec::decode_compact(&encoded, FRAME_WIDTH, FRAME_HEIGHT).unwrap(),
                reference
            );
        }
        let start = Instant::now();
        for _ in 0..iterations {
            black_box(
                Codec::decode_compact(black_box(&encoded), FRAME_WIDTH, FRAME_HEIGHT).unwrap(),
            );
        }
        let elapsed = start.elapsed().as_secs_f64();
        eprintln!(
            "decode {name}: {} bytes; {iterations} frames; {:.3} ms/frame; {:.1} frames/s; fnv64={:016x}",
            encoded.len(),
            elapsed * 1000.0 / iterations as f64,
            iterations as f64 / elapsed,
            fingerprint(&reference)
        );
    }
}
