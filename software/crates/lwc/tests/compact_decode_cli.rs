use std::fs;
use std::path::PathBuf;
use std::process::{Command, Output};
use std::sync::atomic::{AtomicUsize, Ordering};

use lwc::{Codec, FRAME_HEIGHT, FRAME_PIXELS, FRAME_WIDTH};

struct Fixture {
    dir: PathBuf,
}

impl Fixture {
    fn new() -> Self {
        static NEXT_ID: AtomicUsize = AtomicUsize::new(0);
        let dir = std::env::temp_dir().join(format!(
            "lwc-compact-decode-{}-{}",
            std::process::id(),
            NEXT_ID.fetch_add(1, Ordering::Relaxed),
        ));
        fs::create_dir(&dir).unwrap();
        let frame = Codec::encode(&vec![128; FRAME_PIXELS], FRAME_WIDTH, FRAME_HEIGHT).unwrap();
        let used_bits = *Codec::packet_end_bits(&frame).unwrap().last().unwrap();
        fs::write(
            dir.join("compact.lwc"),
            &frame.bytes[..used_bits.div_ceil(16) * 2],
        )
        .unwrap();
        Self { dir }
    }

    fn decode(&self, input: &str, output: &str, flags: &[&str]) -> Output {
        Command::new(env!("CARGO_BIN_EXE_lwc"))
            .arg("decode")
            .arg(self.dir.join(input))
            .arg(self.dir.join(output))
            .args(flags)
            .output()
            .unwrap()
    }
}

impl Drop for Fixture {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.dir).unwrap();
    }
}

#[test]
fn cli_decodes_compact_files() {
    let fixture = Fixture::new();
    for (input, output, flags) in [
        ("compact.lwc", "compact-auto.png", &[][..]),
        ("compact.lwc", "compact-explicit.png", &["--compact"][..]),
    ] {
        let result = fixture.decode(input, output, flags);
        assert!(result.status.success(), "{:?}", result);
        let image = image::open(fixture.dir.join(output)).unwrap().to_luma8();
        assert_eq!(
            image.dimensions(),
            (FRAME_WIDTH as u32, FRAME_HEIGHT as u32)
        );
        assert_eq!(image.as_raw(), &vec![128; FRAME_PIXELS]);
    }
}

#[test]
fn cli_rejects_malformed_compact_input() {
    let fixture = Fixture::new();
    let compact = fs::read(fixture.dir.join("compact.lwc")).unwrap();
    let mut extra_word = compact.clone();
    extra_word.extend_from_slice(&[0, 0]);
    let mut nonzero_alignment = compact.clone();
    *nonzero_alignment.last_mut().unwrap() |= 1;
    for malformed in [
        &compact[..compact.len() - 2],
        &compact[..compact.len() - 1],
        extra_word.as_slice(),
        nonzero_alignment.as_slice(),
    ] {
        fs::write(fixture.dir.join("malformed.lwc"), malformed).unwrap();
        let result = fixture.decode("malformed.lwc", "invalid.png", &[]);
        assert!(!result.status.success(), "{:?}", result);
        assert!(!fixture.dir.join("invalid.png").exists());
    }
}
