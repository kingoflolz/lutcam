#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "$0")/../../../.." && pwd)"
output_dir="${1:-$repo_dir/artifacts/codec-comparison}"
build_dir="${TMPDIR:-/tmp}/lwc-codec-comparison-build"
mkdir -p "$build_dir/reference" "$output_dir"
curl -fL --retry 2 https://standards.iso.org/iso-iec/21122/-5/ed-3/en/ISO_IEC_21122-5_ED-3.zip -o "$build_dir/reference.zip"
unzip -qo "$build_dir/reference.zip" -d "$build_dir/reference"
cmake -S "$build_dir/reference" -B "$build_dir/jxs-build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir/jxs-build" -j 4
svt_revision=4d7e116c3ef259529a0917060e5b113834ed4894
curl -fL --retry 2 "https://codeload.github.com/OpenVisualCloud/SVT-JPEG-XS/tar.gz/$svt_revision" -o "$build_dir/svt.tar.gz"
tar -xzf "$build_dir/svt.tar.gz" -C "$build_dir"
cmake -S "$build_dir/SVT-JPEG-XS-$svt_revision" -B "$build_dir/svt-build" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_SHARED_LIBS=OFF -DCMAKE_OUTPUT_DIRECTORY="$build_dir/svt-bin"
cmake --build "$build_dir/svt-build" -j 4
cargo build --manifest-path "$repo_dir/Cargo.toml" --release -p lwc --example encode_dataset
python3 "$repo_dir/software/crates/lwc/tools/fetch_div2k_sample.py" "$output_dir" --count 12 --seed 20261107
python3 "$repo_dir/software/crates/lwc/tools/compare_codecs.py" "$output_dir" \
  --lwc-bin "$repo_dir/target/release/examples/encode_dataset" --jxs-bin "$build_dir/jxs-build/bin" \
  --svt-decoder "$build_dir/svt-bin/SvtJpegxsDecApp"
python3 "$repo_dir/software/crates/lwc/tools/render_comparison.py" "$output_dir" --inline "$output_dir/comparison-inline.html"
