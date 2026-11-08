#!/usr/bin/env bash
set -euo pipefail

source_dir="${1:-Set12}"
output_dir="${2:-Set12-1280x800}"

if ! command -v magick >/dev/null 2>&1; then
    echo "ImageMagick's 'magick' command is required" >&2
    exit 1
fi

mkdir -p "$output_dir"
found=0
for input in "$source_dir"/*.png; do
    if [[ ! -e "$input" ]]; then
        continue
    fi
    found=1
    output="$output_dir/$(basename "$input")"
    # Repeat the original pixels without filtering or resampling.  Cropping
    # the infinite tile to the hardware raster gives every source image the
    # exact 1280x800 profile while preserving its native sample values.
    magick "$input" \
        -write mpr:source +delete \
        -size 1280x800 "tile:mpr:source" \
        -strip \
        -define png:exclude-chunk=time,date \
        -define png:color-type=0 \
        -depth 8 \
        "$output"
done

if [[ "$found" -eq 0 ]]; then
    echo "no PNG images found in $source_dir" >&2
    exit 1
fi
