#!/usr/bin/env bash
set -euo pipefail

source_dir="${1:-Set12-1280x800}"
output_dir="${2:-comp/JPEG}"
target_bytes="${3:-128000}"

for tool in cjpeg djpeg magick; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "'$tool' is required" >&2
        exit 1
    fi
done

if [[ ! "$target_bytes" =~ ^[1-9][0-9]*$ ]]; then
    echo "target byte count must be a positive integer" >&2
    exit 1
fi

mkdir -p "$output_dir"
tmp_dir="$(mktemp -d "${TMPDIR:-/tmp}/lwc-jpeg-comparison.XXXXXX")"
trap 'rm -rf "$tmp_dir"' EXIT

manifest="$output_dir/metrics.csv"
printf '%s\n' \
    'image,quality,semantic_bytes,unused_budget_bytes,bpp,sse,psnr_db,next_quality_bytes' \
    >"$manifest"

found=0
image_count=0
total_pixels=0
total_bytes=0
total_sse=0
psnr_sum=0

for input in "$source_dir"/*.png; do
    if [[ ! -e "$input" ]]; then
        continue
    fi
    found=1
    image_count=$((image_count + 1))

    name="$(basename "$input")"
    stem="${name%.*}"
    output="$output_dir/$stem.jpg"
    candidate="$tmp_dir/$stem.jpg"
    reconstruction="$tmp_dir/$stem.pgm"

    read -r width height < <(magick identify -format '%w %h\n' "$input")
    pixels=$((width * height))
    total_pixels=$((total_pixels + pixels))

    selected_quality=0
    semantic_bytes=0
    next_quality_bytes=0
    # Search downward, so the first fitting integer quality is the highest
    # fitting quality without assuming that optimized JPEG size is monotonic.
    for ((quality = 100; quality >= 1; quality--)); do
        cjpeg \
            -strict \
            -noicc \
            -grayscale \
            -baseline \
            -dct int \
            -optimize \
            -quality "$quality" \
            -outfile "$candidate" \
            "$input"
        candidate_bytes="$(stat -c '%s' "$candidate")"
        if ((candidate_bytes <= target_bytes)); then
            selected_quality="$quality"
            semantic_bytes="$candidate_bytes"
            mv "$candidate" "$output"
            break
        fi
        next_quality_bytes="$candidate_bytes"
    done

    if ((selected_quality == 0)); then
        echo "no JPEG quality fits $target_bytes bytes for $input" >&2
        exit 1
    fi

    djpeg \
        -strict \
        -grayscale \
        -dct int \
        -pnm \
        -outfile "$reconstruction" \
        "$output"

    normalized_mse="$(
        magick -precision 17 \
            "$input" "$reconstruction" \
            -compose difference -composite \
            -evaluate Pow 2 \
            -format '%[fx:mean]' \
            info:
    )"
    image_sse="$(
        awk -v mse="$normalized_mse" -v pixels="$pixels" \
            'BEGIN { printf "%.0f", mse * 65025 * pixels }'
    )"
    image_psnr="$(
        awk -v sse="$image_sse" -v pixels="$pixels" \
            'BEGIN { printf "%.12f", 10 * log(65025 * pixels / sse) / log(10) }'
    )"
    bpp="$(
        awk -v bytes="$semantic_bytes" -v pixels="$pixels" \
            'BEGIN { printf "%.9f", 8 * bytes / pixels }'
    )"
    unused_bytes=$((target_bytes - semantic_bytes))

    printf '%s,%d,%d,%d,%s,%s,%s,%d\n' \
        "$name" \
        "$selected_quality" \
        "$semantic_bytes" \
        "$unused_bytes" \
        "$bpp" \
        "$image_sse" \
        "$image_psnr" \
        "$next_quality_bytes" \
        >>"$manifest"

    total_bytes=$((total_bytes + semantic_bytes))
    total_sse=$((total_sse + image_sse))
    psnr_sum="$(awk -v sum="$psnr_sum" -v value="$image_psnr" 'BEGIN { printf "%.15f", sum + value }')"
    printf '%-6s quality=%3d bytes=%6d bpp=%s PSNR=%s dB\n' \
        "$stem" "$selected_quality" "$semantic_bytes" "$bpp" "$image_psnr"
done

if [[ "$found" -eq 0 ]]; then
    echo "no PNG images found in $source_dir" >&2
    exit 1
fi

total_budget_bytes=$((image_count * target_bytes))
unused_budget_bytes=$((total_budget_bytes - total_bytes))
aggregate_bpp="$(
    awk -v bytes="$total_bytes" -v pixels="$total_pixels" \
        'BEGIN { printf "%.12f", 8 * bytes / pixels }'
)"
aggregate_psnr="$(
    awk -v sse="$total_sse" -v pixels="$total_pixels" \
        'BEGIN { printf "%.12f", 10 * log(65025 * pixels / sse) / log(10) }'
)"
mean_psnr="$(
    awk -v sum="$psnr_sum" -v count="$image_count" \
        'BEGIN { printf "%.12f", sum / count }'
)"

cat >"$output_dir/README.txt" <<EOF
Optimized baseline grayscale JPEG comparison set

Source: $source_dir
Encoder: $(cjpeg -version 2>&1)
Decoder: $(djpeg -version 2>&1)
Command profile: -strict -noicc -grayscale -baseline -dct int -optimize
Rate selection: highest integer quality with complete JPEG <= $target_bytes bytes

Images: $image_count
Semantic bytes: $total_bytes / $total_budget_bytes
Unused transport budget: $unused_budget_bytes bytes
Semantic rate: $aggregate_bpp bits/pixel
Total SSE: $total_sse
Aggregate PSNR: $aggregate_psnr dB
Mean per-image PSNR: $mean_psnr dB

The JPEG files contain their natural codestreams. They are not extended with
bytes after EOI; unused rate is recorded in metrics.csv.
EOF

printf '\naggregate: bytes=%d/%d, unused=%d, bpp=%s, SSE=%d, PSNR=%s dB, mean=%s dB\n' \
    "$total_bytes" \
    "$total_budget_bytes" \
    "$unused_budget_bytes" \
    "$aggregate_bpp" \
    "$total_sse" \
    "$aggregate_psnr" \
    "$mean_psnr"
