#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
efinity_home="${EFINITY_HOME:-}"
build_root="${1:-build/codec-encoder-top}"
requested_flow="${2:-pnr}"
requested_mode="${3:-area}"
project="codec_encoder_top"
manifest="rtl/lwc/codec_encoder_sources.f"
constraint="rtl/lwc/codec_encoder_top.sdc"

if [[ -z "${efinity_home}" || ! -f "${efinity_home}/bin/setup.sh" ]]; then
    echo "set EFINITY_HOME to an Efinity installation containing bin/setup.sh" >&2
    exit 1
fi
if [[ "${requested_flow}" != "map" && "${requested_flow}" != "pnr" ]]; then
    echo "usage: $0 [build-directory] [map|pnr] [speed|area]" >&2
    exit 1
fi
if [[ "${requested_mode}" != "speed" && "${requested_mode}" != "area" ]]; then
    echo "usage: $0 [build-directory] [map|pnr] [speed|area]" >&2
    exit 1
fi

cd "${repo_root}"
build_root="$(realpath -m "${build_root}")"
snapshot_dir="${build_root}/snapshot"
map_dir="${build_root}/map"
pnr_dir="${build_root}/pnr"
if [[ ! -f "${manifest}" || ! -f "${constraint}" ]]; then
    echo "missing production manifest or constraint" >&2
    exit 1
fi

sources=()
while IFS= read -r source_file; do
    source_file="${source_file%%#*}"
    source_file="${source_file//[[:space:]]/}"
    if [[ -n "${source_file}" ]]; then
        sources+=("${source_file}")
    fi
done < "${manifest}"

if [[ ${#sources[@]} -eq 0 ]]; then
    echo "production source manifest is empty" >&2
    exit 1
fi
for source_file in "${sources[@]}"; do
    if [[ ! -f "${source_file}" ]]; then
        echo "missing production source: ${source_file}" >&2
        exit 1
    fi
done

# Synthesis consumes an immutable source closure. This also leaves an exact
# hash record beside every map/P&R result used as design evidence.
mkdir -p "${snapshot_dir}" "${map_dir}" "${pnr_dir}"
snapshot_inputs=("${sources[@]}" "${manifest}" "${constraint}")
snapshot_names=()
for input_file in "${snapshot_inputs[@]}"; do
    input_name="$(basename "${input_file}")"
    for existing_name in "${snapshot_names[@]}"; do
        if [[ "${input_name}" == "${existing_name}" ]]; then
            echo "duplicate snapshot basename: ${input_name}" >&2
            exit 1
        fi
    done
    snapshot_names+=("${input_name}")
done

hash_before="$(sha256sum "${snapshot_inputs[@]}")"
for input_file in "${snapshot_inputs[@]}"; do
    cp "${input_file}" "${snapshot_dir}/$(basename "${input_file}")"
done
hash_after="$(sha256sum "${snapshot_inputs[@]}")"
if [[ "${hash_before}" != "${hash_after}" ]]; then
    echo "source changed while the synthesis snapshot was being copied" >&2
    exit 1
fi
for input_file in "${snapshot_inputs[@]}"; do
    live_hash="$(sha256sum "${input_file}" | awk '{print $1}')"
    snapshot_hash="$(sha256sum "${snapshot_dir}/$(basename "${input_file}")" \
        | awk '{print $1}')"
    if [[ "${live_hash}" != "${snapshot_hash}" ]]; then
        echo "snapshot copy differs from source: ${input_file}" >&2
        exit 1
    fi
done
printf '%s\n' "${hash_before}" > "${snapshot_dir}/SOURCE_SHA256SUMS"

set +u
# Efinity's setup script contains discovery pipelines that intentionally stop
# after their first match. Under this script's global pipefail setting those
# benign SIGPIPEs surface as status 141 and abort before synthesis.
set +o pipefail
source "${efinity_home}/bin/setup.sh" >/dev/null 2>&1
set -o pipefail
set -u

map_args=()
for source_file in "${sources[@]}"; do
    map_args+=(--v "${snapshot_dir}/$(basename "${source_file}")")
done

# The transform remains add/shift-only. Permit bounded multiplier inference
# for the two exact q-energy lanes and two causal cutoff products; the RTL
# never instantiates vendor DSPs.
efx_map \
    --family Titanium \
    --device Ti60F256 \
    "${map_args[@]}" \
    --root "${project}" \
    --project "${project}" \
    --work-dir "${map_dir}" \
    --output-dir "${map_dir}" \
    --verilog-macros SYNTHESIS \
    --mode "${requested_mode}" \
    --ram-decomp-mode 1 \
    --retiming 0 \
    --max_mult 40 \
    --use-logic-for-small-mem 0 \
    --max-bit-blast-mem-size 0

echo
echo "Map resource summary"
sed -n '/Resource Summary (begin)/,/Resource Summary (end)/p' \
    "${map_dir}/${project}.map.rpt"
echo
echo "Hierarchy resource distribution"
sed -n '1,$p' "${map_dir}/${project}.res.csv"

if [[ "${requested_flow}" == "map" ]]; then
    exit 0
fi

efx_run.py "${project}" \
    -f pnr \
    --family Titanium \
    -d Ti60F256 \
    --timing_model C4 \
    --dir "${snapshot_dir}" \
    --work_dir "${pnr_dir}" \
    --output_dir "${map_dir}" \
    --pnr_opts \
        sdc_file="${snapshot_dir}/$(basename "${constraint}")" \
        print_critical_path=5 \
        seed=1 \
        placer_effort_level=3

echo
echo "Placed resource summary"
sed -n '/Resource Summary (begin)/,/Resource Summary (end)/p' \
    "${map_dir}/${project}.place.rpt"
echo
echo "Clock summary"
sed -n '/Clock Frequency Summary (begin)/,/Clock Relationship Summary (end)/p' \
    "${map_dir}/${project}.timing.rpt"
echo
echo "Five largest setup paths"
awk '
    /Path Details for Max Critical Paths \(begin\)/ { in_max = 1 }
    /Path Details for Max Critical Paths \(end\)/ { in_max = 0 }
    in_max && (/^\+\+\+\+ Path / || /^Path Begin/ || /^Path End/ \
        || /^Slack/ || /^Delay/ || /^Logic Level/ \
        || /^Non-global nets on path/) { print }
' "${map_dir}/${project}.timing.rpt"

# Efinity reports a completed P&R stage even when the requested clock misses
# setup or hold timing. Treat either negative slack as a sign-off failure so
# this script's exit status represents device fit and the 120 MHz contract.
setup_slack="$(awk '
    /Setup \(Max\) Clock Relationship/ { in_setup = 1; next }
    /Hold \(Min\) Clock Relationship/ { in_setup = 0 }
    in_setup && $1 == "codec_clk" && $2 == "codec_clk" { print $4; exit }
' "${map_dir}/${project}.timing.rpt")"
if [[ -z "${setup_slack}" ]]; then
    echo "unable to read codec_clk setup slack" >&2
    exit 1
fi
if ! awk -v slack="${setup_slack}" 'BEGIN { exit !(slack >= 0.0) }'; then
    echo "timing sign-off failed: codec_clk setup slack ${setup_slack} ns" >&2
    exit 1
fi

hold_slack="$(awk '
    /Hold \(Min\) Clock Relationship/ { in_hold = 1; next }
    in_hold && $1 == "codec_clk" && $2 == "codec_clk" { print $4; exit }
' "${map_dir}/${project}.timing.rpt")"
if [[ -z "${hold_slack}" ]]; then
    echo "unable to read codec_clk hold slack" >&2
    exit 1
fi
if ! awk -v slack="${hold_slack}" 'BEGIN { exit !(slack >= 0.0) }'; then
    echo "timing sign-off failed: codec_clk hold slack ${hold_slack} ns" >&2
    exit 1
fi
