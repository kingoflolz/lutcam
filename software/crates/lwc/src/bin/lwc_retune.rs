//! Fast, first-principles table tuner for experimental LWC profiles.
//!
//! This binary is intentionally separate from `scalar_codec.rs`: its output is
//! a proposed profile, not a second normative codec.  It caches the exact 5/3
//! transform once, uses exact packet lengths for every rate decision, and only
//! runs inverse transforms for accepted table vectors.

// This tool reuses only the normative transform and band helpers. The library
// module also contains public shared-frontend reference helpers that are not
// part of retuning.
#[allow(dead_code)]
#[path = "../transform.rs"]
mod transform;

use flate2::read::GzDecoder;
use image::ImageReader;
use std::array;
use std::env;
use std::error::Error;
use std::fmt::Write as _;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use std::time::Instant;

use transform::{Band, bands, forward_transform, inverse_transform};

const WIDTH: usize = 1280;
const HEIGHT: usize = 800;
const PIXELS: usize = WIDTH * HEIGHT;
const GROUP_LANES: usize = 16;
const GROUP_COUNT: usize = PIXELS / GROUP_LANES;
const BAND_COUNT: usize = 12;
const FRAME_BITS: usize = PIXELS;
const RICE_COUNT: usize = 8;
const MAX_PAIR_TOKEN_BITS: usize = 17;
const RAW_PACKET_BITS: u8 = 135;
const COARSE_RAW_PACKET_BITS: u8 = 147;
const ZERO_RUN_CREDIT_START: usize = 16;
const ZERO_RUN_MAX: usize = 256;
const DEFAULT_MAX_SHIFT: u8 = 11;
const DEFAULT_DEADZONE_Q8: u16 = 128;
const DEFAULT_CENTROID_Q8: u16 = 256;
const DEADZONE_CANDIDATES: [u16; 5] = [96, 128, 160, 192, 224];
const CENTROID_CANDIDATES: [u16; 5] = [224, 256, 288, 320, 352];
const BAND_NAMES: [&str; BAND_COUNT] = [
    "LL5", "H5", "H4", "LH3", "HL3", "HH3", "LH2", "HL2", "HH2", "LH1", "HL1", "HH1",
];
const CANONICAL_FINE_SHIFT: [u8; BAND_COUNT] = [1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4];
const CANONICAL_COARSE_SHIFT: [u8; BAND_COUNT] = [2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5];
const CANONICAL_FINE_BIAS: [u16; BAND_COUNT] = [0, 0, 0, 0, 1, 3, 2, 3, 6, 6, 6, 6];
const CANONICAL_COARSE_BIAS: [u16; BAND_COUNT] = [1, 1, 3, 3, 3, 7, 7, 7, 15, 15, 17, 15];
const CANONICAL_FINE_RICE: [u8; BAND_COUNT] = [4, 4, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0];
const CANONICAL_COARSE_RICE: [u8; BAND_COUNT] = [2, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0];
const CANONICAL_COARSE_RICE_ZERO: [u8; BAND_COUNT] = [1, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0];
const CANONICAL_COARSE_RICE_SMALL: [u8; BAND_COUNT] = [3, 1, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0];
const CANONICAL_COARSE_RICE_LARGE: [u8; BAND_COUNT] = [2, 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0];
const CANONICAL_COARSE_RICE_THRESHOLD_EXP: [u8; BAND_COUNT] = [0, 0, 0, 2, 2, 0, 1, 1, 0, 0, 0, 0];
const CANONICAL_NOMINAL_BITS: [u16; BAND_COUNT] =
    [139, 135, 102, 58, 59, 43, 27, 30, 21, 10, 13, 5];
const CANONICAL_COARSE_CUTOFF_BASE: u8 = 5;
const CANONICAL_INCREMENTAL_CUTOFF_BASE: u8 = 6;
const CANONICAL_DEBT_SHIFT: u8 = 14;
const CANONICAL_COARSE_ENERGY_SHIFT: [u8; BAND_COUNT] = [0, 2, 2, 0, 0, 0, 1, 1, 1, 2, 1, 0];
const CANONICAL_INCREMENTAL_ENERGY_SHIFT: [u8; BAND_COUNT] = [3, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0];
const NIBBLE_HUFFMAN_LENGTH: [usize; 16] = [1, 4, 6, 5, 5, 7, 6, 5, 4, 6, 7, 6, 5, 6, 5, 3];
const CANONICAL_RECONSTRUCTION_GAIN_Q8: [[u32; BAND_COUNT]; 2] = [
    [
        512, 512, 1025, 1030, 1022, 2060, 2087, 2081, 4174, 4148, 4167, 4251,
    ],
    [
        1023, 1029, 2051, 2069, 2054, 4124, 4163, 4171, 8236, 8410, 8139, 8019,
    ],
];
const CANONICAL_RECONSTRUCTION_OFFSET_Q8: [[i32; BAND_COUNT]; 2] = [
    [125, 120, 354, 422, 82, -33, 102, -37, 1, 13, -8, -142],
    [
        57, 124, 165, 3, 9, -140, -172, -273, -683, -1028, -551, -655,
    ],
];

fn main() -> Result<(), Box<dyn Error>> {
    let config = Config::parse(env::args().skip(1))?;
    if config.help {
        print_usage();
        return Ok(());
    }

    let started = Instant::now();
    println!("stage=load_and_cache begin");
    let corpus = Corpus::load(&config.set_dir)?;
    println!(
        "stage=load_and_cache images={} groups={} elapsed_s={:.3}",
        corpus.images.len(),
        corpus.images.len() * GROUP_COUNT,
        started.elapsed().as_secs_f64()
    );

    println!("stage=measure_synthesis_diagonal begin");
    let (synthesis_gain, distortion_weight) = measure_synthesis_diagonal();
    for band in 0..BAND_COUNT {
        println!(
            "synthesis band={} gain={:.8} integer_weight={}",
            BAND_NAMES[band], synthesis_gain[band], distortion_weight[band]
        );
    }

    let coder = PairBoundedRice {
        literal_mask: config.literal_mask,
    };
    println!(
        "stage=base_quantizer_census entropy={} max_shift={} begin",
        coder.name(),
        config.max_shift
    );
    let base = BaseCensus::build(&corpus, &coder, config.max_shift, config.rtl_profile);
    println!(
        "stage=base_quantizer_census elapsed_s={:.3}",
        started.elapsed().as_secs_f64()
    );

    if let Some(levels) = config.ladder_levels {
        run_ladder_probe(&corpus, &base, &distortion_weight, levels, started);
        return Ok(());
    }
    if config.mixed_lanes {
        let model = ProfileModel::canonical(&corpus, &coder);
        run_mixed_lane_probe(&corpus, &model, &distortion_weight, started);
        return Ok(());
    }
    if config.mixed_pairs {
        let model = ProfileModel::canonical(&corpus, &coder);
        run_mixed_pair_probe(&corpus, &model, &distortion_weight, started);
        return Ok(());
    }

    println!("stage=lagrangian_step_search begin");
    let target_bits = (corpus.images.len() * FRAME_BITS) as u64;
    let lambda = tune_unrestricted_lambda(&corpus, &base, &distortion_weight, target_bits);
    let mut model = if config.canonical_tables {
        ProfileModel::canonical(&corpus, &coder)
    } else {
        ProfileModel::best_two_level(&corpus, &base, &coder, &distortion_weight, lambda)
    };
    let mut lambda = tune_fixed_lambda(&corpus, &model, &distortion_weight, target_bits, lambda);
    println!(
        "stage=lagrangian_step_search lambda={lambda:.9} elapsed_s={:.3}",
        started.elapsed().as_secs_f64()
    );

    if config.tune_rice_context3 {
        if !config.canonical_tables {
            return Err("--tune-rice-context3 requires --canonical-tables".into());
        }
        run_rice_context3_tuning(
            &corpus,
            &mut model,
            &distortion_weight,
            config.policy,
            started,
        );
        return Ok(());
    }

    if config.tune_fixed_rice {
        if !config.canonical_tables {
            return Err("--tune-fixed-rice requires --canonical-tables".into());
        }
        run_fixed_rice_tuning(
            &corpus,
            &mut model,
            &distortion_weight,
            lambda,
            config.policy,
            started,
        );
        return Ok(());
    }

    // The fixed-Rice and context-search modes deliberately start from the
    // context-free seed above. Every ordinary canonical evaluation instead
    // installs the packet-local coarse profiles used by software and RTL.
    if config.canonical_tables {
        let context_census = RiceContextCensus::build(&corpus, &model);
        apply_context_profiles(&mut model, &context_census, &canonical_context_profiles());
        lambda = tune_fixed_lambda(&corpus, &model, &distortion_weight, target_bits, lambda);
    }

    // Coordinate descent is deliberately ordered.  A trial changes one band
    // and one quantizer parameter; exact coder lengths are regenerated, while
    // every other band's cached state is held fixed.
    for round in 0..config.rounds * usize::from(!config.canonical_tables) {
        println!("stage=coordinate_descent round={} begin", round + 1);
        for band in 0..BAND_COUNT {
            coordinate_band(
                &corpus,
                &coder,
                &distortion_weight,
                lambda,
                band,
                (config.max_shift, config.rtl_profile),
                &mut model,
            );
        }
        lambda = tune_fixed_lambda(&corpus, &model, &distortion_weight, target_bits, lambda);
        println!(
            "stage=coordinate_descent round={} lambda={lambda:.9} elapsed_s={:.3}",
            round + 1,
            started.elapsed().as_secs_f64()
        );
    }

    println!("stage=oracle_budget_fit begin");
    println!(
        "allocator metric={:?} policy={:?}",
        config.metric, config.policy
    );
    let oracle = oracle_decisions(&corpus, &model, &distortion_weight, lambda);
    let nominal_seed = fit_nominal_bits(&corpus, &model, &oracle);
    let causal_metrics = build_causal_metrics(&corpus, &model, &distortion_weight, config.metric);
    let (mut causal, mut causal_states) = if config.canonical_tables && config.fit_controller {
        let fitted = fit_hardware_controller(&corpus, &causal_metrics, config.policy);
        simulate_hardware_policy(
            &corpus,
            &causal_metrics,
            config.policy,
            fitted.nominal_bits,
            fitted.cutoff_base,
            fitted.incremental_cutoff_base,
            fitted.debt_shift,
            true,
        )
    } else if config.canonical_tables {
        simulate_canonical_policy(&corpus, &causal_metrics, config.policy, true)
    } else {
        let causal = fit_causal_policy(&corpus, &causal_metrics, nominal_seed);
        simulate_causal(
            &corpus,
            &causal_metrics,
            causal.nominal_bits,
            causal.cutoff_base,
            causal.debt_shift,
            true,
        )
    };
    let mut selected_coarse_energy_shift = CANONICAL_COARSE_ENERGY_SHIFT;
    let mut selected_incremental_energy_shift = CANONICAL_INCREMENTAL_ENERGY_SHIFT;
    if config.tune_energy_shifts {
        if !config.canonical_tables || !config.fit_controller {
            return Err(
                "--tune-energy-shifts requires --canonical-tables and --fit-controller".into(),
            );
        }
        let (coarse_shifts, incremental_shifts, tuned_causal, tuned_states) =
            tune_hardware_energy_shifts(&corpus, &model, &causal_metrics, config.policy, causal);
        selected_coarse_energy_shift = coarse_shifts;
        selected_incremental_energy_shift = incremental_shifts;
        causal = tuned_causal;
        causal_states = Some(tuned_states);
    }
    if config.tune_exact_controller {
        if !config.canonical_tables || !config.fit_controller {
            return Err(
                "--tune-exact-controller requires --canonical-tables and --fit-controller".into(),
            );
        }
        let (tuned_causal, tuned_states) = tune_hardware_controller_exact(
            &corpus,
            &model,
            &causal_metrics,
            config.policy,
            selected_coarse_energy_shift,
            selected_incremental_energy_shift,
            causal,
        );
        causal = tuned_causal;
        causal_states = Some(tuned_states);
    }
    println!(
        "stage=oracle_budget_fit coarse_cutoff={} incremental_cutoff={} debt_shift={} used_bits={} elapsed_s={:.3}",
        causal.cutoff_base,
        causal.incremental_cutoff_base,
        causal.debt_shift,
        causal.total_used_bits,
        started.elapsed().as_secs_f64()
    );

    println!("stage=accepted_exact_inverse begin");
    let (oracle_gains, oracle_offsets) =
        fit_state_affine_reconstruction(&corpus, &model, &oracle.states, None);
    let oracle_exact = exact_evaluate_states_with_affine(
        &corpus,
        &model,
        &oracle.states,
        &oracle_gains,
        &oracle_offsets,
    );
    let accepted_states = causal_states
        .as_ref()
        .expect("causal states were requested");
    let (mut reconstruction_gains, mut reconstruction_offsets) = if config.canonical_tables {
        (
            CANONICAL_RECONSTRUCTION_GAIN_Q8,
            CANONICAL_RECONSTRUCTION_OFFSET_Q8,
        )
    } else {
        fit_state_affine_reconstruction(&corpus, &model, accepted_states, None)
    };
    let mut exact = exact_evaluate_states_with_affine(
        &corpus,
        &model,
        accepted_states,
        &reconstruction_gains,
        &reconstruction_offsets,
    );
    if config.tune_reconstruction_exact {
        if !config.canonical_tables {
            return Err("--tune-reconstruction-exact requires --canonical-tables".into());
        }
        (reconstruction_gains, reconstruction_offsets, exact) = tune_exact_affine_reconstruction(
            &corpus,
            &model,
            accepted_states,
            reconstruction_gains,
            reconstruction_offsets,
            None,
            4,
        );
    }
    println!(
        "reconstruction aggregate_psnr_db={:.15} mean_psnr_db={:.9} total_sse={}",
        exact.aggregate_psnr, exact.mean_psnr, exact.total_sse,
    );
    if config.canonical_tables {
        let mut held_out_sse = 0u128;
        for excluded in 0..corpus.images.len() {
            let (held_out_gains, held_out_offsets) =
                fit_state_affine_reconstruction(&corpus, &model, accepted_states, Some(excluded));
            let held_out = exact_evaluate_states_with_affine(
                &corpus,
                &model,
                accepted_states,
                &held_out_gains,
                &held_out_offsets,
            );
            held_out_sse += u128::from(held_out.image_psnr[excluded].2);
        }
        println!(
            "coefficient_affine_fit leave_one_out_psnr_db={:.9} total_sse={}",
            psnr(held_out_sse as f64 / (corpus.images.len() * PIXELS) as f64),
            held_out_sse,
        );
    }
    let tuning_elapsed = started.elapsed().as_secs_f64();
    if config.validate_reconstruction_loo {
        if !config.canonical_tables {
            return Err("--validate-reconstruction-loo requires --canonical-tables".into());
        }
        let mut held_out_sse = 0u128;
        for excluded in 0..corpus.images.len() {
            let (fold_gains, fold_offsets) =
                fit_state_affine_reconstruction(&corpus, &model, accepted_states, Some(excluded));
            let (_, _, fold_exact) = tune_exact_affine_reconstruction(
                &corpus,
                &model,
                accepted_states,
                fold_gains,
                fold_offsets,
                Some(excluded),
                4,
            );
            held_out_sse += u128::from(fold_exact.image_psnr[excluded].2);
        }
        println!(
            "exact_reconstruction leave_one_out_psnr_db={:.9} total_sse={}",
            psnr(held_out_sse as f64 / (corpus.images.len() * PIXELS) as f64),
            held_out_sse,
        );
        println!(
            "exact_reconstruction leave_one_out_audit_elapsed_s={:.3}",
            started.elapsed().as_secs_f64()
        );
    }
    emit_tables(Report {
        model: &model,
        context_profiles: config.canonical_tables.then(canonical_context_profiles),
        distortion_weight,
        nominal_bits: causal.nominal_bits,
        lambda,
        causal,
        oracle_bits: oracle.total_bits,
        oracle_exact: &oracle_exact,
        exact: &exact,
        reconstruction_gains,
        reconstruction_offsets,
        coarse_energy_shift: selected_coarse_energy_shift,
        incremental_energy_shift: selected_incremental_energy_shift,
        elapsed: tuning_elapsed,
    });
    emit_adversarial_bounds(&exact);
    if tuning_elapsed > 60.0 {
        eprintln!(
            "warning: tuning took {tuning_elapsed:.3} s on this host (the design target is <60 s)"
        );
    }
    Ok(())
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct Config {
    set_dir: PathBuf,
    max_shift: u8,
    rounds: usize,
    rtl_profile: bool,
    canonical_tables: bool,
    metric: HardwareMetric,
    policy: HardwarePolicy,
    fit_controller: bool,
    tune_energy_shifts: bool,
    tune_exact_controller: bool,
    tune_fixed_rice: bool,
    tune_rice_context3: bool,
    tune_reconstruction_exact: bool,
    validate_reconstruction_loo: bool,
    literal_mask: bool,
    ladder_levels: Option<usize>,
    mixed_lanes: bool,
    mixed_pairs: bool,
    help: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HardwareMetric {
    Exact,
    CorrectionOnly,
    Square,
    Linear,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HardwarePolicy {
    Hull,
    CoarseFloor,
    FineOnly,
    NoDirectBypass,
    DirectFine,
    NoIncremental,
}

impl Config {
    fn parse(args: impl IntoIterator<Item = String>) -> Result<Self, String> {
        let mut result = Self {
            set_dir: PathBuf::from("Set12-1280x800"),
            max_shift: DEFAULT_MAX_SHIFT,
            rounds: 1,
            rtl_profile: false,
            canonical_tables: false,
            metric: HardwareMetric::Exact,
            policy: HardwarePolicy::NoDirectBypass,
            fit_controller: false,
            tune_energy_shifts: false,
            tune_exact_controller: false,
            tune_fixed_rice: false,
            tune_rice_context3: false,
            tune_reconstruction_exact: false,
            validate_reconstruction_loo: false,
            literal_mask: false,
            ladder_levels: None,
            mixed_lanes: false,
            mixed_pairs: false,
            help: false,
        };
        let mut args = args.into_iter();
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "-h" | "--help" => result.help = true,
                "--set" => {
                    result.set_dir = PathBuf::from(
                        args.next()
                            .ok_or_else(|| "--set requires a directory".to_owned())?,
                    );
                }
                "--max-shift" => {
                    let text = args
                        .next()
                        .ok_or_else(|| "--max-shift requires an integer".to_owned())?;
                    result.max_shift = text
                        .parse::<u8>()
                        .map_err(|_| "--max-shift must be an integer".to_owned())?;
                    if !(2..=14).contains(&result.max_shift) {
                        return Err("--max-shift must be in 2..=14".to_owned());
                    }
                }
                "--rounds" => {
                    let text = args
                        .next()
                        .ok_or_else(|| "--rounds requires an integer".to_owned())?;
                    result.rounds = text
                        .parse::<usize>()
                        .map_err(|_| "--rounds must be an integer".to_owned())?;
                    if result.rounds > 3 {
                        return Err("--rounds must be in 0..=3".to_owned());
                    }
                }
                "--rtl-quantizer" => result.rtl_profile = true,
                "--canonical-tables" => {
                    result.rtl_profile = true;
                    result.canonical_tables = true;
                }
                "--metric" => {
                    result.metric = match args
                        .next()
                        .ok_or_else(|| "--metric requires a value".to_owned())?
                        .as_str()
                    {
                        "exact" => HardwareMetric::Exact,
                        "correction-only" => HardwareMetric::CorrectionOnly,
                        "square" => HardwareMetric::Square,
                        "linear" => HardwareMetric::Linear,
                        other => {
                            return Err(format!(
                                "unknown metric {other}: expected exact, correction-only, square, or linear"
                            ));
                        }
                    };
                }
                "--policy" => {
                    result.policy = match args
                        .next()
                        .ok_or_else(|| "--policy requires a value".to_owned())?
                        .as_str()
                    {
                        "hull" => HardwarePolicy::Hull,
                        "coarse-floor" => HardwarePolicy::CoarseFloor,
                        "fine-only" => HardwarePolicy::FineOnly,
                        "no-direct-bypass" => HardwarePolicy::NoDirectBypass,
                        "direct-fine" => HardwarePolicy::DirectFine,
                        "no-incremental" => HardwarePolicy::NoIncremental,
                        other => {
                            return Err(format!(
                                "unknown policy {other}: expected hull, coarse-floor, fine-only, no-direct-bypass, direct-fine, or no-incremental"
                            ));
                        }
                    };
                }
                "--fit-controller" => result.fit_controller = true,
                "--tune-energy-shifts" => result.tune_energy_shifts = true,
                "--tune-exact-controller" => result.tune_exact_controller = true,
                "--tune-fixed-rice" => result.tune_fixed_rice = true,
                "--tune-rice-context3" => result.tune_rice_context3 = true,
                "--tune-reconstruction-exact" => result.tune_reconstruction_exact = true,
                "--validate-reconstruction-loo" => result.validate_reconstruction_loo = true,
                "--literal-mask" => result.literal_mask = true,
                "--ladder-levels" => {
                    let text = args
                        .next()
                        .ok_or_else(|| "--ladder-levels requires an integer".to_owned())?;
                    let levels = text
                        .parse::<usize>()
                        .map_err(|_| "--ladder-levels must be an integer".to_owned())?;
                    if !(2..=5).contains(&levels) {
                        return Err("--ladder-levels must be in 2..=5".to_owned());
                    }
                    result.ladder_levels = Some(levels);
                }
                "--mixed-lanes" => result.mixed_lanes = true,
                "--mixed-pairs" => result.mixed_pairs = true,
                other if !other.starts_with('-') => result.set_dir = PathBuf::from(other),
                other => return Err(format!("unknown argument: {other}")),
            }
        }
        Ok(result)
    }
}

fn print_usage() {
    println!("usage: cargo run --release --bin lwc_retune -- [OPTIONS]");
    println!("  --set DIR          benchmark PNG directory (default Set12-1280x800)");
    println!("  --max-shift N      largest power-of-two quantizer step (default 11)");
    println!("  --rounds N         ordered coordinate-descent passes, 0..=3 (default 1)");
    println!("  --rtl-quantizer    restrict quantizers to the current shift/correction datapath");
    println!("  --canonical-tables hold the canonical quantizer/Rice tables fixed");
    println!("  --metric NAME      allocator proxy: exact, correction-only, square, or linear");
    println!("  --policy NAME      allocator graph (use --help source for experimental names)");
    println!("  --fit-controller   refit nominal charges, cutoffs, and debt shift for the proxy");
    println!(
        "  --tune-energy-shifts coordinate-tune both 12-band metric-shift tables by exact SSE"
    );
    println!("  --tune-exact-controller tune fixed controller constants by exact pixel SSE");
    println!("  --tune-fixed-rice structured <60 s fixed-per-band Rice/profile retune");
    println!("  --tune-rice-context3 retune zero/small/large packet-local Rice contexts");
    println!("  --tune-reconstruction-exact calibrate scalar reconstruction by exact pixel SSE");
    println!(
        "  --validate-reconstruction-loo run the slower leave-one-image-out calibration audit"
    );
    println!("  --literal-mask     evaluate a fixed 16-bit nonzero mask without Huffman logic");
    println!("  --ladder-levels N  probe N consecutive nonzero scalar qualities, 2..=5");
    println!("  --mixed-lanes      probe per-coefficient fine/coarse selection syntax");
    println!("  --mixed-pairs      probe per-pair fine/coarse selection syntax");
}

#[derive(Clone)]
struct CachedImage {
    name: String,
    pixels: Vec<u8>,
    groups: Vec<[i32; GROUP_LANES]>,
}

struct Corpus {
    images: Vec<CachedImage>,
    schedule: Vec<u8>,
    band_entries: [Vec<usize>; BAND_COUNT],
    band_local_for_group: Vec<usize>,
    groups_per_band: [usize; BAND_COUNT],
}

impl Corpus {
    fn load(directory: &Path) -> Result<Self, Box<dyn Error>> {
        let mut paths = std::fs::read_dir(directory)?
            .filter_map(|entry| entry.ok().map(|entry| entry.path()))
            .filter(|path| {
                path.extension()
                    .and_then(|extension| extension.to_str())
                    .is_some_and(|extension| extension.eq_ignore_ascii_case("png"))
            })
            .collect::<Vec<_>>();
        paths.sort();
        if paths.is_empty() {
            return Err(format!("no PNG images found in {}", directory.display()).into());
        }

        let schedule = load_schedule()?;
        let band_list = bands(WIDTH, HEIGHT);
        let mut work = paths
            .into_iter()
            .map(|path| -> Result<_, Box<dyn Error>> {
                let decoded = ImageReader::open(&path)?.decode()?.to_luma8();
                if decoded.dimensions() != (WIDTH as u32, HEIGHT as u32) {
                    return Err(format!(
                        "{} is {}x{}, expected {WIDTH}x{HEIGHT}",
                        path.display(),
                        decoded.width(),
                        decoded.height()
                    )
                    .into());
                }
                let pixels = decoded.into_raw();
                let coefficients = pixels
                    .iter()
                    .map(|&value| i32::from(value))
                    .collect::<Vec<_>>();
                Ok((
                    path.file_name()
                        .and_then(|name| name.to_str())
                        .unwrap_or("<unnamed>")
                        .to_owned(),
                    pixels,
                    coefficients,
                ))
            })
            .collect::<Result<Vec<_>, _>>()?;

        std::thread::scope(|scope| {
            for (_, _, coefficients) in &mut work {
                scope.spawn(move || forward_transform(coefficients, WIDTH, HEIGHT));
            }
        });

        let images = work
            .into_iter()
            .map(|(name, pixels, coefficients)| CachedImage {
                name,
                pixels,
                groups: group_coefficients(&coefficients, &schedule, &band_list),
            })
            .collect::<Vec<_>>();

        let mut band_entries: [Vec<usize>; BAND_COUNT] = array::from_fn(|_| Vec::new());
        let mut groups_per_band = [0usize; BAND_COUNT];
        let mut band_local_for_group = vec![0usize; GROUP_COUNT];
        for (group, &band_id) in schedule.iter().enumerate() {
            let band = usize::from(band_id);
            band_local_for_group[group] = groups_per_band[band];
            groups_per_band[band] += 1;
        }
        for image in 0..images.len() {
            for (group, &band_id) in schedule.iter().enumerate() {
                band_entries[usize::from(band_id)].push(image * GROUP_COUNT + group);
            }
        }
        Ok(Self {
            images,
            schedule,
            band_entries,
            band_local_for_group,
            groups_per_band,
        })
    }

    fn source(&self, global_group: usize) -> &[i32; GROUP_LANES] {
        &self.images[global_group / GROUP_COUNT].groups[global_group % GROUP_COUNT]
    }

    fn local_point_index(&self, image: usize, group: usize, band: usize) -> usize {
        image * self.groups_per_band[band] + self.band_local_for_group[group]
    }
}

fn group_coefficients(
    coefficients: &[i32],
    schedule: &[u8],
    band_list: &[Band],
) -> Vec<[i32; GROUP_LANES]> {
    let mut band_groups = [0usize; BAND_COUNT];
    let mut groups = Vec::with_capacity(GROUP_COUNT);
    for &band_id in schedule {
        let band = usize::from(band_id);
        let ordinal = band_groups[band];
        band_groups[band] += 1;
        groups.push(
            array::from_fn(|lane| {
                let flat = ordinal * GROUP_LANES + lane;
                let rect = band_list[band].rect;
                (rect.y + flat / rect.width) * WIDTH + rect.x + flat % rect.width
            })
            .map(|index| coefficients[index]),
        );
    }
    assert_eq!(groups.len(), GROUP_COUNT);
    groups
}

fn load_schedule() -> Result<Vec<u8>, Box<dyn Error>> {
    let compressed = decode_base64(include_str!("../raw_group_schedule.b64"))?;
    let mut decoder = GzDecoder::new(compressed.as_slice());
    let mut text = String::new();
    decoder.read_to_string(&mut text)?;
    let schedule = text
        .lines()
        .map(str::parse::<u8>)
        .collect::<Result<Vec<_>, _>>()?;
    if schedule.len() != GROUP_COUNT || schedule.iter().any(|&band| usize::from(band) >= BAND_COUNT)
    {
        return Err("embedded group schedule has invalid dimensions".into());
    }
    Ok(schedule)
}

fn decode_base64(text: &str) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut output = Vec::with_capacity(text.len() * 3 / 4);
    let mut accumulator = 0u32;
    let mut bits = 0usize;
    for byte in text.bytes().filter(|byte| !byte.is_ascii_whitespace()) {
        if byte == b'=' {
            break;
        }
        let value = match byte {
            b'A'..=b'Z' => byte - b'A',
            b'a'..=b'z' => byte - b'a' + 26,
            b'0'..=b'9' => byte - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => return Err(format!("invalid base64 byte 0x{byte:02x}").into()),
        };
        accumulator = (accumulator << 6) | u32::from(value);
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            output.push((accumulator >> bits) as u8);
        }
    }
    Ok(output)
}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct QuantSpec {
    shift: u8,
    bias: u16,
    deadzone_q8: u16,
    centroid_q8: u16,
    ties_down: bool,
}

impl QuantSpec {
    fn base(shift: u8) -> Self {
        Self {
            shift,
            bias: 0,
            deadzone_q8: DEFAULT_DEADZONE_Q8,
            centroid_q8: DEFAULT_CENTROID_Q8,
            ties_down: false,
        }
    }

    fn rtl(shift: u8) -> Self {
        debug_assert!(shift > 0);
        let bias = ((1u16 << shift) - 1) / 2;
        Self::rtl_with_bias(shift, bias)
    }

    fn rtl_for_band(shift: u8, band: usize, coarse: bool) -> Self {
        let bias = if coarse {
            CANONICAL_COARSE_BIAS[band]
        } else {
            CANONICAL_FINE_BIAS[band]
        };
        Self::rtl_with_bias(shift, bias)
    }

    fn rtl_with_bias(shift: u8, bias: u16) -> Self {
        let step = 1u16 << shift;
        debug_assert!(bias < step);
        let deadzone_q8 = (u32::from(step - bias) * 256) / u32::from(step);
        Self {
            shift,
            bias,
            deadzone_q8: u16::try_from(deadzone_q8).expect("Q8 deadzone fits in u16"),
            centroid_q8: DEFAULT_CENTROID_Q8,
            ties_down: true,
        }
    }
}

fn quantize(value: i32, spec: QuantSpec) -> i16 {
    if spec.ties_down {
        let magnitude = value.unsigned_abs();
        let bias = u32::from(spec.bias);
        let magnitude = ((magnitude + bias) >> spec.shift).min(if value < 0 { 128 } else { 127 });
        return if value < 0 {
            -(magnitude as i16)
        } else {
            magnitude as i16
        };
    }
    let step = 1u64 << spec.shift;
    let scaled = u64::from(value.unsigned_abs()) * 256;
    let first_threshold = step * u64::from(spec.deadzone_q8);
    if scaled < first_threshold {
        return 0;
    }
    let magnitude = 1 + (scaled - first_threshold) / (step * 256);
    let limit = if value < 0 { 128 } else { 127 };
    let magnitude = magnitude.min(limit);
    if value < 0 {
        -(magnitude as i16)
    } else {
        magnitude as i16
    }
}

fn reconstruct(value: i16, spec: QuantSpec) -> i32 {
    if value == 0 {
        return 0;
    }
    let magnitude_index = u64::from(value.unsigned_abs() - 1);
    let step = 1u64 << spec.shift;
    let scaled = (magnitude_index * 256 + u64::from(spec.centroid_q8)) * step;
    let magnitude = ((scaled + 128) >> 8) as i32;
    if value < 0 { -magnitude } else { magnitude }
}

trait EntropyCoder {
    fn name(&self) -> &'static str;
    fn parameter_count(&self) -> usize;
    fn packet_bits(&self, values: &[i16; GROUP_LANES], parameter: usize, band: usize) -> u8;
}

/// Current hardware-bounded Rice payload model.  A future entropy coder only
/// needs to implement `EntropyCoder`; the quantizer/allocation search remains
/// unchanged.
struct PairBoundedRice {
    literal_mask: bool,
}

impl EntropyCoder for PairBoundedRice {
    fn name(&self) -> &'static str {
        if self.literal_mask {
            "pair-bounded-rice-literal-mask"
        } else {
            "pair-bounded-rice"
        }
    }

    fn parameter_count(&self) -> usize {
        RICE_COUNT
    }

    fn packet_bits(&self, values: &[i16; GROUP_LANES], parameter: usize, band: usize) -> u8 {
        let mask = lane_mask(values);
        if mask == 0 {
            return 1;
        }
        let base_k = parameter as u8;
        let mut previous_magnitude = 0u16;
        let mut coefficient_bits = 0usize;
        let mut pair_bits = 0usize;
        for (lane, &value) in values.iter().enumerate() {
            let k = contextual_rice_k(band, base_k, previous_magnitude);
            let length = coefficient_rice_bits(value, k);
            coefficient_bits += length;
            pair_bits += length;
            previous_magnitude = value.unsigned_abs();
            if lane & 1 != 0 {
                if pair_bits > MAX_PAIR_TOKEN_BITS {
                    return RAW_PACKET_BITS;
                }
                pair_bits = 0;
            }
        }
        let mask_bits = if self.literal_mask {
            16
        } else {
            (0..4)
                .map(|nibble| NIBBLE_HUFFMAN_LENGTH[usize::from((mask >> (4 * nibble)) & 15)])
                .sum::<usize>()
                .min(16)
        };
        (3 + mask_bits + coefficient_bits).min(usize::from(RAW_PACKET_BITS)) as u8
    }
}

fn lane_mask(values: &[i16; GROUP_LANES]) -> u16 {
    values.iter().enumerate().fold(0u16, |mask, (lane, value)| {
        mask | (u16::from(*value != 0) << lane)
    })
}

fn coefficient_rice_bits(value: i16, k: u8) -> usize {
    if value == 0 {
        return 0;
    }
    let remainder = value.unsigned_abs() - 1;
    2 + usize::from(k) + usize::from(remainder >> k)
}

fn contextual_rice_k(band: usize, base_k: u8, previous_magnitude: u16) -> u8 {
    let _ = (band, previous_magnitude);
    base_k
}

#[derive(Clone, Copy, Debug)]
struct Point {
    distortion: u64,
    bits: [u8; RICE_COUNT],
}

fn state_packet_bits(point: Point, entropy_parameter: u8, coarse: bool) -> u8 {
    let bits = point.bits[usize::from(entropy_parameter)];
    if coarse && bits == RAW_PACKET_BITS {
        COARSE_RAW_PACKET_BITS
    } else {
        bits
    }
}

fn point_for_group(
    source: &[i32; GROUP_LANES],
    spec: QuantSpec,
    coder: &impl EntropyCoder,
    band: usize,
) -> Point {
    debug_assert_eq!(coder.parameter_count(), RICE_COUNT);
    let values = source.map(|value| quantize(value, spec));
    let distortion = source
        .iter()
        .zip(values)
        .map(|(&original, quantized)| {
            let error = i64::from(original) - i64::from(reconstruct(quantized, spec));
            (error * error) as u64
        })
        .sum();
    Point {
        distortion,
        bits: array::from_fn(|parameter| coder.packet_bits(&values, parameter, band)),
    }
}

fn build_points(
    corpus: &Corpus,
    band: usize,
    spec: QuantSpec,
    coder: &impl EntropyCoder,
) -> Vec<Point> {
    corpus.band_entries[band]
        .iter()
        .map(|&entry| point_for_group(corpus.source(entry), spec, coder, band))
        .collect()
}

fn zero_distortion(source: &[i32; GROUP_LANES]) -> u64 {
    source
        .iter()
        .map(|&value| {
            let value = i64::from(value);
            (value * value) as u64
        })
        .sum()
}

fn best_entropy_parameter(points: &[Point], coarse: bool) -> u8 {
    let totals: [u64; RICE_COUNT] = array::from_fn(|parameter| {
        points
            .iter()
            .map(|&point| u64::from(state_packet_bits(point, parameter as u8, coarse)))
            .sum()
    });
    totals
        .iter()
        .enumerate()
        .min_by_key(|&(parameter, total)| (*total, parameter))
        .map(|(parameter, _)| parameter as u8)
        .unwrap_or(0)
}

struct BaseCensus {
    max_shift: u8,
    rtl_profile: bool,
    points: Vec<Vec<Point>>,
    fine_rice: Vec<u8>,
    coarse_rice: Vec<u8>,
}

impl BaseCensus {
    fn build(corpus: &Corpus, coder: &impl EntropyCoder, max_shift: u8, rtl_profile: bool) -> Self {
        let shift_count = usize::from(max_shift) + 1;
        let mut points = Vec::with_capacity(BAND_COUNT * shift_count);
        let mut fine_rice = Vec::with_capacity(BAND_COUNT * shift_count);
        let mut coarse_rice = Vec::with_capacity(BAND_COUNT * shift_count);
        for band in 0..BAND_COUNT {
            for shift in 0..=max_shift {
                let spec = if rtl_profile && shift > 0 {
                    QuantSpec::rtl(shift)
                } else {
                    QuantSpec::base(shift)
                };
                let state_points = build_points(corpus, band, spec, coder);
                fine_rice.push(best_entropy_parameter(&state_points, false));
                coarse_rice.push(best_entropy_parameter(&state_points, true));
                points.push(state_points);
            }
        }
        Self {
            max_shift,
            rtl_profile,
            points,
            fine_rice,
            coarse_rice,
        }
    }

    fn index(&self, band: usize, shift: u8) -> usize {
        band * (usize::from(self.max_shift) + 1) + usize::from(shift)
    }

    fn points(&self, band: usize, shift: u8) -> &[Point] {
        &self.points[self.index(band, shift)]
    }

    fn rice(&self, band: usize, shift: u8, coarse: bool) -> u8 {
        let index = self.index(band, shift);
        if coarse {
            self.coarse_rice[index]
        } else {
            self.fine_rice[index]
        }
    }

    fn spec(&self, shift: u8) -> QuantSpec {
        if self.rtl_profile {
            QuantSpec::rtl(shift)
        } else {
            QuantSpec::base(shift)
        }
    }
}

#[derive(Clone)]
struct LadderState {
    spec: QuantSpec,
    entropy_parameter: u8,
    points: Vec<Point>,
}

struct LadderModel {
    bands: [Vec<LadderState>; BAND_COUNT],
    header_extra: u8,
}

fn ladder_header_extra(levels: usize) -> u8 {
    let state_bits = usize::BITS - (levels - 1).leading_zeros();
    (state_bits - 1) as u8
}

fn ladder_packet_bits(point: Point, entropy_parameter: u8, header_extra: u8) -> u8 {
    let bits = point.bits[usize::from(entropy_parameter)];
    if bits == 1 {
        1
    } else {
        bits.saturating_add(header_extra)
    }
}

fn build_consecutive_ladder(
    corpus: &Corpus,
    base: &BaseCensus,
    weights: &[u32; BAND_COUNT],
    levels: usize,
    lambda: f64,
) -> LadderModel {
    let header_extra = ladder_header_extra(levels);
    let bands = array::from_fn(|band| {
        let minimum_shift = u8::from(base.rtl_profile);
        let maximum_start = base.max_shift + 1 - levels as u8;
        let mut best = None::<(f64, u8)>;
        for start in minimum_shift..=maximum_start {
            let mut cost = 0.0;
            for (local, &entry) in corpus.band_entries[band].iter().enumerate() {
                let weight = f64::from(weights[band]);
                let mut group_cost = weight * zero_distortion(corpus.source(entry)) as f64 + lambda;
                for offset in 0..levels {
                    let shift = start + offset as u8;
                    let point = base.points(band, shift)[local];
                    let bits =
                        ladder_packet_bits(point, base.rice(band, shift, false), header_extra);
                    group_cost =
                        group_cost.min(weight * point.distortion as f64 + lambda * f64::from(bits));
                }
                cost += group_cost;
            }
            if best.is_none_or(|current| cost < current.0) {
                best = Some((cost, start));
            }
        }
        let start = best.expect("at least one consecutive ladder").1;
        (0..levels)
            .map(|offset| {
                let shift = start + offset as u8;
                LadderState {
                    spec: base.spec(shift),
                    entropy_parameter: base.rice(band, shift, false),
                    points: base.points(band, shift).to_vec(),
                }
            })
            .collect()
    });
    LadderModel {
        bands,
        header_extra,
    }
}

#[derive(Clone)]
struct LadderSelection {
    states: Vec<u8>,
    bits: u64,
    weighted_distortion: f64,
}

fn select_ladder_image(
    corpus: &Corpus,
    model: &LadderModel,
    weights: &[u32; BAND_COUNT],
    image: usize,
    lambda: f64,
) -> LadderSelection {
    let mut states = Vec::with_capacity(GROUP_COUNT);
    let mut bits = 0u64;
    let mut weighted_distortion = 0.0;
    for group in 0..GROUP_COUNT {
        let band = usize::from(corpus.schedule[group]);
        let local = corpus.local_point_index(image, group, band);
        let weight = f64::from(weights[band]);
        let zero_distortion = zero_distortion(&corpus.images[image].groups[group]) as f64;
        let mut best = (weight * zero_distortion + lambda, 0u8, 1u8, zero_distortion);
        for (state_index, state) in model.bands[band].iter().enumerate() {
            let point = state.points[local];
            let state_bits = ladder_packet_bits(point, state.entropy_parameter, model.header_extra);
            let candidate = (
                weight * point.distortion as f64 + lambda * f64::from(state_bits),
                (state_index + 1) as u8,
                state_bits,
                point.distortion as f64,
            );
            if candidate.0 < best.0 || (candidate.0 == best.0 && candidate.2 < best.2) {
                best = candidate;
            }
        }
        states.push(best.1);
        bits += u64::from(best.2);
        weighted_distortion += weight * best.3;
    }
    LadderSelection {
        states,
        bits,
        weighted_distortion,
    }
}

fn select_ladder_all(
    corpus: &Corpus,
    model: &LadderModel,
    weights: &[u32; BAND_COUNT],
    lambda: f64,
) -> LadderSelection {
    let mut states = Vec::with_capacity(corpus.images.len() * GROUP_COUNT);
    let mut bits = 0u64;
    let mut weighted_distortion = 0.0;
    for image in 0..corpus.images.len() {
        let selection = select_ladder_image(corpus, model, weights, image, lambda);
        states.extend(selection.states);
        bits += selection.bits;
        weighted_distortion += selection.weighted_distortion;
    }
    LadderSelection {
        states,
        bits,
        weighted_distortion,
    }
}

fn tune_ladder_lambda(
    corpus: &Corpus,
    model: &LadderModel,
    weights: &[u32; BAND_COUNT],
    target_bits: u64,
    initial: f64,
) -> f64 {
    let mut low = 0.0;
    let mut high = initial.max(1.0);
    while select_ladder_all(corpus, model, weights, high).bits > target_bits {
        low = high;
        high *= 2.0;
    }
    let mut closest = high;
    let mut closest_error = u64::MAX;
    for _ in 0..20 {
        let lambda = (low + high) * 0.5;
        let selection = select_ladder_all(corpus, model, weights, lambda);
        let error = selection.bits.abs_diff(target_bits);
        if error < closest_error {
            closest = lambda;
            closest_error = error;
        }
        if selection.bits > target_bits {
            low = lambda;
        } else {
            high = lambda;
        }
    }
    closest
}

fn capacity_limited_ladder_selection(
    corpus: &Corpus,
    model: &LadderModel,
    weights: &[u32; BAND_COUNT],
    image: usize,
    initial: f64,
) -> LadderSelection {
    let mut low = 0.0;
    let mut high = initial.max(1.0);
    let mut best = select_ladder_image(corpus, model, weights, image, high);
    while best.bits > FRAME_BITS as u64 {
        low = high;
        high *= 2.0;
        best = select_ladder_image(corpus, model, weights, image, high);
    }
    for _ in 0..28 {
        let lambda = (low + high) * 0.5;
        let selection = select_ladder_image(corpus, model, weights, image, lambda);
        if selection.bits > FRAME_BITS as u64 {
            low = lambda;
        } else {
            high = lambda;
            if selection.weighted_distortion < best.weighted_distortion {
                best = selection;
            }
        }
    }
    best
}

fn fit_ladder_affine(
    corpus: &Corpus,
    model: &LadderModel,
    states: &[u8],
) -> (Vec<[u32; BAND_COUNT]>, Vec<[i32; BAND_COUNT]>) {
    let levels = model.bands[0].len();
    let mut count = vec![[0i128; BAND_COUNT]; levels];
    let mut sum_x = vec![[0i128; BAND_COUNT]; levels];
    let mut sum_y = vec![[0i128; BAND_COUNT]; levels];
    let mut sum_xx = vec![[0i128; BAND_COUNT]; levels];
    let mut sum_xy = vec![[0i128; BAND_COUNT]; levels];
    for image in 0..corpus.images.len() {
        for group in 0..GROUP_COUNT {
            let selected = usize::from(states[image * GROUP_COUNT + group]);
            if selected == 0 {
                continue;
            }
            let state_index = selected - 1;
            let band = usize::from(corpus.schedule[group]);
            let spec = model.bands[band][state_index].spec;
            for &coefficient in &corpus.images[image].groups[group] {
                let q = quantize(coefficient, spec);
                if q == 0 {
                    continue;
                }
                let x = i128::from(q.unsigned_abs());
                let y = i128::from(coefficient.unsigned_abs());
                count[state_index][band] += 1;
                sum_x[state_index][band] += x;
                sum_y[state_index][band] += y;
                sum_xx[state_index][band] += x * x;
                sum_xy[state_index][band] += x * y;
            }
        }
    }
    let mut gains = vec![[0u32; BAND_COUNT]; levels];
    let mut offsets = vec![[0i32; BAND_COUNT]; levels];
    for state_index in 0..levels {
        for band in 0..BAND_COUNT {
            let spec = model.bands[band][state_index].spec;
            gains[state_index][band] = 256u32 << spec.shift;
            let n = count[state_index][band];
            let denominator = n * sum_xx[state_index][band] - sum_x[state_index][band].pow(2);
            if denominator == 0 {
                continue;
            }
            let gain = rounded_divide(
                (n * sum_xy[state_index][band]
                    - sum_x[state_index][band] * sum_y[state_index][band])
                    * 256,
                denominator,
            )
            .clamp(0, i128::from(u32::MAX));
            let offset = rounded_divide(
                sum_y[state_index][band] * 256 - gain * sum_x[state_index][band],
                n,
            )
            .clamp(i128::from(i32::MIN), i128::from(i32::MAX));
            gains[state_index][band] = gain as u32;
            offsets[state_index][band] = offset as i32;
        }
    }
    (gains, offsets)
}

fn exact_evaluate_ladder(
    corpus: &Corpus,
    model: &LadderModel,
    states: &[u8],
    gains_q8: &[[u32; BAND_COUNT]],
    offsets_q8: &[[i32; BAND_COUNT]],
) -> ExactEvaluation {
    assert_eq!(states.len(), corpus.images.len() * GROUP_COUNT);
    let band_list = bands(WIDTH, HEIGHT);
    let (sender, receiver) = mpsc::channel();
    std::thread::scope(|scope| {
        for image in 0..corpus.images.len() {
            let sender = sender.clone();
            let image_states = &states[image * GROUP_COUNT..(image + 1) * GROUP_COUNT];
            let band_list = &band_list;
            scope.spawn(move || {
                let cached = &corpus.images[image];
                let mut coefficients = vec![0i32; PIXELS];
                let mut band_groups = [0usize; BAND_COUNT];
                let mut nonzero = 0usize;
                for (group, &selected) in image_states.iter().enumerate() {
                    let band = usize::from(corpus.schedule[group]);
                    let ordinal = band_groups[band];
                    band_groups[band] += 1;
                    if selected == 0 {
                        continue;
                    }
                    let state_index = usize::from(selected - 1);
                    let spec = model.bands[band][state_index].spec;
                    let rect = band_list[band].rect;
                    let mut group_nonzero = false;
                    for lane in 0..GROUP_LANES {
                        let flat = ordinal * GROUP_LANES + lane;
                        let index =
                            (rect.y + flat / rect.width) * WIDTH + rect.x + flat % rect.width;
                        let quantized = quantize(cached.groups[group][lane], spec);
                        group_nonzero |= quantized != 0;
                        coefficients[index] = reconstruct_with_affine(
                            quantized,
                            gains_q8[state_index][band],
                            offsets_q8[state_index][band],
                        );
                    }
                    nonzero += usize::from(group_nonzero);
                }
                inverse_transform(&mut coefficients, WIDTH, HEIGHT);
                let sse = cached
                    .pixels
                    .iter()
                    .zip(coefficients)
                    .map(|(&original, reconstructed)| {
                        let error = i64::from(original) - i64::from(reconstructed.clamp(0, 255));
                        (error * error) as u64
                    })
                    .sum::<u64>();
                sender.send((image, sse, nonzero)).unwrap();
            });
        }
        drop(sender);
    });
    let mut results = receiver.into_iter().collect::<Vec<_>>();
    results.sort_by_key(|&(image, _, _)| image);
    let total_sse = results.iter().map(|&(_, sse, _)| u128::from(sse)).sum();
    let image_psnr = results
        .iter()
        .map(|&(image, sse, _)| {
            (
                corpus.images[image].name.clone(),
                psnr(sse as f64 / PIXELS as f64),
                sse,
            )
        })
        .collect::<Vec<_>>();
    ExactEvaluation {
        total_sse,
        aggregate_psnr: psnr(total_sse as f64 / (corpus.images.len() * PIXELS) as f64),
        mean_psnr: image_psnr.iter().map(|(_, value, _)| value).sum::<f64>()
            / image_psnr.len() as f64,
        image_psnr,
        max_nonzero_groups: results
            .iter()
            .map(|&(_, _, nonzero)| nonzero)
            .max()
            .unwrap_or(0),
    }
}

fn run_ladder_probe(
    corpus: &Corpus,
    base: &BaseCensus,
    weights: &[u32; BAND_COUNT],
    levels: usize,
    started: Instant,
) {
    let target_bits = (corpus.images.len() * FRAME_BITS) as u64;
    let mut lambda = tune_unrestricted_lambda(corpus, base, weights, target_bits);
    let mut model = build_consecutive_ladder(corpus, base, weights, levels, lambda);
    for _ in 0..2 {
        lambda = tune_ladder_lambda(corpus, &model, weights, target_bits, lambda);
        model = build_consecutive_ladder(corpus, base, weights, levels, lambda);
    }
    lambda = tune_ladder_lambda(corpus, &model, weights, target_bits, lambda);

    let mut states = Vec::with_capacity(corpus.images.len() * GROUP_COUNT);
    let mut frame_bits = Vec::with_capacity(corpus.images.len());
    for image in 0..corpus.images.len() {
        let selection = capacity_limited_ladder_selection(corpus, &model, weights, image, lambda);
        frame_bits.push(selection.bits);
        states.extend(selection.states);
    }
    let (gains, offsets) = fit_ladder_affine(corpus, &model, &states);
    let exact = exact_evaluate_ladder(corpus, &model, &states, &gains, &offsets);
    let shifts = array::from_fn::<_, BAND_COUNT, _>(|band| {
        model.bands[band]
            .iter()
            .map(|state| state.spec.shift)
            .collect::<Vec<_>>()
    });
    let rice = array::from_fn::<_, BAND_COUNT, _>(|band| {
        model.bands[band]
            .iter()
            .map(|state| state.entropy_parameter)
            .collect::<Vec<_>>()
    });
    println!(
        "ladder levels={} header_extra={} lambda={lambda:.9} shifts={shifts:?} rice={rice:?}",
        levels, model.header_extra,
    );
    for (image, ((name, image_psnr, sse), bits)) in
        exact.image_psnr.iter().zip(frame_bits.iter()).enumerate()
    {
        println!(
            "ladder image={} name={} psnr_db={image_psnr:.9} sse={} bits={}",
            image + 1,
            name,
            sse,
            bits,
        );
    }
    println!(
        "ladder aggregate_psnr_db={:.15} mean_psnr_db={:.9} total_sse={} bits_min={} bits_max={} max_nonzero_groups={} elapsed_s={:.3}",
        exact.aggregate_psnr,
        exact.mean_psnr,
        exact.total_sse,
        frame_bits.iter().min().copied().unwrap_or(0),
        frame_bits.iter().max().copied().unwrap_or(0),
        exact.max_nonzero_groups,
        started.elapsed().as_secs_f64(),
    );
}

#[derive(Clone)]
struct QuantState {
    spec: QuantSpec,
    entropy_parameter: u8,
    points: Vec<Point>,
}

#[derive(Clone)]
struct BandModel {
    fine: QuantState,
    coarse: QuantState,
}

#[derive(Clone)]
struct ProfileModel {
    bands: [BandModel; BAND_COUNT],
}

impl ProfileModel {
    fn canonical(corpus: &Corpus, coder: &impl EntropyCoder) -> Self {
        Self {
            bands: array::from_fn(|band| {
                let fine_spec = QuantSpec::rtl_for_band(CANONICAL_FINE_SHIFT[band], band, false);
                let coarse_spec = QuantSpec::rtl_for_band(CANONICAL_COARSE_SHIFT[band], band, true);
                BandModel {
                    fine: QuantState {
                        spec: fine_spec,
                        entropy_parameter: CANONICAL_FINE_RICE[band],
                        points: build_points(corpus, band, fine_spec, coder),
                    },
                    coarse: QuantState {
                        spec: coarse_spec,
                        entropy_parameter: CANONICAL_COARSE_RICE[band],
                        points: build_points(corpus, band, coarse_spec, coder),
                    },
                }
            }),
        }
    }

    fn best_two_level(
        corpus: &Corpus,
        base: &BaseCensus,
        _coder: &impl EntropyCoder,
        weights: &[u32; BAND_COUNT],
        lambda: f64,
    ) -> Self {
        let bands = array::from_fn(|band| {
            let mut best = None::<(f64, u8, u8)>;
            let minimum_shift = u8::from(base.rtl_profile);
            for fine_shift in minimum_shift..base.max_shift {
                for coarse_shift in fine_shift + 1..=base.max_shift {
                    if base.rtl_profile && coarse_shift - fine_shift > 3 {
                        continue;
                    }
                    let fine = base.points(band, fine_shift);
                    let coarse = base.points(band, coarse_shift);
                    let fine_k = base.rice(band, fine_shift, false);
                    let coarse_k = base.rice(band, coarse_shift, true);
                    let mut cost = 0.0;
                    for (local, &entry) in corpus.band_entries[band].iter().enumerate() {
                        let weight = f64::from(weights[band]);
                        let zero = weight * zero_distortion(corpus.source(entry)) as f64 + lambda;
                        let fine_cost = weight * fine[local].distortion as f64
                            + lambda * f64::from(state_packet_bits(fine[local], fine_k, false));
                        let coarse_cost = weight * coarse[local].distortion as f64
                            + lambda * f64::from(state_packet_bits(coarse[local], coarse_k, true));
                        cost += zero.min(fine_cost).min(coarse_cost);
                    }
                    if best.is_none_or(|current| cost < current.0) {
                        best = Some((cost, fine_shift, coarse_shift));
                    }
                }
            }
            let (_, fine_shift, coarse_shift) = best.expect("there is at least one shift pair");
            BandModel {
                fine: QuantState {
                    spec: base.spec(fine_shift),
                    entropy_parameter: base.rice(band, fine_shift, false),
                    points: base.points(band, fine_shift).to_vec(),
                },
                coarse: QuantState {
                    spec: base.spec(coarse_shift),
                    entropy_parameter: base.rice(band, coarse_shift, true),
                    points: base.points(band, coarse_shift).to_vec(),
                },
            }
        });
        Self { bands }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct RiceContextProfile {
    zero_k: u8,
    small_k: u8,
    large_k: u8,
    threshold_exp: u8,
}

impl RiceContextProfile {
    fn fixed(k: u8) -> Self {
        Self {
            zero_k: k,
            small_k: k,
            large_k: k,
            threshold_exp: k.min(3),
        }
    }

    fn k(self, previous_magnitude: u16) -> u8 {
        if previous_magnitude == 0 {
            self.zero_k
        } else if previous_magnitude > (1u16 << self.threshold_exp) {
            self.large_k
        } else {
            self.small_k
        }
    }
}

struct RiceContextCensus {
    values: [[Vec<[i16; GROUP_LANES]>; BAND_COUNT]; 2],
}

impl RiceContextCensus {
    fn build(corpus: &Corpus, model: &ProfileModel) -> Self {
        Self {
            values: array::from_fn(|quality| {
                array::from_fn(|band| {
                    let spec = if quality == 0 {
                        model.bands[band].fine.spec
                    } else {
                        model.bands[band].coarse.spec
                    };
                    corpus.band_entries[band]
                        .iter()
                        .map(|&entry| corpus.source(entry).map(|value| quantize(value, spec)))
                        .collect()
                })
            }),
        }
    }
}

fn context_packet_bits(
    values: &[i16; GROUP_LANES],
    profile: RiceContextProfile,
    coarse: bool,
) -> u8 {
    let mask = lane_mask(values);
    if mask == 0 {
        return 1;
    }
    let mut previous_magnitude = 0u16;
    let mut coefficient_bits = 0usize;
    let mut pair_bits = 0usize;
    for (lane, &value) in values.iter().enumerate() {
        let length = coefficient_rice_bits(value, profile.k(previous_magnitude));
        coefficient_bits += length;
        pair_bits += length;
        previous_magnitude = value.unsigned_abs();
        if lane & 1 != 0 {
            if pair_bits > MAX_PAIR_TOKEN_BITS {
                return if coarse {
                    COARSE_RAW_PACKET_BITS
                } else {
                    RAW_PACKET_BITS
                };
            }
            pair_bits = 0;
        }
    }
    let mask_bits = (0..4)
        .map(|nibble| NIBBLE_HUFFMAN_LENGTH[usize::from((mask >> (4 * nibble)) & 15)])
        .sum::<usize>()
        .min(16);
    let normal = 3 + mask_bits + coefficient_bits;
    if normal >= usize::from(RAW_PACKET_BITS) {
        if coarse {
            COARSE_RAW_PACKET_BITS
        } else {
            RAW_PACKET_BITS
        }
    } else {
        normal as u8
    }
}

fn context_profile_bits(
    values: &[[i16; GROUP_LANES]],
    profile: RiceContextProfile,
    coarse: bool,
) -> u64 {
    values
        .iter()
        .map(|values| u64::from(context_packet_bits(values, profile, coarse)))
        .sum()
}

fn fixed_context_profiles() -> [[RiceContextProfile; BAND_COUNT]; 2] {
    [
        CANONICAL_FINE_RICE.map(RiceContextProfile::fixed),
        CANONICAL_COARSE_RICE.map(RiceContextProfile::fixed),
    ]
}

fn canonical_context_profiles() -> [[RiceContextProfile; BAND_COUNT]; 2] {
    [
        CANONICAL_FINE_RICE.map(RiceContextProfile::fixed),
        array::from_fn(|band| RiceContextProfile {
            zero_k: CANONICAL_COARSE_RICE_ZERO[band],
            small_k: CANONICAL_COARSE_RICE_SMALL[band],
            large_k: CANONICAL_COARSE_RICE_LARGE[band],
            threshold_exp: CANONICAL_COARSE_RICE_THRESHOLD_EXP[band],
        }),
    ]
}

fn proxy_tune_context_profiles(
    census: &RiceContextCensus,
) -> [[RiceContextProfile; BAND_COUNT]; 2] {
    let mut profiles = fixed_context_profiles();
    let quality = 1;
    for (band, profile) in profiles[quality].iter_mut().enumerate() {
        let values = &census.values[quality][band];
        let coarse = true;
        let mut best = *profile;
        let mut best_bits = context_profile_bits(values, best, coarse);
        for _ in 0..2 {
            for parameter in 0..4 {
                let limit = if parameter == 3 { 2 } else { 4 };
                let base = best;
                for value in 0..=limit {
                    let mut candidate = base;
                    match parameter {
                        0 => candidate.zero_k = value,
                        1 => candidate.small_k = value,
                        2 => candidate.large_k = value,
                        3 => candidate.threshold_exp = value,
                        _ => unreachable!(),
                    }
                    let bits = context_profile_bits(values, candidate, coarse);
                    if (bits, value)
                        < (
                            best_bits,
                            match parameter {
                                0 => best.zero_k,
                                1 => best.small_k,
                                2 => best.large_k,
                                3 => best.threshold_exp,
                                _ => unreachable!(),
                            },
                        )
                    {
                        best = candidate;
                        best_bits = bits;
                    }
                }
            }
        }
        *profile = best;
        println!(
            "context_proxy quality={} band={} profile={best:?} bits={best_bits}",
            if coarse { "coarse" } else { "fine" },
            BAND_NAMES[band],
        );
    }
    profiles
}

fn apply_context_profile(
    model: &mut ProfileModel,
    census: &RiceContextCensus,
    quality: usize,
    band: usize,
    profile: RiceContextProfile,
) {
    let state = if quality == 0 {
        &mut model.bands[band].fine
    } else {
        &mut model.bands[band].coarse
    };
    state.entropy_parameter = 0;
    for (point, values) in state.points.iter_mut().zip(&census.values[quality][band]) {
        point.bits[0] = context_packet_bits(values, profile, quality == 1);
    }
}

fn apply_context_profiles(
    model: &mut ProfileModel,
    census: &RiceContextCensus,
    profiles: &[[RiceContextProfile; BAND_COUNT]; 2],
) {
    let quality = 1;
    for (band, &profile) in profiles[quality].iter().enumerate() {
        apply_context_profile(model, census, quality, band, profile);
    }
}

const MIXED_PAIR_TOKEN_BITS: usize = 20;
const MIXED_RAW_PACKET_BITS: u16 = 166;

#[derive(Clone, Copy)]
struct MixedLaneOption {
    state: u8,
    bits: u16,
    distortion: u64,
}

#[derive(Clone, Copy)]
struct MixedLaneOptions {
    zero: MixedLaneOption,
    nonzero: Option<MixedLaneOption>,
}

#[derive(Clone, Copy)]
struct MixedNibbleChoice {
    states: [u8; 4],
    bits: u16,
    distortion: u64,
}

#[derive(Clone)]
struct MixedGroupChoice {
    states: [u8; GROUP_LANES],
    bits: u16,
    distortion: u64,
}

fn mixed_cost(distortion: u64, bits: u16, weight: u32, lambda: f64) -> f64 {
    distortion as f64 * f64::from(weight) + f64::from(bits) * lambda
}

fn mixed_lane_options(
    source: i32,
    fine: &QuantState,
    coarse: &QuantState,
    weight: u32,
    lambda: f64,
) -> MixedLaneOptions {
    let zero = MixedLaneOption {
        state: 0,
        bits: 0,
        distortion: i64::from(source).unsigned_abs().pow(2),
    };
    let mut nonzero = None::<MixedLaneOption>;
    for (state_index, state) in [(1u8, fine), (2u8, coarse)] {
        let q = quantize(source, state.spec);
        if q == 0 {
            continue;
        }
        let error = i64::from(source) - i64::from(reconstruct(q, state.spec));
        let option = MixedLaneOption {
            state: state_index,
            bits: (1 + coefficient_rice_bits(q, state.entropy_parameter)) as u16,
            distortion: error.unsigned_abs().pow(2),
        };
        if nonzero.is_none_or(|current| {
            mixed_cost(option.distortion, option.bits, weight, lambda)
                < mixed_cost(current.distortion, current.bits, weight, lambda)
        }) {
            nonzero = Some(option);
        }
    }
    MixedLaneOptions { zero, nonzero }
}

fn mixed_nibble_choices(
    lanes: &[MixedLaneOptions],
    weight: u32,
    lambda: f64,
) -> (MixedNibbleChoice, Option<MixedNibbleChoice>) {
    debug_assert_eq!(lanes.len(), 4);
    let zero = MixedNibbleChoice {
        states: [0; 4],
        bits: 1,
        distortion: lanes.iter().map(|lane| lane.zero.distortion).sum(),
    };
    if lanes.iter().all(|lane| lane.nonzero.is_none()) {
        return (zero, None);
    }

    let mut states = [0u8; 4];
    let mut bits = 5u16;
    let mut distortion = 0u64;
    let mut selected_nonzero = false;
    for (lane_index, lane) in lanes.iter().enumerate() {
        let selected = lane.nonzero.filter(|candidate| {
            mixed_cost(candidate.distortion, candidate.bits, weight, lambda)
                < mixed_cost(lane.zero.distortion, lane.zero.bits, weight, lambda)
        });
        if let Some(selected) = selected {
            states[lane_index] = selected.state;
            bits += selected.bits;
            distortion += selected.distortion;
            selected_nonzero = true;
        } else {
            distortion += lane.zero.distortion;
        }
    }
    if !selected_nonzero {
        let forced = lanes
            .iter()
            .enumerate()
            .filter_map(|(lane_index, lane)| {
                lane.nonzero.map(|candidate| {
                    let penalty = mixed_cost(candidate.distortion, candidate.bits, weight, lambda)
                        - mixed_cost(lane.zero.distortion, lane.zero.bits, weight, lambda);
                    (penalty, lane_index, candidate)
                })
            })
            .min_by(|left, right| left.0.total_cmp(&right.0))
            .expect("a nonzero nibble candidate exists");
        states[forced.1] = forced.2.state;
        bits += forced.2.bits;
        distortion = distortion - lanes[forced.1].zero.distortion + forced.2.distortion;
    }
    (
        zero,
        Some(MixedNibbleChoice {
            states,
            bits,
            distortion,
        }),
    )
}

fn bound_mixed_group(
    source: &[i32; GROUP_LANES],
    band_model: &BandModel,
    mut choice: MixedGroupChoice,
) -> MixedGroupChoice {
    let mut pair_overflow = false;
    for pair in 0..GROUP_LANES / 2 {
        let mut pair_bits = 0usize;
        for (lane, &selected) in choice.states.iter().enumerate().skip(2 * pair).take(2) {
            let state = match selected {
                0 => continue,
                1 => &band_model.fine,
                2 => &band_model.coarse,
                _ => unreachable!("mixed state is two bits"),
            };
            let q = quantize(source[lane], state.spec);
            pair_bits += 1 + coefficient_rice_bits(q, state.entropy_parameter);
        }
        pair_overflow |= pair_bits > MIXED_PAIR_TOKEN_BITS;
    }
    if pair_overflow || choice.bits > MIXED_RAW_PACKET_BITS {
        choice.bits = MIXED_RAW_PACKET_BITS;
    }
    choice
}

fn choose_mixed_group(
    source: &[i32; GROUP_LANES],
    band_model: &BandModel,
    weight: u32,
    lambda: f64,
) -> MixedGroupChoice {
    let options = array::from_fn::<_, GROUP_LANES, _>(|lane| {
        mixed_lane_options(
            source[lane],
            &band_model.fine,
            &band_model.coarse,
            weight,
            lambda,
        )
    });
    let zero_distortion = options.iter().map(|lane| lane.zero.distortion).sum();
    let zero = MixedGroupChoice {
        states: [0; GROUP_LANES],
        bits: 1,
        distortion: zero_distortion,
    };

    let nibble_options = array::from_fn::<_, 4, _>(|nibble| {
        mixed_nibble_choices(&options[4 * nibble..4 * nibble + 4], weight, lambda)
    });
    let mut prefix_states = [0u8; GROUP_LANES];
    let mut prefix_bits = 2u16;
    let mut prefix_distortion = 0u64;
    let mut prefix_nonzero = false;
    let mut prefix_selected_nonzero = [false; 4];
    for nibble in 0..4 {
        let (zero_nibble, nonzero_nibble) = nibble_options[nibble];
        let selected = nonzero_nibble.filter(|candidate| {
            mixed_cost(candidate.distortion, candidate.bits, weight, lambda)
                < mixed_cost(zero_nibble.distortion, zero_nibble.bits, weight, lambda)
        });
        let selected = selected.unwrap_or(zero_nibble);
        prefix_states[4 * nibble..4 * nibble + 4].copy_from_slice(&selected.states);
        prefix_bits += selected.bits;
        prefix_distortion += selected.distortion;
        prefix_selected_nonzero[nibble] = selected.states.iter().any(|&state| state != 0);
        prefix_nonzero |= prefix_selected_nonzero[nibble];
    }
    if !prefix_nonzero {
        let forced = nibble_options
            .iter()
            .enumerate()
            .filter_map(|(nibble, &(zero_nibble, nonzero_nibble))| {
                nonzero_nibble.map(|candidate| {
                    let penalty = mixed_cost(candidate.distortion, candidate.bits, weight, lambda)
                        - mixed_cost(zero_nibble.distortion, zero_nibble.bits, weight, lambda);
                    (penalty, nibble, zero_nibble, candidate)
                })
            })
            .min_by(|left, right| left.0.total_cmp(&right.0));
        if let Some((_, nibble, zero_nibble, candidate)) = forced {
            prefix_states[4 * nibble..4 * nibble + 4].copy_from_slice(&candidate.states);
            prefix_bits = prefix_bits - zero_nibble.bits + candidate.bits;
            prefix_distortion = prefix_distortion - zero_nibble.distortion + candidate.distortion;
        }
    }
    let prefix = bound_mixed_group(
        source,
        band_model,
        MixedGroupChoice {
            states: prefix_states,
            bits: prefix_bits,
            distortion: prefix_distortion,
        },
    );

    let mut literal_states = [0u8; GROUP_LANES];
    let mut literal_bits = 18u16;
    let mut literal_distortion = 0u64;
    let mut literal_nonzero = false;
    for (lane_index, lane) in options.iter().enumerate() {
        let selected = lane.nonzero.filter(|candidate| {
            mixed_cost(candidate.distortion, candidate.bits, weight, lambda)
                < mixed_cost(lane.zero.distortion, lane.zero.bits, weight, lambda)
        });
        if let Some(selected) = selected {
            literal_states[lane_index] = selected.state;
            literal_bits += selected.bits;
            literal_distortion += selected.distortion;
            literal_nonzero = true;
        } else {
            literal_distortion += lane.zero.distortion;
        }
    }
    if !literal_nonzero {
        let forced = options
            .iter()
            .enumerate()
            .filter_map(|(lane_index, lane)| {
                lane.nonzero.map(|candidate| {
                    let penalty = mixed_cost(candidate.distortion, candidate.bits, weight, lambda)
                        - mixed_cost(lane.zero.distortion, lane.zero.bits, weight, lambda);
                    (penalty, lane_index, candidate)
                })
            })
            .min_by(|left, right| left.0.total_cmp(&right.0));
        if let Some((_, lane_index, candidate)) = forced {
            literal_states[lane_index] = candidate.state;
            literal_bits += candidate.bits;
            literal_distortion =
                literal_distortion - options[lane_index].zero.distortion + candidate.distortion;
        }
    }
    let literal = bound_mixed_group(
        source,
        band_model,
        MixedGroupChoice {
            states: literal_states,
            bits: literal_bits,
            distortion: literal_distortion,
        },
    );

    [zero, prefix, literal]
        .into_iter()
        .min_by(|left, right| {
            mixed_cost(left.distortion, left.bits, weight, lambda).total_cmp(&mixed_cost(
                right.distortion,
                right.bits,
                weight,
                lambda,
            ))
        })
        .expect("mixed group has three syntax choices")
}

struct MixedSelection {
    states: Vec<u8>,
    bits: u64,
    weighted_distortion: f64,
    raw_groups: usize,
}

fn select_mixed_image(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    image: usize,
    lambda: f64,
) -> MixedSelection {
    let mut states = Vec::with_capacity(PIXELS);
    let mut bits = 0u64;
    let mut weighted_distortion = 0.0;
    let mut raw_groups = 0usize;
    for group in 0..GROUP_COUNT {
        let band = usize::from(corpus.schedule[group]);
        let choice = choose_mixed_group(
            &corpus.images[image].groups[group],
            &model.bands[band],
            weights[band],
            lambda,
        );
        states.extend(choice.states);
        bits += u64::from(choice.bits);
        weighted_distortion += choice.distortion as f64 * f64::from(weights[band]);
        raw_groups += usize::from(choice.bits == MIXED_RAW_PACKET_BITS);
    }
    MixedSelection {
        states,
        bits,
        weighted_distortion,
        raw_groups,
    }
}

fn capacity_limited_mixed_selection(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    image: usize,
    initial: f64,
) -> MixedSelection {
    let mut low = 0.0;
    let mut high = initial.max(1.0);
    let mut best = select_mixed_image(corpus, model, weights, image, high);
    while best.bits > FRAME_BITS as u64 {
        low = high;
        high *= 2.0;
        best = select_mixed_image(corpus, model, weights, image, high);
    }
    for _ in 0..22 {
        let lambda = (low + high) * 0.5;
        let selection = select_mixed_image(corpus, model, weights, image, lambda);
        if selection.bits > FRAME_BITS as u64 {
            low = lambda;
        } else {
            high = lambda;
            if selection.weighted_distortion < best.weighted_distortion {
                best = selection;
            }
        }
    }
    best
}

const MIXED_PAIR_RAW_PACKET_BITS: u16 = 150;

#[derive(Clone, Copy)]
struct PairQualityOption {
    states: [u8; 2],
    bits: u16,
    distortion: u64,
}

fn pair_quality_options(source: &[i32], band_model: &BandModel) -> [Option<PairQualityOption>; 3] {
    debug_assert_eq!(source.len(), 2);
    let zero = PairQualityOption {
        states: [0; 2],
        bits: 0,
        distortion: source
            .iter()
            .map(|&value| i64::from(value).unsigned_abs().pow(2))
            .sum(),
    };
    let mut result = [Some(zero), None, None];
    for (quality, state) in [(1usize, &band_model.fine), (2, &band_model.coarse)] {
        let values = [
            quantize(source[0], state.spec),
            quantize(source[1], state.spec),
        ];
        if values == [0, 0] {
            continue;
        }
        let mut bits = 1u16;
        let mut distortion = 0u64;
        let mut states = [0u8; 2];
        for lane in 0..2 {
            let error = i64::from(source[lane]) - i64::from(reconstruct(values[lane], state.spec));
            distortion += error.unsigned_abs().pow(2);
            if values[lane] != 0 {
                states[lane] = quality as u8;
                bits += coefficient_rice_bits(values[lane], state.entropy_parameter) as u16;
            }
        }
        result[quality] = Some(PairQualityOption {
            states,
            bits,
            distortion,
        });
    }
    result
}

fn pair_nibble_choices(
    first: [Option<PairQualityOption>; 3],
    second: [Option<PairQualityOption>; 3],
    weight: u32,
    lambda: f64,
) -> (MixedNibbleChoice, Option<MixedNibbleChoice>) {
    let zero_first = first[0].expect("zero pair exists");
    let zero_second = second[0].expect("zero pair exists");
    let zero = MixedNibbleChoice {
        states: [0; 4],
        bits: 1,
        distortion: zero_first.distortion + zero_second.distortion,
    };
    let mut nonzero = None::<MixedNibbleChoice>;
    for first_quality in 0..3 {
        for second_quality in 0..3 {
            if first_quality == 0 && second_quality == 0 {
                continue;
            }
            let Some(first) = first[first_quality] else {
                continue;
            };
            let Some(second) = second[second_quality] else {
                continue;
            };
            let candidate = MixedNibbleChoice {
                states: [
                    first.states[0],
                    first.states[1],
                    second.states[0],
                    second.states[1],
                ],
                bits: 5 + first.bits + second.bits,
                distortion: first.distortion + second.distortion,
            };
            if nonzero.is_none_or(|current| {
                mixed_cost(candidate.distortion, candidate.bits, weight, lambda)
                    < mixed_cost(current.distortion, current.bits, weight, lambda)
            }) {
                nonzero = Some(candidate);
            }
        }
    }
    (zero, nonzero)
}

fn bound_pair_group(
    source: &[i32; GROUP_LANES],
    band_model: &BandModel,
    mut choice: MixedGroupChoice,
) -> MixedGroupChoice {
    let mut pair_overflow = false;
    for pair in 0..GROUP_LANES / 2 {
        let first_lane = 2 * pair;
        let quality = choice.states[first_lane].max(choice.states[first_lane + 1]);
        if quality == 0 {
            continue;
        }
        let state = if quality == 1 {
            &band_model.fine
        } else {
            &band_model.coarse
        };
        let mut pair_bits = 1usize;
        for &coefficient in source.iter().skip(first_lane).take(2) {
            let q = quantize(coefficient, state.spec);
            if q != 0 {
                pair_bits += coefficient_rice_bits(q, state.entropy_parameter);
            }
        }
        pair_overflow |= pair_bits > MIXED_PAIR_TOKEN_BITS;
    }
    if pair_overflow || choice.bits > MIXED_PAIR_RAW_PACKET_BITS {
        choice.bits = MIXED_PAIR_RAW_PACKET_BITS;
    }
    choice
}

fn choose_mixed_pair_group(
    source: &[i32; GROUP_LANES],
    band_model: &BandModel,
    weight: u32,
    lambda: f64,
) -> MixedGroupChoice {
    let pairs = array::from_fn::<_, 8, _>(|pair| {
        pair_quality_options(&source[2 * pair..2 * pair + 2], band_model)
    });
    let zero_distortion = pairs
        .iter()
        .map(|pair| pair[0].expect("zero pair exists").distortion)
        .sum();
    let zero = MixedGroupChoice {
        states: [0; GROUP_LANES],
        bits: 1,
        distortion: zero_distortion,
    };
    let nibble_options = array::from_fn::<_, 4, _>(|nibble| {
        pair_nibble_choices(pairs[2 * nibble], pairs[2 * nibble + 1], weight, lambda)
    });

    let mut prefix_states = [0u8; GROUP_LANES];
    let mut prefix_bits = 2u16;
    let mut prefix_distortion = 0u64;
    let mut prefix_nonzero = false;
    for nibble in 0..4 {
        let (zero_nibble, nonzero_nibble) = nibble_options[nibble];
        let selected = nonzero_nibble.filter(|candidate| {
            mixed_cost(candidate.distortion, candidate.bits, weight, lambda)
                < mixed_cost(zero_nibble.distortion, zero_nibble.bits, weight, lambda)
        });
        let selected = selected.unwrap_or(zero_nibble);
        prefix_states[4 * nibble..4 * nibble + 4].copy_from_slice(&selected.states);
        prefix_bits += selected.bits;
        prefix_distortion += selected.distortion;
        prefix_nonzero |= selected.states.iter().any(|&state| state != 0);
    }
    if !prefix_nonzero
        && let Some((_, nibble, zero_nibble, candidate)) = nibble_options
            .iter()
            .enumerate()
            .filter_map(|(nibble, &(zero_nibble, nonzero_nibble))| {
                nonzero_nibble.map(|candidate| {
                    let penalty = mixed_cost(candidate.distortion, candidate.bits, weight, lambda)
                        - mixed_cost(zero_nibble.distortion, zero_nibble.bits, weight, lambda);
                    (penalty, nibble, zero_nibble, candidate)
                })
            })
            .min_by(|left, right| left.0.total_cmp(&right.0))
    {
        prefix_states[4 * nibble..4 * nibble + 4].copy_from_slice(&candidate.states);
        prefix_bits = prefix_bits - zero_nibble.bits + candidate.bits;
        prefix_distortion = prefix_distortion - zero_nibble.distortion + candidate.distortion;
    }
    let prefix = bound_pair_group(
        source,
        band_model,
        MixedGroupChoice {
            states: prefix_states,
            bits: prefix_bits,
            distortion: prefix_distortion,
        },
    );

    let mut literal_states = [0u8; GROUP_LANES];
    let mut literal_bits = 18u16;
    let mut literal_distortion = 0u64;
    let mut literal_nonzero = false;
    for pair in 0..8 {
        let chosen =
            (0..3)
                .filter_map(|quality| pairs[pair][quality].map(|candidate| (quality, candidate)))
                .min_by(|left, right| {
                    mixed_cost(left.1.distortion, left.1.bits, weight, lambda).total_cmp(
                        &mixed_cost(right.1.distortion, right.1.bits, weight, lambda),
                    )
                })
                .expect("zero pair exists");
        literal_states[2 * pair..2 * pair + 2].copy_from_slice(&chosen.1.states);
        literal_bits += chosen.1.bits;
        literal_distortion += chosen.1.distortion;
        literal_nonzero |= chosen.0 != 0;
    }
    if !literal_nonzero
        && let Some((_, pair, candidate)) = pairs
            .iter()
            .enumerate()
            .flat_map(|(pair, options)| {
                (1..3).filter_map(move |quality| {
                    options[quality].map(|candidate| {
                        let zero = options[0].expect("zero pair exists");
                        let penalty =
                            mixed_cost(candidate.distortion, candidate.bits, weight, lambda)
                                - mixed_cost(zero.distortion, zero.bits, weight, lambda);
                        (penalty, pair, candidate)
                    })
                })
            })
            .min_by(|left, right| left.0.total_cmp(&right.0))
    {
        let zero_pair = pairs[pair][0].expect("zero pair exists");
        literal_states[2 * pair..2 * pair + 2].copy_from_slice(&candidate.states);
        literal_bits = literal_bits - zero_pair.bits + candidate.bits;
        literal_distortion = literal_distortion - zero_pair.distortion + candidate.distortion;
    }
    let literal = bound_pair_group(
        source,
        band_model,
        MixedGroupChoice {
            states: literal_states,
            bits: literal_bits,
            distortion: literal_distortion,
        },
    );

    [zero, prefix, literal]
        .into_iter()
        .min_by(|left, right| {
            mixed_cost(left.distortion, left.bits, weight, lambda).total_cmp(&mixed_cost(
                right.distortion,
                right.bits,
                weight,
                lambda,
            ))
        })
        .expect("pair group has three syntax choices")
}

fn select_mixed_pair_image(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    image: usize,
    lambda: f64,
) -> MixedSelection {
    let mut states = Vec::with_capacity(PIXELS);
    let mut bits = 0u64;
    let mut weighted_distortion = 0.0;
    let mut raw_groups = 0usize;
    for group in 0..GROUP_COUNT {
        let band = usize::from(corpus.schedule[group]);
        let choice = choose_mixed_pair_group(
            &corpus.images[image].groups[group],
            &model.bands[band],
            weights[band],
            lambda,
        );
        states.extend(choice.states);
        bits += u64::from(choice.bits);
        weighted_distortion += choice.distortion as f64 * f64::from(weights[band]);
        raw_groups += usize::from(choice.bits == MIXED_PAIR_RAW_PACKET_BITS);
    }
    MixedSelection {
        states,
        bits,
        weighted_distortion,
        raw_groups,
    }
}

fn capacity_limited_mixed_pair_selection(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    image: usize,
    initial: f64,
) -> MixedSelection {
    let mut low = 0.0;
    let mut high = initial.max(1.0);
    let mut best = select_mixed_pair_image(corpus, model, weights, image, high);
    while best.bits > FRAME_BITS as u64 {
        low = high;
        high *= 2.0;
        best = select_mixed_pair_image(corpus, model, weights, image, high);
    }
    for _ in 0..22 {
        let lambda = (low + high) * 0.5;
        let selection = select_mixed_pair_image(corpus, model, weights, image, lambda);
        if selection.bits > FRAME_BITS as u64 {
            low = lambda;
        } else {
            high = lambda;
            if selection.weighted_distortion < best.weighted_distortion {
                best = selection;
            }
        }
    }
    best
}

fn fit_mixed_affine(
    corpus: &Corpus,
    model: &ProfileModel,
    states: &[u8],
) -> ([[u32; BAND_COUNT]; 2], [[i32; BAND_COUNT]; 2]) {
    let mut count = [[0i128; BAND_COUNT]; 2];
    let mut sum_x = [[0i128; BAND_COUNT]; 2];
    let mut sum_y = [[0i128; BAND_COUNT]; 2];
    let mut sum_xx = [[0i128; BAND_COUNT]; 2];
    let mut sum_xy = [[0i128; BAND_COUNT]; 2];
    for image in 0..corpus.images.len() {
        for group in 0..GROUP_COUNT {
            let band = usize::from(corpus.schedule[group]);
            for lane in 0..GROUP_LANES {
                let selected = states[(image * GROUP_COUNT + group) * GROUP_LANES + lane];
                if selected == 0 {
                    continue;
                }
                let quality = usize::from(selected - 1);
                let spec = if quality == 0 {
                    model.bands[band].fine.spec
                } else {
                    model.bands[band].coarse.spec
                };
                let coefficient = corpus.images[image].groups[group][lane];
                let q = quantize(coefficient, spec);
                let x = i128::from(q.unsigned_abs());
                let y = i128::from(coefficient.unsigned_abs());
                count[quality][band] += 1;
                sum_x[quality][band] += x;
                sum_y[quality][band] += y;
                sum_xx[quality][band] += x * x;
                sum_xy[quality][band] += x * y;
            }
        }
    }
    let mut gains = array::from_fn(|quality| {
        array::from_fn(|band| {
            let shift = if quality == 0 {
                model.bands[band].fine.spec.shift
            } else {
                model.bands[band].coarse.spec.shift
            };
            256u32 << shift
        })
    });
    let mut offsets = [[0i32; BAND_COUNT]; 2];
    for quality in 0..2 {
        for band in 0..BAND_COUNT {
            let n = count[quality][band];
            let denominator = n * sum_xx[quality][band] - sum_x[quality][band].pow(2);
            if denominator == 0 {
                continue;
            }
            let gain = rounded_divide(
                (n * sum_xy[quality][band] - sum_x[quality][band] * sum_y[quality][band]) * 256,
                denominator,
            )
            .clamp(0, i128::from(u32::MAX));
            let offset =
                rounded_divide(sum_y[quality][band] * 256 - gain * sum_x[quality][band], n)
                    .clamp(i128::from(i32::MIN), i128::from(i32::MAX));
            gains[quality][band] = gain as u32;
            offsets[quality][band] = offset as i32;
        }
    }
    (gains, offsets)
}

fn exact_evaluate_mixed(
    corpus: &Corpus,
    model: &ProfileModel,
    states: &[u8],
    gains_q8: &[[u32; BAND_COUNT]; 2],
    offsets_q8: &[[i32; BAND_COUNT]; 2],
) -> ExactEvaluation {
    assert_eq!(states.len(), corpus.images.len() * PIXELS);
    let band_list = bands(WIDTH, HEIGHT);
    let (sender, receiver) = mpsc::channel();
    std::thread::scope(|scope| {
        for image in 0..corpus.images.len() {
            let sender = sender.clone();
            let band_list = &band_list;
            scope.spawn(move || {
                let cached = &corpus.images[image];
                let mut coefficients = vec![0i32; PIXELS];
                let mut band_groups = [0usize; BAND_COUNT];
                let mut nonzero = 0usize;
                for group in 0..GROUP_COUNT {
                    let band = usize::from(corpus.schedule[group]);
                    let ordinal = band_groups[band];
                    band_groups[band] += 1;
                    let rect = band_list[band].rect;
                    let mut group_nonzero = false;
                    for lane in 0..GROUP_LANES {
                        let selected = states[(image * GROUP_COUNT + group) * GROUP_LANES + lane];
                        if selected == 0 {
                            continue;
                        }
                        let quality = usize::from(selected - 1);
                        let spec = if quality == 0 {
                            model.bands[band].fine.spec
                        } else {
                            model.bands[band].coarse.spec
                        };
                        let flat = ordinal * GROUP_LANES + lane;
                        let index =
                            (rect.y + flat / rect.width) * WIDTH + rect.x + flat % rect.width;
                        let quantized = quantize(cached.groups[group][lane], spec);
                        group_nonzero |= quantized != 0;
                        coefficients[index] = reconstruct_with_affine(
                            quantized,
                            gains_q8[quality][band],
                            offsets_q8[quality][band],
                        );
                    }
                    nonzero += usize::from(group_nonzero);
                }
                inverse_transform(&mut coefficients, WIDTH, HEIGHT);
                let sse = cached
                    .pixels
                    .iter()
                    .zip(coefficients)
                    .map(|(&original, reconstructed)| {
                        let error = i64::from(original) - i64::from(reconstructed.clamp(0, 255));
                        (error * error) as u64
                    })
                    .sum::<u64>();
                sender.send((image, sse, nonzero)).unwrap();
            });
        }
        drop(sender);
    });
    let mut results = receiver.into_iter().collect::<Vec<_>>();
    results.sort_by_key(|&(image, _, _)| image);
    let total_sse = results.iter().map(|&(_, sse, _)| u128::from(sse)).sum();
    let image_psnr = results
        .iter()
        .map(|&(image, sse, _)| {
            (
                corpus.images[image].name.clone(),
                psnr(sse as f64 / PIXELS as f64),
                sse,
            )
        })
        .collect::<Vec<_>>();
    ExactEvaluation {
        total_sse,
        aggregate_psnr: psnr(total_sse as f64 / (corpus.images.len() * PIXELS) as f64),
        mean_psnr: image_psnr.iter().map(|(_, value, _)| value).sum::<f64>()
            / image_psnr.len() as f64,
        image_psnr,
        max_nonzero_groups: results
            .iter()
            .map(|&(_, _, nonzero)| nonzero)
            .max()
            .unwrap_or(0),
    }
}

fn run_mixed_lane_probe(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    started: Instant,
) {
    let mut states = Vec::with_capacity(corpus.images.len() * PIXELS);
    let mut frame_bits = Vec::with_capacity(corpus.images.len());
    let mut raw_groups = Vec::with_capacity(corpus.images.len());
    for image in 0..corpus.images.len() {
        let selection = capacity_limited_mixed_selection(corpus, model, weights, image, 52.0);
        frame_bits.push(selection.bits);
        raw_groups.push(selection.raw_groups);
        states.extend(selection.states);
    }
    let (gains, offsets) = fit_mixed_affine(corpus, model, &states);
    let exact = exact_evaluate_mixed(corpus, model, &states, &gains, &offsets);
    for (image, (((name, image_psnr, sse), bits), raw)) in exact
        .image_psnr
        .iter()
        .zip(frame_bits.iter())
        .zip(raw_groups.iter())
        .enumerate()
    {
        println!(
            "mixed image={} name={} psnr_db={image_psnr:.9} sse={} bits={} raw_groups={}",
            image + 1,
            name,
            sse,
            bits,
            raw,
        );
    }
    println!(
        "mixed aggregate_psnr_db={:.15} mean_psnr_db={:.9} total_sse={} bits_min={} bits_max={} max_nonzero_groups={} packet_bits_max={} pair_token_bits_max={} elapsed_s={:.3}",
        exact.aggregate_psnr,
        exact.mean_psnr,
        exact.total_sse,
        frame_bits.iter().min().copied().unwrap_or(0),
        frame_bits.iter().max().copied().unwrap_or(0),
        exact.max_nonzero_groups,
        MIXED_RAW_PACKET_BITS,
        MIXED_PAIR_TOKEN_BITS,
        started.elapsed().as_secs_f64(),
    );
}

fn run_mixed_pair_probe(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    started: Instant,
) {
    let mut states = Vec::with_capacity(corpus.images.len() * PIXELS);
    let mut frame_bits = Vec::with_capacity(corpus.images.len());
    let mut raw_groups = Vec::with_capacity(corpus.images.len());
    for image in 0..corpus.images.len() {
        let selection = capacity_limited_mixed_pair_selection(corpus, model, weights, image, 52.0);
        frame_bits.push(selection.bits);
        raw_groups.push(selection.raw_groups);
        states.extend(selection.states);
    }
    let (gains, offsets) = fit_mixed_affine(corpus, model, &states);
    let exact = exact_evaluate_mixed(corpus, model, &states, &gains, &offsets);
    for (image, (((name, image_psnr, sse), bits), raw)) in exact
        .image_psnr
        .iter()
        .zip(frame_bits.iter())
        .zip(raw_groups.iter())
        .enumerate()
    {
        println!(
            "mixed-pair image={} name={} psnr_db={image_psnr:.9} sse={} bits={} raw_groups={}",
            image + 1,
            name,
            sse,
            bits,
            raw,
        );
    }
    println!(
        "mixed-pair aggregate_psnr_db={:.15} mean_psnr_db={:.9} total_sse={} bits_min={} bits_max={} max_nonzero_groups={} packet_bits_max={} pair_token_bits_max={} elapsed_s={:.3}",
        exact.aggregate_psnr,
        exact.mean_psnr,
        exact.total_sse,
        frame_bits.iter().min().copied().unwrap_or(0),
        frame_bits.iter().max().copied().unwrap_or(0),
        exact.max_nonzero_groups,
        MIXED_PAIR_RAW_PACKET_BITS,
        MIXED_PAIR_TOKEN_BITS,
        started.elapsed().as_secs_f64(),
    );
}

#[derive(Clone)]
struct SearchResult {
    bits: u64,
    distortion: f64,
}

fn unrestricted_search(
    corpus: &Corpus,
    base: &BaseCensus,
    weights: &[u32; BAND_COUNT],
    lambda: f64,
) -> SearchResult {
    let mut result = SearchResult {
        bits: 0,
        distortion: 0.0,
    };
    for (band, &band_weight) in weights.iter().enumerate() {
        for (local, &entry) in corpus.band_entries[band].iter().enumerate() {
            let weight = f64::from(band_weight);
            let zero_dist = zero_distortion(corpus.source(entry)) as f64 * weight;
            let mut best_cost = zero_dist + lambda;
            let mut best_bits = 1u8;
            let mut best_dist = zero_dist;
            for shift in 0..=base.max_shift {
                let point = base.points(band, shift)[local];
                let bits = state_packet_bits(point, base.rice(band, shift, false), false);
                let distortion = point.distortion as f64 * weight;
                let cost = distortion + lambda * f64::from(bits);
                if cost < best_cost {
                    best_cost = cost;
                    best_bits = bits;
                    best_dist = distortion;
                }
            }
            result.bits += u64::from(best_bits);
            result.distortion += best_dist;
        }
    }
    result
}

fn tune_unrestricted_lambda(
    corpus: &Corpus,
    base: &BaseCensus,
    weights: &[u32; BAND_COUNT],
    target_bits: u64,
) -> f64 {
    let mut low = 0.0;
    let mut high = 1.0;
    while unrestricted_search(corpus, base, weights, high).bits > target_bits {
        low = high;
        high *= 2.0;
    }
    let mut closest = high;
    let mut closest_error = u64::MAX;
    for _ in 0..15 {
        let lambda = (low + high) * 0.5;
        let result = unrestricted_search(corpus, base, weights, lambda);
        let error = result.bits.abs_diff(target_bits);
        if error < closest_error {
            closest = lambda;
            closest_error = error;
        }
        if result.bits > target_bits {
            low = lambda;
        } else {
            high = lambda;
        }
    }
    closest
}

fn fixed_search(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    lambda: f64,
) -> SearchResult {
    let mut result = SearchResult {
        bits: 0,
        distortion: 0.0,
    };
    for (band, &band_weight) in weights.iter().enumerate() {
        let fine = &model.bands[band].fine;
        let coarse = &model.bands[band].coarse;
        for (local, &entry) in corpus.band_entries[band].iter().enumerate() {
            let weight = f64::from(band_weight);
            let zero_dist = zero_distortion(corpus.source(entry)) as f64 * weight;
            let mut best = (zero_dist + lambda, 1u8, zero_dist);
            for (state, is_coarse) in [(fine, false), (coarse, true)] {
                let point = state.points[local];
                let bits = state_packet_bits(point, state.entropy_parameter, is_coarse);
                let distortion = point.distortion as f64 * weight;
                let candidate = (distortion + lambda * f64::from(bits), bits, distortion);
                if candidate.0 < best.0 {
                    best = candidate;
                }
            }
            result.bits += u64::from(best.1);
            result.distortion += best.2;
        }
    }
    result
}

fn tune_fixed_lambda(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    target_bits: u64,
    initial: f64,
) -> f64 {
    let mut low = 0.0;
    let mut high = initial.max(1.0);
    while fixed_search(corpus, model, weights, high).bits > target_bits {
        low = high;
        high *= 2.0;
    }
    let mut closest = high;
    let mut closest_error = u64::MAX;
    for _ in 0..18 {
        let lambda = (low + high) * 0.5;
        let result = fixed_search(corpus, model, weights, lambda);
        let error = result.bits.abs_diff(target_bits);
        if error < closest_error {
            closest = lambda;
            closest_error = error;
        }
        if result.bits > target_bits {
            low = lambda;
        } else {
            high = lambda;
        }
    }
    closest
}

fn band_cost(
    corpus: &Corpus,
    band: usize,
    fine: &QuantState,
    coarse: &QuantState,
    weight: u32,
    lambda: f64,
) -> f64 {
    corpus.band_entries[band]
        .iter()
        .enumerate()
        .map(|(local, &entry)| {
            let weight = f64::from(weight);
            let zero = weight * zero_distortion(corpus.source(entry)) as f64 + lambda;
            let fine_point = fine.points[local];
            let fine_cost = weight * fine_point.distortion as f64
                + lambda * f64::from(state_packet_bits(fine_point, fine.entropy_parameter, false));
            let coarse_point = coarse.points[local];
            let coarse_cost = weight * coarse_point.distortion as f64
                + lambda
                    * f64::from(state_packet_bits(
                        coarse_point,
                        coarse.entropy_parameter,
                        true,
                    ));
            zero.min(fine_cost).min(coarse_cost)
        })
        .sum()
}

fn make_state(
    corpus: &Corpus,
    coder: &impl EntropyCoder,
    band: usize,
    spec: QuantSpec,
    coarse: bool,
) -> QuantState {
    let points = build_points(corpus, band, spec, coder);
    QuantState {
        spec,
        entropy_parameter: best_entropy_parameter(&points, coarse),
        points,
    }
}

fn coordinate_band(
    corpus: &Corpus,
    coder: &impl EntropyCoder,
    weights: &[u32; BAND_COUNT],
    lambda: f64,
    band: usize,
    search_scope: (u8, bool),
    model: &mut ProfileModel,
) {
    let (max_shift, rtl_profile) = search_scope;
    for fine_state in [true, false] {
        let current_spec = if fine_state {
            model.bands[band].fine.spec
        } else {
            model.bands[band].coarse.spec
        };
        let other_shift = if fine_state {
            model.bands[band].coarse.spec.shift
        } else {
            model.bands[band].fine.spec.shift
        };

        let mut dimensions = Vec::<Vec<QuantSpec>>::new();
        let shifts = current_spec.shift.saturating_sub(1)..=current_spec.shift.saturating_add(1);
        dimensions.push(
            shifts
                .filter(|&shift| shift <= max_shift)
                .filter(|&shift| !rtl_profile || shift > 0)
                .filter(|&shift| {
                    if fine_state {
                        shift < other_shift && (!rtl_profile || other_shift - shift <= 3)
                    } else {
                        shift > other_shift && (!rtl_profile || shift - other_shift <= 3)
                    }
                })
                .map(|shift| {
                    if rtl_profile {
                        QuantSpec::rtl(shift)
                    } else {
                        QuantSpec {
                            shift,
                            ..current_spec
                        }
                    }
                })
                .collect(),
        );
        if !rtl_profile {
            dimensions.push(
                DEADZONE_CANDIDATES
                    .into_iter()
                    .map(|deadzone_q8| QuantSpec {
                        deadzone_q8,
                        ..current_spec
                    })
                    .collect(),
            );
            dimensions.push(
                CENTROID_CANDIDATES
                    .into_iter()
                    .map(|centroid_q8| QuantSpec {
                        centroid_q8,
                        ..current_spec
                    })
                    .collect(),
            );
        }

        for candidates in dimensions {
            let current = &model.bands[band];
            let mut best_cost = band_cost(
                corpus,
                band,
                &current.fine,
                &current.coarse,
                weights[band],
                lambda,
            );
            let mut best_state = None;
            for spec in candidates {
                let current_spec = if fine_state {
                    model.bands[band].fine.spec
                } else {
                    model.bands[band].coarse.spec
                };
                if spec == current_spec {
                    continue;
                }
                let trial = make_state(corpus, coder, band, spec, !fine_state);
                let candidate_cost = if fine_state {
                    band_cost(
                        corpus,
                        band,
                        &trial,
                        &model.bands[band].coarse,
                        weights[band],
                        lambda,
                    )
                } else {
                    band_cost(
                        corpus,
                        band,
                        &model.bands[band].fine,
                        &trial,
                        weights[band],
                        lambda,
                    )
                };
                if candidate_cost < best_cost {
                    best_cost = candidate_cost;
                    best_state = Some(trial);
                }
            }
            if let Some(state) = best_state {
                let kind = if fine_state { "fine" } else { "coarse" };
                println!(
                    "accept band={} state={} shift={} deadzone_q8={} centroid_q8={} rice={}",
                    BAND_NAMES[band],
                    kind,
                    state.spec.shift,
                    state.spec.deadzone_q8,
                    state.spec.centroid_q8,
                    state.entropy_parameter
                );
                if fine_state {
                    model.bands[band].fine = state;
                } else {
                    model.bands[band].coarse = state;
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum OracleState {
    Zero,
    Coarse,
    Fine,
}

struct OracleDecisions {
    states: Vec<OracleState>,
    total_bits: u64,
}

fn oracle_decisions(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    lambda: f64,
) -> OracleDecisions {
    let mut states = vec![OracleState::Zero; corpus.images.len() * GROUP_COUNT];
    let mut total_bits = 0u64;
    for image in 0..corpus.images.len() {
        for group in 0..GROUP_COUNT {
            let band = usize::from(corpus.schedule[group]);
            let local = corpus.local_point_index(image, group, band);
            let source = &corpus.images[image].groups[group];
            let weight = f64::from(weights[band]);
            let zero = weight * zero_distortion(source) as f64 + lambda;
            let fine = &model.bands[band].fine;
            let fine_point = fine.points[local];
            let fine_bits = state_packet_bits(fine_point, fine.entropy_parameter, false);
            let fine_cost = weight * fine_point.distortion as f64 + lambda * f64::from(fine_bits);
            let coarse = &model.bands[band].coarse;
            let coarse_point = coarse.points[local];
            let coarse_bits = state_packet_bits(coarse_point, coarse.entropy_parameter, true);
            let coarse_cost =
                weight * coarse_point.distortion as f64 + lambda * f64::from(coarse_bits);
            let state = if fine_cost <= zero && fine_cost <= coarse_cost {
                total_bits += u64::from(fine_bits);
                OracleState::Fine
            } else if coarse_cost <= zero {
                total_bits += u64::from(coarse_bits);
                OracleState::Coarse
            } else {
                total_bits += 1;
                OracleState::Zero
            };
            states[image * GROUP_COUNT + group] = state;
        }
    }
    OracleDecisions { states, total_bits }
}

fn fit_nominal_bits(
    corpus: &Corpus,
    model: &ProfileModel,
    oracle: &OracleDecisions,
) -> [u16; BAND_COUNT] {
    let mut bits = [0u64; BAND_COUNT];
    let mut count = [0u64; BAND_COUNT];
    for image in 0..corpus.images.len() {
        for group in 0..GROUP_COUNT {
            let band = usize::from(corpus.schedule[group]);
            let local = corpus.local_point_index(image, group, band);
            let state_bits = u64::from(match oracle.states[image * GROUP_COUNT + group] {
                OracleState::Zero => 1,
                OracleState::Fine => {
                    let state = &model.bands[band].fine;
                    state_packet_bits(state.points[local], state.entropy_parameter, false)
                }
                OracleState::Coarse => {
                    let state = &model.bands[band].coarse;
                    state_packet_bits(state.points[local], state.entropy_parameter, true)
                }
            });
            bits[band] += state_bits;
            count[band] += 1;
        }
    }
    array::from_fn(|band| ((bits[band] + count[band] / 2) / count[band]).clamp(1, 147) as u16)
}

#[derive(Clone, Copy, Debug)]
struct CausalMetric {
    weight: u32,
    zero_distortion: u64,
    fine_distortion: u64,
    coarse_distortion: u64,
    fine_bits: u8,
    coarse_bits: u8,
    fine_direct_slope: u8,
    fine_slope: u8,
    coarse_slope: u8,
    observable_direct_gain: u32,
    observable_incremental_gain: u32,
}

fn slope_bin(gain: u128, extra_bits: usize) -> u8 {
    if gain == 0 {
        return 0;
    }
    if extra_bits == 0 {
        return 31;
    }
    let value = gain / extra_bits as u128;
    if value == 0 {
        0
    } else {
        (1 + (127 - value.leading_zeros()) as u8).min(31)
    }
}

fn build_causal_metrics(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    hardware_metric: HardwareMetric,
) -> Vec<CausalMetric> {
    let mut metrics = Vec::with_capacity(corpus.images.len() * GROUP_COUNT);
    for image in 0..corpus.images.len() {
        for group in 0..GROUP_COUNT {
            let band = usize::from(corpus.schedule[group]);
            let local = corpus.local_point_index(image, group, band);
            let zero_distortion = zero_distortion(&corpus.images[image].groups[group]);
            let fine = &model.bands[band].fine;
            let coarse = &model.bands[band].coarse;
            let fine_point = fine.points[local];
            let coarse_point = coarse.points[local];
            let fine_bits = state_packet_bits(fine_point, fine.entropy_parameter, false);
            let coarse_bits = state_packet_bits(coarse_point, coarse.entropy_parameter, true);
            let weight = u128::from(weights[band]);
            let (fine_direct_gain, fine_incremental_gain, coarse_gain) = (
                zero_distortion.saturating_sub(fine_point.distortion),
                coarse_point
                    .distortion
                    .saturating_sub(fine_point.distortion),
                zero_distortion.saturating_sub(coarse_point.distortion),
            );
            let observable = observable_group_energy_for(
                &corpus.images[image].groups[group],
                fine.spec,
                coarse.spec,
                hardware_metric,
            );
            metrics.push(CausalMetric {
                weight: weights[band],
                zero_distortion,
                fine_distortion: fine_point.distortion,
                coarse_distortion: coarse_point.distortion,
                fine_bits,
                coarse_bits,
                fine_direct_slope: slope_bin(
                    u128::from(fine_direct_gain) * weight,
                    usize::from(fine_bits.saturating_sub(1)),
                ),
                fine_slope: slope_bin(
                    u128::from(fine_incremental_gain) * weight,
                    usize::from(fine_bits.saturating_sub(coarse_bits)),
                ),
                coarse_slope: slope_bin(
                    u128::from(coarse_gain) * weight,
                    usize::from(coarse_bits.saturating_sub(1)),
                ),
                observable_direct_gain: observable.direct,
                observable_incremental_gain: observable.incremental,
            });
        }
    }
    metrics
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ObservableEnergy {
    direct: u32,
    incremental: u32,
    extra_flag: bool,
}

#[cfg(test)]
fn observable_coefficient_energy(
    source: i32,
    fine_spec: QuantSpec,
    coarse_spec: QuantSpec,
) -> ObservableEnergy {
    observable_coefficient_energy_for(source, fine_spec, coarse_spec, HardwareMetric::Exact)
}

fn observable_coefficient_energy_for(
    source: i32,
    fine_spec: QuantSpec,
    coarse_spec: QuantSpec,
    hardware_metric: HardwareMetric,
) -> ObservableEnergy {
    debug_assert!(coarse_spec.shift > fine_spec.shift);
    let fine = quantize(source, fine_spec);
    let delta = coarse_spec.shift - fine_spec.shift;
    let (coarse, correction) = if fine == 0 {
        (0, false)
    } else {
        let derived = fine.unsigned_abs() >> delta;
        let independent = quantize(source, coarse_spec).unsigned_abs();
        let correction = independent > derived;
        let magnitude = derived + u16::from(correction);
        let coarse = if fine < 0 {
            -(magnitude as i16)
        } else {
            magnitude as i16
        };
        (coarse, correction)
    };

    let magnitude = u32::from(fine.unsigned_abs());
    let residual = (i32::from(fine) - (i32::from(coarse) << delta)).unsigned_abs();
    if hardware_metric == HardwareMetric::Square {
        return ObservableEnergy {
            direct: 8 * magnitude * magnitude,
            incremental: 8 * residual * residual,
            extra_flag: false,
        };
    }
    if hardware_metric == HardwareMetric::Linear {
        return ObservableEnergy {
            direct: 8 * magnitude,
            incremental: 8 * residual,
            extra_flag: false,
        };
    }
    if hardware_metric == HardwareMetric::CorrectionOnly {
        let direct_base = 8 * magnitude * magnitude;
        let direct_adjustment = 4 * magnitude;
        return ObservableEnergy {
            direct: if correction {
                direct_base.saturating_add(direct_adjustment)
            } else {
                direct_base.saturating_sub(direct_adjustment)
            },
            incremental: (8 * residual * residual).saturating_sub(4 * residual),
            extra_flag: false,
        };
    }
    let error = source - (i32::from(fine) << fine_spec.shift);
    let odd = magnitude & 1 != 0;
    // This is the one new bit per coefficient. Two coefficients exactly fill
    // the two spare bits in the native 20-bit pair word.
    let extra_flag = if odd {
        error.unsigned_abs().saturating_mul(4) >= 1u32 << fine_spec.shift
    } else {
        i64::from(fine) * i64::from(error) >= 0
    };
    let coefficient = if odd {
        if extra_flag { 6 } else { 2 }
    } else {
        4
    };
    let direct_base = 8 * magnitude * magnitude;
    let direct_adjustment = coefficient * magnitude;
    let direct_assist = if odd { correction } else { extra_flag };
    let direct = if direct_assist {
        direct_base.saturating_add(direct_adjustment)
    } else {
        direct_base.saturating_sub(direct_adjustment)
    };

    // Deliberately always subtract: after `extra_flag` is stored, this rule
    // needs no unavailable source-error sign at the causal allocator.
    let incremental = (8 * residual * residual).saturating_sub(coefficient * residual);
    ObservableEnergy {
        direct,
        incremental,
        extra_flag,
    }
}

fn observable_group_energy_for(
    source: &[i32; GROUP_LANES],
    fine_spec: QuantSpec,
    coarse_spec: QuantSpec,
    hardware_metric: HardwareMetric,
) -> ObservableEnergy {
    source.iter().fold(
        ObservableEnergy {
            direct: 0,
            incremental: 0,
            extra_flag: false,
        },
        |sum, &coefficient| {
            let energy = observable_coefficient_energy_for(
                coefficient,
                fine_spec,
                coarse_spec,
                hardware_metric,
            );
            ObservableEnergy {
                direct: sum.direct.saturating_add(energy.direct),
                incremental: sum.incremental.saturating_add(energy.incremental),
                // The group caller only consumes the sums. Retaining OR here
                // makes the aggregate useful in focused metric tests.
                extra_flag: sum.extra_flag || energy.extra_flag,
            }
        },
    )
}

#[derive(Clone, Copy, Debug)]
struct CausalPolicy {
    cutoff_base: u8,
    incremental_cutoff_base: u8,
    debt_shift: u8,
    nominal_bits: [u16; BAND_COUNT],
    total_used_bits: u64,
    proxy_distortion: u128,
    nonzero_groups: u64,
    min_frame_bits: usize,
    max_frame_bits: usize,
    max_nonzero_groups: usize,
    min_debt: i64,
    max_debt: i64,
}

#[cfg(test)]
fn half_octave_bin(gain: u64, extra_bits: usize) -> u8 {
    if gain == 0 {
        return 0;
    }
    if extra_bits == 0 {
        return 31;
    }
    let slope = gain / extra_bits as u64;
    if slope == 0 {
        return 0;
    }
    let exponent = 63 - slope.leading_zeros() as u8;
    let base = 1u64 << exponent;
    let upper_half = u8::from(u128::from(slope) * 2 >= u128::from(base) * 3);
    (1 + 2 * exponent + upper_half).min(31)
}

// Exact divider-free form of `half_octave_bin(gain, extra_bits) >= cutoff`.
// Every nontrivial threshold is a constant shift or a 3x shift/add in RTL.
fn slope_passes_cutoff(gain: u64, extra_bits: usize, cutoff: u8) -> bool {
    if cutoff == 0 {
        return true;
    }
    if extra_bits == 0 {
        return gain != 0;
    }
    let threshold = match cutoff {
        1 => 1u64,
        2 | 3 => 2,
        value if value & 1 != 0 => 1u64 << ((value - 1) / 2),
        value => 3u64 << ((value - 4) / 2),
    };
    u128::from(gain) >= extra_bits as u128 * u128::from(threshold)
}

fn hardware_cutoffs(
    pre_debt: i64,
    coarse_base: u8,
    incremental_base: u8,
    debt_shift: u8,
) -> (u8, u8) {
    let debt_octaves = pre_debt >> debt_shift;
    (
        (i64::from(coarse_base) + debt_octaves).clamp(0, 8) as u8,
        (i64::from(incremental_base) + debt_octaves).clamp(0, 8) as u8,
    )
}

fn advance_zero_run(pending: &mut usize, selected_zero: bool) -> bool {
    if !selected_zero {
        *pending = 0;
        return false;
    }
    *pending += 1;
    let realize_credit = *pending >= ZERO_RUN_CREDIT_START;
    if *pending == ZERO_RUN_MAX {
        *pending = 0;
    }
    realize_credit
}

fn simulate_canonical_policy(
    corpus: &Corpus,
    metrics: &[CausalMetric],
    hardware_policy: HardwarePolicy,
    keep_states: bool,
) -> (CausalPolicy, Option<Vec<OracleState>>) {
    simulate_hardware_policy(
        corpus,
        metrics,
        hardware_policy,
        CANONICAL_NOMINAL_BITS,
        CANONICAL_COARSE_CUTOFF_BASE,
        CANONICAL_INCREMENTAL_CUTOFF_BASE,
        CANONICAL_DEBT_SHIFT,
        keep_states,
    )
}

#[allow(clippy::too_many_arguments)]
fn simulate_hardware_policy(
    corpus: &Corpus,
    metrics: &[CausalMetric],
    hardware_policy: HardwarePolicy,
    nominal_bits: [u16; BAND_COUNT],
    cutoff_base: u8,
    incremental_cutoff_base: u8,
    debt_shift: u8,
    keep_states: bool,
) -> (CausalPolicy, Option<Vec<OracleState>>) {
    simulate_hardware_policy_with_energy_shift(
        corpus,
        metrics,
        hardware_policy,
        nominal_bits,
        cutoff_base,
        incremental_cutoff_base,
        debt_shift,
        CANONICAL_COARSE_ENERGY_SHIFT,
        CANONICAL_INCREMENTAL_ENERGY_SHIFT,
        keep_states,
    )
}

#[allow(clippy::too_many_arguments)]
fn simulate_hardware_policy_with_energy_shift(
    corpus: &Corpus,
    metrics: &[CausalMetric],
    hardware_policy: HardwarePolicy,
    nominal_bits: [u16; BAND_COUNT],
    cutoff_base: u8,
    incremental_cutoff_base: u8,
    debt_shift: u8,
    coarse_energy_shift: [u8; BAND_COUNT],
    incremental_energy_shift: [u8; BAND_COUNT],
    keep_states: bool,
) -> (CausalPolicy, Option<Vec<OracleState>>) {
    let mut total_used_bits = 0u64;
    let mut proxy_distortion = 0u128;
    let mut nonzero_groups = 0u64;
    let mut min_frame_bits = usize::MAX;
    let mut max_frame_bits = 0usize;
    let mut max_nonzero_groups = 0usize;
    let mut min_debt = 0i64;
    let mut max_debt = 0i64;
    let mut output_states = keep_states.then(|| Vec::with_capacity(metrics.len()));
    for image in 0..corpus.images.len() {
        let mut debt = 0i64;
        let mut room = FRAME_BITS - GROUP_COUNT;
        let mut frame_bits = 0usize;
        let mut frame_nonzero_groups = 0usize;
        let mut pending_zero_run = 0usize;
        for group in 0..GROUP_COUNT {
            let metric = metrics[image * GROUP_COUNT + group];
            let band = usize::from(corpus.schedule[group]);
            let pre_debt = debt - i64::from(nominal_bits[band]);
            let (coarse_cutoff, incremental_cutoff) =
                hardware_cutoffs(pre_debt, cutoff_base, incremental_cutoff_base, debt_shift);
            let direct_gain = u64::from(metric.observable_direct_gain);
            let incremental_gain = u64::from(metric.observable_incremental_gain);
            let coarse_gain =
                direct_gain.saturating_sub(incremental_gain) << coarse_energy_shift[band];
            let direct_gain = direct_gain << coarse_energy_shift[band];
            let incremental_gain = incremental_gain << incremental_energy_shift[band];
            let fine_extra = usize::from(metric.fine_bits.saturating_sub(1));
            let coarse_extra = usize::from(metric.coarse_bits.saturating_sub(1));
            let incremental_extra =
                usize::from(metric.fine_bits.saturating_sub(metric.coarse_bits));
            let fine_fits = fine_extra <= room;
            let coarse_fits = coarse_extra <= room;
            let coarse_allowed =
                coarse_fits && slope_passes_cutoff(coarse_gain, coarse_extra, coarse_cutoff);
            let state = match hardware_policy {
                HardwarePolicy::Hull if coarse_allowed => {
                    if (metric.fine_bits <= metric.coarse_bits
                        || slope_passes_cutoff(
                            incremental_gain,
                            incremental_extra,
                            incremental_cutoff,
                        ))
                        && fine_fits
                    {
                        OracleState::Fine
                    } else {
                        OracleState::Coarse
                    }
                }
                HardwarePolicy::Hull => {
                    if fine_fits && slope_passes_cutoff(direct_gain, fine_extra, coarse_cutoff) {
                        OracleState::Fine
                    } else {
                        OracleState::Zero
                    }
                }
                HardwarePolicy::CoarseFloor if coarse_fits => {
                    if (metric.fine_bits <= metric.coarse_bits
                        || slope_passes_cutoff(
                            incremental_gain,
                            incremental_extra,
                            incremental_cutoff,
                        ))
                        && fine_fits
                    {
                        OracleState::Fine
                    } else {
                        OracleState::Coarse
                    }
                }
                HardwarePolicy::CoarseFloor => {
                    if fine_fits && slope_passes_cutoff(direct_gain, fine_extra, coarse_cutoff) {
                        OracleState::Fine
                    } else {
                        OracleState::Zero
                    }
                }
                HardwarePolicy::FineOnly => {
                    if fine_fits && slope_passes_cutoff(direct_gain, fine_extra, coarse_cutoff) {
                        OracleState::Fine
                    } else {
                        OracleState::Zero
                    }
                }
                HardwarePolicy::NoDirectBypass => {
                    if coarse_allowed {
                        if (metric.fine_bits <= metric.coarse_bits
                            || slope_passes_cutoff(
                                incremental_gain,
                                incremental_extra,
                                incremental_cutoff,
                            ))
                            && fine_fits
                        {
                            OracleState::Fine
                        } else {
                            OracleState::Coarse
                        }
                    } else {
                        OracleState::Zero
                    }
                }
                HardwarePolicy::DirectFine => {
                    if fine_fits && slope_passes_cutoff(direct_gain, fine_extra, coarse_cutoff) {
                        OracleState::Fine
                    } else if coarse_allowed {
                        OracleState::Coarse
                    } else {
                        OracleState::Zero
                    }
                }
                HardwarePolicy::NoIncremental => {
                    if coarse_allowed {
                        OracleState::Coarse
                    } else if fine_fits
                        && slope_passes_cutoff(direct_gain, fine_extra, coarse_cutoff)
                    {
                        OracleState::Fine
                    } else {
                        OracleState::Zero
                    }
                }
            };
            let (bits, distortion) = match state {
                OracleState::Fine => (metric.fine_bits, metric.fine_distortion),
                OracleState::Coarse => (metric.coarse_bits, metric.coarse_distortion),
                OracleState::Zero => (1, metric.zero_distortion),
            };
            room -= usize::from(bits - 1);
            debt = pre_debt + i64::from(bits);
            total_used_bits += u64::from(bits);
            frame_bits += usize::from(bits);
            if advance_zero_run(&mut pending_zero_run, matches!(state, OracleState::Zero)) {
                room += 1;
                debt -= 1;
                total_used_bits -= 1;
                frame_bits -= 1;
            }
            min_debt = min_debt.min(debt);
            max_debt = max_debt.max(debt);
            proxy_distortion += u128::from(distortion) * u128::from(metric.weight);
            frame_nonzero_groups += usize::from(bits > 1);
            nonzero_groups += u64::from(bits > 1);
            if let Some(states) = &mut output_states {
                states.push(state);
            }
        }
        min_frame_bits = min_frame_bits.min(frame_bits);
        max_frame_bits = max_frame_bits.max(frame_bits);
        max_nonzero_groups = max_nonzero_groups.max(frame_nonzero_groups);
    }
    (
        CausalPolicy {
            cutoff_base,
            incremental_cutoff_base,
            debt_shift,
            nominal_bits,
            total_used_bits,
            proxy_distortion,
            nonzero_groups,
            min_frame_bits,
            max_frame_bits,
            max_nonzero_groups,
            min_debt,
            max_debt,
        },
        output_states,
    )
}

fn fit_hardware_controller(
    corpus: &Corpus,
    metrics: &[CausalMetric],
    hardware_policy: HardwarePolicy,
) -> CausalPolicy {
    let is_better = |candidate: &CausalPolicy, current: &CausalPolicy| {
        (
            candidate.proxy_distortion,
            u64::MAX - candidate.total_used_bits,
        ) < (current.proxy_distortion, u64::MAX - current.total_used_bits)
    };
    let mut best = None::<CausalPolicy>;
    for nominal_scale_q8 in (192u16..=384).step_by(16) {
        let nominal_bits = CANONICAL_NOMINAL_BITS.map(|value| {
            ((u32::from(value) * u32::from(nominal_scale_q8) + 128) >> 8)
                .clamp(1, u32::from(COARSE_RAW_PACKET_BITS)) as u16
        });
        for debt_shift in 10..=18 {
            for cutoff_base in 0..=12 {
                let candidate = simulate_hardware_policy(
                    corpus,
                    metrics,
                    hardware_policy,
                    nominal_bits,
                    cutoff_base,
                    cutoff_base.saturating_add(1),
                    debt_shift,
                    false,
                )
                .0;
                if best
                    .as_ref()
                    .is_none_or(|current| is_better(&candidate, current))
                {
                    best = Some(candidate);
                }
            }
        }
    }

    let mut best = best.expect("hardware-controller grid is nonempty");
    for _ in 0..2 {
        for band in 0..BAND_COUNT {
            let center = i32::from(best.nominal_bits[band]);
            for delta in [-16, -8, -4, 0, 4, 8, 16] {
                let mut nominal_bits = best.nominal_bits;
                nominal_bits[band] =
                    (center + delta).clamp(1, i32::from(COARSE_RAW_PACKET_BITS)) as u16;
                let candidate = simulate_hardware_policy(
                    corpus,
                    metrics,
                    hardware_policy,
                    nominal_bits,
                    best.cutoff_base,
                    best.incremental_cutoff_base,
                    best.debt_shift,
                    false,
                )
                .0;
                if is_better(&candidate, &best) {
                    best = candidate;
                }
            }
        }
    }
    best
}

fn evaluate_hardware_energy_shifts(
    corpus: &Corpus,
    model: &ProfileModel,
    metrics: &[CausalMetric],
    hardware_policy: HardwarePolicy,
    controller: CausalPolicy,
    energy_shifts: [[u8; BAND_COUNT]; 2],
) -> (CausalPolicy, Vec<OracleState>, ExactEvaluation) {
    let (causal, states) = simulate_hardware_policy_with_energy_shift(
        corpus,
        metrics,
        hardware_policy,
        controller.nominal_bits,
        controller.cutoff_base,
        controller.incremental_cutoff_base,
        controller.debt_shift,
        energy_shifts[0],
        energy_shifts[1],
        true,
    );
    let states = states.expect("energy-shift tuning requests causal states");
    let exact = exact_evaluate_states_with_affine(
        corpus,
        model,
        &states,
        &CANONICAL_RECONSTRUCTION_GAIN_Q8,
        &CANONICAL_RECONSTRUCTION_OFFSET_Q8,
    );
    (causal, states, exact)
}

fn tune_hardware_energy_shifts(
    corpus: &Corpus,
    model: &ProfileModel,
    metrics: &[CausalMetric],
    hardware_policy: HardwarePolicy,
    controller: CausalPolicy,
) -> (
    [u8; BAND_COUNT],
    [u8; BAND_COUNT],
    CausalPolicy,
    Vec<OracleState>,
) {
    let mut shifts = [
        CANONICAL_COARSE_ENERGY_SHIFT,
        CANONICAL_INCREMENTAL_ENERGY_SHIFT,
    ];
    let (mut best_causal, mut best_states, mut best_exact) = evaluate_hardware_energy_shifts(
        corpus,
        model,
        metrics,
        hardware_policy,
        controller,
        shifts,
    );
    println!(
        "energy_shift start coarse={:?} incremental={:?} sse={} psnr_db={:.9}",
        shifts[0], shifts[1], best_exact.total_sse, best_exact.aggregate_psnr,
    );

    for round in 0..2 {
        for transition in 0..2 {
            for band in 0..BAND_COUNT {
                let mut band_best = None;
                for shift in 0..=3 {
                    if shift == shifts[transition][band] {
                        continue;
                    }
                    let mut candidate_shifts = shifts;
                    candidate_shifts[transition][band] = shift;
                    let (candidate_causal, candidate_states, candidate_exact) =
                        evaluate_hardware_energy_shifts(
                            corpus,
                            model,
                            metrics,
                            hardware_policy,
                            controller,
                            candidate_shifts,
                        );
                    let candidate_key = (
                        candidate_exact.total_sse,
                        u64::MAX - candidate_causal.total_used_bits,
                        shift,
                    );
                    if band_best.as_ref().is_none_or(
                        |(key, _, _, _): &(
                            (u128, u64, u8),
                            CausalPolicy,
                            Vec<OracleState>,
                            ExactEvaluation,
                        )| candidate_key < *key,
                    ) {
                        band_best = Some((
                            candidate_key,
                            candidate_causal,
                            candidate_states,
                            candidate_exact,
                        ));
                    }
                }
                if let Some((key, candidate_causal, candidate_states, candidate_exact)) = band_best
                {
                    let best_key = (
                        best_exact.total_sse,
                        u64::MAX - best_causal.total_used_bits,
                        shifts[transition][band],
                    );
                    if key < best_key {
                        shifts[transition][band] = key.2;
                        best_causal = candidate_causal;
                        best_states = candidate_states;
                        best_exact = candidate_exact;
                        println!(
                            "energy_shift accept round={} transition={} band={} shift={} sse={} psnr_db={:.9}",
                            round + 1,
                            if transition == 0 {
                                "coarse"
                            } else {
                                "incremental"
                            },
                            BAND_NAMES[band],
                            shifts[transition][band],
                            best_exact.total_sse,
                            best_exact.aggregate_psnr,
                        );
                    }
                }
            }
        }
    }
    println!(
        "energy_shift final coarse={:?} incremental={:?} sse={} psnr_db={:.9}",
        shifts[0], shifts[1], best_exact.total_sse, best_exact.aggregate_psnr,
    );
    (shifts[0], shifts[1], best_causal, best_states)
}

fn exact_controller_key(causal: &CausalPolicy, exact: &ExactEvaluation) -> (u128, u64) {
    (
        exact.total_sse,
        u64::MAX.saturating_sub(causal.total_used_bits),
    )
}

fn tune_hardware_controller_exact(
    corpus: &Corpus,
    model: &ProfileModel,
    metrics: &[CausalMetric],
    hardware_policy: HardwarePolicy,
    coarse_energy_shift: [u8; BAND_COUNT],
    incremental_energy_shift: [u8; BAND_COUNT],
    controller: CausalPolicy,
) -> (CausalPolicy, Vec<OracleState>) {
    tune_hardware_controller_exact_rounds(
        corpus,
        model,
        metrics,
        hardware_policy,
        coarse_energy_shift,
        incremental_energy_shift,
        controller,
        2,
    )
}

#[allow(clippy::too_many_arguments)]
fn tune_hardware_controller_exact_rounds(
    corpus: &Corpus,
    model: &ProfileModel,
    metrics: &[CausalMetric],
    hardware_policy: HardwarePolicy,
    coarse_energy_shift: [u8; BAND_COUNT],
    incremental_energy_shift: [u8; BAND_COUNT],
    controller: CausalPolicy,
    rounds: usize,
) -> (CausalPolicy, Vec<OracleState>) {
    let (mut best_causal, mut best_states, mut best_exact) = evaluate_hardware_energy_shifts(
        corpus,
        model,
        metrics,
        hardware_policy,
        controller,
        [coarse_energy_shift, incremental_energy_shift],
    );
    println!(
        "exact_controller start coarse_cutoff={} incremental_cutoff={} debt_shift={} nominal={:?} sse={} psnr_db={:.9}",
        best_causal.cutoff_base,
        best_causal.incremental_cutoff_base,
        best_causal.debt_shift,
        best_causal.nominal_bits,
        best_exact.total_sse,
        best_exact.aggregate_psnr,
    );

    for round in 0..rounds {
        let cutoff_base = best_causal;
        for cutoff in 0..=12 {
            let mut candidate_controller = cutoff_base;
            candidate_controller.cutoff_base = cutoff;
            let (candidate_causal, candidate_states, candidate_exact) =
                evaluate_hardware_energy_shifts(
                    corpus,
                    model,
                    metrics,
                    hardware_policy,
                    candidate_controller,
                    [coarse_energy_shift, incremental_energy_shift],
                );
            if exact_controller_key(&candidate_causal, &candidate_exact)
                < exact_controller_key(&best_causal, &best_exact)
            {
                best_causal = candidate_causal;
                best_states = candidate_states;
                best_exact = candidate_exact;
            }
        }

        let incremental_cutoff_base = best_causal;
        for cutoff in 0..=12 {
            let mut candidate_controller = incremental_cutoff_base;
            candidate_controller.incremental_cutoff_base = cutoff;
            let (candidate_causal, candidate_states, candidate_exact) =
                evaluate_hardware_energy_shifts(
                    corpus,
                    model,
                    metrics,
                    hardware_policy,
                    candidate_controller,
                    [coarse_energy_shift, incremental_energy_shift],
                );
            if exact_controller_key(&candidate_causal, &candidate_exact)
                < exact_controller_key(&best_causal, &best_exact)
            {
                best_causal = candidate_causal;
                best_states = candidate_states;
                best_exact = candidate_exact;
            }
        }

        let debt_base = best_causal;
        for debt_shift in 10..=18 {
            let mut candidate_controller = debt_base;
            candidate_controller.debt_shift = debt_shift;
            let (candidate_causal, candidate_states, candidate_exact) =
                evaluate_hardware_energy_shifts(
                    corpus,
                    model,
                    metrics,
                    hardware_policy,
                    candidate_controller,
                    [coarse_energy_shift, incremental_energy_shift],
                );
            if exact_controller_key(&candidate_causal, &candidate_exact)
                < exact_controller_key(&best_causal, &best_exact)
            {
                best_causal = candidate_causal;
                best_states = candidate_states;
                best_exact = candidate_exact;
            }
        }

        for band in 0..BAND_COUNT {
            let nominal_base = best_causal;
            let center = i32::from(nominal_base.nominal_bits[band]);
            for delta in [-32, -16, -8, -4, -2, -1, 0, 1, 2, 4, 8, 16, 32] {
                let mut candidate_controller = nominal_base;
                candidate_controller.nominal_bits[band] =
                    (center + delta).clamp(1, i32::from(COARSE_RAW_PACKET_BITS)) as u16;
                let (candidate_causal, candidate_states, candidate_exact) =
                    evaluate_hardware_energy_shifts(
                        corpus,
                        model,
                        metrics,
                        hardware_policy,
                        candidate_controller,
                        [coarse_energy_shift, incremental_energy_shift],
                    );
                if exact_controller_key(&candidate_causal, &candidate_exact)
                    < exact_controller_key(&best_causal, &best_exact)
                {
                    best_causal = candidate_causal;
                    best_states = candidate_states;
                    best_exact = candidate_exact;
                }
            }
        }
        println!(
            "exact_controller round={} coarse_cutoff={} incremental_cutoff={} debt_shift={} nominal={:?} sse={} psnr_db={:.9}",
            round + 1,
            best_causal.cutoff_base,
            best_causal.incremental_cutoff_base,
            best_causal.debt_shift,
            best_causal.nominal_bits,
            best_exact.total_sse,
            best_exact.aggregate_psnr,
        );
    }
    (best_causal, best_states)
}

fn model_rice_tables(model: &ProfileModel) -> [[u8; BAND_COUNT]; 2] {
    [
        model
            .bands
            .each_ref()
            .map(|band| band.fine.entropy_parameter),
        model
            .bands
            .each_ref()
            .map(|band| band.coarse.entropy_parameter),
    ]
}

fn set_model_rice_tables(model: &mut ProfileModel, tables: [[u8; BAND_COUNT]; 2]) {
    for (band, (&fine, &coarse)) in tables[0].iter().zip(&tables[1]).enumerate() {
        model.bands[band].fine.entropy_parameter = fine;
        model.bands[band].coarse.entropy_parameter = coarse;
    }
}

fn rate_optimal_rice_tables(model: &ProfileModel) -> [[u8; BAND_COUNT]; 2] {
    array::from_fn(|quality| {
        array::from_fn(|band| {
            let state = if quality == 0 {
                &model.bands[band].fine
            } else {
                &model.bands[band].coarse
            };
            best_entropy_parameter(&state.points, quality == 1)
        })
    })
}

fn evaluate_fixed_profile(
    corpus: &Corpus,
    model: &ProfileModel,
    weights: &[u32; BAND_COUNT],
    hardware_policy: HardwarePolicy,
    controller: CausalPolicy,
    energy_shifts: [[u8; BAND_COUNT]; 2],
) -> (CausalPolicy, Vec<OracleState>, ExactEvaluation) {
    let metrics = build_causal_metrics(corpus, model, weights, HardwareMetric::Exact);
    evaluate_hardware_energy_shifts(
        corpus,
        model,
        &metrics,
        hardware_policy,
        controller,
        energy_shifts,
    )
}

fn tune_fixed_rice_exact(
    corpus: &Corpus,
    model: &mut ProfileModel,
    weights: &[u32; BAND_COUNT],
    hardware_policy: HardwarePolicy,
    controller: CausalPolicy,
    energy_shifts: [[u8; BAND_COUNT]; 2],
    radius: Option<u8>,
) -> (CausalPolicy, Vec<OracleState>, ExactEvaluation) {
    let (mut best_causal, mut best_states, mut best_exact) = evaluate_fixed_profile(
        corpus,
        model,
        weights,
        hardware_policy,
        controller,
        energy_shifts,
    );
    println!(
        "fixed_rice start radius={radius:?} fine={:?} coarse={:?} sse={} psnr_db={:.9}",
        model_rice_tables(model)[0],
        model_rice_tables(model)[1],
        best_exact.total_sse,
        best_exact.aggregate_psnr,
    );
    for quality in 0..2 {
        for (band, band_name) in BAND_NAMES.iter().enumerate() {
            let coarse = quality == 1;
            let current = if coarse {
                model.bands[band].coarse.entropy_parameter
            } else {
                model.bands[band].fine.entropy_parameter
            };
            let first = radius.map_or(0, |value| current.saturating_sub(value));
            let last = radius.map_or((RICE_COUNT - 1) as u8, |value| {
                current.saturating_add(value).min((RICE_COUNT - 1) as u8)
            });
            let mut band_best = None;
            for rice in first..=last {
                if rice == current {
                    continue;
                }
                if coarse {
                    model.bands[band].coarse.entropy_parameter = rice;
                } else {
                    model.bands[band].fine.entropy_parameter = rice;
                }
                let (candidate_causal, candidate_states, candidate_exact) = evaluate_fixed_profile(
                    corpus,
                    model,
                    weights,
                    hardware_policy,
                    controller,
                    energy_shifts,
                );
                let key = (
                    candidate_exact.total_sse,
                    u64::MAX.saturating_sub(candidate_causal.total_used_bits),
                    rice,
                );
                if band_best.as_ref().is_none_or(
                    |(best_key, _, _, _): &(
                        (u128, u64, u8),
                        CausalPolicy,
                        Vec<OracleState>,
                        ExactEvaluation,
                    )| key < *best_key,
                ) {
                    band_best = Some((key, candidate_causal, candidate_states, candidate_exact));
                }
            }
            if coarse {
                model.bands[band].coarse.entropy_parameter = current;
            } else {
                model.bands[band].fine.entropy_parameter = current;
            }
            if let Some((key, candidate_causal, candidate_states, candidate_exact)) = band_best {
                let best_key = (
                    best_exact.total_sse,
                    u64::MAX.saturating_sub(best_causal.total_used_bits),
                    current,
                );
                if key < best_key {
                    if coarse {
                        model.bands[band].coarse.entropy_parameter = key.2;
                    } else {
                        model.bands[band].fine.entropy_parameter = key.2;
                    }
                    best_causal = candidate_causal;
                    best_states = candidate_states;
                    best_exact = candidate_exact;
                    println!(
                        "fixed_rice accept quality={} band={} k={} sse={} psnr_db={:.9}",
                        if coarse { "coarse" } else { "fine" },
                        band_name,
                        key.2,
                        best_exact.total_sse,
                        best_exact.aggregate_psnr,
                    );
                }
            }
        }
    }
    let tables = model_rice_tables(model);
    println!(
        "fixed_rice final fine={:?} coarse={:?} sse={} psnr_db={:.9}",
        tables[0], tables[1], best_exact.total_sse, best_exact.aggregate_psnr,
    );
    (best_causal, best_states, best_exact)
}

#[allow(clippy::too_many_arguments)]
fn tune_context_profiles_exact(
    corpus: &Corpus,
    model: &mut ProfileModel,
    census: &RiceContextCensus,
    profiles: &mut [[RiceContextProfile; BAND_COUNT]; 2],
    weights: &[u32; BAND_COUNT],
    hardware_policy: HardwarePolicy,
    controller: CausalPolicy,
    radius: Option<u8>,
) -> (CausalPolicy, Vec<OracleState>, ExactEvaluation) {
    let energy_shifts = [
        CANONICAL_COARSE_ENERGY_SHIFT,
        CANONICAL_INCREMENTAL_ENERGY_SHIFT,
    ];
    let (mut best_causal, mut best_states, mut best_exact) = evaluate_fixed_profile(
        corpus,
        model,
        weights,
        hardware_policy,
        controller,
        energy_shifts,
    );
    println!(
        "context_exact start radius={radius:?} sse={} psnr_db={:.9}",
        best_exact.total_sse, best_exact.aggregate_psnr,
    );
    let quality = 1;
    for (band, band_name) in BAND_NAMES.iter().enumerate() {
        for parameter in 0..4 {
            let current = profiles[quality][band];
            let center = match parameter {
                0 => current.zero_k,
                1 => current.small_k,
                2 => current.large_k,
                3 => current.threshold_exp,
                _ => unreachable!(),
            };
            let maximum = if parameter == 3 { 2 } else { 4 };
            let first = radius.map_or(0, |value| center.saturating_sub(value));
            let last = radius.map_or(maximum, |value| center.saturating_add(value).min(maximum));
            let mut parameter_best = None;
            let mut candidates = (first..=last).collect::<Vec<_>>();
            // The packet-level proxy usually puts every coordinate within
            // two steps. Keep the maximum large-K escape in the exact
            // stage as a cheap guard for raw-dominated sparse tails.
            if radius.is_some() && parameter == 2 && last != maximum {
                candidates.push(maximum);
            }
            for value in candidates {
                if value == center {
                    continue;
                }
                let mut candidate_profile = current;
                match parameter {
                    0 => candidate_profile.zero_k = value,
                    1 => candidate_profile.small_k = value,
                    2 => candidate_profile.large_k = value,
                    3 => candidate_profile.threshold_exp = value,
                    _ => unreachable!(),
                }
                apply_context_profile(model, census, quality, band, candidate_profile);
                let (candidate_causal, candidate_states, candidate_exact) = evaluate_fixed_profile(
                    corpus,
                    model,
                    weights,
                    hardware_policy,
                    controller,
                    energy_shifts,
                );
                let key = (
                    candidate_exact.total_sse,
                    u64::MAX.saturating_sub(candidate_causal.total_used_bits),
                    value,
                );
                if parameter_best.as_ref().is_none_or(
                    |(best_key, _, _, _): &(
                        (u128, u64, u8),
                        CausalPolicy,
                        Vec<OracleState>,
                        ExactEvaluation,
                    )| key < *best_key,
                ) {
                    parameter_best =
                        Some((key, candidate_causal, candidate_states, candidate_exact));
                }
            }
            apply_context_profile(model, census, quality, band, current);
            if let Some((key, candidate_causal, candidate_states, candidate_exact)) = parameter_best
            {
                let best_key = (
                    best_exact.total_sse,
                    u64::MAX.saturating_sub(best_causal.total_used_bits),
                    center,
                );
                if key < best_key {
                    let mut accepted = current;
                    match parameter {
                        0 => accepted.zero_k = key.2,
                        1 => accepted.small_k = key.2,
                        2 => accepted.large_k = key.2,
                        3 => accepted.threshold_exp = key.2,
                        _ => unreachable!(),
                    }
                    profiles[quality][band] = accepted;
                    apply_context_profile(model, census, quality, band, accepted);
                    best_causal = candidate_causal;
                    best_states = candidate_states;
                    best_exact = candidate_exact;
                    println!(
                        "context_exact accept quality={} band={} parameter={} value={} profile={accepted:?} sse={} psnr_db={:.9}",
                        if quality == 0 { "fine" } else { "coarse" },
                        band_name,
                        ["zero_k", "small_k", "large_k", "threshold_exp"][parameter],
                        key.2,
                        best_exact.total_sse,
                        best_exact.aggregate_psnr,
                    );
                }
            }
        }
    }
    println!(
        "context_exact final fine={:?} coarse={:?} sse={} psnr_db={:.9}",
        profiles[0], profiles[1], best_exact.total_sse, best_exact.aggregate_psnr,
    );
    (best_causal, best_states, best_exact)
}

fn blend_u32(base: u32, fitted: u32, alpha_q8: u16) -> u32 {
    let delta = i64::from(fitted) - i64::from(base);
    (i64::from(base) + (delta * i64::from(alpha_q8) + delta.signum() * 128) / 256)
        .clamp(0, i64::from(u32::MAX)) as u32
}

fn blend_i32(base: i32, fitted: i32, alpha_q8: u16) -> i32 {
    let delta = i64::from(fitted) - i64::from(base);
    (i64::from(base) + (delta * i64::from(alpha_q8) + delta.signum() * 128) / 256)
        .clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

fn blended_affine_tables(
    fitted_gains: &[[u32; BAND_COUNT]; 2],
    fitted_offsets: &[[i32; BAND_COUNT]; 2],
    gain_alpha_q8: [u16; 2],
    offset_alpha_q8: [u16; 2],
) -> ([[u32; BAND_COUNT]; 2], [[i32; BAND_COUNT]; 2]) {
    (
        array::from_fn(|quality| {
            array::from_fn(|band| {
                blend_u32(
                    CANONICAL_RECONSTRUCTION_GAIN_Q8[quality][band],
                    fitted_gains[quality][band],
                    gain_alpha_q8[quality],
                )
            })
        }),
        array::from_fn(|quality| {
            array::from_fn(|band| {
                blend_i32(
                    CANONICAL_RECONSTRUCTION_OFFSET_Q8[quality][band],
                    fitted_offsets[quality][band],
                    offset_alpha_q8[quality],
                )
            })
        }),
    )
}

type AffineTuneResult = (
    [[u32; BAND_COUNT]; 2],
    [[i32; BAND_COUNT]; 2],
    [u16; 2],
    [u16; 2],
    ExactEvaluation,
);

fn tune_decoder_affine_shrinkage(
    corpus: &Corpus,
    model: &ProfileModel,
    states: &[OracleState],
) -> AffineTuneResult {
    const ALPHAS: [u16; 9] = [0, 32, 64, 96, 128, 160, 192, 224, 256];
    let (fitted_gains, fitted_offsets) =
        fit_state_affine_reconstruction(corpus, model, states, None);
    let mut gain_alpha = [0u16; 2];
    let mut offset_alpha = [0u16; 2];
    let (mut gains, mut offsets) =
        blended_affine_tables(&fitted_gains, &fitted_offsets, gain_alpha, offset_alpha);
    let mut best = exact_evaluate_states_with_affine(corpus, model, states, &gains, &offsets);
    for _ in 0..2 {
        for quality in 0..2 {
            for tune_gain in [true, false] {
                let mut choice = None;
                for alpha in ALPHAS {
                    let mut candidate_gain_alpha = gain_alpha;
                    let mut candidate_offset_alpha = offset_alpha;
                    if tune_gain {
                        candidate_gain_alpha[quality] = alpha;
                    } else {
                        candidate_offset_alpha[quality] = alpha;
                    }
                    let (candidate_gains, candidate_offsets) = blended_affine_tables(
                        &fitted_gains,
                        &fitted_offsets,
                        candidate_gain_alpha,
                        candidate_offset_alpha,
                    );
                    let exact = exact_evaluate_states_with_affine(
                        corpus,
                        model,
                        states,
                        &candidate_gains,
                        &candidate_offsets,
                    );
                    let key = (exact.total_sse, alpha);
                    if choice
                        .as_ref()
                        .is_none_or(|(best_key, _, _, _)| key < *best_key)
                    {
                        choice = Some((key, candidate_gains, candidate_offsets, exact));
                    }
                }
                let (key, candidate_gains, candidate_offsets, candidate_exact) =
                    choice.expect("decoder alpha grid is nonempty");
                if key.0 <= best.total_sse {
                    if tune_gain {
                        gain_alpha[quality] = key.1;
                    } else {
                        offset_alpha[quality] = key.1;
                    }
                    gains = candidate_gains;
                    offsets = candidate_offsets;
                    best = candidate_exact;
                }
            }
        }
    }
    println!(
        "decoder_shrinkage gain_alpha_q8={gain_alpha:?} offset_alpha_q8={offset_alpha:?} gains={gains:?} offsets={offsets:?} sse={} psnr_db={:.9}",
        best.total_sse, best.aggregate_psnr,
    );
    (gains, offsets, gain_alpha, offset_alpha, best)
}

fn decoder_shrinkage_leave_one_out(
    corpus: &Corpus,
    model: &ProfileModel,
    states: &[OracleState],
    gain_alpha_q8: [u16; 2],
    offset_alpha_q8: [u16; 2],
) -> (f64, u128) {
    let mut held_out_sse = 0u128;
    for excluded in 0..corpus.images.len() {
        let (fitted_gains, fitted_offsets) =
            fit_state_affine_reconstruction(corpus, model, states, Some(excluded));
        let (gains, offsets) = blended_affine_tables(
            &fitted_gains,
            &fitted_offsets,
            gain_alpha_q8,
            offset_alpha_q8,
        );
        let held_out = exact_evaluate_states_with_affine(corpus, model, states, &gains, &offsets);
        held_out_sse += u128::from(held_out.image_psnr[excluded].2);
    }
    (
        psnr(held_out_sse as f64 / (corpus.images.len() * PIXELS) as f64),
        held_out_sse,
    )
}

fn run_rice_context3_tuning(
    corpus: &Corpus,
    model: &mut ProfileModel,
    weights: &[u32; BAND_COUNT],
    hardware_policy: HardwarePolicy,
    started: Instant,
) {
    let census = RiceContextCensus::build(corpus, model);
    let fixed_profiles = fixed_context_profiles();
    apply_context_profiles(model, &census, &fixed_profiles);
    let fixed_metrics = build_causal_metrics(corpus, model, weights, HardwareMetric::Exact);
    let (controller, _) = simulate_canonical_policy(corpus, &fixed_metrics, hardware_policy, false);
    let energy_shifts = [
        CANONICAL_COARSE_ENERGY_SHIFT,
        CANONICAL_INCREMENTAL_ENERGY_SHIFT,
    ];
    let (fixed_causal, _, baseline) = evaluate_fixed_profile(
        corpus,
        model,
        weights,
        hardware_policy,
        controller,
        energy_shifts,
    );
    println!(
        "context3 baseline sse={} psnr_db={:.15}",
        baseline.total_sse, baseline.aggregate_psnr,
    );

    let proxy_profiles = proxy_tune_context_profiles(&census);
    apply_context_profiles(model, &census, &proxy_profiles);
    let (proxy_causal, _, proxy_exact) = evaluate_fixed_profile(
        corpus,
        model,
        weights,
        hardware_policy,
        controller,
        energy_shifts,
    );
    let canonical_profiles = canonical_context_profiles();
    apply_context_profiles(model, &census, &canonical_profiles);
    let (canonical_causal, _, canonical_exact) = evaluate_fixed_profile(
        corpus,
        model,
        weights,
        hardware_policy,
        controller,
        energy_shifts,
    );
    let fixed_key = exact_controller_key(&fixed_causal, &baseline);
    let proxy_key = exact_controller_key(&proxy_causal, &proxy_exact);
    let canonical_key = exact_controller_key(&canonical_causal, &canonical_exact);
    let mut profiles = if canonical_key <= fixed_key && canonical_key <= proxy_key {
        println!(
            "context3 seed=canonical sse={} psnr_db={:.9}",
            canonical_exact.total_sse, canonical_exact.aggregate_psnr,
        );
        canonical_profiles
    } else if proxy_key < fixed_key {
        println!(
            "context3 seed=proxy proxy_sse={} proxy_psnr_db={:.9}",
            proxy_exact.total_sse, proxy_exact.aggregate_psnr,
        );
        proxy_profiles
    } else {
        println!(
            "context3 seed=fixed proxy_sse={} proxy_psnr_db={:.9}",
            proxy_exact.total_sse, proxy_exact.aggregate_psnr,
        );
        apply_context_profiles(model, &census, &fixed_profiles);
        fixed_profiles
    };
    apply_context_profiles(model, &census, &profiles);

    let (context_causal, _, _) = tune_context_profiles_exact(
        corpus,
        model,
        &census,
        &mut profiles,
        weights,
        hardware_policy,
        controller,
        Some(2),
    );
    let metrics = build_causal_metrics(corpus, model, weights, HardwareMetric::Exact);
    let (controller, _) = tune_hardware_controller_exact_rounds(
        corpus,
        model,
        &metrics,
        hardware_policy,
        CANONICAL_COARSE_ENERGY_SHIFT,
        CANONICAL_INCREMENTAL_ENERGY_SHIFT,
        context_causal,
        1,
    );
    let (causal, _states, exact) = tune_context_profiles_exact(
        corpus,
        model,
        &census,
        &mut profiles,
        weights,
        hardware_policy,
        controller,
        Some(1),
    );
    // Keep reconstruction fixed while tuning the encoder-side entropy and
    // controller coordinates; decoder calibration is an independent pass.
    let gains = CANONICAL_RECONSTRUCTION_GAIN_Q8;
    let offsets = CANONICAL_RECONSTRUCTION_OFFSET_Q8;
    let zero_k = profiles.map(|quality| quality.map(|profile| profile.zero_k));
    let small_k = profiles.map(|quality| quality.map(|profile| profile.small_k));
    let large_k = profiles.map(|quality| quality.map(|profile| profile.large_k));
    let threshold_exp = profiles.map(|quality| quality.map(|profile| profile.threshold_exp));
    let elapsed = started.elapsed().as_secs_f64();
    println!(
        "context3 zero_k fine={:?} coarse={:?}",
        zero_k[0], zero_k[1]
    );
    println!(
        "context3 small_k fine={:?} coarse={:?}",
        small_k[0], small_k[1]
    );
    println!(
        "context3 large_k fine={:?} coarse={:?}",
        large_k[0], large_k[1]
    );
    println!(
        "context3 threshold_exp fine={:?} coarse={:?}",
        threshold_exp[0], threshold_exp[1]
    );
    println!("context3 reconstruction_gains={gains:?}");
    println!("context3 reconstruction_offsets={offsets:?}");
    println!(
        "context3 result nominal={:?} coarse_cutoff={} incremental_cutoff={} debt_shift={} used_bits={} frame_bits_min={} frame_bits_max={} sse={} psnr_db={:.15} gain_db={:.15} elapsed_s={elapsed:.3}",
        causal.nominal_bits,
        causal.cutoff_base,
        causal.incremental_cutoff_base,
        causal.debt_shift,
        causal.total_used_bits,
        causal.min_frame_bits,
        causal.max_frame_bits,
        exact.total_sse,
        exact.aggregate_psnr,
        exact.aggregate_psnr - baseline.aggregate_psnr,
    );
}

fn run_fixed_rice_tuning(
    corpus: &Corpus,
    model: &mut ProfileModel,
    weights: &[u32; BAND_COUNT],
    mut lambda: f64,
    hardware_policy: HardwarePolicy,
    started: Instant,
) {
    let initial_tables = model_rice_tables(model);
    let rate_tables = rate_optimal_rice_tables(model);
    println!(
        "fixed_rice rate_optimal fine={:?} coarse={:?}",
        rate_tables[0], rate_tables[1],
    );
    let canonical_metrics = build_causal_metrics(corpus, model, weights, HardwareMetric::Exact);
    let (canonical_controller, _) =
        simulate_canonical_policy(corpus, &canonical_metrics, hardware_policy, false);
    let energy_shifts = [
        CANONICAL_COARSE_ENERGY_SHIFT,
        CANONICAL_INCREMENTAL_ENERGY_SHIFT,
    ];
    let (_, _, canonical_exact) = evaluate_fixed_profile(
        corpus,
        model,
        weights,
        hardware_policy,
        canonical_controller,
        energy_shifts,
    );
    set_model_rice_tables(model, rate_tables);
    let (_, _, rate_exact) = evaluate_fixed_profile(
        corpus,
        model,
        weights,
        hardware_policy,
        canonical_controller,
        energy_shifts,
    );
    if rate_exact.total_sse >= canonical_exact.total_sse {
        set_model_rice_tables(model, initial_tables);
    }
    println!(
        "fixed_rice seed={} canonical_sse={} rate_sse={}",
        if rate_exact.total_sse < canonical_exact.total_sse {
            "rate_optimal"
        } else {
            "canonical"
        },
        canonical_exact.total_sse,
        rate_exact.total_sse,
    );

    let (_, _, _) = tune_fixed_rice_exact(
        corpus,
        model,
        weights,
        hardware_policy,
        canonical_controller,
        energy_shifts,
        None,
    );
    let mut metrics = build_causal_metrics(corpus, model, weights, HardwareMetric::Exact);
    let proxy_controller = fit_hardware_controller(corpus, &metrics, hardware_policy);
    let (canonical_causal, canonical_states, canonical_controller_exact) =
        evaluate_hardware_energy_shifts(
            corpus,
            model,
            &metrics,
            hardware_policy,
            canonical_controller,
            energy_shifts,
        );
    let (proxy_causal, proxy_states, proxy_exact) = evaluate_hardware_energy_shifts(
        corpus,
        model,
        &metrics,
        hardware_policy,
        proxy_controller,
        energy_shifts,
    );
    let (controller, _states) = if exact_controller_key(&proxy_causal, &proxy_exact)
        < exact_controller_key(&canonical_causal, &canonical_controller_exact)
    {
        (proxy_causal, proxy_states)
    } else {
        (canonical_causal, canonical_states)
    };
    println!(
        "fixed_rice controller_seed={} canonical_sse={} proxy_sse={}",
        if proxy_exact.total_sse < canonical_controller_exact.total_sse {
            "proxy"
        } else {
            "canonical"
        },
        canonical_controller_exact.total_sse,
        proxy_exact.total_sse,
    );

    let (coarse_energy_shift, incremental_energy_shift, energy_causal, _) =
        tune_hardware_energy_shifts(corpus, model, &metrics, hardware_policy, controller);
    let selected_energy = [coarse_energy_shift, incremental_energy_shift];
    let (controller, _) = tune_hardware_controller_exact(
        corpus,
        model,
        &metrics,
        hardware_policy,
        coarse_energy_shift,
        incremental_energy_shift,
        energy_causal,
    );
    let (_, _, _) = tune_fixed_rice_exact(
        corpus,
        model,
        weights,
        hardware_policy,
        controller,
        selected_energy,
        Some(1),
    );
    metrics = build_causal_metrics(corpus, model, weights, HardwareMetric::Exact);
    let (causal, states) = tune_hardware_controller_exact(
        corpus,
        model,
        &metrics,
        hardware_policy,
        coarse_energy_shift,
        incremental_energy_shift,
        controller,
    );
    let (gains, offsets, gain_alpha, offset_alpha, exact) =
        tune_decoder_affine_shrinkage(corpus, model, &states);
    let (loo_psnr, loo_sse) =
        decoder_shrinkage_leave_one_out(corpus, model, &states, gain_alpha, offset_alpha);
    println!("fixed_rice leave_one_out_psnr_db={loo_psnr:.9} total_sse={loo_sse}");

    let target_bits = (corpus.images.len() * FRAME_BITS) as u64;
    lambda = tune_fixed_lambda(corpus, model, weights, target_bits, lambda);
    let oracle = oracle_decisions(corpus, model, weights, lambda);
    let (oracle_gains, oracle_offsets) =
        fit_state_affine_reconstruction(corpus, model, &oracle.states, None);
    let oracle_exact = exact_evaluate_states_with_affine(
        corpus,
        model,
        &oracle.states,
        &oracle_gains,
        &oracle_offsets,
    );
    let elapsed = started.elapsed().as_secs_f64();
    emit_tables(Report {
        model,
        context_profiles: None,
        distortion_weight: *weights,
        nominal_bits: causal.nominal_bits,
        lambda,
        causal,
        oracle_bits: oracle.total_bits,
        oracle_exact: &oracle_exact,
        exact: &exact,
        reconstruction_gains: gains,
        reconstruction_offsets: offsets,
        coarse_energy_shift,
        incremental_energy_shift,
        elapsed,
    });
    emit_adversarial_bounds(&exact);
    println!(
        "fixed_rice quality_gate_db=34.690000 pass={} margin_db={:.9}",
        exact.aggregate_psnr >= 34.69,
        exact.aggregate_psnr - 34.69,
    );
}

fn simulate_causal(
    corpus: &Corpus,
    metrics: &[CausalMetric],
    nominal_bits: [u16; BAND_COUNT],
    cutoff_base: u8,
    debt_shift: u8,
    keep_states: bool,
) -> (CausalPolicy, Option<Vec<OracleState>>) {
    let mut total_used_bits = 0u64;
    let mut proxy_distortion = 0u128;
    let mut nonzero_groups = 0u64;
    let mut min_frame_bits = usize::MAX;
    let mut max_frame_bits = 0usize;
    let mut max_nonzero_groups = 0usize;
    let mut min_debt = 0i64;
    let mut max_debt = 0i64;
    let mut output_states = keep_states.then(|| Vec::with_capacity(metrics.len()));
    for image in 0..corpus.images.len() {
        let mut debt = 0i64;
        let mut room = FRAME_BITS - GROUP_COUNT;
        let mut frame_bits = 0usize;
        let mut frame_nonzero_groups = 0usize;
        let mut pending_zero_run = 0usize;
        for group in 0..GROUP_COUNT {
            let index = image * GROUP_COUNT + group;
            let metric = metrics[index];
            let band = usize::from(corpus.schedule[group]);
            let pre_debt = debt - i64::from(nominal_bits[band]);
            let cutoff = (i64::from(cutoff_base) + (pre_debt >> debt_shift)).clamp(0, 31) as u8;
            let coarse_allowed = metric.coarse_slope >= cutoff
                && usize::from(metric.coarse_bits.saturating_sub(1)) <= room;
            let fine_fits = usize::from(metric.fine_bits.saturating_sub(1)) <= room;
            // Walk the convex three-state R-D hull.  Fine is charged for its
            // incremental coarse->fine bits when coarse is useful; the direct
            // zero->fine segment remains available when coarse is dominated.
            let state = if coarse_allowed {
                if metric.fine_slope >= cutoff && fine_fits {
                    OracleState::Fine
                } else {
                    OracleState::Coarse
                }
            } else if metric.fine_direct_slope >= cutoff && fine_fits {
                OracleState::Fine
            } else {
                OracleState::Zero
            };
            let (bits, distortion) = match state {
                OracleState::Fine => (metric.fine_bits, metric.fine_distortion),
                OracleState::Coarse => (metric.coarse_bits, metric.coarse_distortion),
                OracleState::Zero => (1, metric.zero_distortion),
            };
            room -= usize::from(bits - 1);
            debt = pre_debt + i64::from(bits);
            total_used_bits += u64::from(bits);
            frame_bits += usize::from(bits);
            if advance_zero_run(&mut pending_zero_run, matches!(state, OracleState::Zero)) {
                room += 1;
                debt -= 1;
                total_used_bits -= 1;
                frame_bits -= 1;
            }
            min_debt = min_debt.min(debt);
            max_debt = max_debt.max(debt);
            proxy_distortion += u128::from(distortion) * u128::from(metric.weight);
            frame_nonzero_groups += usize::from(bits > 1);
            nonzero_groups += u64::from(bits > 1);
            if let Some(states) = &mut output_states {
                states.push(state);
            }
        }
        min_frame_bits = min_frame_bits.min(frame_bits);
        max_frame_bits = max_frame_bits.max(frame_bits);
        max_nonzero_groups = max_nonzero_groups.max(frame_nonzero_groups);
    }
    (
        CausalPolicy {
            cutoff_base,
            incremental_cutoff_base: cutoff_base,
            debt_shift,
            nominal_bits,
            total_used_bits,
            proxy_distortion,
            nonzero_groups,
            min_frame_bits,
            max_frame_bits,
            max_nonzero_groups,
            min_debt,
            max_debt,
        },
        output_states,
    )
}

fn fit_causal_policy(
    corpus: &Corpus,
    metrics: &[CausalMetric],
    nominal_seed: [u16; BAND_COUNT],
) -> CausalPolicy {
    let mut best = None::<CausalPolicy>;
    // Integer per-group charges and five-bit slopes move the causal operating
    // point slightly away from the oracle.  Fit one global Q8 scale rather
    // than independently perturbing twelve budgets: that keeps the search
    // ordered, fast, and much harder to overfit.
    for nominal_scale_q8 in [224u16, 240, 256, 272, 288, 304, 320] {
        let nominal_bits = nominal_seed.map(|value| {
            ((u32::from(value) * u32::from(nominal_scale_q8) + 128) >> 8)
                .clamp(1, u32::from(COARSE_RAW_PACKET_BITS)) as u16
        });
        for debt_shift in 10..=18 {
            for cutoff_base in 0..=24 {
                let candidate = simulate_causal(
                    corpus,
                    metrics,
                    nominal_bits,
                    cutoff_base,
                    debt_shift,
                    false,
                )
                .0;
                if best.is_none_or(|current| {
                    (
                        candidate.proxy_distortion,
                        u64::MAX - candidate.total_used_bits,
                    ) < (current.proxy_distortion, u64::MAX - current.total_used_bits)
                }) {
                    best = Some(candidate);
                }
            }
        }
    }
    best.expect("causal policy grid is nonempty")
}

#[derive(Clone, Debug)]
struct ExactEvaluation {
    total_sse: u128,
    aggregate_psnr: f64,
    mean_psnr: f64,
    image_psnr: Vec<(String, f64, u64)>,
    max_nonzero_groups: usize,
}

fn exact_evaluate_states_with_affine(
    corpus: &Corpus,
    model: &ProfileModel,
    states: &[OracleState],
    gains_q8: &[[u32; BAND_COUNT]; 2],
    offsets_q8: &[[i32; BAND_COUNT]; 2],
) -> ExactEvaluation {
    exact_evaluate_states_internal(corpus, model, states, gains_q8, offsets_q8)
}

fn exact_evaluate_states_internal(
    corpus: &Corpus,
    model: &ProfileModel,
    states: &[OracleState],
    gains_q8: &[[u32; BAND_COUNT]; 2],
    offsets_q8: &[[i32; BAND_COUNT]; 2],
) -> ExactEvaluation {
    assert_eq!(states.len(), corpus.images.len() * GROUP_COUNT);
    let band_list = bands(WIDTH, HEIGHT);
    let (sender, receiver) = mpsc::channel();
    std::thread::scope(|scope| {
        for image in 0..corpus.images.len() {
            let sender = sender.clone();
            let image_states = &states[image * GROUP_COUNT..(image + 1) * GROUP_COUNT];
            let band_list = &band_list;
            scope.spawn(move || {
                let cached = &corpus.images[image];
                let mut coefficients = vec![0i32; PIXELS];
                let mut band_groups = [0usize; BAND_COUNT];
                let mut nonzero = 0usize;
                for (group, &image_state) in image_states.iter().enumerate() {
                    let band = usize::from(corpus.schedule[group]);
                    let ordinal = band_groups[band];
                    band_groups[band] += 1;
                    let (spec, quality) = match image_state {
                        OracleState::Zero => continue,
                        OracleState::Fine => (model.bands[band].fine.spec, 0),
                        OracleState::Coarse => (model.bands[band].coarse.spec, 1),
                    };
                    let rect = band_list[band].rect;
                    let mut group_nonzero = false;
                    for lane in 0..GROUP_LANES {
                        let flat = ordinal * GROUP_LANES + lane;
                        let index =
                            (rect.y + flat / rect.width) * WIDTH + rect.x + flat % rect.width;
                        let quantized = quantize(cached.groups[group][lane], spec);
                        group_nonzero |= quantized != 0;
                        coefficients[index] = reconstruct_with_affine(
                            quantized,
                            gains_q8[quality][band],
                            offsets_q8[quality][band],
                        );
                    }
                    nonzero += usize::from(group_nonzero);
                }
                inverse_transform(&mut coefficients, WIDTH, HEIGHT);
                let sse = cached
                    .pixels
                    .iter()
                    .zip(coefficients)
                    .map(|(&original, reconstructed)| {
                        let error = i64::from(original) - i64::from(reconstructed.clamp(0, 255));
                        (error * error) as u64
                    })
                    .sum::<u64>();
                sender.send((image, sse, nonzero)).unwrap();
            });
        }
        drop(sender);
    });
    let mut results = receiver.into_iter().collect::<Vec<_>>();
    results.sort_by_key(|&(image, _, _)| image);
    let total_sse = results.iter().map(|&(_, sse, _)| u128::from(sse)).sum();
    let samples = corpus.images.len() * PIXELS;
    let aggregate_psnr = psnr(total_sse as f64 / samples as f64);
    let image_psnr = results
        .iter()
        .map(|&(image, sse, _)| {
            (
                corpus.images[image].name.clone(),
                psnr(sse as f64 / PIXELS as f64),
                sse,
            )
        })
        .collect::<Vec<_>>();
    let mean_psnr =
        image_psnr.iter().map(|(_, value, _)| value).sum::<f64>() / image_psnr.len() as f64;
    let max_nonzero_groups = results
        .iter()
        .map(|&(_, _, nonzero)| nonzero)
        .max()
        .unwrap_or(0);
    ExactEvaluation {
        total_sse,
        aggregate_psnr,
        mean_psnr,
        image_psnr,
        max_nonzero_groups,
    }
}

fn reconstruct_with_affine(value: i16, gain_q8: u32, offset_q8: i32) -> i32 {
    if value == 0 {
        return 0;
    }
    let magnitude = i64::from(value.unsigned_abs()) * i64::from(gain_q8) + i64::from(offset_q8);
    let reconstructed = ((magnitude.max(0) + 128) >> 8) as i32;
    if value < 0 {
        -reconstructed
    } else {
        reconstructed
    }
}

fn tune_exact_affine_reconstruction(
    corpus: &Corpus,
    model: &ProfileModel,
    states: &[OracleState],
    mut gains: [[u32; BAND_COUNT]; 2],
    mut offsets: [[i32; BAND_COUNT]; 2],
    excluded_image: Option<usize>,
    passes: usize,
) -> (
    [[u32; BAND_COUNT]; 2],
    [[i32; BAND_COUNT]; 2],
    ExactEvaluation,
) {
    // Coefficient-domain least squares is a useful seed, but the integer 5/3
    // synthesis and output clamp make pixel SSE slightly different. Keep the
    // search bounded: ordered scalar coordinates, fixed step ladders, and
    // only accepted exact inverse-transform improvements.
    let mut best = exact_evaluate_states_with_affine(corpus, model, states, &gains, &offsets);
    let objective = |evaluation: &ExactEvaluation| {
        evaluation
            .image_psnr
            .iter()
            .enumerate()
            .filter(|(image, _)| Some(*image) != excluded_image)
            .map(|(_, (_, _, sse))| u128::from(*sse))
            .sum::<u128>()
    };
    let mut best_objective = objective(&best);
    for pass in 0..passes {
        for quality in 0..2 {
            for band in 0..BAND_COUNT {
                for step in [32u32, 8, 2, 1] {
                    let baseline = gains[quality][band];
                    let mut selected = baseline;
                    let mut selected_eval = best.clone();
                    let mut selected_objective = best_objective;
                    for candidate in [baseline.saturating_sub(step), baseline.saturating_add(step)]
                    {
                        gains[quality][band] = candidate;
                        let trial = exact_evaluate_states_with_affine(
                            corpus, model, states, &gains, &offsets,
                        );
                        let trial_objective = objective(&trial);
                        if trial_objective < selected_objective {
                            selected = candidate;
                            selected_eval = trial;
                            selected_objective = trial_objective;
                        }
                    }
                    gains[quality][band] = selected;
                    best = selected_eval;
                    best_objective = selected_objective;
                }
                for step in [128i32, 32, 8, 2, 1] {
                    let baseline = offsets[quality][band];
                    let mut selected = baseline;
                    let mut selected_eval = best.clone();
                    let mut selected_objective = best_objective;
                    for candidate in [baseline.saturating_sub(step), baseline.saturating_add(step)]
                    {
                        offsets[quality][band] = candidate;
                        let trial = exact_evaluate_states_with_affine(
                            corpus, model, states, &gains, &offsets,
                        );
                        let trial_objective = objective(&trial);
                        if trial_objective < selected_objective {
                            selected = candidate;
                            selected_eval = trial;
                            selected_objective = trial_objective;
                        }
                    }
                    offsets[quality][band] = selected;
                    best = selected_eval;
                    best_objective = selected_objective;
                }
            }
        }
        println!(
            "exact_reconstruction excluded={excluded_image:?} pass={} aggregate_psnr_db={:.15} total_sse={} objective_sse={best_objective}",
            pass + 1,
            best.aggregate_psnr,
            best.total_sse,
        );
    }
    (gains, offsets, best)
}

fn rounded_divide(numerator: i128, denominator: i128) -> i128 {
    debug_assert!(denominator > 0);
    if numerator < 0 {
        -((-numerator + denominator / 2) / denominator)
    } else {
        (numerator + denominator / 2) / denominator
    }
}

fn fit_state_affine_reconstruction(
    corpus: &Corpus,
    model: &ProfileModel,
    states: &[OracleState],
    excluded_image: Option<usize>,
) -> ([[u32; BAND_COUNT]; 2], [[i32; BAND_COUNT]; 2]) {
    let fallback = fit_state_reconstruction_gains(corpus, model, states, excluded_image);
    let mut count = [[0i128; BAND_COUNT]; 2];
    let mut sum_x = [[0i128; BAND_COUNT]; 2];
    let mut sum_y = [[0i128; BAND_COUNT]; 2];
    let mut sum_xx = [[0i128; BAND_COUNT]; 2];
    let mut sum_xy = [[0i128; BAND_COUNT]; 2];
    for image in 0..corpus.images.len() {
        if excluded_image == Some(image) {
            continue;
        }
        for group in 0..GROUP_COUNT {
            let band = usize::from(corpus.schedule[group]);
            let (quality, spec) = match states[image * GROUP_COUNT + group] {
                OracleState::Zero => continue,
                OracleState::Fine => (0, model.bands[band].fine.spec),
                OracleState::Coarse => (1, model.bands[band].coarse.spec),
            };
            for &coefficient in &corpus.images[image].groups[group] {
                let q = quantize(coefficient, spec);
                if q == 0 {
                    continue;
                }
                let x = i128::from(q.unsigned_abs());
                let y = i128::from(coefficient.unsigned_abs());
                count[quality][band] += 1;
                sum_x[quality][band] += x;
                sum_y[quality][band] += y;
                sum_xx[quality][band] += x * x;
                sum_xy[quality][band] += x * y;
            }
        }
    }
    let mut gains = fallback;
    let mut offsets = [[0i32; BAND_COUNT]; 2];
    for quality in 0..2 {
        for band in 0..BAND_COUNT {
            let n = count[quality][band];
            let denominator =
                n * sum_xx[quality][band] - sum_x[quality][band] * sum_x[quality][band];
            if denominator == 0 {
                continue;
            }
            let gain = rounded_divide(
                (n * sum_xy[quality][band] - sum_x[quality][band] * sum_y[quality][band]) * 256,
                denominator,
            )
            .clamp(0, i128::from(u32::MAX));
            let offset =
                rounded_divide(sum_y[quality][band] * 256 - gain * sum_x[quality][band], n)
                    .clamp(i128::from(i32::MIN), i128::from(i32::MAX));
            gains[quality][band] = gain as u32;
            offsets[quality][band] = offset as i32;
        }
    }
    (gains, offsets)
}

fn fit_state_reconstruction_gains(
    corpus: &Corpus,
    model: &ProfileModel,
    states: &[OracleState],
    excluded_image: Option<usize>,
) -> [[u32; BAND_COUNT]; 2] {
    let mut coefficient_times_q = [[0i128; BAND_COUNT]; 2];
    let mut q_squared = [[0u128; BAND_COUNT]; 2];
    for image in 0..corpus.images.len() {
        if excluded_image == Some(image) {
            continue;
        }
        for group in 0..GROUP_COUNT {
            let state = states[image * GROUP_COUNT + group];
            let band = usize::from(corpus.schedule[group]);
            let (quality, spec) = match state {
                OracleState::Zero => continue,
                OracleState::Fine => (0, model.bands[band].fine.spec),
                OracleState::Coarse => (1, model.bands[band].coarse.spec),
            };
            for &coefficient in &corpus.images[image].groups[group] {
                let q = quantize(coefficient, spec);
                coefficient_times_q[quality][band] += i128::from(coefficient) * i128::from(q);
                q_squared[quality][band] += i128::from(q).unsigned_abs().pow(2);
            }
        }
    }
    array::from_fn(|quality| {
        array::from_fn(|band| {
            let denominator = q_squared[quality][band];
            if denominator == 0 {
                let shift = if quality == 0 {
                    model.bands[band].fine.spec.shift
                } else {
                    model.bands[band].coarse.spec.shift
                };
                256u32 << shift
            } else {
                let numerator = coefficient_times_q[quality][band].max(0) as u128 * 256;
                ((numerator + denominator / 2) / denominator).min(u128::from(u32::MAX)) as u32
            }
        })
    })
}

fn psnr(mse: f64) -> f64 {
    if mse == 0.0 {
        f64::INFINITY
    } else {
        10.0 * ((255.0 * 255.0) / mse).log10()
    }
}

fn measure_synthesis_diagonal() -> ([f64; BAND_COUNT], [u32; BAND_COUNT]) {
    const PROBE_WIDTH: usize = 128;
    const PROBE_HEIGHT: usize = 64;
    const AMPLITUDE: i32 = 1024;
    const PROBES: usize = 12;
    let band_list = bands(PROBE_WIDTH, PROBE_HEIGHT);
    let gain = array::from_fn(|band| {
        let rect = band_list[band].rect;
        let area = rect.width * rect.height;
        let probes = PROBES.min(area);
        let mut total = 0.0;
        for probe in 0..probes {
            let flat = ((2 * probe + 1) * area / (2 * probes)).min(area - 1);
            for sign in [-1, 1] {
                let mut coefficients = vec![0i32; PROBE_WIDTH * PROBE_HEIGHT];
                let index = (rect.y + flat / rect.width) * PROBE_WIDTH + rect.x + flat % rect.width;
                coefficients[index] = sign * AMPLITUDE;
                inverse_transform(&mut coefficients, PROBE_WIDTH, PROBE_HEIGHT);
                let energy = coefficients
                    .iter()
                    .map(|&sample| {
                        let delta = i64::from(sample - 128);
                        (delta * delta) as f64
                    })
                    .sum::<f64>();
                total += energy / f64::from(AMPLITUDE * AMPLITUDE);
            }
        }
        total / (2 * probes) as f64
    });
    let minimum = gain.iter().copied().fold(f64::INFINITY, f64::min);
    let weights = gain.map(|value| {
        let exponent = (value / minimum).log2().round().clamp(0.0, 15.0) as u32;
        1u32 << exponent
    });
    (gain, weights)
}

struct Report<'a> {
    model: &'a ProfileModel,
    context_profiles: Option<[[RiceContextProfile; BAND_COUNT]; 2]>,
    distortion_weight: [u32; BAND_COUNT],
    nominal_bits: [u16; BAND_COUNT],
    lambda: f64,
    causal: CausalPolicy,
    oracle_bits: u64,
    oracle_exact: &'a ExactEvaluation,
    exact: &'a ExactEvaluation,
    reconstruction_gains: [[u32; BAND_COUNT]; 2],
    reconstruction_offsets: [[i32; BAND_COUNT]; 2],
    coarse_energy_shift: [u8; BAND_COUNT],
    incremental_energy_shift: [u8; BAND_COUNT],
    elapsed: f64,
}

fn emit_tables(report: Report<'_>) {
    let Report {
        model,
        context_profiles,
        distortion_weight,
        nominal_bits,
        lambda,
        causal,
        oracle_bits,
        oracle_exact,
        exact,
        reconstruction_gains,
        reconstruction_offsets,
        coarse_energy_shift,
        incremental_energy_shift,
        elapsed,
    } = report;
    let fine_shift = model.bands.each_ref().map(|band| band.fine.spec.shift);
    let coarse_shift = model.bands.each_ref().map(|band| band.coarse.spec.shift);
    let fine_deadzone = model
        .bands
        .each_ref()
        .map(|band| band.fine.spec.deadzone_q8);
    let coarse_deadzone = model
        .bands
        .each_ref()
        .map(|band| band.coarse.spec.deadzone_q8);
    let fine_centroid = model
        .bands
        .each_ref()
        .map(|band| band.fine.spec.centroid_q8);
    let coarse_centroid = model
        .bands
        .each_ref()
        .map(|band| band.coarse.spec.centroid_q8);
    let fine_rice = model
        .bands
        .each_ref()
        .map(|band| band.fine.entropy_parameter);
    let coarse_rice = model
        .bands
        .each_ref()
        .map(|band| band.coarse.entropy_parameter);
    let context_tables = context_profiles.map(|profiles| {
        (
            profiles.map(|quality| quality.map(|profile| profile.zero_k)),
            profiles.map(|quality| quality.map(|profile| profile.small_k)),
            profiles.map(|quality| quality.map(|profile| profile.large_k)),
            profiles.map(|quality| quality.map(|profile| profile.threshold_exp)),
        )
    });
    let fine_reconstruction_gain_q8 = reconstruction_gains[0];
    let coarse_reconstruction_gain_q8 = reconstruction_gains[1];
    let fine_reconstruction_offset_q8 = reconstruction_offsets[0];
    let coarse_reconstruction_offset_q8 = reconstruction_offsets[1];

    println!("BEGIN_LWC_TUNING_TABLES");
    println!("const FINE_SHIFT: [u8; 12] = {fine_shift:?};");
    println!("const COARSE_SHIFT: [u8; 12] = {coarse_shift:?};");
    println!("const FINE_DEADZONE_Q8: [u16; 12] = {fine_deadzone:?};");
    println!("const COARSE_DEADZONE_Q8: [u16; 12] = {coarse_deadzone:?};");
    println!("const FINE_CENTROID_Q8: [u16; 12] = {fine_centroid:?};");
    println!("const COARSE_CENTROID_Q8: [u16; 12] = {coarse_centroid:?};");
    if let Some((zero, small, large, threshold_exp)) = context_tables {
        println!("const FINE_RICE_ZERO: [u8; 12] = {:?};", zero[0]);
        println!("const FINE_RICE_SMALL: [u8; 12] = {:?};", small[0]);
        println!("const FINE_RICE_LARGE: [u8; 12] = {:?};", large[0]);
        println!(
            "const FINE_RICE_THRESHOLD_EXP: [u8; 12] = {:?};",
            threshold_exp[0]
        );
        println!("const COARSE_RICE_ZERO: [u8; 12] = {:?};", zero[1]);
        println!("const COARSE_RICE_SMALL: [u8; 12] = {:?};", small[1]);
        println!("const COARSE_RICE_LARGE: [u8; 12] = {:?};", large[1]);
        println!(
            "const COARSE_RICE_THRESHOLD_EXP: [u8; 12] = {:?};",
            threshold_exp[1]
        );
    } else {
        println!("const FINE_RICE: [u8; 12] = {fine_rice:?};");
        println!("const COARSE_RICE: [u8; 12] = {coarse_rice:?};");
    }
    println!("const COARSE_ENERGY_SHIFT: [u8; 12] = {coarse_energy_shift:?};");
    println!("const INCREMENTAL_ENERGY_SHIFT: [u8; 12] = {incremental_energy_shift:?};");
    println!("const FINE_RECONSTRUCTION_GAIN_Q8: [u32; 12] = {fine_reconstruction_gain_q8:?};");
    println!("const COARSE_RECONSTRUCTION_GAIN_Q8: [u32; 12] = {coarse_reconstruction_gain_q8:?};");
    println!("const FINE_RECONSTRUCTION_OFFSET_Q8: [i32; 12] = {fine_reconstruction_offset_q8:?};");
    println!(
        "const COARSE_RECONSTRUCTION_OFFSET_Q8: [i32; 12] = {coarse_reconstruction_offset_q8:?};"
    );
    println!("const NOMINAL_BITS: [u16; 12] = {nominal_bits:?};");
    println!("const DISTORTION_WEIGHT: [u32; 12] = {distortion_weight:?};");
    println!(
        "const RATE_COARSE_CUTOFF_BASE: u8 = {};",
        causal.cutoff_base
    );
    println!(
        "const RATE_INCREMENTAL_CUTOFF_BASE: u8 = {};",
        causal.incremental_cutoff_base
    );
    println!("const RATE_DEBT_SHIFT: u8 = {};", causal.debt_shift);
    println!("END_LWC_TUNING_TABLES");

    println!(
        "oracle aggregate_psnr_db={:.9} mean_psnr_db={:.9} used_bits={} used_bpp={:.9}",
        oracle_exact.aggregate_psnr,
        oracle_exact.mean_psnr,
        oracle_bits,
        oracle_bits as f64 / (oracle_exact.image_psnr.len() * PIXELS) as f64,
    );

    for (name, image_psnr, sse) in &exact.image_psnr {
        println!("exact image={name} psnr_db={image_psnr:.6} sse={sse}");
    }
    println!(
        "exact aggregate_psnr_db={:.15} mean_psnr_db={:.9} total_sse={} physical_bpp=1.000000 used_bits={} used_bpp={:.9}",
        exact.aggregate_psnr,
        exact.mean_psnr,
        exact.total_sse,
        causal.total_used_bits,
        causal.total_used_bits as f64 / (exact.image_psnr.len() * PIXELS) as f64,
    );
    println!(
        "causal proxy_distortion={} nonzero_groups={} frame_bits_min={} frame_bits_max={} max_nonzero_groups={} debt_min={} debt_max={} lambda={lambda:.12}",
        causal.proxy_distortion,
        causal.nonzero_groups,
        causal.min_frame_bits,
        causal.max_frame_bits,
        causal.max_nonzero_groups,
        causal.min_debt,
        causal.max_debt,
    );

    let mut json = String::new();
    let entropy_json = if let Some((zero, small, large, threshold_exp)) = context_tables {
        format!(
            "\"fine_rice_zero\":{:?},\"fine_rice_small\":{:?},\"fine_rice_large\":{:?},\"fine_rice_threshold_exp\":{:?},\"coarse_rice_zero\":{:?},\"coarse_rice_small\":{:?},\"coarse_rice_large\":{:?},\"coarse_rice_threshold_exp\":{:?}",
            zero[0],
            small[0],
            large[0],
            threshold_exp[0],
            zero[1],
            small[1],
            large[1],
            threshold_exp[1]
        )
    } else {
        format!("\"fine_rice\":{fine_rice:?},\"coarse_rice\":{coarse_rice:?}")
    };
    write!(
        json,
        "{{\"fine_shift\":{fine_shift:?},\"coarse_shift\":{coarse_shift:?},\"fine_deadzone_q8\":{fine_deadzone:?},\"coarse_deadzone_q8\":{coarse_deadzone:?},\"fine_centroid_q8\":{fine_centroid:?},\"coarse_centroid_q8\":{coarse_centroid:?},{entropy_json},\"coarse_energy_shift\":{coarse_energy_shift:?},\"incremental_energy_shift\":{incremental_energy_shift:?},\"fine_reconstruction_gain_q8\":{fine_reconstruction_gain_q8:?},\"coarse_reconstruction_gain_q8\":{coarse_reconstruction_gain_q8:?},\"fine_reconstruction_offset_q8\":{fine_reconstruction_offset_q8:?},\"coarse_reconstruction_offset_q8\":{coarse_reconstruction_offset_q8:?},\"nominal_bits\":{nominal_bits:?},\"distortion_weight\":{distortion_weight:?},\"rate_coarse_cutoff_base\":{},\"rate_incremental_cutoff_base\":{},\"rate_debt_shift\":{},\"oracle_psnr_db\":{:.9},\"oracle_used_bits\":{},\"aggregate_psnr_db\":{:.15},\"mean_psnr_db\":{:.9},\"used_bits\":{},\"frame_bits_min\":{},\"frame_bits_max\":{},\"max_nonzero_groups\":{},\"physical_bits\":{},\"elapsed_seconds\":{elapsed:.6}}}",
        causal.cutoff_base,
        causal.incremental_cutoff_base,
        causal.debt_shift,
        oracle_exact.aggregate_psnr,
        oracle_bits,
        exact.aggregate_psnr,
        exact.mean_psnr,
        causal.total_used_bits,
        causal.min_frame_bits,
        causal.max_frame_bits,
        causal.max_nonzero_groups,
        exact.image_psnr.len() * FRAME_BITS,
    )
    .unwrap();
    println!("LWC_TUNING_JSON={json}");
    println!(
        "runtime elapsed_s={elapsed:.3} target_s=60 target_met={}",
        elapsed < 60.0
    );
}

fn emit_adversarial_bounds(exact: &ExactEvaluation) {
    const TRANSFORM_FRAME_CYCLES: usize = PIXELS / 2;
    const MAX_NONZERO_FOR_CADENCE: usize = (TRANSFORM_FRAME_CYCLES - GROUP_COUNT) / (9 - 1);
    let observed_cycles = GROUP_COUNT + 8 * exact.max_nonzero_groups;
    println!(
        "bound packet_bits_min=1 packet_bits_max=135 pair_token_bits_max={} frame_bits_max={} reserve_proves_no_bit_overflow=true",
        MAX_PAIR_TOKEN_BITS, FRAME_BITS,
    );
    println!(
        "bound serializer_worst_cycles={} transform_cycles_at_2ppc={} guaranteed_2ppc=false reason=all_nonzero_requires_9_cycles_per_group",
        GROUP_COUNT * 9,
        TRANSFORM_FRAME_CYCLES,
    );
    println!(
        "bound nonzero_groups_for_2ppc_max={} observed_max_nonzero_groups={} observed_serializer_cycles={} observed_2ppc_cadence_met={}",
        MAX_NONZERO_FOR_CADENCE,
        exact.max_nonzero_groups,
        observed_cycles,
        observed_cycles <= TRANSFORM_FRAME_CYCLES,
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_controller_transition_profile_is_exact() {
        assert_eq!(
            CANONICAL_NOMINAL_BITS,
            [139, 135, 102, 58, 59, 43, 27, 30, 21, 10, 13, 5]
        );
        assert_eq!(
            CANONICAL_COARSE_ENERGY_SHIFT,
            [0, 2, 2, 0, 0, 0, 1, 1, 1, 2, 1, 0]
        );
        assert_eq!(
            CANONICAL_INCREMENTAL_ENERGY_SHIFT,
            [3, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0]
        );
        assert_eq!(
            hardware_cutoffs(
                -6_i64 << CANONICAL_DEBT_SHIFT,
                CANONICAL_COARSE_CUTOFF_BASE,
                CANONICAL_INCREMENTAL_CUTOFF_BASE,
                CANONICAL_DEBT_SHIFT,
            ),
            (0, 0)
        );
        assert_eq!(
            hardware_cutoffs(
                0,
                CANONICAL_COARSE_CUTOFF_BASE,
                CANONICAL_INCREMENTAL_CUTOFF_BASE,
                CANONICAL_DEBT_SHIFT,
            ),
            (5, 6)
        );
        assert_eq!(
            hardware_cutoffs(
                3_i64 << CANONICAL_DEBT_SHIFT,
                CANONICAL_COARSE_CUTOFF_BASE,
                CANONICAL_INCREMENTAL_CUTOFF_BASE,
                CANONICAL_DEBT_SHIFT,
            ),
            (8, 8)
        );
    }

    #[test]
    fn argument_parser_is_deterministic_and_rejects_unbounded_search() {
        let parsed = Config::parse([
            "--set".to_owned(),
            "fixtures".to_owned(),
            "--max-shift".to_owned(),
            "9".to_owned(),
            "--rounds".to_owned(),
            "2".to_owned(),
        ])
        .unwrap();
        assert_eq!(parsed.set_dir, PathBuf::from("fixtures"));
        assert_eq!(parsed.max_shift, 9);
        assert_eq!(parsed.rounds, 2);
        assert!(!parsed.rtl_profile);
        assert!(
            Config::parse(["--rtl-quantizer".to_owned()])
                .unwrap()
                .rtl_profile
        );
        let hardware_probe = Config::parse([
            "--canonical-tables".to_owned(),
            "--metric".to_owned(),
            "correction-only".to_owned(),
            "--policy".to_owned(),
            "fine-only".to_owned(),
        ])
        .unwrap();
        assert_eq!(hardware_probe.metric, HardwareMetric::CorrectionOnly);
        assert_eq!(hardware_probe.policy, HardwarePolicy::FineOnly);
        assert!(
            Config::parse(["--tune-energy-shifts".to_owned()])
                .unwrap()
                .tune_energy_shifts
        );
        assert!(
            Config::parse(["--tune-exact-controller".to_owned()])
                .unwrap()
                .tune_exact_controller
        );
        let granularity_probe = Config::parse([
            "--ladder-levels".to_owned(),
            "3".to_owned(),
            "--mixed-pairs".to_owned(),
        ])
        .unwrap();
        assert_eq!(granularity_probe.ladder_levels, Some(3));
        assert!(granularity_probe.mixed_pairs);
        assert!(Config::parse(["--rounds".to_owned(), "4".to_owned()]).is_err());
        assert!(Config::parse(["--ladder-levels".to_owned(), "6".to_owned()]).is_err());
        assert!(Config::parse(["--metric".to_owned(), "unknown".to_owned()]).is_err());
        assert!(Config::parse(["--policy".to_owned(), "unknown".to_owned()]).is_err());
    }

    #[test]
    fn rtl_profile_matches_shift_quantizer_and_one_bit_coarse_correction() {
        for shift in 1..=11 {
            let spec = QuantSpec::rtl(shift);
            for source in -8192i32..=8191 {
                let magnitude = source.unsigned_abs();
                let bias = ((1u32 << shift) - 1) / 2;
                let rounded = ((magnitude + bias) >> shift).min(if source < 0 { 128 } else { 127 });
                let expected = if source < 0 {
                    -(rounded as i16)
                } else {
                    rounded as i16
                };
                assert_eq!(quantize(source, spec), expected);
            }
        }

        for fine_shift in 1..=8 {
            for delta in 1..=3 {
                let coarse_shift = fine_shift + delta;
                for source in -8192i32..=8191 {
                    let fine = quantize(source, QuantSpec::rtl(fine_shift));
                    let coarse = quantize(source, QuantSpec::rtl(coarse_shift));
                    if fine == 0 {
                        assert_eq!(coarse, 0);
                        continue;
                    }
                    let derived = fine.unsigned_abs() >> delta;
                    let correction = coarse.unsigned_abs() > derived;
                    let corrected = derived + u16::from(correction);
                    let fine_limit = if source < 0 { 128 } else { 127 };
                    let fine_bias = ((1u32 << fine_shift) - 1) / 2;
                    let unbounded_fine = (source.unsigned_abs() + fine_bias) >> fine_shift;
                    if unbounded_fine <= fine_limit {
                        assert_eq!(corrected, coarse.unsigned_abs());
                    }
                }
            }
        }
    }

    #[test]
    fn generalized_quantizer_has_explicit_deadzone_and_centroid() {
        let spec = QuantSpec::base(2);
        assert_eq!(quantize(1, spec), 0);
        assert_eq!(quantize(2, spec), 1);
        assert_eq!(quantize(5, spec), 1);
        assert_eq!(quantize(6, spec), 2);
        assert_eq!(reconstruct(1, spec), 4);
        assert_eq!(reconstruct(-2, spec), -8);
    }

    #[test]
    fn entropy_parameter_ties_choose_the_smaller_hardware_parameter() {
        let points = [
            Point {
                distortion: 0,
                bits: [4, 3, 3, 7, 8, 9, 10, 11],
            },
            Point {
                distortion: 0,
                bits: [2, 3, 3, 7, 8, 9, 10, 11],
            },
        ];
        assert_eq!(best_entropy_parameter(&points, false), 0);
    }

    #[test]
    fn raw_escape_enforces_pair_token_bound() {
        let coder = PairBoundedRice {
            literal_mask: false,
        };
        let mut values = [0i16; GROUP_LANES];
        values[0] = 127;
        values[1] = -128;
        assert_eq!(coder.packet_bits(&values, 0, 8), RAW_PACKET_BITS);
        values = [0; GROUP_LANES];
        assert_eq!(coder.packet_bits(&values, 0, 8), 1);
    }

    #[test]
    fn fixed_rice_model_ignores_previous_magnitude() {
        for band in 0..BAND_COUNT {
            for base_k in 0..RICE_COUNT as u8 {
                for previous in [0, 1, 2, 3, 5, 127, 128] {
                    assert_eq!(contextual_rice_k(band, base_k, previous), base_k);
                }
            }
        }

        let coder = PairBoundedRice {
            literal_mask: false,
        };
        let mut values = [0i16; GROUP_LANES];
        values[0] = 3;
        values[GROUP_LANES - 1] = 3;
        assert_eq!(coder.packet_bits(&values, 1, 3), 21);
    }

    #[test]
    fn richer_probe_headers_and_raw_packets_are_hard_bounded() {
        assert_eq!(ladder_header_extra(2), 0);
        assert_eq!(ladder_header_extra(3), 1);
        assert_eq!(ladder_header_extra(4), 1);
        assert_eq!(ladder_header_extra(5), 2);
        let raw_point = Point {
            distortion: 0,
            bits: [RAW_PACKET_BITS; RICE_COUNT],
        };
        assert_eq!(state_packet_bits(raw_point, 0, false), RAW_PACKET_BITS);
        assert_eq!(
            state_packet_bits(raw_point, 0, true),
            COARSE_RAW_PACKET_BITS
        );
        assert_eq!(ladder_packet_bits(raw_point, 0, 2), 137);
        assert_eq!(MIXED_PAIR_TOKEN_BITS, 20);
        assert_eq!(MIXED_RAW_PACKET_BITS, 166);
        assert_eq!(MIXED_PAIR_RAW_PACKET_BITS, 150);
    }

    #[test]
    fn slope_bins_are_monotone_and_saturating() {
        assert_eq!(slope_bin(0, 1), 0);
        assert!(slope_bin(1024, 2) > slope_bin(32, 2));
        assert_eq!(slope_bin(u128::MAX, 1), 31);
        assert_eq!(slope_bin(1, 0), 31);
    }

    #[test]
    fn zero_run_credit_matches_bounded_rtl_boundaries() {
        let mut pending = 0;
        for position in 1..=ZERO_RUN_MAX {
            assert_eq!(
                advance_zero_run(&mut pending, true),
                position >= ZERO_RUN_CREDIT_START,
                "position={position}",
            );
        }
        assert_eq!(pending, 0);
        for position in 1..ZERO_RUN_CREDIT_START {
            assert!(!advance_zero_run(&mut pending, true));
            assert_eq!(pending, position);
        }
        assert!(!advance_zero_run(&mut pending, false));
        assert_eq!(pending, 0);
    }

    #[test]
    fn observable_metric_uses_only_quantizer_side_information() {
        let fine = QuantSpec::rtl(2);
        let coarse = QuantSpec::rtl(3);
        assert_eq!(
            observable_coefficient_energy(5, fine, coarse),
            ObservableEnergy {
                direct: 14,
                incremental: 2,
                extra_flag: true,
            }
        );
        assert_eq!(
            observable_coefficient_energy(4, fine, coarse),
            ObservableEnergy {
                direct: 6,
                incremental: 6,
                extra_flag: false,
            }
        );
        assert_eq!(
            observable_coefficient_energy(3, fine, coarse),
            ObservableEnergy {
                direct: 2,
                incremental: 2,
                extra_flag: true,
            }
        );
        assert_eq!(
            observable_coefficient_energy(8, fine, coarse),
            ObservableEnergy {
                direct: 40,
                incremental: 0,
                extra_flag: true,
            }
        );
        assert_eq!(
            observable_coefficient_energy(7, fine, coarse),
            ObservableEnergy {
                direct: 24,
                incremental: 0,
                extra_flag: false,
            }
        );
        assert_eq!(
            observable_coefficient_energy(-5, fine, coarse),
            ObservableEnergy {
                direct: 14,
                incremental: 2,
                extra_flag: true,
            }
        );
        assert_eq!(
            observable_coefficient_energy_for(5, fine, coarse, HardwareMetric::CorrectionOnly,),
            ObservableEnergy {
                direct: 12,
                incremental: 4,
                extra_flag: false,
            }
        );
        assert_eq!(
            observable_coefficient_energy_for(5, fine, coarse, HardwareMetric::Square),
            ObservableEnergy {
                direct: 8,
                incremental: 8,
                extra_flag: false,
            }
        );
    }

    #[test]
    fn divider_free_slope_threshold_matches_half_octave_reference() {
        for gain in 0..=512 {
            for extra_bits in 0..=135 {
                for cutoff in 0..=31 {
                    assert_eq!(
                        slope_passes_cutoff(gain, extra_bits, cutoff),
                        half_octave_bin(gain, extra_bits) >= cutoff,
                        "gain={gain} extra_bits={extra_bits} cutoff={cutoff}",
                    );
                }
            }
        }
        for gain in [
            1_023,
            1_024,
            32_767,
            32_768,
            65_535,
            u32::MAX as u64,
            u64::MAX,
        ] {
            for extra_bits in [0, 1, 2, 7, 16, 64, 135] {
                for cutoff in 0..=31 {
                    assert_eq!(
                        slope_passes_cutoff(gain, extra_bits, cutoff),
                        half_octave_bin(gain, extra_bits) >= cutoff,
                        "gain={gain} extra_bits={extra_bits} cutoff={cutoff}",
                    );
                }
            }
        }
    }
}
