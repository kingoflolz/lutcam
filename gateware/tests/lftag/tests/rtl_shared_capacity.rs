//! Capacity checks for the shared numerical core using cycle-timed arrivals.
//!
//! Both sinks remain ready: transport stalls require a separate bound.

use std::path::Path;

use lftag::fixed::rtl_smoke_candidate;
use marlin::verilator::dynamic::DynamicVerilatedModel;
use marlin::verilator::{AsDynamicVerilatedModel, PortDirection};
use rtl_test_support::{TestResult, read_u64, with_model};

const ADAPTER: &str = "tests/lftag/tests/rtl_adapters/shared_capacity_replay.sv";
const PORTS: &[(&str, usize, usize, PortDirection)] = &[
    ("clk", 0, 0, PortDirection::Input),
    ("rst", 0, 0, PortDirection::Input),
    ("in_valid", 1, 0, PortDirection::Input),
    ("in_group_end", 1, 0, PortDirection::Input),
    ("in_centroid_valid", 1, 0, PortDirection::Input),
    ("in_x_q4", 29, 0, PortDirection::Input),
    ("in_y_q4", 27, 0, PortDirection::Input),
    ("in_log_mass_q5_3", 15, 0, PortDirection::Input),
    ("in_scale", 3, 0, PortDirection::Input),
    ("in_frame_seq", 15, 0, PortDirection::Input),
    ("out_ready", 1, 0, PortDirection::Input),
    ("out_valid", 1, 0, PortDirection::Output),
    ("out_last", 1, 0, PortDirection::Output),
    ("out_data", 31, 0, PortDirection::Output),
    ("busy", 1, 0, PortDirection::Output),
    ("fifo_level", 11, 0, PortDirection::Output),
    ("candidate_start", 1, 0, PortDirection::Output),
    ("candidate_done", 1, 0, PortDirection::Output),
    ("candidate_cycles", 19, 0, PortDirection::Output),
    ("fifo_drop_sticky", 1, 0, PortDirection::Output),
    ("protocol_error_sticky", 1, 0, PortDirection::Output),
    ("debug_status", 63, 0, PortDirection::Output),
    ("debug_protocol", 63, 0, PortDirection::Output),
    ("debug_drop", 63, 0, PortDirection::Output),
];

#[derive(Clone, Debug)]
struct Arrival {
    cycle: usize,
    camera: usize,
    x: u16,
    y: u16,
    mass: u8,
    valid: bool,
    scale: u8,
    frame: u8,
    end: bool,
}

#[derive(Default, Debug)]
struct Statistics {
    input_groups: [usize; 2],
    starts: [usize; 2],
    packets: [usize; 2],
    accepted: [usize; 2],
    rejected: [[usize; 16]; 2],
    peak: [usize; 2],
    max_compute: usize,
    last_output: usize,
    drop_mask: u64,
    protocol_mask: u64,
    debug_status: u64,
    debug_drop: u64,
}

fn with_replay(
    test: impl for<'a> FnOnce(&mut DynamicVerilatedModel<'a>) -> TestResult,
) -> TestResult {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let adapter = root.join(ADAPTER);
    with_model(
        &root,
        Path::new("rtl/lftag/sources.f"),
        "shared-capacity-replay-v1",
        "SharedCapacityReplay",
        Path::new(ADAPTER),
        PORTS,
        &[Path::new("rtl/lftag")],
        &[adapter.as_path()],
        test,
    )
}

fn drive(dut: &mut DynamicVerilatedModel<'_>, arrivals: &[Arrival]) -> TestResult {
    let mut valid = 0_u8;
    let mut end = 0_u8;
    let mut centroid_valid = 0_u8;
    let mut x = 0_u32;
    let mut y = 0_u32;
    let mut mass = 0_u16;
    let mut scale = 0_u8;
    let mut frame = 0_u16;
    for a in arrivals {
        assert!(a.camera < 2 && a.x < 32768 && a.y < 16384 && a.scale < 4);
        assert_eq!(
            valid & (1 << a.camera),
            0,
            "two arrivals on one camera/clock"
        );
        valid |= 1 << a.camera;
        end |= u8::from(a.end) << a.camera;
        centroid_valid |= u8::from(a.valid) << a.camera;
        x |= u32::from(a.x) << (15 * a.camera);
        y |= u32::from(a.y) << (14 * a.camera);
        mass |= u16::from(a.mass) << (8 * a.camera);
        scale |= a.scale << (2 * a.camera);
        frame |= u16::from(a.frame) << (8 * a.camera);
    }
    dut.pin("in_valid", valid)?;
    dut.pin("in_group_end", end)?;
    dut.pin("in_centroid_valid", centroid_valid)?;
    dut.pin("in_x_q4", x)?;
    dut.pin("in_y_q4", y)?;
    dut.pin("in_log_mass_q5_3", mass)?;
    dut.pin("in_scale", scale)?;
    dut.pin("in_frame_seq", frame)?;
    Ok(())
}

fn sample_output(
    dut: &DynamicVerilatedModel<'_>,
    stats: &mut Statistics,
    pending: &mut [Vec<u32>; 2],
    cycle: usize,
) -> TestResult {
    let starts = read_u64(dut, "candidate_start")?;
    let dones = read_u64(dut, "candidate_done")?;
    let valid = read_u64(dut, "out_valid")?;
    let last = read_u64(dut, "out_last")?;
    let levels = read_u64(dut, "fifo_level")?;
    for (camera, pending_packet) in pending.iter_mut().enumerate() {
        stats.peak[camera] = stats.peak[camera].max(((levels >> (camera * 6)) & 63) as usize);
        stats.starts[camera] += usize::from(starts & (1 << camera) != 0);
        if dones & (1 << camera) != 0 {
            let compute = ((read_u64(dut, "candidate_cycles")? >> (camera * 10)) & 1023) as usize;
            assert!(compute <= 1000, "solver watchdog failed");
            stats.max_compute = stats.max_compute.max(compute);
        }
        if valid & (1 << camera) != 0 {
            pending_packet.push(read_u64(dut, "out_data")? as u32);
            if last & (1 << camera) != 0 {
                let packet = std::mem::take(pending_packet);
                assert!(matches!(packet.len(), 8 | 22));
                if packet[1] & 0x8000_0000 != 0 {
                    assert_eq!(packet.len(), 22);
                    lftag::PosePacket::from_packet(&packet).unwrap();
                    stats.accepted[camera] += 1;
                } else {
                    assert_eq!(packet.len(), 8);
                    assert!(packet[6] > 0 && packet[6] < 16);
                    stats.rejected[camera][packet[6] as usize] += 1;
                }
                stats.packets[camera] += 1;
                stats.last_output = cycle;
            }
        }
    }
    Ok(())
}

fn run(
    dut: &mut DynamicVerilatedModel<'_>,
    arrivals: &[Arrival],
    permit_drop: bool,
) -> TestResult<Statistics> {
    assert!(!arrivals.is_empty());
    assert!(
        arrivals
            .windows(2)
            .all(|pair| pair[0].cycle <= pair[1].cycle)
    );
    dut.pin("out_ready", 3_u8)?;
    dut.pin("rst", 1_u8)?;
    drive(dut, &[])?;
    for _ in 0..4 {
        rtl_test_support::clock(dut)?;
    }
    dut.pin("rst", 0_u8)?;
    let mut stats = Statistics::default();
    let mut pending: [Vec<u32>; 2] = std::array::from_fn(|_| Vec::new());
    let mut cursor = 0;
    let deadline = arrivals.last().unwrap().cycle + 100_000;
    for cycle in 0..deadline {
        let begin = cursor;
        while cursor < arrivals.len() && arrivals[cursor].cycle == cycle {
            if arrivals[cursor].end {
                stats.input_groups[arrivals[cursor].camera] += 1;
            }
            cursor += 1;
        }
        drive(dut, &arrivals[begin..cursor])?;
        dut.pin("clk", 0_u8)?;
        dut.eval();
        sample_output(dut, &mut stats, &mut pending, cycle)?;
        dut.pin("clk", 1_u8)?;
        dut.eval();
        if cursor == arrivals.len() && read_u64(dut, "busy")? == 0 {
            break;
        }
        assert!(cycle + 1 < deadline, "shared solver did not drain");
    }
    stats.drop_mask = read_u64(dut, "fifo_drop_sticky")?;
    stats.protocol_mask = read_u64(dut, "protocol_error_sticky")?;
    stats.debug_status = read_u64(dut, "debug_status")?;
    stats.debug_drop = read_u64(dut, "debug_drop")?;
    assert_eq!(stats.protocol_mask, 0, "trace assembler protocol error");
    assert_eq!(stats.starts, stats.packets);
    if !permit_drop {
        assert_eq!(stats.drop_mask, 0, "candidate FIFO drops: {stats:?}");
        assert_eq!(stats.packets, stats.input_groups, "lost candidate results");
    }
    Ok(stats)
}

fn bursts(groups: usize, batch: usize, interval: usize) -> Vec<Arrival> {
    let mut arrivals = Vec::new();
    for group in 0..groups {
        let mut candidate = rtl_smoke_candidate(0x1234);
        candidate.centroids.sort_by_key(|point| point.log_mass_q5_3);
        for (index, point) in candidate.centroids.into_iter().enumerate() {
            for camera in 0..2 {
                arrivals.push(Arrival {
                    cycle: group / batch * interval + group % batch * 9 + index,
                    camera,
                    x: point.x_q4 as u16,
                    y: point.y_q4 as u16,
                    mass: point.log_mass_q5_3,
                    valid: point.valid,
                    scale: 0,
                    frame: group as u8,
                    end: index == 8,
                });
            }
        }
    }
    arrivals
}

#[test]
fn synchronized_bursts_of_32_accepted_candidates_drain_without_drops() -> TestResult {
    with_replay(|dut| {
        let stats = run(dut, &bursts(165, 32, 70_000), false)?;
        assert_eq!(stats.accepted, [165, 165]);
        assert_eq!(stats.peak, [31, 32]);
        assert!(stats.last_output < 1_000_000);
        eprintln!("bounded 32-pair burst envelope: {stats:?}");
        Ok(())
    })
}

#[test]
fn unbounded_arrival_burst_reports_whole_candidate_drops() -> TestResult {
    with_replay(|dut| {
        let stats = run(dut, &bursts(165, 165, 0), true)?;
        assert_ne!(stats.drop_mask, 0);
        assert!(stats.packets.iter().all(|&count| count < 165));
        assert_eq!(stats.peak, [32, 32]);
        eprintln!("adversarial all-at-once burst: {stats:?}");
        Ok(())
    })
}

// Native centroid identity: x_q4, y_q4, full log-mass key, centroid_valid.
type NativeFeature = [u32; 4];

fn valid_retained_nine(actual: &[NativeFeature], source: &[NativeFeature]) -> bool {
    if actual.len() != 9 || source.len() < 9 {
        return false;
    }
    let mut keys: Vec<_> = source.iter().map(|feature| feature[2]).collect();
    keys.sort_unstable();
    let cutoff = keys[keys.len() - 9];
    let mut remaining = source.to_vec();
    for feature in actual {
        if feature[2] < cutoff {
            return false;
        }
        let Some(index) = remaining.iter().position(|candidate| candidate == feature) else {
            return false;
        };
        remaining.swap_remove(index);
    }
    !remaining.iter().any(|feature| feature[2] > cutoff)
}

fn assign_source_group(
    source: usize,
    eligible: &[Vec<usize>],
    seen: &mut [bool],
    owners: &mut [Option<usize>],
) -> bool {
    for &actual in &eligible[source] {
        if seen[actual] {
            continue;
        }
        seen[actual] = true;
        if owners[actual]
            .is_none_or(|previous| assign_source_group(previous, eligible, seen, owners))
        {
            owners[actual] = Some(source);
            return true;
        }
    }
    false
}

fn retained_group_assignment(
    actual: &[Vec<NativeFeature>],
    source: &[Vec<NativeFeature>],
) -> Result<Vec<usize>, String> {
    let source: Vec<_> = source.iter().filter(|group| group.len() >= 9).collect();
    if actual.len() != source.len() {
        return Err(format!(
            "group counts actual={} expected={}",
            actual.len(),
            source.len()
        ));
    }
    let eligible: Vec<Vec<usize>> = source
        .iter()
        .map(|group| {
            actual
                .iter()
                .enumerate()
                .filter_map(|(index, candidate)| {
                    valid_retained_nine(candidate, group).then_some(index)
                })
                .collect()
        })
        .collect();
    let mut owners = vec![None; actual.len()];
    for index in 0..source.len() {
        if !assign_source_group(
            index,
            &eligible,
            &mut vec![false; actual.len()],
            &mut owners,
        ) {
            return Err(format!(
                "no one-to-one retained-nine assignment for source group {index}, eligible {:?}",
                eligible[index]
            ));
        }
    }
    Ok(owners.into_iter().map(Option::unwrap).collect())
}

#[test]
fn retained_nine_requires_exact_component_provenance_and_heavier_children() {
    let source: Vec<_> = (0..12)
        .map(|i| [i, 0, if i < 7 { 20 + i } else { 19 }, 1])
        .collect();
    let mut selected = source[..7].to_vec();
    selected.extend_from_slice(&source[10..12]);
    assert!(valid_retained_nine(&selected, &source));
    let mut lighter = selected.clone();
    lighter[0] = source[7];
    assert!(!valid_retained_nine(&lighter, &source));
    let mut duplicate = selected.clone();
    duplicate[8] = duplicate[7];
    assert!(!valid_retained_nine(&duplicate, &source));
    let mut alien = selected.clone();
    alien[8][0] += 100;
    assert!(!valid_retained_nine(&alien, &source));
    let other: Vec<_> = source
        .iter()
        .map(|f| [f[0] + 100, f[1], f[2], f[3]])
        .collect();
    assert!(retained_group_assignment(&[selected.clone(), selected], &[source, other]).is_err());
}
