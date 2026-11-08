//! Linked sensor exposure control from recent, decoded camera images.

use crate::capture::{ExposureSettings, ExposureStatus};
use lwc::{FRAME_HEIGHT, FRAME_WIDTH};

/// Sample the central 80% of the image and discard the darkest/brightest 5%.
/// Small highlights and border pixels should not dominate the exposure meter.
pub fn meter(pixels: &[u8]) -> Option<f32> {
    if pixels.len() != FRAME_WIDTH * FRAME_HEIGHT {
        return None;
    }
    let mut histogram = [0u32; 256];
    let mut count = 0u32;
    for y in (FRAME_HEIGHT / 10..FRAME_HEIGHT * 9 / 10).step_by(8) {
        for x in (FRAME_WIDTH / 10..FRAME_WIDTH * 9 / 10).step_by(8) {
            histogram[usize::from(pixels[y * FRAME_WIDTH + x])] += 1;
            count += 1;
        }
    }
    let trim = count / 20;
    let (start, end) = (trim, count - trim);
    let mut seen = 0;
    let mut weighted = 0u64;
    for (value, amount) in histogram.into_iter().enumerate() {
        let keep = (seen + amount).min(end).saturating_sub(seen.max(start));
        weighted += u64::from(keep) * value as u64;
        seen += amount;
    }
    Some(weighted as f32 / (end - start) as f32)
}

/// Prefer integration time before adding analog gain. A shared setting keeps
/// the two sensors' steady-state exposure duration and gain identical.
/// The caller handles acknowledgement, settling, and stale-frame rejection.
pub fn settings_for_brightness(
    status: &ExposureStatus,
    brightness: [f32; 2],
    target: f32,
) -> Option<ExposureSettings> {
    if !target.is_finite()
        || !(1.0..=254.0).contains(&target)
        || brightness
            .iter()
            .any(|value| !value.is_finite() || !(0.0..=255.0).contains(value))
        || status.min_lines == 0
        || status.min_lines > status.max_lines
        || status.min_gain_code == 0
        || status.min_gain_code > status.max_gain_code
        || !(status.min_lines..=status.max_lines).contains(&status.settings.exposure_lines)
        || !(status.min_gain_code..=status.max_gain_code).contains(&status.settings.gain_code)
    {
        return None;
    }
    let level = (brightness[0] + brightness[1]) * 0.5;
    // Avoid chasing codec noise and small illumination changes.
    if (level - target).abs() <= target * 0.08 {
        return None;
    }
    // Damped correction, limited to half an exposure stop per update.
    let factor = (f64::from(target) / f64::from(level.max(1.0)))
        .sqrt()
        .clamp(std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::SQRT_2);
    let unity_gain = f64::from(status.min_gain_code);
    let exposure = f64::from(status.settings.exposure_lines) * f64::from(status.settings.gain_code)
        / unity_gain;
    let desired = (exposure * factor).clamp(
        f64::from(status.min_lines),
        f64::from(status.max_lines) * f64::from(status.max_gain_code) / unity_gain,
    );
    let exposure_lines = (desired.round() as u32).clamp(status.min_lines, status.max_lines);
    let gain_code = ((desired / f64::from(exposure_lines) * unity_gain).round() as u32)
        .clamp(status.min_gain_code, status.max_gain_code);
    let next = ExposureSettings {
        exposure_lines,
        gain_code,
    };
    (next != status.settings).then_some(next)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn status(lines: u32, gain: u32) -> ExposureStatus {
        ExposureStatus {
            id: 0,
            settings: ExposureSettings {
                exposure_lines: lines,
                gain_code: gain,
            },
            line_time_us: 9.1,
            min_lines: 1,
            max_lines: 885,
            min_gain_code: 16,
            max_gain_code: 255,
        }
    }

    #[test]
    fn meter_ignores_border_and_small_extremes() {
        let mut pixels = vec![0; FRAME_WIDTH * FRAME_HEIGHT];
        for y in FRAME_HEIGHT / 10..FRAME_HEIGHT * 9 / 10 {
            for x in FRAME_WIDTH / 10..FRAME_WIDTH * 9 / 10 {
                pixels[y * FRAME_WIDTH + x] = 80;
            }
        }
        for y in (FRAME_HEIGHT / 10..FRAME_HEIGHT * 9 / 10).step_by(8) {
            pixels[y * FRAME_WIDTH + FRAME_WIDTH / 10] = 255;
            pixels[y * FRAME_WIDTH + FRAME_WIDTH / 10 + 8] = 0;
        }
        assert_eq!(meter(&pixels), Some(80.0));
        assert_eq!(meter(&[]), None);
    }

    #[test]
    fn darkness_increases_exposure_before_gain_with_a_bounded_step() {
        let next = settings_for_brightness(&status(400, 16), [0.0, 0.0], 100.0).unwrap();
        assert_eq!(next.gain_code, 16);
        assert!((565..=566).contains(&next.exposure_lines));
        let next = settings_for_brightness(&status(885, 16), [10.0, 10.0], 100.0).unwrap();
        assert_eq!(next.exposure_lines, 885);
        assert!((22..=23).contains(&next.gain_code));
    }

    #[test]
    fn brightness_reduces_gain_first_and_never_exceeds_limits() {
        let next = settings_for_brightness(&status(885, 128), [255.0, 255.0], 100.0).unwrap();
        assert_eq!(next.exposure_lines, 885);
        assert!((90..=91).contains(&next.gain_code));
        assert_eq!(
            settings_for_brightness(&status(885, 255), [0.0; 2], 100.0),
            None
        );
        assert_eq!(
            settings_for_brightness(&status(1, 16), [255.0; 2], 100.0),
            None
        );
    }

    #[test]
    fn linked_meter_deadband_and_invalid_inputs_do_not_make_commands() {
        assert_eq!(
            settings_for_brightness(&status(400, 16), [60.0, 140.0], 100.0),
            None
        );
        assert_eq!(
            settings_for_brightness(&status(400, 16), [93.0, 101.0], 100.0),
            None
        );
        assert_eq!(
            settings_for_brightness(&status(400, 16), [f32::NAN, 80.0], 100.0),
            None
        );
        assert_eq!(
            settings_for_brightness(&status(400, 16), [80.0; 2], f32::INFINITY),
            None
        );
    }

    #[test]
    fn feedback_converges_in_a_linear_scene_without_oscillating() {
        let mut state = status(400, 16);
        let mut updates = 0;
        loop {
            let brightness =
                (state.settings.exposure_lines as f32 * state.settings.gain_code as f32 / 16.0
                    * 0.04)
                    .min(255.0);
            let Some(next) = settings_for_brightness(&state, [brightness; 2], 100.0) else {
                assert!((92.0..=108.0).contains(&brightness));
                break;
            };
            state.settings = next;
            updates += 1;
            assert!(updates < 20);
        }
    }
}
