//! Canonical adaptive threshold and raster formatter, independent of the RTL.

use lwc::{FrontendPlane, UnifiedFrontendReference};

pub const DEFAULT_WAVELET_STAGE_Y_OFFSETS: [usize; 3] = [0, 27, 127];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AdaptiveThresholdConfig {
    /// Source-raster row at which logical row zero of each scale is placed.
    pub stage_y_offsets: [usize; 3],
}

impl Default for AdaptiveThresholdConfig {
    fn default() -> Self {
        Self {
            stage_y_offsets: DEFAULT_WAVELET_STAGE_Y_OFFSETS,
        }
    }
}

/// The preframed raster fields consumed by the hierarchy reference.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WaveletCompositeFrame {
    pub width: usize,
    pub height: usize,
    pub raw: Vec<Vec<u8>>,
    pub mask: Vec<Vec<u8>>,
    pub feature_valid: Vec<Vec<u8>>,
    pub source_y: Vec<Vec<usize>>,
    pub row_replicated: Vec<Vec<u8>>,
}

#[derive(Clone, Copy, Debug)]
struct CompositeGeometry {
    stage_widths: [usize; 3],
    stage_heights: [usize; 3],
    stage_output_widths: [usize; 3],
    stage_left_pad: [usize; 3],
    stage_vrep: [usize; 3],
    covered: [usize; 3],
    output_width: usize,
    output_height: usize,
}

#[derive(Clone, Copy, Debug)]
struct VerticalLayout {
    first_source_y: usize,
    top_pad: usize,
    local_height: usize,
}

#[derive(Clone, Copy, Debug, Default)]
struct CompositePixel {
    d: u8,
    t: u8,
    feature_valid: u8,
    source_y: usize,
    row_replicated: u8,
}

/// Saturate one centered wavelet low-pass coefficient to the unsigned byte
/// stored in the composite threshold record.
pub fn centered_coefficient_to_u8(coefficient: i32) -> u8 {
    (coefficient + 128).clamp(0, 255) as u8
}

fn validate_plane(name: &str, plane: &FrontendPlane) -> Result<(), String> {
    if plane.width == 0 || plane.height == 0 {
        return Err(format!("{name} dimensions must be positive"));
    }
    if plane.samples.len() != plane.width * plane.height {
        return Err(format!("{name} storage does not match its dimensions"));
    }
    Ok(())
}

fn ll2_threshold_reference(ll4: &FrontendPlane) -> Vec<i32> {
    let rows = 3;
    (0..ll4.height)
        .flat_map(|y| {
            (0..ll4.width).map(move |x| {
                let mut sum = 0i64;
                for previous_row in 0..rows {
                    let sample_y = y.saturating_sub(previous_row);
                    for column in 0..5 {
                        let sample_x = x
                            .saturating_add(column)
                            .saturating_sub(2)
                            .min(ll4.width - 1);
                        sum += i64::from(ll4.samples[sample_y * ll4.width + sample_x]);
                    }
                }
                // Round the complete signed sum once; intermediate horizontal
                // or vertical rounding would produce a different threshold.
                let divisor = (rows * 5) as i64;
                ((sum + divisor / 2).div_euclid(divisor)) as i32
            })
        })
        .collect()
}

pub fn sample_ll2_threshold(
    reference: &[i32],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
) -> i32 {
    // Nearest LL4 center in LL2 coordinates: (4*cx, 4*cy+1).
    // Half-way ties choose the larger coordinate; boundaries replicate.
    reference[((y + 1) / 4).min(height - 1) * width + ((x + 2) / 4).min(width - 1)]
}

/// Masks, RAW threshold values, and the RAW mask before connectivity repair.
#[derive(Clone, Debug)]
pub struct AdaptiveThresholdPlanes {
    pub masks: [Vec<u8>; 3],
    pub raw_thresholds: Vec<i32>,
    pub raw_base_mask: Vec<u8>,
}

/// Evaluate local thresholds with direct integer division and replicated edges.
/// Equality is white; connectivity repair adds only black mask pixels.
pub fn adaptive_threshold_planes(
    frontend: &UnifiedFrontendReference,
) -> Result<AdaptiveThresholdPlanes, String> {
    validate_plane("stage-zero threshold plane", &frontend.stage0)?;
    validate_plane("LL2 threshold plane", &frontend.ll2)?;
    validate_plane("LL4 threshold plane", &frontend.ll4)?;
    if frontend.stage0.width != 4 * frontend.ll2.width
        || frontend.stage0.height != 4 * frontend.ll2.height
    {
        return Err("stage-zero and LL2 threshold dimensions must differ by four".into());
    }
    if frontend.ll2.width != 4 * frontend.ll4.width
        || frontend.ll2.height != 4 * frontend.ll4.height
    {
        return Err("LL2 and LL4 threshold dimensions must differ by four".into());
    }

    let raw_radius = 6;
    let (raw_bias, raw_floor, raw_ceiling) = (20, 40, 128);
    let mut masks = [
        vec![0; frontend.stage0.samples.len()],
        vec![0; frontend.ll2.samples.len()],
        frontend
            .ll4
            .samples
            .iter()
            .map(|&p| u8::from(p >= 0))
            .collect(),
    ];
    let mut reference = vec![0; frontend.ll2.samples.len()];
    for y in 0..frontend.ll2.height {
        for x in 0..frontend.ll2.width {
            let mut sum = 0;
            for dy in 0..3 {
                for dx in 0..2 * raw_radius + 1 {
                    let sx = (x + dx)
                        .saturating_sub(raw_radius)
                        .min(frontend.ll2.width - 1);
                    sum += frontend.ll2.samples[y.saturating_sub(dy) * frontend.ll2.width + sx];
                }
            }
            let divisor = (3 * (2 * raw_radius + 1)) as i32;
            let local = frontend.ll2.samples[y * frontend.ll2.width + x];
            let local_three = frontend.ll2.samples[y * frontend.ll2.width + x.saturating_sub(1)]
                + local
                + frontend.ll2.samples
                    [y * frontend.ll2.width + (x + 1).min(frontend.ll2.width - 1)];
            let local_mean = (local_three + 3 * 128 + 1).div_euclid(3);
            let local_cap = (local_mean + 1).div_euclid(2) + 64;
            reference[y * frontend.ll2.width + x] =
                ((35 * (sum + 128 * divisor) + 8 * divisor * (local + 128) + 32 * divisor)
                    .div_euclid(64 * divisor)
                    + raw_bias)
                    .min(local_cap)
                    .clamp(raw_floor, raw_ceiling);
        }
    }
    let mut raw_thresholds = vec![0; frontend.stage0.samples.len()];
    for y in 0..frontend.stage0.height {
        for x in 0..frontend.stage0.width {
            let i = y * frontend.stage0.width + x;
            raw_thresholds[i] = reference[(y >> 2) * frontend.ll2.width + (x >> 2)];
            masks[0][i] = u8::from(frontend.stage0.samples[i] >= raw_thresholds[i]);
        }
    }
    let filtered = ll2_threshold_reference(&frontend.ll4);
    for y in 0..frontend.ll2.height {
        for x in 0..frontend.ll2.width {
            let i = y * frontend.ll2.width + x;
            let reference =
                sample_ll2_threshold(&filtered, frontend.ll4.width, frontend.ll4.height, x, y);
            masks[1][i] = u8::from(frontend.ll2.samples[i] >= reference);
        }
    }
    let (width, height) = (frontend.stage0.width, frontend.stage0.height);
    let original_raw_mask = masks[0].clone();
    let strength = |i: usize| {
        ((frontend.stage0.samples[i] - raw_thresholds[i]).abs() + 8)
            .div_euclid(16)
            .min(7)
    };
    for y in 1..height {
        for x in 1..width - 1 {
            let i = y * width + x;
            if original_raw_mask[i] != 0 && original_raw_mask[i - width] == 0 {
                let top_continues = y >= 2
                    && original_raw_mask[i - width - 1] != 0
                    && original_raw_mask[i - width + 1] != 0
                    && (original_raw_mask[i - 2 * width - 1] == 0
                        || original_raw_mask[i - 2 * width + 1] == 0);
                let left = original_raw_mask[i - 1] == 0
                    && original_raw_mask[i - width - 1] != 0
                    && (strength(i - width) * strength(i - 1)
                        > strength(i) * strength(i - width - 1)
                        || top_continues && x >= 2 && original_raw_mask[i - 2] != 0);
                let right = original_raw_mask[i + 1] == 0
                    && original_raw_mask[i - width + 1] != 0
                    && (strength(i - width) * strength(i + 1)
                        > strength(i) * strength(i - width + 1)
                        || top_continues && x + 2 < width && original_raw_mask[i + 2] != 0);
                if (left || right) && raw_thresholds[i] <= 120 {
                    masks[0][i] = 0;
                }
            }
        }
    }
    Ok(AdaptiveThresholdPlanes {
        masks,
        raw_thresholds,
        raw_base_mask: original_raw_mask,
    })
}

fn composite_geometry(
    width: usize,
    height: usize,
    stage_y_offsets: [usize; 3],
) -> Result<CompositeGeometry, String> {
    if width == 0 || height == 0 || !width.is_multiple_of(16) || !height.is_multiple_of(16) {
        return Err("wavelet composite dimensions must be positive and divisible by 16".into());
    }
    if stage_y_offsets[0] != 0 {
        return Err("stage-zero y offset must be zero".into());
    }
    let stage_widths = [width, width / 4, width / 16];
    let stage_heights = [height, height / 4, height / 16];
    let stage_left_pad = [0, 4, 4];
    let stage_output_widths = [stage_widths[0], stage_widths[1] + 8, stage_widths[2] + 8];
    let stage_vrep = [1, 4, 16];
    let covered = std::array::from_fn(|index| {
        let active_source_rows = height.saturating_sub(stage_y_offsets[index]);
        stage_heights[index].min(active_source_rows.div_ceil(stage_vrep[index]))
    });
    let output_width = stage_output_widths.iter().sum();
    let output_height = height
        + (0..3)
            .map(|index| stage_heights[index] - covered[index])
            .max()
            .unwrap_or(0);
    Ok(CompositeGeometry {
        stage_widths,
        stage_heights,
        stage_output_widths,
        stage_left_pad,
        stage_vrep,
        covered,
        output_width,
        output_height,
    })
}

fn map_output_y(
    index: usize,
    out_y: usize,
    source_height: usize,
    stage_y_offsets: [usize; 3],
    geometry: CompositeGeometry,
) -> (Option<usize>, u8) {
    if out_y < source_height {
        if let Some(region) = out_y.checked_sub(stage_y_offsets[index])
            && region < source_height
        {
            let logical = region / geometry.stage_vrep[index];
            if logical < geometry.stage_heights[index] {
                return (
                    Some(logical),
                    u8::from(index != 0 && !region.is_multiple_of(geometry.stage_vrep[index])),
                );
            }
        }
    } else {
        let logical = geometry.covered[index] + out_y - source_height;
        if logical < geometry.stage_heights[index] {
            return (Some(logical), 0);
        }
    }
    (None, 0)
}

fn vertical_layouts(
    source_height: usize,
    stage_y_offsets: [usize; 3],
    geometry: CompositeGeometry,
) -> Result<[VerticalLayout; 3], String> {
    let mut result = [VerticalLayout {
        first_source_y: 0,
        top_pad: 0,
        local_height: 0,
    }; 3];
    for (index, layout) in result.iter_mut().enumerate() {
        let rows = (0..geometry.output_height)
            .filter(|&out_y| {
                map_output_y(index, out_y, source_height, stage_y_offsets, geometry)
                    .0
                    .is_some()
            })
            .collect::<Vec<_>>();
        let (&first, &last) = (
            rows.first()
                .ok_or("selected wavelet scale has no output rows")?,
            rows.last().unwrap(),
        );
        let top_pad = 3.min(first);
        let bottom_pad = 3.min(geometry.output_height - 1 - last);
        *layout = VerticalLayout {
            first_source_y: first,
            top_pad,
            local_height: top_pad + last - first + 1 + bottom_pad,
        };
    }
    Ok(result)
}

#[allow(clippy::too_many_arguments)]
fn composite_pixel(
    index: usize,
    out_y: usize,
    out_x: usize,
    logical_y: Option<usize>,
    replicated: u8,
    plane: &FrontendPlane,
    threshold: &[u8],
    layout: VerticalLayout,
    geometry: CompositeGeometry,
) -> CompositePixel {
    let start = layout.first_source_y - layout.top_pad;
    let Some(local_y) = out_y
        .checked_sub(start)
        .filter(|&y| y < layout.local_height)
    else {
        return CompositePixel::default();
    };
    let local_width = geometry.stage_output_widths[index];
    let distances = [
        out_x,
        local_width - 1 - out_x,
        local_y,
        layout.local_height - 1 - local_y,
    ];
    let row_replicated = if logical_y.is_some() { replicated } else { 0 };
    if distances.contains(&0) {
        return CompositePixel {
            row_replicated,
            ..CompositePixel::default()
        };
    }
    if distances.contains(&1) {
        return CompositePixel {
            t: 1,
            row_replicated,
            ..CompositePixel::default()
        };
    }
    if distances[0] < 4 || distances[1] < 4 || distances[2] < 3 || distances[3] < 3 {
        return CompositePixel {
            row_replicated,
            ..CompositePixel::default()
        };
    }
    let Some(source_x) = out_x
        .checked_sub(geometry.stage_left_pad[index])
        .filter(|&x| x < geometry.stage_widths[index])
    else {
        return CompositePixel {
            row_replicated,
            ..CompositePixel::default()
        };
    };
    let Some(source_y) = logical_y else {
        return CompositePixel {
            row_replicated,
            ..CompositePixel::default()
        };
    };
    let sample = plane.samples[source_y * plane.width + source_x];
    CompositePixel {
        d: if index == 0 {
            sample.clamp(0, 255) as u8
        } else {
            centered_coefficient_to_u8(sample)
        },
        t: threshold[source_y * plane.width + source_x],
        feature_valid: u8::from(row_replicated == 0),
        source_y,
        row_replicated,
    }
}

/// Format the threshold planes, repaired mask and analytic moment weights.
pub fn build_wavelet_composite(
    frontend: &UnifiedFrontendReference,
    config: &AdaptiveThresholdConfig,
) -> Result<WaveletCompositeFrame, String> {
    let result = adaptive_threshold_planes(frontend)?;
    let thresholds = &result.masks;
    let width = frontend.stage0.width;
    let height = frontend.stage0.height;
    let geometry = composite_geometry(width, height, config.stage_y_offsets)?;
    let planes = [&frontend.stage0, &frontend.ll2, &frontend.ll4];
    for (index, plane) in planes.iter().enumerate() {
        if (plane.width, plane.height)
            != (geometry.stage_widths[index], geometry.stage_heights[index])
        {
            return Err(format!(
                "wavelet scale {index} dimensions do not match composite geometry"
            ));
        }
    }
    let layouts = vertical_layouts(height, config.stage_y_offsets, geometry)?;
    let mut raw = vec![vec![0; geometry.output_width]; geometry.output_height];
    let mut mask = raw.clone();
    let mut feature_valid = raw.clone();
    let mut source_y = vec![vec![0; geometry.output_width]; geometry.output_height];
    let mut row_replicated = raw.clone();

    for out_y in 0..geometry.output_height {
        let mut output_x = 0;
        for index in 0..3 {
            let (logical_y, replicated) =
                map_output_y(index, out_y, height, config.stage_y_offsets, geometry);
            for local_x in 0..geometry.stage_output_widths[index] {
                let pixel = composite_pixel(
                    index,
                    out_y,
                    local_x,
                    logical_y,
                    replicated,
                    planes[index],
                    &thresholds[index],
                    layouts[index],
                    geometry,
                );
                // Convert every sample to a weight below; feature_valid controls
                // which samples contribute to the accumulated moments.
                raw[out_y][output_x] = 255 - pixel.d;
                mask[out_y][output_x] = pixel.t;
                feature_valid[out_y][output_x] = pixel.feature_valid;
                source_y[out_y][output_x] = pixel.source_y;
                row_replicated[out_y][output_x] = pixel.row_replicated;
                output_x += 1;
            }
        }
    }
    let mut composite = WaveletCompositeFrame {
        width: geometry.output_width,
        height: geometry.output_height,
        raw,
        mask,
        feature_valid,
        source_y,
        row_replicated,
    };
    // Round the analytic curves to five-bit samples before accumulating moments.
    let raw_weights: [u8; 256] = std::array::from_fn(|pixel| {
        let dark = (279_i32 - pixel as i32).clamp(0, 255) as f64;
        (0.85 * (255.0 * (dark / 255.0).powf(5.0)).round()).round() as u8
    });
    let coarse_weights: [u8; 256] = std::array::from_fn(|pixel| {
        (255.0 * ((255 - pixel) as f64 / 255.0).powf(2.5)).round() as u8
    });
    for y in 0..composite.height {
        for x in 0..composite.width {
            let pixel = (255 - composite.raw[y][x]) as usize;
            composite.raw[y][x] = if x < width {
                ((u16::from(raw_weights[pixel]) + 4) >> 3).min(31) as u8
            } else {
                ((u16::from(coarse_weights[pixel]) + 4) >> 3).min(31) as u8
            };
        }
    }
    for y in 0..height {
        for x in 0..width {
            let i = y * width + x;
            if result.masks[0][i] == 0 && result.raw_base_mask[i] != 0 {
                composite.feature_valid[y][x] = 0;
            }
        }
    }
    Ok(composite)
}

/// Run the canonical formatter directly from an unsigned camera frame.
pub fn adaptive_threshold_composite(
    pixels: &[u8],
    width: usize,
    height: usize,
) -> Result<WaveletCompositeFrame, String> {
    let frontend = lwc::unified_frontend_reference(pixels, width, height)?;
    build_wavelet_composite(&frontend, &AdaptiveThresholdConfig::default())
}

pub fn wavelet_composite_geometry(
    width: usize,
    height: usize,
    offsets: [usize; 3],
) -> Result<(usize, usize), String> {
    let g = composite_geometry(width, height, offsets)?;
    Ok((g.output_width, g.output_height))
}
