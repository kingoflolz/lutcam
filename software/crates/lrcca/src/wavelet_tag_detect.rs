//! TagDetect reference using the shared wavelet taps and adaptive frontend.
use crate::adaptive_threshold::{
    AdaptiveThresholdConfig, WaveletCompositeFrame, build_wavelet_composite,
    wavelet_composite_geometry,
};
use crate::hierarchical::{PreframedHierarchicalTwoPPCReferenceModel, PreframedHierarchyConfig};
use crate::tag_detect::{
    CentroidGroupDescriptor, CentroidGroupOutput, TagDetectCentroid, TagDetectOutput,
};
use lwc::unified_frontend_reference;

/// Project a CDF wavelet centroid into integer-centered sensor coordinates.
/// LL4 includes the final vertical Haar stage's four-pixel source offset.
pub fn project_wavelet_centroid(
    centroid: TagDetectCentroid,
    width: usize,
    height: usize,
) -> Option<(f64, f64, usize)> {
    if !centroid.centroid_valid {
        return None;
    }
    let x = f64::from(centroid.x_q5) / 32.0;
    let y = f64::from(centroid.y_q5) / 32.0;
    for (origin, scale) in [(0, 1), (width + 4, 4), (width + width / 4 + 12, 16)] {
        let local_x = x - origin as f64;
        if local_x >= 0.0
            && local_x < (width / scale) as f64
            && y >= 0.0
            && y < (height / scale) as f64
        {
            return Some((
                local_x * scale as f64,
                y * scale as f64 + if scale == 16 { 4.0 } else { 0.0 },
                scale,
            ));
        }
    }
    None
}

#[derive(Clone, Debug)]
pub struct WaveletTagDetectReferenceModel {
    pub width: usize,
    pub height: usize,
    pub output_width: usize,
    pub output_height: usize,
    pub config: AdaptiveThresholdConfig,
    pub hierarchy: PreframedHierarchicalTwoPPCReferenceModel,
}

impl WaveletTagDetectReferenceModel {
    pub fn new(
        width: usize,
        height: usize,
        config: AdaptiveThresholdConfig,
    ) -> Result<Self, String> {
        let (output_width, output_height) =
            wavelet_composite_geometry(width, height, config.stage_y_offsets)?;
        let hierarchy = PreframedHierarchicalTwoPPCReferenceModel::new(
            output_width,
            output_height,
            PreframedHierarchyConfig {
                centroid_width: Some(output_width),
                centroid_height: Some(height),
                use_coarse_child_key: false,
                use_full_log_mass_child_key: true,
                midpoint_reflection_reciprocal: true,
            },
        )?;
        Ok(Self {
            width,
            height,
            output_width,
            output_height,
            config,
            hierarchy,
        })
    }

    /// Build thresholds and weights from this frame.
    pub fn composite_frame(&self, pixels: &[u8]) -> Result<WaveletCompositeFrame, String> {
        let frontend = unified_frontend_reference(pixels, self.width, self.height)?;
        build_wavelet_composite(&frontend, &self.config)
    }

    /// Detect centroid groups from this frame.
    pub fn output_stream(&self, pixels: &[u8]) -> Result<Vec<TagDetectOutput>, String> {
        self.output_stream_from_composite(self.composite_frame(pixels)?)
    }

    fn output_stream_from_composite(
        &self,
        composite: WaveletCompositeFrame,
    ) -> Result<Vec<TagDetectOutput>, String> {
        Ok(self
            .hierarchy
            .output_stream(
                &composite.raw,
                &composite.mask,
                &composite.feature_valid,
                &composite.source_y,
            )?
            .into_iter()
            .map(|event| TagDetectOutput {
                centroid: event
                    .inner_feature
                    .map(TagDetectCentroid::from_inner_feature),
                end_inner: event.end_inner,
            })
            .collect())
    }

    /// Detect centroid groups from this frame.
    pub fn centroid_group_output_stream(
        &self,
        pixels: &[u8],
    ) -> Result<Vec<CentroidGroupOutput>, String> {
        self.centroid_group_output_stream_from_composite(self.composite_frame(pixels)?)
    }

    fn centroid_group_output_stream_from_composite(
        &self,
        composite: WaveletCompositeFrame,
    ) -> Result<Vec<CentroidGroupOutput>, String> {
        Ok(self
            .hierarchy
            .output_stream(
                &composite.raw,
                &composite.mask,
                &composite.feature_valid,
                &composite.source_y,
            )?
            .into_iter()
            .map(|event| CentroidGroupOutput {
                descriptor: event
                    .inner_feature
                    .map(CentroidGroupDescriptor::from_inner_feature),
                group_end: event.end_inner,
            })
            .collect())
    }
}

/// Run the canonical adaptive threshold and TagDetect reference.
pub fn run_wavelet_tag_detect_reference(
    pixels: &[u8],
    width: usize,
    height: usize,
    config: AdaptiveThresholdConfig,
) -> Result<Vec<TagDetectOutput>, String> {
    WaveletTagDetectReferenceModel::new(width, height, config)?.output_stream(pixels)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn centroid_projection_matches_actual_frontend_ramp_sample_centers() {
        let (width, height) = (128, 128);
        // A linear ramp makes each low-pass coefficient directly reveal the
        // source coordinate at which it is centered, independently of the
        // projection helper's formula. Stay away from mirrored image edges.
        for axis in 0..2 {
            let pixels = (0..width * height)
                .map(|index| if axis == 0 { index % width } else { index / width } as u8)
                .collect::<Vec<_>>();
            let frontend = unified_frontend_reference(&pixels, width, height).unwrap();
            for (plane, origin) in [
                (&frontend.stage0, 0),
                (&frontend.ll2, width + 4),
                (&frontend.ll4, width + width / 4 + 12),
            ] {
                for y in 2..plane.height - 2 {
                    for x in 2..plane.width - 2 {
                        let centroid = TagDetectCentroid {
                            centroid_valid: true,
                            x_q5: ((origin + x) * 32) as u32,
                            y_q5: (y * 32) as u32,
                        };
                        let (source_x, source_y, scale) =
                            project_wavelet_centroid(centroid, width, height).unwrap();
                        let measured =
                            plane.samples[y * plane.width + x] + if scale == 1 { 0 } else { 128 };
                        assert_eq!(
                            if axis == 0 { source_x } else { source_y },
                            f64::from(measured),
                            "axis {axis}, scale {scale}, logical ({x}, {y})"
                        );
                    }
                }
            }
        }
    }
}
