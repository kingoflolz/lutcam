//! Hierarchical foreground/background CCA and child centroid reference model.

use std::cell::RefCell;
use std::rc::Rc;

use crate::centroid_compression::{CompressedCentroid, compress_centroid, log_m_q5_3};
use crate::child_arena::ChildArenaReferenceModel;
use crate::two_ppc::{
    CallbackFeatureModel, TwoPpcInputs, TwoPpcOutputs, TwoPpcReferenceModel, iter_2ppc_pairs,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct BBoxFeature {
    pub min_x: usize,
    pub max_x: usize,
    pub min_y: usize,
    pub max_y: usize,
    pub area: usize,
}
impl BBoxFeature {
    pub fn from_tuple(value: (usize, usize, usize, usize, usize)) -> Self {
        Self {
            min_x: value.0,
            max_x: value.1,
            min_y: value.2,
            max_y: value.3,
            area: value.4,
        }
    }
    pub fn as_tuple(self) -> (usize, usize, usize, usize, usize) {
        (self.min_x, self.max_x, self.min_y, self.max_y, self.area)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct MomentFeature {
    pub area: usize,
    pub zom: u64,
    pub x_fom: u64,
    pub y_fom: u64,
}
impl MomentFeature {
    pub fn from_tuple(value: (usize, u64, u64, u64)) -> Self {
        Self {
            area: value.0,
            zom: value.1,
            x_fom: value.2,
            y_fom: value.3,
        }
    }
    pub fn as_tuple(self) -> (usize, u64, u64, u64) {
        (self.area, self.zom, self.x_fom, self.y_fom)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct InnerCentroidFeature {
    pub area: usize,
    pub centroid_valid: bool,
    pub x_q5: u32,
    pub y_q5: u32,
    /// Full unsigned Q5.3 log mass produced by the centroid compressor.
    pub log_m_q5_3: u8,
    /// Sort-only metadata; it is intentionally excluded from feature identity
    /// in the original dataclass but remains visible on the stream.
    pub child_sort_key: u8,
}
pub type InnerFeature = InnerCentroidFeature;
impl PartialEq for InnerCentroidFeature {
    fn eq(&self, other: &Self) -> bool {
        (self.area, self.centroid_valid, self.x_q5, self.y_q5)
            == (other.area, other.centroid_valid, other.x_q5, other.y_q5)
    }
}
impl Eq for InnerCentroidFeature {}
impl PartialOrd for InnerCentroidFeature {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for InnerCentroidFeature {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        (self.area, self.centroid_valid, self.x_q5, self.y_q5).cmp(&(
            other.area,
            other.centroid_valid,
            other.x_q5,
            other.y_q5,
        ))
    }
}
impl InnerCentroidFeature {
    pub fn from_compressed(
        area: usize,
        centroid: CompressedCentroid,
        log_m_q5_3: u8,
        child_sort_key: u8,
    ) -> Self {
        Self {
            area,
            centroid_valid: centroid.valid,
            x_q5: centroid.x_q5,
            y_q5: centroid.y_q5,
            log_m_q5_3,
            child_sort_key,
        }
    }
    pub fn as_tuple(self) -> (usize, bool, u32, u32) {
        (self.area, self.centroid_valid, self.x_q5, self.y_q5)
    }
}

pub fn tagdetect_child_sort_key(area: usize, m: u64) -> u8 {
    let area_key = area.min(7) as u8;
    if area_key == 0 {
        0
    } else if area_key < 7 {
        area_key - 1
    } else {
        6 + (log_m_q5_3(m) >> 7)
    }
}

pub fn full_log_mass_child_sort_key(m: u64) -> u8 {
    log_m_q5_3(m)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct HierarchicalFeature;
impl HierarchicalFeature {
    pub fn from_values(
        _min_x: usize,
        _max_x: usize,
        _min_y: usize,
        _max_y: usize,
        _area: usize,
    ) -> Self {
        Self
    }
    pub fn as_tuple(self) {}
}

#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct HierarchicalFeatureGroup {
    pub outer: HierarchicalFeature,
    pub inner_features: Vec<InnerFeature>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct HierarchicalOutput {
    pub inner_feature: Option<InnerFeature>,
    pub end_inner: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EmptyFeature;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HierarchyFrame {
    pub foreground_topology: Vec<Vec<u8>>,
    pub foreground_feature: Vec<Vec<u8>>,
    pub foreground_raw: Vec<Vec<u8>>,
    pub background: Vec<Vec<u8>>,
    pub source_width: usize,
    pub source_height: usize,
    pub streamed_source_width: usize,
}
impl HierarchyFrame {
    pub fn width(&self) -> usize {
        self.foreground_topology[0].len()
    }
    pub fn height(&self) -> usize {
        self.foreground_topology.len()
    }
}

fn normalize_mask(image: &[Vec<u8>]) -> Result<Vec<Vec<u8>>, String> {
    if image.is_empty() || image[0].is_empty() {
        return Err("mask must not be empty".into());
    }
    let width = image[0].len();
    if image.iter().any(|r| r.len() != width) {
        return Err("all mask rows must have the same width".into());
    }
    Ok(image
        .iter()
        .map(|r| r.iter().map(|&p| u8::from(p != 0)).collect())
        .collect())
}
fn normalize_raw(image: &[Vec<u8>]) -> Result<Vec<Vec<u8>>, String> {
    if image.is_empty() || image[0].is_empty() {
        return Err("raw image must not be empty".into());
    }
    let width = image[0].len();
    if image.iter().any(|r| r.len() != width) {
        return Err("all raw image rows must have the same width".into());
    }
    Ok(image.to_vec())
}
fn pad_even(rows: &[Vec<u8>]) -> Vec<Vec<u8>> {
    if rows[0].len().is_multiple_of(2) {
        rows.to_vec()
    } else {
        rows.iter()
            .map(|r| {
                let mut r = r.clone();
                r.push(0);
                r
            })
            .collect()
    }
}

pub fn build_hierarchy_frame(
    raw_image: &[Vec<u8>],
    mask: Option<&[Vec<u8>]>,
) -> Result<HierarchyFrame, String> {
    let source_raw = normalize_raw(raw_image)?;
    let source_mask = if let Some(mask) = mask {
        let mask = normalize_mask(mask)?;
        if mask.len() != source_raw.len() || mask[0].len() != source_raw[0].len() {
            return Err("raw image and mask dimensions must match".into());
        }
        mask
    } else {
        source_raw
            .iter()
            .map(|r| r.iter().map(|&p| u8::from(p != 0)).collect())
            .collect()
    };
    let source_height = source_mask.len();
    let source_width = source_mask[0].len();
    let masks = pad_even(&source_mask);
    let raws = pad_even(&source_raw);
    let streamed_source_width = masks[0].len();
    let width = streamed_source_width + 6;
    let height = source_height + 6;
    let mut foreground_topology = vec![vec![0; width]; height];
    let mut foreground_feature = foreground_topology.clone();
    let mut foreground_raw = foreground_topology.clone();
    let mut background = foreground_topology.clone();
    for y in 0..height {
        for x in 0..width {
            let d = x.min(y).min(width - 1 - x).min(height - 1 - y);
            if d == 1 {
                foreground_topology[y][x] = 1
            } else if d == 2 {
                background[y][x] = 1
            } else if d >= 3 {
                let m = masks[y - 3][x - 3];
                let raw = raws[y - 3][x - 3];
                foreground_topology[y][x] = m;
                foreground_feature[y][x] = m;
                foreground_raw[y][x] = raw;
                background[y][x] = 1 - m;
            }
        }
    }
    Ok(HierarchyFrame {
        foreground_topology,
        foreground_feature,
        foreground_raw,
        background,
        source_width,
        source_height,
        streamed_source_width,
    })
}

pub fn empty_hierarchical_feature(_width: usize, _height: usize) -> HierarchicalFeature {
    HierarchicalFeature
}
pub fn include_hierarchical_point(
    _feature: HierarchicalFeature,
    _x: usize,
    _y: usize,
) -> HierarchicalFeature {
    HierarchicalFeature
}
pub fn merge_hierarchical_features(
    _left: HierarchicalFeature,
    _right: HierarchicalFeature,
) -> HierarchicalFeature {
    HierarchicalFeature
}
pub fn include_moment_point(feature: MomentFeature, x: usize, y: usize, raw: u8) -> MomentFeature {
    MomentFeature {
        area: feature.area + 1,
        zom: feature.zom + raw as u64,
        x_fom: feature.x_fom + x as u64 * raw as u64,
        y_fom: feature.y_fom + y as u64 * raw as u64,
    }
}
pub fn merge_moment_features(a: MomentFeature, b: MomentFeature) -> MomentFeature {
    MomentFeature {
        area: a.area + b.area,
        zom: a.zom + b.zom,
        x_fom: a.x_fom + b.x_fom,
        y_fom: a.y_fom + b.y_fom,
    }
}
pub fn unframe_hierarchical_feature(_feature: HierarchicalFeature) -> HierarchicalFeature {
    HierarchicalFeature
}
pub fn unframe_moment_feature(feature: MomentFeature) -> MomentFeature {
    MomentFeature {
        area: feature.area,
        zom: feature.zom,
        x_fom: feature.x_fom - 3 * feature.zom,
        y_fom: feature.y_fom - 3 * feature.zom,
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PreframedHierarchyConfig {
    pub centroid_width: Option<usize>,
    pub centroid_height: Option<usize>,
    pub use_coarse_child_key: bool,
    pub use_full_log_mass_child_key: bool,
    pub midpoint_reflection_reciprocal: bool,
}
#[derive(Clone, Debug)]
pub struct PreframedHierarchicalTwoPPCReferenceModel {
    pub width: usize,
    pub height: usize,
    pub centroid_width: usize,
    pub centroid_height: usize,
    pub use_coarse_child_key: bool,
    pub use_full_log_mass_child_key: bool,
    pub midpoint_reflection_reciprocal: bool,
}

impl PreframedHierarchicalTwoPPCReferenceModel {
    pub fn new(
        width: usize,
        height: usize,
        config: PreframedHierarchyConfig,
    ) -> Result<Self, String> {
        if width == 0 || height == 0 {
            return Err("dimensions must be positive".into());
        }
        if !width.is_multiple_of(2) {
            return Err("2ppc streamed width must be even".into());
        }
        if config.use_coarse_child_key && config.use_full_log_mass_child_key {
            return Err("coarse and full-log-mass child sort modes are mutually exclusive".into());
        }
        Ok(Self {
            width,
            height,
            centroid_width: config.centroid_width.unwrap_or(width),
            centroid_height: config.centroid_height.unwrap_or(height),
            use_coarse_child_key: config.use_coarse_child_key,
            use_full_log_mass_child_key: config.use_full_log_mass_child_key,
            midpoint_reflection_reciprocal: config.midpoint_reflection_reciprocal,
        })
    }
    pub fn feature_groups(
        &self,
        raw: &[Vec<u8>],
        mask: &[Vec<u8>],
        feature_valid: &[Vec<u8>],
        source_y: &[Vec<usize>],
    ) -> Result<Vec<HierarchicalFeatureGroup>, String> {
        validate_preframed(self.width, self.height, raw, mask, feature_valid, source_y)?;
        preframed_groups_streaming(
            self.width,
            self.height,
            raw,
            mask,
            feature_valid,
            source_y,
            self.centroid_width,
            self.centroid_height,
            self.use_coarse_child_key,
            self.use_full_log_mass_child_key,
            self.midpoint_reflection_reciprocal,
            9,
            false,
        )
    }

    /// Return every inner candidate for each outer group, without applying
    /// the hardware's nine-entry retained-list policy or short-list filter.
    pub fn feature_candidate_groups(
        &self,
        raw: &[Vec<u8>],
        mask: &[Vec<u8>],
        feature_valid: &[Vec<u8>],
        source_y: &[Vec<usize>],
    ) -> Result<Vec<HierarchicalFeatureGroup>, String> {
        validate_preframed(self.width, self.height, raw, mask, feature_valid, source_y)?;
        preframed_groups_streaming(
            self.width,
            self.height,
            raw,
            mask,
            feature_valid,
            source_y,
            self.centroid_width,
            self.centroid_height,
            self.use_coarse_child_key,
            self.use_full_log_mass_child_key,
            self.midpoint_reflection_reciprocal,
            self.width * self.height,
            true,
        )
    }
    pub fn output_stream(
        &self,
        raw: &[Vec<u8>],
        mask: &[Vec<u8>],
        feature_valid: &[Vec<u8>],
        source_y: &[Vec<usize>],
    ) -> Result<Vec<HierarchicalOutput>, String> {
        Ok(groups_to_stream(self.feature_groups(
            raw,
            mask,
            feature_valid,
            source_y,
        )?))
    }
}

fn validate_preframed(
    width: usize,
    height: usize,
    raw: &[Vec<u8>],
    mask: &[Vec<u8>],
    valid: &[Vec<u8>],
    source_y: &[Vec<usize>],
) -> Result<(), String> {
    for (name, actual_h, good_width) in [
        ("raw", raw.len(), raw.iter().all(|r| r.len() == width)),
        ("mask", mask.len(), mask.iter().all(|r| r.len() == width)),
        (
            "feature-valid",
            valid.len(),
            valid.iter().all(|r| r.len() == width),
        ),
        (
            "source-y",
            source_y.len(),
            source_y.iter().all(|r| r.len() == width),
        ),
    ] {
        if actual_h != height || !good_width {
            return Err(format!("{name} input must match model dimensions"));
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct OuterFeatureState {
    source_present: bool,
    child_list_id: Option<usize>,
}

type SharedChildArena = Rc<RefCell<ChildArenaReferenceModel<InnerFeature>>>;
type OuterFeatureCallbacks = CallbackFeatureModel<OuterFeatureState, OuterFeatureState>;
type InnerMomentCallbacks = CallbackFeatureModel<MomentFeature, MomentFeature>;

#[allow(clippy::too_many_arguments)]
fn preframed_groups_streaming(
    width: usize,
    height: usize,
    raw: &[Vec<u8>],
    mask: &[Vec<u8>],
    feature_valid: &[Vec<u8>],
    source_y: &[Vec<usize>],
    centroid_width: usize,
    centroid_height: usize,
    use_coarse: bool,
    use_full_log_mass: bool,
    midpoint: bool,
    max_child_list_len: usize,
    emit_short_lists: bool,
) -> Result<Vec<HierarchicalFeatureGroup>, String> {
    let mask = normalize_mask(mask)?;
    let feature_valid = normalize_mask(feature_valid)?;
    let foreground_payload = (0..height)
        .map(|y| {
            (0..width)
                .map(|x| u64::from(mask[y][x] != 0 && feature_valid[y][x] != 0))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let background = mask
        .iter()
        .map(|row| row.iter().map(|&pixel| 1 - pixel).collect::<Vec<_>>())
        .collect::<Vec<_>>();
    let background_payload = (0..height)
        .map(|y| {
            (0..width)
                .map(|x| {
                    if background[y][x] != 0 && feature_valid[y][x] != 0 {
                        1 + u64::from(raw[y][x]) + ((source_y[y][x] as u64) << 8)
                    } else {
                        0
                    }
                })
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();

    hierarchical_groups_streaming(
        &mask,
        &foreground_payload,
        &background,
        &background_payload,
        centroid_width,
        centroid_height,
        0,
        use_coarse,
        use_full_log_mass,
        midpoint,
        max_child_list_len,
        emit_short_lists,
    )
}

#[allow(clippy::too_many_arguments)]
fn hierarchical_groups_streaming(
    outer_topology: &[Vec<u8>],
    outer_payload: &[Vec<u64>],
    inner_topology: &[Vec<u8>],
    inner_payload: &[Vec<u64>],
    centroid_width: usize,
    centroid_height: usize,
    x_offset: usize,
    use_coarse: bool,
    use_full_log_mass: bool,
    midpoint: bool,
    max_child_list_len: usize,
    emit_short_lists: bool,
) -> Result<Vec<HierarchicalFeatureGroup>, String> {
    let height = outer_topology.len();
    let width = outer_topology[0].len();
    let arena = make_child_arena(
        width,
        height,
        use_coarse || use_full_log_mass,
        max_child_list_len,
    )?;
    let mut outer = make_outer_model(width, height, Rc::clone(&arena))?;
    let mut inner = make_inner_model(width, height, x_offset)?;
    let outer_inputs = iter_2ppc_pairs(outer_topology)?;
    let inner_inputs = iter_2ppc_pairs(inner_topology)?;
    let pairs_per_row = width / 2;
    let mut outputs = Vec::new();
    let mut delayed_inner_result = TwoPpcOutputs {
        fv_valid: 0,
        fv_data: MomentFeature::default(),
        fv_lane: None,
    };

    for (index, (outer_input, inner_input)) in
        outer_inputs.into_iter().zip(inner_inputs).enumerate()
    {
        let y = index / pairs_per_row;
        let pair = index % pairs_per_row;
        let x = 2 * pair;
        let outer_result = outer.step(
            outer_input,
            Some((outer_payload[y][x], outer_payload[y][x + 1])),
        )?;
        let inner_result = inner.step(
            inner_input,
            Some((inner_payload[y][x], inner_payload[y][x + 1])),
        )?;

        add_inner_result_to_outer(
            &mut outer,
            &arena,
            delayed_inner_result,
            centroid_width,
            centroid_height,
            use_coarse,
            use_full_log_mass,
            midpoint,
        )?;
        if outer_result.fv_valid != 0
            && let Some(group) = read_outer_group(&arena, outer_result.fv_data, emit_short_lists)?
        {
            outputs.push(group);
        }
        delayed_inner_result = inner_result;

        if outer_input.eol != 0 {
            let outer_result = outer.step(TwoPpcInputs::default(), None)?;
            add_inner_result_to_outer(
                &mut outer,
                &arena,
                delayed_inner_result,
                centroid_width,
                centroid_height,
                use_coarse,
                use_full_log_mass,
                midpoint,
            )?;
            if outer_result.fv_valid != 0
                && let Some(group) =
                    read_outer_group(&arena, outer_result.fv_data, emit_short_lists)?
            {
                outputs.push(group);
            }
            delayed_inner_result = TwoPpcOutputs {
                fv_valid: 0,
                fv_data: MomentFeature::default(),
                fv_lane: None,
            };
        }
    }
    Ok(outputs)
}

fn make_child_arena(
    width: usize,
    height: usize,
    use_coarse: bool,
    max_child_list_len: usize,
) -> Result<SharedChildArena, String> {
    let entry_count = width.saturating_mul(height).max(1);
    let arena = if use_coarse {
        ChildArenaReferenceModel::with_sort_key(
            entry_count,
            max_child_list_len,
            true,
            |feature: &InnerFeature| feature.child_sort_key,
        )
    } else {
        ChildArenaReferenceModel::with_sort_key(
            entry_count,
            max_child_list_len,
            false,
            |feature: &InnerFeature| feature.area,
        )
    }
    .map_err(|error| error.to_string())?;
    Ok(Rc::new(RefCell::new(arena)))
}

fn make_outer_model(
    width: usize,
    height: usize,
    arena: SharedChildArena,
) -> Result<TwoPpcReferenceModel<OuterFeatureCallbacks>, String> {
    let merge_arena = Rc::clone(&arena);
    let feature_model = CallbackFeatureModel::new(
        OuterFeatureState::default(),
        |_x, _y, feature_pixel| OuterFeatureState {
            source_present: feature_pixel != 0,
            child_list_id: None,
        },
        move |left, right| OuterFeatureState {
            source_present: left.source_present || right.source_present,
            child_list_id: merge_arena
                .borrow_mut()
                .merge(left.child_list_id, right.child_list_id)
                .expect("outer child-list merge must preserve valid arena handles"),
        },
        |feature| *feature,
    );
    TwoPpcReferenceModel::with_feature_model(width, height, true, feature_model)
}

fn make_inner_model(
    width: usize,
    height: usize,
    x_offset: usize,
) -> Result<TwoPpcReferenceModel<InnerMomentCallbacks>, String> {
    let feature_model = CallbackFeatureModel::new(
        MomentFeature::default(),
        move |x, _y, feature_pixel| {
            if feature_pixel == 0 {
                return MomentFeature::default();
            }
            let payload = feature_pixel - 1;
            let raw = payload & 0xff;
            let source_y = payload >> 8;
            MomentFeature {
                area: 1,
                zom: raw,
                x_fom: (x - x_offset) as u64 * raw,
                y_fom: source_y * raw,
            }
        },
        merge_moment_features,
        |feature| *feature,
    );
    TwoPpcReferenceModel::with_feature_model(width, height, false, feature_model)
}

#[allow(clippy::too_many_arguments)]
fn add_inner_result_to_outer(
    outer: &mut TwoPpcReferenceModel<OuterFeatureCallbacks>,
    arena: &SharedChildArena,
    inner_result: TwoPpcOutputs<MomentFeature>,
    centroid_width: usize,
    centroid_height: usize,
    use_coarse: bool,
    use_full_log_mass: bool,
    midpoint: bool,
) -> Result<(), String> {
    if inner_result.fv_valid == 0 {
        return Ok(());
    }
    let lane = inner_result
        .fv_lane
        .ok_or_else(|| "inner feature output did not identify its close lane".to_owned())?;
    let Some((bank, head)) = outer.parent_head_taps[lane] else {
        return Ok(());
    };
    let outer_feature = outer.feature_for_head(bank, head)?;
    let centroid = compress_centroid(
        inner_result.fv_data.zom,
        inner_result.fv_data.x_fom,
        inner_result.fv_data.y_fom,
        centroid_width,
        centroid_height,
        8,
        midpoint,
    )?;
    let log_mass = full_log_mass_child_sort_key(inner_result.fv_data.zom);
    let child_sort_key = if use_full_log_mass {
        log_mass
    } else if use_coarse {
        tagdetect_child_sort_key(inner_result.fv_data.area, inner_result.fv_data.zom)
    } else {
        0
    };
    let child_feature = InnerCentroidFeature::from_compressed(
        inner_result.fv_data.area,
        centroid,
        log_mass,
        child_sort_key,
    );

    if let Some(child_list_id) = outer_feature.child_list_id {
        arena
            .borrow_mut()
            .add(child_list_id, child_feature)
            .map_err(|error| error.to_string())?;
    } else {
        let child_list_id = {
            let mut arena = arena.borrow_mut();
            let child_list_id = arena.allocate().map_err(|error| error.to_string())?;
            arena
                .init(child_list_id, child_feature)
                .map_err(|error| error.to_string())?;
            child_list_id
        };
        outer.replace_feature_for_head(
            bank,
            head,
            OuterFeatureState {
                child_list_id: Some(child_list_id),
                ..outer_feature
            },
        )?;
    }
    Ok(())
}

fn read_outer_group(
    arena: &SharedChildArena,
    outer_feature: OuterFeatureState,
    emit_short_lists: bool,
) -> Result<Option<HierarchicalFeatureGroup>, String> {
    if !outer_feature.source_present {
        return Ok(None);
    }
    let inner_features = if let Some(child_list_id) = outer_feature.child_list_id {
        let read = if emit_short_lists {
            arena.borrow_mut().read_all(child_list_id)
        } else {
            arena.borrow_mut().read(child_list_id)
        }
        .map_err(|error| error.to_string())?;
        read.features
    } else {
        Vec::new()
    };
    Ok(Some(HierarchicalFeatureGroup {
        outer: HierarchicalFeature,
        inner_features,
    }))
}

fn groups_to_stream(groups: Vec<HierarchicalFeatureGroup>) -> Vec<HierarchicalOutput> {
    let mut output = Vec::new();
    for group in groups {
        let len = group.inner_features.len();
        for (index, feature) in group.inner_features.into_iter().enumerate() {
            output.push(HierarchicalOutput {
                inner_feature: Some(feature),
                end_inner: index + 1 == len,
            });
        }
    }
    output
}

#[derive(Clone, Debug)]
pub struct HierarchicalTwoPPCReferenceModel {
    pub source_width: usize,
    pub source_height: usize,
    pub streamed_source_width: usize,
    pub width: usize,
    pub height: usize,
}
impl HierarchicalTwoPPCReferenceModel {
    pub fn new(source_width: usize, source_height: usize) -> Result<Self, String> {
        if source_width == 0 || source_height == 0 {
            return Err("source dimensions must be positive".into());
        }
        let streamed = source_width + (source_width & 1);
        Ok(Self {
            source_width,
            source_height,
            streamed_source_width: streamed,
            width: streamed + 6,
            height: source_height + 6,
        })
    }
    pub fn features(
        &self,
        raw: &[Vec<u8>],
        mask: Option<&[Vec<u8>]>,
    ) -> Result<Vec<HierarchicalFeature>, String> {
        self.feature_groups(raw, mask)?;
        Ok(Vec::new())
    }
    pub fn feature_groups(
        &self,
        raw: &[Vec<u8>],
        mask: Option<&[Vec<u8>]>,
    ) -> Result<Vec<HierarchicalFeatureGroup>, String> {
        let frame = build_hierarchy_frame(raw, mask)?;
        if frame.source_width != self.source_width || frame.source_height != self.source_height {
            return Err("image dimensions must match model dimensions".into());
        }
        let outer_payload = frame
            .foreground_feature
            .iter()
            .map(|row| row.iter().map(|&pixel| u64::from(pixel)).collect())
            .collect::<Vec<Vec<_>>>();
        let mut inner_payload = vec![vec![0; frame.width()]; frame.height()];
        for (y, row) in inner_payload
            .iter_mut()
            .enumerate()
            .skip(3)
            .take(self.source_height)
        {
            for (x, payload) in row
                .iter_mut()
                .enumerate()
                .skip(3)
                .take(self.streamed_source_width)
            {
                if frame.background[y][x] != 0 {
                    *payload = 1 + u64::from(frame.foreground_raw[y][x]) + (((y - 3) as u64) << 8);
                }
            }
        }
        hierarchical_groups_streaming(
            &frame.foreground_topology,
            &outer_payload,
            &frame.background,
            &inner_payload,
            self.streamed_source_width,
            self.source_height,
            3,
            false,
            false,
            false,
            9,
            false,
        )
    }
    pub fn output_stream(
        &self,
        raw: &[Vec<u8>],
        mask: Option<&[Vec<u8>]>,
    ) -> Result<Vec<HierarchicalOutput>, String> {
        Ok(groups_to_stream(self.feature_groups(raw, mask)?))
    }
}

pub fn run_hierarchical_2ppc_reference(
    raw: &[Vec<u8>],
    mask: Option<&[Vec<u8>]>,
) -> Result<Vec<HierarchicalFeature>, String> {
    HierarchicalTwoPPCReferenceModel::new(raw[0].len(), raw.len())?.features(raw, mask)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hierarchy_frame_has_three_layer_contract() {
        let raw = vec![vec![1; 5]; 4];
        let f = build_hierarchy_frame(&raw, None).unwrap();
        assert_eq!(
            (
                f.source_width,
                f.streamed_source_width,
                f.width(),
                f.height()
            ),
            (5, 6, 12, 10)
        );
        assert_eq!(f.foreground_topology[1][1], 1);
        assert_eq!(f.background[2][2], 1);
    }
    #[test]
    fn moment_unframing_subtracts_border() {
        let m = MomentFeature {
            area: 2,
            zom: 10,
            x_fom: 50,
            y_fom: 70,
        };
        assert_eq!(
            unframe_moment_feature(m),
            MomentFeature {
                area: 2,
                zom: 10,
                x_fom: 20,
                y_fom: 40
            }
        );
    }
    #[test]
    fn preframed_keeps_largest_nine_holes() {
        let (w, h) = (48, 36);
        let mut mask = vec![vec![0; w]; h];
        for row in mask.iter_mut().take(28).skip(4) {
            for p in row.iter_mut().take(44).skip(4) {
                *p = 1;
            }
        }
        for (x, y) in [
            (8, 8),
            (12, 8),
            (16, 8),
            (20, 8),
            (24, 8),
            (28, 8),
            (32, 8),
            (36, 8),
            (8, 12),
            (12, 12),
            (16, 12),
        ] {
            mask[y][x] = 0;
        }
        let raw = vec![vec![1; w]; h];
        let valid = vec![vec![1; w]; h];
        let source_y = (0..h).map(|y| vec![y; w]).collect::<Vec<_>>();
        let c = PreframedHierarchyConfig {
            use_coarse_child_key: true,
            ..PreframedHierarchyConfig::default()
        };
        let model = PreframedHierarchicalTwoPPCReferenceModel::new(w, h, c).unwrap();
        let groups = model
            .feature_groups(&raw, &mask, &valid, &source_y)
            .unwrap();
        let group = groups
            .iter()
            .find(|g| !g.inner_features.is_empty())
            .unwrap();
        assert_eq!(group.inner_features.len(), 9);
    }
}
