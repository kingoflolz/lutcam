//! Two-pixels-per-clock LRCCA reference behavior.

use std::sync::Arc;

use crate::classical::{BBox, decode_bbox, empty_bbox, encode_bbox, include_point, merge_boxes};

pub const INVALID_LABEL: isize = -1;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HeadWord {
    pub mapped: u8,
    pub head: isize,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FeatureWord<F = u128> {
    pub feature: F,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HeadRefWord {
    pub head: isize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DataWord<F = u128> {
    Feature(FeatureWord<F>),
    HeadRef(HeadRefWord),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PToken<F = u128> {
    pub valid: u8,
    pub index: isize,
    pub next: isize,
    pub has_next: u8,
    pub mapped: u8,
    pub head: isize,
    pub tail: isize,
    pub data: Option<F>,
}

impl<F> Default for PToken<F> {
    fn default() -> Self {
        Self {
            valid: 0,
            index: INVALID_LABEL,
            next: INVALID_LABEL,
            has_next: 0,
            mapped: 0,
            head: INVALID_LABEL,
            tail: INVALID_LABEL,
            data: None,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TwoPpcInputs {
    pub rst: u8,
    pub datavalid: u8,
    pub d0: u64,
    pub d1: u64,
    pub sof: u8,
    pub eol: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TwoPpcOutputs<E = u128> {
    pub fv_valid: u8,
    pub fv_data: E,
    pub fv_lane: Option<usize>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TwoPpcEvents {
    pub ec0: u8,
    pub ec1: u8,
    pub ep0: u8,
    pub ep1: u8,
    pub o0: u8,
    pub o1: u8,
    pub sp0: u8,
    pub sp1: u8,
    pub sc0: u8,
    pub sc1: u8,
}

/// Compatibility aliases using the spelling from the Python reference.
pub type TwoPPCInputs = TwoPpcInputs;
pub type TwoPPCOutputs<E = u128> = TwoPpcOutputs<E>;
pub type TwoPPCEvents = TwoPpcEvents;

/// Feature arithmetic used by the streamed 2-PPC connected-component model.
///
/// The three operations correspond directly to the feature callbacks in the
/// original LRCCA reference: make one pixel's contribution, merge two partial
/// component features, and encode a retired component for the output stream.
/// Implementations are cloneable so convenience methods can run a fresh model
/// without mutating the caller's cycle state.
pub trait TwoPpcFeatureModel: Clone {
    type Feature: Clone;
    type Encoded;

    fn empty_feature(&self, width: usize, height: usize) -> Self::Feature;

    fn pixel_feature(
        &self,
        x: usize,
        y: usize,
        feature_pixel: u64,
        width: usize,
        height: usize,
    ) -> Self::Feature;

    fn merge_features(&self, left: Self::Feature, right: Self::Feature) -> Self::Feature;

    fn encode_feature(&self, feature: &Self::Feature, width: usize, height: usize)
    -> Self::Encoded;
}

/// Bounding-box/area feature behavior used by production TwoPPC tests.
#[derive(Clone, Copy, Debug, Default)]
pub struct BBoxFeatureModel;

impl TwoPpcFeatureModel for BBoxFeatureModel {
    type Feature = BBox;
    type Encoded = u128;

    fn empty_feature(&self, width: usize, height: usize) -> Self::Feature {
        empty_bbox(width, height)
    }

    fn pixel_feature(
        &self,
        x: usize,
        y: usize,
        feature_pixel: u64,
        width: usize,
        height: usize,
    ) -> Self::Feature {
        let empty = empty_bbox(width, height);
        if feature_pixel == 0 {
            empty
        } else {
            include_point(empty, x, y)
        }
    }

    fn merge_features(&self, left: Self::Feature, right: Self::Feature) -> Self::Feature {
        merge_boxes(left, right)
    }

    fn encode_feature(
        &self,
        feature: &Self::Feature,
        width: usize,
        height: usize,
    ) -> Self::Encoded {
        encode_bbox(*feature, width, height)
    }
}

/// Cloneable closure-backed equivalent of Python's feature callback trio.
pub struct CallbackFeatureModel<F, E> {
    empty: F,
    pixel: Arc<dyn Fn(usize, usize, u64) -> F>,
    merge: Arc<dyn Fn(F, F) -> F>,
    encode: Arc<dyn Fn(&F) -> E>,
}

impl<F: Clone, E> Clone for CallbackFeatureModel<F, E> {
    fn clone(&self) -> Self {
        Self {
            empty: self.empty.clone(),
            pixel: Arc::clone(&self.pixel),
            merge: Arc::clone(&self.merge),
            encode: Arc::clone(&self.encode),
        }
    }
}

impl<F, E> CallbackFeatureModel<F, E> {
    pub fn new(
        empty: F,
        pixel: impl Fn(usize, usize, u64) -> F + 'static,
        merge: impl Fn(F, F) -> F + 'static,
        encode: impl Fn(&F) -> E + 'static,
    ) -> Self {
        Self {
            empty,
            pixel: Arc::new(pixel),
            merge: Arc::new(merge),
            encode: Arc::new(encode),
        }
    }
}

impl<F: Clone, E> TwoPpcFeatureModel for CallbackFeatureModel<F, E> {
    type Feature = F;
    type Encoded = E;

    fn empty_feature(&self, _width: usize, _height: usize) -> Self::Feature {
        self.empty.clone()
    }

    fn pixel_feature(
        &self,
        x: usize,
        y: usize,
        feature_pixel: u64,
        _width: usize,
        _height: usize,
    ) -> Self::Feature {
        (self.pixel)(x, y, feature_pixel)
    }

    fn merge_features(&self, left: Self::Feature, right: Self::Feature) -> Self::Feature {
        (self.merge)(left, right)
    }

    fn encode_feature(
        &self,
        feature: &Self::Feature,
        _width: usize,
        _height: usize,
    ) -> Self::Encoded {
        (self.encode)(feature)
    }
}

pub fn pair_events(a: u8, b0: u8, b1: u8, c: u8, d0: u8, d1: u8) -> TwoPpcEvents {
    let [a, b0, b1, c, d0, d1] = [a, b0, b1, c, d0, d1].map(|v| u8::from(v != 0));
    TwoPpcEvents {
        ec0: c & (1 - d0),
        ec1: d0 & (1 - d1),
        ep0: a & (1 - b0),
        ep1: b0 & (1 - b1),
        o0: b0 & d0 & ((1 - a) | (1 - c)),
        o1: b1 & d1 & ((1 - b0) | (1 - d0)),
        sp0: (1 - a) & b0,
        sp1: (1 - b0) & b1,
        sc0: (1 - c) & d0,
        sc1: (1 - d0) & d1,
    }
}

fn normalize_binary(image: &[Vec<u8>]) -> Result<Vec<Vec<u8>>, String> {
    if image.is_empty() || image[0].is_empty() {
        return Err("image must not be empty".into());
    }
    let width = image[0].len();
    if image.iter().any(|row| row.len() != width) {
        return Err("all rows must have the same width".into());
    }
    if !width.is_multiple_of(2) {
        return Err("2ppc streamed width must be even".into());
    }
    Ok(image
        .iter()
        .map(|row| row.iter().map(|&v| u8::from(v != 0)).collect())
        .collect())
}

fn mask_edges<T: Copy + Default>(image: &[Vec<T>]) -> Vec<Vec<T>> {
    let mut masked = image.to_vec();
    if masked.is_empty() {
        return masked;
    }
    let width = masked[0].len();
    for pixel in &mut masked[0] {
        *pixel = T::default();
    }
    if masked.len() > 1 {
        for pixel in masked.last_mut().unwrap() {
            *pixel = T::default();
        }
    }
    for row in masked
        .iter_mut()
        .skip(1)
        .take(image.len().saturating_sub(2))
    {
        row[0] = T::default();
        row[width - 1] = T::default();
    }
    masked
}

pub fn hole_fill_8_connectivity(image: &[Vec<u8>]) -> Result<Vec<Vec<u8>>, String> {
    let rows = mask_edges(&normalize_binary(image)?);
    let (height, width) = (rows.len(), rows[0].len());
    let mut filled = rows.clone();
    for y in 0..height {
        for x in 0..width {
            let top = if y > 0 { rows[y - 1][x] } else { 0 };
            let left = if x > 0 { rows[y][x - 1] } else { 0 };
            let right = if x + 1 < width { rows[y][x + 1] } else { 0 };
            if top != 0 && (left != 0 || right != 0) {
                filled[y][x] = 1;
            }
        }
    }
    Ok(filled)
}

pub fn iter_2ppc_pairs(image: &[Vec<u8>]) -> Result<Vec<TwoPpcInputs>, String> {
    let rows = normalize_binary(image)?;
    let pairs = rows[0].len() / 2;
    let mut result = Vec::with_capacity(rows.len() * pairs);
    for (y, row) in rows.iter().enumerate() {
        for pair in 0..pairs {
            result.push(TwoPpcInputs {
                datavalid: 1,
                d0: row[2 * pair] as u64,
                d1: row[2 * pair + 1] as u64,
                sof: u8::from(y == 0 && pair == 0),
                eol: u8::from(pair + 1 == pairs),
                ..Default::default()
            });
        }
    }
    Ok(result)
}

pub fn iter_2ppc_feature_pairs(image: &[Vec<u64>]) -> Result<Vec<TwoPpcInputs>, String> {
    if image.is_empty() || image[0].is_empty() {
        return Err("feature image must not be empty".into());
    }
    let width = image[0].len();
    if !width.is_multiple_of(2) {
        return Err("2ppc streamed width must be even".into());
    }
    if image.iter().any(|r| r.len() != width) {
        return Err("all feature image rows must have the same width".into());
    }
    let pairs = width / 2;
    let mut result = Vec::with_capacity(image.len() * pairs);
    for (y, row) in image.iter().enumerate() {
        for pair in 0..pairs {
            result.push(TwoPpcInputs {
                datavalid: 1,
                d0: row[2 * pair],
                d1: row[2 * pair + 1],
                sof: u8::from(y == 0 && pair == 0),
                eol: u8::from(pair + 1 == pairs),
                ..Default::default()
            });
        }
    }
    Ok(result)
}

#[derive(Clone, Copy, Debug)]
struct HoleFillPendingPair {
    inputs: TwoPpcInputs,
    raw_d0: u8,
    raw_d1: u8,
    feature_d0: u64,
    feature_d1: u64,
}

#[derive(Clone, Copy, Debug)]
struct PairPayload {
    x: usize,
    c_left: u8,
    d0: u8,
    d1: u8,
    feature_d0: u64,
    feature_d1: u64,
}

/// Lane-ordered, cycle-oriented model of the production TwoPPC datapath.
///
/// Unlike a connected-component post-process, this retains the two alternating
/// Next/Head/Data banks, current-run accumulator, previous-run token, streamed
/// retirement timing, and per-lane parent-head taps used by the hierarchy.
#[derive(Clone)]
pub struct TwoPpcReferenceModel<M = BBoxFeatureModel>
where
    M: TwoPpcFeatureModel,
{
    pub width: usize,
    pub height: usize,
    pub hole_fill: bool,
    pub parent_head_taps: [Option<(usize, usize)>; 2],
    feature_model: M,
    label_count: usize,
    next_bank: [Vec<usize>; 2],
    head_bank: [Vec<HeadWord>; 2],
    data_bank: [Vec<DataWord<M::Feature>>; 2],
    tail_scratch: Vec<usize>,
    row: usize,
    pair_index: usize,
    started: bool,
    done: bool,
    prev_row: Vec<u8>,
    curr_row: Vec<u8>,
    a: u8,
    c_pixel: u8,
    curr_bank: usize,
    prev_bank: usize,
    current_label: usize,
    fused: bool,
    head: isize,
    tail: isize,
    current_feature: M::Feature,
    current_run_active: bool,
    previous_issue: usize,
    previous: PToken<M::Feature>,
    last_output_lane: Option<usize>,
    ingress_row: usize,
    ingress_pair_index: usize,
    ingress_started: bool,
    ingress_done: bool,
    hole_pending: Option<HoleFillPendingPair>,
    hole_flush_pending: bool,
    hole_prev_raw_pairs: Vec<(u8, u8)>,
    hole_curr_raw_pairs: Vec<(u8, u8)>,
    hole_prev_filled_pairs: Vec<(u8, u8)>,
    hole_curr_filled_pairs: Vec<(u8, u8)>,
}

pub type TwoPPCReferenceModel<M = BBoxFeatureModel> = TwoPpcReferenceModel<M>;
pub type TwoPPCReference<M = BBoxFeatureModel> = TwoPpcReferenceModel<M>;
pub type TwoPPCDummy<M = BBoxFeatureModel> = TwoPpcReferenceModel<M>;

impl TwoPpcReferenceModel<BBoxFeatureModel> {
    pub fn new(width: usize, height: usize, hole_fill: bool) -> Result<Self, String> {
        Self::with_feature_model(width, height, hole_fill, BBoxFeatureModel)
    }

    pub fn boxes(&self, image: &[Vec<u8>]) -> Result<Vec<BBox>, String> {
        Ok(self
            .feature_outputs(image, None)?
            .into_iter()
            .map(|value| decode_bbox(value, self.width, self.height))
            .collect())
    }
}

impl<M> TwoPpcReferenceModel<M>
where
    M: TwoPpcFeatureModel,
{
    pub fn with_feature_model(
        width: usize,
        height: usize,
        hole_fill: bool,
        feature_model: M,
    ) -> Result<Self, String> {
        if width == 0 || height == 0 {
            return Err("width and height must be positive".into());
        }
        if !width.is_multiple_of(2) {
            return Err("2ppc streamed width must be even".into());
        }
        let empty = feature_model.empty_feature(width, height);
        let mut model = Self {
            width,
            height,
            hole_fill,
            parent_head_taps: [None, None],
            feature_model,
            label_count: width.div_ceil(2),
            next_bank: [Vec::new(), Vec::new()],
            head_bank: [Vec::new(), Vec::new()],
            data_bank: [Vec::new(), Vec::new()],
            tail_scratch: Vec::new(),
            row: 0,
            pair_index: 0,
            started: false,
            done: false,
            prev_row: Vec::new(),
            curr_row: Vec::new(),
            a: 0,
            c_pixel: 0,
            curr_bank: 0,
            prev_bank: 1,
            current_label: 0,
            fused: false,
            head: INVALID_LABEL,
            tail: INVALID_LABEL,
            current_feature: empty.clone(),
            current_run_active: false,
            previous_issue: 0,
            previous: PToken {
                data: Some(empty),
                ..Default::default()
            },
            last_output_lane: None,
            ingress_row: 0,
            ingress_pair_index: 0,
            ingress_started: false,
            ingress_done: false,
            hole_pending: None,
            hole_flush_pending: false,
            hole_prev_raw_pairs: Vec::new(),
            hole_curr_raw_pairs: Vec::new(),
            hole_prev_filled_pairs: Vec::new(),
            hole_curr_filled_pairs: Vec::new(),
        };
        model.reset();
        Ok(model)
    }

    pub fn pair_count(&self) -> usize {
        self.width / 2
    }

    pub fn reset(&mut self) {
        let labels = (0..self.label_count).collect::<Vec<_>>();
        self.next_bank = [labels.clone(), labels];
        let heads = (0..self.label_count)
            .map(|label| HeadWord {
                mapped: 0,
                head: label as isize,
            })
            .collect::<Vec<_>>();
        self.head_bank = [heads.clone(), heads];
        let empty = self.feature_zero();
        let data = (0..self.label_count)
            .map(|_| {
                DataWord::Feature(FeatureWord {
                    feature: empty.clone(),
                })
            })
            .collect::<Vec<_>>();
        self.data_bank = [data.clone(), data];
        self.tail_scratch = (0..self.label_count).collect();

        self.row = 0;
        self.pair_index = 0;
        self.started = false;
        self.done = false;
        self.prev_row = vec![0; self.width];
        self.curr_row = vec![0; self.width];
        self.a = 0;
        self.c_pixel = 0;
        self.curr_bank = 0;
        self.prev_bank = 1;
        self.current_label = 0;
        self.fused = false;
        self.head = INVALID_LABEL;
        self.tail = INVALID_LABEL;
        self.current_feature = empty.clone();
        self.current_run_active = false;
        self.previous_issue = 0;
        self.previous = PToken {
            data: Some(empty),
            ..Default::default()
        };
        self.parent_head_taps = [None, None];
        self.last_output_lane = None;

        self.ingress_row = 0;
        self.ingress_pair_index = 0;
        self.ingress_started = false;
        self.ingress_done = false;
        self.hole_pending = None;
        self.hole_flush_pending = false;
        self.hole_prev_raw_pairs = vec![(0, 0); self.pair_count()];
        self.hole_curr_raw_pairs = vec![(0, 0); self.pair_count()];
        self.hole_prev_filled_pairs = vec![(0, 0); self.pair_count()];
        self.hole_curr_filled_pairs = vec![(0, 0); self.pair_count()];
    }

    pub fn pair_events(&self, a: u8, b0: u8, b1: u8, c: u8, d0: u8, d1: u8) -> TwoPpcEvents {
        pair_events(a, b0, b1, c, d0, d1)
    }

    pub fn step(
        &mut self,
        input: TwoPpcInputs,
        feature: Option<(u64, u64)>,
    ) -> Result<TwoPpcOutputs<M::Encoded>, String> {
        if input.rst != 0 {
            self.reset();
            return Ok(self.empty_output());
        }
        if input.datavalid == 0 {
            if input.sof != 0 || input.eol != 0 {
                return Err("sof/eol are only valid with datavalid".into());
            }
            if self.hole_fill && self.hole_flush_pending {
                let retired = self.flush_hole_fill_row()?;
                return Ok(self.encode_output(retired));
            }
            return Ok(self.empty_output());
        }

        let retired = if self.hole_fill {
            self.consume_hole_fill_pair(input, feature)?
        } else {
            self.consume_pair(input, feature)?
        };
        Ok(self.encode_output(retired))
    }

    pub fn feature_outputs(
        &self,
        connectivity: &[Vec<u8>],
        feature: Option<&[Vec<u64>]>,
    ) -> Result<Vec<M::Encoded>, String> {
        let topology = normalize_binary(connectivity)?;
        if topology.len() != self.height || topology[0].len() != self.width {
            return Err("image dimensions must match the model dimensions".into());
        }
        if let Some(feature) = feature
            && (feature.len() != self.height || feature.iter().any(|row| row.len() != self.width))
        {
            return Err("feature image dimensions must match the model dimensions".into());
        }

        let mut model = Self::with_feature_model(
            self.width,
            self.height,
            self.hole_fill,
            self.feature_model.clone(),
        )?;
        let mut outputs = Vec::new();
        for (index, input) in iter_2ppc_pairs(&topology)?.into_iter().enumerate() {
            let pair = index % model.pair_count();
            let row = index / model.pair_count();
            let feature_pair = feature.map(|rows| (rows[row][2 * pair], rows[row][2 * pair + 1]));
            let eol = input.eol != 0;
            let output = model.step(input, feature_pair)?;
            if output.fv_valid != 0 {
                outputs.push(output.fv_data);
            }
            if model.hole_fill && eol {
                let output = model.step(TwoPpcInputs::default(), None)?;
                if output.fv_valid != 0 {
                    outputs.push(output.fv_data);
                }
            }
        }
        Ok(outputs)
    }

    pub fn feature_for_head(&self, bank: usize, head: usize) -> Result<M::Feature, String> {
        self.check_bank(bank)?;
        self.check_label_usize(head)?;
        if bank == self.curr_bank {
            return self.feature_from_current_head(head);
        }
        self.data_feature(bank, head)
    }

    pub fn replace_feature_for_head(
        &mut self,
        bank: usize,
        head: usize,
        feature: M::Feature,
    ) -> Result<(), String> {
        self.check_bank(bank)?;
        self.check_label_usize(head)?;
        if bank == self.curr_bank && self.active_current_head() == Some(head) {
            self.current_feature = feature.clone();
            if self.previous.valid != 0
                && self.previous.mapped != 0
                && self.previous.head == head as isize
            {
                self.previous.data = Some(feature);
            }
            return Ok(());
        }
        self.set_data_feature(bank, head, feature.clone())?;
        if self.previous.valid != 0
            && self.previous.mapped != 0
            && self.previous.head == head as isize
        {
            self.previous.data = Some(feature);
        }
        Ok(())
    }

    pub fn add_feature_to_head(
        &mut self,
        bank: usize,
        head: usize,
        feature: M::Feature,
    ) -> Result<(), String> {
        self.check_bank(bank)?;
        self.check_label_usize(head)?;
        if bank == self.curr_bank && self.active_current_head() == Some(head) {
            self.current_feature = self
                .feature_model
                .merge_features(self.current_feature.clone(), feature);
            if self.previous.valid != 0
                && self.previous.mapped != 0
                && self.previous.head == head as isize
            {
                self.previous.data = Some(self.current_feature.clone());
            }
            return Ok(());
        }
        let old = self.data_feature(bank, head)?;
        let merged = self.feature_model.merge_features(old, feature);
        self.set_data_feature(bank, head, merged.clone())?;
        if self.previous.valid != 0
            && self.previous.mapped != 0
            && self.previous.head == head as isize
        {
            self.previous.data = Some(merged);
        }
        Ok(())
    }

    /// Outputs are produced at retirement cycles; no deferred frame queue remains.
    pub fn drain(&mut self) -> Vec<TwoPpcOutputs<M::Encoded>> {
        Vec::new()
    }

    fn empty_output(&self) -> TwoPpcOutputs<M::Encoded> {
        TwoPpcOutputs {
            fv_valid: 0,
            fv_data: self.feature_model.encode_feature(
                &self.feature_zero(),
                self.width,
                self.height,
            ),
            fv_lane: None,
        }
    }

    fn encode_output(&self, retired: Option<M::Feature>) -> TwoPpcOutputs<M::Encoded> {
        match retired {
            Some(feature) => TwoPpcOutputs {
                fv_valid: 1,
                fv_data: self
                    .feature_model
                    .encode_feature(&feature, self.width, self.height),
                fv_lane: self.last_output_lane,
            },
            None => self.empty_output(),
        }
    }

    fn consume_pair(
        &mut self,
        input: TwoPpcInputs,
        feature: Option<(u64, u64)>,
    ) -> Result<Option<M::Feature>, String> {
        if self.done {
            return Err("frame already consumed; reset before driving another frame".into());
        }
        if !self.started {
            if input.sof == 0 {
                return Err("sof must be asserted on the first accepted pair".into());
            }
            self.started = true;
            self.start_row();
        } else if input.sof != 0 {
            return Err("sof may only be asserted on the first accepted pair".into());
        }
        let expected_eol = self.pair_index + 1 == self.pair_count();
        if (input.eol != 0) != expected_eol {
            return Err("eol must be asserted exactly on the final pair of each row".into());
        }

        let (d0, d1) = self.mask_edge_pair(input.d0, input.d1);
        let (feature_d0, feature_d1) = feature.unwrap_or((d0 as u64, d1 as u64));
        let (feature_d0, feature_d1) = self.mask_edge_feature_pair(feature_d0, feature_d1);
        let x = 2 * self.pair_index;
        let b0 = self.prev_row[x];
        let b1 = self.prev_row[x + 1];
        let events = pair_events(self.a, b0, b1, self.c_pixel, d0, d1);
        Self::assert_event_invariants(events)?;

        let c_left = self.c_pixel;
        self.parent_head_taps = [None, None];
        self.last_output_lane = None;
        let retired = self.process_pair_events(
            events,
            PairPayload {
                x,
                c_left,
                d0,
                d1,
                feature_d0,
                feature_d1,
            },
        )?;
        let taps = self.parent_head_taps;
        let lane = self.last_output_lane;

        self.curr_row[x] = d0;
        self.curr_row[x + 1] = d1;
        self.a = b1;
        self.c_pixel = d1;
        if expected_eol {
            self.finish_row()?;
        } else {
            self.pair_index += 1;
        }
        self.parent_head_taps = taps;
        self.last_output_lane = lane;
        Ok(retired)
    }

    fn consume_hole_fill_pair(
        &mut self,
        input: TwoPpcInputs,
        feature: Option<(u64, u64)>,
    ) -> Result<Option<M::Feature>, String> {
        if self.ingress_done {
            return Err("frame already consumed; reset before driving another frame".into());
        }
        if self.hole_flush_pending {
            return Err("hole-fill row flush must complete before accepting the next row".into());
        }
        if !self.ingress_started {
            if input.sof == 0 {
                return Err("sof must be asserted on the first accepted pair".into());
            }
            self.ingress_started = true;
        } else if input.sof != 0 {
            return Err("sof may only be asserted on the first accepted pair".into());
        }
        let expected_eol = self.ingress_pair_index + 1 == self.pair_count();
        if (input.eol != 0) != expected_eol {
            return Err("eol must be asserted exactly on the final pair of each row".into());
        }

        let (raw_d0, raw_d1) = self.mask_edge_pair_for(
            self.ingress_row,
            self.ingress_pair_index,
            input.d0,
            input.d1,
        );
        let (feature_d0, feature_d1) = feature.unwrap_or((input.d0, input.d1));
        let (feature_d0, feature_d1) = self.mask_edge_feature_pair_for(
            self.ingress_row,
            self.ingress_pair_index,
            feature_d0,
            feature_d1,
        );

        let retired = if self.hole_pending.is_some() {
            self.consume_pending_hole_fill_pair(raw_d0)?
        } else {
            None
        };
        self.hole_curr_raw_pairs[self.ingress_pair_index] = (raw_d0, raw_d1);
        self.hole_pending = Some(HoleFillPendingPair {
            inputs: TwoPpcInputs {
                datavalid: 1,
                d0: raw_d0 as u64,
                d1: raw_d1 as u64,
                sof: input.sof,
                eol: input.eol,
                ..Default::default()
            },
            raw_d0,
            raw_d1,
            feature_d0,
            feature_d1,
        });
        if expected_eol {
            self.hole_flush_pending = true;
        } else {
            self.ingress_pair_index += 1;
        }
        Ok(retired)
    }

    fn flush_hole_fill_row(&mut self) -> Result<Option<M::Feature>, String> {
        if self.hole_pending.is_none() {
            return Err("hole-fill row flush requires a pending final pair".into());
        }
        let retired = self.consume_pending_hole_fill_pair(0)?;
        self.hole_pending = None;
        self.hole_flush_pending = false;
        self.hole_prev_raw_pairs = self.hole_curr_raw_pairs.clone();
        self.hole_curr_raw_pairs.fill((0, 0));
        self.hole_prev_filled_pairs = self.hole_curr_filled_pairs.clone();
        self.hole_curr_filled_pairs.fill((0, 0));
        self.ingress_row += 1;
        self.ingress_pair_index = 0;
        if self.ingress_row == self.height {
            self.ingress_done = true;
        }
        Ok(retired)
    }

    fn consume_pending_hole_fill_pair(
        &mut self,
        right_raw: u8,
    ) -> Result<Option<M::Feature>, String> {
        let pending = self
            .hole_pending
            .ok_or_else(|| "hole-fill pending pair is not valid".to_owned())?;
        let pair = if pending.inputs.eol != 0 {
            self.pair_count() - 1
        } else {
            self.ingress_pair_index
                .checked_sub(1)
                .ok_or_else(|| "hole-fill pending pair index underflow".to_owned())?
        };
        let (top_raw_d0, top_raw_d1) = self.hole_prev_raw_pairs[pair];
        let left_raw = if pair > 0 {
            self.hole_curr_raw_pairs[pair - 1].1
        } else {
            0
        };
        let filled_d0 = u8::from(
            pending.raw_d0 != 0 || (top_raw_d0 != 0 && (left_raw != 0 || pending.raw_d1 != 0)),
        );
        let filled_d1 = u8::from(
            pending.raw_d1 != 0 || (top_raw_d1 != 0 && (pending.raw_d0 != 0 || right_raw != 0)),
        );
        self.hole_curr_filled_pairs[pair] = (filled_d0, filled_d1);
        self.consume_pair(
            TwoPpcInputs {
                datavalid: 1,
                d0: filled_d0 as u64,
                d1: filled_d1 as u64,
                sof: pending.inputs.sof,
                eol: pending.inputs.eol,
                ..Default::default()
            },
            Some((pending.feature_d0, pending.feature_d1)),
        )
    }

    fn start_row(&mut self) {
        self.curr_bank = self.row & 1;
        self.prev_bank = 1 - self.curr_bank;
        self.curr_row.fill(0);
        self.a = 0;
        self.c_pixel = 0;
        self.current_label = 0;
        self.fused = false;
        self.head = INVALID_LABEL;
        self.tail = INVALID_LABEL;
        self.current_feature = self.feature_zero();
        self.current_run_active = false;
        self.previous_issue = 0;
        self.previous = PToken {
            data: Some(self.feature_zero()),
            ..Default::default()
        };
        self.parent_head_taps = [None, None];
        self.last_output_lane = None;
    }

    fn finish_row(&mut self) -> Result<(), String> {
        if self.previous.valid != 0 {
            return Err("row ended with a live previous-row token".into());
        }
        if self.fused || self.current_run_active {
            return Err("row ended with an active current-row run".into());
        }
        self.prev_row.clone_from(&self.curr_row);
        self.row += 1;
        self.pair_index = 0;
        if self.row == self.height {
            self.done = true;
        } else {
            self.start_row();
        }
        Ok(())
    }

    fn mask_edge_pair(&self, d0: u64, d1: u64) -> (u8, u8) {
        self.mask_edge_pair_for(self.row, self.pair_index, d0, d1)
    }

    fn mask_edge_feature_pair(&self, d0: u64, d1: u64) -> (u64, u64) {
        self.mask_edge_feature_pair_for(self.row, self.pair_index, d0, d1)
    }

    fn mask_edge_pair_for(&self, row: usize, pair: usize, d0: u64, d1: u64) -> (u8, u8) {
        let (mut d0, mut d1) = (u8::from(d0 != 0), u8::from(d1 != 0));
        if row == 0 || row + 1 == self.height {
            return (0, 0);
        }
        if pair == 0 {
            d0 = 0;
        }
        if pair + 1 == self.pair_count() {
            d1 = 0;
        }
        (d0, d1)
    }

    fn mask_edge_feature_pair_for(
        &self,
        row: usize,
        pair: usize,
        mut d0: u64,
        mut d1: u64,
    ) -> (u64, u64) {
        if row == 0 || row + 1 == self.height {
            return (0, 0);
        }
        if pair == 0 {
            d0 = 0;
        }
        if pair + 1 == self.pair_count() {
            d1 = 0;
        }
        (d0, d1)
    }

    fn assert_event_invariants(events: TwoPpcEvents) -> Result<(), String> {
        for (left, right, name) in [
            (events.ec0, events.ec1, "Ec"),
            (events.ep0, events.ep1, "Ep"),
            (events.o0, events.o1, "O"),
            (events.sp0, events.sp1, "Sp"),
            (events.sc0, events.sc1, "Sc"),
        ] {
            if left != 0 && right != 0 {
                return Err(format!("{name}0 and {name}1 cannot both fire"));
            }
        }
        Ok(())
    }

    fn process_pair_events(
        &mut self,
        events: TwoPpcEvents,
        payload: PairPayload,
    ) -> Result<Option<M::Feature>, String> {
        let PairPayload {
            x,
            c_left,
            d0,
            d1,
            feature_d0,
            feature_d1,
        } = payload;
        let mut retired = None;
        if events.sp0 != 0 {
            self.load_previous_token()?;
        }
        if d0 != 0 {
            if events.o0 != 0 {
                self.touch(x, feature_d0)?;
            } else {
                self.add_pixel(x, feature_d0);
            }
        }
        let closed = if events.ec0 != 0 {
            Some(self.close_current()?)
        } else {
            None
        };
        self.record_parent_head_tap(0, c_left, closed)?;
        if events.ep0 != 0 {
            retired = self.retire_previous()?;
            if retired.is_some() {
                self.last_output_lane = Some(0);
            }
        }

        if events.sp1 != 0 {
            self.load_previous_token()?;
        }
        if d1 != 0 {
            if events.o1 != 0 {
                self.touch(x + 1, feature_d1)?;
            } else {
                self.add_pixel(x + 1, feature_d1);
            }
        }
        let closed = if events.ec1 != 0 {
            Some(self.close_current()?)
        } else {
            None
        };
        self.record_parent_head_tap(1, d0, closed)?;
        if events.ep1 != 0 {
            retired = self.retire_previous()?;
            if retired.is_some() {
                self.last_output_lane = Some(1);
            }
        }
        Ok(retired)
    }

    fn feature_zero(&self) -> M::Feature {
        self.feature_model.empty_feature(self.width, self.height)
    }

    fn pixel_feature(&self, x: usize, pixel: u64) -> M::Feature {
        self.feature_model
            .pixel_feature(x, self.row, pixel, self.width, self.height)
    }

    fn add_pixel(&mut self, x: usize, feature_pixel: u64) {
        self.current_run_active = true;
        self.current_feature = self.feature_model.merge_features(
            self.current_feature.clone(),
            self.pixel_feature(x, feature_pixel),
        );
    }

    fn check_bank(&self, bank: usize) -> Result<(), String> {
        if bank > 1 {
            Err(format!("bank out of range: {bank}"))
        } else {
            Ok(())
        }
    }

    fn check_label(&self, label: isize) -> Result<usize, String> {
        let label = usize::try_from(label).map_err(|_| format!("label out of range: {label}"))?;
        self.check_label_usize(label)?;
        Ok(label)
    }

    fn check_label_usize(&self, label: usize) -> Result<(), String> {
        if label >= self.label_count {
            Err(format!("label out of range: {label}"))
        } else {
            Ok(())
        }
    }

    fn data_feature(&self, bank: usize, head: usize) -> Result<M::Feature, String> {
        match &self.data_bank[bank][head] {
            DataWord::Feature(word) => Ok(word.feature.clone()),
            DataWord::HeadRef(_) => Err("feature lookup requires a concrete FEATURE word".into()),
        }
    }

    fn set_data_feature(
        &mut self,
        bank: usize,
        head: usize,
        feature: M::Feature,
    ) -> Result<(), String> {
        if matches!(self.data_bank[bank][head], DataWord::HeadRef(_)) {
            return Err("feature replacement requires a concrete FEATURE word".into());
        }
        self.data_bank[bank][head] = DataWord::Feature(FeatureWord { feature });
        Ok(())
    }

    fn feature_from_current_head(&self, head: usize) -> Result<M::Feature, String> {
        self.check_label_usize(head)?;
        if self.fused && self.head == head as isize {
            Ok(self.current_feature.clone())
        } else {
            self.data_feature(self.curr_bank, head)
        }
    }

    fn active_current_head(&self) -> Option<usize> {
        if !self.current_run_active {
            None
        } else if self.fused {
            usize::try_from(self.head).ok()
        } else {
            Some(self.current_label)
        }
    }

    fn record_parent_head_tap(
        &mut self,
        lane: usize,
        topology_pixel_under_close: u8,
        closed_head: Option<usize>,
    ) -> Result<(), String> {
        if topology_pixel_under_close == 0 {
            self.parent_head_taps[lane] = None;
            return Ok(());
        }
        let head = closed_head
            .or_else(|| self.active_current_head())
            .ok_or_else(|| "foreground parent tap requires a current-row head".to_owned())?;
        self.check_label_usize(head)?;
        self.parent_head_taps[lane] = Some((self.curr_bank, head));
        Ok(())
    }

    fn load_previous_token(&mut self) -> Result<(), String> {
        if self.previous.valid != 0 {
            return Err("cannot load a previous token while another is active".into());
        }
        if self.previous_issue >= self.label_count {
            return Err("previous-row run label budget exceeded".into());
        }
        let index = self.previous_issue;
        self.previous_issue += 1;
        let next = self.next_bank[self.prev_bank][index];
        let head_word = self.head_bank[self.prev_bank][index];
        self.check_label_usize(next)?;
        let head = self.check_label(head_word.head)?;
        self.previous = if head_word.mapped != 0 {
            PToken {
                valid: 1,
                index: index as isize,
                next: next as isize,
                has_next: u8::from(next != index),
                mapped: 1,
                head: head as isize,
                tail: self.tail_scratch[head] as isize,
                data: Some(self.feature_from_current_head(head)?),
            }
        } else {
            PToken {
                valid: 1,
                index: index as isize,
                next: next as isize,
                has_next: u8::from(next != index),
                mapped: 0,
                head: INVALID_LABEL,
                tail: INVALID_LABEL,
                data: Some(self.data_feature(self.prev_bank, index)?),
            }
        };
        Ok(())
    }

    fn touch(&mut self, x: usize, feature_pixel: u64) -> Result<(), String> {
        if self.previous.valid == 0 {
            return Err("overlap requires an active previous-row token".into());
        }
        self.check_label_usize(self.current_label)?;
        self.current_run_active = true;

        let old_mapped = self.previous.mapped != 0;
        let old_head = self.previous.head;
        let old_tail = self.previous.tail;
        let old_data = self
            .previous
            .data
            .clone()
            .unwrap_or_else(|| self.feature_zero());
        let old_fused = self.fused;
        let old_head_current = self.head;
        let old_tail_current = self.tail;
        let old_label = self.current_label;
        let need_previous_merge = !old_mapped || !old_fused || old_head != old_head_current;
        let mut new_feature = self.feature_model.merge_features(
            self.current_feature.clone(),
            self.pixel_feature(x, feature_pixel),
        );
        if need_previous_merge {
            new_feature = self.feature_model.merge_features(new_feature, old_data);
        }
        self.current_feature = new_feature.clone();

        match (old_mapped, old_fused, old_head == old_head_current) {
            (false, false, _) => {
                self.head = old_label as isize;
                self.tail = old_label as isize;
                self.fused = true;
                self.tail_scratch[old_label] = old_label;
                self.previous.mapped = 1;
                self.previous.head = old_label as isize;
                self.previous.tail = old_label as isize;
            }
            (false, true, _) => {
                self.previous.mapped = 1;
                self.previous.head = old_head_current;
                self.previous.tail = old_tail_current;
            }
            (true, false, _) => {
                let old_tail = self.check_label(old_tail)?;
                let old_head = self.check_label(old_head)?;
                self.next_bank[self.curr_bank][old_tail] = old_label;
                self.tail_scratch[old_head] = old_label;
                self.head = old_head as isize;
                self.tail = old_label as isize;
                self.fused = true;
                self.previous.tail = old_label as isize;
            }
            (true, true, true) => {
                self.previous.tail = old_tail_current;
            }
            (true, true, false) => {
                let old_head = self.check_label(old_head)?;
                let old_tail = self.check_label(old_tail)?;
                let active_head = self.check_label(old_head_current)?;
                if old_head >= active_head {
                    return Err(
                        "different-head merge expects incoming head before active head".into(),
                    );
                }
                self.next_bank[self.curr_bank][old_tail] = active_head;
                self.tail_scratch[old_head] = self.check_label(old_tail_current)?;
                self.head = old_head as isize;
                self.tail = old_tail_current;
                self.fused = true;
                self.previous.head = old_head as isize;
                self.previous.tail = old_tail_current;
            }
        }
        self.previous.data = Some(new_feature);
        Ok(())
    }

    fn close_current(&mut self) -> Result<usize, String> {
        self.check_label_usize(self.current_label)?;
        let address = if self.fused {
            self.check_label(self.head)?
        } else {
            self.current_label
        };
        self.data_bank[self.curr_bank][address] = DataWord::Feature(FeatureWord {
            feature: self.current_feature.clone(),
        });
        self.head_bank[self.curr_bank][self.current_label] = HeadWord {
            mapped: 0,
            head: self.current_label as isize,
        };
        if !self.fused || self.tail == self.current_label as isize {
            self.next_bank[self.curr_bank][self.current_label] = self.current_label;
        }
        if self.previous.valid != 0
            && self.previous.mapped != 0
            && self.previous.head == address as isize
        {
            self.previous.tail = if self.fused {
                self.tail
            } else {
                self.current_label as isize
            };
            self.previous.data = Some(self.current_feature.clone());
        }
        self.fused = false;
        self.head = INVALID_LABEL;
        self.tail = INVALID_LABEL;
        self.current_feature = self.feature_zero();
        self.current_run_active = false;
        self.current_label += 1;
        Ok(address)
    }

    fn retire_previous(&mut self) -> Result<Option<M::Feature>, String> {
        if self.previous.valid == 0 {
            return Err("previous-run retirement requires an active token".into());
        }
        let retired = if self.previous.mapped != 0 {
            if self.previous.has_next != 0 {
                let next = self.check_label(self.previous.next)?;
                let head = self.check_label(self.previous.head)?;
                self.head_bank[self.prev_bank][next] = HeadWord {
                    mapped: 1,
                    head: head as isize,
                };
            }
            None
        } else if self.previous.has_next != 0 {
            let next = self.check_label(self.previous.next)?;
            let data = self
                .previous
                .data
                .clone()
                .unwrap_or_else(|| self.feature_zero());
            self.data_bank[self.prev_bank][next] = DataWord::Feature(FeatureWord { feature: data });
            None
        } else {
            Some(
                self.previous
                    .data
                    .clone()
                    .unwrap_or_else(|| self.feature_zero()),
            )
        };
        self.previous = PToken {
            data: Some(self.feature_zero()),
            ..Default::default()
        };
        Ok(retired)
    }
}

pub fn simulate_2ppc_frame(
    model: &TwoPpcReferenceModel,
    image: &[Vec<u8>],
) -> Result<Vec<BBox>, String> {
    model.boxes(image)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_truth_vector() {
        let e = pair_events(0, 1, 0, 0, 1, 0);
        assert_eq!(
            e,
            TwoPpcEvents {
                ec0: 0,
                ec1: 1,
                ep0: 0,
                ep1: 1,
                o0: 1,
                o1: 0,
                sp0: 1,
                sp1: 0,
                sc0: 1,
                sc1: 0
            }
        );
    }

    #[test]
    fn masks_stream_edges_and_finds_components() {
        let image = vec![
            vec![1, 1, 1, 1, 1, 1],
            vec![1, 1, 0, 0, 1, 1],
            vec![1, 1, 1, 0, 1, 1],
            vec![1, 0, 0, 0, 0, 1],
        ];
        let model = TwoPpcReferenceModel::new(6, 4, false).unwrap();
        assert_eq!(
            crate::classical::sort_boxes(&model.boxes(&image).unwrap()),
            vec![(1, 2, 1, 2, 3), (4, 4, 1, 2, 2)]
        );
    }

    #[test]
    fn hole_fill_joins_diagonals() {
        let mut image = vec![vec![0; 8]; 6];
        image[1][1] = 1;
        image[2][2] = 1;
        let model = TwoPpcReferenceModel::new(8, 6, true).unwrap();
        assert_eq!(model.boxes(&image).unwrap(), vec![(1, 2, 1, 2, 2)]);
    }

    #[test]
    fn streaming_controls_are_checked_and_drained() {
        let image = vec![vec![0; 6]; 4];
        let mut model = TwoPpcReferenceModel::new(6, 4, false).unwrap();
        for input in iter_2ppc_pairs(&image).unwrap() {
            model.step(input, None).unwrap();
        }
        assert!(model.drain().is_empty());
    }
}
