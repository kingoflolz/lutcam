//! Integer-exact reference for the production streaming corner selector.

use crate::WeightedCentroid;

/// Result type reported by the compact preflight.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum CornerSelectStatus {
    Valid = 0,
    InvalidInput = 1,
    BaselineTooShort = 2,
    DominantSide = 3,
    CornerSelection = 4,
    InvalidQuadrilateral = 5,
}

/// Per-stage validity bits retained even when a later check rejects the group.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CornerSelectValidity {
    pub input: bool,
    pub baseline: bool,
    pub dominant_side: bool,
    pub corners: bool,
    pub quadrilateral: bool,
}

impl CornerSelectValidity {
    /// Packed in the same order as the RTL output.
    pub const fn bits(self) -> u8 {
        (self.input as u8)
            | ((self.baseline as u8) << 1)
            | ((self.dominant_side as u8) << 2)
            | ((self.corners as u8) << 3)
            | ((self.quadrilateral as u8) << 4)
    }
}

/// Ordered key corners and the five non-key input indices.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CornerSelection {
    pub status: CornerSelectStatus,
    pub validity: CornerSelectValidity,
    /// Top-left, top-right, bottom-right, bottom-left input indices.
    pub corners: [u8; 4],
    /// Ascending input order, excluding the four key corners.
    pub remaining: [u8; 5],
}

impl CornerSelection {
    fn rejected(status: CornerSelectStatus, validity: CornerSelectValidity) -> Self {
        Self {
            status,
            validity,
            corners: [0; 4],
            remaining: [0; 5],
        }
    }
}

fn cross(origin: (i32, i32), baseline: (i32, i32), point: (i32, i32)) -> i64 {
    let dx = i64::from(point.0 - origin.0);
    let dy = i64::from(point.1 - origin.1);
    i64::from(baseline.0) * dy - i64::from(baseline.1) * dx
}

fn squared_distance(left: (i32, i32), right: (i32, i32)) -> i64 {
    let dx = i64::from(right.0 - left.0);
    let dy = i64::from(right.1 - left.1);
    dx * dx + dy * dy
}

/// Production streaming-corner reference.
///
/// ChildArena presents the retained nine descriptors in ascending sort-key
/// order, with the two distinct-mass anchors in slots 7 and 8.  The hardware
/// input envelope is unsigned 15-bit Q4 X and unsigned 14-bit Q4 Y.  After
/// orienting the anchor baseline, the three greatest signed depths identify
/// the tag's bottom row; its minimum and maximum baseline projections are the
/// bottom corners.  This bounded-pose construction avoids retaining angular
/// vectors while leaving descriptor coordinates in the candidate FIFO.
pub fn select_sorted_streaming_corners(
    points: [WeightedCentroid; 9],
    minimum_baseline_q4: u16,
) -> CornerSelection {
    let mut validity = CornerSelectValidity::default();
    if points.iter().any(|point| {
        !point.valid
            || !(0..(1 << 15)).contains(&point.x_q4)
            || !(0..(1 << 14)).contains(&point.y_q4)
    }) {
        return CornerSelection::rejected(CornerSelectStatus::InvalidInput, validity);
    }
    validity.input = true;
    let coordinates = points.map(|point| (point.x_q4, point.y_q4));
    let mut baseline_a = 8;
    let mut baseline_b = 7;
    let mut baseline = (
        coordinates[baseline_b].0 - coordinates[baseline_a].0,
        coordinates[baseline_b].1 - coordinates[baseline_a].1,
    );
    let baseline_squared = squared_distance(coordinates[baseline_a], coordinates[baseline_b]);
    let minimum_squared = i64::from(minimum_baseline_q4) * i64::from(minimum_baseline_q4);
    if baseline_squared < minimum_squared {
        return CornerSelection::rejected(CornerSelectStatus::BaselineTooShort, validity);
    }
    validity.baseline = true;
    if points[6].log_mass_q5_3 >= points[7].log_mass_q5_3 {
        return CornerSelection::rejected(CornerSelectStatus::CornerSelection, validity);
    }

    let mut positive = 0;
    let mut negative = 0;
    for (index, &point) in coordinates.iter().enumerate() {
        if index == baseline_a || index == baseline_b {
            continue;
        }
        let value = cross(coordinates[baseline_a], baseline, point);
        positive += usize::from(value > 0);
        negative += usize::from(value < 0);
    }
    if positive < 6 && negative < 6 {
        return CornerSelection::rejected(CornerSelectStatus::DominantSide, validity);
    }
    if negative > positive {
        core::mem::swap(&mut baseline_a, &mut baseline_b);
        baseline = (-baseline.0, -baseline.1);
    }
    validity.dominant_side = true;

    let mut deepest = [usize::MAX; 3];
    for slot in 0..deepest.len() {
        let mut best = None;
        let mut best_depth = 0i64;
        for (index, &point) in coordinates[..7].iter().enumerate() {
            if deepest[..slot].contains(&index) {
                continue;
            }
            let depth = cross(coordinates[baseline_a], baseline, point);
            if depth > 0 && (best.is_none() || depth > best_depth) {
                best = Some(index);
                best_depth = depth;
            }
        }
        let Some(best) = best else {
            return CornerSelection::rejected(CornerSelectStatus::CornerSelection, validity);
        };
        deepest[slot] = best;
    }

    let projection = |index: usize| {
        let delta = (
            i64::from(coordinates[index].0 - coordinates[baseline_a].0),
            i64::from(coordinates[index].1 - coordinates[baseline_a].1),
        );
        i64::from(baseline.0) * delta.0 + i64::from(baseline.1) * delta.1
    };
    let mut bottom_left = deepest[0];
    let mut bottom_right = deepest[0];
    let mut minimum_projection = projection(deepest[0]);
    let mut maximum_projection = minimum_projection;
    for &index in &deepest[1..] {
        let value = projection(index);
        if value < minimum_projection {
            minimum_projection = value;
            bottom_left = index;
        }
        if value > maximum_projection {
            maximum_projection = value;
            bottom_right = index;
        }
    }
    if bottom_left == bottom_right {
        return CornerSelection::rejected(CornerSelectStatus::CornerSelection, validity);
    }
    validity.corners = true;
    if cross(
        coordinates[baseline_b],
        (
            coordinates[bottom_right].0 - coordinates[baseline_b].0,
            coordinates[bottom_right].1 - coordinates[baseline_b].1,
        ),
        coordinates[bottom_left],
    ) <= 0
    {
        return CornerSelection::rejected(CornerSelectStatus::InvalidQuadrilateral, validity);
    }
    validity.quadrilateral = true;

    let corners = [baseline_a, baseline_b, bottom_right, bottom_left];
    let remaining: [u8; 5] = (0..9)
        .filter(|index| !corners.contains(index))
        .map(|index| index as u8)
        .collect::<Vec<_>>()
        .try_into()
        .expect("four distinct corners leave five points");
    CornerSelection {
        status: CornerSelectStatus::Valid,
        validity,
        corners: corners.map(|index| index as u8),
        remaining,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fixed::rtl_smoke_candidate;

    #[test]
    fn sorted_streaming_contract_matches_the_ideal_geometry() {
        let mut centroids = rtl_smoke_candidate(0x1234).centroids;
        centroids.sort_by_key(|point| point.log_mass_q5_3);
        let selection = select_sorted_streaming_corners(centroids, 16);
        assert_eq!(selection.status, CornerSelectStatus::Valid);
        assert_eq!(selection.validity.bits(), 0x1f);
    }

    #[test]
    fn streaming_contract_enforces_the_active_plane_coordinate_widths() {
        let mut centroids = rtl_smoke_candidate(0x1234).centroids;
        centroids.sort_by_key(|point| point.log_mass_q5_3);

        let maximum_x = centroids.iter().map(|point| point.x_q4).max().unwrap();
        let maximum_y = centroids.iter().map(|point| point.y_q4).max().unwrap();
        for point in &mut centroids {
            point.x_q4 += (1 << 15) - 1 - maximum_x;
            point.y_q4 += (1 << 14) - 1 - maximum_y;
        }
        assert_eq!(
            select_sorted_streaming_corners(centroids, 16).status,
            CornerSelectStatus::Valid
        );

        for (index, x_q4, y_q4) in [
            (8, -1, centroids[8].y_q4),
            (8, 1 << 15, centroids[8].y_q4),
            (7, centroids[7].x_q4, -1),
            (7, centroids[7].x_q4, 1 << 14),
        ] {
            let mut outside = centroids;
            outside[index].x_q4 = x_q4;
            outside[index].y_q4 = y_q4;
            assert_eq!(
                select_sorted_streaming_corners(outside, 16).status,
                CornerSelectStatus::InvalidInput
            );
        }
    }
}
