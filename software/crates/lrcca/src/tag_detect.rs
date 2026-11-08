//! Public centroid records shared by TagDetect and LFTag.

use crate::hierarchical::InnerCentroidFeature;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct TagDetectCentroid {
    pub centroid_valid: bool,
    pub x_q5: u32,
    pub y_q5: u32,
}
impl TagDetectCentroid {
    pub fn from_inner_feature(feature: InnerCentroidFeature) -> Self {
        Self {
            centroid_valid: feature.centroid_valid,
            x_q5: feature.x_q5,
            y_q5: feature.y_q5,
        }
    }
    pub fn as_tuple(self) -> (bool, u32, u32) {
        (self.centroid_valid, self.x_q5, self.y_q5)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct TagDetectOutput {
    pub centroid: Option<TagDetectCentroid>,
    pub end_inner: bool,
}

/// Native-precision descriptor for one centroid in a hierarchical output group.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct CentroidGroupDescriptor {
    pub centroid_valid: bool,
    pub x_q4: u32,
    pub y_q4: u32,
    pub log_m_q5_3: u8,
}
impl CentroidGroupDescriptor {
    pub fn from_inner_feature(feature: InnerCentroidFeature) -> Self {
        debug_assert_eq!(feature.x_q5 & 1, 0);
        debug_assert_eq!(feature.y_q5 & 1, 0);
        Self {
            centroid_valid: feature.centroid_valid,
            x_q4: feature.x_q5 >> 1,
            y_q4: feature.y_q5 >> 1,
            log_m_q5_3: feature.log_m_q5_3,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct CentroidGroupOutput {
    pub descriptor: Option<CentroidGroupDescriptor>,
    pub group_end: bool,
}
