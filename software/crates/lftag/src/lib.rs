//! Standalone LFTag3x3 decoding and localization reference.
//!
//! The crate intentionally implements the sparse, FPGA-oriented algorithm
//! used by this repository rather than reproducing the ordering of the
//! original LFTag software pipeline.  It consumes nine weighted centroids;
//! no source image or image-gradient data is required.

pub mod corner_select;
pub mod decode;
pub mod fixed;
pub mod geometry;
pub mod localize;
pub mod packet;
pub mod rom;

pub use decode::{
    Candidate, Correspondence, DecodeConfig, DecodeResult, Decoder, Plane, RejectReason,
    WeightedCentroid, WeightedPoint,
};
pub use geometry::{
    CanonicalPoint, DATA_CELL_CENTERS, EPSILON, Mat3, Point2, canonical_points, symbol_offset,
};
pub use localize::{
    CameraCalibration, Localization, LocalizationError, Pose, fit_homography, localize,
    localize_wavelet,
};
pub use packet::{
    FixedPose, POSE_PACKET_WORDS, POSE_ROTATION_WORD, POSE_TRANSLATION_WORD, PoseMathError,
    PosePacket, PosePacketError, derive_lambda_q10, pose_from_homography,
};
