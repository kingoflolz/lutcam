//! Software reference models for the LRCCA connected-component and TagDetect
//! gateware.
//!
//! Images are represented as row-major `Vec<Vec<u8>>` values.  The streaming
//! models use integer-valued valid/control fields to mirror their RTL ports.

pub mod adaptive_threshold;
pub mod centroid_compression;
pub mod child_arena;
pub mod classical;
pub mod hierarchical;
pub mod tag_detect;
pub mod two_ppc;
pub mod wavelet_tag_detect;

pub use adaptive_threshold::*;
pub use centroid_compression::*;
pub use child_arena::*;
pub use classical::*;
pub use hierarchical::*;
pub use tag_detect::*;
pub use two_ppc::*;
pub use wavelet_tag_detect::*;
