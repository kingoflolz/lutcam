# lrcca_util_pkg

`lrcca_util_pkg` contains elaboration-time geometry and width calculations
shared by the TagDetect hierarchy. Keeping these expressions in one package
ensures the CCA stages, moment accumulator, centroid compressor, and public
wrappers derive compatible packed-field widths.

`bit_width` returns at least one bit and otherwise computes the bits needed to
represent values below its argument. `min_int` and `max_int` keep geometry
expressions constant and tool-portable. `area_width` derives an exact unsigned
component-area width from image dimensions.

The remaining helpers bound intensity moments without relying on host integer
widths. `moment_zom_width`, `moment_x_width`, and `moment_y_width` cover
uncentered mass and first moments. `centered_moment_bits` sums the absolute
twice-centered coordinate range and adds a sign bit; it is used by the inner
CCA and centroid path.

These functions create hardware widths during elaboration; they do not infer a
runtime arithmetic block. Changing one affects packed interfaces and RAM word
shapes throughout the closure, so width changes require rebuilding all
TagDetect RTL parity tests rather than testing one consumer in isolation.
