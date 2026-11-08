//! Canonical LFTag3x3 geometry and small projective-geometry primitives.

use core::ops::{Add, Mul, Sub};

/// Encoded offset from a nominal grid center, in grid-pitch units.
pub const EPSILON: f64 = 1.0 / 12.0;

/// Data-cell centers in ascending ID-field order.
///
/// Entry `i` carries ID bits `2*i + 1 : 2*i`.
pub const DATA_CELL_CENTERS: [Point2; 7] = [
    Point2::new(0.0, -1.0),
    Point2::new(-1.0, 0.0),
    Point2::new(0.0, 0.0),
    Point2::new(1.0, 0.0),
    Point2::new(-1.0, 1.0),
    Point2::new(0.0, 1.0),
    Point2::new(1.0, 1.0),
];

/// One point in a two-dimensional Cartesian space.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

impl Point2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn dot(self, rhs: Self) -> f64 {
        self.x.mul_add(rhs.x, self.y * rhs.y)
    }

    pub fn cross(self, rhs: Self) -> f64 {
        self.x.mul_add(rhs.y, -self.y * rhs.x)
    }

    pub fn norm_squared(self) -> f64 {
        self.dot(self)
    }

    pub fn norm(self) -> f64 {
        self.norm_squared().sqrt()
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

impl Add for Point2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Point2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul<f64> for Point2 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

/// A row-major 3 by 3 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat3 {
    pub m: [[f64; 3]; 3],
}

impl Mat3 {
    pub const IDENTITY: Self = Self {
        m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    pub const fn new(m: [[f64; 3]; 3]) -> Self {
        Self { m }
    }

    pub fn transform_homogeneous(self, point: Point2) -> [f64; 3] {
        [
            self.m[0][0].mul_add(point.x, self.m[0][1].mul_add(point.y, self.m[0][2])),
            self.m[1][0].mul_add(point.x, self.m[1][1].mul_add(point.y, self.m[1][2])),
            self.m[2][0].mul_add(point.x, self.m[2][1].mul_add(point.y, self.m[2][2])),
        ]
    }

    pub fn transform_point(self, point: Point2) -> Option<Point2> {
        let h = self.transform_homogeneous(point);
        if !h[2].is_finite() || h[2].abs() <= 1.0e-14 {
            return None;
        }
        let result = Point2::new(h[0] / h[2], h[1] / h[2]);
        result.is_finite().then_some(result)
    }

    pub fn determinant(self) -> f64 {
        let m = self.m;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    pub fn inverse(self) -> Option<Self> {
        let m = self.m;
        let cof = [
            [
                m[1][1] * m[2][2] - m[1][2] * m[2][1],
                -(m[1][0] * m[2][2] - m[1][2] * m[2][0]),
                m[1][0] * m[2][1] - m[1][1] * m[2][0],
            ],
            [
                -(m[0][1] * m[2][2] - m[0][2] * m[2][1]),
                m[0][0] * m[2][2] - m[0][2] * m[2][0],
                -(m[0][0] * m[2][1] - m[0][1] * m[2][0]),
            ],
            [
                m[0][1] * m[1][2] - m[0][2] * m[1][1],
                -(m[0][0] * m[1][2] - m[0][2] * m[1][0]),
                m[0][0] * m[1][1] - m[0][1] * m[1][0],
            ],
        ];
        let det = m[0][0] * cof[0][0] + m[0][1] * cof[0][1] + m[0][2] * cof[0][2];
        if !det.is_finite() || det.abs() <= 1.0e-14 {
            return None;
        }
        let mut inverse = [[0.0; 3]; 3];
        for (row, values) in inverse.iter_mut().enumerate() {
            for (column, value) in values.iter_mut().enumerate() {
                *value = cof[column][row] / det;
            }
        }
        Some(Self::new(inverse))
    }

    pub fn scale(self, factor: f64) -> Self {
        let mut result = self;
        for row in &mut result.m {
            for value in row {
                *value *= factor;
            }
        }
        result
    }

    /// Normalize the matrix to `m[2][2] == 1` when possible.
    pub fn normalized(self) -> Option<Self> {
        let scale = self.m[2][2];
        (scale.is_finite() && scale.abs() > 1.0e-14).then(|| self.scale(1.0 / scale))
    }

    pub fn is_finite(self) -> bool {
        self.m.iter().flatten().all(|value| value.is_finite())
    }
}

impl Mul for Mat3 {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        let mut result = [[0.0; 3]; 3];
        for (row_index, row) in result.iter_mut().enumerate() {
            for (column_index, value) in row.iter_mut().enumerate() {
                *value = (0..3)
                    .map(|inner| self.m[row_index][inner] * rhs.m[inner][column_index])
                    .sum();
            }
        }
        Self::new(result)
    }
}

/// A canonical point and its optional two-bit payload symbol.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CanonicalPoint {
    pub position: Point2,
    pub symbol: Option<u8>,
}

/// Convert one two-bit symbol into its canonical displacement.
pub const fn symbol_offset(symbol: u8) -> Point2 {
    let x = if symbol & 0b10 != 0 {
        EPSILON
    } else {
        -EPSILON
    };
    let y = if symbol & 0b01 != 0 {
        EPSILON
    } else {
        -EPSILON
    };
    Point2::new(x, y)
}

/// Generate all nine LFTag3x3 centroids in canonical order.
///
/// The first two entries are the fixed top-left and top-right baselines.  The
/// remaining seven entries are in ascending two-bit ID-field order.
pub fn canonical_points(id: u16) -> [CanonicalPoint; 9] {
    assert!(id < (1 << 14), "LFTag3x3 IDs are fourteen bits");
    let mut points = [CanonicalPoint {
        position: Point2::new(0.0, 0.0),
        symbol: None,
    }; 9];
    points[0].position = Point2::new(-1.0, -1.0);
    points[1].position = Point2::new(1.0, -1.0);
    for (cell_index, center) in DATA_CELL_CENTERS.iter().copied().enumerate() {
        let symbol = ((id >> (2 * cell_index)) & 0b11) as u8;
        points[cell_index + 2] = CanonicalPoint {
            position: center + symbol_offset(symbol),
            symbol: Some(symbol),
        };
    }
    points
}

/// Closed-form homography from the unit square to an ordered quadrilateral.
///
/// The order is top-left, top-right, bottom-right, bottom-left.
pub fn square_to_quad(corners: [Point2; 4]) -> Option<Mat3> {
    if !corners.iter().copied().all(Point2::is_finite) {
        return None;
    }
    let [p0, p1, p2, p3] = corners;
    let dx1 = p1.x - p2.x;
    let dx2 = p3.x - p2.x;
    let dx3 = p0.x - p1.x + p2.x - p3.x;
    let dy1 = p1.y - p2.y;
    let dy2 = p3.y - p2.y;
    let dy3 = p0.y - p1.y + p2.y - p3.y;

    let (g, h) = if dx3.abs() <= 1.0e-14 && dy3.abs() <= 1.0e-14 {
        (0.0, 0.0)
    } else {
        let denominator = dx1 * dy2 - dx2 * dy1;
        if denominator.abs() <= 1.0e-14 || !denominator.is_finite() {
            return None;
        }
        (
            (dx3 * dy2 - dx2 * dy3) / denominator,
            (dx1 * dy3 - dx3 * dy1) / denominator,
        )
    };

    let result = Mat3::new([
        [p1.x - p0.x + g * p1.x, p3.x - p0.x + h * p3.x, p0.x],
        [p1.y - p0.y + g * p1.y, p3.y - p0.y + h * p3.y, p0.y],
        [g, h, 1.0],
    ]);
    (result.determinant().abs() > 1.0e-14 && result.is_finite()).then_some(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generator_uses_low_id_bits_first() {
        let points = canonical_points(0b10_01_11);
        assert_eq!(points[2].symbol, Some(0b11));
        assert_eq!(points[3].symbol, Some(0b01));
        assert_eq!(points[4].symbol, Some(0b10));
    }

    #[test]
    fn square_to_quad_hits_every_corner() {
        let quad = [
            Point2::new(10.0, 20.0),
            Point2::new(80.0, 18.0),
            Point2::new(70.0, 90.0),
            Point2::new(13.0, 75.0),
        ];
        let h = square_to_quad(quad).unwrap();
        let unit = [
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
        ];
        for (actual, expected) in unit
            .into_iter()
            .map(|p| h.transform_point(p).unwrap())
            .zip(quad)
        {
            assert!((actual.x - expected.x).abs() < 1.0e-10);
            assert!((actual.y - expected.y).abs() < 1.0e-10);
        }
    }

    #[test]
    fn inverse_round_trip() {
        let h = Mat3::new([[1.2, 0.1, 4.0], [-0.2, 0.9, 3.0], [0.003, -0.002, 1.0]]);
        let inverse = h.inverse().unwrap();
        let point = Point2::new(-0.7, 1.1);
        let mapped = h.transform_point(point).unwrap();
        let restored = inverse.transform_point(mapped).unwrap();
        assert!((restored.x - point.x).abs() < 1.0e-11);
        assert!((restored.y - point.y).abs() < 1.0e-11);
    }
}
