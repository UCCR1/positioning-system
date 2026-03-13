use core::array;

use crate::{line::Line, vector::Vector};

#[derive(Debug, Copy, Clone)]
pub struct Bounds<const N: usize, T>([(T, T); N]);

#[derive(Debug, Copy, Clone)]
pub struct BoundsHaveNoVolume;

impl<const N: usize, T: Copy + PartialEq> TryFrom<Line<N, T>> for Bounds<N, T> {
    type Error = BoundsHaveNoVolume;

    fn try_from(value: Line<N, T>) -> Result<Self, Self::Error> {
        let Line(start, end) = value;

        for i in 0..N {
            if start[i][0] == end[i][0] {
                return Err(BoundsHaveNoVolume);
            }
        }

        Ok(Self(array::from_fn(|i| (start[i][0], end[i][0]))))
    }
}

impl<const N: usize, T: Copy> Bounds<N, T> {
    pub fn new(bounds: [(T, T); N]) -> Result<Self, BoundsHaveNoVolume>
    where
        T: PartialEq,
    {
        for (a, b) in bounds {
            if a == b {
                return Err(BoundsHaveNoVolume);
            }
        }

        Ok(Self(bounds))
    }

    pub fn contains_point(self, point: Vector<N, T>) -> bool
    where
        T: PartialOrd,
    {
        for i in 0..N {
            let (a, b) = self.0[i];

            if a < b {
                if a > point.0[i][0] || b < point.0[i][0] {
                    return false;
                }
            } else if b < a {
                if b > point.0[i][0] || a < point.0[i][0] {
                    return false;
                }
            }
        }

        return true;
    }
}
