use core::{
    iter::Sum,
    ops::{Deref, DerefMut, Div, Mul, Sub},
};

use num_traits::{ConstOne, ConstZero};
use uom::si::f32::{Angle, Area, Length, Ratio};

use super::Vector;
use crate::vector;

/// A struct representing a Vector of length 1 (approximately).
///
/// UnitVectors cannot be instantiated directly, they are created by calling
/// .normalized() on a regular Vector
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct UnitVector<const N: usize, T>(Vector<N, T>);

impl UnitVector<2, Ratio> {
    pub fn from_angle(angle: Angle) -> Self {
        Self(vector![angle.cos(), angle.sin()])
    }
}

impl<T: ConstOne + ConstZero> UnitVector<2, T> {
    pub const RIGHT: Self = Self(vector![T::ONE, T::ZERO]);

    pub const UP: Self = Self(vector![T::ZERO, T::ONE]);
}

impl<T: ConstOne + ConstZero> UnitVector<3, T> {
    pub const RIGHT: Self = Self(vector![T::ONE, T::ZERO, T::ZERO]);

    pub const UP: Self = Self(vector![T::ZERO, T::ONE, T::ZERO]);

    pub const FORWARD: Self = Self(vector![T::ZERO, T::ZERO, T::ONE]);
}

impl<const N: usize, T> Deref for UnitVector<N, T> {
    type Target = Vector<N, T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize, T> DerefMut for UnitVector<N, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub trait Root {
    type Root;

    fn root(self) -> Self::Root;
}

impl Root for f32 {
    type Root = f32;

    fn root(self) -> Self::Root {
        self.sqrt()
    }
}

impl Root for f64 {
    type Root = f64;

    fn root(self) -> Self::Root {
        self.sqrt()
    }
}

impl Root for Area {
    type Root = Length;

    fn root(self) -> Self::Root {
        self.sqrt()
    }
}

impl Root for Ratio {
    type Root = Ratio;

    fn root(self) -> Self::Root {
        self.sqrt()
    }
}

impl<const N: usize, T: Copy, S, R> Vector<N, T>
where
    Self: Sub<Self, Output = Self>,
    T: Mul<T, Output = S> + Div<Output = R>,
    S: Sum + Root<Root = T>,
{
    pub fn magnitude(self) -> T {
        self.dot(self).root()
    }

    pub fn normalized(self) -> UnitVector<N, R> {
        UnitVector(self / self.magnitude())
    }
}

impl Vector<2, Length> {
    pub fn bend(self, angle: Angle) -> Self {
        if angle == Angle::ZERO {
            return self;
        }

        vector![
            angle.sin() * self.x() / angle - (Ratio::ONE - angle.cos()) * self.y() / angle,
            angle.sin() * self.y() / angle + (Ratio::ONE - angle.cos()) * self.x() / angle
        ]
    }
}

#[macro_export]
macro_rules! real_vector {
    ($quantity:ident::$unit:ident, $($val:expr),*) => {
        $crate::paste::paste! {
            $crate::vector![$(uom::si::f32::$quantity::new::<uom::si::[<$quantity:lower>]::$unit>($val)),*]
        }
    };
}

#[cfg(test)]
mod tests {
    use core::f32::consts::FRAC_1_SQRT_2;

    use approx::assert_relative_eq;
    use uom::si::{angle::degree, f32::Length, length::kilometer, ratio::ratio};

    use super::*;

    #[test]
    fn normalize() {
        let a = vector![1.0f32, 1.0].normalized();

        assert_relative_eq!(a.magnitude(), 1.0);

        assert_relative_eq!(a.x(), FRAC_1_SQRT_2);
        assert_relative_eq!(a.y(), FRAC_1_SQRT_2);
    }

    #[test]
    fn length() {
        let vector = real_vector!(Length::kilometer, 3.0, 4.0);

        assert_relative_eq!(
            vector.magnitude().value,
            Length::new::<kilometer>(5.0).value
        );
    }

    #[test]
    fn from_angle() {
        let vector = UnitVector::from_angle(Angle::new::<degree>(135.0));

        assert_relative_eq!(vector.magnitude().value, Ratio::new::<ratio>(1.0).value);
        assert_relative_eq!(vector.x().value, Ratio::new::<ratio>(-FRAC_1_SQRT_2).value);
        assert_relative_eq!(vector.y().value, Ratio::new::<ratio>(FRAC_1_SQRT_2).value);
    }
}
