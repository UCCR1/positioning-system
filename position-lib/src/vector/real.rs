use uom::{
    ConstZero,
    si::f32::{Angle, Area, Length, Ratio},
};

use super::Vector;

use core::{
    iter::Sum,
    ops::{Deref, DerefMut, Div, Mul, Sub},
};

/// A struct representing a Vector of length 1 (approximately).
///
/// UnitVectors cannot be instantiated directly, they are created by calling .normalized() on a regular Vector
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct UnitVector<const N: usize, T>(Vector<N, T>);

impl UnitVector<2, Ratio> {
    pub fn from_angle(angle: Angle) -> Self {
        Self(Vector([
            Ratio {
                value: libm::cosf(angle.value),
                ..Default::default()
            },
            Ratio {
                value: libm::sinf(angle.value),
                ..Default::default()
            },
        ]))
    }
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

    fn sqrt(self) -> Self::Root;
}

impl Root for f32 {
    type Root = f32;

    fn sqrt(self) -> Self::Root {
        libm::sqrtf(self)
    }
}

impl Root for Area {
    type Root = Length;

    fn sqrt(self) -> Self::Root {
        Length {
            value: libm::sqrtf(self.value),
            ..Default::default()
        }
    }
}

impl Root for Ratio {
    type Root = Ratio;

    fn sqrt(self) -> Self::Root {
        Ratio {
            value: libm::sqrtf(self.value),
            ..Default::default()
        }
    }
}

impl<const N: usize, T, S, R> Vector<N, T>
where
    T: Copy + Mul<T, Output = S> + Sub<Output = T> + Div<Output = R>,
    S: Sum + Root<Root = T>,
{
    pub fn length(self) -> T {
        self.dot(self).sqrt()
    }

    pub fn distance_to(self, target: Self) -> T {
        (target - self).length()
    }

    pub fn normalized(self) -> UnitVector<N, R> {
        UnitVector(self / self.length())
    }
}

impl Vector<2, Length> {
    pub fn bend(self, angle: Angle) -> Self {
        if angle == Angle::ZERO {
            return self;
        }

        Self::new(
            libm::sinf(angle.value) * self.x() / angle
                - (1.0 - libm::cosf(angle.value)) * self.y() / angle,
            libm::sinf(angle.value) * self.y() / angle
                + (1.0 - libm::cosf(angle.value)) * self.x() / angle,
        )
    }
}

#[macro_export]
macro_rules! real_vector {
    ($quantity:ident::$unit:ident, $($val:expr),*) => {
        paste::paste! {
            $crate::vector::Vector([$(uom::si::f32::$quantity::new::<uom::si::[<$quantity:lower>]::$unit>($val),)*])
        }
    };
}

#[cfg(test)]
mod tests {
    use core::f32::consts::FRAC_1_SQRT_2;

    use super::*;

    use approx::assert_relative_eq;
    use uom::si::{angle::degree, f32::Length, length::kilometer, ratio::ratio};

    #[test]
    fn normalize() {
        let a = Vector([1.0f32, 1.0]).normalized();

        assert_relative_eq!(a.length(), 1.0);

        assert_relative_eq!(a.x(), libm::sqrtf(2.0) / 2.0);
        assert_relative_eq!(a.y(), libm::sqrtf(2.0) / 2.0);
    }

    #[test]
    fn length() {
        let vector = real_vector!(Length::kilometer, 3.0, 4.0);

        assert_relative_eq!(vector.length().value, Length::new::<kilometer>(5.0).value);
    }

    #[test]
    fn from_angle() {
        let vector = UnitVector::from_angle(Angle::new::<degree>(135.0));

        assert_relative_eq!(vector.length().value, Ratio::new::<ratio>(1.0).value);
        assert_relative_eq!(vector.x().value, Ratio::new::<ratio>(-FRAC_1_SQRT_2).value);
        assert_relative_eq!(vector.y().value, Ratio::new::<ratio>(FRAC_1_SQRT_2).value);
    }
}
