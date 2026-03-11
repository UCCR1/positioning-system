use super::Vector;

use core::ops::{Deref, DerefMut};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct UnitVector<const N: usize, T>(Vector<N, T>);

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

macro_rules! impl_real {
    ($type:path, $unit:path, |$self:ident| $length:expr) => {
        impl<const N: usize> UnitVector<N, $type> {
            pub fn new(values: [$type; N]) -> UnitVector<N, $unit> {
                let inner = Vector(values);

                UnitVector(inner.normalized())
            }
        }

        impl<const N: usize> Vector<N, $type> {
            pub fn length(self) -> $type {
                let $self = self;

                $length
            }

            pub fn distance_to(self, target: Self) -> $type {
                (target - self).length()
            }

            pub fn normalized(self) -> Vector<N, $unit> {
                self / self.length()
            }
        }
    };
}

impl_real!(f32, f32, |v| libm::sqrtf(v.dot(v)));
impl_real!(f64, f64, |v| libm::sqrt(v.dot(v)));

impl_real!(uom::si::f32::Length, uom::si::f32::Ratio, |v| {
    uom::si::f32::Length::new::<uom::si::length::meter>(libm::sqrtf(v.dot(v).value))
});

impl_real!(uom::si::f32::Ratio, uom::si::f32::Ratio, |v| {
    uom::si::f32::Ratio::new::<uom::si::ratio::ratio>(libm::sqrtf(v.dot(v).value))
});

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
    use super::*;

    use approx::assert_relative_eq;
    use uom::si::{f32::Length, length::kilometer};

    #[test]
    fn normalize() {
        let a = UnitVector::<2, f32>::new([1.0, 1.0]);

        assert_relative_eq!(a.length(), 1.0);

        assert_relative_eq!(a.x(), libm::sqrtf(2.0) / 2.0);
        assert_relative_eq!(a.y(), libm::sqrtf(2.0) / 2.0);
    }

    #[test]
    fn length() {
        let vector = real_vector!(Length::kilometer, 3.0, 4.0);

        assert_relative_eq!(vector.length().value, Length::new::<kilometer>(5.0).value);
    }
}
