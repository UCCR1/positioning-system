use core::ops::{Add, Mul, Neg, Sub};

use num_traits::{One, Zero, real::Real};
use uom::si::{
    angle::radian,
    f32::{Angle, Ratio},
    ratio::ratio,
};

use crate::vector::{Vector, real::UnitVector};

pub type UnitQuaternion<T> = UnitVector<4, T>;
pub type Quaternion<T> = Vector<4, T>;

impl<T> UnitQuaternion<T> {
    pub fn identity() -> Self
    where
        T: Zero + One,
    {
        Self(Quaternion::from_array([
            T::one(),
            T::zero(),
            T::zero(),
            T::zero(),
        ]))
    }
}

impl<T: Copy> Quaternion<T> {
    pub fn hamilton_product<R: Copy, O>(self, rhs: Quaternion<R>) -> Quaternion<O>
    where
        T: Mul<R, Output = O>,
        O: Add<O, Output = O> + Sub<O, Output = O>,
    {
        Quaternion::from_array([
            self.w() * rhs.w() - self.x() * rhs.x() - self.y() * rhs.y() - self.z() * rhs.z(),
            self.w() * rhs.x() + self.x() * rhs.w() + self.y() * rhs.z() - self.z() * rhs.y(),
            self.w() * rhs.y() - self.x() * rhs.z() + self.y() * rhs.w() + self.z() * rhs.x(),
            self.w() * rhs.z() + self.x() * rhs.y() - self.y() * rhs.x() + self.z() * rhs.w(),
        ])
    }

    pub fn w(self) -> T {
        self[0][0]
    }

    pub fn x(self) -> T {
        self[1][0]
    }

    pub fn y(self) -> T {
        self[2][0]
    }

    pub fn z(self) -> T {
        self[3][0]
    }

    pub fn from_direction_component(scalar: T, direction: Vector<3, T>) -> Self {
        Self::from_array([scalar, direction.x(), direction.y(), direction.z()])
    }

    pub fn direction_components(self) -> Vector<3, T> {
        Vector::from_array([self.x(), self.y(), self.z()])
    }
}

impl<T: Copy + Neg<Output = T>> Quaternion<T> {
    pub fn conjugate(self) -> Self {
        Self::from_array([self.w(), -self.x(), -self.y(), -self.z()])
    }
}

impl Quaternion<Ratio> {
    pub fn roll(self) -> Angle {
        Angle::new::<radian>(
            (2.0 * (self.w() * self.x() + self.y() * self.z()))
                .get::<ratio>()
                .atan2(
                    (Ratio::one() - 2.0 * (self.x() * self.x() + self.y() * self.y()))
                        .get::<ratio>(),
                ),
        )
    }

    pub fn pitch(self) -> Angle {
        (2.0 * (self.w() * self.y() - self.z() * self.x())).asin()
    }

    pub fn yaw(self) -> Angle {
        Angle::new::<radian>(
            (2.0 * (self.w() * self.z() + self.x() * self.y()))
                .get::<ratio>()
                .atan2(
                    (Ratio::one() - 2.0 * (self.y() * self.y() + self.z() * self.z()))
                        .get::<ratio>(),
                ),
        )
    }
}
