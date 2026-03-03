use core::{
    iter::Sum,
    ops::{Add, Div, Mul, Neg, Sub},
};

use libm::{cosf, sinf, sqrtf};
use uom::{
    ConstZero,
    si::{
        angle::radian,
        area::square_meter,
        f32::{Angle, Length, Ratio},
        length::meter,
        ratio::ratio,
    },
};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vector<const N: usize, T>(pub [T; N]);

impl<const N: usize, T: Default> Default for Vector<N, T> {
    fn default() -> Self {
        Vector(core::array::from_fn(|_| T::default()))
    }
}

impl<const N: usize, T, O> Vector<N, T>
where
    T: Mul<T, Output = O>,
    O: Sum,
{
    pub fn dot(self, rhs: Self) -> O {
        self.0.into_iter().zip(rhs.0).map(|(a, b)| a * b).sum()
    }
}

pub trait Lengthable {
    type Output;
    fn length(self) -> Self::Output;
}

impl<const N: usize> Lengthable for Vector<N, Length> {
    type Output = Length;

    fn length(self) -> Self::Output {
        Length::new::<meter>(sqrtf(self.dot(self).get::<square_meter>()))
    }
}

impl<const N: usize> Lengthable for Vector<N, Ratio> {
    type Output = Ratio;

    fn length(self) -> Self::Output {
        Ratio::new::<ratio>(sqrtf(self.dot(self).get::<ratio>()))
    }
}

impl<const N: usize> Lengthable for Vector<N, f32> {
    type Output = f32;

    fn length(self) -> Self::Output {
        sqrtf(self.dot(self))
    }
}

impl<const N: usize, T> Vector<N, T>
where
    Self: Lengthable + Sub<Self, Output = Self>,
{
    pub fn distance_to(self, target: Self) -> <Self as Lengthable>::Output {
        (target - self).length()
    }
}

impl<const N: usize, T, O> Vector<N, T>
where
    Self: Copy + Lengthable<Output = T> + Div<T, Output = Vector<N, O>>,
{
    pub fn normalized(self) -> Vector<N, O> {
        self / self.length()
    }
}

impl<const N: usize, T> Vector<N, T> {
    pub fn lerp<S>(self, target: Self, t: S) -> Self
    where
        T: Copy + Mul<S, Output = T> + Add<T, Output = T> + Sub<T, Output = T>,
        S: Copy,
    {
        self + target * t - self * t
    }
}

impl<const N: usize, T, S, O> Vector<N, T>
where
    T: Copy + Mul<T, Output = O> + Mul<S, Output = T>,
    S: Copy,
    O: Div<O, Output = S> + Sum,
{
    pub fn project(self, target: Self) -> Vector<N, T> {
        target * (target.dot(self) / target.dot(target))
    }
}

impl<const N: usize, T, S, O> Vector<N, T>
where
    T: Copy + Mul<T, Output = O> + Mul<S, Output = T> + Sub<T, Output = T> + Add<T, Output = T>,
    S: Copy,
    O: Copy + Div<O, Output = S> + Sum + PartialOrd + Sub<O, Output = O>,
{
    pub fn closest_point(self, start: Self, end: Self) -> Self {
        let v = end - start;
        let d = self - start;

        let cosine = v.dot(d);
        let mag_square = v.dot(v);

        // Crude Check for negativity
        if cosine <= (cosine - cosine) {
            return start;
        } else if cosine >= mag_square {
            return end;
        }

        start + v * (cosine / mag_square)
    }
}

impl<const N: usize, T> Add<Vector<N, T>> for Vector<N, T>
where
    T: Add<T, Output = T> + Copy,
{
    type Output = Vector<N, T>;

    fn add(mut self, rhs: Vector<N, T>) -> Self::Output {
        for i in 0..N {
            self.0[i] = self.0[i] + rhs.0[i];
        }

        self
    }
}

impl<const N: usize, T> Sub<Vector<N, T>> for Vector<N, T>
where
    T: Sub<T, Output = T> + Copy,
{
    type Output = Vector<N, T>;

    fn sub(mut self, rhs: Vector<N, T>) -> Self::Output {
        for i in 0..N {
            self.0[i] = self.0[i] - rhs.0[i];
        }

        self
    }
}

impl<const N: usize, T, Rhs, O> Mul<Rhs> for Vector<N, T>
where
    T: Mul<Rhs, Output = O>,
    Rhs: Copy,
{
    type Output = Vector<N, O>;

    fn mul(self, rhs: Rhs) -> Self::Output {
        Vector(self.0.map(|x| x * rhs))
    }
}

impl<const N: usize, T, Rhs, O> Div<Rhs> for Vector<N, T>
where
    T: Div<Rhs, Output = O>,
    Rhs: Copy,
{
    type Output = Vector<N, O>;

    fn div(self, rhs: Rhs) -> Self::Output {
        Vector(self.0.map(|x| x / rhs))
    }
}

impl<const N: usize, T> Neg for Vector<N, T>
where
    T: Neg<Output = T> + Copy,
{
    type Output = Vector<N, T>;

    fn neg(mut self) -> Self::Output {
        for val in &mut self.0 {
            *val = val.neg();
        }

        self
    }
}

impl<T: Copy> Vector<2, T> {
    pub fn x(self) -> T {
        self.0[0]
    }

    pub fn y(self) -> T {
        self.0[1]
    }
}

impl<T> Vector<2, T>
where
    T: Neg<Output = T>,
{
    pub fn perp(self) -> Self {
        let [x, y] = self.0;

        Self([-y, x])
    }
}

impl Vector<2, Length> {
    pub fn bend(self, angle: Angle) -> Self {
        if angle == Angle::ZERO {
            return self;
        }

        let rads = angle.get::<radian>();

        Self::new(
            sinf(rads) * self.x() / rads - (1.0 - cosf(rads)) * self.y() / rads,
            sinf(rads) * self.y() / rads + (1.0 - cosf(rads)) * self.x() / rads,
        )
    }
}

impl<T> Vector<2, T> {
    pub fn new(x: T, y: T) -> Self {
        Self([x, y])
    }
}

impl<T: Copy> Vector<3, T> {
    pub fn x(self) -> T {
        self.0[0]
    }

    pub fn y(self) -> T {
        self.0[1]
    }

    pub fn z(self) -> T {
        self.0[2]
    }
}

impl<T> Vector<3, T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self([x, y, z])
    }
}

impl<T, O> Vector<3, T>
where
    T: Copy + Mul<T, Output = O>,
    O: Sub<O, Output = O>,
{
    pub fn cross(self, rhs: Self) -> Vector<3, O> {
        Vector::<3, O>::new(
            self.y() * rhs.z() - rhs.y() * self.z(),
            self.z() * rhs.x() - rhs.z() * self.x(),
            self.x() * rhs.y() - rhs.x() * self.y(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add() {
        assert_eq!(
            Vector([1.0, 2.0, 3.0]) + Vector([3.0, 2.0, 1.0]),
            Vector([4.0, 4.0, 4.0])
        );
    }

    #[test]
    fn sub() {
        assert_eq!(
            Vector([1.0, 2.0, 3.0, 4.0, 5.0]) - Vector([1.0, 2.0, 3.0, 4.0, 5.0]),
            Vector::default()
        );
    }

    #[test]
    fn dot() {
        assert_eq!(Vector([1.0, 2.0]).dot(Vector([1.0, 2.0])), 5.0);
    }

    #[test]
    fn length() {
        assert_eq!(Vector([3.0, 4.0]).length(), 5.0);
    }

    #[test]
    fn cross() {
        assert_eq!(
            Vector([1.0, 0.0, 0.0]).cross(Vector([0.0, 1.0, 0.0])),
            Vector([0.0, 0.0, 1.0])
        )
    }
}
