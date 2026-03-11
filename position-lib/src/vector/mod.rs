pub mod real;

use core::{
    iter::Sum,
    ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign},
};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vector<const N: usize, T>(pub [T; N]);

impl<const N: usize, T: Default> Default for Vector<N, T> {
    fn default() -> Self {
        Vector(core::array::from_fn(|_| T::default()))
    }
}

impl<const N: usize, T> Vector<N, T> {
    pub fn dot<O>(self, rhs: Self) -> O
    where
        T: Mul<T, Output = O>,
        O: Sum,
    {
        self.0.into_iter().zip(rhs.0).map(|(a, b)| a * b).sum()
    }

    pub fn product<Rhs, O>(self, rhs: Vector<N, Rhs>) -> Vector<N, O>
    where
        T: Mul<Rhs, Output = O> + Copy,
        Rhs: Copy,
        O: Default,
    {
        let mut output = Vector::<N, O>::default();

        for i in 0..N {
            output.0[i] = self.0[i] * rhs.0[i];
        }

        output
    }

    pub fn lerp<S>(self, target: Self, t: S) -> Self
    where
        T: Copy + Mul<S, Output = T> + Add<T, Output = T> + Sub<T, Output = T>,
        S: Copy,
    {
        self + target * t - self * t
    }

    pub fn project<S, O>(self, target: Self) -> Vector<N, T>
    where
        T: Copy + Mul<T, Output = O> + Mul<S, Output = T>,
        S: Copy,
        O: Div<O, Output = S> + Sum,
    {
        target * (target.dot(self) / target.dot(target))
    }

    pub fn closest_point<S, O>(self, start: Self, end: Self) -> Self
    where
        T: Copy + Mul<T, Output = O> + Mul<S, Output = T> + Sub<T, Output = T> + Add<T, Output = T>,
        S: Copy,
        O: Copy + Div<O, Output = S> + Sum + PartialOrd + Sub<O, Output = O>,
    {
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

impl<const N: usize, T> AddAssign for Vector<N, T>
where
    T: AddAssign + Copy,
{
    fn add_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.0[i] += rhs.0[i];
        }
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

impl<const N: usize, T> SubAssign for Vector<N, T>
where
    T: SubAssign + Copy,
{
    fn sub_assign(&mut self, rhs: Self) {
        for i in 0..N {
            self.0[i] -= rhs.0[i];
        }
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

impl<const N: usize, T, Rhs> MulAssign<Rhs> for Vector<N, T>
where
    T: MulAssign<Rhs>,
    Rhs: Copy,
{
    fn mul_assign(&mut self, rhs: Rhs) {
        for val in &mut self.0 {
            *val *= rhs;
        }
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
    pub const fn x(self) -> T {
        self.0[0]
    }

    pub const fn y(self) -> T {
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

impl<T> Vector<2, T> {
    pub const fn new(x: T, y: T) -> Self {
        Self([x, y])
    }
}

impl<T: Copy> Vector<3, T> {
    pub const fn x(self) -> T {
        self.0[0]
    }

    pub const fn y(self) -> T {
        self.0[1]
    }

    pub const fn z(self) -> T {
        self.0[2]
    }
}

impl<T> Vector<3, T> {
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self([x, y, z])
    }
}

impl<T> Vector<3, T> {
    pub fn cross<O>(self, rhs: Self) -> Vector<3, O>
    where
        T: Copy + Mul<T, Output = O>,
        O: Sub<O, Output = O>,
    {
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
    fn cross() {
        assert_eq!(
            Vector([1.0, 0.0, 0.0]).cross(Vector([0.0, 1.0, 0.0])),
            Vector([0.0, 0.0, 1.0])
        )
    }

    #[test]
    fn product() {
        assert_eq!(
            Vector([3.0, 2.0, 1.0]).product(Vector([1.0, 2.0, 3.0])),
            Vector([3.0, 4.0, 3.0])
        )
    }
}
