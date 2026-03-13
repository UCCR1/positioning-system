pub mod real;

use core::{
    iter::Sum,
    ops::{Div, Mul, Neg, Sub},
};

use super::matrix::Matrix;

pub type Vector<const M: usize, T> = Matrix<M, 1, T>;

impl<const N: usize, T> IntoIterator for Vector<N, T> {
    type Item = T;
    type IntoIter = core::array::IntoIter<T, N>;

    fn into_iter(self) -> Self::IntoIter {
        self.0.map(|[x]| x).into_iter()
    }
}

impl<const N: usize, T> From<[T; N]> for Vector<N, T> {
    fn from(value: [T; N]) -> Self {
        Self(value.map(|x| [x]))
    }
}

impl<const N: usize, T> Vector<N, T> {
    pub fn to_array(self) -> [T; N] {
        self.0.map(|[x]| x)
    }

    pub fn dot<O>(self, rhs: Self) -> O
    where
        T: Mul<T, Output = O> + Default + Copy,
        O: Sum,
    {
        self.into_iter().zip(rhs).map(|(a, b)| a * b).sum()
    }

    pub fn project<S, O>(self, target: Self) -> Vector<N, T>
    where
        T: Copy + Mul<T, Output = O> + Mul<S, Output = T> + Default,
        S: Copy,
        O: Div<O, Output = S> + Sum + Default + PartialEq,
    {
        let denom = target.dot(target);

        // Crude zero check
        if denom == Default::default() {
            return Default::default();
        }

        target * (target.dot(self) / denom)
    }
}

impl<T: Copy> Vector<2, T> {
    pub const fn x(self) -> T {
        self.0[0][0]
    }

    pub const fn y(self) -> T {
        self.0[1][0]
    }

    pub fn perp(self) -> Self
    where
        T: Neg<Output = T>,
    {
        let [[x], [y]] = self.0;

        Self([[-y], [x]])
    }
}

impl<T: Copy> Vector<3, T> {
    pub const fn x(self) -> T {
        self.0[0][0]
    }

    pub const fn y(self) -> T {
        self.0[1][0]
    }

    pub const fn z(self) -> T {
        self.0[2][0]
    }
}

impl<T> Vector<2, T> {
    pub fn cross<O>(self, rhs: Self) -> O
    where
        T: Copy + Mul<T, Output = O>,
        O: Sub<O, Output = O>,
    {
        self.x() * rhs.y() - self.y() * rhs.x()
    }
}

impl<T> Vector<3, T> {
    pub fn cross<O>(self, rhs: Self) -> Vector<3, O>
    where
        T: Copy + Mul<T, Output = O>,
        O: Sub<O, Output = O>,
    {
        [
            self.y() * rhs.z() - rhs.y() * self.z(),
            self.z() * rhs.x() - rhs.z() * self.x(),
            self.x() * rhs.y() - rhs.x() * self.y(),
        ]
        .into()
    }
}

#[macro_export]
macro_rules! vector {
    [$($value:expr),*] => {
        $crate::vector::Vector::from([$($value,)*])
    };
}

#[cfg(test)]
mod tests {
    #[test]
    fn dot() {
        assert_eq!(vector![1.0, 2.0].dot(vector![1.0, 2.0]), 5.0);
    }

    #[test]
    fn cross() {
        assert_eq!(
            vector![1.0, 0.0, 0.0].cross(vector![0.0, 1.0, 0.0]),
            vector![0.0, 0.0, 1.0]
        )
    }
}
