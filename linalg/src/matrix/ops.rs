use core::{
    array,
    iter::Sum,
    ops::{Add, AddAssign, Deref, Div, Index, IndexMut, Mul, Neg, Sub, SubAssign},
};

use num_traits::Zero;

use super::Matrix;

impl<const M: usize, const N: usize, T: Copy> AddAssign for Matrix<M, N, T>
where
    T: AddAssign,
{
    fn add_assign(&mut self, rhs: Self) {
        for i in 0..M {
            for j in 0..N {
                self.0[i][j] += rhs.0[i][j];
            }
        }
    }
}

impl<const M: usize, const N: usize, T: Copy> SubAssign for Matrix<M, N, T>
where
    T: SubAssign,
{
    fn sub_assign(&mut self, rhs: Self) {
        for i in 0..M {
            for j in 0..N {
                self.0[i][j] -= rhs.0[i][j];
            }
        }
    }
}

impl<const M: usize, const N: usize, T> Add<Self> for Matrix<M, N, T>
where
    Self: AddAssign,
{
    type Output = Self;

    fn add(mut self, rhs: Self) -> Self::Output {
        self += rhs;

        self
    }
}

impl<const M: usize, const N: usize, T> Sub<Self> for Matrix<M, N, T>
where
    Self: SubAssign,
{
    type Output = Self;

    fn sub(mut self, rhs: Self) -> Self::Output {
        self -= rhs;

        self
    }
}

impl<const M: usize, const N: usize, T: Copy, Rhs: Copy, O> Mul<Rhs> for Matrix<M, N, T>
where
    T: Mul<Rhs, Output = O>,
{
    type Output = Matrix<M, N, O>;

    fn mul(self, rhs: Rhs) -> Self::Output {
        Matrix(array::from_fn(|i| array::from_fn(|j| self.0[i][j] * rhs)))
    }
}

impl<const M: usize, const N: usize, T: Copy, Rhs: Copy, O> Div<Rhs> for Matrix<M, N, T>
where
    T: Div<Rhs, Output = O>,
{
    type Output = Matrix<M, N, O>;

    fn div(self, rhs: Rhs) -> Self::Output {
        Matrix(array::from_fn(|i| array::from_fn(|j| self.0[i][j] / rhs)))
    }
}

impl<const M: usize, const N: usize, T: Copy> Neg for Matrix<M, N, T>
where
    T: Neg<Output = T>,
{
    type Output = Matrix<M, N, T>;

    fn neg(mut self) -> Self::Output {
        for row in &mut self.0 {
            for val in row {
                *val = val.neg();
            }
        }

        self
    }
}

impl<const M: usize, const N: usize, T: Copy + PartialEq> Sum for Matrix<M, N, T>
where
    Self: Add<Self, Output = Self> + Zero,
{
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::zero(), |a, b| a + b)
    }
}

impl<const M: usize, const N: usize, T> Index<usize> for Matrix<M, N, T> {
    type Output = [T; N];

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl<const M: usize, const N: usize, T> IndexMut<usize> for Matrix<M, N, T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

impl<T> Deref for Matrix<1, 1, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.0[0][0]
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn add() {
        assert_eq!(
            Matrix([[1, 2], [3, 4]]) + Matrix([[1, 2], [3, 4]]),
            Matrix([[2, 4], [6, 8]])
        )
    }

    #[test]
    fn sub() {
        assert_eq!(
            Matrix([[1, 2], [3, 4]]) - Matrix([[1, 2], [3, 4]]),
            Matrix([[0, 0], [0, 0]])
        )
    }

    #[test]
    fn mul() {
        assert_eq!(Matrix([[1, 2], [3, 4]]) * 3, Matrix([[3, 6], [9, 12]]))
    }

    #[test]
    fn div() {
        assert_eq!(Matrix([[2, 4], [6, 8]]) / 2, Matrix([[1, 2], [3, 4]]))
    }

    #[test]
    fn neg() {
        assert_eq!(-Matrix([[2, 4], [6, 8]]), Matrix([[-2, -4], [-6, -8]]))
    }
}
