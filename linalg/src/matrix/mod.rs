use core::{
    array,
    ops::{Add, AddAssign, Div, Mul, Neg, Sub},
};

use num_traits::Zero;
use uom::si::f32::{Angle, Ratio};

use crate::vector::Vector;

pub mod ops;
pub mod solver;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Matrix<const M: usize, const N: usize, T>(pub [[T; N]; M]);

impl<const M: usize, const N: usize, T: Default + Copy> Default for Matrix<M, N, T> {
    fn default() -> Self {
        Matrix([[Default::default(); N]; M])
    }
}

impl<const M: usize, const N: usize, T: Copy + Zero> Zero for Matrix<M, N, T>
where
    Self: Add<Self, Output = Self>,
    T: PartialEq,
{
    fn zero() -> Self {
        Matrix([[T::zero(); N]; M])
    }

    fn is_zero(&self) -> bool {
        self == &Self::zero()
    }
}

impl<const M: usize, const N: usize, T> From<[[T; N]; M]> for Matrix<M, N, T> {
    fn from(value: [[T; N]; M]) -> Self {
        Self(value)
    }
}

impl<const M: usize, const N: usize, T: Copy> Matrix<M, N, T> {
    pub fn from_cols(cols: [[T; M]; N]) -> Self {
        Self(array::from_fn(|i| array::from_fn(|j| cols[j][i])))
    }

    pub fn from_col_vectors(cols: [Vector<M, T>; N]) -> Self {
        Self::from_cols(cols.map(|col| col.to_array()))
    }

    pub fn product<const B: usize, R: Copy, O>(self, rhs: Matrix<N, B, R>) -> Matrix<M, B, O>
    where
        T: Mul<R, Output = O>,
        O: Zero + AddAssign,
    {
        Matrix(array::from_fn(|i| {
            array::from_fn(|j| {
                let mut val = O::zero();

                for k in 0..N {
                    val += self[i][k] * rhs[k][j]
                }

                val
            })
        }))
    }

    pub fn transpose(self) -> Matrix<N, M, T>
    where
        T: Default,
    {
        let mut result = [[T::default(); M]; N]; // Placeholder initialization

        for i in 0..M {
            for j in 0..N {
                result[j][i] = self[i][j];
            }
        }

        Matrix(result)
    }
}

impl<T: Copy> Matrix<2, 2, T> {
    pub fn det<D>(self) -> D
    where
        T: Mul<T, Output = D>,
        D: Sub<D, Output = D>,
    {
        self[0][0] * self[1][1] - self[0][1] * self[1][0]
    }

    pub fn inv<D: Copy, I>(self) -> Option<Matrix<2, 2, I>>
    where
        T: Neg<Output = T> + Mul<T, Output = D> + Div<D, Output = I>,
        D: Sub<D, Output = D> + Zero,
    {
        let det = self.det();

        if det.is_zero() {
            return None;
        }

        Some(Matrix([
            [self[1][1] / det, -self[0][1] / det],
            [-self[1][0] / det, self[0][0] / det],
        ]))
    }
}

impl Matrix<2, 2, Ratio> {
    pub fn rotation_matrix(angle: Angle) -> Self {
        Matrix([[angle.cos(), -angle.sin()], [angle.sin(), angle.cos()]])
    }
}

impl<const N: usize, T: Copy + Default> Matrix<2, N, T> {
    pub fn rotate<O>(self, angle: Angle) -> Matrix<2, N, O>
    where
        O: Zero + AddAssign,
        Ratio: Mul<T, Output = O>,
    {
        Matrix::rotation_matrix(angle).product(self)
    }
}

#[cfg(test)]
mod test {
    use core::f32::consts::FRAC_1_SQRT_2;

    use approx::assert_relative_eq;
    use uom::si::angle::degree;

    use super::*;
    use crate::real_vector;

    #[test]
    fn product() {
        let a = Matrix([[1, 2]]);
        let b = Matrix([[1], [2]]);

        assert_eq!(a.product(b), Matrix([[5]]));
        assert_eq!(b.product(a), Matrix([[1, 2], [2, 4]]));
    }

    #[test]
    fn transpose() {
        let a = Matrix([[1, 2, 3], [4, 5, 6]]);
        let b = Matrix([[1, 4], [2, 5], [3, 6]]);

        assert_eq!(a.transpose(), b);
    }

    #[test]
    fn rotate() {
        let right = real_vector![Length::meter, 1.0, 0.0];
        let up_diag = real_vector![Length::meter, FRAC_1_SQRT_2, FRAC_1_SQRT_2];
        let rotated = right.rotate(Angle::new::<degree>(45.0));

        assert_relative_eq!(rotated.x().value, up_diag.x().value);
        assert_relative_eq!(rotated.y().value, up_diag.y().value);
    }
}
