use core::array;

use core::ops::{AddAssign, Div, Mul, Neg, Sub};

use uom::si::f32::Angle;

pub mod ops;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Matrix<const M: usize, const N: usize, T>(pub [[T; N]; M]);

impl<const M: usize, const N: usize, T: Default + Copy> Default for Matrix<M, N, T> {
    fn default() -> Self {
        Matrix([[Default::default(); N]; M])
    }
}

impl<const M: usize, const N: usize, T> From<[[T; N]; M]> for Matrix<M, N, T> {
    fn from(value: [[T; N]; M]) -> Self {
        Self(value)
    }
}

impl<const M: usize, const N: usize, T> Matrix<M, N, T> {
    pub fn product<const B: usize, R, O>(self, rhs: Matrix<N, B, R>) -> Matrix<M, B, O>
    where
        T: Mul<R, Output = O> + Copy,
        R: Copy,
        O: Default + AddAssign,
    {
        Matrix(array::from_fn(|i| {
            array::from_fn(|j| {
                let mut val = O::default();

                for k in 0..N {
                    val += self[i][k] * rhs[k][j]
                }

                val
            })
        }))
    }

    pub fn transpose(self) -> Matrix<N, M, T>
    where
        T: Default + Copy,
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

impl<T> Matrix<2, 2, T> {
    pub fn det<O>(self) -> O
    where
        T: Mul<T, Output = O> + Copy,
        O: Sub<O, Output = O>,
    {
        self[0][0] * self[1][1] - self[0][1] * self[1][0]
    }

    pub fn inv<O, D>(self) -> Matrix<2, 2, D>
    where
        T: Neg<Output = T> + Mul<T, Output = O> + Div<O, Output = D> + Copy,
        O: Sub<O, Output = O> + Copy,
    {
        let det = self.det();

        Matrix([
            [self[1][1] / det, -self[0][1] / det],
            [-self[1][0] / det, self[0][0] / det],
        ])
    }
}

impl Matrix<2, 2, f32> {
    pub fn rotation_matrix(angle: Angle) -> Self {
        Matrix([
            [libm::cosf(angle.value), -libm::sinf(angle.value)],
            [libm::sinf(angle.value), libm::cosf(angle.value)],
        ])
    }
}

impl<const N: usize, T> Matrix<2, N, T> {
    pub fn rotate<O>(self, angle: Angle) -> Matrix<2, N, O>
    where
        T: Default + Copy,
        O: Default + AddAssign,
        f32: Mul<T, Output = O>,
    {
        Matrix::rotation_matrix(angle).product(self)
    }
}

#[cfg(test)]
mod test {
    use core::f32::consts::FRAC_1_SQRT_2;

    use uom::si::angle::degree;

    use crate::vector;

    use super::*;

    use approx::assert_relative_eq;

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
        let right = vector![1.0, 0.0];
        let up_diag = vector![FRAC_1_SQRT_2, FRAC_1_SQRT_2];
        let rotated = right.rotate(Angle::new::<degree>(45.0));

        assert_relative_eq!(rotated.x(), up_diag.x());
        assert_relative_eq!(rotated.y(), up_diag.y());
    }
}
