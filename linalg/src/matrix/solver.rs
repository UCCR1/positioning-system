use core::ops::{Div, Mul, Neg, SubAssign};

use num_traits::Zero;

use crate::{abs, matrix::Matrix, vector::Vector};

impl<const M: usize, const N: usize, T: Copy> Matrix<M, N, T> {
    pub fn row(self, i: usize) -> Matrix<1, N, T> {
        return Matrix([self.0[i]]);
    }

    pub fn swap_rows(&mut self, a: usize, b: usize) {
        (self[a], self[b]) = (self[b], self[a])
    }

    pub fn solve<S: Copy, R: Copy, O: Copy>(mut self, mut b: Vector<M, R>) -> Option<Vector<N, O>>
    where
        T: Div<T, Output = S>
            + Mul<S, Output = T>
            + Mul<O, Output = R>
            + SubAssign
            + Neg<Output = T>
            + PartialOrd
            + Zero,
        R: Mul<S, Output = R> + SubAssign<R> + Div<T, Output = O> + Zero,
        O: Zero,
    {
        row_reduce(&mut self, &mut b);

        let mut result = [O::zero(); N];

        for i in (0..M).rev() {
            let mut sum = b[i][0];

            for j in (i + 1)..N {
                sum -= self[i][j] * result[j];
            }

            if self[i][i].is_zero() {
                if !sum.is_zero() {
                    return None;
                }
            } else {
                result[i] = sum / self[i][i];
            }
        }

        Some(result.into())
    }
}

pub fn row_reduce<const M: usize, const N: usize, T: Copy, R: Copy, S: Copy>(
    coeff_matrix: &mut Matrix<M, N, T>,
    constants: &mut Vector<M, R>,
) where
    T: Div<T, Output = S> + Mul<S, Output = T> + SubAssign + Neg<Output = T> + PartialOrd + Zero,
    R: Mul<S, Output = R> + SubAssign,
{
    let mut h = 0;
    let mut k = 0;

    while h < M && k < N {
        let mut pivot_row = h;

        for i in h..M {
            if abs(coeff_matrix[i][k]) > abs(coeff_matrix[pivot_row][k]) {
                pivot_row = i;
            }
        }

        if coeff_matrix[pivot_row][k] == T::zero() {
            k += 1;
        } else {
            coeff_matrix.swap_rows(h, pivot_row);
            constants.swap_rows(h, pivot_row);

            for i in (h + 1)..M {
                let scalar = coeff_matrix[i][k] / coeff_matrix[h][k];

                coeff_matrix[i] = (coeff_matrix.row(i) - coeff_matrix.row(h) * scalar).0[0];
                constants[i][0] = (constants.row(i) - constants.row(h) * scalar).0[0][0];

                coeff_matrix[i][k] = T::zero();
            }

            h += 1;
            k += 1;
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_row_reduce() {
        let mut A = Matrix([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        let mut b = Matrix([[1.0], [1.0], [1.0]]);

        row_reduce(&mut A, &mut b);
    }

    #[test]
    fn test_solve1() {
        let A = Matrix([[2.0, 1.0], [1.0, 1.0]]);
        let b = Matrix([[3.0], [2.0]]);

        let solution = A.solve(b).unwrap();

        assert_eq!(solution.x(), 1.0);
        assert_eq!(solution.y(), 1.0);
    }

    #[test]
    fn test_solve2() {
        let A = Matrix([[5.0, 0.0], [5.0, -5.0]]);
        let b = Matrix([[2.0], [0.0]]);

        let solution = A.solve(b).unwrap();

        assert_eq!(solution.x(), 2.0 / 5.0);
        assert_eq!(solution.y(), 2.0 / 5.0);
    }
}
