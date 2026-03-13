use core::ops::{Div, Mul, Neg, SubAssign};

use crate::{matrix::Matrix, vector::Vector};

fn hack_abs<T: PartialOrd + Default + Neg<Output = T>>(val: T) -> T {
    if val >= Default::default() { val } else { -val }
}

impl<const M: usize, const N: usize, T: Copy + Default> Matrix<M, N, T> {
    pub fn row(self, i: usize) -> Matrix<1, N, T> {
        return Matrix([self.0[i]]);
    }

    pub fn swap_rows(&mut self, a: usize, b: usize) {
        (self[a], self[b]) = (self[b], self[a])
    }

    pub fn solve<S, R, O>(mut self, mut b: Vector<M, R>) -> Option<Vector<N, O>>
    where
        T: Div<T, Output = S>
            + Mul<S, Output = T>
            + Mul<O, Output = R>
            + SubAssign<T>
            + Neg<Output = T>
            + PartialOrd
            + Default
            + Copy,
        R: Copy + Default + Mul<S, Output = R> + SubAssign<R> + PartialEq + Div<T, Output = O>,
        S: Copy,
        O: Copy + Default,
    {
        row_reduce(&mut self, &mut b);

        let mut result = [O::default(); N];

        for i in (0..M).rev() {
            let mut sum = b[i][0];

            for j in (i + 1)..N {
                sum -= self[i][j] * result[j];
            }

            if self[i][i] == Default::default() {
                if sum != Default::default() {
                    return None;
                }
            } else {
                result[i] = sum / self[i][i];
            }
        }

        Some(result.into())
    }
}

pub fn row_reduce<const M: usize, const N: usize, T, R, S>(
    A: &mut Matrix<M, N, T>,
    b: &mut Vector<M, R>,
) where
    T: Div<T, Output = S>
        + Mul<S, Output = T>
        + SubAssign<T>
        + Neg<Output = T>
        + PartialOrd
        + Default
        + Copy,
    R: Copy + Default + Mul<S, Output = R> + SubAssign<R>,
    S: Copy,
{
    let mut h = 0;
    let mut k = 0;

    while h < M && k < N {
        let pivot_row = (h..M)
            .max_by(|a, b| {
                hack_abs(A[*a][k])
                    .partial_cmp(&hack_abs(A[*b][k]))
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
            .unwrap();

        if A[pivot_row][k] == Default::default() {
            k += 1;
        } else {
            A.swap_rows(h, pivot_row);
            b.swap_rows(h, pivot_row);

            for i in (h + 1)..M {
                let scalar = A[i][k] / A[h][k];

                A[i] = (A.row(i) - A.row(h) * scalar).0[0];
                b[i][0] = (b.row(i) - b.row(h) * scalar).0[0][0];

                A[i][k] = Default::default();
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
