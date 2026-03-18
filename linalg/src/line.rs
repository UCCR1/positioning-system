use core::{
    iter::Sum,
    ops::{AddAssign, Div, Mul, Neg, Sub, SubAssign},
};

use num_traits::{ConstOne, ConstZero, One, Zero};
use uom::si::f32::{Length, Ratio};

use crate::{
    matrix::Matrix,
    vector::{Vector, real::Root},
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line<const N: usize, T>(pub Vector<N, T>, pub Vector<N, T>);

impl<const N: usize, T: Copy> Line<N, T> {
    pub fn length<S, R>(self) -> T
    where
        T: Mul<T, Output = S> + Div<Output = R> + SubAssign,
        S: Sum + Root<Root = T>,
    {
        let Self(start, end) = self;

        (end - start).magnitude()
    }

    pub fn lerp<S: Copy>(self, t: S) -> Vector<N, T>
    where
        T: Mul<S, Output = T> + AddAssign + SubAssign,
    {
        let Self(start, end) = self;

        start + end * t - start * t
    }

    pub fn clamped_lerp<S: Copy>(self, t: S) -> Vector<N, T>
    where
        T: Mul<S, Output = T> + AddAssign + SubAssign,
        S: Zero + One + PartialOrd,
    {
        if t <= S::zero() {
            self.lerp(S::zero())
        } else if t >= S::one() {
            self.lerp(S::one())
        } else {
            self.lerp(t)
        }
    }

    pub fn bounded_lerp<S: Copy>(self, t: S) -> Option<Vector<N, T>>
    where
        T: Mul<S, Output = T> + AddAssign + SubAssign,
        S: Zero + One + PartialOrd,
    {
        if t >= S::zero() && t <= S::one() {
            Some(self.lerp(t))
        } else {
            None
        }
    }

    pub fn n_intersection<S: Copy>(self, other: Self) -> Option<Vector<N, T>>
    where
        T: AddAssign
            + SubAssign
            + Div<T, Output = S>
            + Mul<S, Output = T>
            + Neg<Output = T>
            + PartialOrd
            + Zero,
        S: Copy + Zero + One + PartialOrd,
    {
        let Self(p1, o1) = self;
        let Self(p2, o2) = other;

        let v1 = o1 - p1;
        let v2 = o2 - p2;

        let coeff_matrix = Matrix::from_col_vectors([v1, -v2]);

        let constants = p2 - p1;

        let scalars = coeff_matrix.solve(constants)?;

        let [s] = scalars[0];
        let [t] = scalars[1];

        if s < S::zero() || s > S::one() || t < S::zero() || t > S::one() {
            return None;
        }

        Some(p1 + v1 * s)
    }
}

impl<T: Copy> Line<2, T> {
    pub fn intersection<D: Copy, I: Copy, S: Copy>(self, other: Self) -> Option<Vector<2, T>>
    where
        T: AddAssign
            + SubAssign
            + Neg<Output = T>
            + Mul<T, Output = D>
            + Div<D, Output = I>
            + Mul<S, Output = T>,
        D: Sub<D, Output = D> + Zero,
        S: Zero + AddAssign + Zero + PartialOrd + One,
        I: Mul<T, Output = S>,
    {
        let Self(p1, o1) = self;
        let Self(p2, o2) = other;

        let v1 = o1 - p1;
        let v2 = o2 - p2;

        let coeff_matrix = Matrix::from_col_vectors([v1, -v2]);

        let constants = p2 - p1;

        let scalars = coeff_matrix.inv()?.product(constants);

        let [s] = scalars[0];
        let [t] = scalars[1];

        if s < S::zero() || s > S::one() || t < S::zero() || t > S::one() {
            return None;
        }

        Some(p1 + v1 * s)
    }
}

pub enum LineCircleIntersection<const N: usize, T> {
    None,
    Single(Vector<N, T>),
    Dual(Vector<N, T>, Vector<N, T>),
}

impl<const N: usize> Line<N, Length> {
    pub fn closest_point(self, point: Vector<N, Length>) -> Vector<N, Length> {
        let Self(start, end) = self;

        let v = end - start;
        let d = point - start;

        let t = v.dot(d) / v.dot(v);

        self.clamped_lerp(t)
    }

    pub fn circle_intersection_scalars(
        self,
        center: Vector<N, Length>,
        radius: Length,
    ) -> Option<(Ratio, Ratio)> {
        let Self(start, end) = self;

        let v = end - start;
        let d = start - center;

        let a = v.dot(v);
        let b = 2.0 * d.dot(v);
        let c = d.dot(d) - radius * radius;

        let discriminant = b * b - (4.0 * a * c);

        if discriminant.is_sign_negative() {
            return None;
        }

        Some((
            (discriminant.sqrt() - b) / (2.0 * a),
            (-discriminant.sqrt() - b) / (2.0 * a),
        ))
    }

    pub fn circle_intersection(
        self,
        center: Vector<N, Length>,
        radius: Length,
    ) -> LineCircleIntersection<N, Length> {
        let t = self.circle_intersection_scalars(center, radius);

        match t {
            Some((t1, t2)) => {
                let p1 = self.bounded_lerp(t1);
                let p2 = self.bounded_lerp(t2);

                match (p1, p2) {
                    (Some(p1), Some(p2)) => {
                        if t1 == t2 {
                            LineCircleIntersection::Single(p1)
                        } else {
                            LineCircleIntersection::Dual(p1, p2)
                        }
                    }
                    (Some(p), None) | (None, Some(p)) => LineCircleIntersection::Single(p),
                    (None, None) => LineCircleIntersection::None,
                }
            }
            None => LineCircleIntersection::None,
        }
    }

    pub fn circle_intersection_snapped(
        self,
        center: Vector<N, Length>,
        radius: Length,
    ) -> Vector<N, Length> {
        let t = self.circle_intersection_scalars(center, radius);

        let Some((t, _)) = t else {
            return self.closest_point(center);
        };

        self.clamped_lerp(t)
    }
}

#[cfg(test)]
mod test {
    use approx::assert_relative_eq;
    use uom::si::{f32::Length, length::meter};

    use crate::{line::Line, real_vector};

    #[test]
    fn test_find_intersection() {
        let line_seg1 = Line(
            real_vector!(Length::meter, 0.0, 0.0),
            real_vector!(Length::meter, 5.0, 5.0),
        );
        let line_seg2 = Line(
            real_vector!(Length::meter, 2.0, 0.0),
            real_vector!(Length::meter, 2.0, 5.0),
        );

        assert_relative_eq!(
            line_seg1.intersection(line_seg2).unwrap().x().value,
            real_vector!(Length::meter, 2.0, 2.0).x().value,
            max_relative = 0.01
        );

        assert_relative_eq!(
            line_seg1.intersection(line_seg2).unwrap().y().value,
            real_vector!(Length::meter, 2.0, 2.0).y().value,
            max_relative = 0.01
        );
    }

    #[test]
    fn test_find_intersection2() {
        let line_seg1 = Line(
            real_vector!(Length::meter, 0.0, 0.0),
            real_vector!(Length::meter, 5.0, 5.0),
        );
        let line_seg2 = Line(
            real_vector!(Length::meter, 0.0, 5.0),
            real_vector!(Length::meter, 5.0, 0.0),
        );

        assert_relative_eq!(
            line_seg1.intersection(line_seg2).unwrap().x().value,
            real_vector!(Length::meter, 2.5, 2.5).x().value,
            max_relative = 0.01
        );

        assert_relative_eq!(
            line_seg1.intersection(line_seg2).unwrap().y().value,
            real_vector!(Length::meter, 2.5, 2.5).y().value,
            max_relative = 0.01
        );
    }

    #[test]
    fn test_find_intersection_outside_segments() {
        let line_seg1 = Line(
            real_vector!(Length::meter, 0.0, 0.0),
            real_vector!(Length::meter, 5.0, 5.0),
        );
        let line_seg2 = Line(
            real_vector!(Length::meter, 2.0, 0.0),
            real_vector!(Length::meter, 5.0, 0.0),
        );

        assert_eq!(line_seg1.intersection(line_seg2), None)
    }

    #[test]
    fn test_find_intersection_parellel_lines() {
        let line_seg1 = Line(
            real_vector!(Length::meter, 0.0, 0.0),
            real_vector!(Length::meter, 5.0, 5.0),
        );
        let line_seg2 = Line(
            real_vector!(Length::meter, 0.0, -1.0),
            real_vector!(Length::meter, 5.0, 4.0),
        );

        assert_eq!(line_seg1.intersection(line_seg2), None)
    }

    #[test]
    fn test_circle_intersection_end() {
        let start = real_vector!(Length::meter, 0.0, 0.0);
        let end = real_vector!(Length::meter, 0.0, 1.0);
        let line_seg1 = Line(start, end);

        let position = real_vector!(Length::meter, -2.0, -2.0);

        let length = Length::new::<meter>(1.5);

        assert_eq!(
            line_seg1.circle_intersection_scalars(position, length),
            None
        );
        assert_eq!(
            line_seg1.circle_intersection_snapped(position, length),
            start
        );
    }
}
