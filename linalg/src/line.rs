use core::{
    iter::Sum,
    ops::{AddAssign, Div, Mul, Neg, Sub, SubAssign},
};

use crate::{
    matrix::Matrix,
    vector::{Vector, real::Root},
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line<const N: usize, T>(pub (Vector<N, T>, Vector<N, T>));

impl<const N: usize, T: Copy + Default> Line<N, T> {
    pub fn length<S, R>(self) -> T
    where
        T: Mul<T, Output = S> + Div<Output = R> + SubAssign,
        S: Sum + Root<Root = T>,
    {
        let (start, end) = self.0;

        (end - start).magnitude()
    }

    pub fn lerp<S>(self, t: S) -> Vector<N, T>
    where
        T: Mul<S, Output = T> + AddAssign + SubAssign,
        S: Copy,
    {
        let (start, end) = self.0;

        start + end * t - start * t
    }

    pub fn closest_point<S, O>(self, point: Vector<N, T>) -> Vector<N, T>
    where
        T: Mul<T, Output = O> + Mul<S, Output = T> + AddAssign + SubAssign,
        S: Copy,
        O: Copy + Div<O, Output = S> + Sum + PartialOrd + Sub<O, Output = O>,
    {
        let (start, end) = self.0;

        let v = end - start;
        let d = point - start;

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

    pub fn contains_point<S, R>(self, point: Vector<N, T>, tolerance: S) -> bool
    where
        T: Mul<T, Output = S> + Div<Output = R> + SubAssign + PartialOrd + Sum,
        S: Sum + Root<Root = T> + PartialOrd + Sub<S, Output = S>,
    {
        let (start, end) = self.0;

        let ab = end - start;
        let ap = point - start;

        let dot_product = ap.dot(ab);

        let mag_square = ab.magnitude() * ap.magnitude();

        // We don't have .abs() on these generic types
        if dot_product == mag_square {
            true
        } else if dot_product < mag_square {
            mag_square - dot_product <= tolerance
        } else {
            dot_product - mag_square <= tolerance
        }
    }

    pub fn intersection<S>(self, other: Self) -> Option<Vector<N, T>>
    where
        T: AddAssign
            + SubAssign
            + Div<T, Output = S>
            + Mul<S, Output = T>
            + Neg<Output = T>
            + PartialOrd,
        S: Copy + Default,
    {
        let (p1, o1) = self.0;
        let (p2, o2) = other.0;

        let v1 = o1 - p1;
        let v2 = o2 - p2;

        let A = Matrix::from_col_vectors([v1, -v2]);

        let b = p2 - p1;

        let scalars = A.solve(b)?;

        // TODO: Clamp to 0<>1 range

        Some(p1 + v1 * scalars[0][0])
    }
}

#[cfg(test)]
mod test {
    use approx::assert_relative_eq;
    use uom::si::{area::square_meter, quantities::Area};

    use crate::{line::Line, real_vector};

    #[test]
    fn contains_point() {
        let line = Line((
            real_vector![Length::meter, 0.0, 0.0],
            real_vector![Length::meter, 2.0, 2.0],
        ));

        assert!(line.contains_point(
            real_vector![Length::meter, 1.0, 1.0],
            Area::new::<square_meter>(0.00001)
        ));
    }

    #[test]
    fn test_find_intersection() {
        let line_seg1 = Line((
            real_vector!(Length::meter, 1.0, 1.0),
            real_vector!(Length::meter, 5.0, 5.0),
        ));
        let line_seg2 = Line((
            real_vector!(Length::meter, 2.0, 1.0),
            real_vector!(Length::meter, 2.0, 5.0),
        ));

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
}
