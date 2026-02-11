use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vector<const N: usize>(pub [f32; N]);

impl<const N: usize> Vector<N> {
    pub const ZERO: Self = Self([0.0; N]);

    pub fn dot(self, rhs: Self) -> f32 {
        self.0.into_iter().zip(rhs.0).map(|(a, b)| a * b).sum()
    }

    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn distance_to(self, target: Self) -> f32 {
        (target - self).length()
    }

    pub fn lerp(self, target: Self, t: f32) -> Self {
        target * t + self * (1.0 - t)
    }

    pub fn project(self, target: Self) -> Self {
        target * (target.dot(self) / target.dot(target))
    }

    pub fn closest_point(self, start: Self, end: Self) -> Self {
        let v = end - start;
        let d = self - start;

        let t = (v.dot(d) / v.dot(v)).clamp(0.0, 1.0);

        start + v * t
    }

    pub fn normalized(self) -> Self {
        self / self.length()
    }
}

impl<const N: usize> Add<Vector<N>> for Vector<N> {
    type Output = Vector<N>;

    fn add(self, rhs: Vector<N>) -> Self::Output {
        let mut result = [0.0; N];

        for (i, (a, b)) in self.0.into_iter().zip(rhs.0.into_iter()).enumerate() {
            result[i] = a + b;
        }

        Self(result)
    }
}

impl<const N: usize> Mul<f32> for Vector<N> {
    type Output = Vector<N>;

    fn mul(self, rhs: f32) -> Self::Output {
        let mut result = [0.0; N];

        for i in 0..N {
            result[i] = self.0[i] * rhs;
        }

        Self(result)
    }
}

impl<const N: usize> Neg for Vector<N> {
    type Output = Vector<N>;

    fn neg(self) -> Self::Output {
        self * -1.0
    }
}

impl<const N: usize> Sub<Vector<N>> for Vector<N> {
    type Output = Vector<N>;

    fn sub(self, rhs: Vector<N>) -> Self::Output {
        self + -rhs
    }
}

impl<const N: usize> Div<f32> for Vector<N> {
    type Output = Vector<N>;

    fn div(self, rhs: f32) -> Self::Output {
        self * (1.0 / rhs)
    }
}

impl Vector<2> {
    pub fn x(self) -> f32 {
        self.0[0]
    }

    pub fn y(self) -> f32 {
        self.0[1]
    }

    pub fn new(x: f32, y: f32) -> Self {
        Self([x, y])
    }
}

impl Vector<3> {
    pub fn x(self) -> f32 {
        self.0[0]
    }

    pub fn y(self) -> f32 {
        self.0[1]
    }

    pub fn z(self) -> f32 {
        self.0[2]
    }

    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self([x, y, z])
    }

    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
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
            Vector::ZERO
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
