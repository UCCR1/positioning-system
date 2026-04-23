use core::array;

use linalg::{
    matrix::Matrix,
    vector::{Vector, real::UnitVector},
};
use uom::si::f32::{Angle, Length, Ratio};

#[derive(Copy, Clone)]
pub struct TrackingWheel {
    pub location: Vector<2, Length>,
    pub direction: UnitVector<2, Ratio>,
}

pub struct Odometry<const N: usize> {
    tracking_wheels: [TrackingWheel; N],

    weighting_matrix: Matrix<N, 2, Ratio>,
}

impl<const N: usize> Odometry<N> {
    pub fn new(wheels: [TrackingWheel; N]) -> Self {
        let d =
            Matrix::<N, 2, Ratio>::from(wheels.map(|wheel| wheel.direction.to_array())).transpose();

        let ddt = d.product(d.transpose());

        let ddt_inv = ddt.inv().expect("Tracking wheel system is not solvable");

        let weighting_matrix = d.transpose().product(ddt_inv);

        Self {
            tracking_wheels: wheels,
            weighting_matrix,
        }
    }

    /// Calculates local change in position relative to robot reference frame,
    /// pre change in angle
    pub fn update(&self, wheel_travel: [Length; N], angle_change: Angle) -> Vector<2, Length> {
        let travels: [Length; N] = array::from_fn(|i| {
            let travel = wheel_travel[i];
            let position = self.tracking_wheels[i];

            let measured_travel = *position.direction * travel;

            let rotation_travel = position.location.perp() * angle_change;

            let actual_travel = measured_travel - rotation_travel.project(measured_travel);

            actual_travel.magnitude()
        });

        let true_travel = self
            .weighting_matrix
            .transpose()
            .product(travels.into())
            .bend(angle_change);

        true_travel
    }
}
