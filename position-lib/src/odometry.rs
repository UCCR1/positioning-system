use uom::si::f32::{Angle, Length, Ratio};

use crate::vector::{Vector, real::UnitVector};

#[derive(Copy, Clone)]
pub struct TrackingWheel {
    pub location: Vector<2, Length>,
    pub direction: UnitVector<2, Ratio>,
    pub weighting: Vector<2, Ratio>,
}

pub struct Odometry<const N: usize> {
    tracking_wheels: [TrackingWheel; N],

    global_position: Vector<2, Length>,
    global_heading: Angle,
}

impl<const N: usize> Odometry<N> {
    pub fn new(wheels: [TrackingWheel; N]) -> Self {
        Self {
            tracking_wheels: wheels,
            global_position: Default::default(),
            global_heading: Default::default(),
        }
    }

    pub fn update(&mut self, wheel_travel: [Length; N], delta_heading: Angle) {
        let center_travel: Vector<2, Length> = wheel_travel
            .into_iter()
            .zip(self.tracking_wheels)
            .map(|(travel, position)| {
                let measured_travel: Vector<2, Length> = *position.direction * travel;

                let rotation_travel: Vector<2, Length> = -position.location.perp() * delta_heading;

                measured_travel - rotation_travel.project(measured_travel)
            })
            .fold(Default::default(), |a, b| a + b);

        let true_travel = center_travel/*.bend(-delta_heading)*/;

        self.global_position = self.global_position + true_travel;
        self.global_heading += delta_heading;
    }
}
