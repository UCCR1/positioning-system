use uom::si::f32::{Angle, Length, Ratio};

use crate::vector::Vector;

#[derive(Copy, Clone)]
pub struct TrackingWheel {
    location: Vector<2, Length>,
    direction: Vector<2, Ratio>,
}

pub struct Odometry<const N: usize> {
    tracking_wheels: [TrackingWheel; N],

    global_position: Vector<2, Length>,
    global_heading: Angle,
}

impl<const N: usize> Odometry<N> {
    pub fn update(&mut self, wheel_travel: [Length; N], delta_heading: Angle) {
        let projected_travel_vectors =
            wheel_travel
                .into_iter()
                .zip(self.tracking_wheels)
                .map(|(travel, position)| {
                    let measured_travel: Vector<2, Length> =
                        position.direction.normalized() * travel;

                    let rotation_travel: Vector<2, Length> =
                        -position.location.perp().bend(-delta_heading) * delta_heading;

                    measured_travel - rotation_travel.project(measured_travel)
                });

        let center_travel: Vector<2, Length> =
            projected_travel_vectors.fold(Vector::default(), |a, b| a + b) / N as f32;

        center_travel.closest_point(center_travel, center_travel);

        let true_travel = center_travel.bend(-delta_heading);

        self.global_position = self.global_position + true_travel;
        self.global_heading += delta_heading;
    }
}
