use std::f32::consts::SQRT_2;

use uom::si::{
    f32::{Angle, Length, Ratio},
    ratio::ratio,
};

pub struct Odometry {
    tracking_width: Length,

    tracking_offset: Length,

    turn_tolerance: Angle,

    global_x: Length,
    global_y: Length,
    global_heading: Angle,
}

impl Odometry {
    pub fn update(
        &mut self,
        left_wheel_travel: Length,
        right_wheel_travel: Length,
        delta_heading: Angle,
    ) {
        let local_delta_x = (left_wheel_travel - right_wheel_travel) * SQRT_2 / 4.0;
        let local_delta_y = (left_wheel_travel + right_wheel_travel) * SQRT_2 / 4.0;

        let (true_delta_x, true_delta_y) = if delta_heading.abs() > self.turn_tolerance {
            let center_delta_x = local_delta_x
                - delta_heading * (self.tracking_offset + self.tracking_width * 0.5) / SQRT_2;
            let center_delta_y = local_delta_y;

            (
                delta_heading.sin() * center_delta_x / delta_heading
                    + (Ratio::new::<ratio>(1.0) - delta_heading.cos()) * center_delta_y
                        / delta_heading,
                delta_heading.sin() * center_delta_y / delta_heading
                    - (Ratio::new::<ratio>(1.0) - delta_heading.cos()) * center_delta_x
                        / delta_heading,
            )
        } else {
            (local_delta_x, local_delta_y)
        };

        self.global_x +=
            true_delta_y * self.global_heading.sin() + true_delta_x * self.global_heading.cos();

        self.global_y +=
            true_delta_y * self.global_heading.cos() + true_delta_x * self.global_heading.sin();

        self.global_heading += delta_heading;
    }
}
