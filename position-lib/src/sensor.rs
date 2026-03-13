use core::f32::consts::PI;

use uom::si::{
    angle::radian,
    f32::{Angle, Length, ReciprocalLength},
    length::meter,
    reciprocal_length::reciprocal_meter,
};

fn normal_likelyhood_length(x: Length, mean: Length, std_dev: Length) -> ReciprocalLength {
    let variance = std_dev * std_dev;

    let exponent = (x - mean) * (x - mean) / (2.0 * variance);
    let likelyhood = (1.0 / (2.0 * PI * variance).sqrt()) * exponent.exp();

    likelyhood
}

pub trait DistSensor {
    fn offset_x() -> Length;
    fn offset_y() -> Length;
    fn offset_rot() -> Angle;

    fn expected_meas(x: Length, y: Length, rot: Angle) -> Length;
    fn measurement_std_dev(x: Length, y: Length, rot: Angle) -> Length;

    fn likelyhood(&self, robot_state: &[f32], measurements: &[f32]) -> f32 {
        let curr_x = Length::new::<meter>(robot_state[0]);
        let curr_y = Length::new::<meter>(robot_state[1]);
        let curr_rot = Angle::new::<radian>(robot_state[2]);

        let curr_expected = Self::expected_meas(
            curr_x + Self::offset_x(),
            curr_y + Self::offset_y(),
            curr_rot + Self::offset_rot(),
        );

        let curr_std_dev = Self::measurement_std_dev(
            curr_x + Self::offset_x(),
            curr_y + Self::offset_y(),
            curr_rot + Self::offset_rot(),
        );

        let measured_dist = Length::new::<meter>(measurements[0]);

        normal_likelyhood_length(measured_dist, curr_expected, curr_std_dev)
            .get::<reciprocal_meter>()
    }
}
