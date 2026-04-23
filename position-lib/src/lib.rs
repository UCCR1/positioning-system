#![no_std]

use linalg::{real_vector, vector::Vector};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use uom::si::{
    angle::radian,
    f32::{Angle, Length},
    length::meter,
};

pub mod lidar_localization;
pub mod monte_carlo;
pub mod odometry;
pub mod probability;

#[derive(Copy, Clone, Default, Debug)]
pub struct Pose {
    position: Vector<2, Length>,
    heading: Angle,
}

impl Pose {
    pub fn with_noise(self, noise: PoseNoiseParameters, rng: &mut impl Rng) -> Self {
        let angle_distribution = Normal::new(0.0, noise.angle_deviation.get::<radian>()).unwrap();
        let position_distribution =
            Normal::new(0.0, noise.position_deviation.get::<meter>()).unwrap();

        Self {
            heading: self.heading + Angle::new::<radian>(angle_distribution.sample(rng)),
            position: self.position
                + real_vector![
                    Length::meter,
                    position_distribution.sample(rng),
                    position_distribution.sample(rng)
                ],
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct PoseNoiseParameters {
    position_deviation: Length,
    angle_deviation: Angle,
}
