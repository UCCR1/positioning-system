use core::array;

use num_traits::ConstZero;
use position_lib::odometry::{Odometry, TrackingWheel};
use uom::si::f32::{Angle, Length};

use crate::encoder::QuadratureEncoder;

pub struct OdometryTask<const N: usize> {
    module: Odometry<N>,

    wheel_diameter: Length,

    encoders: [QuadratureEncoder; N],
    last_angle: Angle,

    last_positions: [Angle; N],
}

impl<const N: usize> OdometryTask<N> {
    pub fn new(
        encoders: [QuadratureEncoder; N],
        wheels: [TrackingWheel; N],
        wheel_diameter: Length,
    ) -> Self {
        let module = Odometry::new(wheels);

        Self {
            module,
            encoders,
            wheel_diameter,

            last_angle: Default::default(),
            last_positions: [Default::default(); N],
        }
    }

    pub fn update(&mut self) {
        let new_positions: [Angle; N] = array::from_fn(|i| self.encoders[i].position());

        let travels: [Length; N] = array::from_fn(|i| {
            (new_positions[i] - self.last_positions[i]) * self.wheel_diameter / 2.0
        });

        let angle_change = Angle::ZERO;

        self.module.update(travels, angle_change);

        self.last_positions = new_positions;
    }
}
