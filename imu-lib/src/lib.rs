#![no_std]

use core::{fmt::Debug, marker::PhantomData};

use linalg::{quaternion::UnitQuaternion, vector::Vector};
use uom::si::f32::{Acceleration, AngularVelocity, Length, Ratio, Velocity};

pub mod devices;
pub mod inertial_sensor;

const G: Acceleration = Acceleration {
    value: 9.80665,
    dimension: PhantomData,
    units: PhantomData,
};

pub trait AngularVelocitySensor {
    type Error: Debug;

    fn get_angular_velocity(&mut self) -> Result<Vector<3, AngularVelocity>, Self::Error>;
}

pub trait AngularPositionSensor {
    type Error: Debug;

    fn get_angular_position(&mut self) -> Result<UnitQuaternion<Ratio>, Self::Error>;
}

pub trait LinearAccelerationSensor {
    type Error: Debug;

    fn get_linear_acceleration(&mut self) -> Result<Vector<3, Acceleration>, Self::Error>;
}

pub trait LinearVelocitySensor {
    type Error: Debug;

    fn get_linear_velocity(&mut self) -> Result<Vector<3, Velocity>, Self::Error>;
}

pub trait LinearPositionSensor {
    type Error: Debug;

    fn get_linear_position(&mut self) -> Result<Vector<3, Length>, Self::Error>;
}
