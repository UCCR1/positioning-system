#![no_std]

use core::{fmt::Debug, marker::PhantomData};

use embedded_hal::spi::SpiDevice;
use linalg::{quaternion::UnitQuaternion, vector::Vector};
use uom::si::f32::{Acceleration, AngularVelocity, Length, Ratio, Velocity};

use crate::registers::{ReadRegister, RegisterError, WriteRegister};

pub mod inertial_sensor;
pub mod macros;
pub mod modules;
pub mod registers;

pub const G: Acceleration = Acceleration {
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

pub(crate) trait RegisterDevice<D: SpiDevice> {
    fn device(&mut self) -> &mut D;

    fn read_register<const N: usize, T: ReadRegister<N>>(
        &mut self,
    ) -> Result<T, RegisterError<D::Error>> {
        T::read(self.device())
    }

    fn write_register<const N: usize, T: WriteRegister<N>>(
        &mut self,
        value: T,
    ) -> Result<(), RegisterError<D::Error>> {
        value.write(self.device())?;

        Ok(())
    }
}
