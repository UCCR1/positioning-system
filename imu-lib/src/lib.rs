#![no_std]

use core::fmt::Debug;

use embedded_hal::spi::SpiDevice;
use linalg::vector::Vector;
use uom::si::f32::{AngularVelocity, Velocity};

use crate::registers::{ReadRegister, RegisterError, WriteRegister};

pub mod macros;
pub mod modules;
pub mod registers;

pub const G: f32 = 9.80665;

pub trait Imu {
    type Error: Debug;

    fn get_angular_velocity(&mut self) -> Result<Vector<3, AngularVelocity>, Self::Error>;

    fn get_linear_velocity(&mut self) -> Result<Vector<3, Velocity>, Self::Error>;
}

pub trait SpiImu<D: SpiDevice> {
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
