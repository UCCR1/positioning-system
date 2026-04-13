#![no_std]

use core::{fmt::Debug, marker::PhantomData};

use embedded_hal::spi::SpiDevice;
use heapless::Deque;
use linalg::vector::Vector;
use uom::si::f32::{Acceleration, AngularVelocity};

use crate::registers::{ReadRegister, RegisterError, WriteRegister};

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

pub trait LinearAccelerationSensor {
    type Error: Debug;

    fn get_linear_acceleration(&mut self) -> Result<Vector<3, Acceleration>, Self::Error>;
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

pub struct CalibratableImu<const N: usize, T> {
    imu: T,

    gyroscopic_drift: Vector<3, AngularVelocity>,

    gyroscope_calibration_samples: Deque<Vector<3, AngularVelocity>, N>,
}

impl<const N: usize, T> CalibratableImu<N, T> {
    pub fn new(imu: T) -> Self {
        Self {
            imu,

            gyroscopic_drift: Vector::default(),

            gyroscope_calibration_samples: Deque::new(),
        }
    }

    pub fn base(&mut self) -> &mut T {
        &mut self.imu
    }
}

impl<const N: usize, E, T: AngularVelocitySensor<Error = E> + LinearAccelerationSensor<Error = E>>
    CalibratableImu<N, T>
{
    fn compute_statistics(&mut self) {
        self.gyroscopic_drift = Vector::default();

        for sample in &self.gyroscope_calibration_samples {
            self.gyroscopic_drift += *sample;
        }

        self.gyroscopic_drift =
            self.gyroscopic_drift / self.gyroscope_calibration_samples.len() as f32;
    }

    pub fn reset_calibration(&mut self) {
        self.gyroscope_calibration_samples = Deque::new();
        self.gyroscopic_drift = Vector::default();
    }

    pub fn calibration_complete(&self) -> bool {
        self.gyroscope_calibration_samples.is_full()
    }

    pub fn calibrate(&mut self) -> Result<(), E> {
        let gyroscope_reading = self.base().get_angular_velocity()?;

        if self.gyroscope_calibration_samples.is_full() {
            self.gyroscope_calibration_samples.pop_front();
        }

        let _ = self
            .gyroscope_calibration_samples
            .push_back(gyroscope_reading);

        self.compute_statistics();

        Ok(())
    }
}

impl<const N: usize, T: AngularVelocitySensor> AngularVelocitySensor for CalibratableImu<N, T> {
    type Error = T::Error;

    fn get_angular_velocity(&mut self) -> Result<Vector<3, AngularVelocity>, T::Error> {
        let reading = self.base().get_angular_velocity()?;

        Ok(reading - self.gyroscopic_drift)
    }
}
