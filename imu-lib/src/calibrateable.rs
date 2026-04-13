use heapless::Deque;
use linalg::vector::Vector;
use uom::si::f32::AngularVelocity;

use crate::{AngularVelocitySensor, LinearAccelerationSensor};

pub struct CalibratableImu<const N: usize, T> {
    base: T,

    gyroscopic_drift: Vector<3, AngularVelocity>,

    gyroscope_calibration_samples: Deque<Vector<3, AngularVelocity>, N>,
}

impl<const N: usize, T> CalibratableImu<N, T> {
    pub fn new(base: T) -> Self {
        Self {
            base,

            gyroscopic_drift: Vector::default(),

            gyroscope_calibration_samples: Deque::new(),
        }
    }

    pub fn base(&mut self) -> &mut T {
        &mut self.base
    }
}

impl<const N: usize, E, T> CalibratableImu<N, T>
where
    T: AngularVelocitySensor<Error = E> + LinearAccelerationSensor<Error = E>,
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

impl<const N: usize, T> AngularVelocitySensor for CalibratableImu<N, T>
where
    T: AngularVelocitySensor,
{
    type Error = T::Error;

    fn get_angular_velocity(&mut self) -> Result<Vector<3, AngularVelocity>, T::Error> {
        let reading = self.base().get_angular_velocity()?;

        Ok(reading - self.gyroscopic_drift)
    }
}
