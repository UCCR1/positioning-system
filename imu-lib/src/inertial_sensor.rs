use core::convert::Infallible;

use heapless::Deque;
use linalg::{
    quaternion::{Quaternion, UnitQuaternion},
    vector::Vector,
};
use thiserror::Error;
use uom::{
    num_traits::Zero,
    si::f32::{Acceleration, AngularVelocity, Ratio, Time},
};

use crate::{AngularPositionSensor, AngularVelocitySensor, LinearAccelerationSensor};
const NUM_CALIBRATION_SAMPLES: usize = 500;

#[derive(Error, Debug)]
pub enum InertialSensorError<E> {
    #[error("Calibration Incomplete")]
    IncompleteCalibration,
    #[error("Register Error {0}")]
    RegisterError(#[from] E),
}

pub struct InertialSensor<T> {
    device: T,

    // Calibration
    gyroscopic_drift: Vector<3, AngularVelocity>,
    linear_drift: Vector<3, Acceleration>,

    calibration_samples:
        Deque<(Vector<3, AngularVelocity>, Vector<3, Acceleration>), NUM_CALIBRATION_SAMPLES>,

    // Integration
    current_rotation: UnitQuaternion<Ratio>,
}

impl<T> InertialSensor<T> {
    pub fn new(device: T) -> Self {
        Self {
            device,

            gyroscopic_drift: Vector::default(),
            linear_drift: Vector::default(),

            calibration_samples: Deque::new(),

            current_rotation: UnitQuaternion::identity(),
        }
    }

    pub fn device(&mut self) -> &mut T {
        &mut self.device
    }
}

impl<E, T> InertialSensor<T>
where
    T: AngularVelocitySensor<Error = E> + LinearAccelerationSensor<Error = E>,
{
    pub fn reset_calibration(&mut self) {
        self.calibration_samples = Deque::new();

        self.gyroscopic_drift = Vector::default();
        self.linear_drift = Vector::default();
    }

    pub fn calibration_complete(&self) -> bool {
        self.calibration_samples.is_full()
    }

    pub fn calibrate(&mut self) -> Result<(), E> {
        let gyroscope_reading = self.device.get_angular_velocity()?;
        let accelerometer_reading = self.device.get_linear_acceleration()?;

        if self.calibration_samples.is_full() {
            self.calibration_samples.pop_front();
        }

        let _ = self
            .calibration_samples
            .push_back((gyroscope_reading, accelerometer_reading));

        for (gyroscope_sample, accelerometer_sample) in &self.calibration_samples {
            self.gyroscopic_drift += *gyroscope_sample;
            self.linear_drift += *accelerometer_sample;
        }

        self.gyroscopic_drift = self.gyroscopic_drift / self.calibration_samples.len() as f32;
        self.linear_drift = self.linear_drift / self.calibration_samples.len() as f32;

        Ok(())
    }

    pub fn integrate(&mut self, dt: Time) -> Result<(), InertialSensorError<E>> {
        if !self.calibration_complete() {
            return Err(InertialSensorError::IncompleteCalibration);
        }

        let current_velocity = self.get_angular_velocity()?;

        let p = Quaternion::from_array([
            AngularVelocity::zero(),
            current_velocity.x(),
            current_velocity.y(),
            current_velocity.z(),
        ]);

        let q_dot = self.current_rotation.hamilton_product(p) * 0.5;

        self.current_rotation = (*self.current_rotation + q_dot * dt).normalized();

        Ok(())
    }
}

impl<T> AngularVelocitySensor for InertialSensor<T>
where
    T: AngularVelocitySensor,
{
    type Error = T::Error;

    fn get_angular_velocity(&mut self) -> Result<Vector<3, AngularVelocity>, T::Error> {
        let reading = self.device.get_angular_velocity()?;

        Ok(reading - self.gyroscopic_drift)
    }
}

impl<T> LinearAccelerationSensor for InertialSensor<T>
where
    T: LinearAccelerationSensor,
{
    type Error = T::Error;

    fn get_linear_acceleration(&mut self) -> Result<Vector<3, Acceleration>, Self::Error> {
        self.device.get_linear_acceleration() // TODO: Account for linear drift rotated by angular position due gravity
    }
}

impl<T> AngularPositionSensor for InertialSensor<T> {
    type Error = Infallible;

    fn get_angular_position(&mut self) -> Result<UnitQuaternion<Ratio>, Self::Error> {
        Ok(self.current_rotation)
    }
}
