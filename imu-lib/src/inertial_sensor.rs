use core::convert::Infallible;

use heapless::Deque;
use linalg::{
    quaternion::{Quaternion, UnitQuaternion},
    vector::Vector,
};
use num_traits::{One, zero};
use thiserror::Error;
use uom::{
    num_traits::Zero,
    si::f32::{Acceleration, AngularVelocity, Length, Ratio, Time, Velocity},
};

use crate::{
    AngularPositionSensor, AngularVelocitySensor, LinearAccelerationSensor, LinearPositionSensor,
    LinearVelocitySensor,
};
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

    current_linear_velocity: Vector<3, Velocity>,
    current_linear_position: Vector<3, Length>,
}

impl<T> InertialSensor<T> {
    pub fn new(device: T) -> Self {
        Self {
            device,

            gyroscopic_drift: Vector::default(),
            linear_drift: Vector::default(),

            calibration_samples: Deque::new(),

            current_rotation: UnitQuaternion::identity(),

            current_linear_velocity: Vector::default(),
            current_linear_position: Vector::default(),
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

    fn calibration_rotation_from_gravity_drift(&mut self) {
        let drift_direction = self.linear_drift.normalized();

        let rotation_axis = Vector::k().cross(*drift_direction);

        let d = Vector::k().dot(*drift_direction);

        self.current_rotation =
            Quaternion::from_direction_component(Ratio::one() + d, rotation_axis).normalized();
    }

    fn calculate_calibration_properties(&mut self) {
        for (gyroscope_sample, accelerometer_sample) in &self.calibration_samples {
            self.gyroscopic_drift += *gyroscope_sample;
            self.linear_drift += *accelerometer_sample;
        }

        self.gyroscopic_drift = self.gyroscopic_drift / self.calibration_samples.len() as f32;
        self.linear_drift = self.linear_drift / self.calibration_samples.len() as f32;

        // self.calibration_rotation_from_gravity_drift();
    }

    pub fn calibrate(&mut self) -> Result<(), E> {
        // Get sensor readings and add to sample queue
        let gyroscope_reading = self.device.get_angular_velocity()?;
        let accelerometer_reading = self.device.get_linear_acceleration()?;

        if self.calibration_samples.is_full() {
            self.calibration_samples.pop_front();
        }

        let _ = self
            .calibration_samples
            .push_back((gyroscope_reading, accelerometer_reading));

        self.calculate_calibration_properties();

        Ok(())
    }

    pub fn integrate(&mut self, dt: Time) -> Result<(), InertialSensorError<E>> {
        if !self.calibration_complete() {
            return Err(InertialSensorError::IncompleteCalibration);
        }

        let current_velocity = self.get_angular_velocity()?;

        let p = Quaternion::from_direction_component(AngularVelocity::zero(), current_velocity);

        let q_dot = self.current_rotation.hamilton_product(p) * 0.5;

        self.current_rotation = (*self.current_rotation + q_dot * dt).normalized();

        let global_acceleration = self.get_global_linear_acceleration()?;

        // 2-Layer integration of linear acceleration is incredibly prone to error
        // buildup, so it does not currently give any useful quality of data
        self.current_linear_position = self.current_linear_position
            + self.current_linear_velocity * dt
            + global_acceleration * dt * dt * 0.5;

        self.current_linear_velocity = self.current_linear_velocity + global_acceleration * dt;

        Ok(())
    }

    pub fn get_local_linear_acceleration(&mut self) -> Result<Vector<3, Acceleration>, E> {
        let rot_inv = (self.current_rotation.conjugate()
            / self.current_rotation.dot(*self.current_rotation))
        .normalized();

        let gravity_component = rot_inv
            .hamilton_product(Quaternion::from_direction_component(
                zero(),
                self.linear_drift,
            ))
            .hamilton_product(*self.current_rotation);

        let local_acceleration =
            self.get_linear_acceleration()? - gravity_component.direction_components();

        Ok(local_acceleration)
    }

    pub fn get_global_linear_acceleration(&mut self) -> Result<Vector<3, Acceleration>, E> {
        Ok(self
            .current_rotation
            .hamilton_product(Quaternion::from_direction_component(
                zero(),
                self.get_local_linear_acceleration()?,
            ))
            .hamilton_product(self.current_rotation.conjugate())
            .direction_components())
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
        self.device.get_linear_acceleration()
    }
}

impl<T> AngularPositionSensor for InertialSensor<T> {
    type Error = Infallible;

    fn get_angular_position(&mut self) -> Result<UnitQuaternion<Ratio>, Self::Error> {
        Ok(self.current_rotation)
    }
}

impl<T> LinearVelocitySensor for InertialSensor<T> {
    type Error = Infallible;

    fn get_linear_velocity(&mut self) -> Result<Vector<3, Velocity>, Self::Error> {
        Ok(self.current_linear_velocity)
    }
}

impl<T> LinearPositionSensor for InertialSensor<T> {
    type Error = Infallible;

    fn get_linear_position(&mut self) -> Result<Vector<3, Length>, Self::Error> {
        Ok(self.current_linear_position)
    }
}
