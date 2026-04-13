use linalg::{
    quaternion::{Quaternion, UnitQuaternion},
    vector::Vector,
};
use uom::{
    num_traits::Zero,
    si::f32::{AngularVelocity, Ratio, Time},
};

use crate::{AngularPositionSensor, AngularVelocitySensor};

pub struct AngularVelocityIntegrator<T> {
    inner: T,

    current_rotation: UnitQuaternion<Ratio>,
}

impl<T> AngularVelocityIntegrator<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner,
            current_rotation: UnitQuaternion::identity(),
        }
    }

    pub fn inner(&mut self) -> &mut T {
        &mut self.inner
    }
}

impl<T: AngularVelocitySensor> AngularVelocityIntegrator<T> {
    pub fn poll(&mut self, dt: Time) -> Result<(), T::Error> {
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

impl<T: AngularVelocitySensor> AngularVelocitySensor for AngularVelocityIntegrator<T> {
    type Error = T::Error;

    fn get_angular_velocity(&mut self) -> Result<Vector<3, AngularVelocity>, Self::Error> {
        self.inner.get_angular_velocity()
    }
}

impl<T: AngularVelocitySensor> AngularPositionSensor for AngularVelocityIntegrator<T> {
    type Error = T::Error;

    fn get_angular_position(&mut self) -> Result<UnitQuaternion<Ratio>, Self::Error> {
        Ok(self.current_rotation)
    }
}
