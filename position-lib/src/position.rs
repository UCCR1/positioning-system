extern crate alloc;
use crate::{
    sensor::DistSensor,
    odometry::Odometry,
};
use alloc::boxed::Box;
use heapless::Vec;
use kalman_filters::ParticleFilter;

struct Position<const N: usize> {
    sensors: Vec<Box<dyn DistSensor>, 0>,
    odom: Odometry<N>,
    particle_filter: ParticleFilter<f32>,
}

impl<const N: usize> Position<N> { 
}
