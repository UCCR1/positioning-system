use linalg::{line::Line, vector::Vector};
use uom::si::f32::{Length, ReciprocalLength};

use crate::{
    Pose,
    probability::{exponential, normal},
};

pub struct BeamProbabilityParameters {
    standard_deviation: Length,
    hit_weighting: f32,

    short_falloff: ReciprocalLength,
    short_weighting: f32,
}

pub struct LidarSensor<const N: usize> {
    relative_pose: Pose,

    beam_parameters: BeamProbabilityParameters,

    static_obstacles: [Line<2, Length>; N],
}

impl<const N: usize> LidarSensor<N> {
    fn beam_probability(&self, actual: Length, expected: Length) -> f32 {
        let p_hit = normal(
            expected.value,
            actual.value,
            self.beam_parameters.standard_deviation.value,
        );

        let p_short = if actual < expected {
            exponential(actual.value, self.beam_parameters.short_falloff.value)
        } else {
            0.0
        };

        self.beam_parameters.hit_weighting * p_hit + self.beam_parameters.short_weighting * p_short
    }

    fn raycast(
        &self,
        center: Vector<2, Length>,
        ray: Vector<2, Length>,
    ) -> Option<Vector<2, Length>> {
        let sensor_lineseg = Line(center, center + ray);

        self.static_obstacles
            .into_iter()
            .filter_map(|seg| seg.intersection(sensor_lineseg))
            .min_by_key(|intersection| Line(*intersection, center).length().value.to_bits())
    }

    pub fn particle_weighting(&self, pose: Pose, samples: &[Vector<2, Length>]) -> f32 {
        let center = self.relative_pose.position + pose.position;

        samples
            .into_iter()
            .map(|sample| {
                let ray = sample.rotate(-(self.relative_pose.heading + pose.heading)) * 1000.0;

                let Some(expected) = self.raycast(center, ray) else {
                    return 0.0;
                };

                self.beam_probability(sample.magnitude(), (expected - center).magnitude())
            })
            .product()
    }
}
