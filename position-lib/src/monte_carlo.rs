use core::array;

use rand::{Rng, RngExt};

use crate::{Pose, PoseNoiseParameters};

struct MonteCarloLocalizer<const N: usize> {
    particles: [(Pose, f32); N],
}

impl<const N: usize> MonteCarloLocalizer<N> {
    pub fn new<R: Rng>(
        starting_pose: Pose,
        starting_uncertainty: PoseNoiseParameters,
        rng: &mut R,
    ) -> Self {
        Self {
            particles: array::from_fn(|_| {
                (
                    starting_pose.with_noise(starting_uncertainty, rng),
                    1.0 / N as f32,
                )
            }),
        }
    }

    pub fn predict(
        &mut self,
        motion_model: impl Fn(Pose) -> Pose,
        motion_noise_parameters: PoseNoiseParameters,
        rng: &mut impl Rng,
    ) {
        for (pose, _) in &mut self.particles {
            *pose = motion_model(*pose).with_noise(motion_noise_parameters, rng);
        }
    }

    pub fn update(&mut self, pose_weight: impl Fn(Pose) -> f32, rng: &mut impl Rng) {
        let mut total_weight = 0.0;

        for (pose, weight) in &mut self.particles {
            *weight = pose_weight(*pose);

            total_weight += *weight;
        }

        // Normalize weights so they sum to 1
        for (_, weight) in &mut self.particles {
            *weight = *weight / total_weight;
        }

        // Resample using systematic resampling

        let mut cummulative_sum = self.particles.map(|(_, weight)| weight);

        for i in 1..N {
            cummulative_sum[i] += cummulative_sum[i - 1];
        }

        let step = 1.0 / N as f32;

        let offset = rng.random::<f32>() / N as f32;

        let mut j = 0;

        self.particles = array::from_fn(|i| {
            let u = offset + step * i as f32;

            while j < N - 1 && cummulative_sum[j] < u {
                j += 1;
            }

            (self.particles[j].0, step)
        });
    }
}
