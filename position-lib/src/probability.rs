use core::f32::consts::PI;

use uom::num_traits::Float;

pub(crate) fn normal(x: f32, mean: f32, std_dev: f32) -> f32 {
    let variance = std_dev.powi(2);

    let exponent = (x - mean).powi(2) / (2.0 * variance);
    let likelyhood = (1.0 / (2.0 * PI * variance).sqrt()) * exponent.exp();

    likelyhood
}

pub(crate) fn exponential(x: f32, lambda: f32) -> f32 {
    lambda * (-x * lambda).exp()
}
