#![no_std]

use core::ops::Neg;

use num_traits::Zero;

pub mod bounds;
pub mod line;
pub mod matrix;
pub mod vector;

pub fn abs<T: PartialOrd + Zero + Neg<Output = T>>(val: T) -> T {
    if val >= T::zero() { val } else { -val }
}

pub extern crate paste;
