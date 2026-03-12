#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_hal::{
    clock::CpuClock,
    main,
    time::{Duration, Instant},
};

use position_lib::{
    linalg::vector::real::UnitVector,
    odometry::{Odometry, TrackingWheel},
    real_vector,
};
use uom::{
    ConstZero,
    si::{
        angle::degree,
        f32::{Angle, Length},
        length::inch,
    },
};

#[macro_use]
mod encoder;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.2.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let _peripherals = esp_hal::init(config);

    declare_encoders!(encoder_module, [0, 1]);

    let (left_encoder, right_encoder) = encoder_module::init(
        _peripherals.PCNT,
        (_peripherals.GPIO14.into(), _peripherals.GPIO15.into()),
        (_peripherals.GPIO2.into(), _peripherals.GPIO3.into()),
    );

    let mut odometry = Odometry::new([
        TrackingWheel {
            direction: UnitVector::from_angle(Angle::new::<degree>(45.0)),
            location: real_vector!(Length::millimeter, -20.0, 20.0), // TODO: Determine this
        },
        TrackingWheel {
            direction: UnitVector::from_angle(Angle::new::<degree>(45.0)),
            location: real_vector!(Length::millimeter, 20.0, 20.0), // TODO: Determine this
        },
    ]);

    let wheel_radius = Length::new::<inch>(1.0);

    let mut last_left_position = Angle::ZERO;
    let mut last_right_position = Angle::ZERO;

    loop {
        let delay_start = Instant::now();

        let new_left_position = left_encoder.position();
        let new_right_position = right_encoder.position();

        let left_travel = (new_left_position - last_left_position) * wheel_radius;
        let right_travel = (new_right_position - last_right_position) * wheel_radius;

        odometry.update([left_travel, right_travel], Angle::ZERO);

        last_left_position = new_left_position;
        last_right_position = new_right_position;

        while delay_start.elapsed() < Duration::from_millis(10) {}
    }
}
