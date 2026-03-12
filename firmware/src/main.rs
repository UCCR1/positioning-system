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

use position_lib::{linalg::vector::real::UnitVector, odometry::TrackingWheel, real_vector};
use uom::si::{
    angle::degree,
    f32::{Angle, Length},
    length::inch,
};

use crate::odometry::OdometryTask;

#[macro_use]
mod encoder;
mod odometry;

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

    let mut odometry = OdometryTask::new(
        [left_encoder, right_encoder],
        [
            TrackingWheel {
                direction: UnitVector::from_angle(Angle::new::<degree>(45.0)),
                location: real_vector!(Length::millimeter, -20.0, 20.0), // TODO: Determine this
            },
            TrackingWheel {
                direction: UnitVector::from_angle(Angle::new::<degree>(45.0)),
                location: real_vector!(Length::millimeter, 20.0, 20.0), // TODO: Determine this
            },
        ],
        Length::new::<inch>(2.0),
    );

    loop {
        let delay_start = Instant::now();

        odometry.update();

        while delay_start.elapsed() < Duration::from_millis(10) {}
    }
}
