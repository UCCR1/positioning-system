#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_hal::{
    clock::CpuClock,
    time::{Duration, Instant},
};
use linalg::{real_vector, vector::real::UnitVector};
use position_lib::odometry::TrackingWheel;
use uom::si::{
    angle::degree,
    f32::{Angle, Length},
    length::inch,
};

use crate::{
    imu::{Imu, task::start_imu_task},
    lidar::Lidar,
    odometry::{OdometryTask, start_odometry_task},
};

#[macro_use]
mod encoder;
mod imu;
mod lidar;
mod odometry;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz);
    let _peripherals = esp_hal::init(config);

    declare_encoders!(encoder_module, [0, 1]);

    let (left_encoder, right_encoder) = encoder_module::init(
        _peripherals.PCNT,
        (_peripherals.GPIO14.into(), _peripherals.GPIO15.into()),
        (_peripherals.GPIO17.into(), _peripherals.GPIO3.into()),
    );

    let imu = Imu::new(
        _peripherals.SPI2.into(),
        _peripherals.GPIO12.into(),
        _peripherals.GPIO19.into(),
        _peripherals.GPIO13.into(),
        _peripherals.GPIO21.into(),
        _peripherals.GPIO1.into(),
        _peripherals.GPIO2.into(),
    );

    let odometry = OdometryTask::new(
        [left_encoder, right_encoder],
        [
            TrackingWheel {
                direction: UnitVector::from_angle(Angle::new::<degree>(45.0)),
                location: real_vector!(Length::millimeter, -20.0, 20.0),
            },
            TrackingWheel {
                direction: UnitVector::from_angle(Angle::new::<degree>(45.0)),
                location: real_vector!(Length::millimeter, 20.0, 20.0),
            },
        ],
        Length::new::<inch>(2.0),
    );

    let mut lidar = Lidar::new(
        _peripherals.UART1.into(),
        _peripherals.GPIO18.into(),
        _peripherals.GPIO4.into(),
    );

    spawner.spawn(start_imu_task(imu)).unwrap();
    spawner.spawn(start_odometry_task(odometry)).unwrap();

    loop {
        let delay_start = Instant::now();

        lidar.update();

        while delay_start.elapsed() < Duration::from_millis(10) {}
    }
}
