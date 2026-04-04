#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    spi::master::{self as spi},
    timer::timg::TimerGroup,
};
use esp_println::println;
use imu_lib::modules::lsm6ds3tr::Lsm6ds3tr;
use linalg::{real_vector, vector::real::UnitVector};
use position_lib::odometry::TrackingWheel;
use uom::si::{
    angle::degree,
    f32::{Angle, Length},
    length::inch,
};

use crate::{
    imu::start_imu_task,
    lidar::Lidar,
    odometry::{OdometryTask, start_odometry_task},
};

#[macro_use]
mod encoder;
mod imu;
mod lidar;
mod odometry;

#[panic_handler]
fn panic(e: &core::panic::PanicInfo) -> ! {
    println!("{e:?}");
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);

    let _peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(_peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    declare_encoders!(encoder_module, [0, 1]);

    let (left_encoder, right_encoder) = encoder_module::init(
        _peripherals.PCNT,
        (_peripherals.GPIO13.into(), _peripherals.GPIO35.into()),
        (_peripherals.GPIO14.into(), _peripherals.GPIO27.into()),
    );

    let spi_config = spi::Config::default();
    let spi = spi::Spi::new(_peripherals.SPI2, spi_config)
        .unwrap()
        .with_miso(_peripherals.GPIO33)
        .with_mosi(_peripherals.GPIO32)
        .with_sck(_peripherals.GPIO25);

    let cs_config = OutputConfig::default();

    let spi_device = ExclusiveDevice::new(
        spi,
        Output::new(_peripherals.GPIO26, Level::High, cs_config),
        NoDelay,
    )
    .unwrap();

    let interrupts_config = InputConfig::default().with_pull(Pull::None);

    let int1 = Input::new(_peripherals.GPIO34, interrupts_config);

    spawner
        .spawn(start_imu_task(Lsm6ds3tr::new(spi_device), int1))
        .unwrap();

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
        _peripherals.GPIO5.into(),
    );

    spawner.spawn(start_odometry_task(odometry)).unwrap();

    loop {
        Timer::after_millis(20).await;

        lidar.update().unwrap();
    }
}
