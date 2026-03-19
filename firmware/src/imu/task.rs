use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Instant;
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{Blocking, gpio::Output, spi::master::Spi};
use linalg::vector::Vector;
use num_traits::{ConstZero, Zero};
use uom::si::{
    f32::{Angle, AngularVelocity, Time},
    time::microsecond,
};

use crate::imu::{Imu, RegisterError};

pub static HEADING_SIGNAL: Signal<CriticalSectionRawMutex, Angle> = Signal::new();

async fn imu_task<'a, D: SpiDevice>(mut imu: Imu<'a, D>) -> Result<(), RegisterError<D::Error>> {
    let mut angle = Angle::ZERO;

    let mut last_velocity: Vector<3, AngularVelocity> = Vector::zero();
    let mut last_time = Instant::now();

    loop {
        imu.wait_for_data().await;

        let current_velocity = imu.get_angular_velocity()?;
        let current_time = Instant::now();

        let dt = Time::new::<microsecond>((current_time - last_time).as_micros() as f32);
        last_time = current_time;

        let avg_y: AngularVelocity = (last_velocity + current_velocity).y() * 0.5;

        angle += (avg_y * dt).into();

        last_velocity = current_velocity;

        HEADING_SIGNAL.signal(angle);
    }
}

#[embassy_executor::task]
pub async fn start_imu_task(
    imu: Imu<'static, ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, NoDelay>>,
) {
    imu_task(imu).await.unwrap();
}
