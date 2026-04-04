use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Instant;
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{
    Blocking,
    gpio::{Input, Output},
    spi::master::Spi,
};
use imu_lib::{Imu, modules::lsm6dsv::Lsm6dsv, registers::RegisterError};
use linalg::{quaternion::{Quaternion, UnitQuaternion}, vector::Vector};
use num_traits::{Zero};
use uom::si::{
    angle::{degree, radian}, angular_velocity::degree_per_second, f32::{Angle, AngularVelocity, Ratio, Time}, time::microsecond
};

pub static HEADING_SIGNAL: Signal<CriticalSectionRawMutex, Angle> = Signal::new();

async fn imu_task<'a, D: SpiDevice>(
    mut imu: Lsm6dsv<D>,
    mut int1: Input<'static>,
) -> Result<(), RegisterError<D::Error>> {
    let mut current_quat: UnitQuaternion<Ratio> = UnitQuaternion::identity();

    let mut last_time = Instant::now();

    let mut i = 0;
    loop {
        // int1.wait_for_rising_edge().await;

        embassy_time::Timer::after_millis(5).await;

        let mut current_velocity = imu.get_angular_velocity()?;
        let current_time = Instant::now();

        // TODO: Use actual calibration
        current_velocity[2][0] -= AngularVelocity::new::<degree_per_second>(-0.422830138554562);

        let dt = Time::new::<microsecond>((current_time - last_time).as_micros() as f32);
        last_time = current_time;

        let angle: Vector<3, Angle
        > = (current_velocity * dt * 0.5).into();

        let dq = Quaternion::from_array([Angle::new::<radian>(1.0), angle.x(), angle.y(), angle.z()]);

        current_quat = current_quat.hamilton_product(dq).normalized();

        if (i >= 20) {
            i = 0;

            esp_println::println!("theta: {} deg", current_quat.yaw().get::<degree>())
        }  else {
            i += 1;
        }

        // HEADING_SIGNAL.signal(angle);
    }
}

#[embassy_executor::task]
pub async fn start_imu_task(
    imu: Lsm6dsv<ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, NoDelay>>,
    int1: Input<'static>,
) {
    imu_task(imu, int1).await.unwrap();
}
