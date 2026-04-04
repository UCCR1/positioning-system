use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Instant;
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{
    Blocking,
    gpio::{Input, Output},
    spi::master::Spi,
};
use esp_println::println;
use imu_lib::{Imu, modules::lsm6dsv::Lsm6dsv, registers::RegisterError};
use linalg::{
    quaternion::{Quaternion, UnitQuaternion},
    vector::Vector,
};
use num_traits::Zero;
use uom::si::{
    angle::{degree, radian},
    angular_velocity::degree_per_second,
    f32::{Angle, AngularVelocity, Ratio, Time},
    time::microsecond,
};

pub static HEADING_SIGNAL: Signal<CriticalSectionRawMutex, Angle> = Signal::new();

async fn imu_task<'a, D: SpiDevice>(
    mut imu: Lsm6dsv<D>,
    mut int1: Input<'static>,
) -> Result<(), RegisterError<D::Error>> {
    let mut current_rotation: UnitQuaternion<Ratio> = UnitQuaternion::identity();

    let mut last_time = Instant::now();

    let mut total_samples = 0;
    let mut average_drift: Vector<3, AngularVelocity> = Vector::zero();

    let mut i = 0;

    loop {
        // int1.wait_for_rising_edge().await;

        embassy_time::Timer::after_millis(5).await;

        let mut current_velocity = imu.get_angular_velocity()?;

        let current_time = Instant::now();

        // TODO: Use actual calibration
        if total_samples < 1000 {
            average_drift = (average_drift * total_samples as f32 + current_velocity)
                / ((total_samples + 1) as f32);
            total_samples += 1;

            current_velocity = Vector::zero();
        } else {
            current_velocity -= average_drift;
        }

        let dt = Time::new::<microsecond>((current_time - last_time).as_micros() as f32);
        last_time = current_time;

        let p = Quaternion::from_array([
            AngularVelocity::zero(),
            current_velocity.x(),
            current_velocity.y(),
            current_velocity.z(),
        ]);

        let q_dot = current_rotation.hamilton_product(p) * 0.5;

        current_rotation = (*current_rotation + (q_dot * dt).into()).normalized();

        if i > 10 {
            // println!("{current_velocity:?}");
            println!("Yaw: {:.3} deg", current_rotation.yaw().get::<degree>());
            i = 0;
        } else {
            i += 1;
        }

        HEADING_SIGNAL.signal(current_rotation.yaw());
    }
}

#[embassy_executor::task]
pub async fn start_imu_task(
    imu: Lsm6dsv<ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, NoDelay>>,
    int1: Input<'static>,
) {
    imu_task(imu, int1).await.unwrap();
}
