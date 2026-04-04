use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{
    Blocking,
    gpio::{Input, Output},
    spi::master::Spi,
};
use imu_lib::{Imu, modules::lsm6dsv::Lsm6dsv, registers::RegisterError};
use linalg::{
    quaternion::{Quaternion, UnitQuaternion},
    vector::Vector,
};
use num_traits::Zero;
use uom::si::f32::{Angle, AngularVelocity, Frequency, Ratio};

pub static HEADING_SIGNAL: Signal<CriticalSectionRawMutex, Angle> = Signal::new();

const IMU_FREQUENCY: imu_lib::modules::lsm6dsv::GyroscopeDataRate =
    imu_lib::modules::lsm6dsv::GyroscopeDataRate::_240Hz;

async fn imu_task<'a, D: SpiDevice>(
    mut imu: Lsm6dsv<D>,
    mut int1: Input<'static>,
) -> Result<(), RegisterError<D::Error>> {
    let mut current_rotation: UnitQuaternion<Ratio> = UnitQuaternion::identity();

    let mut total_samples = 0;
    let mut average_drift: Vector<3, AngularVelocity> = Vector::zero();

    imu.set_gyroscope_data_rate(IMU_FREQUENCY)?;
    imu.set_gyroscope_full_scale(imu_lib::modules::lsm6dsv::GyroscopeFullScaleSelection::Dps1000)?;
    imu.enable_interrupts(true)?;
    imu.set_data_ready_interrupts_int1(true, false)?;

    let dt = 1.0 / Frequency::from(IMU_FREQUENCY);

    loop {
        int1.wait_for_high().await;

        if !imu.gyroscope_data_ready()? {
            continue;
        }

        let mut current_velocity = imu.get_angular_velocity()?;

        // TODO: Use actual calibration
        if total_samples < 1000 {
            average_drift = (average_drift * total_samples as f32 + current_velocity)
                / ((total_samples + 1) as f32);
            total_samples += 1;

            current_velocity = Vector::zero();
        } else {
            current_velocity -= average_drift;
        }

        let p = Quaternion::from_array([
            AngularVelocity::zero(),
            current_velocity.x(),
            current_velocity.y(),
            current_velocity.z(),
        ]);

        let q_dot = current_rotation.hamilton_product(p) * 0.5;

        current_rotation = (*current_rotation + q_dot * dt).normalized();

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
