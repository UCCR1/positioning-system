use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{
    Blocking,
    gpio::{Input, Output},
    spi::master::Spi,
};
use imu_lib::{
    CalibratableImu,
    modules::lsm6dsv::{GyroscopeDataRate, GyroscopeFullScaleSelection, Lsm6dsv},
    registers::RegisterError,
};
use linalg::quaternion::{Quaternion, UnitQuaternion};
use num_traits::Zero;
use uom::si::f32::{Angle, AngularVelocity, Frequency, Ratio};

pub static HEADING_SIGNAL: Signal<CriticalSectionRawMutex, Angle> = Signal::new();

const IMU_FREQUENCY: GyroscopeDataRate = GyroscopeDataRate::_240Hz;

const NUM_CALIBRATION_SAMPLES: usize = 500;

async fn imu_task<'a, D: SpiDevice>(
    imu: Lsm6dsv<D>,
    mut int1: Input<'static>,
) -> Result<(), RegisterError<D::Error>> {
    let mut current_rotation: UnitQuaternion<Ratio> = UnitQuaternion::identity();

    let mut imu = CalibratableImu::<NUM_CALIBRATION_SAMPLES, _>::new(imu);

    imu.set_gyroscope_data_rate(IMU_FREQUENCY)?;
    imu.set_gyroscope_full_scale(GyroscopeFullScaleSelection::Dps2000)?;
    imu.enable_interrupts(true)?;
    imu.set_data_ready_interrupts_int1(true, false)?;

    let dt = 1.0 / Frequency::from(IMU_FREQUENCY);

    loop {
        int1.wait_for_high().await;

        if !imu.gyroscope_data_ready()? {
            continue;
        }

        if !imu.calibration_complete() {
            imu.calibrate()?;

            continue;
        }

        let current_velocity = imu.get_calibrated_angular_velocity()?;

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
