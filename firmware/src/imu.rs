use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{
    Blocking,
    gpio::{Input, Output},
    spi::master::Spi,
};
use esp_println::println;
use imu_lib::{
    AngularPositionSensor, LinearAccelerationSensor,
    calibrateable::CalibratableImu,
    integrator::AngularVelocityIntegrator,
    modules::lsm6dsv::{GyroscopeDataRate, GyroscopeFullScaleSelection, Lsm6dsv},
    registers::RegisterError,
};
use uom::si::f32::{Angle, Frequency};

pub static HEADING_SIGNAL: Signal<CriticalSectionRawMutex, Angle> = Signal::new();

const IMU_FREQUENCY: GyroscopeDataRate = GyroscopeDataRate::_240Hz;

const NUM_CALIBRATION_SAMPLES: usize = 500;

async fn imu_task<'a, D: SpiDevice>(
    mut imu: Lsm6dsv<D>,
    mut int1: Input<'static>,
) -> Result<(), RegisterError<D::Error>> {
    imu.set_gyroscope_data_rate(IMU_FREQUENCY)?;
    imu.set_gyroscope_full_scale(GyroscopeFullScaleSelection::Dps2000)?;
    imu.enable_interrupts(true)?;
    imu.set_data_ready_interrupts_int1(true, false)?;

    let mut integrating_imu =
        AngularVelocityIntegrator::new(CalibratableImu::<NUM_CALIBRATION_SAMPLES, _>::new(imu));

    let dt = 1.0 / Frequency::from(IMU_FREQUENCY);

    loop {
        int1.wait_for_high().await;

        if !integrating_imu.base().base().gyroscope_data_ready()? {
            continue;
        }

        if !integrating_imu.base().calibration_complete() {
            integrating_imu.base().calibrate()?;

            continue;
        }

        integrating_imu.poll(dt)?;

        HEADING_SIGNAL.signal(integrating_imu.get_angular_position()?.yaw());
    }
}

#[embassy_executor::task]
pub async fn start_imu_task(
    imu: Lsm6dsv<ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, NoDelay>>,
    int1: Input<'static>,
) {
    imu_task(imu, int1).await.unwrap();
}
