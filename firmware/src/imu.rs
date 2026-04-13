use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{
    Blocking,
    gpio::{Input, Output},
    spi::master::Spi,
};
use imu_lib::{
    AngularPositionSensor, LinearAccelerationSensor,
    devices::{
        RegisterError,
        lsm6dsv::{
            AccelerometerDataRate, AccelerometerFullScaleSelection, GyroscopeDataRate,
            GyroscopeFullScaleSelection, Lsm6dsv,
        },
    },
    inertial_sensor::{InertialSensor, InertialSensorError},
};
use linalg::vector::Vector;
use uom::si::{
    f32::{Acceleration, Angle, Frequency},
    frequency::hertz,
};

pub static HEADING_SIGNAL: Signal<CriticalSectionRawMutex, Angle> = Signal::new();
pub static ACCELERATION_SIGNAL: Signal<CriticalSectionRawMutex, Vector<3, Acceleration>> =
    Signal::new();

async fn imu_task<'a, D: SpiDevice>(
    mut imu: Lsm6dsv<D>,
    mut int1: Input<'static>,
) -> Result<(), RegisterError<D::Error>> {
    imu.set_gyroscope_data_rate(GyroscopeDataRate::_240Hz)?;
    imu.set_accelerometer_data_rate(AccelerometerDataRate::_240Hz)?;

    imu.set_accelerometer_full_scale(AccelerometerFullScaleSelection::PM16G)?;
    imu.set_gyroscope_full_scale(GyroscopeFullScaleSelection::Dps2000)?;

    imu.enable_interrupts(true)?;
    imu.set_data_ready_interrupts_int1(true, true)?;

    let mut inertial_sensor = InertialSensor::new(imu);

    let dt = 1.0 / Frequency::new::<hertz>(240.0);

    loop {
        int1.wait_for_high().await;

        let status_register = inertial_sensor.device().get_status_register()?;

        if !status_register.gyroscope_data_ready || !status_register.accelerometer_data_ready {
            continue;
        }

        if !inertial_sensor.calibration_complete() {
            inertial_sensor.calibrate()?;

            continue;
        }

        match inertial_sensor.integrate(dt) {
            Err(InertialSensorError::IncompleteCalibration) => {
                inertial_sensor.calibrate()?;

                continue;
            }
            Err(InertialSensorError::RegisterError(e)) => {
                return Err(e);
            }
            _ => {}
        }

        let Ok(angular_position) = inertial_sensor.get_angular_position();

        HEADING_SIGNAL.signal(angular_position.yaw());

        ACCELERATION_SIGNAL.signal(inertial_sensor.get_linear_acceleration()?);
    }
}

#[embassy_executor::task]
pub async fn start_imu_task(
    imu: Lsm6dsv<ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, NoDelay>>,
    int1: Input<'static>,
) {
    imu_task(imu, int1).await.unwrap();
}
