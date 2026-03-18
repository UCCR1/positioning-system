use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{Blocking, gpio::Output, spi::master::Spi};
use uom::si::{angle::degree, f32::Angle};

use crate::imu::{Imu, RegisterError, registers::GyroscopeOutputAll};

pub static HEADING_SIGNAL: Signal<CriticalSectionRawMutex, Angle> = Signal::new();

async fn imu_task<'a, D: SpiDevice>(mut imu: Imu<'a, D>) -> Result<(), RegisterError<D::Error>> {
    loop {
        imu.wait_for_data().await;

        let data: GyroscopeOutputAll = imu.read()?;

        let heading = Angle::new::<degree>(1.0); // TODO: Integrate gyroscope velocities to find heading

        HEADING_SIGNAL.signal(heading);
    }
}

#[embassy_executor::task]
pub async fn start_imu_task(
    imu: Imu<'static, ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, NoDelay>>,
) {
    imu_task(imu).await.unwrap();
}
