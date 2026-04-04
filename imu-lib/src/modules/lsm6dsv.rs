use embedded_hal::spi::SpiDevice;
use linalg::vector::Vector;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use uom::si::{
    angular_velocity::degree_per_second,
    f32::{AngularVelocity, Velocity},
    velocity::meter_per_second,
};

use crate::{G, Imu, SpiImu, declare_registers, registers::RegisterError};


#[derive(TryFromPrimitive, IntoPrimitive, Copy, Clone, Debug, Default)]
#[repr(u8)]
pub enum OperatingMode {
    #[default]
    HighPerformance = 0b000,
    HighAccuracyODR = 0b001,
    ODRTriggered = 0b011,
    SleepMode = 0b100,
    LowPowerMode = 0b101,
}

#[derive(TryFromPrimitive, IntoPrimitive, Copy, Clone, Debug, Default)]
#[repr(u8)]
pub enum GyroscopeFullScaleSelection {
    #[default]
    Dps125 = 0b0000,
    Dps250 = 0b0001,
    Dps500 = 0b0010,
    Dps1000 = 0b0011,
    Dps2000 = 0b0100,
    Dps4000 = 0b1100,
}

impl GyroscopeFullScaleSelection {
    pub fn as_velocity(self) -> AngularVelocity {
        AngularVelocity::new::<degree_per_second>(match self {
            Self::Dps125 => 125.0,
            Self::Dps250 => 250.0,
            Self::Dps500 => 500.0,
            Self::Dps1000 => 1000.0,
            Self::Dps2000 => 2000.0,
            Self::Dps4000 => 4000.0,
        })
    }
}

#[derive(TryFromPrimitive, IntoPrimitive, Copy, Clone, Debug, Default)]
#[repr(u8)]
pub enum AccelerometerFullScaleSelection {
    #[default]
    PM2G = 0b00,
    PM4G = 0b01,
    PM8G = 0b10,
    PM16G = 0b11,
}

impl AccelerometerFullScaleSelection {
    pub fn as_velocity(self) -> Velocity {
        Velocity::new::<meter_per_second>(match self {
            AccelerometerFullScaleSelection::PM2G => G * 2.0,
            AccelerometerFullScaleSelection::PM4G => G * 4.0,
            AccelerometerFullScaleSelection::PM8G => G * 8.0,
            AccelerometerFullScaleSelection::PM16G => G * 16.0,
        })
    }
}

declare_registers! {
    GyroscopeOutputAll (0x22, 6, Read) {
        pub x: i16,
        pub y: i16,
        pub z: i16,
    }

    AccelerometerOutputAll (0x28, 6, Read) {
        pub x: i16,
        pub y: i16,
        pub z: i16,
    }

    GyroscopeControl (0x11, 1, Read, Write) {
        #[bits(4)]
        data_rate: u8,
        #[bits(3)]
        operating_mode: OperatingMode,
    }
}

pub struct Lsm6dsv<D: SpiDevice> {
    device: D,

    gyroscope_full_scale: GyroscopeFullScaleSelection,
    accelerometer_full_scale: AccelerometerFullScaleSelection,
}

impl<D: SpiDevice> Lsm6dsv<D> {
    pub fn new(mut device: D) -> Self {
        crate::registers::WriteRegister::write(GyroscopeControl {
            data_rate: 0b1100, // 7.68kHz,
            operating_mode: OperatingMode::HighPerformance,
        }, &mut device).unwrap();

        Self {
            device,

            gyroscope_full_scale: Default::default(),
            accelerometer_full_scale: Default::default(),
        }
    }
}

impl<D: SpiDevice> SpiImu<D> for Lsm6dsv<D> {
    fn device(&mut self) -> &mut D {
        &mut self.device
    }
}

impl<D: SpiDevice> Imu for Lsm6dsv<D> {
    type Error = RegisterError<D::Error>;

    fn get_angular_velocity(
        &mut self,
    ) -> Result<Vector<3, AngularVelocity>, RegisterError<D::Error>> {
        let GyroscopeOutputAll { x, y, z } = self.read_register()?;

        let ratios = [x, y, z].map(|val| val as f32 / i16::MAX as f32);

        let full_scale_velocity = self.gyroscope_full_scale.as_velocity();

        let velocities = ratios.map(|ratio| ratio * full_scale_velocity);

        Ok(Vector::from_array(velocities))
    }

    fn get_linear_velocity(&mut self) -> Result<Vector<3, Velocity>, RegisterError<D::Error>> {
        let AccelerometerOutputAll { x, y, z } = self.read_register()?;

        let ratios = [x, y, z].map(|val| val as f32 / i32::MAX as f32);

        let full_scale_velocity = self.accelerometer_full_scale.as_velocity();

        let velocities = ratios.map(|ratio| ratio * full_scale_velocity);

        Ok(Vector::from_array(velocities))
    }
}
