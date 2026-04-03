use embedded_hal::spi::SpiDevice;
use linalg::vector::Vector;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use uom::si::{
    angular_velocity::degree_per_second,
    f32::{AngularVelocity, Velocity},
    velocity::meter_per_second,
};

use crate::{G, Imu, SpiImu, declare_registers, registers::RegisterError};

#[derive(TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum AnalogBandwidth {
    _1500Hz = 0,
    _400Hz = 1,
}

#[derive(TryFromPrimitive, IntoPrimitive, Copy, Clone, Debug, Default)]
#[repr(u8)]
pub enum GyroscopeFullScaleSelection {
    #[default]
    Dps245 = 0b00,
    Dps500 = 0b01,
    Dps1000 = 0b10,
    Dps2000 = 0b11,
}

const DATA_RATE: u8 = 0b1010; // 6.66kHz

impl From<GyroscopeFullScaleSelection> for AngularRateSensorControl {
    fn from(value: GyroscopeFullScaleSelection) -> Self {
        Self {
            data_rate: DATA_RATE,
            dps125: false,
            full_scale: value,
        }
    }
}

impl GyroscopeFullScaleSelection {
    pub fn as_velocity(self) -> AngularVelocity {
        AngularVelocity::new::<degree_per_second>(match self {
            GyroscopeFullScaleSelection::Dps245 => 245.0,
            GyroscopeFullScaleSelection::Dps500 => 500.0,
            GyroscopeFullScaleSelection::Dps1000 => 1000.0,
            GyroscopeFullScaleSelection::Dps2000 => 2000.0,
        })
    }
}

#[derive(TryFromPrimitive, IntoPrimitive, Copy, Clone, Debug, Default)]
#[repr(u8)]
pub enum AccelerometerFullScaleSelection {
    #[default]
    PM2G = 0b00,
    PM16G = 0b01,
    PM4G = 0b10,
    PM8G = 0b11,
}

impl From<AccelerometerFullScaleSelection> for AccelerometerControl {
    fn from(value: AccelerometerFullScaleSelection) -> Self {
        Self {
            data_rate: DATA_RATE,
            full_scale: value,
            analog_bandwidth_selection: AnalogBandwidth::_400Hz,
            digital_bandwidth_selection: false,
        }
    }
}

impl AccelerometerFullScaleSelection {
    pub fn as_velocity(self) -> Velocity {
        Velocity::new::<meter_per_second>(match self {
            AccelerometerFullScaleSelection::PM2G => G * 2.0,
            AccelerometerFullScaleSelection::PM16G => G * 16.0,
            AccelerometerFullScaleSelection::PM4G => G * 4.0,
            AccelerometerFullScaleSelection::PM8G => G * 8.0,
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

    AccelerometerControl (0x10, 1, Read, Write) {
        #[bits(1)]
        pub analog_bandwidth_selection: AnalogBandwidth,
        pub digital_bandwidth_selection: bool,
        #[bits(2)]
        pub full_scale: AccelerometerFullScaleSelection,
        #[bits(4)]
        pub data_rate: u8,
    }

    AngularRateSensorControl (0x11, 1, Read, Write) {
        #[skip(1)]
        #[bits(1)]
        dps125: bool,
        #[bits(2)]
        full_scale: GyroscopeFullScaleSelection,
        #[bits(4)]
        data_rate: u8,
    }
}

pub struct Lsm6ds3tr<D: SpiDevice> {
    device: D,

    gyroscope_full_scale: GyroscopeFullScaleSelection,
    accelerometer_full_scale: AccelerometerFullScaleSelection,
}

impl<D: SpiDevice> Lsm6ds3tr<D> {
    pub fn new(device: D) -> Self {
        Self {
            device,

            gyroscope_full_scale: Default::default(),
            accelerometer_full_scale: Default::default(),
        }
    }

    pub fn set_gyroscope_full_scale(
        &mut self,
        full_scale: GyroscopeFullScaleSelection,
    ) -> Result<(), RegisterError<D::Error>> {
        self.gyroscope_full_scale = full_scale;

        self.write_register(AngularRateSensorControl::from(full_scale))
    }
}

impl<D: SpiDevice> SpiImu<D> for Lsm6ds3tr<D> {
    fn device(&mut self) -> &mut D {
        &mut self.device
    }
}

impl<D: SpiDevice> Imu for Lsm6ds3tr<D> {
    type Error = RegisterError<D::Error>;

    fn get_angular_velocity(
        &mut self,
    ) -> Result<Vector<3, AngularVelocity>, RegisterError<D::Error>> {
        let GyroscopeOutputAll { x, y, z } = self.read_register()?;

        let ratios = [x, y, z].map(|val| val as f32 / i32::MAX as f32);

        let full_scale_velocity = self.gyroscope_full_scale.as_velocity();

        let velocities = ratios.map(|ratio| ratio * full_scale_velocity);

        Ok(velocities.into())
    }

    fn get_linear_velocity(&mut self) -> Result<Vector<3, Velocity>, RegisterError<D::Error>> {
        let AccelerometerOutputAll { x, y, z } = self.read_register()?;

        let ratios = [x, y, z].map(|val| val as f32 / i32::MAX as f32);

        let full_scale_velocity = self.accelerometer_full_scale.as_velocity();

        let velocities = ratios.map(|ratio| ratio * full_scale_velocity);

        Ok(velocities.into())
    }
}
