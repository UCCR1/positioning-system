use embedded_hal::spi::SpiDevice;
use linalg::vector::Vector;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use uom::si::{
    angular_velocity::degree_per_second,
    f32::{Acceleration, AngularVelocity},
};

use crate::{
    AngularVelocitySensor, G, LinearAccelerationSensor, RegisterDevice, declare_registers,
    registers::RegisterError,
};

#[derive(TryFromPrimitive, IntoPrimitive, Debug)]
#[repr(u8)]
pub enum AnalogBandwidth {
    _1500Hz = 0,
    _400Hz = 1,
}

#[derive(TryFromPrimitive, IntoPrimitive, Copy, Clone, Debug, Default)]
#[repr(u8)]
pub enum GyroscopeFullScaleSelection {
    Dps125 = 0b100,
    #[default]
    Dps250 = 0b000, // Datasheet says 245 and 250 in different locations?
    Dps500 = 0b001,
    Dps1000 = 0b010,
    Dps2000 = 0b011,
}

const DATA_RATE: u8 = 0b1010; // 6.66kHz

impl From<GyroscopeFullScaleSelection> for AngularRateSensorControl {
    fn from(value: GyroscopeFullScaleSelection) -> Self {
        Self {
            data_rate: DATA_RATE,
            full_scale: value,
        }
    }
}

impl GyroscopeFullScaleSelection {
    pub fn as_velocity_per_lsb(self) -> AngularVelocity {
        AngularVelocity::new::<degree_per_second>(match self {
            Self::Dps125 => 4.375e-3,
            Self::Dps250 => 8.75e-3,
            Self::Dps500 => 17.5e-3,
            Self::Dps1000 => 35e-3,
            Self::Dps2000 => 70e-3,
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
    pub fn as_acceleration_per_lsb(self) -> Acceleration {
        match self {
            Self::PM2G => G * 0.061e-3,
            Self::PM16G => G * 0.122e-3,
            Self::PM4G => G * 0.244e-3,
            Self::PM8G => G * 0.488e-3,
        }
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
        #[bits(3)]
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

impl<D: SpiDevice> RegisterDevice<D> for Lsm6ds3tr<D> {
    fn device(&mut self) -> &mut D {
        &mut self.device
    }
}

impl<D: SpiDevice> AngularVelocitySensor for Lsm6ds3tr<D> {
    type Error = RegisterError<D::Error>;

    fn get_angular_velocity(
        &mut self,
    ) -> Result<Vector<3, AngularVelocity>, RegisterError<D::Error>> {
        let GyroscopeOutputAll { x, y, z } = self.read_register()?;

        let gyroscope_sensitivity = self.gyroscope_full_scale.as_velocity_per_lsb();

        let velocities = [x, y, z].map(|val| val as f32 * gyroscope_sensitivity);

        Ok(Vector::from_array(velocities))
    }
}

impl<D: SpiDevice> LinearAccelerationSensor for Lsm6ds3tr<D> {
    type Error = RegisterError<D::Error>;

    fn get_linear_acceleration(
        &mut self,
    ) -> Result<Vector<3, Acceleration>, RegisterError<D::Error>> {
        let AccelerometerOutputAll { x, y, z } = self.read_register()?;

        let accelerometer_sensitivity = self.accelerometer_full_scale.as_acceleration_per_lsb();

        let accelerations = [x, y, z].map(|val| val as f32 * accelerometer_sensitivity);

        Ok(Vector::from_array(accelerations))
    }
}
