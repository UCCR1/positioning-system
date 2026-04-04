use embedded_hal::spi::SpiDevice;
use linalg::vector::Vector;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use uom::si::{
    angular_velocity::degree_per_second,
    f32::{Acceleration, AngularVelocity, Frequency},
    frequency::hertz,
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
pub enum GyroscopeDataRate {
    #[default]
    Off = 0b0000,
    _7_5Hz = 0b0010,
    _15Hz = 0b0011,
    _30Hz = 0b0100,
    _60Hz = 0b0101,
    _120Hz = 0b0110,
    _240Hz = 0b0111,
    _480Hz = 0b1000,
    _960Hz = 0b1001,
    _1_92Khz = 0b1010,
    _3_84Khz = 0b1011,
    _7_68Khz = 0b1100,
}

impl From<GyroscopeDataRate> for Frequency {
    fn from(value: GyroscopeDataRate) -> Self {
        Self::new::<hertz>(match value {
            GyroscopeDataRate::Off => 0.0,
            GyroscopeDataRate::_7_5Hz => 7.5,
            GyroscopeDataRate::_15Hz => 15.0,
            GyroscopeDataRate::_30Hz => 30.0,
            GyroscopeDataRate::_60Hz => 60.0,
            GyroscopeDataRate::_120Hz => 120.0,
            GyroscopeDataRate::_240Hz => 240.0,
            GyroscopeDataRate::_480Hz => 480.0,
            GyroscopeDataRate::_960Hz => 960.0,
            GyroscopeDataRate::_1_92Khz => 1920.0,
            GyroscopeDataRate::_3_84Khz => 3840.0,
            GyroscopeDataRate::_7_68Khz => 7680.0,
        })
    }
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
    pub fn as_velocity_per_lsb(self) -> AngularVelocity {
        AngularVelocity::new::<degree_per_second>(match self {
            Self::Dps125 => 4.375e-3,
            Self::Dps250 => 8.75e-3,
            Self::Dps500 => 17.5e-3,
            Self::Dps1000 => 35e-3,
            Self::Dps2000 => 70e-3,
            Self::Dps4000 => 140e-3,
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
    pub fn as_acceleration_per_lsb(self) -> Acceleration {
        match self {
            Self::PM2G => G * 0.061e-3,
            Self::PM4G => G * 0.122e-3,
            Self::PM8G => G * 0.244e-3,
            Self::PM16G => G * 0.488e-3,
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

    GyroscopeControlRegister2 (0x11, 1, Read, Write) {
        #[bits(4)]
        data_rate: GyroscopeDataRate,
        #[bits(3)]
        operating_mode: OperatingMode,
    }

    GyroscopeControlRegister6 (0x15, 1, Read, Write) {
        #[bits(4)]
        full_scale: GyroscopeFullScaleSelection,
        #[bits(3)]
        lpf_bandwidth: u8,
    }

    Interrupt1Control (0x0D, 1, Read, Write) {
        accelerometer_data_ready: bool,
        gyroscope_data_ready: bool,
        fifo_threshold: bool,
        fifo_overrun: bool,
        fifo_full: bool,
        cnt_bdr: bool,
    }

    Interrupt2Control (0x0E, 1, Read, Write) {
        accelerometer_data_ready: bool,
        gyroscope_data_ready: bool,
        fifo_threshold: bool,
        fifo_overrun: bool,
        fifo_full: bool,
        cnt_bdr: bool,
        embedded_func_reroute: bool,
    }

    FunctionsEnable (0x50, 1, Read, Write) {
        #[bits(2)]
        inactivity_enable: u8,
        #[skip(1)]
        stop_resetting_interrupt_flags: bool,
        #[skip(2)]
        timestamp_enable: bool,
        interrupts_enabled: bool,
    }

    StatusRegister (0x1E, 1, Read) {
        accelerometer_data_ready: bool,
        gyroscope_data_ready: bool,
        temperature_data_ready: bool,
        #[skip(1)]
        eis_gyroscope_data_ready: bool,
        ois_accelerometer_or_gyroscope_data_ready: bool,
        timestamp_endcount: bool,
    }
}

pub struct Lsm6dsv<D: SpiDevice> {
    device: D,

    gyroscope_full_scale: GyroscopeFullScaleSelection,
    accelerometer_full_scale: AccelerometerFullScaleSelection,
}

impl<D: SpiDevice> Lsm6dsv<D> {
    pub fn new(device: D) -> Self {
        Self {
            device,

            gyroscope_full_scale: Default::default(),
            accelerometer_full_scale: Default::default(),
        }
    }

    pub fn set_gyroscope_data_rate(
        &mut self,
        data_rate: GyroscopeDataRate,
    ) -> Result<(), RegisterError<D::Error>> {
        self.write_register(GyroscopeControlRegister2 {
            data_rate,
            operating_mode: OperatingMode::HighPerformance,
        })
    }

    pub fn set_data_ready_interrupts_int1(
        &mut self,
        gyroscope: bool,
        accelerometer: bool,
    ) -> Result<(), RegisterError<D::Error>> {
        self.write_register(Interrupt1Control {
            accelerometer_data_ready: accelerometer,
            gyroscope_data_ready: gyroscope,
            cnt_bdr: false,
            fifo_full: false,
            fifo_overrun: false,
            fifo_threshold: false,
        })
    }

    pub fn set_data_ready_interrupts_int2(
        &mut self,
        gyroscope: bool,
        accelerometer: bool,
    ) -> Result<(), RegisterError<D::Error>> {
        self.write_register(Interrupt2Control {
            accelerometer_data_ready: accelerometer,
            gyroscope_data_ready: gyroscope,
            cnt_bdr: false,
            fifo_full: false,
            fifo_overrun: false,
            fifo_threshold: false,
            embedded_func_reroute: false,
        })
    }

    pub fn enable_interrupts(
        &mut self,
        interrupts_enabled: bool,
    ) -> Result<(), RegisterError<D::Error>> {
        self.write_register(FunctionsEnable {
            inactivity_enable: 0,
            interrupts_enabled,
            stop_resetting_interrupt_flags: false,
            timestamp_enable: false,
        })
    }

    pub fn set_gyroscope_full_scale(
        &mut self,
        full_scale: GyroscopeFullScaleSelection,
    ) -> Result<(), RegisterError<D::Error>> {
        self.gyroscope_full_scale = full_scale;

        self.write_register(GyroscopeControlRegister6 {
            lpf_bandwidth: 0,
            full_scale,
        })
    }

    pub fn gyroscope_data_ready(&mut self) -> Result<bool, RegisterError<D::Error>> {
        let status: StatusRegister = self.read_register()?;

        Ok(status.gyroscope_data_ready)
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

        let gyroscope_sensitivity = self.gyroscope_full_scale.as_velocity_per_lsb();

        let velocities = [x, y, z].map(|val| val as f32 * gyroscope_sensitivity);

        Ok(Vector::from_array(velocities))
    }

    fn get_linear_acceleration(
        &mut self,
    ) -> Result<Vector<3, Acceleration>, RegisterError<D::Error>> {
        let AccelerometerOutputAll { x, y, z } = self.read_register()?;

        let accelerometer_sensitivity = self.accelerometer_full_scale.as_acceleration_per_lsb();

        let accelerations = [x, y, z].map(|val| val as f32 * accelerometer_sensitivity);

        Ok(Vector::from_array(accelerations))
    }
}
