use num_enum::{IntoPrimitive, TryFromPrimitive};
use uom::si::{angular_velocity::degree_per_second, f32::AngularVelocity};

macro_rules! type_impls {
    ($name:ident, $bytes:literal, Read) => {
        impl crate::imu::ReadRegister<$bytes> for $name {}
    };

    ($name:ident, $bytes:literal, Write) => {
        impl crate::imu::WriteRegister<$bytes> for $name {}
    };

    ($name:ident, $bytes:literal, $($types:ident),+) => {
        $(type_impls!{$name, $bytes, $types})+
    };
}

macro_rules! registers {
    {$(
        $name:ident ($address:literal, $bytes:literal, $($types:ident),+) $body:tt
    )*} => {
        $(
            #[packbits::pack(bytes = $bytes)]
            pub struct $name $body

            impl crate::imu::Register for $name {
                const ADDRESS: u8 = $address;
            }

            type_impls! {
                $name,
                $bytes,
                $($types),+
            }
        )*
    };
}

#[derive(TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
pub enum AnalogBandwidth {
    _1500Hz = 0,
    _400Hz = 1,
}

#[derive(TryFromPrimitive, IntoPrimitive, Copy, Clone, Debug, Default)]
#[repr(u8)]
pub enum FullScaleSelection {
    #[default]
    Dps245 = 0b00,
    Dps500 = 0b01,
    Dps1000 = 0b10,
    Dps2000 = 0b11,
}

const DATA_RATE: u8 = 0b1010; // 6.66kHz

impl From<FullScaleSelection> for AngularRateSensorControl {
    fn from(value: FullScaleSelection) -> Self {
        Self {
            data_rate: DATA_RATE,
            dps125: false,
            full_scale: value,
        }
    }
}

impl FullScaleSelection {
    pub fn as_velocity(self) -> AngularVelocity {
        AngularVelocity::new::<degree_per_second>(match self {
            FullScaleSelection::Dps245 => 240.0,
            FullScaleSelection::Dps500 => 500.0,
            FullScaleSelection::Dps1000 => 1000.0,
            FullScaleSelection::Dps2000 => 2000.0,
        })
    }
}

registers! {
    GyroscopeOutputAll (0x22, 6, Read) {
        pub x: i16,
        pub y: i16,
        pub z: i16,
    }

    AccelerometerControl (0x10, 1, Read, Write) {
        #[bits(1)]
        pub analog_bandwidth_selection: AnalogBandwidth,
        pub digital_bandwidth_selection: bool,
        #[bits(2)]
        pub accelerometer_full_scale: u8,
        #[bits(4)]
        pub data_rate: u8,
    }

    AngularRateSensorControl (0x11, 1, Read, Write) {
        #[skip(1)]
        #[bits(1)]
        dps125: bool,
        #[bits(2)]
        full_scale: FullScaleSelection,
        #[bits(4)]
        data_rate: u8,
    }
}
