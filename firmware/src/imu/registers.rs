use num_enum::{IntoPrimitive, TryFromPrimitive};

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
}
