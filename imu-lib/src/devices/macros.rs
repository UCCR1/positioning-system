#[macro_export]
macro_rules! type_impls {
    ($name:ident, $bytes:literal, Read) => {
        impl $crate::devices::ReadRegister<$bytes> for $name {}
    };

    ($name:ident, $bytes:literal, Write) => {
        impl $crate::devices::WriteRegister<$bytes> for $name {}
    };

    ($name:ident, $bytes:literal, $($types:ident),+) => {
        $($crate::type_impls!{$name, $bytes, $types})+
    };
}

#[macro_export]
macro_rules! declare_registers {
    {$(
        $name:ident ($address:literal, $bytes:literal, $($types:ident),+) $body:tt
    )*} => {
        $(
            #[derive(Debug)]
            #[packbits::pack(bytes = $bytes)]
            pub struct $name $body

            impl $crate::devices::Register for $name {
                const ADDRESS: u8 = $address;
            }

            $crate::type_impls! {
                $name,
                $bytes,
                $($types),+
            }
        )*
    };
}
