use embedded_hal::spi::{Operation, SpiDevice};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{
    Blocking,
    gpio::{AnyPin, Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    peripherals::Interrupt,
    spi::master::{AnySpi, Config, Spi},
    xtensa_lx::interrupt,
};

pub mod registers;

pub trait Register {
    const ADDRESS: u8;
}

pub enum RegisterError<P, D> {
    ParseFailed(P),
    DeviceError(D),
}

pub trait ReadRegister<const N: usize>: Register + TryFrom<[u8; N]> {
    fn read_bytes<D: SpiDevice>(device: &mut D) -> Result<[u8; N], D::Error> {
        let mut data = [0u8; N];

        device.transaction(&mut [
            Operation::Write(&[Self::ADDRESS | (1 << 7)]),
            Operation::Read(&mut data),
        ])?;

        Ok(data)
    }

    fn read<D: embedded_hal::spi::SpiDevice>(
        device: &mut D,
    ) -> Result<Self, RegisterError<<Self as TryFrom<[u8; N]>>::Error, D::Error>> {
        let bytes = Self::read_bytes(device).map_err(|e| RegisterError::DeviceError(e))?;

        let data = bytes
            .try_into()
            .map_err(|e| RegisterError::ParseFailed(e))?;

        Ok(data)
    }
}

pub trait WriteRegister<const N: usize>: Register + TryInto<[u8; N]> {
    fn write_bytes<D: SpiDevice>(device: &mut D, bytes: [u8; N]) -> Result<(), D::Error> {
        device.transaction(&mut [Operation::Write(&[Self::ADDRESS]), Operation::Write(&bytes)])?;

        Ok(())
    }

    fn write<D: embedded_hal::spi::SpiDevice>(
        self,
        device: &mut D,
    ) -> Result<(), RegisterError<<Self as TryInto<[u8; N]>>::Error, D::Error>> {
        let bytes = self.try_into().map_err(|e| RegisterError::ParseFailed(e))?;

        Self::write_bytes(device, bytes).map_err(|e| RegisterError::DeviceError(e))?;

        Ok(())
    }
}

pub struct Imu<D: SpiDevice> {
    device: D,
}

impl<D: SpiDevice> Imu<D> {
    pub fn read<const N: usize, T: ReadRegister<N>>(
        &mut self,
    ) -> Result<T, RegisterError<<T as TryFrom<[u8; N]>>::Error, D::Error>> {
        T::read(&mut self.device)
    }

    pub fn write<const N: usize, T: WriteRegister<N>>(
        &mut self,
        value: T,
    ) -> Result<(), RegisterError<<T as TryInto<[u8; N]>>::Error, D::Error>> {
        value.write(&mut self.device)?;

        Ok(())
    }
}

impl<'a> Imu<ExclusiveDevice<Spi<'a, Blocking>, Output<'a>, NoDelay>> {
    pub fn new(
        peripheral: AnySpi<'a>,
        sck: AnyPin<'a>,
        miso: AnyPin<'a>,
        mosi: AnyPin<'a>,
        cs: AnyPin<'a>,
        int1: AnyPin<'a>,
        int2: AnyPin<'a>,
    ) -> Self {
        let config = Config::default();
        let spi = Spi::new(peripheral, config)
            .unwrap()
            .with_miso(miso)
            .with_mosi(mosi)
            .with_sck(sck);

        let cs_config = OutputConfig::default();

        let device =
            ExclusiveDevice::new(spi, Output::new(cs, Level::High, cs_config), NoDelay).unwrap();

        let input_config = InputConfig::default().with_pull(Pull::Down);

        let mut int1 = Input::new(int1, input_config);
        int1.listen(esp_hal::gpio::Event::RisingEdge);

        let mut int2 = Input::new(int2, input_config);
        int2.listen(esp_hal::gpio::Event::RisingEdge);

        Self { device }
    }
}
