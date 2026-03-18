use embedded_hal::spi::{Operation, SpiDevice};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::{
    Blocking,
    gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pull},
    spi::master::{AnySpi, Config, Spi},
};

pub mod registers;
pub mod task;

pub trait Register {
    const ADDRESS: u8;
}

#[derive(Debug)]
pub enum RegisterError<D> {
    ParseFailed,
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
    ) -> Result<Self, RegisterError<D::Error>> {
        let bytes = Self::read_bytes(device).map_err(|e| RegisterError::DeviceError(e))?;

        let data = bytes.try_into().map_err(|_| RegisterError::ParseFailed)?;

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
    ) -> Result<(), RegisterError<D::Error>> {
        let bytes = self.try_into().map_err(|_| RegisterError::ParseFailed)?;

        Self::write_bytes(device, bytes).map_err(|e| RegisterError::DeviceError(e))?;

        Ok(())
    }
}

pub struct Imu<'a, D: SpiDevice> {
    spi_device: D,

    int1: Input<'a>,
    int2: Input<'a>,
}

impl<D: SpiDevice> Imu<'_, D> {
    pub fn read<const N: usize, T: ReadRegister<N>>(
        &mut self,
    ) -> Result<T, RegisterError<D::Error>> {
        T::read(&mut self.spi_device)
    }

    pub fn write<const N: usize, T: WriteRegister<N>>(
        &mut self,
        value: T,
    ) -> Result<(), RegisterError<D::Error>> {
        value.write(&mut self.spi_device)?;

        Ok(())
    }

    pub fn wait_for_data(&mut self) -> impl Future<Output = ()> {
        self.int1.wait_for_rising_edge()
    }
}

impl<'a> Imu<'a, ExclusiveDevice<Spi<'a, Blocking>, Output<'a>, NoDelay>> {
    pub fn new(
        peripheral: AnySpi<'a>,
        sck: AnyPin<'a>,
        miso: AnyPin<'a>,
        mosi: AnyPin<'a>,
        cs: AnyPin<'a>,
        int1_pin: AnyPin<'a>,
        int2_pin: AnyPin<'a>,
    ) -> Self {
        let config = Config::default();
        let spi = Spi::new(peripheral, config)
            .unwrap()
            .with_miso(miso)
            .with_mosi(mosi)
            .with_sck(sck);

        let cs_config = OutputConfig::default();

        let spi_device =
            ExclusiveDevice::new(spi, Output::new(cs, Level::High, cs_config), NoDelay).unwrap();

        let input_config = InputConfig::default().with_pull(Pull::None);

        let int1 = Input::new(int1_pin, input_config);
        let int2 = Input::new(int2_pin, input_config);

        Self {
            spi_device,
            int1,
            int2,
        }
    }
}
