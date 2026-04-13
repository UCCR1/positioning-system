use core::ops::Mul;

use embedded_hal::spi::{Operation, SpiDevice};
use linalg::vector::Vector;

pub mod lsm6ds3tr;
pub mod lsm6dsv;
mod macros;

trait Register {
    const ADDRESS: u8;
}

#[derive(Debug)]
pub enum RegisterError<D> {
    ParseFailed,
    DeviceError(D),
}

trait ReadRegister<const N: usize>: Register + TryFrom<[u8; N]> {
    fn read_bytes<D: SpiDevice>(device: &mut D) -> Result<[u8; N], D::Error> {
        let mut data = [0u8; N];

        device.transaction(&mut [
            Operation::Write(&[Self::ADDRESS | (1 << 7)]),
            Operation::Read(&mut data),
        ])?;

        Ok(data)
    }

    fn read<D: SpiDevice>(device: &mut D) -> Result<Self, RegisterError<D::Error>> {
        let bytes = Self::read_bytes(device).map_err(|e| RegisterError::DeviceError(e))?;

        let data = bytes.try_into().map_err(|_| RegisterError::ParseFailed)?;

        Ok(data)
    }
}

trait WriteRegister<const N: usize>: Register + TryInto<[u8; N]> {
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

trait RegisterDevice<D: SpiDevice> {
    fn device(&mut self) -> &mut D;

    fn read_register<const N: usize, T: ReadRegister<N>>(
        &mut self,
    ) -> Result<T, RegisterError<D::Error>> {
        T::read(self.device())
    }

    fn write_register<const N: usize, T: WriteRegister<N>>(
        &mut self,
        value: T,
    ) -> Result<(), RegisterError<D::Error>> {
        value.write(self.device())?;

        Ok(())
    }
}

fn make_full_scale_vector<O: Copy, const N: usize>(data: [i16; N], sensitivity: O) -> Vector<N, O>
where
    O: Mul<f32, Output = O>,
{
    Vector::from_array(data.map(|val| sensitivity * val as f32))
}
