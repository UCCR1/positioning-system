use esp_hal::{
    Blocking,
    gpio::AnyPin,
    uart::{AnyUart, Config, RxError, Uart, UartRx},
};
use heapless::spsc::Queue;
use lidar_lib::data::{LidarDataReader, LidarPoint, LidarReadError};

const LIDAR_BAUDRATE: u32 = 230400;

const MAX_HISTORY: usize = 300;

use thiserror::Error;

#[derive(Error, Debug)]
pub enum LidarError {
    #[error("Failed to read received bytes")]
    ReadError(#[from] LidarReadError),
    #[error("Failed to load serial data from UART")]
    SerialError(#[from] RxError),
}

pub struct Lidar<'a> {
    rx: UartRx<'a, Blocking>,

    reader: LidarDataReader,

    points: Queue<LidarPoint, MAX_HISTORY>,
}

impl<'a> Lidar<'a> {
    pub fn new(uart: AnyUart<'a>, rx: AnyPin<'a>, pwm: AnyPin<'a>) -> Self {
        let config = Config::default().with_baudrate(LIDAR_BAUDRATE);

        let uart = Uart::new(uart, config).unwrap().with_rx(rx);

        let (rx, _) = uart.split();

        Self {
            rx,
            reader: LidarDataReader::new(),
            points: Default::default(),
        }
    }

    pub fn get_points(&self) -> impl Iterator<Item = &LidarPoint> {
        self.points.iter()
    }

    pub fn update(&mut self) -> Result<(), LidarError> {
        let mut tmp_buf = [0; 100];

        let read_size = self.rx.read(&mut tmp_buf)?;

        if let Some(data) = self.reader.read_slice(&tmp_buf[..read_size])? {
            for point in data.points {
                if self.points.is_full() {
                    self.points.dequeue();
                }

                let _ = self.points.enqueue(point);
            }
        }

        Ok(())
    }
}
