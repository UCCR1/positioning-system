use esp_hal::{
    Blocking,
    gpio::AnyPin,
    uart::{AnyUart, Config, RxError, Uart, UartRx},
};
use heapless::{CapacityError, Vec, spsc::Queue};
use lidar_lib::data::{LidarParseError, LidarPoint, PACKET_HEADER, PACKET_SIZE, parse_packet};

const LIDAR_BAUDRATE: u32 = 230400;

const MAX_HISTORY: usize = 300;

use thiserror::Error;

#[derive(Error, Debug)]
pub enum LidarError {
    #[error("Packet failed to parse")]
    PacketError(#[from] LidarParseError),
    #[error("Failed to read serial data")]
    SerialError(#[from] RxError),
    #[error("Breached data buffer capacity")]
    CapacityError(#[from] CapacityError),
}

pub struct Lidar<'a> {
    rx: UartRx<'a, Blocking>,

    data_buffer: Vec<u8, 100>,

    points: Queue<LidarPoint, MAX_HISTORY>,
}

impl<'a> Lidar<'a> {
    pub fn new(uart: AnyUart<'a>, rx: AnyPin<'a>, pwm: AnyPin<'a>) -> Self {
        let config = Config::default().with_baudrate(LIDAR_BAUDRATE);

        let uart = Uart::new(uart, config).unwrap().with_rx(rx);

        let (rx, _) = uart.split();

        Self {
            rx,
            data_buffer: Default::default(),
            points: Default::default(),
        }
    }

    pub fn get_points(&self) -> impl Iterator<Item = &LidarPoint> {
        self.points.iter()
    }

    pub fn update(&mut self) -> Result<(), LidarError> {
        let mut tmp_buf = [0; 100];

        let read_size = self.rx.read(&mut tmp_buf)?;

        if read_size > 0 {
            self.data_buffer.extend_from_slice(&tmp_buf[..read_size])?;
        }

        while self.data_buffer.len() > 2 && !self.data_buffer.starts_with(&PACKET_HEADER) {
            self.data_buffer.drain(..1);
        }

        if self.data_buffer.len() >= PACKET_SIZE && self.data_buffer.starts_with(&PACKET_HEADER) {
            let packet = self.data_buffer.drain(..PACKET_SIZE);

            let data = parse_packet(packet.as_slice())?;

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
