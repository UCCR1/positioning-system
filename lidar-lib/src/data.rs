use alloc::vec::Vec;

use position_lib::{linalg::vector::Vector, real_vector};
use uom::si::{
    angle::degree,
    angular_velocity::degree_per_second,
    f32::{Angle, AngularVelocity, Length},
};
use zerocopy::FromBytes;

use super::crc::get_crc_checksum;

const NUM_POINTS: usize = 12;

#[repr(packed)]
#[derive(FromBytes, Clone, Copy, Debug)]
struct DataPoint {
    distance: u16,
    intensity: u8,
}

#[repr(packed)]
#[derive(FromBytes, Debug)]
pub struct Packet {
    header: u8,
    ver_len: u8,
    speed: u16,
    start_angle: u16,
    point: [DataPoint; NUM_POINTS],
    end_angle: u16,
    timestamp: u16,
    crc8: u8,
}

pub struct LidarPoint(pub Vector<2, Length>, pub u8);

pub struct LidarData {
    pub speed: AngularVelocity,
    pub start_angle: Angle,
    pub end_angle: Angle,
    pub points: Vec<LidarPoint>,
}

#[derive(Debug)]
pub enum LidarParseError {
    InvalidHeader,
    CRCError { source: u8, calculated: u8 },
    SizeError,
}

pub fn parse_packet(packet: &[u8]) -> Result<LidarData, LidarParseError> {
    if packet[0] != 0x54 || packet[1] != 0x2C {
        return Err(LidarParseError::InvalidHeader);
    }

    let data = Packet::read_from_bytes(packet).map_err(|_| LidarParseError::SizeError)?;

    let packet_crc = get_crc_checksum(&packet[0..(packet.len() - 1)]);

    if packet_crc != data.crc8 {
        return Err(LidarParseError::CRCError {
            source: data.crc8,
            calculated: packet_crc,
        });
    }

    // TODO: This angle wrapping might not be suitable if end is smaller than start
    let start_angle = Angle::new::<degree>(data.start_angle as f32 / 100.0) % Angle::FULL_TURN;
    let end_angle = Angle::new::<degree>(data.end_angle as f32 / 100.0) % Angle::FULL_TURN;

    let angle_diff = (end_angle - start_angle + Angle::FULL_TURN) % Angle::FULL_TURN;

    let increment = angle_diff / (NUM_POINTS - 1) as f32;

    let angles = (0..NUM_POINTS).map(|i| start_angle + i as f32 * increment);

    let points = angles.zip(data.point).map(|(angle, point)| {
        LidarPoint(
            real_vector![Length::millimeter, 0.0, point.distance as f32].rotate(-angle),
            point.intensity,
        )
    });

    Ok(LidarData {
        speed: AngularVelocity::new::<degree_per_second>(data.speed as f32),
        start_angle,
        end_angle,
        points: points.collect(),
    })
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test() {
        const TEST_PACKET: [u8; size_of::<Packet>()] = [
            0x54, 0x2C, 0x68, 0x08, 0xAB, 0x7E, 0xE0, 0x00, 0xE4, 0xDC, 0x00, 0xE2, 0xD9, 0x00,
            0xE5, 0xD5, 0x00, 0xE3, 0xD3, 0x00, 0xE4, 0xD0, 0x00, 0xE9, 0xCD, 0x00, 0xE4, 0xCA,
            0x00, 0xE2, 0xC7, 0x00, 0xE9, 0xC5, 0x00, 0xE5, 0xC2, 0x00, 0xE5, 0xC0, 0x00, 0xE5,
            0xBE, 0x82, 0x3A, 0x1A, 0x50,
        ];

        let data = parse_packet(&TEST_PACKET).unwrap();

        let expected_start = Angle::new::<degree>(324.27);
        let expected_end = Angle::new::<degree>(334.7);

        assert_eq!(data.speed.get::<degree_per_second>(), 2152.0);
        assert_eq!(data.start_angle, expected_start);
        assert_eq!(data.end_angle, expected_end);

        assert_eq!(data.points[0].1, 228);
        assert_eq!(
            data.points[0].0,
            real_vector!(Length::millimeter, 0.0, 224.0).rotate(-expected_start)
        );

        assert_eq!(data.points[11].1, 229);
        assert_eq!(
            data.points[11].0,
            real_vector!(Length::millimeter, 0.0, 192.0).rotate(-expected_end)
        );
    }
}
