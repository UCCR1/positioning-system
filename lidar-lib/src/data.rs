#[repr(packed)]
#[derive(Clone, Copy, Debug)]
struct LidarPoint {
    distance: u16,
    intensity: u8,
}

#[repr(packed)]
#[derive(Debug)]
struct Packet {
    header: u8,
    ver_len: u8,
    speed: u16,
    start_angle: u16,
    point: [LidarPoint; 12],
    end_angle: u16,
    timestamp: u16,
    crc8: u8,
}
