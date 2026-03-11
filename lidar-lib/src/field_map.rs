use crate::line_seg::LineSeg;
use alloc::vec::Vec;
use libm::{cosf, sinf};
use position_lib::vector::Vector;

pub struct FieldMap {
    field: Vec<LineSeg>,
}

impl FieldMap {
    pub fn intersect_pos(&self, pos: Vector<2>, angle: f32) -> Option<Vector<2>> {
        let sensor_lineseg = LineSeg(
            pos,
            Vector([
                pos.x() + 1000.0 * sinf(angle),
                pos.y() + 1000.0 * cosf(angle),
            ]),
        );

        self.field
            .iter()
            .filter_map(|seg| seg.find_intersection(sensor_lineseg))
            .min_by_key(|intersection| intersection.distance_to(pos).to_bits())
    }
}
