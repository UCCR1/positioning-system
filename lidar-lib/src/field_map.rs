use crate::line_seg::LineSeg;
use alloc::vec::Vec;
use libm::{cosf, sinf};
use position_lib::linalg::vector::Vector;

use uom::si::{
    f32::{Angle, Length},
    length::meter,
};

pub struct FieldMap {
    field: Vec<LineSeg>,
}

impl FieldMap {
    pub fn intersect_pos(&self, pos: Vector<2, Length>, angle: Angle) -> Option<Vector<2, Length>> {
        let sensor_lineseg = LineSeg(
            pos,
            [
                pos.x() + Length::new::<meter>(1000.0) * sinf(angle.value),
                pos.y() + Length::new::<meter>(1000.0) * cosf(angle.value),
            ]
            .into(),
        );

        self.field
            .iter()
            .filter_map(|seg| seg.find_intersection(sensor_lineseg))
            .min_by_key(|intersection| intersection.distance_to(pos).value.to_bits())
    }
}
