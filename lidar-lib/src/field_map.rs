use heapless::Vec;
use libm::{cosf, sinf};
use linalg::{line::Line, vector::Vector};
use uom::si::{
    f32::{Angle, Length},
    length::meter,
};

pub struct FieldMap {
    field: Vec<Line<2, Length>, 100>,
}

impl FieldMap {
    pub fn intersect_pos(&self, pos: Vector<2, Length>, angle: Angle) -> Option<Vector<2, Length>> {
        let sensor_lineseg = Line(
            pos,
            [
                pos.x() + Length::new::<meter>(1000.0) * sinf(angle.value),
                pos.y() + Length::new::<meter>(1000.0) * cosf(angle.value),
            ]
            .into(),
        );

        self.field
            .iter()
            .filter_map(|seg| seg.intersection(sensor_lineseg))
            .min_by_key(|intersection| Line(*intersection, pos).length().value.to_bits())
    }
}
