use heapless::Vec;
use linalg::{
    line::Line,
    vector::{Vector, real::UnitVector},
};
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
            pos + (*UnitVector::from_angle(angle)) * Length::new::<meter>(1000.0),
        );

        self.field
            .iter()
            .filter_map(|seg| seg.intersection(sensor_lineseg))
            .min_by_key(|intersection| Line(*intersection, pos).length().value.to_bits())
    }
}
