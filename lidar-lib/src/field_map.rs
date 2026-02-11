use crate::line_seg::LineSeg;
use crate::vector::Vector;
use alloc::vec::Vec;
use libm::{cosf, sinf};

pub struct FieldMap {
    field: Vec<LineSeg>,
}

impl FieldMap {
    pub fn intersect_pos(&self, pos: Vector<2>, angle: f32) -> Option<Vector<2>> {
        let mut closest_intersect_dist: f32 = 10000.0;
        let mut intersect_pos: Option<Vector<2>> = None;

        let sensor_lineseg = LineSeg(
            pos,
            Vector([
                pos.x() + 1000.0 * sinf(angle),
                pos.y() + 1000.0 * cosf(angle),
            ]),
        );

        for seg in self.field.iter() {
            let intersect = seg.find_intersection(sensor_lineseg);
            if let Some(intersect) = intersect {
                let distance_to_point = intersect.distance_to(pos);

                if distance_to_point < closest_intersect_dist {
                    closest_intersect_dist = distance_to_point;
                    intersect_pos = Some(intersect);
                }
            }
        }
        intersect_pos
    }
}
