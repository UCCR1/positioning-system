use linalg::{vector, vector::Vector};
use uom::{
    ConstZero,
    si::f32::{Area, Length},
};

#[derive(Clone, Copy)]
pub struct LineSeg(pub Vector<2, Length>, pub Vector<2, Length>);

impl LineSeg {
    pub fn is_point_in_bound(self, point: Vector<2, Length>) -> bool {
        if point.x() > self.0.x().max(self.1.x()) || point.x() < self.0.x().min(self.1.x()) {
            return false;
        } else if point.y() > self.0.y().max(self.1.y()) || point.y() < self.0.y().min(self.1.y()) {
            return false;
        }
        true
    }

    pub fn find_intersection(self, seg: LineSeg) -> Option<Vector<2, Length>> {
        let denom = (self.0.x() - self.1.x()) * (seg.0.y() - seg.1.y())
            - (self.0.y() - self.1.y()) * (seg.0.x() - seg.1.x());

        if denom == Area::ZERO {
            return None;
        }

        let numer_1 = self.0.x() * self.1.y() - self.0.y() * self.1.x();
        let numer_2 = seg.0.x() * seg.1.y() - seg.0.y() * seg.1.x();

        let px = (numer_1 * (seg.0.x() - seg.1.x()) - (self.0.x() - self.1.x()) * numer_2) / denom;
        let py = (numer_1 * (seg.0.y() - seg.1.y()) - (self.0.y() - self.1.y()) * numer_2) / denom;

        let intersect = vector![px, py];

        if !self.is_point_in_bound(intersect) || !seg.is_point_in_bound(intersect) {
            return None;
        }

        Some(intersect)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use linalg::real_vector;

    use super::*;

    #[test]
    fn test_point_in_bound() {
        let point = real_vector!(Length::meter, 0.0, 0.0);
        let line_seg = LineSeg(
            real_vector![Length::meter, -1.0, -1.0],
            real_vector![Length::meter, 1.0, 1.0],
        );

        assert!(line_seg.is_point_in_bound(point));
    }

    #[test]
    fn test_point_not_in_bound() {
        let point = real_vector!(Length::meter, 2.0, 0.0);
        let line_seg = LineSeg(
            real_vector!(Length::meter, -1.0, -1.0),
            real_vector!(Length::meter, 1.0, 1.0),
        );

        assert!(!line_seg.is_point_in_bound(point));
    }

    #[test]
    fn test_find_intersection() {
        let line_seg1 = LineSeg(
            real_vector!(Length::meter, 0.0, 0.0),
            real_vector!(Length::meter, 5.0, 5.0),
        );
        let line_seg2 = LineSeg(
            real_vector!(Length::meter, 2.0, 0.0),
            real_vector!(Length::meter, 2.0, 5.0),
        );

        assert_relative_eq!(
            line_seg1.find_intersection(line_seg2).unwrap().x().value,
            real_vector!(Length::meter, 2.0, 2.0).x().value,
            max_relative = 0.01
        );

        assert_relative_eq!(
            line_seg1.find_intersection(line_seg2).unwrap().y().value,
            real_vector!(Length::meter, 2.0, 2.0).y().value,
            max_relative = 0.01
        );
    }

    #[test]
    fn test_find_intersection2() {
        let line_seg1 = LineSeg(
            real_vector!(Length::meter, 0.0, 0.0),
            real_vector!(Length::meter, 5.0, 5.0),
        );
        let line_seg2 = LineSeg(
            real_vector!(Length::meter, 0.0, 5.0),
            real_vector!(Length::meter, 5.0, 0.0),
        );

        assert_relative_eq!(
            line_seg1.find_intersection(line_seg2).unwrap().x().value,
            real_vector!(Length::meter, 2.5, 2.5).x().value,
            max_relative = 0.01
        );

        assert_relative_eq!(
            line_seg1.find_intersection(line_seg2).unwrap().y().value,
            real_vector!(Length::meter, 2.5, 2.5).y().value,
            max_relative = 0.01
        );
    }

    #[test]
    fn test_find_intersection_outside_segments() {
        let line_seg1 = LineSeg(
            real_vector!(Length::meter, 0.0, 0.0),
            real_vector!(Length::meter, 5.0, 5.0),
        );
        let line_seg2 = LineSeg(
            real_vector!(Length::meter, 2.0, 0.0),
            real_vector!(Length::meter, 5.0, 0.0),
        );

        assert_eq!(line_seg1.find_intersection(line_seg2), None)
    }

    #[test]
    fn test_find_intersection_parellel_lines() {
        let line_seg1 = LineSeg(
            real_vector!(Length::meter, 0.0, 0.0),
            real_vector!(Length::meter, 5.0, 5.0),
        );
        let line_seg2 = LineSeg(
            real_vector!(Length::meter, 0.0, -1.0),
            real_vector!(Length::meter, 5.0, 4.0),
        );

        assert_eq!(line_seg1.find_intersection(line_seg2), None)
    }
}
