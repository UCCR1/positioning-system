use crate::vector::Vector;

#[derive(Clone, Copy)]
pub struct LineSeg(Vector<2>, Vector<2>);

impl LineSeg {
    pub fn is_point_in_bound(self, point: Vector<2>) -> bool {
        if point.x() > self.0.x().max(self.1.x()) || point.x() < self.0.x().min(self.1.x()) {
            return false;
        } else if point.y() > self.0.y().max(self.1.y()) || point.y() < self.0.y().min(self.1.y()) {
            return false;
        }
        true
    }

    pub fn find_intersection(self, seg: LineSeg) -> Option<Vector<2>> {
        let denom = (self.0.x() - self.1.x()) * (seg.0.y() - seg.1.y())
            - (self.0.y() - self.1.y()) * (seg.0.x() - seg.1.x());

        if denom == 0.0 {
            return None;
        }

        let numer_1 = self.0.x() * self.1.y() - self.0.y() * self.1.x();
        let numer_2 = seg.0.x() * seg.1.y() - seg.0.y() * seg.1.x();

        let px = (numer_1 * (seg.0.x() - seg.1.x()) - (self.0.x() - self.1.x()) * numer_2) / denom;
        let py = (numer_1 * (seg.0.y() - seg.1.y()) - (self.0.y() - self.1.y()) * numer_2) / denom;

        let intersect = Vector([px, py]);

        if !self.is_point_in_bound(intersect) || !seg.is_point_in_bound(intersect) {
            return None;
        }

        Some(intersect)
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn test_point_in_bound() {
        let point = Vector([0.0, 0.0]);
        let line_seg = LineSeg(Vector([-1.0, -1.0]), Vector([1.0, 1.0]));

        assert!(line_seg.is_point_in_bound(point));
    }

    #[test]
    fn test_point_not_in_bound() {
        let point = Vector([2.0, 0.0]);
        let line_seg = LineSeg(Vector([-1.0, -1.0]), Vector([1.0, 1.0]));

        assert!(!line_seg.is_point_in_bound(point));
    }

    #[test]
    fn test_find_intersection() {
        let line_seg1 = LineSeg(Vector([0.0, 0.0]), Vector([5.0, 5.0]));
        let line_seg2 = LineSeg(Vector([2.0, 0.0]), Vector([2.0, 5.0]));

        assert_relative_eq!(
            line_seg1.find_intersection(line_seg2).unwrap().x(),
            Vector([2.0, 2.0]).x(),
            max_relative = 0.01
        );
        assert_relative_eq!(
            line_seg1.find_intersection(line_seg2).unwrap().y(),
            Vector([2.0, 2.0]).y(),
            max_relative = 0.01
        );
    }

    #[test]
    fn test_find_intersection2() {
        let line_seg1 = LineSeg(Vector([0.0, 0.0]), Vector([5.0, 5.0]));
        let line_seg2 = LineSeg(Vector([0.0, 5.0]), Vector([5.0, 0.0]));

        assert_relative_eq!(
            line_seg1.find_intersection(line_seg2).unwrap().x(),
            Vector([2.5, 2.5]).x(),
            max_relative = 0.01
        );
        assert_relative_eq!(
            line_seg1.find_intersection(line_seg2).unwrap().y(),
            Vector([2.5, 2.5]).y(),
            max_relative = 0.01
        );
    }

    #[test]
    fn test_find_intersection_outside_segments() {
        let line_seg1 = LineSeg(Vector([0.0, 0.0]), Vector([5.0, 5.0]));
        let line_seg2 = LineSeg(Vector([2.0, 0.0]), Vector([5.0, 0.0]));

        assert_eq!(line_seg1.find_intersection(line_seg2), None)
    }

    #[test]
    fn test_find_intersection_parellel_lines() {
        let line_seg1 = LineSeg(Vector([0.0, 0.0]), Vector([5.0, 5.0]));
        let line_seg2 = LineSeg(Vector([0.0, -1.0]), Vector([5.0, 4.0]));

        assert_eq!(line_seg1.find_intersection(line_seg2), None)
    }
}
