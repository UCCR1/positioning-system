use core::array;

use uom::{
    ConstZero,
    si::f32::{Angle, Length, Ratio},
};

use crate::vector::{Vector, real::UnitVector};

#[derive(Copy, Clone)]
pub struct TrackingWheel {
    pub location: Vector<2, Length>,
    pub direction: UnitVector<2, Ratio>,
}

pub struct Odometry<const N: usize> {
    tracking_wheels: [TrackingWheel; N],

    weighting_matrix: [[Ratio; 2]; N],

    global_position: Vector<2, Length>,
    global_angle: Angle,
}

impl<const N: usize> Odometry<N> {
    pub fn new(wheels: [TrackingWheel; N]) -> Self {
        let ddt: Vector<2, Vector<2, Ratio>> = wheels
            .iter()
            .map(|v| v.direction.product(*v.direction))
            .sum();

        assert!(
            ddt.det().abs().value > 1e-6,
            "Tracking wheel system is not solvable"
        );

        let ddt_inv = ddt.inv();

        let weighting_matrix = wheels.map(|wheel| {
            let dx = wheel.direction.x();
            let dy = wheel.direction.y();

            [
                dx * ddt_inv[0][0] + dy * ddt_inv[1][0],
                dx * ddt_inv[0][1] + dy * ddt_inv[1][1],
            ]
        });

        Self {
            tracking_wheels: wheels,
            weighting_matrix,
            global_position: Default::default(),
            global_angle: Default::default(),
        }
    }

    pub fn update(&mut self, wheel_travel: [Length; N], angle_change: Angle) {
        let travels: [Length; N] = array::from_fn(|i| {
            let travel = wheel_travel[i];
            let position = self.tracking_wheels[i];

            let measured_travel: Vector<2, Length> = *position.direction * travel;

            let rotation_travel: Vector<2, Length> = position.location.perp() * angle_change;

            let actual_travel: Vector<2, Length> =
                measured_travel - rotation_travel.project(measured_travel);

            actual_travel.length()
        });

        let mut dx = Length::ZERO;
        let mut dy = Length::ZERO;

        for (i, [wx, wy]) in self.weighting_matrix.iter().enumerate() {
            dx += *wx * travels[i];
            dy += *wy * travels[i];
        }

        let true_travel = Vector([dx, dy]).bend(angle_change);

        self.global_position = self.global_position + true_travel; // TODO: rotate true_travel by global_angle
        self.global_angle += angle_change;
    }
}

#[cfg(test)]
mod test {
    use core::f32::consts::FRAC_1_SQRT_2;

    use uom::{
        ConstZero,
        si::{
            angle::{degree, radian},
            length::meter,
            quantities::{Angle, Length},
        },
    };

    use crate::{
        odometry::{Odometry, TrackingWheel},
        vector::{Vector, real::UnitVector},
    };

    use approx::assert_relative_eq;

    #[test]
    fn horiz_and_diag_system() {
        // This test case does not test rotation, so the position of the wheels is irrelevant
        let wheels = [
            TrackingWheel {
                direction: UnitVector::from_angle(Angle::ZERO),
                location: Vector([Length::ZERO, Length::ZERO]),
            },
            TrackingWheel {
                direction: UnitVector::from_angle(Angle::new::<degree>(45.0)),
                location: Vector([Length::ZERO, Length::ZERO]),
            },
        ];

        let mut odom = Odometry::new(wheels);

        odom.update(
            [Length::ZERO, Length::new::<meter>(FRAC_1_SQRT_2)],
            Angle::ZERO,
        );

        // If the horizontal tracking wheel doesen't move, there should be no X change
        assert_relative_eq!(odom.global_position.x().value, 0.0);
        assert_relative_eq!(odom.global_position.y().value, 1.0);

        odom.update(
            [
                Length::new::<meter>(1.0),
                Length::new::<meter>(FRAC_1_SQRT_2),
            ],
            Angle::ZERO,
        );

        // If the horizontal tracking wheel does move, and the right tracking wheel moves forwards by 1/SQRT(2), we must be moving only horizontal
        assert_relative_eq!(odom.global_position.x().value, 1.0);
        assert_relative_eq!(odom.global_position.y().value, 1.0);

        // If the horizontal tracking wheel does move, and the right tracking wheel does not, we are moving perpendicular to the diagonal wheel
        odom.update([Length::new::<meter>(1.0), Length::ZERO], Angle::ZERO);

        assert_relative_eq!(odom.global_position.x().value, 2.0);
        assert_relative_eq!(odom.global_position.y().value, 0.0);
    }

    #[test]
    fn dual_diag_system() {
        let wheels = [
            TrackingWheel {
                direction: UnitVector::from_angle(Angle::new::<degree>(45.0)),
                location: Vector([
                    -Length::new::<meter>(FRAC_1_SQRT_2),
                    Length::new::<meter>(FRAC_1_SQRT_2),
                ]),
            },
            TrackingWheel {
                direction: UnitVector::from_angle(Angle::new::<degree>(135.0)),
                location: Vector([
                    Length::new::<meter>(FRAC_1_SQRT_2),
                    Length::new::<meter>(FRAC_1_SQRT_2),
                ]),
            },
        ];

        let mut odom = Odometry::new(wheels);

        // If both wheels rotate forward by 1/SQRT(2), we have moved forward by 1 unit
        odom.update(
            [
                Length::new::<meter>(FRAC_1_SQRT_2),
                Length::new::<meter>(FRAC_1_SQRT_2),
            ],
            Angle::ZERO,
        );

        assert_relative_eq!(odom.global_position.x().value, 0.0);
        assert_relative_eq!(odom.global_position.y().value, 1.0);

        // If the left wheel rotates backwards by 1 unit, and right wheel forwards by 1 unit, AND we have rotated left by 1 radian, then we should not have had a change in position
        odom.update(
            [-Length::new::<meter>(1.0), Length::new::<meter>(1.0)],
            Angle::new::<radian>(1.0),
        );

        assert_relative_eq!(odom.global_position.x().value, 0.0);
        assert_relative_eq!(odom.global_position.y().value, 1.0);
        assert_relative_eq!(odom.global_angle.value, 1.0);
    }
}
