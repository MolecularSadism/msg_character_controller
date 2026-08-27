//! Configuration for flying propulsion.

use bevy::prelude::*;

use crate::flight::FlightFrame;

/// Configuration for flying propulsion.
#[derive(Reflect, Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct FlyingConfig {
    /// Maximum flying speed (units/second).
    pub max_speed: f32,

    /// Ratio of vertical flying speed relative to horizontal.
    pub vertical_speed_ratio: f32,

    /// Acceleration rate for flying (units/second^2).
    pub acceleration: f32,

    /// Ratio of vertical flying acceleration relative to horizontal.
    pub vertical_acceleration_ratio: f32,

    /// Gravity compensation ratio when flying upward (0.0-1.0+).
    pub gravity_compensation: f32,

    /// Linear damping applied via the physics backend while fly mode is active.
    ///
    /// Drives the backend's linear damping component (e.g. Avian's `LinearDamping`)
    /// instead of manually decaying velocity, so it composes cleanly with impulses
    /// and external forces.
    pub damping: f32,
}

impl Default for FlyingConfig {
    fn default() -> Self {
        Self {
            max_speed: 150.0,
            vertical_speed_ratio: 0.6,
            acceleration: 500.0,
            vertical_acceleration_ratio: 0.6,
            gravity_compensation: 0.05,
            damping: 0.0,
        }
    }
}

/// The per-axis accelerations and speed caps of a [`FlyingConfig`], converged
/// toward their means as gravity confidence falls.
///
/// Produced by [`FlyingConfig::converged_axes`]; "vertical" is along the
/// flight frame's up axis, "horizontal" across it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConvergedAxes {
    /// Acceleration along the frame's up axis (units/second^2).
    pub vertical_acceleration: f32,
    /// Acceleration across the frame's up axis (units/second^2).
    pub horizontal_acceleration: f32,
    /// Speed cap along the frame's up axis (units/second).
    pub vertical_max_speed: f32,
    /// Speed cap across the frame's up axis (units/second).
    pub horizontal_max_speed: f32,
}

impl FlyingConfig {
    /// The per-axis accelerations and speed caps, converged toward their means
    /// as `gravity_confidence` falls.
    ///
    /// Under gravity the split is real — holding altitude is not the same job
    /// as crossing ground, and an actor may be authored to do one far harder
    /// than the other. In a null neither is a job at all, and keeping the
    /// split would only rotate the actor's acceleration away from the
    /// direction its intent asked for.
    #[must_use]
    pub fn converged_axes(&self, gravity_confidence: f32) -> ConvergedAxes {
        let isotropy = 1.0 - gravity_confidence.clamp(0.0, 1.0);
        let converge = |vertical: f32, horizontal: f32| {
            let mean = vertical.midpoint(horizontal);
            (
                vertical.lerp(mean, isotropy),
                horizontal.lerp(mean, isotropy),
            )
        };
        let (vertical_acceleration, horizontal_acceleration) = converge(
            self.acceleration * self.vertical_acceleration_ratio,
            self.acceleration,
        );
        let (vertical_max_speed, horizontal_max_speed) =
            converge(self.max_speed * self.vertical_speed_ratio, self.max_speed);
        ConvergedAxes {
            vertical_acceleration,
            horizontal_acceleration,
            vertical_max_speed,
            horizontal_max_speed,
        }
    }

    /// The world-space acceleration `intent` asks for, in `frame`, under a
    /// gravity of the given magnitude.
    ///
    /// This is exactly the thrust the controller applies to actors carrying a
    /// [`FlightOrientation`](crate::flight::FlightOrientation). `intent.y`
    /// thrusts along the frame's up, `intent.x` across it; components within
    /// the intent deadzone (`0.001`) are ignored. Each axis pushes at its
    /// (confidence-converged, see [`Self::converged_axes`]) acceleration
    /// while the speed already along it is under that axis' cap, and stops
    /// pushing at the cap rather than clamping — the actor coasts at the
    /// limit, and opposite intent still brakes past it.
    ///
    /// Upward intent additionally boosts the vertical acceleration by
    /// `gravity_length * `[`Self::gravity_compensation`], scaled by the
    /// clamped confidence — there is only as much gravity to compensate as
    /// there is trust in it. Pass `0.0` where no field applies.
    ///
    /// At zero `gravity_confidence` thrust is isotropic, so the arbitrary
    /// basis of [`FlightFrame::UNORIENTED`] cancels out of the round trip and
    /// the actor accelerates exactly along its intent.
    #[must_use]
    pub fn thrust(
        &self,
        intent: Vec2,
        frame: FlightFrame,
        velocity: Vec2,
        gravity_confidence: f32,
        gravity_length: f32,
    ) -> Vec2 {
        let ConvergedAxes {
            mut vertical_acceleration,
            horizontal_acceleration,
            vertical_max_speed,
            horizontal_max_speed,
        } = self.converged_axes(gravity_confidence);
        if intent.y > 0.0 {
            vertical_acceleration +=
                gravity_length * self.gravity_compensation * gravity_confidence.clamp(0.0, 1.0);
        }

        let speed = frame.project(velocity);
        frame.to_world(Vec2::new(
            axis_thrust(
                intent.x,
                speed.x,
                horizontal_acceleration,
                horizontal_max_speed,
            ),
            axis_thrust(intent.y, speed.y, vertical_acceleration, vertical_max_speed),
        ))
    }
}

/// The intent magnitude below which an axis is treated as at rest, matching
/// the thresholds of [`MovementIntent::is_flying`](crate::intent::MovementIntent::is_flying)
/// and [`MovementIntent::is_flying_horizontal`](crate::intent::MovementIntent::is_flying_horizontal).
const INTENT_DEADZONE: f32 = 0.001;

/// Acceleration along one frame axis: the authored value while there is
/// headroom under the cap, nothing once the actor is already at it.
fn axis_thrust(intent: f32, current_speed: f32, acceleration: f32, max_speed: f32) -> f32 {
    if intent.abs() <= INTENT_DEADZONE {
        return 0.0;
    }
    let has_headroom = if intent > 0.0 {
        current_speed < max_speed
    } else {
        current_speed > -max_speed
    };
    if has_headroom {
        intent * acceleration
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A flyer authored with a strong "vertical" thruster and a weak lateral
    /// one — the split that makes the frame matter.
    fn anisotropic_config() -> FlyingConfig {
        FlyingConfig {
            max_speed: 100.0,
            vertical_speed_ratio: 1.0,
            acceleration: 50.0,
            vertical_acceleration_ratio: 8.0, // 400.0 vertical
            gravity_compensation: 0.0,
            damping: 0.0,
        }
    }

    #[test]
    fn thrust_follows_the_frame_not_the_world_axes() {
        // Up is +X here, so "vertical" intent must push along +X, not along world +Y.
        let frame = FlightFrame::new(Dir2::X);
        let thrust = anisotropic_config().thrust(Vec2::Y, frame, Vec2::ZERO, 1.0, 0.0);

        assert!(
            thrust.abs_diff_eq(Vec2::X * 400.0, 1.0e-3),
            "expected thrust along the frame's up, got {thrust:?}"
        );
    }

    #[test]
    fn thrust_keeps_its_axes_apart_under_gravity() {
        // Full gravity: the authored split is the point, and a diagonal intent
        // is skewed toward the strong axis on purpose.
        let frame = FlightFrame::new(Dir2::Y);
        let intent = Vec2::new(1.0, 1.0).normalize();
        let thrust = anisotropic_config().thrust(intent, frame, Vec2::ZERO, 1.0, 0.0);

        assert!(
            thrust.y > thrust.x * 2.0,
            "the strong axis should dominate, got {thrust:?}"
        );
    }

    #[test]
    fn thrust_accelerates_along_the_intent_in_a_null() {
        // No gravity: no axis is privileged, so the actor must accelerate in
        // the direction it asked for rather than one the frame skewed.
        let frame = FlightFrame::new(Dir2::Y);
        let intent = Vec2::new(1.0, 1.0).normalize();
        let thrust = anisotropic_config().thrust(intent, frame, Vec2::ZERO, 0.0, 0.0);

        let skew = thrust.normalize_or_zero().angle_to(frame.to_world(intent));
        assert!(
            skew.abs() < 1.0e-3,
            "acceleration skewed by {skew} rad: {thrust:?}"
        );
    }

    #[test]
    fn thrust_isotropy_arrives_gradually() {
        // The split must close smoothly across the confidence ramp; a step
        // here would be a seam the player sees as a lurch on the way out of
        // the field.
        let frame = FlightFrame::new(Dir2::Y);
        let intent = Vec2::new(1.0, 1.0).normalize();
        let world_intent = frame.to_world(intent);

        let mut previous = f32::INFINITY;
        for step in 0..=10 {
            let confidence = 1.0 - step as f32 / 10.0;
            let thrust = anisotropic_config().thrust(intent, frame, Vec2::ZERO, confidence, 0.0);
            let skew = thrust.normalize_or_zero().angle_to(world_intent).abs();
            assert!(
                skew <= previous + 1.0e-3,
                "skew rose at confidence {confidence}"
            );
            previous = skew;
        }
        assert!(
            previous < 1.0e-3,
            "and it should close entirely, left at {previous}"
        );
    }

    #[test]
    fn thrust_stops_pushing_an_axis_that_is_already_at_its_cap() {
        let frame = FlightFrame::new(Dir2::Y);
        let config = anisotropic_config();
        let vertical_cap = config.max_speed * config.vertical_speed_ratio;
        let at_cap = frame.to_world(Vec2::new(0.0, vertical_cap));

        assert_eq!(config.thrust(Vec2::Y, frame, at_cap, 1.0, 0.0), Vec2::ZERO);
        // ...but the opposite intent still brakes it.
        assert!(config.thrust(-Vec2::Y, frame, at_cap, 1.0, 0.0).length() > 0.0);
    }

    #[test]
    fn thrust_ignores_intent_within_the_deadzone() {
        // Sub-deadzone analog noise must not thrust at all — with no damping
        // it would otherwise accelerate the actor indefinitely in a null.
        let frame = FlightFrame::new(Dir2::Y);
        let noise = Vec2::splat(0.0005);

        assert_eq!(
            anisotropic_config().thrust(noise, frame, Vec2::ZERO, 1.0, 0.0),
            Vec2::ZERO
        );
        assert_eq!(
            anisotropic_config().thrust(-noise, frame, Vec2::ZERO, 0.0, 0.0),
            Vec2::ZERO
        );
    }

    #[test]
    fn thrust_compensates_gravity_only_when_climbing() {
        let frame = FlightFrame::new(Dir2::Y);
        let config = FlyingConfig {
            gravity_compensation: 0.5,
            ..anisotropic_config()
        };
        let gravity_length = 980.0;
        let base = config.acceleration * config.vertical_acceleration_ratio;
        let boost = gravity_length * config.gravity_compensation;

        let up = config.thrust(Vec2::Y, frame, Vec2::ZERO, 1.0, gravity_length);
        assert!(
            up.abs_diff_eq(Vec2::Y * (base + boost), 1.0e-3),
            "upward thrust must carry the compensation boost, got {up:?}"
        );

        // Downward and lateral intent get no boost — there is no climb to help.
        let down = config.thrust(-Vec2::Y, frame, Vec2::ZERO, 1.0, gravity_length);
        assert!(
            down.abs_diff_eq(-Vec2::Y * base, 1.0e-3),
            "downward thrust must not be boosted, got {down:?}"
        );
    }

    #[test]
    fn gravity_compensation_fades_with_confidence() {
        // Half trust in the field means half as much gravity to compensate.
        let frame = FlightFrame::new(Dir2::Y);
        let config = FlyingConfig {
            gravity_compensation: 0.5,
            ..anisotropic_config()
        };
        let gravity_length = 980.0;

        let half = config.thrust(Vec2::Y, frame, Vec2::ZERO, 0.5, gravity_length);
        let dry = config.thrust(Vec2::Y, frame, Vec2::ZERO, 0.5, 0.0);
        let boost = frame.project(half - dry).y;
        assert!(
            (boost - gravity_length * config.gravity_compensation * 0.5).abs() < 1.0e-3,
            "the boost must scale with confidence, got {boost}"
        );
    }
}
