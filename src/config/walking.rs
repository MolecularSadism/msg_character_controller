//! Configuration for ground movement.

use bevy::prelude::*;

/// Configuration for ground movement.
#[derive(Reflect, Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(default))]
pub struct WalkingConfig {
    /// Maximum horizontal movement speed (units/second).
    pub max_speed: f32,

    /// Horizontal acceleration rate (units/second^2).
    pub acceleration: f32,

    /// Friction/deceleration when no input (0.0-1.0).
    pub friction: f32,

    /// Air control multiplier (0.0-1.0).
    pub air_control: f32,

    /// Air friction/deceleration when airborne and no input (0.0-1.0).
    pub air_friction: f32,

    /// Whether the character can cling to walls by walking into them.
    pub wall_clinging: bool,

    /// Dampening applied to downward motion when clinging to a wall (0.0-1.0).
    pub wall_clinging_dampening: f32,

    /// Whether wall clinging dampening should also dampen upward movement.
    pub wall_clinging_dampen_upward: bool,

    /// Maximum slope angle the character can walk up (radians).
    pub max_slope_angle: f32,

    /// Extra downward force when walking uphill.
    pub uphill_gravity_multiplier: f32,

    /// Most force walking applies (mass × units/second²): a character accelerates at most
    /// `max_force / mass`, so a heavy one pushes no harder than a light one. `None` leaves
    /// [`Self::acceleration`] uncapped.
    pub max_force: Option<f32>,

    /// How far a partial walk intent scales the drive (0.0-1.0). At 1.0, accelerating toward
    /// the walk direction uses `acceleration × |intent|`, so a slow walk pushes softly; at 0.0
    /// every intent drives with the full acceleration. Braking always uses the full acceleration.
    pub intent_force_scaling: f32,
}

impl WalkingConfig {
    /// Walking acceleration for a character of `mass`: [`Self::acceleration`], capped by
    /// [`Self::max_force`].
    #[must_use]
    pub fn acceleration_for(&self, mass: f32) -> f32 {
        match self.max_force {
            Some(max_force) if mass > 0.0 => self.acceleration.min(max_force / mass),
            _ => self.acceleration,
        }
    }

    /// Share of the walking acceleration a speed change of `velocity_diff` gets under the walk
    /// intent `walk`: scaled by [`Self::intent_force_scaling`] when it drives toward the walk
    /// direction, whole when it brakes.
    #[must_use]
    pub fn intent_drive_factor(&self, walk: f32, velocity_diff: f32) -> f32 {
        let drives = walk.abs() > 0.001 && velocity_diff * walk > 0.0;
        if !drives {
            return 1.0;
        }
        let scaling = self.intent_force_scaling.clamp(0.0, 1.0);
        1.0 - scaling * (1.0 - walk.abs().min(1.0))
    }
}

impl Default for WalkingConfig {
    fn default() -> Self {
        Self {
            max_speed: 150.0,
            acceleration: 400.0,
            friction: 0.06,
            air_control: 0.15,
            air_friction: 0.02,
            wall_clinging: true,
            wall_clinging_dampening: 0.5,
            wall_clinging_dampen_upward: false,
            max_slope_angle: std::f32::consts::FRAC_PI_3,
            uphill_gravity_multiplier: 1.0,
            max_force: None,
            intent_force_scaling: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config(max_force: Option<f32>, intent_force_scaling: f32) -> WalkingConfig {
        WalkingConfig {
            acceleration: 400.0,
            max_force,
            intent_force_scaling,
            ..Default::default()
        }
    }

    #[test]
    fn max_force_caps_acceleration_by_mass() {
        let config = config(Some(1000.0), 0.0);
        assert_eq!(config.acceleration_for(10.0), 100.0);
        assert_eq!(
            config.acceleration_for(1.0),
            400.0,
            "a light character keeps the configured acceleration"
        );
    }

    #[test]
    fn without_max_force_acceleration_is_uncapped() {
        assert_eq!(config(None, 0.0).acceleration_for(1000.0), 400.0);
    }

    #[test]
    fn intent_scaling_softens_a_partial_drive() {
        let config = config(None, 1.0);
        assert_eq!(config.intent_drive_factor(0.25, 10.0), 0.25);
        assert_eq!(config.intent_drive_factor(-0.5, -10.0), 0.5);
        assert_eq!(config.intent_drive_factor(1.0, 10.0), 1.0);
    }

    #[test]
    fn intent_scaling_blends_with_its_setting() {
        assert_eq!(config(None, 0.5).intent_drive_factor(0.0, 0.0), 1.0);
        assert_eq!(config(None, 0.5).intent_drive_factor(0.2, 10.0), 0.6);
        assert_eq!(config(None, 0.0).intent_drive_factor(0.2, 10.0), 1.0);
    }

    #[test]
    fn braking_always_gets_the_full_acceleration() {
        let config = config(None, 1.0);
        assert_eq!(
            config.intent_drive_factor(0.0, -10.0),
            1.0,
            "no intent: braking"
        );
        assert_eq!(
            config.intent_drive_factor(0.25, -10.0),
            1.0,
            "faster than the walk asks: braking"
        );
    }
}
