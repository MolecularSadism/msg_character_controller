//! Configuration for rotation stabilization.

use bevy::prelude::*;

/// Configuration for rotation stabilization.
#[derive(Reflect, Debug, Clone, Copy)]
pub struct UprightTorqueConfig {
    /// Whether to apply torque to keep the character upright.
    pub enabled: bool,

    /// Strength of the upright torque spring.
    pub strength: f32,

    /// Damping coefficient for the upright torque.
    pub damping: f32,

    /// Target angle for upright torque (radians). None = derived from gravity.
    pub target_angle: Option<f32>,

    /// Maximum torque to apply for uprighting (None = formula-based).
    pub max_torque: Option<f32>,

    /// Maximum angular velocity at which torque is still applied.
    pub max_angular_velocity: Option<f32>,
}

impl Default for UprightTorqueConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            strength: 120.0,
            damping: 5.0,
            target_angle: None,
            max_torque: None,
            max_angular_velocity: Some(10.0),
        }
    }
}
