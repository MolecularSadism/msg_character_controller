//! Configuration for ground movement.

use bevy::prelude::*;

/// Configuration for ground movement.
#[derive(Reflect, Debug, Clone, Copy)]
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
        }
    }
}
