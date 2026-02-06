//! Configuration for wall jump mechanics.

use bevy::prelude::*;

/// Configuration for wall jump mechanics.
#[derive(Reflect, Debug, Clone, Copy)]
pub struct WallJumpingConfig {
    /// Whether wall jumping is enabled.
    pub enabled: bool,

    /// Angle of wall jump from vertical (in radians).
    pub angle: f32,

    /// How much downward velocity should be compensated on wall jumps (0.0-1.0).
    pub velocity_compensation: f32,

    /// Whether wall jumps should retain the same upward height as normal jumps.
    pub retain_height: bool,

    /// Duration (seconds) after a wall jump during which movement toward the wall is blocked.
    pub movement_block_duration: f32,
}

impl Default for WallJumpingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            angle: std::f32::consts::FRAC_PI_4,
            velocity_compensation: 0.5,
            retain_height: true,
            movement_block_duration: 0.15,
        }
    }
}
