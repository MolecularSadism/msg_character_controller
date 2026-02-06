//! Configuration for flying propulsion.

use bevy::prelude::*;

/// Configuration for flying propulsion.
#[derive(Reflect, Debug, Clone, Copy)]
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
}

impl Default for FlyingConfig {
    fn default() -> Self {
        Self {
            max_speed: 150.0,
            vertical_speed_ratio: 0.6,
            acceleration: 500.0,
            vertical_acceleration_ratio: 0.6,
            gravity_compensation: 0.05,
        }
    }
}
