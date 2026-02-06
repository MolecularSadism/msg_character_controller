//! Configuration for ShapeCaster sensor dimensions.

use bevy::prelude::*;

/// Configuration for ShapeCaster sensor dimensions.
#[derive(Reflect, Debug, Clone, Copy)]
pub struct SensorConfig {
    /// Ground cast length multiplier (multiplied by float_height).
    pub ground_cast_multiplier: f32,

    /// Width of the ground detection shapecast.
    pub ground_cast_width: f32,

    /// Wall cast length multiplier (multiplied by ground_cast_width).
    pub wall_cast_multiplier: f32,

    /// Height of wall detection shapecasts.
    pub wall_cast_height: f32,

    /// Ceiling cast length multiplier (multiplied by float_height).
    pub ceiling_cast_multiplier: f32,

    /// Width of ceiling detection shapecast.
    pub ceiling_cast_width: f32,
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            ground_cast_multiplier: 1.0,
            ground_cast_width: 11.0,
            wall_cast_multiplier: 1.0,
            wall_cast_height: 12.0,
            ceiling_cast_multiplier: 1.0,
            ceiling_cast_width: 12.0,
        }
    }
}
