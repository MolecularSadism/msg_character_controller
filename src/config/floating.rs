//! Configuration for floating mechanics (hovering behavior).

use bevy::prelude::*;

/// Configuration for floating mechanics (hovering behavior).
#[derive(Reflect, Debug, Clone, Copy)]
pub struct FloatingConfig {
    /// Target height to float above ground (in world units/pixels).
    /// This is the distance from the BOTTOM of the collider to the ground.
    pub float_height: f32,

    /// Distance beyond `riding_height` where the character is still considered grounded.
    /// The spring remains active within this range, restoring the character to `riding_height`.
    pub grounding_distance: f32,

    /// Distance for wall and ceiling detection (extends from collider surface).
    pub surface_detection_distance: f32,

    /// Multiplier for downward spring force when character is within `grounding_distance`
    /// above the `riding_height`. This helps keep the character grounded when slightly floating.
    pub grounding_strength: f32,
}

impl Default for FloatingConfig {
    fn default() -> Self {
        Self {
            float_height: 6.0,
            grounding_distance: 4.0,
            surface_detection_distance: 2.0,
            grounding_strength: 3.0,
        }
    }
}
