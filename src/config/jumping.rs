//! Configuration for core jump mechanics.

use bevy::prelude::*;

/// Configuration for core jump mechanics.
#[derive(Reflect, Debug, Clone, Copy)]
pub struct JumpingConfig {
    /// Jump impulse strength (applied as velocity).
    pub speed: f32,

    /// Coyote time duration in seconds.
    pub coyote_time: f32,

    /// Jump buffer duration in seconds.
    pub buffer_time: f32,

    /// Gravity multiplier when jump is cancelled early.
    pub fall_gravity: f32,

    /// Duration (seconds) after jumping during which the jump can be cancelled.
    pub cancel_window: f32,

    /// Duration (seconds) for which fall gravity is applied after cancellation.
    pub fall_gravity_duration: f32,

    /// Duration (seconds) after a jump during which the character is considered
    /// to have "recently jumped" (blocks fall gravity and coyote timer reset).
    pub recently_jumped_duration: f32,

    /// Maximum duration (seconds) of jump ascent before fall gravity is forced.
    pub max_ascent_duration: f32,

    /// How much pre-existing upward velocity reduces the jump impulse (0.0-1.0).
    pub upward_velocity_compensation: f32,
}

impl Default for JumpingConfig {
    fn default() -> Self {
        Self {
            speed: 120.0,
            coyote_time: 0.15,
            buffer_time: 0.1,
            fall_gravity: 3.0,
            cancel_window: 2.0,
            fall_gravity_duration: 0.15,
            recently_jumped_duration: 0.15,
            max_ascent_duration: 0.45,
            upward_velocity_compensation: 1.0,
        }
    }
}
