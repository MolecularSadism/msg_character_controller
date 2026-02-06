//! Configuration for spring-damper system.

use bevy::prelude::*;

/// Configuration for spring-damper system.
#[derive(Reflect, Debug, Clone, Copy)]
pub struct SpringConfig {
    /// Spring strength for the floating system.
    pub strength: f32,

    /// Spring damping coefficient.
    pub damping: f32,

    /// Maximum spring force to apply (None = formula-based).
    pub max_force: Option<f32>,

    /// Maximum vertical velocity at which spring force is still applied.
    pub max_velocity: Option<f32>,

    /// Duration (seconds) after jump/upward propulsion during which downward spring forces are filtered.
    pub jump_filter_duration: f32,
}

impl Default for SpringConfig {
    fn default() -> Self {
        Self {
            strength: 300.0,
            damping: 13.0,
            max_force: Some(3000.0),
            max_velocity: None,
            jump_filter_duration: 0.15,
        }
    }
}
