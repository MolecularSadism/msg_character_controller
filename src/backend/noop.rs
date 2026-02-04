use bevy::prelude::*;

/// Empty plugin for backends that don't need additional setup.
pub struct NoOpBackendPlugin;

impl Plugin for NoOpBackendPlugin {
    fn build(&self, _app: &mut App) {}
}
