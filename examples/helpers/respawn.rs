//! Respawn helper for examples.
//!
//! Provides a reusable function for respawning the player at a given position,
//! resetting all physics and controller state.

use bevy::prelude::*;
use msg_character_controller::prelude::*;

/// Respawns the player at the given position, resetting controller state.
///
/// This function handles the backend-agnostic parts of respawning:
/// - Sets the transform to the given spawn position (with z=1.0) and resets rotation
/// - Resets the character controller (preserving gravity)
/// - Clears movement intent (including any pending jump request)
///
/// The caller is responsible for resetting physics-specific components
/// (velocity, external forces/impulses) which vary by backend.
///
/// # Arguments
///
/// * `spawn_pos` - The 2D position to respawn at (will be extended to z=1.0)
/// * `transform` - The entity's transform component
/// * `controller` - The entity's character controller
/// * `movement_intent` - The entity's movement intent component
pub fn respawn_player(
    spawn_pos: Vec2,
    transform: &mut Transform,
    controller: &mut CharacterController,
    movement_intent: &mut MovementIntent,
) {
    // Reset position to spawn point
    transform.translation = spawn_pos.extend(1.0);
    transform.rotation = Quat::IDENTITY;

    // Reset controller state (keep gravity)
    let gravity = controller.gravity;
    *controller = CharacterController::with_gravity(gravity);

    // Reset movement intent and jump request
    movement_intent.clear();
    movement_intent.clear_jump_request();
}
