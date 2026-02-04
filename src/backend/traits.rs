//! Physics backend abstraction.
//!
//! This module defines the trait that physics backends must implement
//! to work with the character controller. This allows easy swapping
//! between physics engines (`Rapier2D`, XPBD, custom, etc.).

use bevy::prelude::*;

/// Trait for physics backend implementations.
///
/// Implement this trait to integrate a physics engine with the character
/// controller. The backend handles physics operations like force application
/// and velocity manipulation.
///
/// # Example
///
/// For an example implementation, see the `rapier` module's `Rapier2dBackend`
/// which implements this trait for Bevy `Rapier2D`.
///
/// ```rust
/// use bevy::prelude::*;
/// use msg_character_controller::prelude::*;
///
/// // The trait is implemented by physics backends like Rapier2dBackend
/// #[cfg(feature = "rapier2d")]
/// fn example_usage() {
///     // Access backend methods statically
///     // In practice, these are called by the controller systems
/// }
/// ```
pub trait CharacterPhysicsBackend: 'static + Send + Sync {
    /// The velocity component type used by this backend.
    type VelocityComponent: Component;

    /// Returns the plugin that sets up this backend.
    fn plugin() -> impl Plugin;

    /// Get the current velocity of an entity.
    fn get_velocity(world: &World, entity: Entity) -> Vec2;

    /// Set the velocity of an entity.
    fn set_velocity(world: &mut World, entity: Entity, velocity: Vec2);

    /// Apply an impulse to an entity.
    ///
    /// Impulse is an instantaneous change in momentum (velocity).
    fn apply_impulse(world: &mut World, entity: Entity, impulse: Vec2);

    /// Apply a force to an entity.
    ///
    /// Force is applied over the physics timestep.
    fn apply_force(world: &mut World, entity: Entity, force: Vec2);

    /// Apply a torque to an entity.
    ///
    /// Torque is applied over the physics timestep. Positive values rotate
    /// counter-clockwise, negative values rotate clockwise.
    fn apply_torque(world: &mut World, entity: Entity, torque: f32);

    /// Get the current angular velocity of an entity.
    fn get_angular_velocity(world: &World, entity: Entity) -> f32;

    /// Get the current rotation angle of an entity (in radians).
    fn get_rotation(world: &World, entity: Entity) -> f32;

    /// Get the gravity vector for an entity.
    ///
    /// Returns the gravity that affects this entity. For spherical worlds,
    /// this may vary based on position.
    fn get_gravity(world: &World, entity: Entity) -> Vec2;

    /// Get the current position of an entity.
    fn get_position(world: &World, entity: Entity) -> Vec2;

    /// Get the fixed timestep delta time.
    fn get_fixed_timestep(world: &World) -> f32;

    /// Get the collision groups for an entity (memberships, filters).
    /// Returns None if the entity doesn't have collision groups.
    fn get_collision_groups(_world: &World, _entity: Entity) -> Option<(u32, u32)> {
        // Default implementation returns None
        None
    }

    /// Get the collider bottom offset for an entity.
    /// This is the distance from the collider center to its bottom.
    fn get_collider_bottom_offset(_world: &World, _entity: Entity) -> f32 {
        // Default implementation returns 0
        0.0
    }

    /// Get the mass of an entity.
    ///
    /// Used to scale forces so that config parameters produce consistent
    /// acceleration regardless of actual body mass.
    ///
    /// # Panics
    /// Panics if the entity does not have valid mass properties.
    fn get_mass(world: &World, entity: Entity) -> f32;

    /// Get the principal moment of inertia of an entity.
    ///
    /// In 2D this is a scalar. Used to scale torques so that config parameters
    /// produce consistent angular acceleration regardless of actual body inertia.
    ///
    /// # Panics
    /// Panics if the entity does not have valid inertia properties.
    fn get_principal_inertia(world: &World, entity: Entity) -> f32;

    /// Check if rotation is locked for an entity.
    ///
    /// When rotation is locked, the upright torque system should skip the entity
    /// since applying torque would have no effect.
    ///
    /// Default implementation returns false (rotation allowed).
    fn is_rotation_locked(_world: &World, _entity: Entity) -> bool {
        false
    }

    /// Indicates whether this backend provides custom gravity systems.
    ///
    /// If true, the backend's plugin should register its own gravity systems,
    /// and the generic gravity systems will not be registered by the main plugin.
    ///
    /// This allows backends to use physics-engine-specific approaches (like Avian's
    /// Forces component) instead of the generic impulse-based gravity.
    ///
    /// Default implementation returns false (use generic gravity systems).
    #[must_use] 
    fn provides_custom_gravity() -> bool {
        false
    }
}

/// Empty plugin for backends that don't need additional setup.
#[allow(dead_code)]
pub struct NoOpBackendPlugin;

impl Plugin for NoOpBackendPlugin {
    fn build(&self, _app: &mut App) {}
}
