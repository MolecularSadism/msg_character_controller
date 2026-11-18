//! Physics backend abstraction.
//!
//! This module defines the trait that physics backends must implement
//! to work with the character controller. This allows easy swapping
//! between physics engines (`Rapier2D`, XPBD, custom, etc.).

use bevy::prelude::*;

use crate::sensors::{SensorCastMode, SensorHit, ShapeCastQuery};

/// Trait for physics backend implementations.
///
/// Implement this trait to integrate a physics engine with the character
/// controller. The backend handles physics operations like force application
/// and velocity manipulation, and answers the controller's sensor casts.
/// Everything else, including how casts are aimed and how hits become floor,
/// wall, ceiling and step state, is backend-neutral (see [`crate::sensors`]),
/// so a backend can live outside this crate.
///
/// All methods take the [`World`] and run from exclusive systems on the main
/// thread, so a backend may keep a `!Send` physics world as a non-send
/// resource.
///
/// # Sensor casts
///
/// The minimum a backend provides for detection is [`Self::cast_shape`]. With
/// the default [`SensorCastMode::Immediate`] the controller calls it for every
/// enabled cast once per fixed step. A backend that keeps persistent casters
/// in its engine (Avian) returns [`SensorCastMode::Persistent`] and fills
/// readings itself; see [`crate::sensors`] for that data flow.
///
/// [`Avian2dBackend`](crate::backend::Avian2dBackend) is the in-crate
/// implementation; the `fake_backend` integration test is an engine-free one.
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

    /// Set the linear damping coefficient on an entity.
    ///
    /// Implementations should drive the physics backend's linear damping
    /// component directly (e.g. Avian's `LinearDamping`) rather than
    /// manipulating velocity manually, so damping composes correctly with
    /// impulses, forces, and integration.
    ///
    /// Default implementation is a no-op so backends without first-class
    /// damping support compile cleanly.
    fn set_linear_damping(_world: &mut World, _entity: Entity, _damping: f32) {}

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
    ///
    /// The sensor casts use `filters` as their mask unless
    /// `config.sensors.collision_mask` overrides it; with neither, every bit.
    fn get_collision_groups(_world: &World, _entity: Entity) -> Option<(u32, u32)> {
        // Default implementation returns None
        None
    }

    /// Get the collider bottom offset for an entity.
    /// This is the distance from the collider center to its bottom
    /// (capsule: half segment length + radius; circle: radius; box: half height).
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

    /// Get the side radius of an entity's collider: the distance from its
    /// center to its left or right side (capsule or circle: radius; box: half
    /// width). Sizes the wall and stair casts.
    ///
    /// Default implementation returns 0.
    fn get_collider_radius(_world: &World, _entity: Entity) -> f32 {
        0.0
    }

    /// Whether the entity's body is disabled (not simulated).
    ///
    /// A disabled character's sensor casts are held off and its contact state
    /// is left untouched, as with [`CasterDisabled`](crate::CasterDisabled).
    ///
    /// Default implementation returns false.
    fn is_body_disabled(_world: &World, _entity: Entity) -> bool {
        false
    }

    /// How this backend answers the controller's sensor casts.
    ///
    /// Default: [`SensorCastMode::Immediate`], the controller calls
    /// [`Self::cast_shape`].
    #[must_use]
    fn sensor_cast_mode() -> SensorCastMode {
        SensorCastMode::Immediate
    }

    /// Sweep a shape through the world and return the nearest hit.
    ///
    /// Contract:
    /// - Sweep `query.shape` (offsets from `query.origin`, world orientation)
    ///   from `query.origin` along `query.direction` for at most
    ///   `query.max_distance`.
    /// - Only colliders whose collision-group memberships share a bit with
    ///   `query.filter.mask` count.
    /// - Never report a collider of `query.filter.exclude` (the casting
    ///   character). Engines without per-entity query exclusion post-filter
    ///   their hits.
    /// - Of the remaining hits return the one with the smallest distance.
    ///   `distance` is how far the shape travelled (`fraction * max_distance`),
    ///   0 if it starts overlapping; `normal` is the outward surface normal of
    ///   the hit collider in world space; `entity` is the hit collider's entity.
    /// - Whether sensor (non-solid) colliders are reported is the backend's
    ///   choice. Avian reports them, as it always has; a new backend should
    ///   decide deliberately and document it.
    ///
    /// Called from an exclusive system on the main thread. Not called for
    /// [`SensorCastMode::Persistent`] backends.
    fn cast_shape(world: &World, query: &ShapeCastQuery) -> Option<SensorHit>;

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
