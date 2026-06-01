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

    /// Optional override for the collision mask used by all detection casters.
    ///
    /// Stored as raw collision-layer bits (the same representation as Avian's
    /// `LayerMask`). When `Some`, the casters query only colliders whose
    /// membership layers overlap these bits, **ignoring** whatever the parent
    /// actor's `CollisionLayers.filters` are set to. When `None` (the default),
    /// casters inherit the actor's filters as before.
    ///
    /// This lets the controller probe a dedicated set of "ground" layers
    /// (e.g. game-side `GroundPhysics` + `LooseGroundPhysics`) independently of
    /// the layers the actor's body physically collides with.
    pub collision_mask: Option<u32>,
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
            collision_mask: None,
        }
    }
}
