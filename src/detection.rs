//! Detection result structures.
//!
//! These structures hold the results of physics queries (raycasts) used
//! for ground detection, wall detection, and stair stepping.

use bevy::prelude::*;

/// Information about a raycast/shapecast result.
#[derive(Debug, Clone, Copy, Default)]
pub struct SensorCast {
    /// Whether the raycast hit something.
    pub hit: bool,
    /// Distance to the hit point (if hit).
    pub distance: f32,
    /// Normal of the surface at hit point.
    pub normal: Vec2,
    /// World position of the hit point.
    pub point: Vec2,
    /// Entity that was hit (if any).
    pub entity: Option<Entity>,
}

impl SensorCast {
    /// Create an empty (no hit) result.
    pub fn miss() -> Self {
        Self::default()
    }

    /// Create a hit result.
    pub fn hit(distance: f32, normal: Vec2, point: Vec2, entity: Option<Entity>) -> Self {
        Self {
            hit: true,
            distance,
            normal,
            point,
            entity,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_cast_miss() {
        let cast = SensorCast::miss();
        assert!(!cast.hit);
        assert_eq!(cast.distance, 0.0);
        assert!(cast.entity.is_none());
    }

    #[test]
    fn sensor_cast_hit() {
        let cast = SensorCast::hit(5.0, Vec2::Y, Vec2::new(10.0, 0.0), None);
        assert!(cast.hit);
        assert_eq!(cast.distance, 5.0);
        assert_eq!(cast.normal, Vec2::Y);
        assert_eq!(cast.point, Vec2::new(10.0, 0.0));
    }

    #[test]
    fn sensor_cast_with_entity() {
        let entity = Entity::from_raw(42);
        let cast = SensorCast::hit(3.0, Vec2::X, Vec2::ZERO, Some(entity));
        assert!(cast.hit);
        assert_eq!(cast.entity, Some(entity));
    }
}
