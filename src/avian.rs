//! Avian2D physics backend implementation.
//!
//! This module provides the physics backend for Avian2D (bevy_avian2d).
//! Enable with the `avian2d` feature.

use bevy::prelude::*;
use avian2d::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::collision::CollisionData;
use crate::config::{CharacterController, ControllerConfig};

/// Marker component for ground detection ShapeCaster.
#[derive(Component, Debug, Clone, Copy)]
pub struct GroundCaster;

/// Marker component for left wall detection ShapeCaster.
#[derive(Component, Debug, Clone, Copy)]
pub struct LeftWallCaster;

/// Marker component for right wall detection ShapeCaster.
#[derive(Component, Debug, Clone, Copy)]
pub struct RightWallCaster;

/// Marker component for ceiling detection ShapeCaster.
#[derive(Component, Debug, Clone, Copy)]
pub struct CeilingCaster;

/// Marker component for stair detection ShapeCaster (forward-down cast).
#[derive(Component, Debug, Clone, Copy)]
pub struct StairCaster;

/// Marker component for current ground detection ShapeCaster (for step height calculation).
#[derive(Component, Debug, Clone, Copy)]
pub struct CurrentGroundCaster;

/// Component linking a caster child entity back to its parent character.
#[derive(Component, Debug, Clone, Copy)]
pub struct CasterParent(pub Entity);

/// Marker component to track that casters have been spawned for this character.
#[derive(Component, Debug, Clone, Copy)]
pub struct CastersSpawned;

/// Message fired when a reactive force (Newton's 3rd law) is applied to an entity.
///
/// This message is used to track forces applied to ground entities so they can be
/// properly cleared in the next frame. Without this, forces would accumulate
/// indefinitely on entities that characters stand on.
#[derive(Message, Debug, Clone)]
pub struct ReactiveForceApplied {
    /// The entity that received the reactive force.
    pub entity: Entity,
    /// The force that was applied.
    pub force: Vec2,
}

/// Avian2D physics backend for the character controller.
///
/// This backend uses `avian2d` for physics operations including
/// force application and velocity manipulation. Collision detection
/// (shapecasting/raycasting) is handled by dedicated Avian systems
/// that use `ShapeCaster` components for deferred spatial queries.
pub struct Avian2dBackend;

impl CharacterPhysicsBackend for Avian2dBackend {
    type VelocityComponent = LinearVelocity;

    fn plugin() -> impl Plugin {
        Avian2dBackendPlugin
    }

    fn get_velocity(world: &World, entity: Entity) -> Vec2 {
        world
            .get::<LinearVelocity>(entity)
            .map(|v| v.0)
            .unwrap_or(Vec2::ZERO)
    }

    fn set_velocity(world: &mut World, entity: Entity, velocity: Vec2) {
        if let Some(mut vel) = world.get_mut::<LinearVelocity>(entity) {
            vel.0 = velocity;
        }
    }

    fn apply_impulse(world: &mut World, entity: Entity, impulse: Vec2) {
        // In Avian 0.4, impulses are applied directly to velocity
        // Impulse = mass * delta_v, so delta_v = impulse / mass
        let mass = Self::get_mass(world, entity);
        if mass <= 0.0 {
            return;
        }
        let delta_v = impulse / mass;
        if let Some(mut vel) = world.get_mut::<LinearVelocity>(entity) {
            vel.0 += delta_v;
        }
    }

    fn apply_force(world: &mut World, entity: Entity, force: Vec2) {
        // Accumulate into CharacterController instead of directly modifying forces.
        // Forces will be applied at the end of the frame by finalize_controller_forces.
        if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
            controller.add_force(force);
        }
    }

    fn apply_torque(world: &mut World, entity: Entity, torque: f32) {
        // Accumulate into CharacterController instead of directly modifying torque.
        // Torque will be applied at the end of the frame by finalize_controller_forces.
        if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
            controller.add_torque(torque);
        }
    }

    fn get_angular_velocity(world: &World, entity: Entity) -> f32 {
        world
            .get::<AngularVelocity>(entity)
            .map(|v| v.0)
            .unwrap_or(0.0)
    }

    fn get_rotation(world: &World, entity: Entity) -> f32 {
        // Try Avian's Rotation component first, then fall back to Transform
        world
            .get::<Rotation>(entity)
            .map(|r| r.as_radians())
            .or_else(|| {
                world.get::<Transform>(entity).map(|t| {
                    let (_, _, z) = t.rotation.to_euler(EulerRot::XYZ);
                    z
                })
            })
            .or_else(|| {
                world.get::<GlobalTransform>(entity).map(|t| {
                    let (_, rotation, _) = t.to_scale_rotation_translation();
                    let (_, _, z) = rotation.to_euler(EulerRot::XYZ);
                    z
                })
            })
            .unwrap_or(0.0)
    }

    fn get_gravity(_world: &World, _entity: Entity) -> Vec2 {
        // Gravity is stored in CharacterController.gravity
        Vec2::ZERO
    }

    fn get_position(world: &World, entity: Entity) -> Vec2 {
        // Try Avian's Position component first, then fall back to Transform
        world
            .get::<Position>(entity)
            .map(|p| p.0)
            .or_else(|| world.get::<Transform>(entity).map(|t| t.translation.xy()))
            .or_else(|| {
                world
                    .get::<GlobalTransform>(entity)
                    .map(|t| t.translation().xy())
            })
            .unwrap_or(Vec2::ZERO)
    }

    fn get_fixed_timestep(world: &World) -> f32 {
        world
            .get_resource::<Time<Fixed>>()
            .map(|t| t.delta_secs())
            .filter(|&d| d > 0.0)
            .unwrap_or(1.0 / 60.0)
    }

    fn get_collision_groups(world: &World, entity: Entity) -> Option<(u32, u32)> {
        world
            .get::<CollisionLayers>(entity)
            .map(|cl| (cl.memberships.0, cl.filters.0))
    }

    fn get_collider_bottom_offset(world: &World, entity: Entity) -> f32 {
        world
            .get::<Collider>(entity)
            .map(get_collider_bottom_offset)
            .unwrap_or(0.0)
    }

    fn get_mass(world: &World, entity: Entity) -> f32 {
        let Some(computed_mass) = world.get::<ComputedMass>(entity) else {
            return 0.0;
        };
        let mass = computed_mass.value();
        if mass <= 0.0 || !mass.is_finite() {
            return 0.0;
        }
        mass
    }

    fn get_principal_inertia(world: &World, entity: Entity) -> f32 {
        let Some(computed_inertia) = world.get::<ComputedAngularInertia>(entity) else {
            return 0.0;
        };
        let inertia = computed_inertia.value();
        if inertia <= 0.0 || !inertia.is_finite() {
            return 0.0;
        }
        inertia
    }

    fn is_rotation_locked(world: &World, entity: Entity) -> bool {
        world
            .get::<LockedAxes>(entity)
            .is_some_and(|axes| axes.is_rotation_locked())
    }

    fn provides_custom_gravity() -> bool {
        // Avian backend provides custom gravity systems that use Forces component
        true
    }
}

/// Plugin that sets up Avian2D-specific systems for the character controller.
pub struct Avian2dBackendPlugin;

impl Plugin for Avian2dBackendPlugin {
    fn build(&self, app: &mut App) {
        use crate::CharacterControllerSet;

        // Register the reactive force message
        app.add_message::<ReactiveForceApplied>();

        // Phase 1: Preparation - Clear forces and update caster components
        // First clear forces, then spawn casters for new controllers, then update all caster directions
        app.add_systems(
            FixedUpdate,
            (
                clear_controller_forces,
                clear_reactive_forces,
                spawn_detection_casters,
                update_ground_caster_direction,
                update_wall_caster_directions,
                update_ceiling_caster_direction,
                update_stair_casters,
            )
                .chain()
                .in_set(CharacterControllerSet::Preparation),
        );

        // Phase 3: Sensors - Read ShapeHits from caster components
        // Ground detection must run first because it sets collider_bottom_offset
        // which ceiling detection uses for capsule_half_height().
        // Wall, ceiling, and stair detection can run in parallel after ground detection.
        // Falling detection can run after collision detection to determine vertical movement.
        app.add_systems(
            FixedUpdate,
            (
                avian_ground_detection,
                (avian_wall_detection, avian_ceiling_detection, avian_stair_detection, avian_detect_falling),
            )
                .chain()
                .in_set(CharacterControllerSet::Sensors),
        );

        // Phase 4: Force Accumulation - Avian-specific gravity using Forces component
        app.add_systems(
            FixedUpdate,
            avian_accumulate_gravity.in_set(CharacterControllerSet::ForceAccumulation),
        );

        // Phase 5: Intent Application - Avian-specific fall gravity using Forces component
        app.add_systems(
            FixedUpdate,
            avian_apply_fall_gravity.in_set(CharacterControllerSet::IntentApplication),
        );

        // Phase 6: Final Application - Apply accumulated forces to physics
        app.add_systems(
            FixedUpdate,
            apply_controller_forces.in_set(CharacterControllerSet::FinalApplication),
        );
    }
}

/// Automatically spawn detection casters for new character controllers.
///
/// This system detects newly added CharacterController entities and spawns
/// all necessary ShapeCaster child entities for collision detection.
fn spawn_detection_casters(
    mut commands: Commands,
    q_new_controllers: Query<
        (Entity, &CharacterController, &ControllerConfig),
        (Without<CastersSpawned>, With<CharacterController>),
    >,
) {
    for (entity, controller, config) in &q_new_controllers {
        // Spawn ground caster
        spawn_ground_caster(&mut commands, entity, config);

        // Spawn wall casters
        spawn_wall_casters(&mut commands, entity, config);

        // Spawn ceiling caster
        spawn_ceiling_caster(&mut commands, entity, config);

        // Spawn stair casters if enabled
        if let Some(stair_config) = &controller.stair_config {
            if stair_config.enabled {
                spawn_stair_casters(&mut commands, entity, controller);
            }
        }

        // Mark this entity as having casters spawned
        commands.entity(entity).insert(CastersSpawned);
    }
}

/// Get the distance from collider center to bottom for a given collider.
/// For capsules, this is half_height + radius.
pub fn get_collider_bottom_offset(collider: &Collider) -> f32 {
    // Try to get capsule shape parameters
    if let Some(capsule) = collider.shape_scaled().as_capsule() {
        // Capsule: half-length of segment + radius
        // In Parry, capsule.segment is a field, and segment.a/segment.b are methods
        let segment = capsule.segment;
        let half_height = (segment.a.y - segment.b.y).abs() / 2.0;
        half_height + capsule.radius
    } else if let Some(ball) = collider.shape_scaled().as_ball() {
        // Ball: just the radius
        ball.radius
    } else if let Some(cuboid) = collider.shape_scaled().as_cuboid() {
        // Cuboid: half the height (y dimension)
        cuboid.half_extents.y
    } else {
        // Unknown shape: use 0 as fallback (float_height measured from center)
        0.0
    }
}

/// Spawn a ground detection ShapeCaster as a child of the character entity.
///
/// This function creates a child entity with a ShapeCaster component configured
/// for ground detection. The caster direction and distance will be updated
/// dynamically by the `update_ground_caster_direction` system.
///
/// Note: Collision layers are inherited from the parent in the update system.
pub fn spawn_ground_caster(commands: &mut Commands, parent: Entity, config: &ControllerConfig) {
    let half_width = config.ground_cast_width / 2.0;

    let child = commands.spawn((
        Name::new("GroundCaster"),
        GroundCaster,
        CasterParent(parent),
        ShapeCaster::new(
            Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
            Vec2::ZERO,
            0.0,
            Dir2::NEG_Y,
        ).with_max_distance(20.0)
         .with_max_hits(1)
         .with_ignore_self(true),
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    commands.entity(parent).add_child(child);
}

/// Spawn left wall detection ShapeCaster as a child of the character entity.
///
/// This function creates a child entity with a ShapeCaster component configured
/// for left wall detection. The caster direction will be updated dynamically
/// by the `update_wall_caster_directions` system.
///
/// Note: Collision layers are inherited from the parent in the update system.
pub fn spawn_left_wall_caster(commands: &mut Commands, parent: Entity, config: &ControllerConfig) {
    let half_height = config.wall_cast_height / 2.0;
    let child = commands.spawn((
        LeftWallCaster,
        CasterParent(parent),
        ShapeCaster::new(
            Collider::segment(Vec2::new(0.0, -half_height), Vec2::new(0.0, half_height)),
            Vec2::ZERO,
            0.0,
            Dir2::NEG_X,
        ).with_max_distance(10.0)
         .with_max_hits(1)
         .with_ignore_self(true),
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    commands.entity(parent).add_child(child);
}

/// Spawn right wall detection ShapeCaster as a child of the character entity.
///
/// This function creates a child entity with a ShapeCaster component configured
/// for right wall detection. The caster direction will be updated dynamically
/// by the `update_wall_caster_directions` system.
///
/// Note: Collision layers are inherited from the parent in the update system.
pub fn spawn_right_wall_caster(commands: &mut Commands, parent: Entity, config: &ControllerConfig) {
    let half_height = config.wall_cast_height / 2.0;
    let child = commands.spawn((
        RightWallCaster,
        CasterParent(parent),
        ShapeCaster::new(
            Collider::segment(Vec2::new(0.0, -half_height), Vec2::new(0.0, half_height)),
            Vec2::ZERO,
            0.0,
            Dir2::X,
        ).with_max_distance(10.0)
         .with_max_hits(1)
         .with_ignore_self(true),
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    commands.entity(parent).add_child(child);
}

/// Spawn both left and right wall casters.
///
/// Convenience function to spawn both wall casters at once.
pub fn spawn_wall_casters(commands: &mut Commands, parent: Entity, config: &ControllerConfig) {
    spawn_left_wall_caster(commands, parent, config);
    spawn_right_wall_caster(commands, parent, config);
}

/// Spawn ceiling detection ShapeCaster as a child of the character entity.
///
/// This function creates a child entity with a ShapeCaster component configured
/// for ceiling detection. The caster direction will be updated dynamically
/// by the `update_ceiling_caster_direction` system.
///
/// Note: Collision layers are inherited from the parent in the update system.
pub fn spawn_ceiling_caster(commands: &mut Commands, parent: Entity, config: &ControllerConfig) {
    let half_width = config.ceiling_cast_width / 2.0;
    let child = commands.spawn((
        CeilingCaster,
        CasterParent(parent),
        ShapeCaster::new(
            Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
            Vec2::ZERO,
            0.0,
            Dir2::Y,
        ).with_max_distance(10.0)
         .with_max_hits(1)
         .with_ignore_self(true),
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    commands.entity(parent).add_child(child);
}

/// Spawn stair detection ShapeCasters as children of the character entity.
///
/// This function creates two child entities:
/// - StairCaster: forward-down cast for detecting the step surface
/// - CurrentGroundCaster: downward cast for detecting current ground level
///
/// Both casters start disabled and are enabled/positioned dynamically by the
/// `update_stair_casters` system when the character is walking.
///
/// Note: Collision layers are inherited from the parent in the update system.
pub fn spawn_stair_casters(commands: &mut Commands, parent: Entity, controller: &CharacterController) {
    // Get stair config, or use default if not set
    let stair_config = controller.stair_config.as_ref()
        .cloned()
        .unwrap_or_default();

    let half_width = stair_config.stair_cast_width / 2.0;

    // Stair caster (forward-down detection) - starts disabled
    let stair_child = commands.spawn((
        StairCaster,
        CasterParent(parent),
        ShapeCaster::new(
            Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
            Vec2::ZERO,
            0.0,
            Dir2::NEG_Y,
        ).with_max_distance(stair_config.max_climb_height)
         .with_max_hits(1)
         .with_ignore_self(true)
         .disable(), // Start disabled
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    commands.entity(parent).add_child(stair_child);

    // Current ground caster (for step height reference) - starts disabled
    let ground_child = commands.spawn((
        CurrentGroundCaster,
        CasterParent(parent),
        ShapeCaster::new(
            Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
            Vec2::ZERO,
            0.0,
            Dir2::NEG_Y,
        ).with_max_distance(20.0)
         .with_max_hits(1)
         .with_ignore_self(true)
         .disable(), // Start disabled
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    commands.entity(parent).add_child(ground_child);
}

/// Update ground caster direction and configuration based on gravity.
///
/// This system runs in the Preparation phase, before Avian updates ShapeHits.
fn update_ground_caster_direction(
    mut q_casters: Query<(&CasterParent, &mut ShapeCaster), With<GroundCaster>>,
    q_controllers: Query<(&CharacterController, &ControllerConfig, Option<&CollisionLayers>)>,
) {
    for (caster_parent, mut shape_caster) in &mut q_casters {
        let Ok((controller, config, collision_layers)) = q_controllers.get(caster_parent.0) else {
            continue;
        };

        // Update direction to ideal_down (gravity-relative)
        let down = controller.ideal_down();
        if let Ok(dir) = Dir2::new(down) {
            shape_caster.direction = dir;
        }

        // Update shape rotation to align with ideal_up
        shape_caster.shape_rotation = controller.ideal_up_angle() - std::f32::consts::FRAC_PI_2;

        // Update max_distance to cover riding_height + grounding_distance + buffer
        let riding_height = controller.riding_height(config);
        shape_caster.max_distance = riding_height + config.grounding_distance + 1.0;

        // Inherit collision layers from parent
        if let Some(layers) = collision_layers {
            // Use the character's filters as the mask - this finds entities whose memberships
            // overlap with what the character is allowed to collide with
            shape_caster.query_filter = SpatialQueryFilter::from_mask(layers.filters);
        } else {
            // No collision layers specified - use default which includes all entities
            shape_caster.query_filter = SpatialQueryFilter::default();
        }

        // Exclude the parent entity from the cast
        shape_caster.query_filter.excluded_entities.insert(caster_parent.0);
    }
}

/// Update wall caster directions and configuration based on gravity.
///
/// This system runs in the Preparation phase, before Avian updates ShapeHits.
fn update_wall_caster_directions(
    mut q_left_casters: Query<(&CasterParent, &mut ShapeCaster), With<LeftWallCaster>>,
    mut q_right_casters: Query<(&CasterParent, &mut ShapeCaster), (With<RightWallCaster>, Without<LeftWallCaster>)>,
    q_controllers: Query<(&CharacterController, &ControllerConfig, Option<&CollisionLayers>, Option<&Collider>)>,
) {
    // Update left wall casters
    for (caster_parent, mut shape_caster) in &mut q_left_casters {
        let Ok((controller, config, collision_layers, collider)) = q_controllers.get(caster_parent.0) else {
            continue;
        };

        // Update direction to ideal_left (gravity-relative)
        let left = controller.ideal_left();
        if let Ok(dir) = Dir2::new(left) {
            shape_caster.direction = dir;
        }

        // Update shape rotation to align with ideal_up (vertical segment)
        shape_caster.shape_rotation = controller.ideal_up_angle();

        // Update max_distance: surface_detection_distance + radius + buffer
        let radius = collider.map(get_collider_radius).unwrap_or(0.0);
        shape_caster.max_distance = config.surface_detection_distance + radius + 1.0;

        // Inherit collision layers from parent
        if let Some(layers) = collision_layers {
            shape_caster.query_filter = SpatialQueryFilter::from_mask(layers.filters);
        } else {
            shape_caster.query_filter = SpatialQueryFilter::default();
        }

        // Exclude the parent entity from the cast
        shape_caster.query_filter.excluded_entities.insert(caster_parent.0);
    }

    // Update right wall casters
    for (caster_parent, mut shape_caster) in &mut q_right_casters {
        let Ok((controller, config, collision_layers, collider)) = q_controllers.get(caster_parent.0) else {
            continue;
        };

        // Update direction to ideal_right (gravity-relative)
        let right = controller.ideal_right();
        if let Ok(dir) = Dir2::new(right) {
            shape_caster.direction = dir;
        }

        // Update shape rotation to align with ideal_up (vertical segment)
        shape_caster.shape_rotation = controller.ideal_up_angle();

        // Update max_distance: surface_detection_distance + radius + buffer
        let radius = collider.map(get_collider_radius).unwrap_or(0.0);
        shape_caster.max_distance = config.surface_detection_distance + radius + 1.0;

        // Inherit collision layers from parent
        if let Some(layers) = collision_layers {
            shape_caster.query_filter = SpatialQueryFilter::from_mask(layers.filters);
        } else {
            shape_caster.query_filter = SpatialQueryFilter::default();
        }

        // Exclude the parent entity from the cast
        shape_caster.query_filter.excluded_entities.insert(caster_parent.0);
    }
}

/// Update ceiling caster direction and configuration based on gravity.
///
/// This system runs in the Preparation phase, before Avian updates ShapeHits.
fn update_ceiling_caster_direction(
    mut q_casters: Query<(&CasterParent, &mut ShapeCaster), With<CeilingCaster>>,
    q_controllers: Query<(&CharacterController, &ControllerConfig, Option<&CollisionLayers>)>,
) {
    for (caster_parent, mut shape_caster) in &mut q_casters {
        let Ok((controller, config, collision_layers)) = q_controllers.get(caster_parent.0) else {
            continue;
        };

        // Update direction to ideal_up (gravity-relative)
        let up = controller.ideal_up();
        if let Ok(dir) = Dir2::new(up) {
            shape_caster.direction = dir;
        }

        // Update shape rotation to align with ideal_up
        shape_caster.shape_rotation = controller.ideal_up_angle() - std::f32::consts::FRAC_PI_2;

        // Update max_distance: surface_detection_distance + capsule_half_height + buffer
        shape_caster.max_distance = config.surface_detection_distance + controller.capsule_half_height() + 1.0;

        // Inherit collision layers from parent
        if let Some(layers) = collision_layers {
            shape_caster.query_filter = SpatialQueryFilter::from_mask(layers.filters);
        } else {
            shape_caster.query_filter = SpatialQueryFilter::default();
        }

        // Exclude the parent entity from the cast
        shape_caster.query_filter.excluded_entities.insert(caster_parent.0);
    }
}

/// Update stair caster configuration based on movement intent.
///
/// This system runs in the Preparation phase, before Avian updates ShapeHits.
/// Stair casters are only enabled when the character is walking.
fn update_stair_casters(
    mut q_stair_casters: Query<(&CasterParent, &mut ShapeCaster), With<StairCaster>>,
    mut q_ground_casters: Query<(&CasterParent, &mut ShapeCaster), (With<CurrentGroundCaster>, Without<StairCaster>)>,
    q_controllers: Query<(
        &CharacterController,
        Option<&MovementIntent>,
        Option<&CollisionLayers>,
        Option<&Collider>,
    )>,
) {
    // Update stair casters
    for (caster_parent, mut shape_caster) in &mut q_stair_casters {
        let Ok((controller, movement_intent, collision_layers, collider)) = q_controllers.get(caster_parent.0) else {
            continue;
        };

        // Check if stair stepping is enabled and we have movement intent
        let stair_config = match &controller.stair_config {
            Some(config) if config.enabled => config,
            _ => {
                shape_caster.enabled = false;
                continue;
            }
        };

        let Some(intent) = movement_intent else {
            shape_caster.enabled = false;
            continue;
        };

        // Enable only when walking
        if intent.is_walking() {
            shape_caster.enabled = true;

            // Get ideal directions from gravity
            let down = controller.ideal_down();
            let right = controller.ideal_right();

            // Calculate forward offset based on walk direction
            let walk_direction = intent.walk;
            let move_dir = right * walk_direction.signum();

            // Get collider radius
            let radius = collider.map(get_collider_radius).unwrap_or(0.0);

            // Calculate horizontal offset: radius + stair_cast_offset
            let horizontal_offset = radius + stair_config.stair_cast_offset;

            // Calculate vertical offset: position at max_climb_height above feet
            // vertical_offset = up * (max_climb_height - collider_bottom_offset)
            let up = controller.ideal_up();
            let vertical_offset = up * (stair_config.max_climb_height - controller.collider_bottom_offset);

            // Set origin: forward_offset + vertical_offset
            let origin = move_dir * horizontal_offset + vertical_offset;
            shape_caster.origin = origin;

            // Update direction to ideal_down
            if let Ok(dir) = Dir2::new(down) {
                shape_caster.direction = dir;
            }

            // Update shape rotation to align with ideal_up
            shape_caster.shape_rotation = controller.ideal_up_angle() - std::f32::consts::FRAC_PI_2;

            // Update max_distance
            shape_caster.max_distance = stair_config.max_climb_height;

            // Inherit collision layers from parent
            if let Some(layers) = collision_layers {
                shape_caster.query_filter = SpatialQueryFilter::from_mask(layers.filters);
            } else {
                shape_caster.query_filter = SpatialQueryFilter::default();
            }

            // Exclude the parent entity from the cast
            shape_caster.query_filter.excluded_entities.insert(caster_parent.0);
        } else {
            shape_caster.enabled = false;
        }
    }

    // Update current ground casters
    for (caster_parent, mut shape_caster) in &mut q_ground_casters {
        let Ok((controller, movement_intent, collision_layers, _)) = q_controllers.get(caster_parent.0) else {
            continue;
        };

        // Check if stair stepping is enabled
        let stair_config = match &controller.stair_config {
            Some(config) if config.enabled => config,
            _ => {
                shape_caster.enabled = false;
                continue;
            }
        };

        let Some(intent) = movement_intent else {
            shape_caster.enabled = false;
            continue;
        };

        // Enable only when walking
        if intent.is_walking() {
            shape_caster.enabled = true;

            // Origin at character center
            shape_caster.origin = Vec2::ZERO;

            // Update direction to ideal_down
            let down = controller.ideal_down();
            if let Ok(dir) = Dir2::new(down) {
                shape_caster.direction = dir;
            }

            // Update shape rotation to align with ideal_up
            shape_caster.shape_rotation = controller.ideal_up_angle() - std::f32::consts::FRAC_PI_2;

            // Update max_distance to cover max_climb_height + float_height + tolerance + buffer
            shape_caster.max_distance = stair_config.max_climb_height + controller.collider_bottom_offset + stair_config.stair_tolerance + 2.0;

            // Inherit collision layers from parent
            if let Some(layers) = collision_layers {
                shape_caster.query_filter = SpatialQueryFilter::from_mask(layers.filters);
            } else {
                shape_caster.query_filter = SpatialQueryFilter::default();
            }

            // Exclude the parent entity from the cast
            shape_caster.query_filter.excluded_entities.insert(caster_parent.0);
        } else {
            shape_caster.enabled = false;
        }
    }
}

use crate::intent::MovementIntent;

/// Avian-specific ground detection system using ShapeCaster components.
///
/// This system reads ShapeHits from ground caster child entities to detect the floor.
/// The GroundCaster child entities must be spawned and updated by the update systems.
fn avian_ground_detection(
    q_casters: Query<(&CasterParent, &ShapeHits), With<GroundCaster>>,
    mut q_controllers: Query<(
        Entity,
        &GlobalTransform,
        &mut CharacterController,
        &ControllerConfig,
        Option<&Collider>,
    )>,
) {
    // First, reset detection state for all controllers
    for (_, _, mut controller, _, _) in &mut q_controllers {
        controller.reset_detection_state();
    }

    // Now process ground hits
    for (caster_parent, shape_hits) in &q_casters {
        let Ok((_, transform, mut controller, _config, collider)) = q_controllers.get_mut(caster_parent.0) else {
            continue;
        };

        // Update collider_bottom_offset from actual collider dimensions
        controller.collider_bottom_offset = collider.map(get_collider_bottom_offset).unwrap_or(0.0);

        // Get the first (closest) hit
        let Some(hit) = shape_hits.first() else {
            continue;
        };

        let position = transform.translation().xy();
        let up = controller.ideal_up();
        let down = controller.ideal_down();

        // Calculate slope angle from normal
        let normal = hit.normal1;
        let dot = normal.dot(up).clamp(-1.0, 1.0);
        let slope_angle = dot.acos();

        // Calculate hit point
        let hit_point = position + down * hit.distance;

        // Store floor collision data
        controller.floor = Some(CollisionData::new(
            hit.distance,
            normal,
            hit_point,
            Some(hit.entity),
        ));
        controller.slope_angle = slope_angle;
    }
}

/// Get the capsule radius from a collider.
fn get_collider_radius(collider: &Collider) -> f32 {
    if let Some(capsule) = collider.shape_scaled().as_capsule() {
        capsule.radius
    } else if let Some(ball) = collider.shape_scaled().as_ball() {
        ball.radius
    } else if let Some(cuboid) = collider.shape_scaled().as_cuboid() {
        // Use half the width as an approximation
        cuboid.half_extents.x
    } else {
        0.0
    }
}

/// Avian-specific wall detection system using ShapeCaster components.
///
/// This system reads ShapeHits from left and right wall caster child entities.
fn avian_wall_detection(
    q_left_casters: Query<(&CasterParent, &ShapeHits), With<LeftWallCaster>>,
    q_right_casters: Query<(&CasterParent, &ShapeHits), With<RightWallCaster>>,
    mut q_controllers: Query<(&GlobalTransform, &mut CharacterController)>,
) {
    // Process left wall hits
    for (caster_parent, shape_hits) in &q_left_casters {
        let Ok((transform, mut controller)) = q_controllers.get_mut(caster_parent.0) else {
            continue;
        };

        let Some(hit) = shape_hits.first() else {
            continue;
        };

        let position = transform.translation().xy();
        let left = controller.ideal_left();
        let normal = hit.normal1;
        let hit_point = position + left * hit.distance;

        controller.left_wall = Some(CollisionData::new(
            hit.distance,
            normal,
            hit_point,
            Some(hit.entity),
        ));
    }

    // Process right wall hits
    for (caster_parent, shape_hits) in &q_right_casters {
        let Ok((transform, mut controller)) = q_controllers.get_mut(caster_parent.0) else {
            continue;
        };

        let Some(hit) = shape_hits.first() else {
            continue;
        };

        let position = transform.translation().xy();
        let right = controller.ideal_right();
        let normal = hit.normal1;
        let hit_point = position + right * hit.distance;

        controller.right_wall = Some(CollisionData::new(
            hit.distance,
            normal,
            hit_point,
            Some(hit.entity),
        ));
    }
}

/// Avian-specific ceiling detection system using ShapeCaster components.
///
/// This system reads ShapeHits from ceiling caster child entities.
fn avian_ceiling_detection(
    q_casters: Query<(&CasterParent, &ShapeHits), With<CeilingCaster>>,
    mut q_controllers: Query<(&GlobalTransform, &mut CharacterController)>,
) {
    for (caster_parent, shape_hits) in &q_casters {
        let Ok((transform, mut controller)) = q_controllers.get_mut(caster_parent.0) else {
            continue;
        };

        let Some(hit) = shape_hits.first() else {
            continue;
        };

        let position = transform.translation().xy();
        let up = controller.ideal_up();
        let normal = hit.normal1;
        let hit_point = position + up * hit.distance;

        controller.ceiling = Some(CollisionData::new(
            hit.distance,
            normal,
            hit_point,
            Some(hit.entity),
        ));
    }
}

/// Avian-specific stair detection system using ShapeCaster components.
///
/// This system reads ShapeHits from stair caster child entities to calculate step height.
fn avian_stair_detection(
    q_stair_casters: Query<(&CasterParent, Option<&ShapeHits>, &ShapeCaster), With<StairCaster>>,
    q_ground_casters: Query<(&CasterParent, Option<&ShapeHits>, &ShapeCaster), (With<CurrentGroundCaster>, Without<StairCaster>)>,
    mut q_controllers: Query<(Entity, &GlobalTransform, &mut CharacterController)>,
) {
    for (entity, transform, mut controller) in &mut q_controllers {
        let position = transform.translation().xy();
        let up = controller.ideal_up();
        let down = controller.ideal_down();

        // Check if stair stepping is enabled
        let stair_config = match &controller.stair_config {
            Some(config) if config.enabled => config,
            _ => continue,
        };

        // Find stair caster hits and origin
        let mut stair_hit: Option<&ShapeHitData> = None;
        let mut stair_origin = Vec2::ZERO;
        for (caster_parent, shape_hits, shape_caster) in &q_stair_casters {
            if caster_parent.0 != entity {
                continue;
            }

            if let Some(hits) = shape_hits {
                stair_hit = hits.first();
                stair_origin = shape_caster.origin;
            }
            break;
        }

        // Find current ground caster hits and origin
        let mut ground_hit: Option<&ShapeHitData> = None;
        let mut ground_origin = Vec2::ZERO;
        for (caster_parent, shape_hits, shape_caster) in &q_ground_casters {
            if caster_parent.0 != entity {
                continue;
            }

            if let Some(hits) = shape_hits {
                ground_hit = hits.first();
                ground_origin = shape_caster.origin;
            }
            break;
        }

        // Calculate step height if both hits are present
        if let (Some(stair), Some(ground)) = (stair_hit, ground_hit) {
            // Calculate the actual hit points in world space
            // Stair caster casts from: position + stair_origin (forward + up offset)
            // Current ground caster casts from: position + ground_origin (character center)

            let stair_cast_origin = position + stair_origin;
            let ground_cast_origin = position + ground_origin;

            // Calculate the actual surface points
            let step_surface_point = stair_cast_origin + down * stair.distance;
            let current_ground_point = ground_cast_origin + down * ground.distance;

            // Calculate step height: how much higher is the step surface compared to current ground?
            // Use dot product with up direction to get the vertical component
            let step_height = (step_surface_point - current_ground_point).dot(up);

            // Check if the step is valid:
            // 1. Step height is above stair_tolerance (needs climbing, not just spring)
            // 2. Step height is below max_climb_height (climbable)
            // 3. Step surface is mostly horizontal (normal points up)

            let normal_up_component = stair.normal1.dot(up);

            if step_height > stair_config.stair_tolerance
                && step_height <= stair_config.max_climb_height
                && normal_up_component > 0.7
            {
                controller.step_detected = true;
                controller.step_height = step_height;
            } else {
                controller.step_detected = false;
                controller.step_height = 0.0;
            }
        } else {
            // No valid stair detected
            controller.step_detected = false;
            controller.step_height = 0.0;
        }
    }
}

/// Clear controller forces at the start of each frame.
///
/// This system runs BEFORE any controller force systems. It:
/// 1. Subtracts the forces we applied last frame from ConstantForce
/// 2. Clears the accumulators for the new frame
///
/// This ensures that external user forces are preserved while our forces
/// are "isolated" between frames.
pub fn clear_controller_forces(
    mut q: Query<(
        &mut CharacterController,
        Option<&mut ConstantForce>,
        Option<&mut ConstantTorque>,
    )>,
) {
    for (mut controller, constant_force, constant_torque) in &mut q {
        // Get the forces we applied last frame and clear for the new frame
        let (force_to_subtract, torque_to_subtract) = controller.prepare_new_frame();

        // Subtract our previously applied forces
        if let Some(mut force) = constant_force {
            force.0 -= force_to_subtract;
        }
        if let Some(mut torque) = constant_torque {
            torque.0 -= torque_to_subtract;
        }
    }
}

/// Clear reactive forces (ground reaction forces) from the previous frame.
///
/// This system runs in the Preparation phase alongside clear_controller_forces.
/// It reads ReactiveForceApplied messages from the previous frame and subtracts
/// those forces from the corresponding entities' ConstantForce components.
///
/// This ensures that ground entities don't accumulate forces indefinitely
/// when characters stand on them.
pub fn clear_reactive_forces(
    mut messages: MessageReader<ReactiveForceApplied>,
    mut forces: Query<&mut ConstantForce>,
) {
    const EPSILON: f32 = 1e-6;

    for msg in messages.read() {
        let Ok(mut force) = forces.get_mut(msg.entity) else {
            continue;
        };

        // Subtract the force that was applied last frame
        force.0 -= msg.force;

        // Clean up near-zero values to avoid floating point drift
        if force.0.x.abs() < EPSILON {
            force.0.x = 0.0;
        }
        if force.0.y.abs() < EPSILON {
            force.0.y = 0.0;
        }
    }
}

/// Detect falling state by checking vertical velocity.
///
/// This system runs early to set the `falling` flag on the CharacterController,
/// which is then used by fall gravity and other systems. This avoids query conflicts
/// with the Forces API that needs mutable access to velocity.
pub fn avian_detect_falling(
    mut query: Query<(&mut CharacterController, &LinearVelocity)>,
) {
    for (mut controller, velocity) in &mut query {
        let up = controller.ideal_up();
        let vertical_velocity = velocity.0.dot(up);
        controller.falling = vertical_velocity < 0.0;
    }
}

/// Apply gravity acceleration to airborne characters using Avian's Forces API.
///
/// This is the Avian-specific gravity system that uses the `Forces` QueryData
/// with `apply_linear_acceleration()`. This is the idiomatic Avian 0.4+ way to
/// apply non-persistent forces that get cleared automatically each frame.
///
/// This approach supports dynamic gravity that can change every frame (e.g., for
/// spherical planets with radial gravity).
pub fn avian_accumulate_gravity(
    mut query: Query<(Forces, &CharacterController, &ControllerConfig)>,
) {
    for (mut forces, controller, config) in &mut query {
        // Only apply gravity to airborne entities
        if !controller.is_grounded(config) {
            // Apply gravity as linear acceleration
            // Forces API handles mass multiplication and integration automatically
            forces.apply_linear_acceleration(controller.gravity);
        }
    }
}

/// Apply fall gravity for early jump cancellation using Avian's Forces API.
///
/// This system enables players to cancel jumps early by releasing the jump button,
/// making jumps feel less floaty. Fall gravity is triggered when:
///
/// 1. We jumped recently (within `jump_cancel_window`)
/// 2. AND either:
///    - Jump button is not held (player let go)
///    - OR we're moving downward (crossed the zenith)
///
/// When triggered, fall gravity is applied for `fall_gravity_duration`,
/// multiplying gravity by `fall_gravity`.
///
/// Note: This system applies additional gravity on top of the base gravity
/// applied by `avian_accumulate_gravity`. Both use the Forces API.
pub fn avian_apply_fall_gravity(
    mut query: Query<(
        Forces,
        &mut CharacterController,
        &ControllerConfig,
        &MovementIntent,
    )>,
) {
    for (mut forces, mut controller, config, intent) in &mut query {
        // Only process airborne entities with fall_gravity > 1.0
        if controller.is_grounded(config) || config.fall_gravity <= 1.0 {
            continue;
        }

        // Check if we should trigger fall gravity
        // Uses controller.falling (set by avian_detect_falling system) to detect
        // when we've crossed the jump apex, avoiding query conflicts with Forces.
        let max_ascent_expired = controller.jump_max_ascent_expired();
        let should_trigger = controller.in_jump_cancel_window()
            && !controller.recently_jumped()
            && (!intent.jump_pressed || controller.falling || max_ascent_expired);

        // Trigger fall gravity if conditions are met
        if should_trigger && !controller.fall_gravity_active() {
            controller.trigger_fall_gravity(config.fall_gravity_duration);
        }

        // Apply fall gravity if the timer is active
        if controller.fall_gravity_active() {
            // Apply additional gravity acceleration
            // The regular gravity system applies: gravity
            // We want to add: gravity * (fall_gravity - 1)
            // This way total becomes: gravity * fall_gravity
            let fall_multiplier = config.fall_gravity - 1.0;
            forces.apply_linear_acceleration(controller.gravity * fall_multiplier);
        }
    }
}

/// Apply controller forces at the end of each frame.
///
/// This system runs AFTER all controller force systems. It:
/// 1. Applies accumulated forces to ConstantForce/ConstantTorque
/// 2. Stores what we applied for next frame's subtraction
/// 3. Applies ground reaction forces to dynamic ground bodies
/// 4. Fires ReactiveForceApplied messages for ground reaction forces
///
/// This ensures our forces are integrated by Avian's physics step.
pub fn apply_controller_forces(
    mut q: Query<(
        &mut CharacterController,
        Option<&mut ConstantForce>,
        Option<&mut ConstantTorque>,
    )>,
    mut ground_forces: Query<(&mut ConstantForce, &RigidBody), Without<CharacterController>>,
    mut reactive_messages: MessageWriter<ReactiveForceApplied>,
) {
    for (mut controller, constant_force, constant_torque) in &mut q {
        // Get accumulated forces and prepare for next frame
        let (force_to_apply, torque_to_apply) = controller.finalize_frame();

        // Apply our accumulated forces
        if let Some(mut force) = constant_force {
            force.0 += force_to_apply;
        }
        if let Some(mut torque) = constant_torque {
            torque.0 += torque_to_apply;
        }

        // Apply ground reaction force to dynamic ground bodies
        if let Some((ground_entity, reaction_force)) = controller.take_ground_reaction() {
            if let Ok((mut ground_force, rigid_body)) = ground_forces.get_mut(ground_entity) {
                // Only apply reaction force to dynamic bodies
                if *rigid_body == RigidBody::Dynamic {
                    ground_force.0 += reaction_force;

                    // Fire message so the force can be cleared next frame
                    reactive_messages.write(ReactiveForceApplied {
                        entity: ground_entity,
                        force: reaction_force,
                    });
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CharacterControllerPlugin;

    fn create_test_app() -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(bevy::transform::TransformPlugin);
        // Insert SceneSpawner resource required by Avian's ColliderHierarchyPlugin
        app.insert_resource(bevy::scene::SceneSpawner::default());
        app.add_plugins(PhysicsPlugins::default());
        app.insert_resource(Time::<Fixed>::from_hz(60.0));
        app.finish();
        app.cleanup();
        app
    }

    #[test]
    fn avian_backend_get_position() {
        let mut app = create_test_app();

        let entity = app
            .world_mut()
            .spawn((
                Transform::from_xyz(100.0, 200.0, 0.0),
                RigidBody::Dynamic,
            ))
            .id();

        app.update();

        let pos = Avian2dBackend::get_position(app.world(), entity);
        assert!((pos.x - 100.0).abs() < 0.01);
        assert!((pos.y - 200.0).abs() < 0.01);
    }

    #[test]
    fn avian_backend_velocity() {
        let mut app = create_test_app();

        let entity = app
            .world_mut()
            .spawn((
                Transform::default(),
                RigidBody::Dynamic,
                LinearVelocity(Vec2::new(50.0, 30.0)),
            ))
            .id();

        app.update();

        let vel = Avian2dBackend::get_velocity(app.world(), entity);
        assert!((vel.x - 50.0).abs() < 0.01);
        assert!((vel.y - 30.0).abs() < 0.01);

        Avian2dBackend::set_velocity(app.world_mut(), entity, Vec2::new(100.0, 0.0));

        let vel = Avian2dBackend::get_velocity(app.world(), entity);
        assert!((vel.x - 100.0).abs() < 0.01);
        assert!(vel.y.abs() < 0.01);
    }

    fn create_test_app_with_controller() -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(bevy::transform::TransformPlugin);
        app.insert_resource(bevy::scene::SceneSpawner::default());
        app.add_plugins(PhysicsPlugins::default());
        app.add_plugins(CharacterControllerPlugin::<Avian2dBackend>::default());
        app.insert_resource(Time::<Fixed>::from_hz(60.0));
        app.finish();
        app.cleanup();
        app
    }

    #[test]
    fn character_controller_requires_physics_components() {
        let mut app = create_test_app_with_controller();

        let entity = app
            .world_mut()
            .spawn((
                Transform::default(),
                CharacterController::new(),
                ControllerConfig::default(),
                Collider::capsule(4.0, 8.0),
                GravityScale(0.0),
            ))
            .id();

        app.update();

        // CharacterController #[require] should have inserted these
        assert!(app.world().get::<RigidBody>(entity).is_some());
        assert!(app.world().get::<LinearVelocity>(entity).is_some());
        assert!(app.world().get::<ConstantForce>(entity).is_some());
        assert!(app.world().get::<ConstantTorque>(entity).is_some());
        assert!(app.world().get::<LockedAxes>(entity).is_some());
    }
}
