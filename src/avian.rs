//! Avian2D physics backend implementation.
//!
//! This module provides the physics backend for Avian2D (bevy_avian2d).
//! Enable with the `avian2d` feature.

use bevy::prelude::*;
use avian2d::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::collision::CollisionData;
use crate::config::{CharacterController, ControllerConfig, StairConfig};

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
/// that use `SpatialQuery` as a system parameter.
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
}

/// Plugin that sets up Avian2D-specific systems for the character controller.
pub struct Avian2dBackendPlugin;

impl Plugin for Avian2dBackendPlugin {
    fn build(&self, app: &mut App) {
        use crate::CharacterControllerSet;

        // Register the reactive force message
        app.add_message::<ReactiveForceApplied>();

        // Phase 1: Preparation - Clear forces from previous frame
        // Both controller forces and reactive forces (ground reactions) are cleared here
        app.add_systems(
            FixedUpdate,
            (clear_controller_forces, clear_reactive_forces)
                .in_set(CharacterControllerSet::Preparation),
        );

        // Phase 3: Sensors - Avian-specific detection systems
        // Ground detection must run first because it sets collider_bottom_offset
        // which ceiling detection uses for capsule_half_height().
        // Wall and ceiling detection can run in parallel after ground detection.
        app.add_systems(
            FixedUpdate,
            (
                avian_ground_detection,
                (avian_wall_detection, avian_ceiling_detection),
            )
                .chain()
                .in_set(CharacterControllerSet::Sensors),
        );

        // Phase 6: Final Application - Apply accumulated forces to physics
        app.add_systems(
            FixedUpdate,
            apply_controller_forces.in_set(CharacterControllerSet::FinalApplication),
        );
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

// Avian-specific detection systems that use SpatialQuery as a system parameter

/// Perform a shapecast using SpatialQuery.
fn avian_shapecast(
    spatial_query: &SpatialQuery,
    origin: Vec2,
    direction: Vec2,
    max_distance: f32,
    shape_width: f32,
    shape_height: f32,
    shape_rotation: f32,
    exclude_entity: Entity,
    collision_layers: Option<CollisionLayers>,
) -> Option<CollisionData> {
    // Determine if we're creating a horizontal or vertical segment based on dimensions
    let shape = if shape_height > shape_width {
        // Vertical segment (for wall detection)
        let half_height = shape_height / 2.0;
        Collider::segment(Vec2::new(0.0, -half_height), Vec2::new(0.0, half_height))
    } else {
        // Horizontal segment (for ground/ceiling detection)
        let half_width = shape_width / 2.0;
        Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0))
    };

    // Create filter to exclude the casting entity and respect collision layers
    let filter = if let Some(layers) = collision_layers {
        // Use the character's filters as the mask - this finds entities whose memberships
        // overlap with what the character is allowed to collide with
        SpatialQueryFilter::from_mask(layers.filters).with_excluded_entities([exclude_entity])
    } else {
        // No collision layers specified - use default which includes all entities
        SpatialQueryFilter::default().with_excluded_entities([exclude_entity])
    };

    // Create shape cast config
    let config = ShapeCastConfig::from_max_distance(max_distance);

    // Perform the shapecast
    spatial_query
        .cast_shape(
            &shape,
            origin,
            shape_rotation,
            Dir2::new(direction).unwrap_or(Dir2::NEG_Y),
            &config,
            &filter,
        )
        .map(|hit| {
            // Extract normal from hit (normal1 is the surface normal on the hit shape)
            let normal = hit.normal1;
            // Calculate hit point
            let hit_point = origin + direction * hit.distance;
            CollisionData::new(hit.distance, normal, hit_point, Some(hit.entity))
        })
}

use crate::intent::MovementIntent;

/// Avian-specific ground detection system using shapecast.
///
/// Floor raycast covers: riding_height + grounding_distance
/// (which is float_height + capsule_half_height + grounding_distance)
///
/// **Important**: Raycasts use the "ideal up" direction derived from gravity,
/// NOT from the actor's Transform rotation. This ensures ground detection
/// works correctly even when the actor is physically rotated.
fn avian_ground_detection(
    spatial_query: SpatialQuery,
    mut q_controllers: Query<(
        Entity,
        &GlobalTransform,
        &ControllerConfig,
        Option<&MovementIntent>,
        &mut CharacterController,
        Option<&CollisionLayers>,
        Option<&Collider>,
    )>,
) {
    for (entity, transform, config, movement_intent, mut controller, collision_layers, collider) in
        &mut q_controllers
    {
        let position = transform.translation().xy();

        // Get collider radius for stair detection offset
        let radius = collider.map(get_collider_radius).unwrap_or(0.0);

        // Update collider_bottom_offset from actual collider dimensions
        controller.collider_bottom_offset = collider.map(get_collider_bottom_offset).unwrap_or(0.0);

        // Use ideal up/down direction from gravity, NOT from orientation or transform.
        // This ensures raycasts work correctly regardless of actor rotation.
        let down = controller.ideal_down();
        let up = controller.ideal_up();

        // Clone collision layers for the shapecast
        let collision_layers_clone = collision_layers.cloned();

        // Calculate ground cast length:
        // riding_height + grounding_distance = float_height + capsule_half_height + grounding_distance
        // Add a small buffer to avoid edge cases with floating point precision
        let riding_height = controller.riding_height(config);
        let ground_cast_length = riding_height + config.grounding_distance + 1.0;

        // Compute rotation angle for the shape to align with ideal up direction (from gravity).
        // This ensures the shapecast shape is oriented correctly in world space.
        let shape_rotation = controller.ideal_up_angle() - std::f32::consts::FRAC_PI_2;

        // Reset detection state
        controller.reset_detection_state();

        // Perform shapecast for ground detection
        if let Some(ground_hit) = avian_shapecast(
            &spatial_query,
            position,
            down,
            ground_cast_length,
            config.ground_cast_width,
            0.0, // height not used for ground detection
            shape_rotation,
            entity,
            collision_layers_clone.clone(),
        ) {
            let normal = ground_hit.normal;
            let dot = normal.dot(up).clamp(-1.0, 1.0);
            let slope_angle = dot.acos();

            // Store floor collision data
            controller.floor = Some(ground_hit);
            controller.slope_angle = slope_angle;
        }

        // Check for stairs if enabled and we have movement intent
        if let (Some(stair), Some(intent)) = (controller.stair_config.as_ref(), movement_intent) {
            if stair.enabled && intent.is_walking() {
                if let Some(step_height) = check_stair_step(
                    &spatial_query,
                    entity,
                    position,
                    radius,
                    config.float_height,
                    intent,
                    &controller,
                    stair,
                    collision_layers_clone,
                ) {
                    controller.step_detected = true;
                    controller.step_height = step_height;
                }
            }
        }
    }
}

/// Check for a climbable stair in front of the character using movement intent.
///
/// This function casts a shapecast downward from a position in front of the character
/// (in the direction of movement intent) to detect steps.
///
/// The cast origin is positioned at `max_climb_height` above the player's feet (bottom
/// of collider), and casts downward to the feet level. This allows detecting steps
/// even when the stair surface would be above the player's center.
///
/// **Important**: Uses the "ideal up" direction derived from gravity, NOT from
/// the actor's Transform rotation.
///
/// Returns Some(step_height) if a climbable stair is detected, where step_height is
/// the height above the current ground level that needs to be climbed.
fn check_stair_step(
    spatial_query: &SpatialQuery,
    entity: Entity,
    position: Vec2,
    collider_radius: f32,
    float_height: f32,
    intent: &MovementIntent,
    controller: &CharacterController,
    config: &StairConfig,
    collision_layers: Option<CollisionLayers>,
) -> Option<f32> {
    // Use ideal directions from gravity, NOT from orientation or transform.
    // This ensures stair detection works correctly regardless of actor rotation.
    let down = controller.ideal_down();
    let up = controller.ideal_up();
    let right = controller.ideal_right();

    // Determine movement direction from intent
    let walk_direction = intent.walk;
    if walk_direction.abs() < 0.001 {
        return None; // No horizontal intent
    }

    let move_dir = right * walk_direction.signum();

    // Calculate the cast origin:
    // - Horizontal: offset in front of the player by (radius + stair_cast_offset)
    // - Vertical: at max_climb_height above the player's feet (bottom of collider)
    //   - Feet are at: position + down * collider_bottom_offset
    //   - Origin should be: feet + up * max_climb_height
    //     = position + down * collider_bottom_offset + up * max_climb_height
    //     = position + up * (max_climb_height - collider_bottom_offset)
    //   This can place the origin above player center when max_climb_height > collider_bottom_offset
    let horizontal_offset = collider_radius + config.stair_cast_offset;
    let vertical_offset = up * (config.max_climb_height - controller.collider_bottom_offset);
    let cast_origin = position + move_dir * horizontal_offset + vertical_offset;

    // Cast distance: from origin (at max_climb_height above feet) down to the feet level
    let cast_distance = config.max_climb_height;

    // Use shapecast for more reliable detection
    // Shape rotation based on ideal up direction from gravity
    let shape_rotation = controller.ideal_up_angle() - std::f32::consts::FRAC_PI_2;

    let stair_hit = avian_shapecast(
        spatial_query,
        cast_origin,
        down,
        cast_distance,
        config.stair_cast_width,
        0.0, // height not used for downward detection
        shape_rotation,
        entity,
        collision_layers.clone(),
    )?;

    // Calculate the ground height at the stair position relative to our current height
    // stair_hit.distance is how far down from cast_origin we hit
    // The step surface is at: cast_origin + down * stair_hit.distance
    let step_surface_point = cast_origin + down * stair_hit.distance;

    // Get the current ground level under the character (from center position)
    // Use a longer cast distance here to ensure we detect the ground below
    let ground_cast_distance =
        config.max_climb_height + float_height + config.stair_tolerance + 2.0;
    let current_ground_hit = avian_shapecast(
        spatial_query,
        position,
        down,
        ground_cast_distance,
        config.stair_cast_width,
        0.0, // height not used for ground detection
        shape_rotation,
        entity,
        collision_layers,
    );

    let current_ground_distance = current_ground_hit.map(|h| h.distance).unwrap_or(f32::MAX);

    // Calculate step height: how much higher is the step surface compared to our current ground?
    // The step is at cast_origin + down * stair_hit.distance
    // Our ground is at position + down * current_ground_distance
    // The height difference in the "up" direction:
    let current_ground_point = position + down * current_ground_distance;
    let step_height_in_up = (step_surface_point - current_ground_point).dot(up);

    // step_height is positive when the step surface is above current ground
    // (the dot product with up is positive when step is higher)
    let step_height = step_height_in_up;

    // Check if the step height is within climbable range:
    // - Higher than float_height + tolerance (needs climbing, not just spring)
    // - Lower than max_climb_height (not too high to climb)
    if step_height > float_height + config.stair_tolerance && step_height <= config.max_climb_height
    {
        // Also verify the step has adequate depth (horizontal surface)
        // Check that the normal is mostly upward (floor surface, not wall)
        let normal_up_component = stair_hit.normal.dot(up);
        if normal_up_component > 0.7 {
            return Some(step_height);
        }
    }

    None
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

/// Avian-specific wall detection system using shapecast.
///
/// Wall cast length: surface_detection_distance + radius
///
/// **Important**: Raycasts use the "ideal up" direction derived from gravity,
/// NOT from the actor's Transform rotation. This ensures wall detection
/// works correctly even when the actor is physically rotated.
fn avian_wall_detection(
    spatial_query: SpatialQuery,
    mut q_controllers: Query<(
        Entity,
        &GlobalTransform,
        &ControllerConfig,
        &mut CharacterController,
        Option<&CollisionLayers>,
        Option<&Collider>,
    )>,
) {
    for (entity, transform, config, mut controller, collision_layers, collider) in
        &mut q_controllers
    {
        let position = transform.translation().xy();

        // Use ideal directions from gravity, NOT from orientation or transform.
        // This ensures wall detection works correctly regardless of actor rotation.
        let left = controller.ideal_left();
        let right = controller.ideal_right();

        // Clone collision layers
        let collision_layers_clone = collision_layers.cloned();

        // Compute rotation angle for the shape based on ideal up direction from gravity
        let shape_rotation = controller.ideal_up_angle();

        // Wall cast length: surface_detection_distance + radius + small buffer for precision
        let radius = collider.map(get_collider_radius).unwrap_or(0.0);
        let wall_cast_length = config.surface_detection_distance + radius + 1.0;

        // Shapecast left
        if let Some(left_hit) = avian_shapecast(
            &spatial_query,
            position,
            left,
            wall_cast_length,
            0.0, // width not used for wall detection
            config.wall_cast_height,
            shape_rotation,
            entity,
            collision_layers_clone.clone(),
        ) {
            controller.left_wall = Some(left_hit);
        }

        // Shapecast right
        if let Some(right_hit) = avian_shapecast(
            &spatial_query,
            position,
            right,
            wall_cast_length,
            0.0, // width not used for wall detection
            config.wall_cast_height,
            shape_rotation,
            entity,
            collision_layers_clone,
        ) {
            controller.right_wall = Some(right_hit);
        }
    }
}

/// Avian-specific ceiling detection system using shapecast.
///
/// Ceiling cast length: surface_detection_distance + capsule_half_height
///
/// **Important**: Raycasts use the "ideal up" direction derived from gravity,
/// NOT from the actor's Transform rotation. This ensures ceiling detection
/// works correctly even when the actor is physically rotated.
fn avian_ceiling_detection(
    spatial_query: SpatialQuery,
    mut q_controllers: Query<(
        Entity,
        &GlobalTransform,
        &ControllerConfig,
        &mut CharacterController,
        Option<&CollisionLayers>,
    )>,
) {
    for (entity, transform, config, mut controller, collision_layers) in &mut q_controllers {
        let position = transform.translation().xy();

        // Use ideal up direction from gravity, NOT from orientation or transform.
        // This ensures ceiling detection works correctly regardless of actor rotation.
        let up = controller.ideal_up();

        // Clone collision layers
        let collision_layers_clone = collision_layers.cloned();

        // Shape rotation based on ideal up direction from gravity
        let shape_rotation = controller.ideal_up_angle() - std::f32::consts::FRAC_PI_2;

        // Ceiling cast length: surface_detection_distance + capsule_half_height + small buffer for precision
        let ceiling_cast_length =
            config.surface_detection_distance + controller.capsule_half_height() + 1.0;

        // Shapecast upward
        if let Some(ceiling_hit) = avian_shapecast(
            &spatial_query,
            position,
            up,
            ceiling_cast_length,
            config.ceiling_cast_width,
            0.0, // height not used for ceiling detection
            shape_rotation,
            entity,
            collision_layers_clone,
        ) {
            controller.ceiling = Some(ceiling_hit);
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
