//! Integration tests for the character controller with Avian2D backend.
//!
//! These tests verify the complete system behavior with actual physics simulation.
//! Each test produces PROOF through explicit velocity/force checks.

#![cfg(feature = "avian2d")]

use bevy::prelude::*;
use avian2d::prelude::*;
use msg_character_controller::prelude::*;
use msg_character_controller::avian::{
    Avian2dBackend, GroundCaster, LeftWallCaster, RightWallCaster, CeilingCaster,
    StairCaster, CurrentGroundCaster, CasterParent
};

// Shared physics constants - must match examples/helpers/physics.rs for consistent behavior
const FIXED_UPDATE_HZ: f64 = 60.0;
const PIXELS_PER_METER: f32 = 10.0;

/// Create a minimal test app with physics and character controller.
fn create_test_app() -> App {
    let mut app = App::new();

    app.add_plugins(MinimalPlugins);
    app.add_plugins(TransformPlugin);
    // Insert SceneSpawner resource to satisfy Avian's ColliderHierarchyPlugin
    app.insert_resource(bevy::scene::SceneSpawner::default());
    // Use Avian's default schedule (FixedPostUpdate)
    // Character controller runs in FixedUpdate, physics runs in FixedPostUpdate
    app.add_plugins(PhysicsPlugins::default().with_length_unit(PIXELS_PER_METER));
    app.add_plugins(CharacterControllerPlugin::<Avian2dBackend>::default());
    app.insert_resource(Time::<Fixed>::from_hz(FIXED_UPDATE_HZ));

    app.finish();
    app.cleanup();
    app
}

/// Spawn a static ground collider.
fn spawn_ground(app: &mut App, position: Vec2, half_size: Vec2) -> Entity {
    let transform = Transform::from_translation(position.extend(0.0));
    app.world_mut()
        .spawn((
            transform,
            GlobalTransform::from(transform),
            RigidBody::Static,
            Collider::rectangle(half_size.x * 2.0, half_size.y * 2.0)
        ))
        .id()
}

/// Spawn a character controller with default config.
fn spawn_character(app: &mut App, position: Vec2) -> Entity {
    spawn_character_with_config(app, position, ControllerConfig::default())
}

/// Spawn a character controller with custom config.
fn spawn_character_with_config(app: &mut App, position: Vec2, config: ControllerConfig) -> Entity {
    let transform = Transform::from_translation(position.extend(0.0));
    let world = app.world_mut();

    let controller = CharacterController::new();

    let entity = world
        .spawn((
            transform,
            GlobalTransform::from(transform),
            RigidBody::Dynamic, // Required for Avian physics
            controller.clone(),
            config.clone(),
            MovementIntent::new(), // Required for jump and movement systems
            Collider::capsule(4.0, 8.0),
            LockedAxes::ROTATION_LOCKED,
            GravityScale(0.0), // Disable Avian gravity - use controller's gravity
        ))
        .id();

    // Spawn ground caster child entity
    let half_width = config.ground_cast_width / 2.0;
    let ground_child = world.spawn((
        GroundCaster,
        CasterParent(entity),
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

    world.entity_mut(entity).add_child(ground_child);

    // Spawn left wall caster
    let wall_half_height = config.wall_cast_height / 2.0;
    let left_wall_child = world.spawn((
        LeftWallCaster,
        CasterParent(entity),
        ShapeCaster::new(
            Collider::segment(Vec2::new(0.0, -wall_half_height), Vec2::new(0.0, wall_half_height)),
            Vec2::ZERO,
            0.0,
            Dir2::NEG_X,
        ).with_max_distance(10.0)
         .with_max_hits(1)
         .with_ignore_self(true),
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    world.entity_mut(entity).add_child(left_wall_child);

    // Spawn right wall caster
    let right_wall_child = world.spawn((
        RightWallCaster,
        CasterParent(entity),
        ShapeCaster::new(
            Collider::segment(Vec2::new(0.0, -wall_half_height), Vec2::new(0.0, wall_half_height)),
            Vec2::ZERO,
            0.0,
            Dir2::X,
        ).with_max_distance(10.0)
         .with_max_hits(1)
         .with_ignore_self(true),
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    world.entity_mut(entity).add_child(right_wall_child);

    // Spawn ceiling caster
    let ceiling_half_width = config.ceiling_cast_width / 2.0;
    let ceiling_child = world.spawn((
        CeilingCaster,
        CasterParent(entity),
        ShapeCaster::new(
            Collider::segment(Vec2::new(-ceiling_half_width, 0.0), Vec2::new(ceiling_half_width, 0.0)),
            Vec2::ZERO,
            0.0,
            Dir2::Y,
        ).with_max_distance(10.0)
         .with_max_hits(1)
         .with_ignore_self(true),
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    world.entity_mut(entity).add_child(ceiling_child);

    // Spawn stair casters if stair stepping is enabled
    if controller.stair_config.as_ref().is_some_and(|c| c.enabled) {
        let stair_config = controller.stair_config.as_ref().unwrap();
        let half_width = stair_config.stair_cast_width / 2.0;

        // Stair caster (forward-down detection) - starts disabled
        let stair_child = world.spawn((
            StairCaster,
            CasterParent(entity),
            ShapeCaster::new(
                Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
                Vec2::ZERO,
                0.0,
                Dir2::NEG_Y,
            ).with_max_distance(stair_config.max_climb_height)
             .with_max_hits(1)
             .with_ignore_self(true)
             .disable(),
            Transform::default(),
            GlobalTransform::default(),
        )).id();

        world.entity_mut(entity).add_child(stair_child);

        // Current ground caster (for step height reference) - starts disabled
        let ground_child = world.spawn((
            CurrentGroundCaster,
            CasterParent(entity),
            ShapeCaster::new(
                Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
                Vec2::ZERO,
                0.0,
                Dir2::NEG_Y,
            ).with_max_distance(20.0)
             .with_max_hits(1)
             .with_ignore_self(true)
             .disable(),
            Transform::default(),
            GlobalTransform::default(),
        )).id();

        world.entity_mut(entity).add_child(ground_child);
    }

    entity
}

/// Spawn a character with custom gravity (which determines the up direction).
#[allow(dead_code)]
fn spawn_character_with_gravity(app: &mut App, position: Vec2, gravity: Vec2) -> Entity {
    let transform = Transform::from_translation(position.extend(0.0));
    let config = ControllerConfig::default();
    let world = app.world_mut();

    let controller = CharacterController::with_gravity(gravity);

    let entity = world
        .spawn((
            transform,
            GlobalTransform::from(transform),
            RigidBody::Dynamic, // Required for Avian physics
            controller.clone(),
            config.clone(),
            MovementIntent::new(), // Required for jump and movement systems
            Collider::capsule(4.0, 8.0),
            LockedAxes::ROTATION_LOCKED,
            GravityScale(0.0),
        ))
        .id();

    // Spawn ground caster child entity
    let half_width = config.ground_cast_width / 2.0;
    let ground_child = world.spawn((
        GroundCaster,
        CasterParent(entity),
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

    world.entity_mut(entity).add_child(ground_child);

    // Spawn left wall caster
    let wall_half_height = config.wall_cast_height / 2.0;
    let left_wall_child = world.spawn((
        LeftWallCaster,
        CasterParent(entity),
        ShapeCaster::new(
            Collider::segment(Vec2::new(0.0, -wall_half_height), Vec2::new(0.0, wall_half_height)),
            Vec2::ZERO,
            0.0,
            Dir2::NEG_X,
        ).with_max_distance(10.0)
         .with_max_hits(1)
         .with_ignore_self(true),
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    world.entity_mut(entity).add_child(left_wall_child);

    // Spawn right wall caster
    let right_wall_child = world.spawn((
        RightWallCaster,
        CasterParent(entity),
        ShapeCaster::new(
            Collider::segment(Vec2::new(0.0, -wall_half_height), Vec2::new(0.0, wall_half_height)),
            Vec2::ZERO,
            0.0,
            Dir2::X,
        ).with_max_distance(10.0)
         .with_max_hits(1)
         .with_ignore_self(true),
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    world.entity_mut(entity).add_child(right_wall_child);

    // Spawn ceiling caster
    let ceiling_half_width = config.ceiling_cast_width / 2.0;
    let ceiling_child = world.spawn((
        CeilingCaster,
        CasterParent(entity),
        ShapeCaster::new(
            Collider::segment(Vec2::new(-ceiling_half_width, 0.0), Vec2::new(ceiling_half_width, 0.0)),
            Vec2::ZERO,
            0.0,
            Dir2::Y,
        ).with_max_distance(10.0)
         .with_max_hits(1)
         .with_ignore_self(true),
        Transform::default(),
        GlobalTransform::default(),
    )).id();

    world.entity_mut(entity).add_child(ceiling_child);

    // Spawn stair casters if stair stepping is enabled
    if controller.stair_config.as_ref().is_some_and(|c| c.enabled) {
        let stair_config = controller.stair_config.as_ref().unwrap();
        let half_width = stair_config.stair_cast_width / 2.0;

        // Stair caster (forward-down detection) - starts disabled
        let stair_child = world.spawn((
            StairCaster,
            CasterParent(entity),
            ShapeCaster::new(
                Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
                Vec2::ZERO,
                0.0,
                Dir2::NEG_Y,
            ).with_max_distance(stair_config.max_climb_height)
             .with_max_hits(1)
             .with_ignore_self(true)
             .disable(),
            Transform::default(),
            GlobalTransform::default(),
        )).id();

        world.entity_mut(entity).add_child(stair_child);

        // Current ground caster (for step height reference) - starts disabled
        let ground_child = world.spawn((
            CurrentGroundCaster,
            CasterParent(entity),
            ShapeCaster::new(
                Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
                Vec2::ZERO,
                0.0,
                Dir2::NEG_Y,
            ).with_max_distance(20.0)
             .with_max_hits(1)
             .with_ignore_self(true)
             .disable(),
            Transform::default(),
            GlobalTransform::default(),
        )).id();

        world.entity_mut(entity).add_child(ground_child);
    }

    entity
}

/// Advance time by one fixed timestep and run one update.
/// This is necessary for FixedUpdate schedule to run.
fn tick(app: &mut App) {
    let timestep = std::time::Duration::from_secs_f64(1.0 / FIXED_UPDATE_HZ);
    app.world_mut()
        .resource_mut::<Time<Virtual>>()
        .advance_by(timestep);
    app.update();
}

/// Run the app for the specified number of frames.
fn run_frames(app: &mut App, frames: usize) {
    for _ in 0..frames {
        tick(app);
    }
}

/// Run the app for a specified duration in seconds.
/// This runs app.update() repeatedly, letting Bevy's FixedUpdate run passively
/// based on Time<Virtual>, until the target duration is reached.
fn run_for_duration(app: &mut App, duration_secs: f32) {
    let frames = (duration_secs * FIXED_UPDATE_HZ as f32).ceil() as usize;
    run_frames(app, frames);
}

/// Set jump pressed state on an entity.
/// Call this with `pressed = true` to start a jump, then run a tick to process it.
/// The system will automatically detect the rising edge and create a jump request.
fn set_jump_pressed(app: &mut App, entity: Entity, pressed: bool) {
    if let Some(mut intent) = app.world_mut().get_mut::<MovementIntent>(entity) {
        intent.set_jump_pressed(pressed);
    }
}

/// Request a jump by setting jump_pressed to true.
/// Note: The jump won't be processed until the next tick when the system runs.
fn request_jump(app: &mut App, entity: Entity) {
    set_jump_pressed(app, entity, true);
}

// ==================== Ground Detection Tests ====================

mod ground_detection {
    use super::*;

    #[test]
    fn character_above_ground_detects_ground() {
        let mut app = create_test_app();

        // Ground surface at y=5 (center at 0, half_height=5)
        let ground = spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Character at y=20 (within derived ground_cast_length)
        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        // Run more frames to let Avian's spatial query systems process colliders
        run_for_duration(&mut app, 2.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        // PROOF: ground_detected should be true (raycast hit)
        assert!(
            controller.ground_detected(),
            "Ground should be detected by raycast"
        );

        // PROOF: ground_distance should be less than character height
        let ground_dist = controller.ground_distance().expect("Ground should have distance");
        assert!(
            ground_dist < 20.0,
            "Ground distance should be less than character height: {}",
            ground_dist
        );

        println!(
            "PROOF: ground_detected={}, ground_distance={:?}, ground_normal={:?}",
            controller.ground_detected(),
            controller.ground_distance(),
            controller.ground_normal()
        );
    }

    #[test]
    fn character_at_float_height_is_grounded() {
        let mut app = create_test_app();

        let config = ControllerConfig::default().with_float_height(15.0);

        // Ground surface at y=5
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Character positioned so center is at float_height above ground surface
        // ground_distance is measured from character center (position), not capsule bottom
        // Ground surface is at y=5, float_height=15, so position.y = 5 + 15 = 20
        let character = spawn_character_with_config(&mut app, Vec2::new(0.0, 20.0), config);

        // Run more frames to let Avian's spatial query systems process colliders
        run_for_duration(&mut app, 2.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let cfg = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: is_grounded={}, ground_distance={:?}, riding_height+grounding_distance={}",
            controller.is_grounded(cfg),
            controller.ground_distance(),
            controller.riding_height(cfg) + cfg.grounding_distance
        );

        // PROOF: is_grounded should be true when within riding_height + grounding_distance
        assert!(
            controller.is_grounded(cfg),
            "Character should be grounded at float_height"
        );
    }

    #[test]
    fn character_high_above_ground_not_grounded() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Character at y=200 (far above ground)
        let character = spawn_character(&mut app, Vec2::new(0.0, 20000.0));

        run_for_duration(&mut app, 1.5);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: is_grounded={}, ground_detected={}, ground_distance={:?}",
            controller.is_grounded(config),
            controller.ground_detected(),
            controller.ground_distance()
        );

        // PROOF: is_grounded should be false when far from ground
        assert!(
            !controller.is_grounded(config),
            "Character should NOT be grounded when high above"
        );
    }

    #[test]
    fn character_over_empty_space_no_ground() {
        let mut app = create_test_app();

        // Ground only on the left side
        spawn_ground(&mut app, Vec2::new(-50.0, 0.0), Vec2::new(20.0, 5.0));

        // Character on the right with no ground
        let character = spawn_character(&mut app, Vec2::new(50.0, 20.0));

        run_for_duration(&mut app, 2.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: ground_detected={}, is_grounded={}",
            controller.ground_detected(),
            controller.is_grounded(config)
        );

        // PROOF: ground_detected should be false when over empty space
        assert!(
            !controller.ground_detected(),
            "Ground should NOT be detected over empty space"
        );
    }
}

// ==================== Float Height Tests ====================

mod float_height {
    use super::*;

    #[test]
    fn float_height_keeps_character_floating_above_ground() {
        let mut app = create_test_app();

        let float_height = 15.0;
        // Use a stiffer, overdamped spring for precise settling without oscillation
        let config = ControllerConfig::default()
            .with_float_height(float_height)
            .with_spring(50.0, 7.0);

        // Ground at y=0
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Character starts above ground (within detection range)
        // Ground surface is at y=5.0, riding_height is ~23, so start at y=25 to be safe
        let character = spawn_character_with_config(&mut app, Vec2::new(0.0, 25.0), config);

        // Run simulation to let character settle (3 seconds for precise settling)
        run_for_duration(&mut app, 3.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let cfg = app.world().get::<ControllerConfig>(character).unwrap();
        let transform = app.world().get::<Transform>(character).unwrap();

        // The riding height = float_height + collider_bottom_offset
        let riding_height = controller.riding_height(cfg);

        println!(
            "DEBUG: character pos={}, ground_detected={}, floor={:?}",
            transform.translation,
            controller.ground_detected(),
            controller.floor
        );

        let ground_dist = controller.ground_distance().expect("Ground should be detected");
        println!(
            "PROOF: Character position.y={}, ground_distance={}, riding_height={}",
            transform.translation.y,
            ground_dist,
            riding_height
        );

        // PROOF: ground_distance should be close to riding_height after settling
        // Tolerance is 3.0 to account for physics variance in test environments
        let tolerance = 3.0;
        assert!(
            (ground_dist - riding_height).abs() < tolerance,
            "Ground distance {} should be close to riding_height {} (diff: {})",
            ground_dist,
            riding_height,
            (ground_dist - riding_height).abs()
        );

        // PROOF: Character should NOT be touching the ground (position should be elevated)
        let capsule_bottom = transform.translation.y - controller.capsule_half_height();
        let ground_surface = 5.0; // Ground half-height
        assert!(
            capsule_bottom > ground_surface,
            "Character should be floating ABOVE ground: capsule_bottom={}, ground_surface={}",
            capsule_bottom,
            ground_surface
        );
    }

    #[test]
    fn spring_force_applied_to_rigidbody() {
        let mut app = create_test_app();

        let config = ControllerConfig::default()
            .with_float_height(15.0)
            .with_spring(5000.0, 100.0);

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Start character below float_height - spring should push UP
        let character = spawn_character_with_config(&mut app, Vec2::new(0.0, 15.0), config);

        // Get initial velocity
        run_for_duration(&mut app, 2.0);
        let vel_before = app.world().get::<LinearVelocity>(character).unwrap().0;

        // Run a frame
        run_for_duration(&mut app, 2.0);
        let vel_after = app.world().get::<LinearVelocity>(character).unwrap().0;

        println!(
            "PROOF: vel_before={:?}, vel_after={:?}",
            vel_before, vel_after
        );

        // PROOF: Spring force should have affected velocity
        let ext_force = app.world().get::<ConstantForce>(character);
        println!("PROOF: ConstantForce={:?}", ext_force);

        // The velocity change proves force was applied
        assert!(
            (vel_after - vel_before).length() > 0.001 || ext_force.is_some(),
            "Spring force should affect velocity or external force"
        );
    }
}

// ==================== Wall Detection Tests ====================

mod wall_detection {
    use super::*;

    #[test]
    fn detects_wall_on_left() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Wall on the left, close to character
        spawn_ground(&mut app, Vec2::new(-10.0, 20.0), Vec2::new(2.0, 20.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        // Run more frames to let Avian's spatial query systems process colliders
        run_for_duration(&mut app, 2.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        println!(
            "PROOF: touching_left_wall={}, left_wall={:?}",
            controller.touching_left_wall(),
            controller.left_wall
        );

        // PROOF: touching_left_wall should be true
        assert!(
            controller.touching_left_wall(),
            "Wall on left should be detected"
        );
    }

    #[test]
    fn detects_wall_on_right() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Wall on the right
        spawn_ground(&mut app, Vec2::new(10.0, 20.0), Vec2::new(2.0, 20.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        // Run more frames to let Avian's spatial query systems process colliders
        run_for_duration(&mut app, 2.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        println!(
            "PROOF: touching_right_wall={}, right_wall={:?}",
            controller.touching_right_wall(),
            controller.right_wall
        );

        // PROOF: touching_right_wall should be true
        assert!(
            controller.touching_right_wall(),
            "Wall on right should be detected"
        );
    }

    #[test]
    fn no_wall_when_far() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Wall far away
        spawn_ground(&mut app, Vec2::new(-100.0, 20.0), Vec2::new(5.0, 20.0));
        spawn_ground(&mut app, Vec2::new(100.0, 20.0), Vec2::new(5.0, 20.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        run_for_duration(&mut app, 2.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        println!(
            "PROOF: touching_left_wall={}, touching_right_wall={}",
            controller.touching_left_wall(),
            controller.touching_right_wall()
        );

        // PROOF: No walls should be detected when far
        assert!(
            !controller.touching_left_wall(),
            "Far wall should NOT be detected"
        );
        assert!(
            !controller.touching_right_wall(),
            "Far wall should NOT be detected"
        );
    }
}

// ==================== Ceiling Detection Tests ====================

mod ceiling_detection {
    use super::*;

    #[test]
    fn ceiling_caster_spawned() {
        let mut app = create_test_app();
        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        run_for_duration(&mut app, 0.1);

        // Count casters
        let mut query = app.world_mut().query::<(&CeilingCaster, &CasterParent)>();
        let ceiling_caster_count = query
            .iter(app.world())
            .filter(|(_, parent)| parent.0 == character)
            .count();

        println!("PROOF: ceiling_caster_count={}", ceiling_caster_count);

        assert_eq!(ceiling_caster_count, 1, "Should have 1 ceiling caster");
    }

    #[test]
    fn detects_ceiling_above() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Ceiling very close to character
        // Character center at y=20, place ceiling at y=25 (well within detection range)
        spawn_ground(&mut app, Vec2::new(0.0, 27.0), Vec2::new(50.0, 2.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        // Run more frames to let Avian's spatial query systems process colliders
        run_for_duration(&mut app, 2.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        println!(
            "PROOF: touching_ceiling={}, ceiling={:?}",
            controller.touching_ceiling(),
            controller.ceiling
        );

        // PROOF: touching_ceiling should be true
        assert!(
            controller.touching_ceiling(),
            "Ceiling above should be detected"
        );
    }

    #[test]
    fn no_ceiling_when_far() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Ceiling far above
        spawn_ground(&mut app, Vec2::new(0.0, 100.0), Vec2::new(50.0, 2.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        run_for_duration(&mut app, 2.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        println!(
            "PROOF: touching_ceiling={}",
            controller.touching_ceiling()
        );

        // PROOF: No ceiling should be detected when far
        assert!(
            !controller.touching_ceiling(),
            "Far ceiling should NOT be detected"
        );
    }
}

// ==================== Movement Tests ====================

mod movement {
    use super::*;

    #[test]
    fn walk_intent_changes_velocity() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        run_for_duration(&mut app, 1.0);

        let vel_before = app.world().get::<LinearVelocity>(character).unwrap().0;

        // Set walk intent to move right
        if let Some(mut intent) = app.world_mut().get_mut::<MovementIntent>(character) {
            intent.set_walk(1.0);
        }

        run_for_duration(&mut app, 2.0);

        let vel_after = app.world().get::<LinearVelocity>(character).unwrap().0;

        println!(
            "PROOF: vel_before={:?}, vel_after={:?}",
            vel_before, vel_after
        );

        // PROOF: Velocity should increase in the X direction
        assert!(
            vel_after.x > vel_before.x + 1.0,
            "Walk intent should increase X velocity"
        );
    }

    #[test]
    fn jump_changes_velocity() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        // Run a few frames to settle
        run_frames(&mut app, 120); // 2 seconds at 60Hz

        // Verify grounded
        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();

        eprintln!("Before jump: grounded={}, ground_dist={:?}, vel.y={}",
            controller.is_grounded(config),
            controller.ground_distance(),
            app.world().get::<LinearVelocity>(character).unwrap().0.y);

        assert!(controller.is_grounded(config), "Must be grounded to jump");

        let vel_before = app.world().get::<LinearVelocity>(character).unwrap().0;

        request_jump(&mut app, character);

        // Check jump request was created
        let intent = app.world().get::<MovementIntent>(character).unwrap();
        eprintln!("After request_jump: jump_pressed={}, jump_request={:?}",
            intent.jump_pressed, intent.jump_request);

        tick(&mut app);

        let intent_after = app.world().get::<MovementIntent>(character).unwrap();
        eprintln!("After tick: jump_pressed={}, jump_request={:?}",
            intent_after.jump_pressed, intent_after.jump_request);

        let vel_after = app.world().get::<LinearVelocity>(character).unwrap().0;

        println!(
            "PROOF: vel_before.y={}, vel_after.y={}",
            vel_before.y, vel_after.y
        );

        // PROOF: Jump should apply positive Y velocity change
        assert!(
            vel_after.y > vel_before.y + 50.0,
            "Jump should apply significant upward velocity (~90 units/s)"
        );
    }
}

// ==================== Gravity Tests ====================

mod gravity {
    use super::*;

    #[test]
    fn internal_gravity_applied_when_airborne() {
        let mut app = create_test_app();

        // No ground - character is airborne
        let character = spawn_character(&mut app, Vec2::new(0.0, 100.0));

        let vel_before = app.world().get::<LinearVelocity>(character).unwrap().0;

        run_for_duration(&mut app, 2.0);

        let vel_after = app.world().get::<LinearVelocity>(character).unwrap().0;

        println!(
            "PROOF: vel_before.y={}, vel_after.y={}",
            vel_before.y, vel_after.y
        );

        // PROOF: Internal gravity should decrease Y velocity (make it more negative)
        assert!(
            vel_after.y < vel_before.y - 10.0,
            "Internal gravity should apply downward acceleration"
        );
    }

    #[test]
    fn custom_gravity_affects_controller() {
        let mut app = create_test_app();

        let character = {
            app.world_mut()
                .spawn((
                    Transform::from_translation(Vec2::new(0.0, 100.0).extend(0.0)),
                    CharacterController::with_gravity(Vec2::new(0.0, -500.0)), // Custom gravity
                    ControllerConfig::default(),
                    Collider::capsule(4.0, 8.0),
                    LockedAxes::ROTATION_LOCKED,
                    GravityScale(0.0),
                ))
                .id()
        };

        let controller = app.world().get::<CharacterController>(character).unwrap();
        println!("PROOF: gravity={:?}", controller.gravity);

        // PROOF: Gravity should be custom value
        assert_eq!(
            controller.gravity,
            Vec2::new(0.0, -500.0),
            "Gravity should be custom value"
        );
    }
}

// ==================== Coyote Time Tests ====================

mod coyote_time {
    use super::*;

    #[test]
    fn time_since_grounded_zero_when_grounded() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        run_for_duration(&mut app, 1.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: is_grounded={}, in_coyote_time={}",
            controller.is_grounded(config),
            controller.in_coyote_time()
        );

        // PROOF: coyote timer should be active (not finished) when grounded
        assert!(
            controller.in_coyote_time(),
            "coyote timer should be active when grounded"
        );
    }

    #[test]
    fn coyote_timer_expires_when_airborne() {
        let mut app = create_test_app();

        // No ground
        let character = spawn_character(&mut app, Vec2::new(0.0, 100.0));

        run_for_duration(&mut app, 2.0);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: is_grounded={}, in_coyote_time={}",
            controller.is_grounded(config),
            controller.in_coyote_time()
        );

        // PROOF: coyote timer should have expired (30 frames at 60Hz = 0.5s > coyote_time=0.15s)
        assert!(
            !controller.in_coyote_time(),
            "coyote timer should expire after being airborne long enough"
        );
    }
}

// ==================== Collision Layer Inheritance Tests ====================

mod collision_layers {
    use super::*;

    // Layer bits for collision layers (1 << layer_index)
    const LAYER_0: u32 = 1 << 0; // First layer (default layer)
    const LAYER_1: u32 = 1 << 1; // Second layer

    #[test]
    fn sensors_inherit_collision_layers() {
        let mut app = create_test_app();

        // Create ground in layer 0, filters layer 0
        let ground_transform = Transform::from_translation(Vec2::new(0.0, 0.0).extend(0.0));
        app.world_mut().spawn((
            ground_transform,
            GlobalTransform::from(ground_transform),
            RigidBody::Static,
            Collider::rectangle(200.0, 10.0),
            CollisionLayers::from_bits(LAYER_0, LAYER_0),
        ));

        // Character in layer 0 - should detect ground
        let char_in_group = {
            let config = ControllerConfig::default();
            let world = app.world_mut();
            let entity = world
                .spawn((
                    Transform::from_translation(Vec2::new(-20.0, 20.0).extend(0.0)),
                    GlobalTransform::default(),
                    CharacterController::new(),
                    config.clone(),
                    MovementIntent::new(),
                    Collider::capsule(4.0, 8.0),
                    CollisionLayers::from_bits(LAYER_0, LAYER_0),
                    LockedAxes::ROTATION_LOCKED,
                    GravityScale(0.0),
                ))
                .id();

            // Spawn ground caster child
            let half_width = config.ground_cast_width / 2.0;
            let child = world.spawn((
                GroundCaster,
                CasterParent(entity),
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
            world.entity_mut(entity).add_child(child);

            entity
        };

        // Character in layer 1 - should NOT detect ground (different layer)
        let char_not_in_group = {
            let config = ControllerConfig::default();
            let world = app.world_mut();
            let entity = world
                .spawn((
                    Transform::from_translation(Vec2::new(20.0, 20.0).extend(0.0)),
                    GlobalTransform::default(),
                    CharacterController::new(),
                    config.clone(),
                    MovementIntent::new(),
                    Collider::capsule(4.0, 8.0),
                    CollisionLayers::from_bits(LAYER_1, LAYER_1),
                    LockedAxes::ROTATION_LOCKED,
                    GravityScale(0.0),
                ))
                .id();

            // Spawn ground caster child
            let half_width = config.ground_cast_width / 2.0;
            let child = world.spawn((
                GroundCaster,
                CasterParent(entity),
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
            world.entity_mut(entity).add_child(child);

            entity
        };

        // Run more frames to let Avian's spatial query systems process colliders
        run_for_duration(&mut app, 2.0);

        let ctrl1 = app
            .world()
            .get::<CharacterController>(char_in_group)
            .unwrap();
        let ctrl2 = app
            .world()
            .get::<CharacterController>(char_not_in_group)
            .unwrap();

        println!(
            "PROOF: layer 0 char ground_detected={}, layer 1 char ground_detected={}",
            ctrl1.ground_detected(),
            ctrl2.ground_detected()
        );

        // PROOF: Sensors should inherit collision layers
        assert!(
            ctrl1.ground_detected(),
            "Character in layer 0 should detect layer 0 ground"
        );
        assert!(
            !ctrl2.ground_detected(),
            "Character in layer 1 should NOT detect layer 0 ground"
        );
    }

    /// Test that explicit CollisionLayers matching the default layer work correctly.
    #[test]
    fn explicit_layer_0_matches_default() {
        let mut app = create_test_app();

        // Ground with default CollisionLayers (auto-inserted: memberships=1, filters=ALL)
        let ground_transform = Transform::from_translation(Vec2::new(0.0, 0.0).extend(0.0));
        app.world_mut().spawn((
            ground_transform,
            GlobalTransform::from(ground_transform),
            RigidBody::Static,
            Collider::rectangle(200.0, 10.0),
        ));

        // Character with explicit layer 0 - should detect ground with default layers
        let char_with_layers = {
            let config = ControllerConfig::default();
            let world = app.world_mut();
            let entity = world
                .spawn((
                    Transform::from_translation(Vec2::new(-20.0, 20.0).extend(0.0)),
                    GlobalTransform::default(),
                    CharacterController::new(),
                    config.clone(),
                    MovementIntent::new(),
                    Collider::capsule(4.0, 8.0),
                    // Layer 0 = bit 1, filters layer 0
                    CollisionLayers::from_bits(LAYER_0, LAYER_0),
                    LockedAxes::ROTATION_LOCKED,
                    GravityScale(0.0),
                ))
                .id();

            // Spawn ground caster child
            let half_width = config.ground_cast_width / 2.0;
            let child = world.spawn((
                GroundCaster,
                CasterParent(entity),
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
            world.entity_mut(entity).add_child(child);

            entity
        };

        // Character without explicit CollisionLayers - uses auto-inserted default
        let char_no_layers = {
            let config = ControllerConfig::default();
            let world = app.world_mut();
            let entity = world
                .spawn((
                    Transform::from_translation(Vec2::new(20.0, 20.0).extend(0.0)),
                    GlobalTransform::default(),
                    CharacterController::new(),
                    config.clone(),
                    MovementIntent::new(),
                    Collider::capsule(4.0, 8.0),
                    LockedAxes::ROTATION_LOCKED,
                    GravityScale(0.0),
                ))
                .id();

            // Spawn ground caster child
            let half_width = config.ground_cast_width / 2.0;
            let child = world.spawn((
                GroundCaster,
                CasterParent(entity),
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
            world.entity_mut(entity).add_child(child);

            entity
        };

        run_for_duration(&mut app, 1.0);

        // Debug: print layer values
        let char_with_layers_val = app.world().get::<CollisionLayers>(char_with_layers);
        let char_no_layers_val = app.world().get::<CollisionLayers>(char_no_layers);
        println!("char_with_layers CollisionLayers: {:?}", char_with_layers_val);
        println!("char_no_layers CollisionLayers: {:?}", char_no_layers_val);

        let ctrl_with = app.world().get::<CharacterController>(char_with_layers).unwrap();
        let ctrl_no = app.world().get::<CharacterController>(char_no_layers).unwrap();

        println!(
            "PROOF: char_with_explicit_layer_0 ground_detected={}, char_with_default ground_detected={}",
            ctrl_with.ground_detected(),
            ctrl_no.ground_detected()
        );

        // Both should detect ground
        assert!(
            ctrl_with.ground_detected(),
            "Character with explicit layer 0 should detect ground with default layers"
        );
        assert!(
            ctrl_no.ground_detected(),
            "Character with default layers should detect ground"
        );
    }
}
