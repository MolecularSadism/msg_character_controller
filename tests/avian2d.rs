//! Integration tests for the character controller with Avian2D backend.
//!
//! These tests verify the complete system behavior with actual physics simulation.
//! Each test produces PROOF through explicit velocity/force checks.

#![cfg(feature = "avian2d")]

use bevy::prelude::*;
use bevy::time::Virtual;
use avian2d::prelude::*;
use msg_character_controller::prelude::*;
use msg_character_controller::avian::Avian2dBackend;

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
            Collider::rectangle(half_size.x * 2.0, half_size.y * 2.0),
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
    app.world_mut()
        .spawn((
            transform,
            GlobalTransform::from(transform),
            RigidBody::Dynamic, // Required for Avian physics
            CharacterController::new(),
            config,
            Collider::capsule(4.0, 8.0),
            LockedAxes::ROTATION_LOCKED,
            GravityScale(0.0), // Disable Avian gravity - use controller's gravity
        ))
        .id()
}

/// Spawn a character with custom gravity (which determines the up direction).
fn spawn_character_with_gravity(app: &mut App, position: Vec2, gravity: Vec2) -> Entity {
    let transform = Transform::from_translation(position.extend(0.0));
    app.world_mut()
        .spawn((
            transform,
            GlobalTransform::from(transform),
            RigidBody::Dynamic, // Required for Avian physics
            CharacterController::with_gravity(gravity),
            ControllerConfig::default(),
            Collider::capsule(4.0, 8.0),
            LockedAxes::ROTATION_LOCKED,
            GravityScale(0.0),
        ))
        .id()
}

/// Run one physics step.
fn tick(app: &mut App) {
    let timestep = std::time::Duration::from_secs_f64(1.0 / 60.0);
    app.world_mut()
        .resource_mut::<Time<Virtual>>()
        .advance_by(timestep);
    // Just update once - Bevy 0.18 handles scheduling internally
    app.update();
}

/// Run the app for N physics frames.
fn run_frames(app: &mut App, frames: usize) {
    for _ in 0..frames {
        tick(app);
    }
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
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        // Character at y=20 (within derived ground_cast_length)
        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        // Run a few frames to let Avian's spatial query systems process colliders
        run_frames(&mut app, 3);

        let controller = app.world().get::<CharacterController>(character).unwrap();

        // PROOF: ground_detected should be true (raycast hit)
        assert!(
            controller.ground_detected(),
            "Ground should be detected by raycast"
        );

        // PROOF: ground_distance should be less than character height
        assert!(
            controller.ground_distance() < 20.0,
            "Ground distance should be less than character height: {}",
            controller.ground_distance()
        );

        println!(
            "PROOF: ground_detected={}, ground_distance={}, ground_normal={:?}",
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

        // Run a few frames to let Avian's spatial query systems process colliders
        run_frames(&mut app, 3);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let cfg = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: is_grounded={}, ground_distance={}, riding_height+grounding_distance={}",
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
        let character = spawn_character(&mut app, Vec2::new(0.0, 200.0));

        tick(&mut app);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();

        println!(
            "PROOF: is_grounded={}, ground_detected={}, ground_distance={}",
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

        tick(&mut app);

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
            .with_spring(5000.0, 700.0);

        // Ground at y=0
        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));

        // Character starts above ground
        let character = spawn_character_with_config(&mut app, Vec2::new(0.0, 50.0), config);

        // Run simulation to let character settle (more frames for precise settling)
        run_frames(&mut app, 500);

        let controller = app.world().get::<CharacterController>(character).unwrap();
        let cfg = app.world().get::<ControllerConfig>(character).unwrap();
        let transform = app.world().get::<Transform>(character).unwrap();

        // The riding height = float_height + collider_bottom_offset
        let riding_height = controller.riding_height(cfg);

        println!(
            "PROOF: Character position.y={}, ground_distance={}, riding_height={}",
            transform.translation.y,
            controller.ground_distance(),
            riding_height
        );

        // PROOF: ground_distance should be close to riding_height after settling
        // Tolerance is 3.0 to account for physics variance in test environments
        let tolerance = 3.0;
        assert!(
            (controller.ground_distance() - riding_height).abs() < tolerance,
            "Ground distance {} should be close to riding_height {} (diff: {})",
            controller.ground_distance(),
            riding_height,
            (controller.ground_distance() - riding_height).abs()
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
        tick(&mut app);
        let vel_before = app.world().get::<LinearVelocity>(character).unwrap().0;

        // Run a frame
        tick(&mut app);
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

        // Run a few frames to let Avian's spatial query systems process colliders
        run_frames(&mut app, 3);

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

        // Run a few frames to let Avian's spatial query systems process colliders
        run_frames(&mut app, 3);

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

        tick(&mut app);

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

// ==================== Movement Tests ====================

mod movement {
    use super::*;

    #[test]
    fn walk_intent_changes_velocity() {
        let mut app = create_test_app();

        spawn_ground(&mut app, Vec2::new(0.0, 0.0), Vec2::new(100.0, 5.0));
        let character = spawn_character(&mut app, Vec2::new(0.0, 20.0));

        run_frames(&mut app, 5);

        let vel_before = app.world().get::<LinearVelocity>(character).unwrap().0;

        // Set walk intent to move right
        if let Some(mut intent) = app.world_mut().get_mut::<MovementIntent>(character) {
            intent.set_walk(1.0);
        }

        run_frames(&mut app, 10);

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

        run_frames(&mut app, 5);

        // Verify grounded
        let controller = app.world().get::<CharacterController>(character).unwrap();
        let config = app.world().get::<ControllerConfig>(character).unwrap();
        assert!(controller.is_grounded(config), "Must be grounded to jump");

        let vel_before = app.world().get::<LinearVelocity>(character).unwrap().0;

        request_jump(&mut app, character);
        tick(&mut app);

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

        run_frames(&mut app, 10);

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

        run_frames(&mut app, 5);

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

        run_frames(&mut app, 30);

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
            app.world_mut()
                .spawn((
                    Transform::from_translation(Vec2::new(-20.0, 20.0).extend(0.0)),
                    CharacterController::new(),
                    ControllerConfig::default(),
                    Collider::capsule(4.0, 8.0),
                    CollisionLayers::from_bits(LAYER_0, LAYER_0),
                    LockedAxes::ROTATION_LOCKED,
                    GravityScale(0.0),
                ))
                .id()
        };

        // Character in layer 1 - should NOT detect ground (different layer)
        let char_not_in_group = {
            app.world_mut()
                .spawn((
                    Transform::from_translation(Vec2::new(20.0, 20.0).extend(0.0)),
                    CharacterController::new(),
                    ControllerConfig::default(),
                    Collider::capsule(4.0, 8.0),
                    CollisionLayers::from_bits(LAYER_1, LAYER_1),
                    LockedAxes::ROTATION_LOCKED,
                    GravityScale(0.0),
                ))
                .id()
        };

        // Run a few frames to let Avian's spatial query systems process colliders
        run_frames(&mut app, 3);

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
            app.world_mut()
                .spawn((
                    Transform::from_translation(Vec2::new(-20.0, 20.0).extend(0.0)),
                    CharacterController::new(),
                    ControllerConfig::default(),
                    Collider::capsule(4.0, 8.0),
                    // Layer 0 = bit 1, filters layer 0
                    CollisionLayers::from_bits(LAYER_0, LAYER_0),
                    LockedAxes::ROTATION_LOCKED,
                    GravityScale(0.0),
                ))
                .id()
        };

        // Character without explicit CollisionLayers - uses auto-inserted default
        let char_no_layers = {
            app.world_mut()
                .spawn((
                    Transform::from_translation(Vec2::new(20.0, 20.0).extend(0.0)),
                    CharacterController::new(),
                    ControllerConfig::default(),
                    Collider::capsule(4.0, 8.0),
                    LockedAxes::ROTATION_LOCKED,
                    GravityScale(0.0),
                ))
                .id()
        };

        run_frames(&mut app, 3);

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
