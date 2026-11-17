//! The detection pipeline driven by a backend with no physics engine.
//!
//! `FakeBackend` answers `cast_shape` from a scripted scene (a stepped floor
//! and two walls) and logs every query, so these tests prove the ground, wall,
//! ceiling and stair logic runs on nothing but the `CharacterPhysicsBackend`
//! trait. The file uses no Avian API and also runs with
//! `--no-default-features`.

use std::sync::Mutex;

use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use msg_character_controller::backend::NoOpBackendPlugin;
use msg_character_controller::prelude::*;
use msg_character_controller::sensors::{
    CastShape, SensorCastMode, SensorHit, SensorKind, ShapeCastQuery,
};

/// Position of a fake body. Nothing moves it: there is no physics.
#[derive(Component, Default)]
struct FakeBody {
    position: Vec2,
    velocity: Vec2,
}

/// The scripted scene the fake casts run against.
#[derive(Resource)]
struct Scene {
    floor: Entity,
    step: Entity,
    wall: Entity,
    /// Floor top is `y = 0` left of `step_x` and `y = step_height` from it on.
    step_x: f32,
    step_height: f32,
    /// A wall face at this x, facing +X (hit by left casts).
    left_wall_x: f32,
}

/// Every query `cast_shape` received.
#[derive(Resource, Default)]
struct CastLog(Mutex<Vec<ShapeCastQuery>>);

struct FakeBackend;

impl FakeBackend {
    fn floor_hit(scene: &Scene, query: &ShapeCastQuery) -> Option<SensorHit> {
        let CastShape::Segment { a, b } = query.shape else {
            return None;
        };
        // The highest floor under the swept segment is hit first.
        let right = query.origin.x + a.x.max(b.x);
        let (top, entity) = if right >= scene.step_x {
            (scene.step_height, scene.step)
        } else {
            (0.0, scene.floor)
        };
        let distance = query.origin.y - top;
        (0.0..=query.max_distance)
            .contains(&distance)
            .then_some(SensorHit {
                entity,
                distance,
                normal: Vec2::Y,
            })
    }

    fn left_wall_hit(scene: &Scene, query: &ShapeCastQuery) -> Option<SensorHit> {
        let distance = query.origin.x - scene.left_wall_x;
        (0.0..=query.max_distance)
            .contains(&distance)
            .then_some(SensorHit {
                entity: scene.wall,
                distance,
                normal: Vec2::X,
            })
    }
}

impl CharacterPhysicsBackend for FakeBackend {
    type VelocityComponent = FakeBody;

    fn plugin() -> impl Plugin {
        NoOpBackendPlugin
    }

    fn get_velocity(world: &World, entity: Entity) -> Vec2 {
        world
            .get::<FakeBody>(entity)
            .map_or(Vec2::ZERO, |b| b.velocity)
    }

    fn set_velocity(world: &mut World, entity: Entity, velocity: Vec2) {
        if let Some(mut body) = world.get_mut::<FakeBody>(entity) {
            body.velocity = velocity;
        }
    }

    fn apply_impulse(_world: &mut World, _entity: Entity, _impulse: Vec2) {}

    fn apply_force(_world: &mut World, _entity: Entity, _force: Vec2) {}

    fn apply_torque(_world: &mut World, _entity: Entity, _torque: f32) {}

    fn get_angular_velocity(_world: &World, _entity: Entity) -> f32 {
        0.0
    }

    fn get_rotation(_world: &World, _entity: Entity) -> f32 {
        0.0
    }

    fn get_gravity(_world: &World, _entity: Entity) -> Vec2 {
        Vec2::ZERO
    }

    fn get_position(world: &World, entity: Entity) -> Vec2 {
        world
            .get::<FakeBody>(entity)
            .map_or(Vec2::ZERO, |b| b.position)
    }

    fn get_fixed_timestep(_world: &World) -> f32 {
        1.0 / 60.0
    }

    fn get_collider_bottom_offset(_world: &World, _entity: Entity) -> f32 {
        8.0
    }

    fn get_collider_radius(_world: &World, _entity: Entity) -> f32 {
        4.0
    }

    fn get_mass(_world: &World, _entity: Entity) -> f32 {
        1.0
    }

    fn get_principal_inertia(_world: &World, _entity: Entity) -> f32 {
        1.0
    }

    fn is_rotation_locked(_world: &World, _entity: Entity) -> bool {
        true
    }

    fn sensor_cast_mode() -> SensorCastMode {
        SensorCastMode::Immediate
    }

    fn cast_shape(world: &World, query: &ShapeCastQuery) -> Option<SensorHit> {
        world.resource::<CastLog>().0.lock().unwrap().push(*query);
        let scene = world.resource::<Scene>();
        if query.direction == Dir2::NEG_Y {
            Self::floor_hit(scene, query)
        } else if query.direction == Dir2::NEG_X {
            Self::left_wall_hit(scene, query)
        } else {
            None
        }
    }
}

fn app_with_scene() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(CharacterControllerPlugin::<FakeBackend>::default());
    app.insert_resource(Time::<Fixed>::from_hz(60.0));
    app.insert_resource(TimeUpdateStrategy::FixedTimesteps(1));
    app.init_resource::<CastLog>();
    let floor = app.world_mut().spawn_empty().id();
    let step = app.world_mut().spawn_empty().id();
    let wall = app.world_mut().spawn_empty().id();
    app.insert_resource(Scene {
        floor,
        step,
        wall,
        step_x: 7.0,
        step_height: 10.0,
        left_wall_x: -5.0,
    });
    app.finish();
    app.cleanup();
    // The first update only starts the clock; every later one runs exactly one
    // fixed step.
    app.update();
    app
}

fn spawn_character(app: &mut App, config: ControllerConfig, walk: f32) -> Entity {
    let mut intent = MovementIntent::new();
    intent.set_walk(walk);
    app.world_mut()
        .spawn((
            FakeBody {
                position: Vec2::new(0.0, 15.0),
                ..default()
            },
            CharacterController::new(),
            config,
            intent,
        ))
        .id()
}

fn queries_of(app: &App, exclude: Entity) -> Vec<ShapeCastQuery> {
    app.world()
        .resource::<CastLog>()
        .0
        .lock()
        .unwrap()
        .iter()
        .filter(|q| q.filter.exclude == exclude)
        .copied()
        .collect()
}

#[test]
fn detection_runs_on_scripted_casts() {
    let mut app = app_with_scene();
    let character = spawn_character(&mut app, ControllerConfig::default(), 1.0);

    // Step 1 learns the collider bottom offset; step 2 casts far enough to
    // reach the floor 15 units below.
    app.update();
    app.update();

    let scene = app.world().resource::<Scene>();
    let (floor_entity, step_entity, wall_entity) = (scene.floor, scene.step, scene.wall);
    let controller = app.world().get::<CharacterController>(character).unwrap();

    let floor = controller.floor.expect("floor detected");
    assert_eq!(floor.entity, Some(floor_entity));
    assert!((floor.distance - 15.0).abs() < 1e-4);
    assert_eq!(floor.normal, Vec2::Y);
    assert!((floor.point - Vec2::new(0.0, 0.0)).length() < 1e-4);
    assert!(controller.slope_angle.abs() < 1e-4);

    let left = controller.left_wall.expect("left wall detected");
    assert_eq!(left.entity, Some(wall_entity));
    assert!((left.distance - 5.0).abs() < 1e-4);
    assert!((left.point - Vec2::new(-5.0, 15.0)).length() < 1e-4);
    assert!(controller.right_wall.is_none());
    assert!(controller.ceiling.is_none());

    // Walking right: the stair cast starts at radius + offset ahead and
    // (max_climb_height - bottom_offset) up, and finds the 10-unit step.
    let sensors = app
        .world()
        .get::<msg_character_controller::sensors::CharacterSensors>(character)
        .unwrap();
    let stair_hit = sensors.reading(SensorKind::Stair).and_then(|r| r.hit);
    assert_eq!(stair_hit.map(|h| h.entity), Some(step_entity));
    assert!(controller.step_detected, "step detected");
    assert!((controller.step_height - 10.0).abs() < 1e-4);

    let stair = queries_of(&app, character)
        .into_iter()
        .rev()
        .find(|q| q.origin == Vec2::new(9.0, 18.0))
        .expect("stair cast issued");
    assert!((stair.max_distance - 9.0).abs() < 1e-4);
    assert_eq!(stair.filter.mask, u32::MAX);
}

#[test]
fn idle_character_skips_stair_casts() {
    let mut app = app_with_scene();
    let character = spawn_character(&mut app, ControllerConfig::default(), 0.0);
    app.update();

    // Ground, two walls and ceiling: four casts per step, no stair casts.
    let queries = queries_of(&app, character);
    assert_eq!(queries.len(), 4);
    let controller = app.world().get::<CharacterController>(character).unwrap();
    assert!(!controller.step_detected);
}

#[test]
fn caster_disabled_holds_every_cast() {
    let mut app = app_with_scene();
    let character = spawn_character(&mut app, ControllerConfig::default(), 1.0);
    app.world_mut().entity_mut(character).insert(CasterDisabled);
    app.update();
    app.update();

    assert!(queries_of(&app, character).is_empty());
    let controller = app.world().get::<CharacterController>(character).unwrap();
    assert!(controller.floor.is_none());
    assert!(controller.left_wall.is_none());
    assert!(!controller.step_detected);

    // Lifting the hold resumes casting.
    app.world_mut()
        .entity_mut(character)
        .remove::<CasterDisabled>();
    app.update();
    assert!(!queries_of(&app, character).is_empty());
}

#[test]
fn collision_mask_override_reaches_the_backend() {
    let mut app = app_with_scene();
    let mut config = ControllerConfig::default();
    config.sensors.collision_mask = Some(0b100);
    let character = spawn_character(&mut app, config, 0.0);
    app.update();

    let queries = queries_of(&app, character);
    assert!(!queries.is_empty());
    assert!(queries.iter().all(|q| q.filter.mask == 0b100));

    let sensors = app
        .world()
        .get::<msg_character_controller::sensors::CharacterSensors>(character)
        .unwrap();
    assert!(sensors.reading(SensorKind::Ground).is_some());
}
