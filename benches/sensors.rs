//! Whole-step benchmark for the sensor pipeline on the Avian backend.
//!
//! Spawns a row of characters hovering over one static floor and times a full
//! `app.update()` (one `FixedMain` step: controller systems plus Avian's
//! physics step, which runs the detection casts). Half the characters walk into a
//! wall, so their stair and wall casts are live and hitting. The sensor work is a slice of the measured
//! step, not all of it; compare runs of this bench against each other only.

use avian2d::prelude::*;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use msg_character_controller::prelude::*;

const SPACING: f32 = 30.0;

fn build_app(characters: usize) -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(TransformPlugin);
    app.insert_resource(bevy::world_serialization::WorldInstanceSpawner::default());
    app.add_plugins(PhysicsPlugins::default().with_length_unit(10.0));
    app.add_plugins(CharacterControllerPlugin::<Avian2dBackend>::default());
    app.insert_resource(Time::<Fixed>::from_hz(60.0));
    app.insert_resource(TimeUpdateStrategy::FixedTimesteps(1));
    app.finish();
    app.cleanup();

    #[allow(clippy::cast_precision_loss)]
    let width = characters as f32 * SPACING + 100.0;
    app.world_mut().spawn((
        Transform::from_xyz(width / 2.0 - 50.0, 0.0, 0.0),
        RigidBody::Static,
        Collider::rectangle(width, 10.0),
    ));

    for i in 0..characters {
        #[allow(clippy::cast_precision_loss)]
        let x = i as f32 * SPACING;
        let mut intent = MovementIntent::new();
        if i % 2 == 0 {
            // Walk into a tall wall so the character stays put while its
            // stair and wall casts keep hitting something.
            let dir = if i % 4 == 0 { 1.0 } else { -1.0 };
            intent.set_walk(dir);
            app.world_mut().spawn((
                Transform::from_xyz(x + dir * 12.0, 55.0, 0.0),
                RigidBody::Static,
                Collider::rectangle(4.0, 100.0),
            ));
        }
        app.world_mut().spawn((
            Transform::from_xyz(x, 20.0, 0.0),
            CharacterController::new().with_stair_config(StairConfig::default()),
            ControllerConfig::default(),
            intent,
            Collider::capsule(4.0, 8.0),
            LockedAxes::ROTATION_LOCKED,
            GravityScale(0.0),
        ));
    }

    // Settle: let casters spawn and characters reach their float height.
    for _ in 0..120 {
        app.update();
    }
    app
}

fn full_step(c: &mut Criterion) {
    let mut group = c.benchmark_group("sensors_full_step");
    for characters in [64, 512] {
        let mut app = build_app(characters);
        group.bench_with_input(
            BenchmarkId::from_parameter(characters),
            &characters,
            |b, _| b.iter(|| app.update()),
        );
    }
    group.finish();
}

criterion_group!(benches, full_step);
criterion_main!(benches);
