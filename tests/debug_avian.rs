#![cfg(feature = "avian2d")]

use bevy::prelude::*;
use bevy::ecs::system::RunSystemOnce;
use bevy::scene::ScenePlugin;
use avian2d::prelude::*;

#[derive(Resource, Default)]
struct RaycastResult {
    hit: Option<RayHitData>,
}

fn perform_raycast(spatial_query: SpatialQuery, mut result: ResMut<RaycastResult>) {
    let ray_origin = Vec2::new(0.0, 10.0);
    let ray_direction = Dir2::NEG_Y;
    let max_distance = 100.0;

    result.hit = spatial_query.cast_ray(
        ray_origin,
        ray_direction,
        max_distance,
        true,
        &SpatialQueryFilter::default(),
    );

    println!("Ray from {:?} downward:", ray_origin);
    println!("  Hit: {:?}", result.hit);
}

#[test]
fn test_simple_spatial_query() {
    let mut app = App::new();

    app.add_plugins(MinimalPlugins);
    app.add_plugins(TransformPlugin);
    // Insert SceneSpawner resource to satisfy Avian's ColliderHierarchyPlugin
    app.insert_resource(bevy::scene::SceneSpawner::default());
    app.add_plugins(PhysicsPlugins::default().with_length_unit(10.0));
    app.insert_resource(Time::<Fixed>::from_hz(60.0));
    app.insert_resource(RaycastResult::default());

    app.finish();
    app.cleanup();

    // Spawn a static box at origin
    let ground = app.world_mut().spawn((
        Transform::from_xyz(0.0, -10.0, 0.0),
        GlobalTransform::default(),
        RigidBody::Static,
        Collider::rectangle(100.0, 10.0),
    )).id();

    // Run several frames to let physics initialize
    for _ in 0..10 {
        app.update();
    }

    println!("Ground entity: {:?}", ground);
    println!("Ground transform: {:?}", app.world().get::<GlobalTransform>(ground));
    println!("Ground collider: {:?}", app.world().get::<Collider>(ground));
    println!("Ground RigidBody: {:?}", app.world().get::<RigidBody>(ground));

    // Run the raycast system
    app.world_mut().run_system_once(perform_raycast);

    let result = app.world().resource::<RaycastResult>();
    assert!(result.hit.is_some(), "Raycast should hit the ground box");
}
