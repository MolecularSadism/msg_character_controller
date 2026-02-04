//! Physics backend abstraction for examples.
//!
//! This module provides helpers for spawning physics entities using Avian2D.

use bevy::prelude::*;
use bevy::time::Fixed;
use msg_character_controller::prelude::*;

use super::{Player, create_capsule_mesh, create_circle_mesh, create_rectangle_mesh, create_triangle_mesh};

// Re-export Avian2D types for convenience
pub use avian2d::prelude::*;

// ==================== Shared Physics Constants ====================

/// Fixed update rate in Hz.
pub const FIXED_UPDATE_HZ: f64 = 60.0;

/// Default pixels per meter conversion.
pub const DEFAULT_PIXELS_PER_METER: f32 = 10.0;

// ==================== Backend Type Alias ====================

/// The active physics backend type.
pub type ActiveBackend = Avian2dBackend;

// ==================== Backend Name ====================

/// Returns the name of the active physics backend.
pub fn backend_name() -> &'static str {
    "Avian2D"
}

// ==================== Physics Plugin ====================

/// Adds the physics plugin for the active backend.
pub struct ExamplePhysicsPlugin {
    pub pixels_per_meter: f32,
}

impl Default for ExamplePhysicsPlugin {
    fn default() -> Self {
        Self {
            pixels_per_meter: DEFAULT_PIXELS_PER_METER,
        }
    }
}

impl ExamplePhysicsPlugin {
    pub fn new(pixels_per_meter: f32) -> Self {
        Self { pixels_per_meter }
    }
}

impl Plugin for ExamplePhysicsPlugin {
    fn build(&self, app: &mut App) {
        // Set fixed timestep for consistent physics simulation
        app.insert_resource(Time::<Fixed>::from_hz(FIXED_UPDATE_HZ));

        app.add_plugins(PhysicsPlugins::default().with_length_unit(self.pixels_per_meter));
        app.add_plugins(PhysicsDebugPlugin)
                .insert_gizmo_config(
            PhysicsGizmos {
                collider_color: Some(Color::WHITE),
                raycast_color: Some(Color::WHITE),
                raycast_normal_color: Some(Color::WHITE),
                shapecast_color: Some(Color::WHITE),
                shapecast_normal_color: Some(Color::WHITE),
                ..default()
            },
            GizmoConfig::default(),
        )
        ;
    }
}

// ==================== Static Collider Spawning ====================

/// Spawns a static rectangular collider (wall, floor, platform, etc.).
///
/// # Arguments
/// * `commands` - Bevy commands
/// * `meshes` - Mesh assets
/// * `materials` - Color material assets
/// * `position` - Center position of the collider
/// * `half_size` - Half-width and half-height of the rectangle
/// * `color` - Visual color
pub fn spawn_static_box(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    position: Vec2,
    half_size: Vec2,
    color: Color,
) {
    let mesh = meshes.add(create_rectangle_mesh(half_size.x, half_size.y));
    let material = materials.add(ColorMaterial::from_color(color));

    let entity = commands.spawn((
        Transform::from_translation(position.extend(0.0)),
        GlobalTransform::default(),
        RigidBody::Static,
        Collider::rectangle(half_size.x * 2.0, half_size.y * 2.0),
        Mesh2d(mesh),
        MeshMaterial2d(material),
    )).id();
    eprintln!("spawn_static_box: Created platform entity {:?} at position {:?} with half_size {:?}", entity, position, half_size);
}

/// Spawns a static rectangular collider with rotation.
pub fn spawn_static_box_rotated(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    position: Vec2,
    half_size: Vec2,
    rotation: Quat,
    color: Color,
) {
    let mesh = meshes.add(create_rectangle_mesh(half_size.x, half_size.y));
    let material = materials.add(ColorMaterial::from_color(color));

    commands.spawn((
        Transform::from_translation(position.extend(0.0)).with_rotation(rotation),
        GlobalTransform::default(),
        RigidBody::Static,
        Collider::rectangle(half_size.x * 2.0, half_size.y * 2.0),
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}

/// Spawns a static circular collider.
pub fn spawn_static_ball(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    position: Vec2,
    radius: f32,
    color: Color,
) {
    let mesh = meshes.add(create_circle_mesh(radius, 24));
    let material = materials.add(ColorMaterial::from_color(color));

    commands.spawn((
        Transform::from_translation(position.extend(0.0)),
        GlobalTransform::default(),
        RigidBody::Static,
        Collider::circle(radius),
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}

/// Spawns a static triangle slope collider.
pub fn spawn_static_slope(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    position: Vec2,
    vertices: &[Vec2; 3],
    color: Color,
) {
    let mesh = meshes.add(create_triangle_mesh(vertices));
    let material = materials.add(ColorMaterial::from_color(color));

    let collider = Collider::convex_hull(vertices.to_vec())
        .expect("Failed to create slope collider");
    commands.spawn((
        Transform::from_translation(position.extend(0.0)),
        GlobalTransform::default(),
        RigidBody::Static,
        collider,
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}

/// Spawns a static triangle slope collider with rotation.
pub fn spawn_static_slope_rotated(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    position: Vec2,
    vertices: &[Vec2; 3],
    rotation: Quat,
    color: Color,
) {
    let mesh = meshes.add(create_triangle_mesh(vertices));
    let material = materials.add(ColorMaterial::from_color(color));

    let collider = Collider::convex_hull(vertices.to_vec())
        .expect("Failed to create slope collider");
    commands.spawn((
        Transform::from_translation(position.extend(0.0)).with_rotation(rotation),
        GlobalTransform::default(),
        RigidBody::Static,
        collider,
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}

// ==================== Dynamic Collider Spawning ====================

/// Spawns a dynamic ball (for ballpit, etc.).
pub fn spawn_dynamic_ball(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    position: Vec2,
    radius: f32,
    color: Color,
) {
    let mesh = meshes.add(create_circle_mesh(radius, 24));
    let material = materials.add(ColorMaterial::from_color(color));

    commands.spawn((
        Transform::from_translation(position.extend(0.0)),
        GlobalTransform::default(),
        RigidBody::Dynamic,
        Collider::circle(radius),
        ConstantForce::default(),
        Restitution::new(0.3),
        Friction::new(0.5),
        LinearDamping(0.2),
        AngularDamping(0.5),
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}

// ==================== Player Spawning ====================

/// Player spawn configuration.
pub struct PlayerSpawnConfig {
    pub position: Vec2,
    pub half_height: f32,
    pub radius: f32,
    pub gravity: Vec2,
    pub config: ControllerConfig,
    pub color: Color,
    /// If true, locks rotation (for platformers).
    pub lock_rotation: bool,
    /// Angular damping (for spherical planets where rotation is used).
    pub angular_damping: f32,
}

impl Default for PlayerSpawnConfig {
    fn default() -> Self {
        Self {
            position: Vec2::ZERO,
            half_height: 8.0,
            radius: 6.0,
            gravity: Vec2::new(0.0, -98.1),
            config: ControllerConfig::default(),
            color: Color::srgb(0.2, 0.6, 0.9),
            lock_rotation: false,
            angular_damping: 0.0,
        }
    }
}

/// Spawns a player character with the active physics backend.
pub fn spawn_player(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    spawn_config: PlayerSpawnConfig,
) {
    let mesh = meshes.add(create_capsule_mesh(
        spawn_config.half_height / 2.0,
        spawn_config.radius,
        12,
    ));
    let material = materials.add(ColorMaterial::from_color(spawn_config.color));

    let mut entity = commands.spawn((
        Player,
        Transform::from_translation(spawn_config.position.extend(1.0)),
        GlobalTransform::default(),
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));

    entity.insert((
        CharacterController::with_gravity(spawn_config.gravity),
        spawn_config.config,
        MovementIntent::default(),
    ));

    entity.insert((
        Collider::capsule(spawn_config.radius, spawn_config.half_height / 2.0),
        GravityScale(0.0),
    ));

    if spawn_config.lock_rotation {
        entity.insert(LockedAxes::ROTATION_LOCKED);
    }

    if spawn_config.angular_damping > 0.0 {
        entity.insert(AngularDamping(spawn_config.angular_damping));
    }
}

// ==================== Polyline Collider (for hilly planet) ====================

/// Spawns a static polyline collider (for hilly planet terrain).
pub fn spawn_polyline_collider(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    position: Vec2,
    vertices: Vec<Vec2>,
    color: Color,
) {
    use super::create_polygon_mesh;

    // Create indices for closed polyline
    let mut indices: Vec<[u32; 2]> = Vec::with_capacity(vertices.len());
    for i in 0..vertices.len() {
        let next = (i + 1) % vertices.len();
        indices.push([i as u32, next as u32]);
    }

    let collider = Collider::polyline(vertices.clone(), Some(indices));
    let mesh = meshes.add(create_polygon_mesh(&vertices));
    let material = materials.add(ColorMaterial::from_color(color));

    commands.spawn((
        Transform::from_translation(position.extend(-1.0)),
        GlobalTransform::default(),
        RigidBody::Static,
        collider,
        Mesh2d(mesh),
        MeshMaterial2d(material),
    ));
}
