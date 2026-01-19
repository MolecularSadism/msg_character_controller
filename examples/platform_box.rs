//! Platform Box Example
//!
//! A playable example with a character in a box environment featuring:
//! - A floor
//! - Walls on both sides
//! - A platform in the middle
//! - A triangle slope on the right
//!
//! ## Controls
//! - **A/D** or **Left/Right**: Move horizontally
//! - **W/Up**: Jump
//! - **Space** (hold): Propulsion (fly upward)
//! - **S/Down** (hold): Propulsion (fly downward)
//!
//! The camera follows the player.
//!
//! ## Running
//! ```bash
//! # With Avian2D (default):
//! cargo run --example platform_box --features examples
//!
//! # With Rapier2D:
//! cargo run --example platform_box --features "examples,rapier2d" --no-default-features
//! ```

mod helpers;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use helpers::{
    ActiveBackend, CharacterControllerUiPlugin, ControlsPlugin, DefaultControllerSettings,
    ExamplePhysicsPlugin, Player, PlayerSpawnConfig, SpawnConfig, backend_name,
    spawn_player, spawn_static_ball, spawn_static_box, spawn_static_slope,
};
use msg_character_controller::prelude::*;

// ==================== Constants ====================

const PLAYER_HALF_HEIGHT: f32 = 8.0;
const PLAYER_RADIUS: f32 = 6.0;

const BOX_WIDTH: f32 = 800.0;
const BOX_HEIGHT: f32 = 600.0;
const WALL_THICKNESS: f32 = 20.0;

const PLATFORM_WIDTH: f32 = 200.0;
const PLATFORM_HEIGHT: f32 = 20.0;
const PLATFORM_Y: f32 = 100.0;

const PX_PER_M: f32 = 10.0;

// ==================== Main ====================

fn spawn_position() -> Vec2 {
    Vec2::new(-200.0, -BOX_HEIGHT / 2.0 + WALL_THICKNESS + 50.0)
}

fn default_gravity() -> Vec2 {
    Vec2::new(0.0, -9.81 * PX_PER_M)
}

fn default_config() -> ControllerConfig {
    ControllerConfig::default().with_float_height(6.0)
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!("Platform Box ({}) - Character Controller Example", backend_name()).into(),
                resolution: (1280, 720).into(),
                ..default()
            }),
            ..default()
        }))
        // Physics (backend-agnostic)
        .add_plugins(ExamplePhysicsPlugin::new(PX_PER_M))
        // Character controller
        .add_plugins(CharacterControllerPlugin::<ActiveBackend>::default())
        // Controls (input handling and camera follow)
        .add_plugins(ControlsPlugin::default())
        // Egui for settings UI
        .add_plugins(EguiPlugin::default())
        // Configure spawn position and default settings for the UI plugin
        .insert_resource(SpawnConfig::new(spawn_position()))
        .insert_resource(DefaultControllerSettings::new(
            default_config(),
            default_gravity(),
        ))
        // Character controller UI panels (unified plugin with settings + diagnostics)
        .add_plugins(CharacterControllerUiPlugin::<Player>::default())
        // Systems
        .add_systems(Startup, setup)
        .run();
}

// ==================== Setup ====================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Spawn environment
    spawn_box(&mut commands, &mut meshes, &mut materials);
    spawn_obstacles(&mut commands, &mut meshes, &mut materials);
    spawn_slope(&mut commands, &mut meshes, &mut materials);

    // Spawn player
    spawn_player(
        &mut commands,
        &mut meshes,
        &mut materials,
        PlayerSpawnConfig {
            position: spawn_position(),
            half_height: PLAYER_HALF_HEIGHT,
            radius: PLAYER_RADIUS,
            gravity: default_gravity(),
            config: default_config(),
            ..default()
        },
    );

    // UI instructions - use Pickable::IGNORE to prevent blocking mouse events
    commands.spawn((
        Text::new("A/D: Move | W: Jump | Space: Propel Up | S: Propel Down"),
        TextFont {
            font_size: 20.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        Pickable::IGNORE,
    ));
}

fn spawn_box(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    let half_width = BOX_WIDTH / 2.0;
    let half_height = BOX_HEIGHT / 2.0;
    let half_wall = WALL_THICKNESS / 2.0;

    // Floor
    spawn_static_box(
        commands,
        meshes,
        materials,
        Vec2::new(0.0, -half_height - half_wall),
        Vec2::new(half_width, half_wall),
        Color::srgb(0.3, 0.3, 0.3),
    );

    // Ceiling
    spawn_static_box(
        commands,
        meshes,
        materials,
        Vec2::new(0.0, half_height + half_wall),
        Vec2::new(half_width, half_wall),
        Color::srgb(0.3, 0.3, 0.3),
    );

    // Left wall
    spawn_static_box(
        commands,
        meshes,
        materials,
        Vec2::new(-half_width + half_wall, 0.0),
        Vec2::new(half_wall, half_height),
        Color::srgb(0.3, 0.3, 0.3),
    );

    // Right wall
    spawn_static_box(
        commands,
        meshes,
        materials,
        Vec2::new(half_width - half_wall, 0.0),
        Vec2::new(half_wall, half_height),
        Color::srgb(0.3, 0.3, 0.3),
    );
}

fn spawn_obstacles(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    spawn_static_box(
        commands,
        meshes,
        materials,
        Vec2::new(0.0, PLATFORM_Y),
        Vec2::new(PLATFORM_WIDTH / 2.0, PLATFORM_HEIGHT / 2.0),
        Color::srgb(0.4, 0.5, 0.3),
    );

    for i in 0..=5 {
        spawn_static_ball(
            commands,
            meshes,
            materials,
            Vec2::new(50.0 + 13.0 * i as f32, -BOX_HEIGHT / 2.0 + 10.0 * i as f32),
            5.0,
            Color::srgb(0.8, 0.2, 0.2),
        );
    }

    for i in 0..=5 {
        spawn_static_ball(
            commands,
            meshes,
            materials,
            Vec2::new(
                -BOX_WIDTH / 2.0 + WALL_THICKNESS + 150.0 + 20.0 * i as f32,
                -BOX_HEIGHT / 2.0 + 5.0,
            ),
            5.0,
            Color::srgb(0.8, 0.2, 0.2),
        );
    }

    for i in 0..=5 {
        spawn_static_ball(
            commands,
            meshes,
            materials,
            Vec2::new(
                -BOX_WIDTH / 2.0 + WALL_THICKNESS + 20.0 + 20.0 * i as f32,
                -BOX_HEIGHT / 2.0,
            ),
            5.0,
            Color::srgb(0.8, 0.2, 0.2),
        );
    }
}

fn spawn_slope(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    let slope_x = 250.0;
    let slope_y = -BOX_HEIGHT / 2.0;

    // Triangle vertices (relative to center)
    let vertices = [
        Vec2::new(-80.0, 0.0),  // Bottom left
        Vec2::new(80.0, 0.0),   // Bottom right
        Vec2::new(80.0, 100.0), // Top right
    ];

    spawn_static_slope(
        commands,
        meshes,
        materials,
        Vec2::new(slope_x, slope_y),
        &vertices,
        Color::srgb(0.5, 0.4, 0.3),
    );
}
