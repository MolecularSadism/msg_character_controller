//! Ballpit Example
//!
//! A playable example demonstrating dynamic ground interaction.
//! The character can stand on dynamic balls and push them down through
//! the spring force reaction system (Newton's 3rd law).
//!
//! ## Features
//! - Dynamic balls that respond to the player standing on them
//! - Walls to contain the balls
//! - A floor that the balls rest on
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
//! cargo run --example ballpit --features examples
//!
//! # With Rapier2D:
//! cargo run --example ballpit --features "examples,rapier2d" --no-default-features
//! ```

mod helpers;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use helpers::{
    ActiveBackend, CharacterControllerUiPlugin, ControlsPlugin, DefaultControllerSettings,
    ExamplePhysicsPlugin, Player, PlayerSpawnConfig, SpawnConfig, backend_name,
    spawn_dynamic_ball, spawn_player, spawn_static_box,
};
use msg_character_controller::prelude::*;

// ==================== Constants ====================

const PLAYER_HALF_HEIGHT: f32 = 8.0;
const PLAYER_RADIUS: f32 = 6.0;

const BOX_WIDTH: f32 = 600.0;
const BOX_HEIGHT: f32 = 400.0;
const WALL_THICKNESS: f32 = 20.0;

const BALL_RADIUS: f32 = 12.0;
const BALL_COUNT: u32 = 40;

const PX_PER_M: f32 = 10.0;

// ==================== Main ====================

fn spawn_position() -> Vec2 {
    Vec2::new(0.0, BOX_HEIGHT / 2.0 - 50.0)
}

fn default_gravity() -> Vec2 {
    Vec2::new(0.0, -9.81 * PX_PER_M)
}

fn default_config() -> ControllerConfig {
    ControllerConfig::default()
        .with_float_height(6.0)
        .with_spring(400.0, 15.0)
        .with_spring_max_force(4000.0)
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!("Ballpit ({}) - Dynamic Ground Reaction Example", backend_name()),
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
    spawn_balls(&mut commands, &mut meshes, &mut materials);

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
        Text::new("A/D: Move | W: Jump | Space: Propel Up | S: Propel Down\nStand on balls to push them down!"),
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

fn spawn_balls(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    let half_width = BOX_WIDTH / 2.0 - WALL_THICKNESS - BALL_RADIUS - 5.0;
    let base_y = -BOX_HEIGHT / 2.0 + BALL_RADIUS + 5.0;

    // Spawn balls in a grid pattern
    let cols: u32 = 8;
    let rows = BALL_COUNT / cols + 1;
    let cols_f32 = cols as f32;
    let ball_count_f32 = BALL_COUNT as f32;
    let spacing_x = (half_width * 2.0) / cols_f32;
    let spacing_y = BALL_RADIUS * 2.5;

    let mut count: u32 = 0;
    for row in 0..rows {
        let row_f32 = row as f32;
        for col in 0..cols {
            if count >= BALL_COUNT {
                break;
            }

            // Offset every other row for a more natural look
            let x_offset = if row % 2 == 0 { 0.0 } else { spacing_x / 2.0 };
            let column_f32 = col as f32;
            let x = -half_width + spacing_x / 2.0 + column_f32 * spacing_x + x_offset;
            let y = base_y + row_f32 * spacing_y;

            // Vary ball color slightly for visual interest
            let count_f32 = count as f32;
            let hue = (count_f32 / ball_count_f32) * 0.3 + 0.55; // Blue to purple range
            let color = Color::hsl(hue * 360.0, 0.7, 0.5);

            spawn_dynamic_ball(
                commands,
                meshes,
                materials,
                Vec2::new(x, y),
                BALL_RADIUS,
                color,
            );

            count += 1;
        }
    }
}
