//! Stairs Example
//!
//! A playable example demonstrating stair stepping with multiple floors connected
//! by staircases of different step heights:
//!
//! - Staircase 1 (left side): step height 4px — many shallow steps
//! - Staircase 2 (right side): step height 8px — moderate steps
//! - Staircase 3 (left side): step height 12px — tall steps (requires raised max_climb_height)
//! - Staircase 4 (right side): alternating step heights 4px and 12px
//!
//! The layout follows a zigzag pattern where each floor extends from the opposite
//! side to where the next staircase is located:
//!
//! ```text
//! |__________________________        |   <- Platform 4 (left)
//! |          _______________xxxxxx|  <- Platform 3 (right), stairs on right
//! |xxxxxx___________________      |  <- Platform 2 (left),  stairs on left
//! |          _______________xxxxxx|  <- Platform 1 (right), stairs on right
//! |xxxxxx_________________________|  <- Ground floor,        stairs on left
//! ```
//!
//! ## Controls
//! See the [`ControlsPlugin`](helpers::ControlsPlugin) docs for the full controls reference.
//!
//! ## Running
//! ```bash
//! cargo run --example stairs --features examples
//! ```

mod helpers;

use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use helpers::{
    ActiveBackend, CharacterControllerUiPlugin, ControlsPlugin, DefaultControllerSettings,
    ExamplePhysicsPlugin, Player, PlayerSpawnConfig, SpawnConfig, backend_name, spawn_player,
    spawn_static_box,
};
use msg_character_controller::prelude::*;

// ==================== Constants ====================

const PLAYER_HALF_HEIGHT: f32 = 8.0;
const PLAYER_RADIUS: f32 = 6.0;

const BOX_WIDTH: f32 = 900.0;
const BOX_HEIGHT: f32 = 800.0;
const WALL_THICKNESS: f32 = 20.0;
const PLATFORM_THICKNESS: f32 = 15.0;

const PX_PER_M: f32 = 10.0;

/// Interior x bounds (left wall inner face to right wall inner face).
const INTERIOR_LEFT: f32 = -(BOX_WIDTH / 2.0) + WALL_THICKNESS;
const INTERIOR_RIGHT: f32 = (BOX_WIDTH / 2.0) - WALL_THICKNESS;

/// Y coordinate of each level's top surface.
const LEVEL_HEIGHT_STEP: f32 = 96.0;
const LEVEL_0_Y: f32 = -(BOX_HEIGHT / 2.0); // ground surface
const LEVEL_1_Y: f32 = LEVEL_0_Y + LEVEL_HEIGHT_STEP;
const LEVEL_2_Y: f32 = LEVEL_1_Y + LEVEL_HEIGHT_STEP;
const LEVEL_3_Y: f32 = LEVEL_2_Y + LEVEL_HEIGHT_STEP;
const LEVEL_4_Y: f32 = LEVEL_3_Y + LEVEL_HEIGHT_STEP;

/// Horizontal depth of each step in all staircases.
const STEP_DEPTH: f32 = 8.0;

// ==================== Main ====================

fn spawn_position() -> Vec2 {
    // Spawn in the open ground area, clear of SC1 (which ends at x ≈ -238).
    Vec2::new(0.0, LEVEL_0_Y + WALL_THICKNESS + 50.0)
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
                title: format!("Stairs ({}) - Character Controller Example", backend_name()),
                resolution: (1280, 720).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(ExamplePhysicsPlugin::new(PX_PER_M))
        .add_plugins(CharacterControllerPlugin::<ActiveBackend>::default())
        .add_plugins(ControlsPlugin::default())
        .add_plugins(EguiPlugin::default())
        .insert_resource(SpawnConfig::new(spawn_position()))
        .insert_resource(DefaultControllerSettings::new(
            default_config(),
            default_gravity(),
        ))
        .add_plugins(CharacterControllerUiPlugin::<Player>::default())
        .add_systems(Startup, (setup, configure_stair_climbing).chain())
        .run();
}

// ==================== Setup ====================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    spawn_enclosure(&mut commands, &mut meshes, &mut materials);
    spawn_levels(&mut commands, &mut meshes, &mut materials);

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

/// Raise max_climb_height so the 12px steps are climbable (default is 11.0).
fn configure_stair_climbing(mut query: Query<&mut CharacterController, With<Player>>) {
    if let Ok(mut controller) = query.single_mut() {
        controller.stair_config = Some(StairConfig::default().with_max_climb_height(13.0));
    }
}

// ==================== Environment ====================

/// Spawn the outer box (floor, ceiling, walls).
fn spawn_enclosure(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    let hw = BOX_WIDTH / 2.0;
    let hh = BOX_HEIGHT / 2.0;
    let hwt = WALL_THICKNESS / 2.0;
    let wall_color = Color::srgb(0.3, 0.3, 0.3);

    // Floor
    spawn_static_box(
        commands,
        meshes,
        materials,
        Vec2::new(0.0, -hh - hwt),
        Vec2::new(hw, hwt),
        wall_color,
    );
    // Ceiling
    spawn_static_box(
        commands,
        meshes,
        materials,
        Vec2::new(0.0, hh + hwt),
        Vec2::new(hw, hwt),
        wall_color,
    );
    // Left wall
    spawn_static_box(
        commands,
        meshes,
        materials,
        Vec2::new(-hw + hwt, 0.0),
        Vec2::new(hwt, hh),
        wall_color,
    );
    // Right wall
    spawn_static_box(
        commands,
        meshes,
        materials,
        Vec2::new(hw - hwt, 0.0),
        Vec2::new(hwt, hh),
        wall_color,
    );
}

fn spawn_levels(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    let platform_color = Color::srgb(0.35, 0.50, 0.30);
    let stair_color_1 = Color::srgb(0.60, 0.45, 0.30); // h=4  stairs
    let stair_color_2 = Color::srgb(0.55, 0.35, 0.55); // h=8  stairs
    let stair_color_3 = Color::srgb(0.30, 0.45, 0.60); // h=12 stairs
    let stair_color_4 = Color::srgb(0.60, 0.55, 0.25); // alternating stairs

    // SC1: ground → Level 1, left side, step height 4 (24 steps).
    // Lowest step faces right so the character approaches from the open ground floor.
    let sc1_steps: Vec<f32> = vec![4.0; 24];
    let sc1_end_x = INTERIOR_LEFT + sc1_steps.len() as f32 * STEP_DEPTH;
    spawn_staircase(
        commands,
        meshes,
        materials,
        sc1_end_x,
        LEVEL_0_Y,
        &sc1_steps,
        STEP_DEPTH,
        false,
        stair_color_1,
    );
    spawn_platform(
        commands,
        meshes,
        materials,
        sc1_end_x,
        INTERIOR_RIGHT,
        LEVEL_1_Y,
        platform_color,
    );

    // SC2: Level 1 → Level 2, right side, step height 8 (12 steps).
    // Lowest step faces left so the character approaches from Platform 1.
    let sc2_steps: Vec<f32> = vec![8.0; 12];
    let sc2_end_x = INTERIOR_RIGHT - sc2_steps.len() as f32 * STEP_DEPTH;
    spawn_staircase(
        commands,
        meshes,
        materials,
        sc2_end_x,
        LEVEL_1_Y,
        &sc2_steps,
        STEP_DEPTH,
        true,
        stair_color_2,
    );
    spawn_platform(
        commands,
        meshes,
        materials,
        INTERIOR_LEFT,
        sc2_end_x,
        LEVEL_2_Y,
        platform_color,
    );

    // SC3: Level 2 → Level 3, left side, step height 12 (8 steps).
    // Lowest step faces right so the character approaches from Platform 2.
    let sc3_steps: Vec<f32> = vec![12.0; 8];
    let sc3_end_x = INTERIOR_LEFT + sc3_steps.len() as f32 * STEP_DEPTH;
    spawn_staircase(
        commands,
        meshes,
        materials,
        sc3_end_x,
        LEVEL_2_Y,
        &sc3_steps,
        STEP_DEPTH,
        false,
        stair_color_3,
    );
    spawn_platform(
        commands,
        meshes,
        materials,
        sc3_end_x,
        INTERIOR_RIGHT,
        LEVEL_3_Y,
        platform_color,
    );

    // SC4: Level 3 → Level 4, right side, alternating step heights 4 and 12 (12 steps).
    // Lowest step faces left so the character approaches from Platform 3.
    let sc4_steps: Vec<f32> = (0..12)
        .map(|i| if i % 2 == 0 { 4.0 } else { 12.0 })
        .collect();
    let sc4_end_x = INTERIOR_RIGHT - sc4_steps.len() as f32 * STEP_DEPTH;
    spawn_staircase(
        commands,
        meshes,
        materials,
        sc4_end_x,
        LEVEL_3_Y,
        &sc4_steps,
        STEP_DEPTH,
        true,
        stair_color_4,
    );
    spawn_platform(
        commands,
        meshes,
        materials,
        INTERIOR_LEFT,
        sc4_end_x,
        LEVEL_4_Y,
        platform_color,
    );
}

// ==================== Helpers ====================

/// Spawn a horizontal platform box whose top surface is at `top_y`, spanning `x_left..x_right`.
fn spawn_platform(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    x_left: f32,
    x_right: f32,
    top_y: f32,
    color: Color,
) {
    let half_w = (x_right - x_left) / 2.0;
    let half_h = PLATFORM_THICKNESS / 2.0;
    let center_x = (x_left + x_right) / 2.0;
    let center_y = top_y - half_h;
    spawn_static_box(
        commands,
        meshes,
        materials,
        Vec2::new(center_x, center_y),
        Vec2::new(half_w, half_h),
        color,
    );
}

fn spawn_staircase(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    start_x: f32,
    base_y: f32,
    step_heights: &[f32],
    step_depth: f32,
    ascending_right: bool,
    color: Color,
) {
    let half_depth = step_depth / 2.0;
    let dir = if ascending_right { 1.0 } else { -1.0 };

    let mut cumulative_height = 0.0;
    for (i, &step_h) in step_heights.iter().enumerate() {
        cumulative_height += step_h;
        let half_h = cumulative_height / 2.0;
        let center_x = start_x + dir * (i as f32 * step_depth + half_depth);
        let center_y = base_y + half_h;
        spawn_static_box(
            commands,
            meshes,
            materials,
            Vec2::new(center_x, center_y),
            Vec2::new(half_depth, half_h),
            color,
        );
    }
}
