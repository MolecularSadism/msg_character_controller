//! Spherical Planet Example
//!
//! A playable example with a character walking on a spherical planet featuring:
//! - A circular planet with radial gravity
//! - A platform on the planet surface
//! - A triangle slope structure
//! - Dynamic orientation that adjusts to the planet's surface
//!
//! ## Controls
//! - **A/D** or **Left/Right**: Move horizontally
//! - **W/Up**: Jump
//! - **Space** (hold): Propulsion (fly up relative to planet)
//! - **S/Down** (hold): Propulsion (fly down toward planet)
//!
//! The camera follows the player and the character's "up" direction
//! always points away from the planet center.
//!
//! ## Running
//! ```bash
//! # With Avian2D (default):
//! cargo run --example spherical_planet --features examples
//!
//! # With Rapier2D:
//! cargo run --example spherical_planet --features "examples,rapier2d" --no-default-features
//! ```

mod helpers;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use helpers::{
    ActiveBackend, CharacterControllerUiPlugin, CharacterControllerUiState, ControlsPlugin,
    DefaultControllerSettings, ExamplePhysicsPlugin, Player, PlayerSpawnConfig, SpawnConfig,
    backend_name, spawn_player, spawn_static_ball, spawn_static_box_rotated, spawn_static_slope_rotated,
};
use msg_character_controller::prelude::*;

// ==================== Constants ====================

const PLAYER_SIZE: f32 = 16.0;
const PLAYER_HALF_HEIGHT: f32 = 8.0;
const PLAYER_RADIUS: f32 = 6.0;

const PLANET_RADIUS: f32 = 300.0;
const PLANET_CENTER: Vec2 = Vec2::ZERO;

const PLATFORM_WIDTH: f32 = 80.0;
const PLATFORM_HEIGHT: f32 = 15.0;

const GRAVITY_STRENGTH: f32 = 100.0;

// ==================== Components ====================

/// Component storing the planet center for gravity calculations.
#[derive(Resource)]
struct PlanetConfig {
    center: Vec2,
    gravity_strength: f32,
}

impl Default for PlanetConfig {
    fn default() -> Self {
        Self {
            center: PLANET_CENTER,
            gravity_strength: GRAVITY_STRENGTH,
        }
    }
}

// ==================== Config Functions ====================

fn spawn_position() -> Vec2 {
    // Spawn on top of the planet
    PLANET_CENTER + Vec2::Y * (PLANET_RADIUS + PLAYER_SIZE + 30.0)
}

fn default_config() -> ControllerConfig {
    ControllerConfig::default()
        .with_float_height(PLAYER_HALF_HEIGHT)
        .with_ground_cast_width(PLAYER_RADIUS)
        .with_upright_torque_enabled(false) // We handle rotation via orientation
}

// ==================== Main ====================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!("Spherical Planet ({}) - Character Controller Example", backend_name()).into(),
                resolution: (1280, 720).into(),
                ..default()
            }),
            ..default()
        }))
        // Physics (backend-agnostic)
        .add_plugins(ExamplePhysicsPlugin::new(100.0))
        // Character controller
        .add_plugins(CharacterControllerPlugin::<ActiveBackend>::default())
        // Controls (input handling only - we have custom camera follow for planet)
        .add_plugins(ControlsPlugin::default())
        // Egui for settings UI
        .add_plugins(EguiPlugin::default())
        // Resources
        .init_resource::<PlanetConfig>()
        // Configure spawn position and default settings for the UI plugin
        .insert_resource(SpawnConfig::with_dynamic_position(spawn_position))
        .insert_resource(DefaultControllerSettings::new(
            default_config(),
            // Initial gravity will be updated by the orientation system
            -Vec2::Y * GRAVITY_STRENGTH,
        ))
        // Character controller UI panels (unified plugin with settings + diagnostics)
        .add_plugins(CharacterControllerUiPlugin::<Player>::default())
        // Systems
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            // Update orientation and gravity before controller systems
            update_player_orientation_and_gravity.before(
                msg_character_controller::systems::accumulate_spring_force::<ActiveBackend>,
            ),
        )
        // Extra settings UI for planet-specific configuration
        .add_systems(EguiPrimaryContextPass, planet_gravity_settings_ui)
        .run();
}

// ==================== Setup ====================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Spawn planet
    spawn_static_ball(
        &mut commands,
        &mut meshes,
        &mut materials,
        PLANET_CENTER,
        PLANET_RADIUS,
        Color::srgb(0.25, 0.35, 0.25),
    );

    // Spawn platform on top of planet
    spawn_surface_platform(
        &mut commands,
        &mut meshes,
        &mut materials,
        0.0,
        PLATFORM_WIDTH,
        PLATFORM_HEIGHT,
    );

    // Spawn slope structure on the side
    spawn_surface_slope(
        &mut commands,
        &mut meshes,
        &mut materials,
        std::f32::consts::FRAC_PI_4,
    );

    // Spawn player on top of planet
    let spawn_angle = std::f32::consts::FRAC_PI_2;
    let direction = Vec2::new(spawn_angle.cos(), spawn_angle.sin());
    let spawn_pos = PLANET_CENTER + direction * (PLANET_RADIUS + 50.0);
    let initial_gravity = -direction * GRAVITY_STRENGTH;

    spawn_player(
        &mut commands,
        &mut meshes,
        &mut materials,
        PlayerSpawnConfig {
            position: spawn_pos,
            half_height: PLAYER_HALF_HEIGHT,
            radius: PLAYER_RADIUS,
            gravity: initial_gravity,
            config: default_config(),
            angular_damping: 5.0,
            ..default()
        },
    );

    // UI instructions
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

    // Planet info
    commands.spawn((
        Text::new("Walking on a spherical planet with radial gravity"),
        TextFont {
            font_size: 18.0,
            ..default()
        },
        TextColor(Color::srgb(0.7, 0.7, 0.7)),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        Pickable::IGNORE,
    ));
}

/// Spawn a platform on the planet surface at a given angle (radians from top).
fn spawn_surface_platform(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    angle_offset: f32,
    width: f32,
    height: f32,
) {
    // Angle 0 = top of planet, positive = clockwise
    let surface_angle = std::f32::consts::FRAC_PI_2 - angle_offset;
    let direction = Vec2::new(surface_angle.cos(), surface_angle.sin());
    let position = PLANET_CENTER + direction * (PLANET_RADIUS + height / 2.0 + 20.0);

    // Rotation to align with surface (tangent)
    let rotation = Quat::from_rotation_z(surface_angle - std::f32::consts::FRAC_PI_2);

    spawn_static_box_rotated(
        commands,
        meshes,
        materials,
        position,
        Vec2::new(width / 2.0, height / 2.0),
        rotation,
        Color::srgb(0.4, 0.5, 0.3),
    );
}

/// Spawn a triangular slope on the planet surface.
fn spawn_surface_slope(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    angle_offset: f32,
) {
    // Place slope on the surface
    let surface_angle = std::f32::consts::FRAC_PI_2 - angle_offset;
    let direction = Vec2::new(surface_angle.cos(), surface_angle.sin());
    let position = PLANET_CENTER + direction * (PLANET_RADIUS + 25.0);

    // Rotation to align with surface
    let rotation = Quat::from_rotation_z(surface_angle - std::f32::consts::FRAC_PI_2);

    // Triangle vertices in local space
    let vertices = [
        Vec2::new(-40.0, 0.0), // Bottom left
        Vec2::new(40.0, 0.0),  // Bottom right
        Vec2::new(40.0, 50.0), // Top right
    ];

    spawn_static_slope_rotated(
        commands,
        meshes,
        materials,
        position,
        &vertices,
        rotation,
        Color::srgb(0.5, 0.4, 0.3),
    );
}

// ==================== Planetary Systems ====================

/// Updates the player's gravity to match the planet.
///
/// Gravity points toward the planet center and is stored in CharacterController.
/// The up direction is derived from gravity via controller.ideal_up().
/// The internal gravity system then applies it when not grounded.
fn update_player_orientation_and_gravity(
    planet: Res<PlanetConfig>,
    mut query: Query<(&mut Transform, &mut CharacterController), With<Player>>,
) {
    for (mut transform, mut controller) in &mut query {
        let position = transform.translation.xy();
        let to_player = position - planet.center;
        let new_up = to_player.normalize_or_zero();

        if new_up != Vec2::ZERO {
            // Update gravity to point toward planet center
            // Up direction is automatically derived from gravity via controller.ideal_up()
            controller.gravity = -new_up * planet.gravity_strength;

            // Rotate the transform to visually match the up direction
            let angle = new_up.to_angle() - std::f32::consts::FRAC_PI_2;
            transform.rotation = Quat::from_rotation_z(angle);
        }
    }
}

// ==================== Planet Gravity Settings UI ====================

/// Extra settings UI for planet-specific configuration.
/// This adds a "Planet Gravity" section to the Controller Settings window.
fn planet_gravity_settings_ui(
    mut contexts: EguiContexts,
    mut planet_config: ResMut<PlanetConfig>,
    ui_state: Res<CharacterControllerUiState>,
) {
    // Skip if panels are hidden or not yet initialized
    if ui_state.frame_count <= 2 || !ui_state.show_panels {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    // Planet-specific settings in a separate window
    egui::Window::new("Planet Settings")
        .default_pos([10.0, 500.0])
        .default_width(300.0)
        .collapsible(true)
        .show(ctx, |ui| {
            ui.heading("Planet Gravity");
            ui.separator();
            ui.horizontal(|ui| {
                ui.label("Gravity Strength:");
                ui.add(
                    egui::DragValue::new(&mut planet_config.gravity_strength)
                        .speed(10.0)
                        .range(0.0..=2000.0),
                );
            });
            ui.label(format!(
                "Current: {:.1} px/s²",
                planet_config.gravity_strength
            ));
            ui.add_space(4.0);
            if ui.button("Reset Planet Gravity").clicked() {
                planet_config.gravity_strength = GRAVITY_STRENGTH;
            }
        });
}
