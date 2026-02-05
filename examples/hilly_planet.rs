//! Hilly Planet Example
//!
//! A playable example with a character walking on a spherical planet with hilly terrain:
//! - Uses 16 pixels per meter units
//! - Spherical planet with procedural hilly surface
//! - Realistic gravity: 9.81 * 16 = 156.96 px/s²
//! - Propulsion system with gravity compensation for upward thrust
//!
//! ## Controls
//! - **A/D** or **Left/Right**: Move horizontally
//! - **W/Up**: Jump (single impulse when grounded)
//! - **Space** (hold): Propulsion (thrust upward with gravity compensation)
//! - **S/Down** (hold): Propulsion (thrust downward)
//!
//! The propulsion system provides vertical thrust that is automatically
//! boosted by gravity magnitude to help counteract it when going up.
//!
//! ## Running
//! ```bash
//! # With Avian2D (default):
//! cargo run --example hilly_planet --features examples
//!
//! # With Rapier2D:
//! cargo run --example hilly_planet --features "examples,rapier2d" --no-default-features
//! ```

mod helpers;

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use helpers::{
    ActiveBackend, CharacterControllerUiPlugin, CharacterControllerUiState, ControlsPlugin,
    DefaultControllerSettings, ExamplePhysicsPlugin, Player, PlayerSpawnConfig, SpawnConfig,
    backend_name, spawn_player, spawn_polyline_collider, spawn_static_box_rotated, spawn_static_slope_rotated,
};
use msg_character_controller::prelude::*;
use std::f32::consts::{PI, TAU};

// ==================== Constants ====================

// Physics scale: 16 pixels = 1 meter
const PIXELS_PER_METER: f32 = 16.0;

// Gravity: 9.81 m/s² * 16 px/m = 156.96 px/s²
const GRAVITY_STRENGTH: f32 = 9.81 * PIXELS_PER_METER;

// Player dimensions (in pixels)
const PLAYER_HALF_HEIGHT: f32 = 8.0;
const PLAYER_RADIUS: f32 = 6.0;

// Planet configuration
const PLANET_CENTER: Vec2 = Vec2::ZERO;
const PLANET_BASE_RADIUS: f32 = 300.0;
const HILL_AMPLITUDE: f32 = 30.0; // Height variation for hills
const HILL_FREQUENCY: f32 = 12.0; // Number of major hills around the planet
const PLANET_SEGMENTS: u32 = 96; // Segments for planet surface

// ==================== Components ====================

/// Component storing the planet configuration.
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

/// Generate the radius at a given angle with hills.
fn planet_radius_at_angle(angle: f32) -> f32 {
    // Create multiple overlapping sine waves for interesting terrain
    let hill1 = (angle * HILL_FREQUENCY).sin() * HILL_AMPLITUDE;
    let hill2 = (angle * (HILL_FREQUENCY * 2.3)).sin() * (HILL_AMPLITUDE * 0.4);
    let hill3 = (angle * (HILL_FREQUENCY * 0.5)).sin() * (HILL_AMPLITUDE * 0.6);

    PLANET_BASE_RADIUS + hill1 + hill2 + hill3
}

fn spawn_position() -> Vec2 {
    // Spawn on top of the planet
    let spawn_angle = PI / 2.0;
    let direction = Vec2::new(spawn_angle.cos(), spawn_angle.sin());
    let surface_radius = planet_radius_at_angle(spawn_angle);
    PLANET_CENTER + direction * (surface_radius + 40.0)
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
                title: format!("Hilly Planet ({}) - 16px/m Physics", backend_name()),
                resolution: (1280, 720).into(),
                ..default()
            }),
            ..default()
        }))
        // Physics with 16 pixels per meter (backend-agnostic)
        .add_plugins(ExamplePhysicsPlugin::new(PIXELS_PER_METER))
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
    // Spawn hilly planet
    spawn_hilly_planet(&mut commands, &mut meshes, &mut materials);

    // Spawn player on top of planet
    let spawn_angle = PI / 2.0;
    let direction = Vec2::new(spawn_angle.cos(), spawn_angle.sin());
    let surface_radius = planet_radius_at_angle(spawn_angle);
    let spawn_pos = PLANET_CENTER + direction * (surface_radius + 40.0);
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
            font_size: 18.0,
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

    // Physics info
    commands.spawn((
        Text::new(format!(
            "16 px/meter | Gravity: {GRAVITY_STRENGTH:.2} px/s² (9.81 m/s²)"
        )),
        TextFont {
            font_size: 16.0,
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

    // Propulsion info
    commands.spawn((
        Text::new("Hold Space to propel upward (with gravity compensation)"),
        TextFont {
            font_size: 16.0,
            ..default()
        },
        TextColor(Color::srgb(0.6, 0.6, 0.8)),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(30.0),
            left: Val::Px(10.0),
            ..default()
        },
        Pickable::IGNORE,
    ));
}

fn spawn_hilly_planet(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
) {
    // Generate vertices for the hilly planet surface
    let mut vertices: Vec<Vec2> = Vec::with_capacity(PLANET_SEGMENTS as usize);
    let segments_f = PLANET_SEGMENTS as f32;

    for i in 0..PLANET_SEGMENTS {
        let i_f32 = i as f32;
        let angle = (i_f32 / segments_f) * TAU;
        let radius = planet_radius_at_angle(angle);
        let x = angle.cos() * radius;
        let y = angle.sin() * radius;
        vertices.push(Vec2::new(x, y));
    }

    // Use the backend-agnostic polyline collider spawner
    spawn_polyline_collider(
        commands,
        meshes,
        materials,
        PLANET_CENTER,
        vertices,
        Color::srgb(0.2, 0.35, 0.2),
    );

    // Add some distinct landmark features on the surface

    // A flat platform area (good landing spot)
    spawn_surface_platform(commands, meshes, materials, PI * 0.5, 60.0, 8.0); // Top

    // A ramp/slope structure
    spawn_surface_slope(commands, meshes, materials, PI * 0.25); // Upper right

    // Another platform on the side
    spawn_surface_platform(commands, meshes, materials, PI, 50.0, 6.0); // Left side

    // Small stepping stones
    spawn_surface_platform(commands, meshes, materials, PI * 1.5, 30.0, 5.0); // Bottom
    spawn_surface_platform(commands, meshes, materials, PI * 1.75, 25.0, 5.0); // Bottom right
}

/// Spawn a platform on the planet surface at a given angle.
fn spawn_surface_platform(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    angle: f32,
    width: f32,
    height: f32,
) {
    let direction = Vec2::new(angle.cos(), angle.sin());
    let surface_radius = planet_radius_at_angle(angle);
    let position = PLANET_CENTER + direction * (surface_radius + height / 2.0 + 5.0);

    // Rotation to align with surface (tangent)
    let rotation = Quat::from_rotation_z(angle - PI / 2.0);

    spawn_static_box_rotated(
        commands,
        meshes,
        materials,
        position,
        Vec2::new(width / 2.0, height / 2.0),
        rotation,
        Color::srgb(0.4, 0.5, 0.35),
    );
}

/// Spawn a triangular slope on the planet surface.
fn spawn_surface_slope(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    angle: f32,
) {
    let direction = Vec2::new(angle.cos(), angle.sin());
    let surface_radius = planet_radius_at_angle(angle);
    let position = PLANET_CENTER + direction * (surface_radius + 20.0);

    // Rotation to align with surface
    let rotation = Quat::from_rotation_z(angle - PI / 2.0);

    // Triangle vertices in local space
    let vertices = [
        Vec2::new(-30.0, 0.0), // Bottom left
        Vec2::new(30.0, 0.0),  // Bottom right
        Vec2::new(30.0, 40.0), // Top right
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
/// Gravity points toward the planet center and is stored in `CharacterController`.
/// The up direction is derived from gravity via `controller.ideal_up()`.
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
            let angle = new_up.to_angle() - PI / 2.0;
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
                        .speed(1.0)
                        .range(0.0..=500.0),
                );
            });
            ui.label(format!(
                "Current: {:.2} px/s² ({:.2} m/s²)",
                planet_config.gravity_strength,
                planet_config.gravity_strength / PIXELS_PER_METER
            ));
            ui.add_space(4.0);
            if ui.button("Reset Planet Gravity").clicked() {
                planet_config.gravity_strength = GRAVITY_STRENGTH;
            }
        });
}
