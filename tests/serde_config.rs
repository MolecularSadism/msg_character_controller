//! Serde coverage for the public config tree (`--features serde`):
//! a full round-trip and a partial deserialize backed by the `Default` impls.

#![cfg(feature = "serde")]

use msg_character_controller::prelude::{ControllerConfig, StairConfig};

/// A config survives serialize → deserialize with every field intact,
/// including nested sub-configs and `Option` fields.
#[test]
fn controller_config_round_trips_through_ron() {
    let mut original = ControllerConfig::default();
    original.floating.float_height = 17.5;
    original.spring.strength = 1234.0;
    original.spring.max_force = Some(3000.0);
    original.spring.max_velocity = None;
    original.walking.max_speed = 88.0;
    original.walking.wall_clinging = true;
    original.flying.max_speed = 210.0;
    original.flying.vertical_acceleration_ratio = 8.0;
    original.jumping.coyote_time = 0.21;
    original.wall_jumping.enabled = true;
    original.wall_jumping.angle = 0.9;
    original.upright.target_angle = Some(1.25);
    original.sensors.collision_mask = Some(0b1010);

    let text = ron::ser::to_string(&original).expect("ControllerConfig serializes");
    let parsed: ControllerConfig = ron::from_str(&text).expect("serialized config parses");

    assert_eq!(parsed.floating.float_height, 17.5);
    assert_eq!(parsed.spring.strength, 1234.0);
    assert_eq!(parsed.spring.max_force, Some(3000.0));
    assert_eq!(parsed.spring.max_velocity, None);
    assert_eq!(parsed.walking.max_speed, 88.0);
    assert!(parsed.walking.wall_clinging);
    assert_eq!(parsed.flying.max_speed, 210.0);
    assert_eq!(parsed.flying.vertical_acceleration_ratio, 8.0);
    assert_eq!(parsed.jumping.coyote_time, 0.21);
    assert!(parsed.wall_jumping.enabled);
    assert_eq!(parsed.wall_jumping.angle, 0.9);
    assert_eq!(parsed.upright.target_angle, Some(1.25));
    assert_eq!(parsed.sensors.collision_mask, Some(0b1010));
}

/// A partial file sets only what it names; struct-level `#[serde(default)]`
/// fills the rest from the `Default` impls — at every nesting level.
#[test]
fn partial_config_falls_back_to_defaults() {
    let parsed: ControllerConfig = ron::from_str(
        "(walking: (max_speed: 42.0), spring: (max_force: Some(3000.0)), flying: (damping: 0.7))",
    )
    .expect("partial config parses");
    let default = ControllerConfig::default();

    // Named fields take the file's values.
    assert_eq!(parsed.walking.max_speed, 42.0);
    assert_eq!(parsed.spring.max_force, Some(3000.0));
    assert_eq!(parsed.flying.damping, 0.7);

    // Everything else falls back to the Default impls.
    assert_eq!(parsed.walking.acceleration, default.walking.acceleration);
    assert_eq!(parsed.spring.strength, default.spring.strength);
    assert_eq!(parsed.flying.max_speed, default.flying.max_speed);
    assert_eq!(parsed.floating.float_height, default.floating.float_height);
    assert_eq!(parsed.jumping.speed, default.jumping.speed);
    assert_eq!(parsed.wall_jumping.enabled, default.wall_jumping.enabled);
    assert_eq!(parsed.upright.strength, default.upright.strength);
    assert_eq!(
        parsed.sensors.ground_cast_width,
        default.sensors.ground_cast_width
    );
}

/// The empty document is exactly the `Default` configuration.
#[test]
fn empty_config_is_the_default() {
    let parsed: ControllerConfig = ron::from_str("()").expect("empty config parses");
    let default = ControllerConfig::default();
    assert_eq!(parsed.walking.max_speed, default.walking.max_speed);
    assert_eq!(parsed.spring.strength, default.spring.strength);
    assert_eq!(parsed.flying.acceleration, default.flying.acceleration);
}

/// `StairConfig` (carried on `CharacterController`, not `ControllerConfig`)
/// round-trips and partially deserializes too.
#[test]
fn stair_config_round_trips_and_defaults() {
    let original = StairConfig {
        max_climb_height: 15.0,
        enabled: false,
        ..StairConfig::default()
    };
    let text = ron::ser::to_string(&original).expect("StairConfig serializes");
    let parsed: StairConfig = ron::from_str(&text).expect("serialized StairConfig parses");
    assert_eq!(parsed.max_climb_height, 15.0);
    assert!(!parsed.enabled);

    let partial: StairConfig =
        ron::from_str("(max_climb_height: 9.0)").expect("partial StairConfig parses");
    assert_eq!(partial.max_climb_height, 9.0);
    assert_eq!(
        partial.min_step_depth,
        StairConfig::default().min_step_depth
    );
    assert_eq!(partial.enabled, StairConfig::default().enabled);
}
