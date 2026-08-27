//! Micro-benchmarks for the flight-frame math: the confidence-blended
//! [`FlyingConfig::thrust`] law and the [`FlightFrame`] basis round trip.
//!
//! Inputs are fixed constants so runs are deterministic and comparable.

use bevy::math::{Dir2, Vec2};
use criterion::{Criterion, black_box, criterion_group, criterion_main};
use msg_character_controller::config::FlyingConfig;
use msg_character_controller::flight::FlightFrame;

fn thrust(c: &mut Criterion) {
    let config = FlyingConfig {
        max_speed: 150.0,
        vertical_speed_ratio: 0.6,
        acceleration: 500.0,
        vertical_acceleration_ratio: 0.6,
        gravity_compensation: 0.05,
        damping: 0.0,
    };
    let frame = FlightFrame::new(Dir2::from_xy(0.6, 0.8).unwrap());
    let intent = Vec2::new(0.4, 0.9);
    let velocity = Vec2::new(35.0, -12.0);

    c.bench_function("flying_config_thrust", |b| {
        b.iter(|| {
            black_box(config.thrust(
                black_box(intent),
                black_box(frame),
                black_box(velocity),
                black_box(0.75),
                black_box(980.0),
            ))
        })
    });
}

fn frame_round_trip(c: &mut Criterion) {
    let frame = FlightFrame::new(Dir2::from_xy(0.6, 0.8).unwrap());
    let world = Vec2::new(3.0, -4.0);

    c.bench_function("flight_frame_project_to_world_round_trip", |b| {
        b.iter(|| black_box(frame.to_world(frame.project(black_box(world)))))
    });
}

criterion_group!(benches, thrust, frame_round_trip);
criterion_main!(benches);
