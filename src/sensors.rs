//! Backend-neutral ground, wall, ceiling and stair detection.
//!
//! The controller senses its surroundings with six shape casts per character,
//! one per [`SensorKind`]. Everything about *what* to cast and *what a hit
//! means* lives here and is the same for every physics backend; a backend only
//! answers the casts (see [`CharacterPhysicsBackend::cast_shape`] and
//! [`CharacterPhysicsBackend::sensor_cast_mode`]).
//!
//! # Data flow per fixed step
//!
//! 1. [`SensorSystems::Request`] (in [`Preparation`]): the controller writes
//!    each character's [`SensorCast`]s into its [`CharacterSensors`] and
//!    clears last step's [`SensorReading`]s.
//! 2. [`SensorSystems::Submit`] (in [`Preparation`]): a backend with
//!    [`SensorCastMode::Persistent`] hands the requests to its engine here
//!    (Avian: copies them onto `ShapeCaster` child entities).
//! 3. [`SensorSystems::Cast`] (in [`Sensors`]): the backend fills in
//!    [`SensorReading`]s. For [`SensorCastMode::Immediate`] the controller does
//!    this itself, calling [`CharacterPhysicsBackend::cast_shape`] once per
//!    enabled cast at the character's current position and rotation.
//! 4. [`SensorSystems::Detect`] (in [`Sensors`]): the controller turns the
//!    readings into `floor`, walls, ceiling and step state on the
//!    [`CharacterController`].
//!
//! # Timing
//!
//! An immediate backend casts against the world as it stands at the start of
//! the `Sensors` phase, with this step's requests. Avian's persistent casters
//! run inside its physics step, so the hits read in a step come from the
//! previous physics step, cast with the previous step's requests.
//!
//! [`Preparation`]: crate::CharacterControllerPhaseSystems::Preparation
//! [`Sensors`]: crate::CharacterControllerPhaseSystems::Sensors

use bevy::prelude::*;

use crate::CasterDisabled;
use crate::backend::CharacterPhysicsBackend;
use crate::collision::CollisionData;
use crate::config::{CharacterController, ControllerConfig};
use crate::intent::MovementIntent;

/// Number of sensor casts per character.
pub const SENSOR_COUNT: usize = 6;

/// One of the controller's detection casts.
#[derive(Reflect, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorKind {
    /// Downward cast that finds the floor the character floats over.
    Ground,
    /// Cast toward the character's local left.
    LeftWall,
    /// Cast toward the character's local right.
    RightWall,
    /// Upward cast that finds a ceiling.
    Ceiling,
    /// Forward-down cast ahead of a walking character that finds a step surface.
    Stair,
    /// Downward cast under a walking character, the reference the step height
    /// is measured against.
    StairGround,
}

impl SensorKind {
    /// All kinds, in storage order.
    pub const ALL: [Self; SENSOR_COUNT] = [
        Self::Ground,
        Self::LeftWall,
        Self::RightWall,
        Self::Ceiling,
        Self::Stair,
        Self::StairGround,
    ];

    /// Storage index of this kind inside [`CharacterSensors`].
    #[must_use]
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// Convex shape swept by a cast.
///
/// Points are offsets from the cast origin. In a [`SensorCast`] they are in the
/// character's local frame; in a [`ShapeCastQuery`] they are already rotated
/// into world orientation.
#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum CastShape {
    /// A line segment from `a` to `b` (zero radius).
    Segment {
        /// First endpoint.
        a: Vec2,
        /// Second endpoint.
        b: Vec2,
    },
}

impl CastShape {
    /// A segment of length `width` centred on the origin, along local X.
    #[must_use]
    pub fn horizontal_segment(width: f32) -> Self {
        let half = width / 2.0;
        Self::Segment {
            a: Vec2::new(-half, 0.0),
            b: Vec2::new(half, 0.0),
        }
    }

    /// A segment of length `height` centred on the origin, along local Y.
    #[must_use]
    pub fn vertical_segment(height: f32) -> Self {
        let half = height / 2.0;
        Self::Segment {
            a: Vec2::new(0.0, -half),
            b: Vec2::new(0.0, half),
        }
    }

    /// This shape rotated by `rotation` about the cast origin.
    #[must_use]
    pub fn rotated(self, rotation: Rot2) -> Self {
        match self {
            Self::Segment { a, b } => Self::Segment {
                a: rotation * a,
                b: rotation * b,
            },
        }
    }
}

/// Which colliders a cast may hit.
#[derive(Reflect, Debug, Clone, Copy, PartialEq, Eq)]
pub struct CastFilter {
    /// Only colliders whose collision-group memberships share a bit with this
    /// mask are hit.
    pub mask: u32,
    /// The character the cast belongs to. Colliders of this entity must never
    /// be reported. A backend whose characters own further collider entities
    /// (compound children, the backend's own helper shapes) should skip those
    /// too.
    pub exclude: Entity,
}

/// One detection cast as the controller requests it, in the character's
/// local frame.
///
/// The cast starts at `origin` (local offset from the character's position),
/// sweeps `shape` along `direction` for up to `max_distance`. Origin,
/// direction and shape all turn with the character's rotation, the way a
/// child caster entity would.
#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct SensorCast {
    /// Whether the cast runs this step. A disabled cast reports no hit.
    pub enabled: bool,
    /// Swept shape, local orientation.
    pub shape: CastShape,
    /// Start offset from the character's position, local frame.
    pub origin: Vec2,
    /// Sweep direction, local frame.
    pub direction: Dir2,
    /// Sweep length.
    pub max_distance: f32,
    /// Hit filter.
    pub filter: CastFilter,
}

impl SensorCast {
    fn new(direction: Dir2, enabled: bool) -> Self {
        Self {
            enabled,
            shape: CastShape::horizontal_segment(0.0),
            origin: Vec2::ZERO,
            direction,
            max_distance: 0.0,
            filter: CastFilter {
                mask: u32::MAX,
                exclude: Entity::PLACEHOLDER,
            },
        }
    }
}

/// The nearest hit of a cast.
#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct SensorHit {
    /// Entity of the collider that was hit.
    pub entity: Entity,
    /// Distance the shape travelled along the cast direction before touching
    /// (`Box2D`: `fraction * max_distance`). Zero when the cast starts
    /// overlapping.
    pub distance: f32,
    /// Outward surface normal of the hit collider at the contact, world space.
    pub normal: Vec2,
}

/// What a backend reports for one cast in one step.
#[derive(Reflect, Debug, Clone, Copy, PartialEq)]
pub struct SensorReading {
    /// World-space point the detection logic measures hit points from.
    pub origin: Vec2,
    /// Nearest hit, if any.
    pub hit: Option<SensorHit>,
}

/// A cast resolved into world space, as handed to
/// [`CharacterPhysicsBackend::cast_shape`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeCastQuery {
    /// Swept shape; points are offsets from `origin` in world orientation.
    pub shape: CastShape,
    /// World-space start of the sweep.
    pub origin: Vec2,
    /// World-space sweep direction.
    pub direction: Dir2,
    /// Sweep length.
    pub max_distance: f32,
    /// Hit filter.
    pub filter: CastFilter,
}

/// How a backend answers the controller's sensor casts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SensorCastMode {
    /// The controller calls [`CharacterPhysicsBackend::cast_shape`] for every
    /// enabled cast in [`SensorSystems::Cast`], against the world as it is at
    /// that moment.
    #[default]
    Immediate,
    /// The backend keeps its own persistent casters: it reads the requests in
    /// [`SensorSystems::Submit`], runs them during its physics step, and writes
    /// [`SensorReading`]s in [`SensorSystems::Cast`] through
    /// [`CharacterSensors::set_reading`]. `cast_shape` is not called.
    Persistent,
}

/// Sensor requests and readings of one character.
///
/// Required by [`CharacterController`], so every character has one.
#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
pub struct CharacterSensors {
    casts: [SensorCast; SENSOR_COUNT],
    readings: [Option<SensorReading>; SENSOR_COUNT],
    shapes_initialized: bool,
}

impl Default for CharacterSensors {
    fn default() -> Self {
        let mut sensors = Self {
            casts: [
                SensorCast::new(Dir2::NEG_Y, true),
                SensorCast::new(Dir2::NEG_X, true),
                SensorCast::new(Dir2::X, true),
                SensorCast::new(Dir2::Y, true),
                SensorCast::new(Dir2::NEG_Y, false),
                SensorCast::new(Dir2::NEG_Y, false),
            ],
            readings: [None; SENSOR_COUNT],
            shapes_initialized: false,
        };
        sensors.set_shapes(
            &crate::config::SensorConfig::default(),
            crate::config::StairConfig::default().stair_cast_width,
        );
        sensors
    }
}

impl CharacterSensors {
    /// The cast requested for `kind` this step.
    #[must_use]
    pub fn cast(&self, kind: SensorKind) -> &SensorCast {
        &self.casts[kind.index()]
    }

    /// All requested casts, indexed by [`SensorKind::index`].
    #[must_use]
    pub fn casts(&self) -> &[SensorCast; SENSOR_COUNT] {
        &self.casts
    }

    /// The reading the backend reported for `kind` this step.
    ///
    /// `None` means the backend has no sensor of this kind for the character
    /// (Avian: no caster child), which is not the same as a cast that hit
    /// nothing.
    #[must_use]
    pub fn reading(&self, kind: SensorKind) -> Option<&SensorReading> {
        self.readings[kind.index()].as_ref()
    }

    /// Report the result of the cast for `kind` this step. Called by backends.
    ///
    /// If the backend reports several results for one kind in a step (several
    /// casters of that kind), the last one with a hit wins.
    pub fn set_reading(&mut self, kind: SensorKind, reading: SensorReading) {
        let slot = &mut self.readings[kind.index()];
        if reading.hit.is_some() || slot.is_none_or(|current| current.hit.is_none()) {
            *slot = Some(reading);
        }
    }

    /// Set the swept shapes from the sensor and stair cast dimensions.
    fn set_shapes(&mut self, sensors: &crate::config::SensorConfig, stair_width: f32) {
        self.casts[SensorKind::Ground.index()].shape =
            CastShape::horizontal_segment(sensors.ground_cast_width);
        self.casts[SensorKind::LeftWall.index()].shape =
            CastShape::vertical_segment(sensors.wall_cast_height);
        self.casts[SensorKind::RightWall.index()].shape =
            CastShape::vertical_segment(sensors.wall_cast_height);
        self.casts[SensorKind::Ceiling.index()].shape =
            CastShape::horizontal_segment(sensors.ceiling_cast_width);
        self.casts[SensorKind::Stair.index()].shape = CastShape::horizontal_segment(stair_width);
        self.casts[SensorKind::StairGround.index()].shape =
            CastShape::horizontal_segment(stair_width);
    }
}

/// Ordered steps of the sensor pipeline. See the [module docs](self).
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorSystems {
    /// Controller writes cast requests. In `Preparation`.
    Request,
    /// Persistent backends hand requests to their engine. In `Preparation`,
    /// after [`Self::Request`].
    Submit,
    /// Backend fills readings. In `Sensors`, first.
    Cast,
    /// Controller turns readings into contact state. In `Sensors`, after
    /// [`Self::Cast`].
    Detect,
}

/// Register [`CharacterSensors`], order the [`SensorSystems`] inside the
/// controller phases and add the backend-neutral sensor systems for backend `B`.
pub(crate) fn register_systems<B: CharacterPhysicsBackend>(app: &mut App) {
    use crate::CharacterControllerPhaseSystems as Phase;

    app.register_type::<CharacterSensors>();
    app.configure_sets(
        FixedUpdate,
        (SensorSystems::Request, SensorSystems::Submit)
            .chain()
            .in_set(Phase::Preparation),
    );
    app.configure_sets(
        FixedUpdate,
        (SensorSystems::Cast, SensorSystems::Detect)
            .chain()
            .in_set(Phase::Sensors),
    );
    app.add_systems(
        FixedUpdate,
        update_sensor_casts::<B>.in_set(SensorSystems::Request),
    );
    if B::sensor_cast_mode() == SensorCastMode::Immediate {
        app.add_systems(
            FixedUpdate,
            run_immediate_sensor_casts::<B>.in_set(SensorSystems::Cast),
        );
    }
    app.add_systems(
        FixedUpdate,
        detect_surroundings::<B>.in_set(SensorSystems::Detect),
    );
}

/// Collision mask for a character's casts: the config override if set,
/// otherwise the actor's filter bits, otherwise every bit.
#[must_use]
pub fn cast_mask(config: &ControllerConfig, collision_groups: Option<(u32, u32)>) -> u32 {
    config
        .sensors
        .collision_mask
        .unwrap_or_else(|| collision_groups.map_or(u32::MAX, |(_, filters)| filters))
}

/// Per-character backend data gathered before the cast requests are written.
pub(crate) struct RequestInputs {
    entity: Entity,
    held: bool,
    mask: u32,
    radius: f32,
}

/// Write every character's cast requests for this step.
///
/// The casts are held off (disabled) while the character carries
/// [`CasterDisabled`] or its body is disabled
/// ([`CharacterPhysicsBackend::is_body_disabled`]). Stair casts run only while
/// the character walks with stair stepping enabled. The swept shapes are taken
/// from the config the first time a character is seen and kept after that, as
/// Avian's caster colliders always were.
///
/// Also clears last step's readings, so a backend that reports nothing leaves
/// none behind.
#[allow(clippy::type_complexity)]
pub(crate) fn update_sensor_casts<B: CharacterPhysicsBackend>(
    world: &mut World,
    characters: &mut QueryState<
        (Entity, &ControllerConfig, Has<CasterDisabled>),
        With<CharacterController>,
    >,
    targets: &mut QueryState<(
        &CharacterController,
        &ControllerConfig,
        Option<&MovementIntent>,
        &mut CharacterSensors,
    )>,
    mut inputs: Local<Vec<RequestInputs>>,
) {
    inputs.clear();
    {
        let world_ref: &World = world;
        for (entity, config, caster_disabled) in characters.iter(world_ref) {
            inputs.push(RequestInputs {
                entity,
                held: caster_disabled || B::is_body_disabled(world_ref, entity),
                mask: cast_mask(config, B::get_collision_groups(world_ref, entity)),
                radius: B::get_collider_radius(world_ref, entity),
            });
        }
    }

    for input in &*inputs {
        let Ok((controller, config, intent, mut sensors)) = targets.get_mut(world, input.entity)
        else {
            continue;
        };
        if !sensors.shapes_initialized {
            sensors.set_shapes(
                &config.sensors,
                controller.stair_config.unwrap_or_default().stair_cast_width,
            );
            sensors.shapes_initialized = true;
        }
        sensors.readings = [None; SENSOR_COUNT];
        write_casts(&mut sensors, controller, config, intent, input);
    }
}

fn write_casts(
    sensors: &mut CharacterSensors,
    controller: &CharacterController,
    config: &ControllerConfig,
    intent: Option<&MovementIntent>,
    input: &RequestInputs,
) {
    let filter = CastFilter {
        mask: input.mask,
        exclude: input.entity,
    };
    let held = input.held;

    // Ground: reach riding height plus the grounding margin.
    let ground = &mut sensors.casts[SensorKind::Ground.index()];
    ground.enabled = !held;
    if !held {
        ground.direction = Dir2::NEG_Y;
        ground.max_distance =
            controller.riding_height(config) + config.floating.grounding_distance + 1.0;
        ground.filter = filter;
    }

    // Walls: surface detection distance beyond the collider's side.
    for (kind, direction) in [
        (SensorKind::LeftWall, Dir2::NEG_X),
        (SensorKind::RightWall, Dir2::X),
    ] {
        let wall = &mut sensors.casts[kind.index()];
        wall.enabled = !held;
        if !held {
            wall.direction = direction;
            wall.max_distance = config.floating.surface_detection_distance + input.radius + 1.0;
            wall.filter = filter;
        }
    }

    // Ceiling: surface detection distance beyond the collider's top.
    let ceiling = &mut sensors.casts[SensorKind::Ceiling.index()];
    ceiling.enabled = !held;
    if !held {
        ceiling.direction = Dir2::Y;
        ceiling.max_distance =
            config.floating.surface_detection_distance + controller.capsule_half_height() + 1.0;
        ceiling.filter = filter;
    }

    // Stair casts: only while walking with stair stepping on.
    let stair_config = controller.stair_config.filter(|stair| stair.enabled);
    let walking = intent.filter(|intent| intent.is_walking());

    let stair = &mut sensors.casts[SensorKind::Stair.index()];
    match (held, stair_config, intent, walking) {
        (true, ..) | (false, None, ..) | (false, Some(_), None, _) => stair.enabled = false,
        (false, Some(stair_config), Some(intent), Some(_)) => {
            stair.enabled = true;
            let horizontal_offset = input.radius + stair_config.stair_cast_offset;
            let vertical_offset = controller.ideal_up()
                * (stair_config.max_climb_height - controller.collider_bottom_offset);
            // Only move to a side when the walk direction is unambiguous; the
            // origin keeps its last value otherwise, which damps near-zero
            // flicker between sides.
            if intent.walk.abs() > f32::EPSILON {
                let move_dir = controller.ideal_right() * intent.walk.signum();
                stair.origin = move_dir * horizontal_offset + vertical_offset;
            }
            stair.direction = Dir2::NEG_Y;
            // float_height extends reach to ground level; min_step_depth
            // shortens it so steps shallower than that are never found.
            stair.max_distance = stair_config.max_climb_height + config.floating.float_height
                - stair_config.min_step_depth;
            stair.filter = filter;
        }
        (false, Some(_), Some(_), None) => {
            stair.enabled = false;
            stair.origin = Vec2::ZERO;
        }
    }

    let stair_ground = &mut sensors.casts[SensorKind::StairGround.index()];
    match (held, stair_config, walking) {
        (false, Some(stair_config), Some(_)) => {
            stair_ground.enabled = true;
            stair_ground.origin = Vec2::ZERO;
            stair_ground.direction = Dir2::NEG_Y;
            stair_ground.max_distance =
                stair_config.max_climb_height + controller.collider_bottom_offset + 2.0;
            stair_ground.filter = filter;
        }
        _ => stair_ground.enabled = false,
    }
}

/// One character's readings from an immediate cast pass.
pub(crate) struct CastBatch {
    entity: Entity,
    readings: [Option<SensorReading>; SENSOR_COUNT],
}

/// Answer every character's casts through [`CharacterPhysicsBackend::cast_shape`].
///
/// Registered for backends with [`SensorCastMode::Immediate`]. Each cast is
/// placed at the character's current position and rotation: origin
/// `position + rotation * cast.origin`, direction and shape rotated likewise.
/// Every kind gets a reading; a disabled cast reads no hit.
pub(crate) fn run_immediate_sensor_casts<B: CharacterPhysicsBackend>(
    world: &mut World,
    characters: &mut QueryState<(Entity, &CharacterSensors)>,
    targets: &mut QueryState<&mut CharacterSensors>,
    mut batch: Local<Vec<CastBatch>>,
) {
    batch.clear();
    {
        let world_ref: &World = world;
        for (entity, sensors) in characters.iter(world_ref) {
            let position = B::get_position(world_ref, entity);
            let rotation = Rot2::radians(B::get_rotation(world_ref, entity));
            let mut readings = [None; SENSOR_COUNT];
            for (slot, cast) in readings.iter_mut().zip(sensors.casts.iter()) {
                let origin = position + rotation * cast.origin;
                let hit = if cast.enabled {
                    B::cast_shape(
                        world_ref,
                        &ShapeCastQuery {
                            shape: cast.shape.rotated(rotation),
                            origin,
                            direction: rotation * cast.direction,
                            max_distance: cast.max_distance,
                            filter: cast.filter,
                        },
                    )
                } else {
                    None
                };
                *slot = Some(SensorReading { origin, hit });
            }
            batch.push(CastBatch { entity, readings });
        }
    }

    for entry in &*batch {
        if let Ok(mut sensors) = targets.get_mut(world, entry.entity) {
            sensors.readings = entry.readings;
        }
    }
}

/// Turn this step's readings into contact state on each character.
///
/// Skips characters whose body is disabled. For the rest: resets the
/// detection state, then sets `floor` and slope (and refreshes the collider
/// bottom offset), the walls, the ceiling, and the step state. Hit points are
/// placed along the gravity-relative directions from each reading's origin.
pub(crate) fn detect_surroundings<B: CharacterPhysicsBackend>(
    world: &mut World,
    characters: &mut QueryState<Entity, (With<CharacterController>, With<CharacterSensors>)>,
    targets: &mut QueryState<(&mut CharacterController, &CharacterSensors)>,
    mut active: Local<Vec<(Entity, f32)>>,
) {
    active.clear();
    {
        let world_ref: &World = world;
        for entity in characters.iter(world_ref) {
            if !B::is_body_disabled(world_ref, entity) {
                active.push((entity, B::get_collider_bottom_offset(world_ref, entity)));
            }
        }
    }

    for &(entity, bottom_offset) in &*active {
        let Ok((mut controller, sensors)) = targets.get_mut(world, entity) else {
            continue;
        };
        apply_readings(&mut controller, sensors, bottom_offset);
    }
}

/// Apply one character's readings to its controller (see `detect_surroundings`).
pub(crate) fn apply_readings(
    controller: &mut CharacterController,
    sensors: &CharacterSensors,
    collider_bottom_offset: f32,
) {
    controller.reset_detection_state();

    let up = controller.ideal_up();
    let down = controller.ideal_down();

    if let Some(reading) = sensors.reading(SensorKind::Ground) {
        controller.collider_bottom_offset = collider_bottom_offset;
        if let Some(hit) = reading.hit {
            let dot = hit.normal.dot(up).clamp(-1.0, 1.0);
            controller.floor = Some(CollisionData::new(
                hit.distance,
                hit.normal,
                reading.origin + down * hit.distance,
                Some(hit.entity),
            ));
            controller.slope_angle = dot.acos();
        }
    }

    if let Some(hit) = hit_of(sensors, SensorKind::LeftWall) {
        controller.left_wall = Some(contact(hit, controller.ideal_left()));
    }
    if let Some(hit) = hit_of(sensors, SensorKind::RightWall) {
        controller.right_wall = Some(contact(hit, controller.ideal_right()));
    }
    if let Some(hit) = hit_of(sensors, SensorKind::Ceiling) {
        controller.ceiling = Some(contact(hit, up));
    }

    let Some(stair_config) = controller.stair_config.filter(|stair| stair.enabled) else {
        return;
    };
    // Stair readings count only while the stair casts are enabled this step.
    let enabled_hit = |kind| {
        if sensors.cast(kind).enabled {
            hit_of(sensors, kind)
        } else {
            None
        }
    };
    if let (Some((stair_origin, stair)), Some((ground_origin, ground))) = (
        enabled_hit(SensorKind::Stair),
        enabled_hit(SensorKind::StairGround),
    ) {
        let step_surface_point = stair_origin + down * stair.distance;
        let current_ground_point = ground_origin + down * ground.distance;
        let step_height = (step_surface_point - current_ground_point).dot(up);
        // Climbable and mostly horizontal; the minimum height is enforced by
        // the cast length (min_step_depth).
        if step_height <= stair_config.max_climb_height && stair.normal.dot(up) > 0.7 {
            controller.step_detected = true;
            controller.step_height = step_height;
        }
    }
}

fn hit_of(sensors: &CharacterSensors, kind: SensorKind) -> Option<(Vec2, SensorHit)> {
    sensors
        .reading(kind)
        .and_then(|reading| reading.hit.map(|hit| (reading.origin, hit)))
}

fn contact((origin, hit): (Vec2, SensorHit), direction: Vec2) -> CollisionData {
    CollisionData::new(
        hit.distance,
        hit.normal,
        origin + direction * hit.distance,
        Some(hit.entity),
    )
}
