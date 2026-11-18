//! `Avian2D` physics backend implementation.
//!
//! This module provides the physics backend for `Avian2D` (`bevy_avian2d`).
//! Enable with the `avian2d` feature.

use avian2d::prelude::*;
use bevy::ecs::system::SystemState;
use bevy::prelude::*;

use crate::backend::CharacterPhysicsBackend;
use crate::config::{CharacterController, ControllerConfig};
use crate::intent::MovementIntent;
use crate::sensors::{
    CastFilter, CastShape, CharacterSensors, SensorCastMode, SensorHit, SensorKind, SensorReading,
    SensorSystems, ShapeCastQuery,
};

/// Marker component for ground detection `ShapeCaster`.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct GroundCaster;

/// Marker component for left wall detection `ShapeCaster`.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct LeftWallCaster;

/// Marker component for right wall detection `ShapeCaster`.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct RightWallCaster;

/// Marker component for ceiling detection `ShapeCaster`.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct CeilingCaster;

/// Marker component for stair detection `ShapeCaster` (forward-down cast).
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct StairCaster;

/// Marker component for current ground detection `ShapeCaster` (for step height calculation).
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct CurrentGroundCaster;

/// Relationship: Caster belongs to a character.
///
/// This goes on caster entities and points to the character they belong to.
/// Used to look up the character's controller and config.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
#[relationship(relationship_target = CharacterCasters)]
pub struct CasterOfCharacter(pub Entity);

/// Relationship target: Character has multiple casters.
///
/// This goes on the character entity and tracks all its casters.
/// Bevy manages the `Vec<Entity>` automatically.
#[derive(Component, Reflect, Debug, Clone)]
#[reflect(Component)]
#[relationship_target(relationship = CasterOfCharacter)]
pub struct CharacterCasters(Vec<Entity>); // Private field - managed by Bevy

/// Marker component to track that casters have been spawned for this character.
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct CastersSpawned;

/// Message fired when a reactive force (Newton's 3rd law) is applied to an entity.
///
/// This message is used to track forces applied to ground entities so they can be
/// properly cleared in the next frame. Without this, forces would accumulate
/// indefinitely on entities that characters stand on.
#[derive(Message, Reflect, Debug, Clone)]
#[reflect(Message)]
pub struct ReactiveForceApplied {
    /// The entity that received the reactive force.
    pub entity: Entity,
    /// The force that was applied.
    pub force: Vec2,
}

/// `Avian2D` physics backend for the character controller.
///
/// This backend uses `avian2d` for physics operations including
/// force application and velocity manipulation. It answers the controller's
/// sensor casts with persistent `ShapeCaster` child entities
/// ([`SensorCastMode::Persistent`]), which Avian runs in parallel during its
/// physics step; `cast_shape` performs the same cast one-shot through
/// `SpatialQuery`.
pub struct Avian2dBackend;

impl CharacterPhysicsBackend for Avian2dBackend {
    type VelocityComponent = LinearVelocity;

    fn plugin() -> impl Plugin {
        Avian2dBackendPlugin
    }

    fn get_velocity(world: &World, entity: Entity) -> Vec2 {
        world
            .get::<LinearVelocity>(entity)
            .map_or(Vec2::ZERO, |v| v.0)
    }

    fn set_velocity(world: &mut World, entity: Entity, velocity: Vec2) {
        if let Some(mut vel) = world.get_mut::<LinearVelocity>(entity) {
            vel.0 = velocity;
        }
    }

    fn apply_impulse(world: &mut World, entity: Entity, impulse: Vec2) {
        // In Avian 0.4, impulses are applied directly to velocity
        // Impulse = mass * delta_v, so delta_v = impulse / mass
        let mass = Self::get_mass(world, entity);
        if mass <= 0.0 {
            return;
        }
        let delta_v = impulse / mass;
        if let Some(mut vel) = world.get_mut::<LinearVelocity>(entity) {
            vel.0 += delta_v;
        }
    }

    fn set_linear_damping(world: &mut World, entity: Entity, damping: f32) {
        let Ok(mut entity_mut) = world.get_entity_mut(entity) else {
            return;
        };
        if let Some(mut current) = entity_mut.get_mut::<LinearDamping>() {
            if (current.0 - damping).abs() > f32::EPSILON {
                current.0 = damping;
            }
        } else {
            entity_mut.insert(LinearDamping(damping));
        }
    }

    fn apply_force(world: &mut World, entity: Entity, force: Vec2) {
        // Accumulate into CharacterController instead of directly modifying forces.
        // Forces will be applied at the end of the frame by finalize_controller_forces.
        if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
            controller.add_force(force);
        }
    }

    fn apply_torque(world: &mut World, entity: Entity, torque: f32) {
        // Accumulate into CharacterController instead of directly modifying torque.
        // Torque will be applied at the end of the frame by finalize_controller_forces.
        if let Some(mut controller) = world.get_mut::<CharacterController>(entity) {
            controller.add_torque(torque);
        }
    }

    fn get_angular_velocity(world: &World, entity: Entity) -> f32 {
        world.get::<AngularVelocity>(entity).map_or(0.0, |v| v.0)
    }

    fn get_rotation(world: &World, entity: Entity) -> f32 {
        // Try Avian's Rotation component first, then fall back to Transform
        world
            .get::<Rotation>(entity)
            .map(|r| r.as_radians())
            .or_else(|| {
                world.get::<Transform>(entity).map(|t| {
                    let (_, _, z) = t.rotation.to_euler(EulerRot::XYZ);
                    z
                })
            })
            .or_else(|| {
                world.get::<GlobalTransform>(entity).map(|t| {
                    let (_, rotation, _) = t.to_scale_rotation_translation();
                    let (_, _, z) = rotation.to_euler(EulerRot::XYZ);
                    z
                })
            })
            .unwrap_or(0.0)
    }

    fn get_gravity(_world: &World, _entity: Entity) -> Vec2 {
        // Gravity is stored in CharacterController.gravity
        Vec2::ZERO
    }

    fn get_position(world: &World, entity: Entity) -> Vec2 {
        // Try Avian's Position component first, then fall back to Transform
        world
            .get::<Position>(entity)
            .map(|p| p.0)
            .or_else(|| world.get::<Transform>(entity).map(|t| t.translation.xy()))
            .or_else(|| {
                world
                    .get::<GlobalTransform>(entity)
                    .map(|t| t.translation().xy())
            })
            .unwrap_or(Vec2::ZERO)
    }

    fn get_fixed_timestep(world: &World) -> f32 {
        world
            .get_resource::<Time<Fixed>>()
            .map(bevy::prelude::Time::delta_secs)
            .filter(|&d| d > 0.0)
            .unwrap_or(1.0 / 60.0)
    }

    fn get_collision_groups(world: &World, entity: Entity) -> Option<(u32, u32)> {
        world
            .get::<CollisionLayers>(entity)
            .map(|cl| (cl.memberships.0, cl.filters.0))
    }

    fn get_collider_bottom_offset(world: &World, entity: Entity) -> f32 {
        world
            .get::<Collider>(entity)
            .map_or(0.0, get_collider_bottom_offset)
    }

    fn get_mass(world: &World, entity: Entity) -> f32 {
        let Some(computed_mass) = world.get::<ComputedMass>(entity) else {
            return 0.0;
        };
        let mass = computed_mass.value();
        if mass <= 0.0 || !mass.is_finite() {
            return 0.0;
        }
        mass
    }

    fn get_principal_inertia(world: &World, entity: Entity) -> f32 {
        let Some(computed_inertia) = world.get::<ComputedAngularInertia>(entity) else {
            return 0.0;
        };
        let inertia = computed_inertia.value();
        if inertia <= 0.0 || !inertia.is_finite() {
            return 0.0;
        }
        inertia
    }

    fn is_rotation_locked(world: &World, entity: Entity) -> bool {
        world
            .get::<LockedAxes>(entity)
            .is_some_and(avian2d::prelude::LockedAxes::is_rotation_locked)
    }

    fn provides_custom_gravity() -> bool {
        // Avian backend provides custom gravity systems that use Forces component
        true
    }

    fn get_collider_radius(world: &World, entity: Entity) -> f32 {
        world
            .get::<Collider>(entity)
            .map_or(0.0, get_collider_radius)
    }

    fn is_body_disabled(world: &World, entity: Entity) -> bool {
        world.get::<RigidBodyDisabled>(entity).is_some()
    }

    fn sensor_cast_mode() -> SensorCastMode {
        // Persistent `ShapeCaster` children, run by Avian in parallel during
        // its physics step.
        SensorCastMode::Persistent
    }

    fn cast_shape(world: &World, query: &ShapeCastQuery) -> Option<SensorHit> {
        let state = world.get_resource::<AvianSpatialQueryState>()?;
        let mut state = state
            .0
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        // Fails only while Avian's spatial query resources are missing.
        let spatial_query = state.get(world).ok()?;
        let shape = match query.shape {
            CastShape::Segment { a, b } => Collider::segment(a, b),
        };
        spatial_query
            .cast_shape(
                &shape,
                query.origin,
                0.0,
                query.direction,
                &ShapeCastConfig::from_max_distance(query.max_distance),
                &spatial_query_filter(&query.filter),
            )
            .map(|hit| SensorHit {
                entity: hit.entity,
                distance: hit.distance,
                normal: hit.normal1,
            })
    }
}

/// Cached [`SpatialQuery`] state backing [`Avian2dBackend::cast_shape`], which
/// only gets `&World`.
#[derive(Resource)]
struct AvianSpatialQueryState(std::sync::Mutex<SystemState<SpatialQuery<'static, 'static>>>);

/// Plugin that sets up Avian2D-specific systems for the character controller.
pub struct Avian2dBackendPlugin;

impl Plugin for Avian2dBackendPlugin {
    fn build(&self, app: &mut App) {
        use crate::CharacterControllerPhaseSystems;

        let spatial_query_state = SystemState::<SpatialQuery>::new(app.world_mut());
        app.insert_resource(AvianSpatialQueryState(std::sync::Mutex::new(
            spatial_query_state,
        )));
        app.add_message::<ReactiveForceApplied>();
        app.register_type::<ReactiveForceApplied>();
        app.register_type::<GroundCaster>();
        app.register_type::<LeftWallCaster>();
        app.register_type::<RightWallCaster>();
        app.register_type::<CeilingCaster>();
        app.register_type::<StairCaster>();
        app.register_type::<CurrentGroundCaster>();
        app.register_type::<CasterOfCharacter>();
        app.register_type::<CharacterCasters>();
        app.register_type::<CastersSpawned>();

        // Phase 1: Preparation - clear last step's forces.
        app.add_systems(
            FixedUpdate,
            (clear_controller_forces, clear_reactive_forces)
                .chain()
                .in_set(CharacterControllerPhaseSystems::Preparation),
        );

        // Sensors run on persistent `ShapeCaster` children: spawn them for new
        // characters and copy this step's cast requests onto them, then Avian's
        // physics step runs them and the next step reads the hits back.
        app.add_systems(
            FixedUpdate,
            (spawn_detection_casters, submit_sensor_casts)
                .chain()
                .in_set(SensorSystems::Submit),
        );
        app.add_systems(FixedUpdate, read_sensor_hits.in_set(SensorSystems::Cast));
        app.add_systems(
            FixedUpdate,
            avian_detect_falling.in_set(SensorSystems::Detect),
        );

        // Phase 4: Force Accumulation - Avian-specific gravity using Forces component
        app.add_systems(
            FixedUpdate,
            avian_accumulate_gravity.in_set(CharacterControllerPhaseSystems::ForceAccumulation),
        );

        // Phase 5: Intent Application - Avian-specific fall gravity using Forces component
        app.add_systems(
            FixedUpdate,
            avian_apply_fall_gravity.in_set(CharacterControllerPhaseSystems::IntentApplication),
        );

        // Phase 6: Final Application - Apply accumulated forces to physics
        app.add_systems(
            FixedUpdate,
            apply_controller_forces.in_set(CharacterControllerPhaseSystems::FinalApplication),
        );
    }
}

/// Automatically spawn detection casters for new character controllers.
///
/// This system detects newly added `CharacterController` entities and spawns
/// all necessary `ShapeCaster` child entities directly as children of the character.
/// The casters inherit the character's Transform (position and rotation).
fn spawn_detection_casters(
    mut commands: Commands,
    q_new_controllers: Query<
        (Entity, &CharacterController, &ControllerConfig),
        (Without<CastersSpawned>, With<CharacterController>),
    >,
) {
    for (entity, controller, config) in &q_new_controllers {
        // Spawn all casters as direct children of the character
        spawn_ground_caster(&mut commands, entity, config);
        spawn_wall_casters(&mut commands, entity, config);
        spawn_ceiling_caster(&mut commands, entity, config);

        // Spawn stair casters if enabled
        if let Some(stair_config) = &controller.stair_config
            && stair_config.enabled
        {
            spawn_stair_casters(&mut commands, entity, controller);
        }

        // Mark this entity as having casters spawned
        commands.entity(entity).insert(CastersSpawned);
    }
}

/// Get the distance from collider center to bottom for a given collider.
/// For capsules, this is `half_height` + radius.
#[must_use]
pub fn get_collider_bottom_offset(collider: &Collider) -> f32 {
    // Try to get capsule shape parameters
    if let Some(capsule) = collider.shape_scaled().as_capsule() {
        // Capsule: half-length of segment + radius
        // In Parry, capsule.segment is a field, and segment.a/segment.b are methods
        let segment = capsule.segment;
        let half_height = (segment.a.y - segment.b.y).abs() / 2.0;
        half_height + capsule.radius
    } else if let Some(ball) = collider.shape_scaled().as_ball() {
        // Ball: just the radius
        ball.radius
    } else if let Some(cuboid) = collider.shape_scaled().as_cuboid() {
        // Cuboid: half the height (y dimension)
        cuboid.half_extents.y
    } else {
        // Unknown shape: use 0 as fallback (float_height measured from center)
        0.0
    }
}

/// Build the [`SpatialQueryFilter`] for a [`CastFilter`]: hits colliders whose
/// memberships share a bit with `filter.mask`, never the character itself.
#[must_use]
pub fn spatial_query_filter(filter: &CastFilter) -> SpatialQueryFilter {
    let mut query_filter = SpatialQueryFilter::from_mask(LayerMask(filter.mask));
    query_filter.excluded_entities.insert(filter.exclude);
    query_filter
}

/// Which [`SensorKind`] a caster entity serves, from its marker component.
type CasterKindFlags = (
    Has<GroundCaster>,
    Has<LeftWallCaster>,
    Has<RightWallCaster>,
    Has<CeilingCaster>,
    Has<StairCaster>,
    Has<CurrentGroundCaster>,
);

fn caster_kind(flags: (bool, bool, bool, bool, bool, bool)) -> Option<SensorKind> {
    match flags {
        (true, ..) => Some(SensorKind::Ground),
        (_, true, ..) => Some(SensorKind::LeftWall),
        (_, _, true, ..) => Some(SensorKind::RightWall),
        (_, _, _, true, ..) => Some(SensorKind::Ceiling),
        (_, _, _, _, true, _) => Some(SensorKind::Stair),
        (.., true) => Some(SensorKind::StairGround),
        _ => None,
    }
}

/// Copy each character's cast requests onto its `ShapeCaster` children.
///
/// Runs in [`SensorSystems::Submit`], after the controller wrote this step's
/// requests and before Avian's physics step runs the casters. The caster's
/// `enabled` flag carries the request's, so a held or idle cast is switched
/// off in Avian (which runs every *enabled* caster each step, whatever the
/// parent body does) and reports no hits. The swept collider is left as
/// spawned. The query filter is rebuilt only when its mask or excluded
/// character changes.
fn submit_sensor_casts(
    mut casters: Query<(
        Entity,
        &CasterOfCharacter,
        &mut ShapeCaster,
        CasterKindFlags,
    )>,
    characters: Query<&CharacterSensors, With<ControllerConfig>>,
) {
    for (caster_entity, owner, mut shape_caster, flags) in &mut casters {
        let Some(kind) = caster_kind(flags) else {
            continue;
        };
        let Ok(sensors) = characters.get(owner.0) else {
            continue;
        };
        let cast = sensors.cast(kind);

        shape_caster.enabled = cast.enabled;
        shape_caster.origin = cast.origin;
        shape_caster.direction = cast.direction;
        shape_caster.shape_rotation = Rot2::IDENTITY;
        shape_caster.max_distance = cast.max_distance;

        // Avian adds the caster entity itself to the exclusions on every cast
        // (`ignore_self`); anything else means the filter is stale.
        let current = &shape_caster.query_filter;
        let up_to_date = current.mask.0 == cast.filter.mask
            && current.excluded_entities.contains(&cast.filter.exclude)
            && current
                .excluded_entities
                .iter()
                .all(|&e| e == cast.filter.exclude || e == caster_entity);
        if !up_to_date {
            shape_caster.query_filter = spatial_query_filter(&cast.filter);
        }
    }
}

/// Report each caster's nearest hit to its character's [`CharacterSensors`].
///
/// Runs in [`SensorSystems::Cast`]. The hits are the ones Avian found in its
/// last physics step. The reading's origin is the caster's global translation
/// plus its local `origin`, as the Avian detection always measured it.
fn read_sensor_hits(
    casters: Query<(
        &CasterOfCharacter,
        &ShapeCaster,
        Option<&ShapeHits>,
        &GlobalTransform,
        CasterKindFlags,
    )>,
    mut characters: Query<&mut CharacterSensors>,
) {
    for (owner, shape_caster, hits, transform, flags) in &casters {
        let Some(kind) = caster_kind(flags) else {
            continue;
        };
        let Ok(mut sensors) = characters.get_mut(owner.0) else {
            continue;
        };
        let hit = hits.and_then(|hits| hits.first()).map(|hit| SensorHit {
            entity: hit.entity,
            distance: hit.distance,
            normal: hit.normal1,
        });
        sensors.set_reading(
            kind,
            SensorReading {
                origin: transform.translation().xy() + shape_caster.origin,
                hit,
            },
        );
    }
}

/// Spawn a ground detection `ShapeCaster` as a direct child of the character entity.
///
/// This function creates a child entity with a `ShapeCaster` component configured
/// for ground detection. The caster direction and distance will be updated
/// dynamically by the `submit_sensor_casts` system.
///
/// The caster inherits the character's Transform, including both position and rotation.
///
/// # Parameters
/// - `character`: The character entity (parent in hierarchy and `CasterOfCharacter` linkage)
/// - `config`: Configuration for the character controller
///
/// Note: Collision layers are inherited from the character in the update system.
pub fn spawn_ground_caster(commands: &mut Commands, character: Entity, config: &ControllerConfig) {
    let half_width = config.sensors.ground_cast_width / 2.0;

    let child = commands
        .spawn((
            Name::new("GroundCaster"),
            GroundCaster,
            CasterOfCharacter(character), // Relationship: belongs to this character
            ShapeCaster::new(
                Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
                Vec2::ZERO,
                0.0,
                Dir2::NEG_Y,
            )
            .with_max_distance(20.0)
            .with_max_hits(1)
            .with_ignore_self(true),
            Transform::default(),
            GlobalTransform::default(),
        ))
        .id();

    // Set up parent-child relationship for Transform propagation
    commands.entity(character).add_child(child);
}

/// Spawn left wall detection `ShapeCaster` as a direct child of the character entity.
///
/// This function creates a child entity with a `ShapeCaster` component configured
/// for left wall detection. The caster direction will be updated dynamically
/// by the `submit_sensor_casts` system.
///
/// The caster inherits the character's Transform, including both position and rotation.
///
/// # Parameters
/// - `character`: The character entity (parent in hierarchy and `CasterOfCharacter` linkage)
/// - `config`: Configuration for the character controller
///
/// Note: Collision layers are inherited from the character in the update system.
pub fn spawn_left_wall_caster(
    commands: &mut Commands,
    character: Entity,
    config: &ControllerConfig,
) {
    let half_height = config.sensors.wall_cast_height / 2.0;
    let child = commands
        .spawn((
            LeftWallCaster,
            CasterOfCharacter(character), // Relationship: belongs to this character
            ShapeCaster::new(
                Collider::segment(Vec2::new(0.0, -half_height), Vec2::new(0.0, half_height)),
                Vec2::ZERO,
                0.0,
                Dir2::NEG_X,
            )
            .with_max_distance(10.0)
            .with_max_hits(1)
            .with_ignore_self(true),
            Transform::default(),
            GlobalTransform::default(),
        ))
        .id();

    // Set up parent-child relationship for Transform propagation
    commands.entity(character).add_child(child);
}

/// Spawn right wall detection `ShapeCaster` as a direct child of the character entity.
///
/// This function creates a child entity with a `ShapeCaster` component configured
/// for right wall detection. The caster direction will be updated dynamically
/// by the `submit_sensor_casts` system.
///
/// The caster inherits the character's Transform, including both position and rotation.
///
/// # Parameters
/// - `character`: The character entity (parent in hierarchy and `CasterOfCharacter` linkage)
/// - `config`: Configuration for the character controller
///
/// Note: Collision layers are inherited from the character in the update system.
pub fn spawn_right_wall_caster(
    commands: &mut Commands,
    character: Entity,
    config: &ControllerConfig,
) {
    let half_height = config.sensors.wall_cast_height / 2.0;
    let child = commands
        .spawn((
            RightWallCaster,
            CasterOfCharacter(character), // Relationship: belongs to this character
            ShapeCaster::new(
                Collider::segment(Vec2::new(0.0, -half_height), Vec2::new(0.0, half_height)),
                Vec2::ZERO,
                0.0,
                Dir2::X,
            )
            .with_max_distance(10.0)
            .with_max_hits(1)
            .with_ignore_self(true),
            Transform::default(),
            GlobalTransform::default(),
        ))
        .id();

    // Set up parent-child relationship for Transform propagation
    commands.entity(character).add_child(child);
}

/// Spawn both left and right wall casters.
///
/// Convenience function to spawn both wall casters at once.
///
/// # Parameters
/// - `character`: The character entity (parent in hierarchy and `CasterOfCharacter` linkage)
/// - `config`: Configuration for the character controller
pub fn spawn_wall_casters(commands: &mut Commands, character: Entity, config: &ControllerConfig) {
    spawn_left_wall_caster(commands, character, config);
    spawn_right_wall_caster(commands, character, config);
}

/// Spawn ceiling detection `ShapeCaster` as a direct child of the character entity.
///
/// This function creates a child entity with a `ShapeCaster` component configured
/// for ceiling detection. The caster direction will be updated dynamically
/// by the `submit_sensor_casts` system.
///
/// The caster inherits the character's Transform, including both position and rotation.
///
/// # Parameters
/// - `character`: The character entity (parent in hierarchy and `CasterOfCharacter` linkage)
/// - `config`: Configuration for the character controller
///
/// Note: Collision layers are inherited from the character in the update system.
pub fn spawn_ceiling_caster(commands: &mut Commands, character: Entity, config: &ControllerConfig) {
    let half_width = config.sensors.ceiling_cast_width / 2.0;
    let child = commands
        .spawn((
            CeilingCaster,
            CasterOfCharacter(character), // Relationship: belongs to this character
            ShapeCaster::new(
                Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
                Vec2::ZERO,
                0.0,
                Dir2::Y,
            )
            .with_max_distance(10.0)
            .with_max_hits(1)
            .with_ignore_self(true),
            Transform::default(),
            GlobalTransform::default(),
        ))
        .id();

    // Set up parent-child relationship for Transform propagation
    commands.entity(character).add_child(child);
}

/// Spawn stair detection `ShapeCasters` as direct children of the character entity.
///
/// This function creates two child entities:
/// - `StairCaster`: forward-down cast for detecting the step surface
/// - `CurrentGroundCaster`: downward cast for detecting current ground level
///
/// Both casters start disabled and are enabled/positioned dynamically by the
/// `submit_sensor_casts` system when the character is walking.
/// The casters inherit the character's Transform, including both position and rotation.
///
/// # Parameters
/// - `character`: The character entity (parent in hierarchy and `CasterOfCharacter` linkage)
/// - `controller`: The character controller (for stair config)
///
/// Note: Collision layers are inherited from the character in the update system.
pub fn spawn_stair_casters(
    commands: &mut Commands,
    character: Entity,
    controller: &CharacterController,
) {
    // Get stair config, or use default if not set
    let stair_config = controller.stair_config.unwrap_or_default();

    let half_width = stair_config.stair_cast_width / 2.0;

    // Stair caster (forward-down detection) - starts disabled
    // Note: ShapeCaster::disable() is &mut self (not a builder), so we set enabled=false separately.
    let mut stair_shape_caster = ShapeCaster::new(
        Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
        Vec2::ZERO,
        0.0,
        Dir2::NEG_Y,
    )
    .with_max_distance(stair_config.max_climb_height)
    .with_max_hits(1)
    .with_ignore_self(true);
    stair_shape_caster.enabled = false;

    let stair_child = commands
        .spawn((
            StairCaster,
            CasterOfCharacter(character), // Relationship: belongs to this character
            stair_shape_caster,
            Transform::default(),
            GlobalTransform::default(),
        ))
        .id();

    // Set up parent-child relationship for Transform propagation
    commands.entity(character).add_child(stair_child);

    // Current ground caster (for step height reference) - starts disabled
    let mut ground_shape_caster = ShapeCaster::new(
        Collider::segment(Vec2::new(-half_width, 0.0), Vec2::new(half_width, 0.0)),
        Vec2::ZERO,
        0.0,
        Dir2::NEG_Y,
    )
    .with_max_distance(20.0)
    .with_max_hits(1)
    .with_ignore_self(true);
    ground_shape_caster.enabled = false;

    let ground_child = commands
        .spawn((
            CurrentGroundCaster,
            CasterOfCharacter(character), // Relationship: belongs to this character
            ground_shape_caster,
            Transform::default(),
            GlobalTransform::default(),
        ))
        .id();

    // Set up parent-child relationship for Transform propagation
    commands.entity(character).add_child(ground_child);
}

/// Get the side radius of a collider: capsule or ball radius, half a cuboid's width.
#[must_use]
pub fn get_collider_radius(collider: &Collider) -> f32 {
    if let Some(capsule) = collider.shape_scaled().as_capsule() {
        capsule.radius
    } else if let Some(ball) = collider.shape_scaled().as_ball() {
        ball.radius
    } else if let Some(cuboid) = collider.shape_scaled().as_cuboid() {
        // Use half the width as an approximation
        cuboid.half_extents.x
    } else {
        0.0
    }
}

/// Clear controller forces at the start of each frame.
///
/// This system runs BEFORE any controller force systems. It:
/// 1. Subtracts the forces we applied last frame from `ConstantForce`
/// 2. Clears the accumulators for the new frame
///
/// This ensures that external user forces are preserved while our forces
/// are "isolated" between frames.
pub fn clear_controller_forces(
    mut q: Query<(
        &mut CharacterController,
        Option<&mut ConstantForce>,
        Option<&mut ConstantTorque>,
    )>,
) {
    for (mut controller, constant_force, constant_torque) in &mut q {
        // Get the forces we applied last frame and clear for the new frame
        let (force_to_subtract, torque_to_subtract) = controller.prepare_new_frame();

        // Subtract our previously applied forces
        if let Some(mut force) = constant_force {
            force.0 -= force_to_subtract;
        }
        if let Some(mut torque) = constant_torque {
            torque.0 -= torque_to_subtract;
        }
    }
}

/// Clear reactive forces (ground reaction forces) from the previous frame.
///
/// This system runs in the Preparation phase alongside `clear_controller_forces`.
/// It reads `ReactiveForceApplied` messages from the previous frame and subtracts
/// those forces from the corresponding entities' `ConstantForce` components.
///
/// This ensures that ground entities don't accumulate forces indefinitely
/// when characters stand on them.
pub fn clear_reactive_forces(
    mut messages: MessageReader<ReactiveForceApplied>,
    mut forces: Query<&mut ConstantForce>,
) {
    const EPSILON: f32 = 1e-6;

    for msg in messages.read() {
        let Ok(mut force) = forces.get_mut(msg.entity) else {
            continue;
        };

        // Subtract the force that was applied last frame
        force.0 -= msg.force;

        // Clean up near-zero values to avoid floating point drift
        if force.0.x.abs() < EPSILON {
            force.0.x = 0.0;
        }
        if force.0.y.abs() < EPSILON {
            force.0.y = 0.0;
        }
    }
}

/// Detect falling state by checking vertical velocity.
///
/// This system runs early to set the `falling` flag on the `CharacterController`,
/// which is then used by fall gravity and other systems. This avoids query conflicts
/// with the Forces API that needs mutable access to velocity.
pub fn avian_detect_falling(
    mut query: Query<(&mut CharacterController, &LinearVelocity), Without<RigidBodyDisabled>>,
) {
    for (mut controller, velocity) in &mut query {
        let up = controller.ideal_up();
        let vertical_velocity = velocity.0.dot(up);
        controller.falling = vertical_velocity < 0.0;
    }
}

/// Apply gravity acceleration to airborne characters using Avian's Forces API.
///
/// This is the Avian-specific gravity system that uses the `Forces` `QueryData`
/// with `apply_linear_acceleration()`. This is the idiomatic Avian 0.4+ way to
/// apply non-persistent forces that get cleared automatically each frame.
///
/// This approach supports dynamic gravity that can change every frame (e.g., for
/// spherical planets with radial gravity).
pub fn avian_accumulate_gravity(
    mut query: Query<(Forces, &CharacterController, &ControllerConfig), Without<RigidBodyDisabled>>,
) {
    for (mut forces, controller, config) in &mut query {
        // Only apply gravity to airborne entities
        if !controller.is_grounded(config) {
            // Apply gravity as linear acceleration
            // Forces API handles mass multiplication and integration automatically
            forces.apply_linear_acceleration(controller.gravity);
        }
    }
}

/// Apply fall gravity for early jump cancellation using Avian's Forces API.
///
/// This system enables players to cancel jumps early by releasing the jump button,
/// making jumps feel less floaty. Fall gravity is triggered when:
///
/// 1. We jumped recently (within `jump_cancel_window`)
/// 2. AND either:
///    - Jump button is not held (player let go)
///    - OR we're moving downward (crossed the zenith)
///
/// When triggered, fall gravity is applied for `fall_gravity_duration`,
/// multiplying gravity by `fall_gravity`.
///
/// Note: This system applies additional gravity on top of the base gravity
/// applied by `avian_accumulate_gravity`. Both use the Forces API.
pub fn avian_apply_fall_gravity(
    mut query: Query<
        (
            Forces,
            &mut CharacterController,
            &ControllerConfig,
            &MovementIntent,
        ),
        Without<RigidBodyDisabled>,
    >,
) {
    for (mut forces, mut controller, config, intent) in &mut query {
        // Only process airborne entities with fall_gravity > 1.0
        if controller.is_grounded(config) || config.jumping.fall_gravity <= 1.0 {
            continue;
        }

        // Check if we should trigger fall gravity
        // Uses controller.falling (set by avian_detect_falling system) to detect
        // when we've crossed the jump apex, avoiding query conflicts with Forces.
        let max_ascent_expired = controller.jump_max_ascent_expired();
        let should_trigger = controller.in_jump_cancel_window()
            && !controller.recently_jumped()
            && (!intent.jump_pressed || controller.falling || max_ascent_expired);

        // Trigger fall gravity if conditions are met
        if should_trigger && !controller.fall_gravity_active() {
            controller.trigger_fall_gravity(config.jumping.fall_gravity_duration);
        }

        // Apply fall gravity if the timer is active
        if controller.fall_gravity_active() {
            // Apply additional gravity acceleration
            // The regular gravity system applies: gravity
            // We want to add: gravity * (fall_gravity - 1)
            // This way total becomes: gravity * fall_gravity
            let fall_multiplier = config.jumping.fall_gravity - 1.0;
            forces.apply_linear_acceleration(controller.gravity * fall_multiplier);
        }
    }
}

/// Apply controller forces at the end of each frame.
///
/// This system runs AFTER all controller force systems. It:
/// 1. Applies accumulated forces to ConstantForce/ConstantTorque
/// 2. Stores what we applied for next frame's subtraction
/// 3. Applies ground reaction forces to dynamic ground bodies
/// 4. Fires `ReactiveForceApplied` messages for ground reaction forces
///
/// This ensures our forces are integrated by Avian's physics step.
pub fn apply_controller_forces(
    mut q: Query<
        (
            &mut CharacterController,
            Option<&mut ConstantForce>,
            Option<&mut ConstantTorque>,
        ),
        Without<RigidBodyDisabled>,
    >,
    mut ground_forces: Query<(&mut ConstantForce, &RigidBody), Without<CharacterController>>,
    mut reactive_messages: MessageWriter<ReactiveForceApplied>,
) {
    for (mut controller, constant_force, constant_torque) in &mut q {
        // Get accumulated forces and prepare for next frame
        let (force_to_apply, torque_to_apply) = controller.finalize_frame();

        // Apply our accumulated forces
        if let Some(mut force) = constant_force {
            force.0 += force_to_apply;
        }
        if let Some(mut torque) = constant_torque {
            torque.0 += torque_to_apply;
        }

        // Apply ground reaction force to dynamic ground bodies
        if let Some((ground_entity, reaction_force)) = controller.take_ground_reaction()
            && let Ok((mut ground_force, rigid_body)) = ground_forces.get_mut(ground_entity)
        {
            // Only apply reaction force to dynamic bodies
            if *rigid_body == RigidBody::Dynamic {
                ground_force.0 += reaction_force;

                // Fire message so the force can be cleared next frame
                reactive_messages.write(ReactiveForceApplied {
                    entity: ground_entity,
                    force: reaction_force,
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CharacterControllerPlugin;

    fn create_test_app() -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(bevy::transform::TransformPlugin);
        // Insert WorldInstanceSpawner resource required by Avian's ColliderHierarchyPlugin
        app.insert_resource(bevy::world_serialization::WorldInstanceSpawner::default());
        app.add_plugins(PhysicsPlugins::default());
        app.insert_resource(Time::<Fixed>::from_hz(60.0));
        app.finish();
        app.cleanup();
        app
    }

    #[test]
    fn avian_backend_get_position() {
        let mut app = create_test_app();

        let entity = app
            .world_mut()
            .spawn((Transform::from_xyz(100.0, 200.0, 0.0), RigidBody::Dynamic))
            .id();

        app.update();

        let pos = Avian2dBackend::get_position(app.world(), entity);
        assert!((pos.x - 100.0).abs() < 0.01);
        assert!((pos.y - 200.0).abs() < 0.01);
    }

    #[test]
    fn avian_backend_velocity() {
        let mut app = create_test_app();

        let entity = app
            .world_mut()
            .spawn((
                Transform::default(),
                RigidBody::Dynamic,
                LinearVelocity(Vec2::new(50.0, 30.0)),
            ))
            .id();

        app.update();

        let vel = Avian2dBackend::get_velocity(app.world(), entity);
        assert!((vel.x - 50.0).abs() < 0.01);
        assert!((vel.y - 30.0).abs() < 0.01);

        Avian2dBackend::set_velocity(app.world_mut(), entity, Vec2::new(100.0, 0.0));

        let vel = Avian2dBackend::get_velocity(app.world(), entity);
        assert!((vel.x - 100.0).abs() < 0.01);
        assert!(vel.y.abs() < 0.01);
    }

    fn create_test_app_with_controller() -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(bevy::transform::TransformPlugin);
        app.insert_resource(bevy::world_serialization::WorldInstanceSpawner::default());
        app.add_plugins(PhysicsPlugins::default());
        app.add_plugins(CharacterControllerPlugin::<Avian2dBackend>::default());
        app.insert_resource(Time::<Fixed>::from_hz(60.0));
        app.finish();
        app.cleanup();
        app
    }

    #[test]
    fn character_controller_requires_physics_components() {
        let mut app = create_test_app_with_controller();

        let entity = app
            .world_mut()
            .spawn((
                Transform::default(),
                CharacterController::new(),
                ControllerConfig::default(),
                Collider::capsule(4.0, 8.0),
                GravityScale(0.0),
            ))
            .id();

        app.update();

        // CharacterController #[require] should have inserted these
        assert!(app.world().get::<RigidBody>(entity).is_some());
        assert!(app.world().get::<LinearVelocity>(entity).is_some());
        assert!(app.world().get::<ConstantForce>(entity).is_some());
        assert!(app.world().get::<ConstantTorque>(entity).is_some());
        assert!(app.world().get::<LockedAxes>(entity).is_some());
    }
}
