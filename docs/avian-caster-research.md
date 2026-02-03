# Avian2D Spatial Query Component API Research

## Executive Summary

Avian2D provides two complementary approaches for spatial queries:

1. **Component-Based API**: `RayCaster`/`ShapeCaster` components that run automatically every frame
2. **System Parameter API**: `SpatialQuery` system parameter for on-demand queries

Our current implementation uses the **system parameter approach** with `SpatialQuery`, which matches our needs for precise per-frame control in a fixed update schedule.

## Component-Based API: ShapeCaster & RayCaster

### ShapeCaster

A Bevy ECS component that performs automatic shapecasting every frame.

#### Configuration Fields

| Field | Type | Default | Purpose |
|-------|------|---------|---------|
| `enabled` | `bool` | `true` | Enable/disable the caster |
| `shape` | `Collider` | Required | The shape to cast (capsule, segment, circle, etc.) |
| `origin` | `Vector` | Required | Local origin relative to entity position |
| `shape_rotation` | `Scalar` | Required | Local rotation in radians |
| `direction` | `Dir2` | Required | Direction to cast (local space) |
| `max_distance` | `Scalar` | Infinite | Maximum cast distance |
| `max_hits` | `u32` | `1` | Maximum number of hits to return |
| `target_distance` | `Scalar` | `0.0` | Separation distance threshold |
| `compute_contact_on_penetration` | `bool` | `false` | Calculate contacts at zero distance |
| `ignore_origin_penetration` | `bool` | `false` | Skip initial penetrations |
| `ignore_self` | `bool` | `true` | Skip hits against own collider |
| `query_filter` | `SpatialQueryFilter` | Default | Collision filtering rules |

#### Construction Pattern

```rust
// Basic construction
let caster = ShapeCaster::new(
    Collider::segment(Vec2::NEG_X * 10.0, Vec2::X * 10.0),  // horizontal segment
    Vec2::ZERO,              // origin
    0.0,                     // rotation
    Dir2::NEG_Y              // cast downward
);

// Builder pattern configuration
let caster = caster
    .with_max_distance(100.0)
    .with_max_hits(5)
    .with_ignore_self(true)
    .with_query_filter(SpatialQueryFilter::from_mask(my_collision_layers));
```

#### Accessing Results

Results are stored in a separate `ShapeHits` component:

```rust
fn read_shape_hits(
    query: Query<(&ShapeCaster, &ShapeHits), With<MyCharacter>>
) {
    for (caster, hits) in &query {
        if !caster.enabled {
            continue;
        }

        // ShapeHits derefs to Vec<ShapeHitData>
        for hit in hits.iter() {
            println!("Hit entity {:?} at distance {}", hit.entity, hit.distance);
            println!("  Normal: {}", hit.normal1);
            println!("  Point on surface: {}", hit.point1);
            println!("  Point on cast shape: {}", hit.point2);
        }

        // Get closest hit (hits are sorted by distance)
        if let Some(closest) = hits.first() {
            println!("Closest hit: {:?}", closest);
        }
    }
}
```

### RayCaster

Similar to ShapeCaster but for ray queries (infinitely thin line).

#### Configuration Fields

| Field | Type | Default | Purpose |
|-------|------|---------|---------|
| `enabled` | `bool` | `true` | Enable/disable the ray caster |
| `origin` | `Vector` | Required | Local ray starting point |
| `direction` | `Dir2` | Required | Local ray direction |
| `max_distance` | `Scalar` | Infinite | Maximum ray travel distance |
| `max_hits` | `u32` | Unbounded | Maximum hits to return |
| `solid` | `bool` | `true` | Treat shapes as solid vs hollow |
| `ignore_self` | `bool` | `true` | Skip hits against own collider |
| `query_filter` | `SpatialQueryFilter` | Default | Collision filtering rules |

#### Construction Pattern

```rust
// Create from origin and direction
let ray_caster = RayCaster::new(Vec2::ZERO, Dir2::NEG_Y)
    .with_max_distance(50.0)
    .with_max_hits(1)  // Only need closest hit
    .with_solidness(true);

// Or from a Ray2d
let ray = Ray2d::new(Vec2::ZERO, Dir2::NEG_Y);
let ray_caster = RayCaster::from_ray(ray)
    .with_max_distance(50.0);
```

#### Accessing Results

Results are in a `RayHits` component:

```rust
fn read_ray_hits(
    query: Query<(&RayCaster, &RayHits), With<MyCharacter>>
) {
    for (caster, hits) in &query {
        if !caster.enabled {
            continue;
        }

        // RayHits are NOT sorted by default - use iter_sorted() for distance order
        for hit in hits.iter_sorted() {
            println!("Hit entity {:?} at distance {}", hit.entity, hit.distance);
            println!("  Normal: {}", hit.normal);
        }

        // Or iterate unsorted (more efficient, no allocation)
        for hit in hits.iter() {
            // Process hits in arbitrary order
        }
    }
}
```

## Hit Data Structures

### ShapeHitData

Complete collision information from shape casting:

```rust
pub struct ShapeHitData {
    pub entity: Entity,      // Entity that was hit
    pub distance: Scalar,    // Distance traveled before hit
    pub point1: Vector,      // Closest point on hit shape (world space)
    pub point2: Vector,      // Closest point on cast shape (world space)
    pub normal1: Vector,     // Outward normal on hit shape at point1
    pub normal2: Vector,     // Outward normal on cast shape at point2
}
```

**Key Points:**
- `point1` and `normal1` refer to the surface that was hit
- `point2` and `normal2` refer to the casting shape
- When shapes are penetrating or `target_distance > 0`, `point1` and `point2` differ
- Implements `Copy`, `Clone` - lightweight to pass around
- Sorted by distance in `ShapeHits` vector

### RayHitData

Simpler hit information for rays:

```rust
pub struct RayHitData {
    pub entity: Entity,      // Entity that was hit
    pub distance: Scalar,    // Distance along ray to intersection
    pub normal: Vector,      // Surface normal at intersection (world space)
}
```

**Key Points:**
- More lightweight than `ShapeHitData` (no point2/normal2)
- Implements `Copy`, `Clone`
- **NOT sorted by default** - use `RayHits::iter_sorted()` for distance order
- Point of intersection can be calculated: `ray_origin + ray_direction * distance`

## Current Implementation Analysis

Our current Avian implementation (`src/avian.rs`) uses the **system parameter approach** with `SpatialQuery`:

```rust
fn avian_ground_detection(
    spatial_query: SpatialQuery,  // System parameter, not component
    mut q_controllers: Query<...>
) {
    // Create filter
    let filter = if let Some(layers) = collision_layers {
        SpatialQueryFilter::from_mask(layers.filters)
            .with_excluded_entities([exclude_entity])
    } else {
        SpatialQueryFilter::default()
            .with_excluded_entities([exclude_entity])
    };

    // Create config
    let config = ShapeCastConfig::from_max_distance(max_distance);

    // Perform cast
    spatial_query.cast_shape(
        &shape,
        origin,
        shape_rotation,
        Dir2::new(direction).unwrap_or(Dir2::NEG_Y),
        &config,
        &filter,
    )
}
```

### Data We Need

Our `CollisionData` structure requires:

```rust
pub struct CollisionData {
    pub distance: f32,       // ✓ Available in both APIs
    pub normal: Vec2,        // ✓ Available (normal1 for shapes, normal for rays)
    pub point: Vec2,         // ✓ Available (point1 for shapes, calculated for rays)
    pub entity: Option<Entity>, // ✓ Available in both APIs
}
```

All required data is available from both the component API and system parameter API.

## Comparison: Component API vs System Parameter API

| Aspect | Component API | System Parameter API (Current) |
|--------|---------------|-------------------------------|
| **Update Frequency** | Every frame automatically | On-demand per system call |
| **Control** | Less (auto-updates) | More (manual control) |
| **Overhead** | Potential waste if not always needed | Only when called |
| **Local Coordinates** | Automatic (follows entity) | Manual (we pass position) |
| **Results** | Stored in component (queryable) | Returned immediately |
| **Multiple Queries** | Requires multiple components | Single parameter, multiple calls |
| **Scheduling** | Automatic with Avian's schedule | We control when it runs |
| **State Persistence** | Results persist until next update | Results must be stored manually |

### Use Case Fit

Our character controller needs:

1. **Precise timing control**: We run in `FixedUpdate` in specific phases (Sensors phase)
2. **Conditional queries**: We only check stairs when walking, only certain casts per frame
3. **Multiple queries per entity**: Ground, left wall, right wall, ceiling, stairs (5 casts)
4. **Frame isolation**: Detection state is reset each frame, no persistence needed

**Recommendation: Continue using System Parameter API**

The system parameter approach is a better fit because:

- We need precise control over when queries run (Sensors phase in FixedUpdate)
- We conditionally perform queries (stairs only when walking)
- We perform multiple different queries per character per frame
- We don't need results to persist beyond the current frame
- We manually store results in `CharacterController` fields (`floor`, `left_wall`, etc.)

## Component API Considerations (If We Switch)

If we wanted to use the component-based API, here's how it would work:

### Setup

```rust
// Spawn character with caster components
commands.spawn((
    // Character components
    Transform::default(),
    CharacterController::new(),
    ControllerConfig::default(),
    Collider::capsule(4.0, 8.0),

    // Ground caster
    ShapeCaster::new(
        Collider::segment(Vec2::NEG_X * 5.0, Vec2::X * 5.0),
        Vec2::ZERO,
        0.0,
        Dir2::NEG_Y
    )
    .with_max_distance(50.0)
    .with_max_hits(1),

    // Would need separate components for each cast direction
    // or a marker component to differentiate them
    GroundCaster,  // Custom marker
));

// Would need 5 separate caster components per character:
// - GroundCaster (down)
// - LeftWallCaster (left)
// - RightWallCaster (right)
// - CeilingCaster (up)
// - StairCaster (conditional, down-forward)
```

### Reading Results

```rust
fn avian_ground_detection_component(
    query: Query<(
        &ShapeCaster,
        &ShapeHits,
        &mut CharacterController,
        &ControllerConfig,
        &GlobalTransform,
    ), With<GroundCaster>>
) {
    for (caster, hits, mut controller, config, transform) in &query {
        // Reset state
        controller.reset_detection_state();

        // Process hits (already sorted by distance)
        if let Some(ground_hit) = hits.first() {
            let normal = ground_hit.normal1;
            let up = controller.ideal_up();
            let dot = normal.dot(up).clamp(-1.0, 1.0);
            let slope_angle = dot.acos();

            controller.floor = Some(CollisionData::new(
                ground_hit.distance,
                normal,
                ground_hit.point1,
                Some(ground_hit.entity),
            ));
            controller.slope_angle = slope_angle;
        }
    }
}
```

### Challenges with Component API

1. **Multiple Casters per Entity**: Need 5 different caster components per character
   - Requires marker components to differentiate them (`GroundCaster`, `WallCaster`, etc.)
   - Complex entity setup

2. **Dynamic Configuration**: Our casts use gravity-relative directions that change
   - Spherical planets: gravity changes based on position
   - Would need to update `origin`, `direction`, `shape_rotation` every frame
   - Loses the "automatic" benefit of component API

3. **Conditional Queries**: Stair detection only when walking
   - Would need to enable/disable caster component based on movement state
   - Adds complexity vs just not calling the query

4. **Query Coordination**: Multiple systems reading different casters
   - Need careful marker components to avoid query conflicts
   - More complex than single detection system

5. **Results Timing**: Components update during Avian's spatial query pipeline
   - May not align perfectly with our Sensors phase
   - Less control over execution order

## Gotchas and Limitations

### Component API

1. **Automatic Updates**: Casters run every frame when enabled
   - Can't easily skip frames or add conditional logic
   - Overhead when queries aren't needed

2. **Hit Ordering**:
   - `ShapeHits` are sorted by distance (good!)
   - `RayHits` are NOT sorted by default (must use `iter_sorted()`)

3. **Max Hits**:
   - Setting `max_hits` too low means you might miss closer hits
   - Setting it too high increases computational cost
   - For "closest only", use `max_hits = 1`

4. **Local Coordinates**:
   - Casters use local space (relative to entity transform)
   - Convenient for fixed casts, but we need gravity-relative directions
   - Would require updating direction every frame anyway

5. **Filter Configuration**:
   - Must configure `query_filter` with collision layers
   - No default inheritance from entity's collider
   - Must manually sync with `CollisionLayers` component

### System Parameter API

1. **Manual Calling**: Must call for each query
   - Pro: Complete control
   - Con: More boilerplate

2. **No State Persistence**: Results aren't automatically stored
   - Must manually store in components (we already do this)

3. **Direction Validation**: Must ensure direction vector is normalized
   - `Dir2::new()` returns `Result` that can fail
   - Use `.unwrap_or()` for fallback

4. **Collision Layers**: Must manually construct `SpatialQueryFilter`
   - Current code does this correctly with `from_mask()`

## Recommendations

### For Our Implementation

1. **Keep System Parameter API**: Current approach is ideal for our use case
   - Precise control over timing (Sensors phase)
   - Conditional execution (stairs)
   - Multiple queries per character
   - Dynamic gravity-relative directions

2. **Current Pattern is Solid**: The `avian_shapecast()` helper function is well-designed
   - Encapsulates shape creation (segments)
   - Handles collision filtering correctly
   - Converts to our `CollisionData` format cleanly

3. **Potential Optimizations**:
   - Cache `Dir2::new()` results if directions don't change often
   - Consider using raycasts instead of shapecasts where appropriate (simpler, faster)
   - Profile to see if shapecasts are bottleneck (unlikely with 5 per character)

### When to Consider Component API

Consider switching to component API if:

1. **Fixed Directions**: Casts always go in entity-relative directions (not gravity-relative)
2. **Always Active**: All casts run every frame unconditionally
3. **Simple Setup**: Single cast per entity, or small number of fixed casts
4. **Persistent Results**: Need to query results from multiple systems

This doesn't match our use case, so the system parameter API is the right choice.

## Code Examples

### Current Pattern (System Parameter - RECOMMENDED)

```rust
fn avian_ground_detection(
    spatial_query: SpatialQuery,
    mut q_controllers: Query<(
        Entity,
        &GlobalTransform,
        &ControllerConfig,
        &mut CharacterController,
        Option<&CollisionLayers>,
    )>
) {
    for (entity, transform, config, mut controller, collision_layers) in &mut q_controllers {
        let position = transform.translation().xy();
        let down = controller.ideal_down();

        // Create shape
        let shape = Collider::segment(
            Vec2::new(-config.ground_cast_width / 2.0, 0.0),
            Vec2::new(config.ground_cast_width / 2.0, 0.0)
        );

        // Create filter
        let filter = if let Some(layers) = collision_layers {
            SpatialQueryFilter::from_mask(layers.filters)
                .with_excluded_entities([entity])
        } else {
            SpatialQueryFilter::default()
                .with_excluded_entities([entity])
        };

        // Configure cast
        let config = ShapeCastConfig::from_max_distance(50.0);

        // Perform cast
        if let Some(hit) = spatial_query.cast_shape(
            &shape,
            position,
            0.0,  // rotation
            Dir2::new(down).unwrap_or(Dir2::NEG_Y),
            &config,
            &filter,
        ) {
            controller.floor = Some(CollisionData::new(
                hit.distance,
                hit.normal1,
                position + down * hit.distance,
                Some(hit.entity),
            ));
        }
    }
}
```

### Alternative Pattern (Component API - Not Recommended for Our Use Case)

```rust
// Marker components for different cast types
#[derive(Component)]
struct GroundCaster;

#[derive(Component)]
struct WallCasterLeft;

#[derive(Component)]
struct WallCasterRight;

// Setup (complex with multiple casters)
fn spawn_character(mut commands: Commands) {
    commands.spawn((
        Transform::default(),
        CharacterController::new(),
        ControllerConfig::default(),
        Collider::capsule(4.0, 8.0),
    ))
    .with_children(|parent| {
        // Ground caster as child
        parent.spawn((
            ShapeCaster::new(
                Collider::segment(Vec2::NEG_X * 5.0, Vec2::X * 5.0),
                Vec2::ZERO,
                0.0,
                Dir2::NEG_Y
            )
            .with_max_distance(50.0)
            .with_max_hits(1),
            GroundCaster,
        ));

        // Would need similar spawns for left wall, right wall, ceiling, stairs
    });
}

// Update caster directions based on gravity (needed for spherical planets)
fn update_caster_directions(
    mut query: Query<(&mut ShapeCaster, &Parent, &GroundCaster)>,
    controllers: Query<&CharacterController>,
) {
    for (mut caster, parent, _marker) in &mut query {
        let Ok(controller) = controllers.get(parent.get()) else { continue };

        // Update direction based on gravity
        let down = controller.ideal_down();
        caster.direction = Dir2::new(down).unwrap_or(Dir2::NEG_Y);

        // Update rotation
        caster.shape_rotation = controller.ideal_up_angle() - FRAC_PI_2;
    }
}

// Read results
fn read_ground_caster(
    mut query: Query<(&ShapeHits, &Parent), With<GroundCaster>>,
    mut controllers: Query<&mut CharacterController>,
) {
    for (hits, parent) in &query {
        let Ok(mut controller) = controllers.get_mut(parent.get()) else { continue };

        controller.reset_detection_state();

        if let Some(hit) = hits.first() {
            controller.floor = Some(CollisionData::new(
                hit.distance,
                hit.normal1,
                hit.point1,
                Some(hit.entity),
            ));
        }
    }
}
```

## Conclusion

The **System Parameter API** (`SpatialQuery`) is the correct choice for our character controller:

- Provides precise control over query timing
- Supports conditional queries (stairs only when walking)
- Works well with dynamic gravity-relative directions
- Simpler to manage multiple queries per character
- Better fits our architecture of centralized detection systems

The **Component API** (`ShapeCaster`/`RayCaster`) would be better for:

- Always-on raycasts (laser beams, line-of-sight checks)
- Fixed entity-relative directions
- Situations where query results need to be accessed by multiple systems
- Simpler setups with one or two casts per entity

Our current implementation is well-designed and should not be changed to use the component API.
