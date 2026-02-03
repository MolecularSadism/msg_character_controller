# Task: Replace Manual Shapecasting with Avian ECS Components

## Problem Analysis

The Avian2D implementation currently uses manual shapecasting with `SpatialQuery` system parameter and helper functions. This is overly complex. Avian provides simple ECS components that handle this automatically:

**Current approach (COMPLEX):**
```rust
fn avian_shapecast(spatial_query: &SpatialQuery, ...) -> Option<CollisionData> {
    // 50+ lines of manual query setup
}
```

**Target approach (SIMPLE):**
```rust
// Add ShapeCaster component to entity
// Query ShapeHits component to read results
fn read_ground_hits(query: Query<&ShapeHits, With<GroundDetector>>) {
    for hits in &query {
        for hit in hits.iter() {
            // Process hit.entity, hit.distance, etc.
        }
    }
}
```

## Solution: Pure ECS Pattern

1. Add `ShapeCaster`/`RayCaster` components to character entities (or child entities)
2. Avian automatically populates `ShapeHits`/`RayHits` components
3. Query `ShapeHits`/`RayHits` in systems to read results
4. No helpers, no manual queries, just ECS

## Tasks

### Task 1: Research Avian Caster Components
**Owner:** Senior Engineer
**Status:** Pending
**Description:**
- Review `ShapeCaster` component API and configuration
- Review `RayCaster` component API and configuration
- Understand `ShapeHits` and `RayHits` result format
- Determine component update timing (when are hits available?)
- Check how to configure: direction, max_distance, shape, collision layers
- Document the simple ECS pattern for the team

### Task 2: Design Component Architecture
**Owner:** Senior Engineer
**Status:** Blocked by Task 1
**Description:**
- Decide: Add caster components to character entity OR spawn child entities?
- Design marker components if needed (e.g., `GroundCaster`, `WallCasterLeft`, etc.)
- Plan how to update caster direction/origin each frame (based on gravity)
- Design query structure for reading hits
- Map hit data to `CollisionData` struct
- Write design doc with code examples

### Task 3: Implement Ground Detection with ECS
**Owner:** Implementation Agent
**Status:** Blocked by Task 2
**Description:**
- Add `ShapeCaster` component for ground detection
- Configure: horizontal segment, downward direction, max_distance
- Add system to update caster origin/direction based on character position and gravity
- Add system to query `ShapeHits` and populate `CharacterController.floor`
- Remove `avian_shapecast()` helper calls
- Remove `SpatialQuery` parameter

### Task 4: Implement Wall Detection with ECS
**Owner:** Implementation Agent
**Status:** Blocked by Task 2
**Description:**
- Add `ShapeCaster` components for left/right wall detection
- Configure: vertical segments, left/right directions, max_distance
- Add system to update caster origins/directions
- Add system to query `ShapeHits` and populate `CharacterController.left_wall`/`right_wall`
- Remove manual shapecasting code

### Task 5: Implement Ceiling Detection with ECS
**Owner:** Implementation Agent
**Status:** Blocked by Task 2
**Description:**
- Add `ShapeCaster` component for ceiling detection
- Configure: horizontal segment, upward direction, max_distance
- Add system to update caster origin/direction
- Add system to query `ShapeHits` and populate `CharacterController.ceiling`
- Remove manual shapecasting code

### Task 6: Implement Stair Detection with ECS
**Owner:** Implementation Agent
**Status:** Blocked by Task 2
**Description:**
- Add `ShapeCaster` components for stair detection (forward-down + current ground)
- Configure based on movement intent and position
- Add system to update caster configuration
- Add system to query `ShapeHits` and calculate step height
- Populate `CharacterController.step_detected` and `step_height`
- Remove `check_stair_step()` helper function

### Task 7: Remove All Helper Functions
**Owner:** Cleanup Agent
**Status:** Blocked by Tasks 3-6
**Description:**
- Remove `avian_shapecast()` helper (lines 262-315)
- Remove all `SpatialQuery` system parameters
- Clean up imports
- Verify pure ECS pattern throughout

### Task 8: Add Unit Tests
**Owner:** Test Agent
**Status:** Blocked by Tasks 3-6
**Description:**
- Test that caster components are properly configured
- Test hit results are correctly read
- Test collision layers are inherited
- Add to `src/avian.rs` test module

### Task 9: Run Integration Tests
**Owner:** Test Agent
**Status:** Blocked by Task 8
**Description:**
- Run `tests/avian2d.rs` - all tests must pass
- Run `tests/debug_avian.rs`
- Verify ground/wall/ceiling detection accuracy
- No regressions allowed

### Task 10: Manual Testing with Examples
**Owner:** QA Agent
**Status:** Blocked by Task 9
**Description:**
- `cargo run --example platform_box --features examples`
- `cargo run --example spherical_planet --features examples`
- `cargo run --example float_test --features examples`
- Verify all character controller behavior works correctly
- Document any issues

## Success Criteria

1. ✅ No manual shapecasting - all use `ShapeCaster`/`RayCaster` components
2. ✅ No `SpatialQuery` parameters - pure ECS queries for `ShapeHits`/`RayHits`
3. ✅ No helper functions - simple query patterns like shown in example
4. ✅ All tests pass
5. ✅ All examples work
6. ✅ Behavior identical to before
7. ✅ Code is dramatically simpler

## Reference Pattern

```rust
use avian2d::prelude::*;
use bevy::prelude::*;

fn print_hits(query: Query<&ShapeHits, With<ShapeCaster>>) {
    for hits in &query {
        for hit in hits.iter() {
            println!("Hit entity {} with distance {}", hit.entity, hit.distance);
        }
    }
}
```

## Notes

- Keep it simple - use ECS, not helpers
- Rapier backend (`src/rapier.rs`) unchanged - it correctly uses RapierContext
- This is a refactor, not a feature change
- Maintain exact behavior
