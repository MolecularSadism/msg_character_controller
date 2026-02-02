# Bevy 0.18 Migration Notes

## Status: In Progress (Iteration 1)

### Completed
- ✅ Created `bevy-0.17` branch to preserve old version
- ✅ Created `bevy-0.18-migration` branch for migration work
- ✅ Updated `Cargo.toml` dependencies:
  - Bevy: 0.17 → 0.18
  - avian2d: 0.4 → 0.5
  - bevy_egui: 0.37 → 0.39
- ✅ Temporarily disabled Rapier2D backend (awaiting Bevy 0.18 support)
- ✅ Code compiles with warnings only (no errors)
- ✅ Added GlobalTransform to test entity spawning

### Known Issues

#### Avian2d 0.5 Spatial Queries in Tests
**Status**: BLOCKING - All Avian2d tests failing

**Symptoms**:
- Spatial queries (raycasts/shapecasts) return no hits in test environment
- Ground detection always returns f32::MAX distance
- Both component-based and SpatialQuery system parameter approaches fail
- Issue reproduces in minimal test case

**Investigation**:
- Entities have correct components (Transform, GlobalTransform, Collider, RigidBody)
- Running 10+ frames doesn't help
- PhysicsPlugins configured identically to working 0.17 setup
- Avian 0.5 claims to be "just a Bevy 0.18 update with no other breaking changes"
- Tried: simplified tick(), adding/removing RigidBody, explicit CollisionLayers, avoiding layer bit 0
- Docs confirm only Collider required for detection, but spatial queries return no hits
- **Suspect**: May be test environment issue, schedule ordering, or Avian 0.5 bug

**Test Failures (8/17)**:
- `collision_layers::explicit_layer_0_matches_default`
- `collision_layers::sensors_inherit_collision_layers`
- `coyote_time::time_since_grounded_zero_when_grounded`
- `ground_detection::character_above_ground_detects_ground`
- `ground_detection::character_at_float_height_is_grounded`
- `movement::jump_changes_velocity`
- `wall_detection::detects_wall_on_left`
- `wall_detection::detects_wall_on_right`

**Next Steps**:
1. Check Avian's own test suite for 0.5
2. Search for Bevy 0.18 testing pattern changes
3. Consider reaching out to Avian maintainers
4. May need to wait for Avian 0.5.1 bug fix

### TODO
- ⏳ Fix Avian2d spatial query issue in tests
- ⏳ Bump version to 0.3.0
- ⏳ Run full test suite (cargo test)
- ⏳ Run cargo check on all examples
- ⏳ Configure pedantic clippy
- ⏳ Run clippy on all targets
- ⏳ Run bevy_lint
- ⏳ Write missing tests
- ⏳ Clean up CI configuration
- ⏳ Verify all examples run

### Rapier2D Status
- Feature temporarily disabled (commented out in Cargo.toml)
- Waiting for bevy_rapier2d 0.33+ with Bevy 0.18 support
- Will re-enable when available

## Build Commands

```bash
# Check compilation (currently works with warnings)
cargo check --all-targets --all-features

# Run tests (currently 8 failures in avian2d tests)
cargo test --all-targets --all-features

# Run examples (not yet tested)
cargo run --example platform_box --features examples
```
