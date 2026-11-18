# Benchmark baselines

Criterion baseline `base`, captured 2026-08-26.

| | |
|---|---|
| Commit | `17be46ccc37c` |
| Branch | `claude/flight-frame-and-serde` |
| Toolchain | rustc 1.98.0 (88d9e12ae 2026-08-18) |
| Host | Linux x86_64 container (shared/virtualised) |

## Results

Mean with 95% confidence interval.

| Benchmark | Mean | 95% CI |
|---|---:|---|
| `flight_frame_project_to_world_round_trip` | 1.472 ns | 1.458 ns – 1.484 ns |
| `flying_config_thrust` | 11.6 ns | 11.58 ns – 11.63 ns |

## Reproducing

The bench target must be named. A bare `cargo bench` also runs the lib test
harness, which rejects criterion's flags with
`error: Unrecognized option: 'baseline'`.

```sh
# capture
cargo bench --bench flight -- --save-baseline base

# compare against it
cargo bench --bench flight -- --baseline base
```

## How much to trust these

Taken in a shared virtualised container. Treat them as an order-of-magnitude
record, not a regression gate.

Re-running `take_batch/3x1000_pending` on byte-identical code about an hour
later on the *same* host reported `+31%` with `p = 0.00`. Criterion's
significance test measures sampling noise within a run; it cannot see the
host's load drifting between runs. So a reported change of this size here is
not evidence of a real regression.

To draw a conclusion from a comparison, capture the baseline and the
comparison back to back in one sitting, and treat anything under roughly
1.5x as inconclusive. Comparing these absolute numbers against a different
machine is meaningless.

## `sensors` bench: backend-neutral sensor pipeline

Captured 2026-11-17 on branch `feat/backend-neutral-sensors`, rustc 1.97.0, same
kind of shared container. `before` is `main` at `fab735d` with only the bench
added; `after` is the branch. The two binaries ran back to back, alternating,
twice. Mean of a full `app.update()` (one fixed step, Avian included):

| Round | Characters | before | after | criterion change |
|---|---:|---:|---:|---|
| 1 | 64 | 2.160 ms | 1.965 ms | −2.3 % (p = 0.19, no change) |
| 1 | 512 | 7.983 ms | 7.438 ms | −6.8 % (p = 0.00) |
| 2 | 64 | 1.838 ms | 2.054 ms | +12.7 % (p = 0.00) |
| 2 | 512 | 7.360 ms | 7.393 ms | +0.4 % (p = 0.77, no change) |

The sign flips between rounds; read it as no measurable difference. The Avian
backend still runs its casts on persistent `ShapeCaster`s inside the physics
step; what changed is that requests and detection run in exclusive systems.

```sh
cargo bench --bench sensors -- --save-baseline base
cargo bench --bench sensors -- --baseline base
```
