# Benchmark baselines

Criterion baseline `base`, captured 2026-08-26.

| | |
|---|---|
| Commit | `361dbb3ffa1c` |
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

```sh
cargo bench -- --save-baseline base   # capture
cargo bench -- --baseline base        # compare against it
```

These were taken in a shared virtualised container, so absolute figures carry
more run-to-run noise than a dedicated machine. Comparisons made with
`--baseline base` on the same host are meaningful; comparing these absolute
numbers against a different machine is not.
