---
active: true
iteration: 1
max_iterations: 30
completion_promise: "DONE"
started_at: "2026-01-31T19:09:47Z"
---

- Publish a branch bevy-0.17 to keep an old version available - Make a new branch for migration - Migrate the crate to Bevy 0.18 - Bump the version number to v0.3.0 - Fully run cargo test, cargo check all examples all targets, same for clippy, run bevy_lint all targets all features - Completely clean up the project CI including pedantic clippy - Set the project clippy to use these exceptions: cargo clippy --all-targets --all-features -- -W clippy::pedantic -A clippy::needless_pass_by_value -A clippy::too_many_arguments -A clippy::type_complexity - Write integration and unit tests where they are missing, particularly where migration required breaking changes - The tests and all examples have to run through the full cleanup pipeline - You are on windows with WSL, avoid firing stop hooks, just launch directly into follow-up iterations
