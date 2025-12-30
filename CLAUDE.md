# Project Notes for Claude

## Rapier Physics Integration

**IMPORTANT**: Rapier runs in `PostFixedUpdate`, NOT in `FixedUpdate`.

Do NOT order CharacterControllerSet relative to Rapier's PhysicsSet. The character controller systems run in FixedUpdate, and Rapier processes ExternalForce/ExternalImpulse in PostFixedUpdate after FixedUpdate completes.

The correct flow is:
1. FixedUpdate: Character controller systems accumulate forces into ExternalForce/ExternalImpulse
2. PostFixedUpdate: Rapier reads ExternalForce/ExternalImpulse and simulates physics
