//! Movement intent components.
//!
//! Intents represent the desired movement direction from player input or AI.
//! The controller systems read these intents and apply appropriate physics.

use bevy::prelude::*;

/// Unified movement intent for walking and flying.
///
/// This component combines horizontal walking and vertical propulsion into
/// a single intent. The controller systems use this to apply appropriate
/// movement physics based on character state.
///
/// # Example
///
/// ```rust
/// use msg_character_controller::prelude::*;
///
/// // Create intent moving right
/// let mut intent = MovementIntent::new();
/// intent.set_walk(1.0);
/// assert!(intent.is_walking());
///
/// // Add upward propulsion
/// intent.set_fly(1.0);
/// assert!(intent.is_flying_up());
///
/// // Clear everything
/// intent.clear();
/// assert!(!intent.is_walking());
/// assert!(!intent.is_flying());
/// ```
#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct MovementIntent {
    /// Horizontal movement intent (-1.0 = left, 1.0 = right).
    pub walk: f32,
    /// Vertical propulsion intent (-1.0 = down, 1.0 = up).
    pub fly: f32,
    /// Speed multiplier for walking (0.0 to 1.0).
    pub walk_speed: f32,
    /// Speed multiplier for flying (0.0 to 1.0).
    pub fly_speed: f32,
}

impl Default for MovementIntent {
    fn default() -> Self {
        Self {
            walk: 0.0,
            fly: 0.0,
            walk_speed: 1.0,
            fly_speed: 1.0,
        }
    }
}

impl MovementIntent {
    /// Create a new empty movement intent.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the walking direction (-1.0 = left, 1.0 = right).
    pub fn set_walk(&mut self, direction: f32) {
        self.walk = direction.clamp(-1.0, 1.0);
    }

    /// Set the flying direction (-1.0 = down, 1.0 = up).
    pub fn set_fly(&mut self, direction: f32) {
        self.fly = direction.clamp(-1.0, 1.0);
    }

    /// Set the walk speed multiplier (0.0 to 1.0).
    pub fn set_walk_speed(&mut self, multiplier: f32) {
        self.walk_speed = multiplier.clamp(0.0, 1.0);
    }

    /// Set the fly speed multiplier (0.0 to 1.0).
    pub fn set_fly_speed(&mut self, multiplier: f32) {
        self.fly_speed = multiplier.clamp(0.0, 1.0);
    }

    /// Clear all movement intents.
    pub fn clear(&mut self) {
        self.walk = 0.0;
        self.fly = 0.0;
    }

    /// Clear only walking intent.
    pub fn clear_walk(&mut self) {
        self.walk = 0.0;
    }

    /// Clear only flying intent.
    pub fn clear_fly(&mut self) {
        self.fly = 0.0;
    }

    /// Check if there is active walking input.
    pub fn is_walking(&self) -> bool {
        self.walk.abs() > 0.001
    }

    /// Check if there is active flying input.
    pub fn is_flying(&self) -> bool {
        self.fly.abs() > 0.001
    }

    /// Check if flying upward.
    pub fn is_flying_up(&self) -> bool {
        self.fly > 0.001
    }

    /// Check if flying downward.
    pub fn is_flying_down(&self) -> bool {
        self.fly < -0.001
    }

    /// Get the effective walking direction with speed multiplier applied.
    pub fn effective_walk(&self) -> f32 {
        self.walk * self.walk_speed
    }

    /// Get the effective flying direction with speed multiplier applied.
    pub fn effective_fly(&self) -> f32 {
        self.fly * self.fly_speed
    }
}

// === Legacy type aliases for backwards compatibility ===

/// Walking movement intent (1D horizontal axis).
///
/// **Deprecated**: Use `MovementIntent` instead.
#[deprecated(since = "0.2.0", note = "Use MovementIntent instead")]
pub type WalkIntent = MovementIntent;

/// Vertical propulsion intent.
///
/// **Deprecated**: Use `MovementIntent` instead.
#[deprecated(since = "0.2.0", note = "Use MovementIntent instead")]
pub type PropulsionIntent = MovementIntent;

/// Jump request component.
///
/// Add this component to request a jump. The controller will consume
/// this request and attempt to execute a jump if conditions allow
/// (grounded, within coyote time, etc.).
#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component)]
pub struct JumpRequest {
    /// Whether a jump is currently requested.
    pub requested: bool,
    /// Time the request was made (for jump buffering).
    pub request_time: f32,
    /// Whether this request has been consumed.
    pub consumed: bool,
}

impl JumpRequest {
    /// Request a jump.
    pub fn request(&mut self, current_time: f32) {
        if !self.requested {
            self.requested = true;
            self.request_time = current_time;
            self.consumed = false;
        }
    }

    /// Check if the request is valid (not consumed and within buffer time).
    pub fn is_valid(&self, current_time: f32, buffer_time: f32) -> bool {
        self.requested && !self.consumed && (current_time - self.request_time) < buffer_time
    }

    /// Consume the jump request.
    pub fn consume(&mut self) {
        self.consumed = true;
    }

    /// Reset the request.
    pub fn reset(&mut self) {
        self.requested = false;
        self.consumed = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==================== MovementIntent Tests ====================

    #[test]
    fn movement_intent_new() {
        let intent = MovementIntent::new();
        assert_eq!(intent.walk, 0.0);
        assert_eq!(intent.fly, 0.0);
        assert_eq!(intent.walk_speed, 1.0);
        assert_eq!(intent.fly_speed, 1.0);
    }

    #[test]
    fn movement_intent_set_walk() {
        let mut intent = MovementIntent::new();
        intent.set_walk(0.5);
        assert_eq!(intent.walk, 0.5);

        // Clamps to valid range
        intent.set_walk(5.0);
        assert_eq!(intent.walk, 1.0);

        intent.set_walk(-5.0);
        assert_eq!(intent.walk, -1.0);
    }

    #[test]
    fn movement_intent_set_fly() {
        let mut intent = MovementIntent::new();
        intent.set_fly(0.8);
        assert_eq!(intent.fly, 0.8);

        intent.set_fly(-0.6);
        assert_eq!(intent.fly, -0.6);
    }

    #[test]
    fn movement_intent_speed_multipliers() {
        let mut intent = MovementIntent::new();
        intent.set_walk(1.0);
        intent.set_walk_speed(0.5);
        assert_eq!(intent.effective_walk(), 0.5);

        intent.set_fly(-1.0);
        intent.set_fly_speed(0.5);
        assert_eq!(intent.effective_fly(), -0.5);

        // Clamps to valid range
        intent.set_walk_speed(2.0);
        assert_eq!(intent.walk_speed, 1.0);

        intent.set_fly_speed(-1.0);
        assert_eq!(intent.fly_speed, 0.0);
    }

    #[test]
    fn movement_intent_is_walking() {
        let mut intent = MovementIntent::new();
        assert!(!intent.is_walking());

        intent.set_walk(0.5);
        assert!(intent.is_walking());

        intent.set_walk(0.0001); // Below threshold
        assert!(!intent.is_walking());
    }

    #[test]
    fn movement_intent_is_flying() {
        let mut intent = MovementIntent::new();
        assert!(!intent.is_flying());
        assert!(!intent.is_flying_up());
        assert!(!intent.is_flying_down());

        intent.set_fly(0.5);
        assert!(intent.is_flying());
        assert!(intent.is_flying_up());
        assert!(!intent.is_flying_down());

        intent.set_fly(-0.5);
        assert!(intent.is_flying());
        assert!(!intent.is_flying_up());
        assert!(intent.is_flying_down());
    }

    #[test]
    fn movement_intent_clear() {
        let mut intent = MovementIntent::new();
        intent.set_walk(1.0);
        intent.set_fly(1.0);

        intent.clear();
        assert!(!intent.is_walking());
        assert!(!intent.is_flying());
        assert_eq!(intent.walk, 0.0);
        assert_eq!(intent.fly, 0.0);
    }

    #[test]
    fn movement_intent_clear_walk() {
        let mut intent = MovementIntent::new();
        intent.set_walk(1.0);
        intent.set_fly(1.0);

        intent.clear_walk();
        assert!(!intent.is_walking());
        assert!(intent.is_flying());
    }

    #[test]
    fn movement_intent_clear_fly() {
        let mut intent = MovementIntent::new();
        intent.set_walk(1.0);
        intent.set_fly(1.0);

        intent.clear_fly();
        assert!(intent.is_walking());
        assert!(!intent.is_flying());
    }

    // ==================== JumpRequest Tests ====================

    #[test]
    fn jump_request_default() {
        let request = JumpRequest::default();
        assert!(!request.requested);
        assert!(!request.consumed);
    }

    #[test]
    fn jump_request_request() {
        let mut request = JumpRequest::default();
        request.request(1.0);

        assert!(request.requested);
        assert!(!request.consumed);
        assert_eq!(request.request_time, 1.0);
    }

    #[test]
    fn jump_request_only_requests_once() {
        let mut request = JumpRequest::default();
        request.request(1.0);
        request.request(2.0); // Should not update time

        assert_eq!(request.request_time, 1.0);
    }

    #[test]
    fn jump_request_is_valid() {
        let mut request = JumpRequest::default();
        request.request(1.0);

        // Within buffer time
        assert!(request.is_valid(1.05, 0.1));

        // Outside buffer time
        assert!(!request.is_valid(1.2, 0.1));
    }

    #[test]
    fn jump_request_consume() {
        let mut request = JumpRequest::default();
        request.request(1.0);
        assert!(request.is_valid(1.0, 0.1));

        request.consume();
        assert!(!request.is_valid(1.0, 0.1));
    }

    #[test]
    fn jump_request_reset() {
        let mut request = JumpRequest::default();
        request.request(1.0);
        request.consume();

        request.reset();
        assert!(!request.requested);
        assert!(!request.consumed);
    }

    #[test]
    fn jump_request_buffer_edge_case() {
        let mut request = JumpRequest::default();
        request.request(1.0);

        // Exactly at buffer time boundary
        assert!(!request.is_valid(1.1, 0.1));
    }
}
