//! The frame flying intent is expressed in, for hosts whose "up" is sampled
//! rather than fixed.
//!
//! Flying intent is two scalars, not a world vector: `y` is thrust along the
//! actor's up axis and `x` thrust across it. That split only means something
//! if the producer of the intent and the consumer of it agree on which way is
//! up, and if there is an up at all. [`FlightFrame`] is that agreement, and
//! [`FlightOrientation`] is how a host supplies it per actor — see
//! [`FlightOrientation`] for the host contract.

use bevy::prelude::*;

/// The basis flying intent is expressed in: intent `x` along [`Self::right`],
/// intent `y` along [`Self::up`].
///
/// The basis is orthonormal by construction — `right` is always `up` turned a
/// quarter-turn clockwise, which is what lets [`project`](Self::project) and
/// [`to_world`](Self::to_world) round-trip without skew or scale. The only
/// ways to obtain a frame are [`Self::new`] (or the equivalent `From<Dir2>`)
/// and [`Self::UNORIENTED`], all of which uphold that invariant.
///
/// # In a null
///
/// The *meaning* of the vertical/horizontal split does not survive a gravity
/// null. With no gravity there is no altitude to hold and no reason for thrust
/// across the frame to behave differently from thrust along it, so
/// [`FlyingConfig::thrust`](crate::config::FlyingConfig::thrust) fades the two
/// axes' accelerations and speed caps together as confidence falls. At zero
/// they are equal, and the actor accelerates along its intent rather than in a
/// direction the frame skewed it into — which is what makes
/// [`Self::UNORIENTED`] at zero confidence the safe fallback for an actor that
/// has never seen gravity: the arbitrary basis cancels out of the
/// [`project`](Self::project)/[`to_world`](Self::to_world) round trip, and the
/// actor is never stranded waiting for orientation.
#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
pub struct FlightFrame {
    /// The direction `intent.y` thrusts along.
    up: Vec2,
    /// The direction `intent.x` thrusts along: `up` turned a quarter-turn clockwise.
    right: Vec2,
}

impl FlightFrame {
    /// The frame whose up axis is `up`.
    #[must_use]
    pub fn new(up: Dir2) -> Self {
        let up = up.as_vec2();
        Self {
            up,
            right: Vec2::new(up.y, -up.x),
        }
    }

    /// The frame to fly by where there is no up: world axes, chosen arbitrarily.
    ///
    /// Only ever paired with zero confidence, which is what makes the
    /// arbitrary choice harmless — thrust is isotropic there, so the basis
    /// cancels out of the round trip.
    pub const UNORIENTED: Self = Self {
        up: Vec2::Y,
        right: Vec2::X,
    };

    /// The direction `intent.y` thrusts along.
    #[must_use]
    pub fn up(&self) -> Vec2 {
        self.up
    }

    /// The direction `intent.x` thrusts along: [`Self::up`] turned a
    /// quarter-turn clockwise.
    #[must_use]
    pub fn right(&self) -> Vec2 {
        self.right
    }

    /// Projects a world-space direction onto this frame.
    #[must_use]
    pub fn project(&self, world_direction: Vec2) -> Vec2 {
        Vec2::new(
            world_direction.dot(self.right),
            world_direction.dot(self.up),
        )
    }

    /// Expands frame-space components back into a world-space vector.
    #[must_use]
    pub fn to_world(&self, framed: Vec2) -> Vec2 {
        self.right * framed.x + self.up * framed.y
    }
}

impl From<Dir2> for FlightFrame {
    /// The frame whose up axis is `up` — see [`FlightFrame::new`].
    fn from(up: Dir2) -> Self {
        Self::new(up)
    }
}

/// Host-written per-actor flight orientation: the frame flight intent is
/// expressed in, and how far it is to be trusted.
///
/// # Host contract
///
/// Insert this on actors whose up axis comes from a sampled source (a
/// spherical or multi-body gravity field, say) and **write it each physics
/// tick** from that sample, before the controller consumes it: schedule the
/// sampling system in `FixedUpdate`
/// `.before(`[`CharacterControllerSet::IntentApplication`](crate::CharacterControllerSet::IntentApplication)`)`,
/// the set [`apply_fly`](crate::systems::apply_fly) reads the component in.
/// The two fields are only sound together: a sampled up with its sampled
/// confidence, or — where the field has never resolved an up at all —
/// [`FlightFrame::UNORIENTED`] with the confidence forced to zero (which is
/// [`FlightOrientation::default`]). Refusing to fly without a reference
/// instead would strand every actor in true zero g, where the wait for
/// orientation is forever, not a moment.
///
/// Actors **without** this component keep the existing fixed-up behavior
/// untouched, so merely having this type in the crate changes nothing for
/// existing consumers. Adding the component, however, moves the actor onto a
/// different thrust law, not just a different basis: the fixed-up path is a
/// velocity servo toward intent-scaled target speeds, while oriented actors
/// are driven by [`FlyingConfig::thrust`](crate::config::FlyingConfig::thrust)
/// — per-axis acceleration up to each axis' speed cap, coasting at the limit.
#[derive(Component, Debug, Clone, Copy, PartialEq, Reflect)]
#[reflect(Component)]
pub struct FlightOrientation {
    /// The basis flight intent is expressed in.
    pub frame: FlightFrame,
    /// How far gravity is to be trusted within the frame, in `[0, 1]`.
    ///
    /// At `1.0` the authored vertical/horizontal split applies in full; as it
    /// falls to `0.0` thrust becomes isotropic and the frame stops mattering.
    pub confidence: f32,
}

impl Default for FlightOrientation {
    /// The safe fallback for an actor that has never seen gravity: an
    /// arbitrary basis at zero confidence, where thrust is isotropic and the
    /// arbitrary choice cancels out.
    fn default() -> Self {
        Self {
            frame: FlightFrame::UNORIENTED,
            confidence: 0.0,
        }
    }
}

impl FlightOrientation {
    /// An orientation with the given frame and confidence.
    #[must_use]
    pub fn new(frame: FlightFrame, confidence: f32) -> Self {
        Self { frame, confidence }
    }

    /// A fully-trusted fixed up axis: the authored vertical/horizontal split
    /// applies in full, in a frame that never moves.
    ///
    /// This fixes the *basis* only. The actor still flies by the frame path's
    /// thrust law
    /// ([`FlyingConfig::thrust`](crate::config::FlyingConfig::thrust)), which
    /// is not the same law as the component-less fixed-up path — see the
    /// [host contract](Self#host-contract).
    #[must_use]
    pub fn fixed(up: Dir2) -> Self {
        Self {
            frame: FlightFrame::new(up),
            confidence: 1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_unoriented_frame_is_an_ordinary_frame() {
        assert_eq!(FlightFrame::UNORIENTED, FlightFrame::new(Dir2::Y));
    }

    #[test]
    fn a_frame_from_a_direction_matches_new() {
        assert_eq!(FlightFrame::from(Dir2::X), FlightFrame::new(Dir2::X));
    }

    #[test]
    fn every_frame_keeps_its_basis_orthonormal() {
        let frame = FlightFrame::new(Dir2::from_xy(3.0, 4.0).unwrap());
        let quarter_turn = Vec2::new(frame.up().y, -frame.up().x);
        assert!(
            frame.right().abs_diff_eq(quarter_turn, 1.0e-6),
            "right must be up turned a quarter-turn clockwise, got {:?}",
            frame.right()
        );
        assert!(frame.up().dot(frame.right()).abs() < 1.0e-6);
    }

    #[test]
    fn project_and_to_world_round_trip() {
        let frame = FlightFrame::new(Dir2::from_xy(1.0, 1.0).unwrap());
        let world = Vec2::new(3.0, -2.0);
        let round_tripped = frame.to_world(frame.project(world));
        assert!(
            world.abs_diff_eq(round_tripped, 1.0e-5),
            "expected {world:?}, got {round_tripped:?}"
        );
    }

    #[test]
    fn default_orientation_is_unoriented_and_weightless() {
        let orientation = FlightOrientation::default();
        assert_eq!(orientation.frame, FlightFrame::UNORIENTED);
        assert_eq!(orientation.confidence, 0.0);
    }

    #[test]
    fn fixed_orientation_is_fully_trusted() {
        let orientation = FlightOrientation::fixed(Dir2::X);
        assert_eq!(orientation.frame, FlightFrame::new(Dir2::X));
        assert_eq!(orientation.confidence, 1.0);
    }
}
