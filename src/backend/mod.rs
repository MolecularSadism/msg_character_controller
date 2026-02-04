mod traits;
mod noop;

#[cfg(feature = "avian2d")]
pub mod avian;

pub use traits::CharacterPhysicsBackend;
pub use noop::NoOpBackendPlugin;

#[cfg(feature = "avian2d")]
pub use avian::Avian2dBackend;