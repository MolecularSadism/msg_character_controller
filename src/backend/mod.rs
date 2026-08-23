mod noop;
mod traits;

#[cfg(feature = "avian2d")]
pub mod avian;

pub use noop::NoOpBackendPlugin;
pub use traits::CharacterPhysicsBackend;

#[cfg(feature = "avian2d")]
pub use avian::Avian2dBackend;
