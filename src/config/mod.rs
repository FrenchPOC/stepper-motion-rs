//! Configuration module for stepper-motion.
//!
//! Provides types for loading and validating motor and trajectory configurations
//! from TOML files (with `std` feature) or pre-parsed data.

mod limits;
mod mechanical;
mod motor;
mod system;
mod trajectory;
pub mod units;
#[cfg(feature = "std")]
mod loader;
mod validation;

pub use limits::{LimitPolicy, SoftLimits, StepLimits};
pub use mechanical::MechanicalConstraints;
pub use motor::MotorConfig;
pub use system::SystemConfig;
pub use trajectory::{TrajectoryConfig, WaypointTrajectory};
pub use validation::validate_config;

#[cfg(feature = "std")]
pub use loader::load_config;

// Re-export unit types at config level
pub use units::{Degrees, DegreesPerSec, DegreesPerSecSquared, Microsteps, Steps};
