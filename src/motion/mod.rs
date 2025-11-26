//! Motion module for stepper-motion.
//!
//! Provides motion profile calculation and step execution.

mod executor;
mod profile;

pub use executor::MotionExecutor;
pub use profile::{Direction, MotionPhase, MotionProfile};
