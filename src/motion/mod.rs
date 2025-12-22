//! Motion module for stepper-motion.
//!
//! Provides motion profile calculation and step execution.
//!
//! When the `async` feature is enabled, this module also provides async motion
//! execution using `embassy-time` for non-blocking delays.

mod executor;
mod profile;

#[cfg(feature = "async")]
mod async_executor;

pub use executor::MotionExecutor;
pub use profile::{Direction, MotionPhase, MotionProfile};

#[cfg(feature = "async")]
pub use async_executor::{AsyncMotionExecutor, async_delay_ns, async_delay_us};
