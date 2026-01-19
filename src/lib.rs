//! # stepper-motion
//!
//! Configuration-driven stepper motor motion control with embedded-hal 1.0 support.
//!
//! ## Features
//!
//! - **Configuration-driven**: Define motors and trajectories in TOML files
//! - **embedded-hal 1.0**: Uses `OutputPin` for STEP/DIR, `DelayNs` for timing
//! - **no_std compatible**: Core library works without standard library
//! - **Asymmetric profiles**: Independent acceleration and deceleration rates
//! - **Position tracking**: Absolute position tracked at all times
//! - **Type-state safety**: Compile-time motor state verification
//! - **Async support**: Optional async motor control using `embassy-time` (compatible with std and no_std)
//!
//! ## Quick Start
//!
//! ```rust,ignore
//! use stepper_motion::{StepperMotor, SystemConfig};
//!
//! // Load configuration from TOML
//! let config: SystemConfig = stepper_motion::load_config("motion.toml")?;
//!
//! // Create motor with embedded-hal pins
//! let mut motor = StepperMotor::builder()
//!     .from_config(&config, "x_axis")?
//!     .step_pin(step_pin)
//!     .dir_pin(dir_pin)
//!     .delay(delay)
//!     .build()?;
//!
//! // Execute named trajectory
//! motor.execute("home")?;
//! ```
//!
//! ## Async Usage (requires `async` feature)
//!
//! ```rust,ignore
//! use stepper_motion::{AsyncStepperMotor, SystemConfig};
//! use embassy_time::Delay;
//!
//! // Create async motor with embassy-time delay
//! let mut motor = AsyncStepperMotor::builder()
//!     .from_config(&config, "x_axis")?
//!     .step_pin(step_pin)
//!     .dir_pin(dir_pin)
//!     .delay(Delay)
//!     .build()?;
//!
//! // Execute trajectory asynchronously - other tasks can run during delays
//! motor = motor.execute_async("home", &registry).await?;
//! ```
//!
//! ## Feature Flags
//!
//! - `std` (default): Enables file I/O and TOML parsing
//! - `alloc`: Enables heap allocation for no_std with allocator
//! - `defmt`: Enables defmt logging for embedded targets
//! - `async`: Enables async motor control using `embedded-hal-async` and `embassy-time`

#![cfg_attr(not(feature = "std"), no_std)]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![deny(unsafe_code)]
// Allow large error types - necessary for no_std with heapless strings
#![allow(clippy::result_large_err)]

#[cfg(feature = "alloc")]
extern crate alloc;

// Core modules
pub mod config;
pub mod error;
pub mod motion;
pub mod motor;
pub mod trajectory;

// Re-exports for ergonomic API
pub use config::{MotorConfig, SystemConfig, TrajectoryConfig, validate_config};
pub use config::{HomingConfig, HomingDirection, HomingPhase, HomingStrategy};
pub use config::{SwitchConfig, SwitchPolarity, SwitchesConfig};
pub use error::{Error, HomingError, Result};
pub use motion::{Direction, MotionPhase, MotionProfile};
pub use motor::{state, MotorSystem, StepperMotor};
pub use motor::{execute_homing_blocking, HomingExecutor, HomingSwitches};
pub use trajectory::TrajectoryRegistry;

// Configuration loading (std only)
#[cfg(feature = "std")]
pub use config::load_config;

// Unit types
pub use config::units::{Degrees, DegreesPerSec, DegreesPerSecSquared, Microsteps, Steps};

// Async re-exports (async feature only)
#[cfg(feature = "async")]
pub use motion::{AsyncMotionExecutor, async_delay_ns, async_delay_us};
#[cfg(feature = "async")]
pub use motor::{AsyncMotorRunner, AsyncMotorSystem, AsyncStepperMotor, AsyncStepperMotorBuilder};
#[cfg(feature = "async")]
pub use motor::execute_homing_async;
