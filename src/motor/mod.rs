//! Motor module for stepper-motion.
//!
//! Provides the stepper motor driver with type-state safety and position tracking.
//!
//! When the `async` feature is enabled, this module also provides async motor
//! drivers using `embedded-hal-async` and `embassy-time`.

mod builder;
mod driver;
mod position;
pub mod state;
mod system;

#[cfg(feature = "async")]
mod async_builder;
#[cfg(feature = "async")]
mod async_driver;
#[cfg(feature = "async")]
mod async_system;

pub use builder::StepperMotorBuilder;
pub use driver::StepperMotor;
pub use position::Position;
pub use state::{Fault, Homing, Idle, MotorState, Moving, StateName};
pub use system::MotorSystem;

#[cfg(feature = "async")]
pub use async_builder::AsyncStepperMotorBuilder;
#[cfg(feature = "async")]
pub use async_driver::{AsyncMotorRunner, AsyncStepperMotor};
#[cfg(feature = "async")]
pub use async_system::AsyncMotorSystem;
