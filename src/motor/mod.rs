//! Motor module for stepper-motion.
//!
//! Provides the stepper motor driver with type-state safety and position tracking.

mod builder;
mod driver;
mod position;
pub mod state;
mod system;

pub use builder::StepperMotorBuilder;
pub use driver::StepperMotor;
pub use position::Position;
pub use state::{Fault, Homing, Idle, MotorState, Moving, StateName};
pub use system::MotorSystem;
