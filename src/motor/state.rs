//! Motor state type-state markers.
//!
//! Uses Rust's type system to enforce valid state transitions at compile time.

/// Motor is idle and ready for commands.
#[derive(Debug, Clone, Copy, Default)]
pub struct Idle;

/// Motor is currently executing a move.
#[derive(Debug, Clone, Copy)]
pub struct Moving;

/// Motor is executing a homing sequence.
#[derive(Debug, Clone, Copy)]
pub struct Homing;

/// Motor encountered an error and needs recovery.
#[derive(Debug, Clone, Copy)]
pub struct Fault;

/// Trait for motor states.
pub trait MotorState: private::Sealed {}

impl MotorState for Idle {}
impl MotorState for Moving {}
impl MotorState for Homing {}
impl MotorState for Fault {}

mod private {
    pub trait Sealed {}
    impl Sealed for super::Idle {}
    impl Sealed for super::Moving {}
    impl Sealed for super::Homing {}
    impl Sealed for super::Fault {}
}

/// State name for display/debugging.
pub trait StateName {
    /// Get the state name as a static string.
    fn name() -> &'static str;
}

impl StateName for Idle {
    fn name() -> &'static str {
        "Idle"
    }
}

impl StateName for Moving {
    fn name() -> &'static str {
        "Moving"
    }
}

impl StateName for Homing {
    fn name() -> &'static str {
        "Homing"
    }
}

impl StateName for Fault {
    fn name() -> &'static str {
        "Fault"
    }
}
