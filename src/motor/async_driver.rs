//! Async stepper motor driver.
//!
//! Provides async motor control using embedded-hal-async traits and embassy-time
//! for non-blocking step timing. Compatible with both std and no_std environments.

use core::marker::PhantomData;

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs as AsyncDelayNs;
use embassy_time::{Duration, Timer};

use crate::config::units::{Degrees, Steps};
use crate::config::MechanicalConstraints;
use crate::error::{Error, MotorError, Result};
use crate::motion::{AsyncMotionExecutor, Direction, MotionPhase, MotionProfile};
use crate::motor::position::Position;
use crate::motor::state::{Idle, MotorState, Moving, StateName};

/// Async stepper motor driver with type-state safety.
///
/// This is the async version of `StepperMotor`, using `embedded-hal-async` traits
/// and `embassy-time` for non-blocking delays. It allows cooperative multitasking
/// while executing motion profiles.
///
/// # Generic Parameters
///
/// - `STEP`: STEP pin type (must implement `OutputPin`)
/// - `DIR`: DIR pin type (must implement `OutputPin`)
/// - `DELAY`: Async delay provider (must implement `embedded_hal_async::delay::DelayNs`)
/// - `STATE`: Type-state marker (defaults to `Idle`)
///
/// # Example
///
/// ```rust,ignore
/// use stepper_motion::motor::AsyncStepperMotor;
/// use embassy_time::Delay;
///
/// // Create async motor
/// let mut motor = AsyncStepperMotor::new(
///     step_pin,
///     dir_pin,
///     Delay,
///     constraints,
///     "x_axis".try_into().unwrap(),
///     false,
///     0,
/// );
///
/// // Move asynchronously - other tasks can run during delays
/// motor = motor.move_to_async(Degrees(90.0)).await?;
/// ```
pub struct AsyncStepperMotor<STEP, DIR, DELAY, STATE = Idle>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: AsyncDelayNs,
    STATE: MotorState,
{
    /// STEP pin (pulse to move one step).
    step_pin: STEP,

    /// DIR pin (high = CW, low = CCW, or inverted).
    dir_pin: DIR,

    /// Async delay provider.
    delay: DELAY,

    /// Current absolute position.
    position: Position,

    /// Current direction (cached to avoid unnecessary pin writes).
    current_direction: Option<Direction>,

    /// Mechanical constraints from configuration.
    constraints: MechanicalConstraints,

    /// Motor name for logging/debugging.
    name: heapless::String<32>,

    /// Whether direction pin logic is inverted.
    invert_direction: bool,

    /// Backlash compensation in steps (applied on direction change).
    backlash_steps: i64,

    /// Async motion executor for current move (if any).
    executor: Option<AsyncMotionExecutor>,

    /// Type-state marker.
    _state: PhantomData<STATE>,
}

impl<STEP, DIR, DELAY, STATE> AsyncStepperMotor<STEP, DIR, DELAY, STATE>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: AsyncDelayNs,
    STATE: MotorState + StateName,
{
    /// Get the motor name.
    #[inline]
    pub fn name(&self) -> &str {
        self.name.as_str()
    }

    /// Get current position in steps.
    #[inline]
    pub fn position_steps(&self) -> Steps {
        self.position.steps()
    }

    /// Get current position in degrees.
    #[inline]
    pub fn position_degrees(&self) -> Degrees {
        self.position.degrees()
    }

    /// Get the mechanical constraints.
    #[inline]
    pub fn constraints(&self) -> &MechanicalConstraints {
        &self.constraints
    }

    /// Get the current state name.
    #[inline]
    pub fn state_name(&self) -> &'static str {
        STATE::name()
    }
}

impl<STEP, DIR, DELAY> AsyncStepperMotor<STEP, DIR, DELAY, Idle>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: AsyncDelayNs,
{
    /// Create a new async motor in the Idle state.
    pub fn new(
        step_pin: STEP,
        dir_pin: DIR,
        delay: DELAY,
        constraints: MechanicalConstraints,
        name: heapless::String<32>,
        invert_direction: bool,
        backlash_steps: i64,
    ) -> Self {
        Self {
            step_pin,
            dir_pin,
            delay,
            position: Position::new(constraints.steps_per_degree),
            current_direction: None,
            constraints,
            name,
            invert_direction,
            backlash_steps,
            executor: None,
            _state: PhantomData,
        }
    }

    /// Start a move to an absolute position in degrees.
    ///
    /// Returns a motor in the `Moving` state.
    pub fn move_to(
        mut self,
        target: Degrees,
    ) -> core::result::Result<AsyncStepperMotor<STEP, DIR, DELAY, Moving>, (Self, Error)> {
        // Calculate steps to target
        let target_steps = Steps::from_degrees(target, self.constraints.steps_per_degree);
        let delta_steps = target_steps.0 - self.position.steps().0;

        if delta_steps == 0 {
            return Err((self, Error::Motion(crate::error::MotionError::MoveTooShort {
                steps: 0,
                minimum: 1,
            })));
        }

        // Check limits
        let limit_check = self.constraints.limits.as_ref().and_then(|limits| {
            if limits.apply(target_steps.0).is_none() {
                Some(if delta_steps > 0 {
                    limits.max_steps
                } else {
                    limits.min_steps
                })
            } else {
                None
            }
        });

        if let Some(limit) = limit_check {
            return Err((
                self,
                Error::Motor(MotorError::LimitExceeded {
                    position: target_steps.0,
                    limit,
                }),
            ));
        }

        // Create motion profile
        let profile = MotionProfile::symmetric_trapezoidal(
            delta_steps,
            self.constraints.max_velocity_steps_per_sec,
            self.constraints.max_acceleration_steps_per_sec2,
        );

        // Set direction
        let direction = profile.direction;
        if self.set_direction(direction).is_err() {
            return Err((self, Error::Motor(MotorError::PinError)));
        }

        // Create async executor
        let executor = AsyncMotionExecutor::new(profile);

        // Transition to Moving state
        Ok(AsyncStepperMotor {
            step_pin: self.step_pin,
            dir_pin: self.dir_pin,
            delay: self.delay,
            position: self.position,
            current_direction: self.current_direction,
            constraints: self.constraints,
            name: self.name,
            invert_direction: self.invert_direction,
            backlash_steps: self.backlash_steps,
            executor: Some(executor),
            _state: PhantomData,
        })
    }

    /// Move by a relative amount in degrees.
    pub fn move_by(
        self,
        delta: Degrees,
    ) -> core::result::Result<AsyncStepperMotor<STEP, DIR, DELAY, Moving>, (Self, Error)> {
        let target = Degrees(self.position.degrees().0 + delta.0);
        self.move_to(target)
    }

    /// Set the current position as the origin (zero).
    pub fn set_origin(&mut self) {
        self.position.set_origin();
    }

    /// Set the current position to a specific value.
    pub fn set_position(&mut self, degrees: Degrees) {
        self.position.set_degrees(degrees);
    }

    /// Execute a named trajectory from a registry asynchronously.
    ///
    /// This method looks up the trajectory by name, validates it against
    /// the motor's constraints, and executes it to completion using async delays.
    pub async fn execute_async(
        self,
        trajectory_name: &str,
        registry: &crate::trajectory::TrajectoryRegistry,
    ) -> core::result::Result<Self, (Self, Error)> {
        // Look up trajectory
        let trajectory = match registry.get(trajectory_name) {
            Some(t) => t,
            None => {
                let mut msg: heapless::String<64> = heapless::String::new();
                let _ = msg.push_str("trajectory '");
                let _ = msg.push_str(trajectory_name);
                let _ = msg.push_str("' not found");
                return Err((
                    self,
                    Error::Trajectory(crate::error::TrajectoryError::InvalidName(msg)),
                ));
            }
        };

        // Verify this trajectory is for this motor
        if trajectory.motor.as_str() != self.name.as_str() {
            let mut msg: heapless::String<64> = heapless::String::new();
            let _ = msg.push_str("trajectory '");
            let _ = msg.push_str(trajectory_name);
            let _ = msg.push_str("' is for motor '");
            let _ = msg.push_str(trajectory.motor.as_str());
            let _ = msg.push_str("'");
            return Err((
                self,
                Error::Trajectory(crate::error::TrajectoryError::InvalidName(msg)),
            ));
        }

        // Execute the move to the target position
        let target = trajectory.target_degrees;
        self.move_to_async(target).await
    }

    /// Move to an absolute position and run to completion asynchronously.
    ///
    /// This is the async equivalent of `move_to_blocking`. Other tasks can
    /// run during the step delays.
    pub async fn move_to_async(
        self,
        target: Degrees,
    ) -> core::result::Result<Self, (Self, Error)> {
        match self.move_to(target) {
            Ok(moving) => {
                match moving.run_to_completion_async().await {
                    Ok(idle) => Ok(idle),
                    Err(e) => {
                        panic!("Motor step error during async move: {:?}", e);
                    }
                }
            }
            Err(e) => Err(e),
        }
    }

    fn set_direction(&mut self, direction: Direction) -> core::result::Result<(), ()> {
        if self.current_direction == Some(direction) {
            return Ok(());
        }

        let pin_high = match direction {
            Direction::Clockwise => !self.invert_direction,
            Direction::CounterClockwise => self.invert_direction,
        };

        if pin_high {
            self.dir_pin.set_high().map_err(|_| ())?;
        } else {
            self.dir_pin.set_low().map_err(|_| ())?;
        }

        self.current_direction = Some(direction);
        Ok(())
    }
}

impl<STEP, DIR, DELAY> AsyncStepperMotor<STEP, DIR, DELAY, Moving>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: AsyncDelayNs,
{
    /// Execute one step pulse asynchronously.
    ///
    /// Returns `true` if the move is complete.
    pub async fn step_async(&mut self) -> Result<bool> {
        let executor = self.executor.as_mut().ok_or(MotorError::NotInitialized)?;

        if executor.is_complete() {
            return Ok(true);
        }

        // Generate step pulse
        self.step_pin.set_high().map_err(|_| MotorError::PinError)?;

        // Pulse width using async delay (typically 1-10 microseconds is sufficient)
        Timer::after(Duration::from_micros(2)).await;

        self.step_pin.set_low().map_err(|_| MotorError::PinError)?;

        // Update position
        let direction = executor.profile().direction;
        self.position.move_steps(direction.sign());

        // Get delay for next step and wait asynchronously
        let interval_ns = executor.current_interval_ns();

        // Advance executor
        let has_more = executor.advance();

        if has_more {
            // Async delay until next step (subtract pulse width)
            let delay_ns = interval_ns.saturating_sub(2000);
            if delay_ns > 0 {
                Timer::after(Duration::from_nanos(delay_ns as u64)).await;
            }
        }

        Ok(!has_more)
    }

    /// Check if the move is complete.
    #[inline]
    pub fn is_complete(&self) -> bool {
        self.executor
            .as_ref()
            .map(|e| e.is_complete())
            .unwrap_or(true)
    }

    /// Get move progress (0.0 to 1.0).
    #[inline]
    pub fn progress(&self) -> f32 {
        self.executor.as_ref().map(|e| e.progress()).unwrap_or(1.0)
    }

    /// Get current motion phase.
    #[inline]
    pub fn phase(&self) -> MotionPhase {
        self.executor
            .as_ref()
            .map(|e| e.phase())
            .unwrap_or(MotionPhase::Complete)
    }

    /// Complete the move and return to Idle state.
    ///
    /// This should be called after `is_complete()` returns true or
    /// to abandon a move in progress.
    pub fn finish(self) -> AsyncStepperMotor<STEP, DIR, DELAY, Idle> {
        AsyncStepperMotor {
            step_pin: self.step_pin,
            dir_pin: self.dir_pin,
            delay: self.delay,
            position: self.position,
            current_direction: self.current_direction,
            constraints: self.constraints,
            name: self.name,
            invert_direction: self.invert_direction,
            backlash_steps: self.backlash_steps,
            executor: None,
            _state: PhantomData,
        }
    }

    /// Run the move to completion asynchronously.
    ///
    /// This is the async equivalent of `run_to_completion`. Other tasks
    /// can run during the step delays, enabling cooperative multitasking.
    pub async fn run_to_completion_async(mut self) -> Result<AsyncStepperMotor<STEP, DIR, DELAY, Idle>> {
        while !self.is_complete() {
            self.step_async().await?;
        }
        Ok(self.finish())
    }
}

/// Async motor executor that wraps an async motor reference.
///
/// This provides a way to run async motor operations without consuming the motor.
pub struct AsyncMotorRunner<'a, STEP, DIR, DELAY>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: AsyncDelayNs,
{
    motor: &'a mut AsyncStepperMotor<STEP, DIR, DELAY, Moving>,
}

impl<'a, STEP, DIR, DELAY> AsyncMotorRunner<'a, STEP, DIR, DELAY>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: AsyncDelayNs,
{
    /// Create a new async runner for a moving motor.
    pub fn new(motor: &'a mut AsyncStepperMotor<STEP, DIR, DELAY, Moving>) -> Self {
        Self { motor }
    }

    /// Execute one step asynchronously.
    pub async fn step(&mut self) -> Result<bool> {
        self.motor.step_async().await
    }

    /// Check if motion is complete.
    pub fn is_complete(&self) -> bool {
        self.motor.is_complete()
    }

    /// Get progress (0.0 to 1.0).
    pub fn progress(&self) -> f32 {
        self.motor.progress()
    }
}

#[cfg(test)]
mod tests {
    // Tests require async runtime and embedded-hal-mock with async support
}
