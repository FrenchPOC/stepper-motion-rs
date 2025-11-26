//! Stepper motor driver.
//!
//! Generic over embedded-hal 1.0 pin types with type-state safety.

use core::marker::PhantomData;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

use crate::config::units::{Degrees, Steps};
use crate::config::MechanicalConstraints;
use crate::error::{Error, MotorError, Result};
use crate::motion::{Direction, MotionExecutor, MotionPhase, MotionProfile};

use super::position::Position;
use super::state::{Idle, MotorState, Moving, StateName};

/// Stepper motor driver with type-state safety.
///
/// Generic over:
/// - `STEP`: STEP pin type (must implement `OutputPin`)
/// - `DIR`: DIR pin type (must implement `OutputPin`)
/// - `DELAY`: Delay provider (must implement `DelayNs`)
/// - `STATE`: Type-state marker (defaults to `Idle`)
pub struct StepperMotor<STEP, DIR, DELAY, STATE = Idle>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: DelayNs,
    STATE: MotorState,
{
    /// STEP pin (pulse to move one step).
    step_pin: STEP,

    /// DIR pin (high = CW, low = CCW, or inverted).
    dir_pin: DIR,

    /// Delay provider for step timing.
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

    /// Motion executor for current move (if any).
    executor: Option<MotionExecutor>,

    /// Type-state marker.
    _state: PhantomData<STATE>,
}

impl<STEP, DIR, DELAY, STATE> StepperMotor<STEP, DIR, DELAY, STATE>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: DelayNs,
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

impl<STEP, DIR, DELAY> StepperMotor<STEP, DIR, DELAY, Idle>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: DelayNs,
{
    /// Create a new motor in the Idle state.
    pub(crate) fn new(
        step_pin: STEP,
        dir_pin: DIR,
        delay: DELAY,
        constraints: MechanicalConstraints,
        name: heapless::String<32>,
        invert_direction: bool,
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
    ) -> core::result::Result<StepperMotor<STEP, DIR, DELAY, Moving>, (Self, Error)> {
        // Calculate steps to target
        let target_steps = Steps::from_degrees(target, self.constraints.steps_per_degree);
        let delta_steps = target_steps.0 - self.position.steps().0;

        if delta_steps == 0 {
            // Already at target, return self unchanged
            return Err((self, Error::Motion(crate::error::MotionError::MoveTooShort {
                steps: 0,
                minimum: 1,
            })));
        }

        // Check limits - extract limit value before potentially moving self
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

        // Create executor
        let executor = MotionExecutor::new(profile);

        // Transition to Moving state
        Ok(StepperMotor {
            step_pin: self.step_pin,
            dir_pin: self.dir_pin,
            delay: self.delay,
            position: self.position,
            current_direction: self.current_direction,
            constraints: self.constraints,
            name: self.name,
            invert_direction: self.invert_direction,
            executor: Some(executor),
            _state: PhantomData,
        })
    }

    /// Move by a relative amount in degrees.
    pub fn move_by(
        self,
        delta: Degrees,
    ) -> core::result::Result<StepperMotor<STEP, DIR, DELAY, Moving>, (Self, Error)> {
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

impl<STEP, DIR, DELAY> StepperMotor<STEP, DIR, DELAY, Moving>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: DelayNs,
{
    /// Execute one step pulse.
    ///
    /// Returns `true` if the move is complete.
    pub fn step(&mut self) -> Result<bool> {
        let executor = self.executor.as_mut().ok_or(MotorError::NotInitialized)?;

        if executor.is_complete() {
            return Ok(true);
        }

        // Generate step pulse
        self.step_pin.set_high().map_err(|_| MotorError::PinError)?;

        // Pulse width (typically 1-10 microseconds is sufficient)
        self.delay.delay_us(2);

        self.step_pin.set_low().map_err(|_| MotorError::PinError)?;

        // Update position
        let direction = executor.profile().direction;
        self.position.move_steps(direction.sign());

        // Get delay for next step
        let interval_ns = executor.current_interval_ns();

        // Advance executor
        let has_more = executor.advance();

        if has_more {
            // Delay until next step (subtract pulse width)
            let delay_ns = interval_ns.saturating_sub(2000);
            if delay_ns > 0 {
                self.delay.delay_ns(delay_ns);
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
    pub fn finish(self) -> StepperMotor<STEP, DIR, DELAY, Idle> {
        StepperMotor {
            step_pin: self.step_pin,
            dir_pin: self.dir_pin,
            delay: self.delay,
            position: self.position,
            current_direction: self.current_direction,
            constraints: self.constraints,
            name: self.name,
            invert_direction: self.invert_direction,
            executor: None,
            _state: PhantomData,
        }
    }

    /// Run the move to completion (blocking).
    pub fn run_to_completion(mut self) -> Result<StepperMotor<STEP, DIR, DELAY, Idle>> {
        while !self.is_complete() {
            self.step()?;
        }
        Ok(self.finish())
    }
}

#[cfg(test)]
mod tests {
    // Tests require embedded-hal-mock, which is in dev-dependencies
}
