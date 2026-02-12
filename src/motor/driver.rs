//! Stepper motor driver.
//!
//! Generic over embedded-hal 1.0 pin types with type-state safety.

use core::marker::PhantomData;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};

use crate::config::units::{Degrees, DegreesPerSec, Steps};
use crate::config::MechanicalConstraints;
use crate::error::{Error, MotorError, Result};
use crate::motion::{Direction, MotionExecutor, MotionPhase, MotionProfile};

use super::homing::HomingSwitches;
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

    /// Backlash compensation in steps (applied on direction change).
    backlash_steps: i64,

    /// Motion executor for current move (if any).
    executor: Option<MotionExecutor>,

    /// Constant step interval for continuous mode (if running continuously).
    continuous_interval_ns: Option<u32>,

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

    fn current_motion_direction(&self) -> Option<Direction> {
        self.executor
            .as_ref()
            .map(|e| e.profile().direction)
            .or(self.current_direction)
    }

    fn next_position_steps(&self, direction: Direction) -> i64 {
        self.position.steps().0 + direction.sign()
    }

    fn soft_limit_for_next_step(&self, direction: Direction) -> Option<i64> {
        let next_position = self.next_position_steps(direction);
        self.constraints.limits.as_ref().and_then(|limits| {
            if limits.contains(next_position) {
                None
            } else if direction.sign() > 0 {
                Some(limits.max_steps)
            } else {
                Some(limits.min_steps)
            }
        })
    }

    fn hardware_limit_error(limit_type: &str) -> MotorError {
        MotorError::HardwareLimitTriggered {
            limit_type: heapless::String::try_from(limit_type).unwrap_or_default(),
        }
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
            continuous_interval_ns: None,
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
            return Err((
                self,
                Error::Motion(crate::error::MotionError::MoveTooShort {
                    steps: 0,
                    minimum: 1,
                }),
            ));
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
            backlash_steps: self.backlash_steps,
            executor: Some(executor),
            continuous_interval_ns: None,
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

    /// Start continuous forward motion at a constant velocity.
    ///
    /// Forward is the positive/clockwise direction in motor coordinates.
    /// Use [`StepperMotor::<STEP, DIR, DELAY, Moving>::stop`] to stop.
    pub fn start_continuous_forward(
        mut self,
        velocity: DegreesPerSec,
    ) -> core::result::Result<StepperMotor<STEP, DIR, DELAY, Moving>, (Self, Error)> {
        if velocity.0 <= 0.0 {
            return Err((
                self,
                Error::Motion(crate::error::MotionError::InvalidVelocity {
                    requested: velocity.0,
                }),
            ));
        }

        let max_velocity = self.constraints.max_velocity.0;
        if velocity.0 > max_velocity {
            return Err((
                self,
                Error::Motion(crate::error::MotionError::VelocityExceedsLimit {
                    requested: velocity.0,
                    max: max_velocity,
                }),
            ));
        }

        let direction = Direction::Clockwise;
        if let Some(limit) = self.soft_limit_for_next_step(direction) {
            let next_position = self.next_position_steps(direction);
            return Err((
                self,
                Error::Motor(MotorError::LimitExceeded {
                    position: next_position,
                    limit,
                }),
            ));
        }

        if self.set_direction(direction).is_err() {
            return Err((self, Error::Motor(MotorError::PinError)));
        }

        let interval_ns = self
            .constraints
            .velocity_to_interval_ns(self.constraints.velocity_to_steps(velocity.0));

        Ok(StepperMotor {
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
            continuous_interval_ns: Some(interval_ns),
            _state: PhantomData,
        })
    }

    /// Set the current position as the origin (zero).
    pub fn set_origin(&mut self) {
        self.position.set_origin();
    }

    /// Set the current position to a specific value.
    pub fn set_position(&mut self, degrees: Degrees) {
        self.position.set_degrees(degrees);
    }

    /// Execute a named trajectory from a registry.
    ///
    /// This method looks up the trajectory by name, validates it against
    /// the motor's constraints, and executes it to completion.
    ///
    /// # Arguments
    ///
    /// * `trajectory_name` - Name of the trajectory in the registry
    /// * `registry` - The trajectory registry to look up the trajectory
    ///
    /// # Returns
    ///
    /// Returns `Ok(self)` with the motor back in Idle state after the move completes.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The trajectory is not found in the registry
    /// - The trajectory's target motor doesn't match this motor's name
    /// - The move fails due to limits or hardware errors
    pub fn execute(
        self,
        trajectory_name: &str,
        registry: &crate::trajectory::TrajectoryRegistry,
    ) -> core::result::Result<Self, (Self, Error)> {
        // Look up trajectory
        let trajectory = match registry.get(trajectory_name) {
            Some(t) => t,
            None => {
                // Build error with available names
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
        self.move_to_blocking(target)
    }

    /// Move to an absolute position and run to completion (blocking).
    ///
    /// This is a convenience method that combines `move_to` and `run_to_completion`.
    pub fn move_to_blocking(self, target: Degrees) -> core::result::Result<Self, (Self, Error)> {
        match self.move_to(target) {
            Ok(moving) => {
                match moving.run_to_completion() {
                    Ok(idle) => Ok(idle),
                    Err(e) => {
                        // In practice, step errors are rare and typically unrecoverable
                        // We can't return the motor in a good state here
                        panic!("Motor step error during move: {:?}", e);
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
        if let Some(executor) = self.executor.as_ref() {
            if executor.is_complete() {
                return Ok(true);
            }
        }

        let direction = self
            .current_motion_direction()
            .ok_or(MotorError::NotInitialized)?;
        if let Some(limit) = self.soft_limit_for_next_step(direction) {
            return Err(MotorError::LimitExceeded {
                position: self.next_position_steps(direction),
                limit,
            }
            .into());
        }

        // Generate step pulse
        self.step_pin.set_high().map_err(|_| MotorError::PinError)?;

        // Pulse width (typically 1-10 microseconds is sufficient)
        self.delay.delay_us(2);

        self.step_pin.set_low().map_err(|_| MotorError::PinError)?;

        // Update position
        self.position.move_steps(direction.sign());

        if let Some(interval_ns) = self.continuous_interval_ns {
            let delay_ns = interval_ns.saturating_sub(2000);
            if delay_ns > 0 {
                self.delay.delay_ns(delay_ns);
            }
            return Ok(false);
        }

        // Profiled motion path
        let executor = self.executor.as_mut().ok_or(MotorError::NotInitialized)?;
        let interval_ns = executor.current_interval_ns();
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

    /// Execute one step after checking home/limit switches.
    ///
    /// This is useful for continuous motion where physical switches must be
    /// checked before each pulse.
    pub fn step_with_switch_checks<HOME, MIN, MAX>(
        &mut self,
        switches: &mut HomingSwitches<'_, HOME, MIN, MAX>,
    ) -> Result<bool>
    where
        HOME: InputPin,
        MIN: InputPin,
        MAX: InputPin,
    {
        if switches.is_home_triggered()? {
            return Err(Self::hardware_limit_error("home").into());
        }
        if switches.is_min_limit_triggered()? {
            return Err(Self::hardware_limit_error("min").into());
        }
        if switches.is_max_limit_triggered()? {
            return Err(Self::hardware_limit_error("max").into());
        }

        self.step()
    }

    /// Check whether this motor is running in continuous mode.
    #[inline]
    pub fn is_continuous(&self) -> bool {
        self.continuous_interval_ns.is_some()
    }

    /// Check if the move is complete.
    #[inline]
    pub fn is_complete(&self) -> bool {
        if self.is_continuous() {
            return false;
        }
        self.executor
            .as_ref()
            .map(|e| e.is_complete())
            .unwrap_or(true)
    }

    /// Get move progress (0.0 to 1.0).
    #[inline]
    pub fn progress(&self) -> f32 {
        if self.is_continuous() {
            return 0.0;
        }
        self.executor.as_ref().map(|e| e.progress()).unwrap_or(1.0)
    }

    /// Get current motion phase.
    #[inline]
    pub fn phase(&self) -> MotionPhase {
        if self.is_continuous() {
            return MotionPhase::Cruising;
        }
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
            backlash_steps: self.backlash_steps,
            executor: None,
            continuous_interval_ns: None,
            _state: PhantomData,
        }
    }

    /// Stop motion and return to Idle state.
    #[inline]
    pub fn stop(self) -> StepperMotor<STEP, DIR, DELAY, Idle> {
        self.finish()
    }

    /// Run the move to completion (blocking).
    pub fn run_to_completion(mut self) -> Result<StepperMotor<STEP, DIR, DELAY, Idle>> {
        if self.is_continuous() {
            return Err(MotorError::InvalidState(
                heapless::String::try_from("continuous_mode").unwrap_or_default(),
            )
            .into());
        }

        while !self.is_complete() {
            self.step()?;
        }
        Ok(self.finish())
    }
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use embedded_hal::digital::ErrorType;

    use super::*;
    use crate::config::units::{DegreesPerSec, DegreesPerSecSquared};
    use crate::config::{LimitPolicy, StepLimits, SwitchConfig, SwitchPolarity};

    struct MockOutputPin;

    impl ErrorType for MockOutputPin {
        type Error = Infallible;
    }

    impl OutputPin for MockOutputPin {
        fn set_low(&mut self) -> core::result::Result<(), Self::Error> {
            Ok(())
        }

        fn set_high(&mut self) -> core::result::Result<(), Self::Error> {
            Ok(())
        }
    }

    struct MockDelay;

    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    struct MockInputPin {
        high: bool,
    }

    impl ErrorType for MockInputPin {
        type Error = Infallible;
    }

    impl InputPin for MockInputPin {
        fn is_high(&mut self) -> core::result::Result<bool, Self::Error> {
            Ok(self.high)
        }

        fn is_low(&mut self) -> core::result::Result<bool, Self::Error> {
            Ok(!self.high)
        }
    }

    fn constraints(limits: Option<StepLimits>) -> MechanicalConstraints {
        MechanicalConstraints {
            steps_per_revolution: 200,
            steps_per_degree: 1.0,
            max_velocity_steps_per_sec: 500.0,
            max_acceleration_steps_per_sec2: 1000.0,
            min_step_interval_ns: 2_000_000,
            limits,
            max_velocity: DegreesPerSec(500.0),
            max_acceleration: DegreesPerSecSquared(1000.0),
        }
    }

    fn build_motor(
        limits: Option<StepLimits>,
    ) -> StepperMotor<MockOutputPin, MockOutputPin, MockDelay, Idle> {
        StepperMotor::new(
            MockOutputPin,
            MockOutputPin,
            MockDelay,
            constraints(limits),
            heapless::String::try_from("test").unwrap(),
            false,
            0,
        )
    }

    #[test]
    fn start_and_stop_continuous_forward() {
        let motor = build_motor(None);
        let mut moving = match motor.start_continuous_forward(DegreesPerSec(120.0)) {
            Ok(moving) => moving,
            Err((_motor, err)) => panic!("continuous start should succeed: {}", err),
        };

        assert!(moving.is_continuous());
        assert!(!moving.is_complete());

        moving.step().expect("first step");
        moving.step().expect("second step");
        assert_eq!(moving.position_steps().0, 2);

        let idle = moving.stop();
        assert_eq!(idle.position_steps().0, 2);
    }

    #[test]
    fn continuous_forward_enforces_soft_limit() {
        let limits = StepLimits {
            min_steps: -10,
            max_steps: 2,
            policy: LimitPolicy::Reject,
        };
        let motor = build_motor(Some(limits));
        let mut moving = match motor.start_continuous_forward(DegreesPerSec(100.0)) {
            Ok(moving) => moving,
            Err((_motor, err)) => panic!("continuous start should succeed: {}", err),
        };

        moving.step().expect("step 1");
        moving.step().expect("step 2");

        let err = moving.step().expect_err("third step should hit limit");
        match err {
            Error::Motor(MotorError::LimitExceeded { position, limit }) => {
                assert_eq!(position, 3);
                assert_eq!(limit, 2);
            }
            other => panic!("unexpected error: {:?}", other),
        }
    }

    #[test]
    fn switch_checks_stop_continuous_motion() {
        let motor = build_motor(None);
        let mut moving = match motor.start_continuous_forward(DegreesPerSec(100.0)) {
            Ok(moving) => moving,
            Err((_motor, err)) => panic!("continuous start should succeed: {}", err),
        };

        let mut home = MockInputPin { high: false };
        let mut switches: HomingSwitches<'_, MockInputPin, MockInputPin, MockInputPin> =
            HomingSwitches::new(
                Some(&mut home),
                Some(SwitchConfig::new(SwitchPolarity::NO)),
                None,
                None,
                None,
                None,
            );

        let err = moving
            .step_with_switch_checks(&mut switches)
            .expect_err("home switch should stop motion");

        match err {
            Error::Motor(MotorError::HardwareLimitTriggered { limit_type }) => {
                assert_eq!(limit_type.as_str(), "home");
            }
            other => panic!("unexpected error: {:?}", other),
        }
    }
}
