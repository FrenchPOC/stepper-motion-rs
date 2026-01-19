//! Homing execution for stepper motors with physical switches.
//!
//! This module provides the homing executor that coordinates motor movement
//! with switch readings to find the home position.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};

use crate::config::{HomingConfig, HomingDirection, HomingPhase, HomingStrategy};
use crate::config::{SwitchConfig, MechanicalConstraints};
use crate::config::units::Degrees;
use crate::error::{HomingError, MotorError, Result};
use crate::motion::Direction;

/// Switch inputs for homing.
///
/// This struct holds references to the physical switch input pins
/// used during homing operations.
pub struct HomingSwitches<'a, HOME, MIN, MAX>
where
    HOME: InputPin,
    MIN: InputPin,
    MAX: InputPin,
{
    /// Home switch input pin.
    pub home: Option<&'a mut HOME>,
    /// Home switch configuration.
    pub home_config: Option<SwitchConfig>,
    /// Minimum limit switch input pin.
    pub limit_min: Option<&'a mut MIN>,
    /// Minimum limit switch configuration.
    pub limit_min_config: Option<SwitchConfig>,
    /// Maximum limit switch input pin.
    pub limit_max: Option<&'a mut MAX>,
    /// Maximum limit switch configuration.
    pub limit_max_config: Option<SwitchConfig>,
}

impl<'a, HOME, MIN, MAX> HomingSwitches<'a, HOME, MIN, MAX>
where
    HOME: InputPin,
    MIN: InputPin,
    MAX: InputPin,
{
    /// Create a new switch set with all switches explicitly specified.
    pub fn new(
        home: Option<&'a mut HOME>,
        home_config: Option<SwitchConfig>,
        limit_min: Option<&'a mut MIN>,
        limit_min_config: Option<SwitchConfig>,
        limit_max: Option<&'a mut MAX>,
        limit_max_config: Option<SwitchConfig>,
    ) -> Self {
        Self {
            home,
            home_config,
            limit_min,
            limit_min_config,
            limit_max,
            limit_max_config,
        }
    }

    /// Check if home switch is triggered.
    pub fn is_home_triggered(&mut self) -> core::result::Result<bool, MotorError> {
        match (&mut self.home, &self.home_config) {
            (Some(pin), Some(config)) => {
                let state = pin.is_high().map_err(|_| MotorError::SwitchReadError)?;
                Ok(config.is_active(state))
            }
            _ => Ok(false),
        }
    }

    /// Check if minimum limit switch is triggered.
    pub fn is_min_limit_triggered(&mut self) -> core::result::Result<bool, MotorError> {
        match (&mut self.limit_min, &self.limit_min_config) {
            (Some(pin), Some(config)) => {
                let state = pin.is_high().map_err(|_| MotorError::SwitchReadError)?;
                Ok(config.is_active(state))
            }
            _ => Ok(false),
        }
    }

    /// Check if maximum limit switch is triggered.
    pub fn is_max_limit_triggered(&mut self) -> core::result::Result<bool, MotorError> {
        match (&mut self.limit_max, &self.limit_max_config) {
            (Some(pin), Some(config)) => {
                let state = pin.is_high().map_err(|_| MotorError::SwitchReadError)?;
                Ok(config.is_active(state))
            }
            _ => Ok(false),
        }
    }

    /// Check if any limit switch in the direction of travel is triggered.
    pub fn is_limit_triggered_in_direction(
        &mut self,
        direction: HomingDirection,
    ) -> core::result::Result<bool, MotorError> {
        match direction {
            HomingDirection::ToMin => self.is_min_limit_triggered(),
            HomingDirection::ToMax => self.is_max_limit_triggered(),
        }
    }
}

/// Homing executor state machine.
///
/// Manages the homing sequence through various phases:
/// 1. Fast approach towards switch
/// 2. Backoff after triggering
/// 3. Slow approach for precision
/// 4. Move to final offset position
pub struct HomingExecutor {
    /// Homing configuration.
    config: HomingConfig,
    /// Current homing phase.
    phase: HomingPhase,
    /// Current step count within current phase.
    steps_in_phase: i64,
    /// Maximum steps for current phase.
    max_steps_in_phase: i64,
    /// Total steps taken during homing.
    total_steps: i64,
    /// Steps per degree for conversions.
    steps_per_degree: f32,
    /// Current step interval in nanoseconds.
    current_interval_ns: u32,
    /// Fast approach interval (nanoseconds).
    fast_interval_ns: u32,
    /// Slow approach interval (nanoseconds).
    slow_interval_ns: u32,
    /// Current direction.
    direction: Direction,
}

impl HomingExecutor {
    /// Create a new homing executor.
    pub fn new(config: HomingConfig, constraints: &MechanicalConstraints) -> Self {
        let steps_per_degree = constraints.steps_per_degree;

        // Calculate step intervals from velocities
        let fast_steps_per_sec = config.fast_velocity.0 * steps_per_degree;
        let slow_steps_per_sec = config.slow_velocity.0 * steps_per_degree;

        let fast_interval_ns = if fast_steps_per_sec > 0.0 {
            (1_000_000_000.0 / fast_steps_per_sec) as u32
        } else {
            100_000 // 100µs default
        };

        let slow_interval_ns = if slow_steps_per_sec > 0.0 {
            (1_000_000_000.0 / slow_steps_per_sec) as u32
        } else {
            1_000_000 // 1ms default
        };

        // Calculate max steps for fast approach
        let max_travel_steps = (config.max_travel.0 * steps_per_degree) as i64;

        // Initial direction based on config
        let direction = match config.direction {
            HomingDirection::ToMin => Direction::CounterClockwise,
            HomingDirection::ToMax => Direction::Clockwise,
        };

        Self {
            config,
            phase: HomingPhase::FastApproach,
            steps_in_phase: 0,
            max_steps_in_phase: max_travel_steps,
            total_steps: 0,
            steps_per_degree,
            current_interval_ns: fast_interval_ns,
            fast_interval_ns,
            slow_interval_ns,
            direction,
        }
    }

    /// Get the current homing phase.
    #[inline]
    pub fn phase(&self) -> HomingPhase {
        self.phase
    }

    /// Check if homing is complete.
    #[inline]
    pub fn is_complete(&self) -> bool {
        matches!(self.phase, HomingPhase::Complete | HomingPhase::Failed)
    }

    /// Get current direction.
    #[inline]
    pub fn direction(&self) -> Direction {
        self.direction
    }

    /// Get current step interval in nanoseconds.
    #[inline]
    pub fn current_interval_ns(&self) -> u32 {
        self.current_interval_ns
    }

    /// Get total steps taken during homing.
    #[inline]
    pub fn total_steps(&self) -> i64 {
        self.total_steps
    }

    /// Get the homing configuration.
    #[inline]
    pub fn config(&self) -> &HomingConfig {
        &self.config
    }

    /// Process a step and update state.
    ///
    /// Returns `true` if homing should continue stepping,
    /// `false` if the current phase is complete or homing failed.
    pub fn step(&mut self) -> bool {
        if self.is_complete() {
            return false;
        }

        self.steps_in_phase += 1;
        self.total_steps += self.direction.sign();

        // Check if we've exceeded max steps for this phase
        if self.steps_in_phase >= self.max_steps_in_phase {
            match self.phase {
                HomingPhase::FastApproach => {
                    // Switch not found - fail
                    self.phase = HomingPhase::Failed;
                    return false;
                }
                HomingPhase::Backoff => {
                    // Backoff complete, start slow approach
                    self.transition_to_slow_approach();
                }
                HomingPhase::SlowApproach => {
                    // Switch not found during slow approach - fail
                    self.phase = HomingPhase::Failed;
                    return false;
                }
                HomingPhase::MovingToOffset => {
                    // Offset move complete
                    self.phase = HomingPhase::Complete;
                    return false;
                }
                _ => {}
            }
        }

        true
    }

    /// Notify executor that home switch was triggered.
    ///
    /// Call this when the home switch state changes to active.
    pub fn on_home_switch_triggered(&mut self) {
        match self.phase {
            HomingPhase::FastApproach => {
                self.transition_to_backoff();
            }
            HomingPhase::SlowApproach => {
                // Found home position with precision
                self.transition_to_offset_move();
            }
            _ => {}
        }
    }

    /// Notify executor that home switch was released.
    ///
    /// Call this when the home switch state changes to inactive during backoff.
    pub fn on_home_switch_released(&mut self) {
        if self.phase == HomingPhase::Backoff {
            // Continue backing off to ensure we're fully clear
        }
    }

    /// Notify that an unexpected limit was hit.
    pub fn on_unexpected_limit(&mut self) {
        self.phase = HomingPhase::Failed;
    }

    fn transition_to_backoff(&mut self) {
        self.phase = HomingPhase::Backoff;
        self.steps_in_phase = 0;

        // Reverse direction for backoff
        self.direction = self.direction.opposite();

        // Calculate backoff steps
        self.max_steps_in_phase = (self.config.backoff_distance.0 * self.steps_per_degree) as i64;

        // Use fast interval for backoff
        self.current_interval_ns = self.fast_interval_ns;
    }

    fn transition_to_slow_approach(&mut self) {
        self.phase = HomingPhase::SlowApproach;
        self.steps_in_phase = 0;

        // Reverse direction back towards switch
        self.direction = self.direction.opposite();

        // Max steps is backoff distance + margin
        self.max_steps_in_phase =
            ((self.config.backoff_distance.0 + 10.0) * self.steps_per_degree) as i64;

        // Use slow interval for precision
        self.current_interval_ns = self.slow_interval_ns;
    }

    fn transition_to_offset_move(&mut self) {
        if self.config.home_offset.0.abs() < 0.001 {
            // No offset needed, we're done
            self.phase = HomingPhase::Complete;
            return;
        }

        self.phase = HomingPhase::MovingToOffset;
        self.steps_in_phase = 0;

        // Direction based on offset sign
        self.direction = if self.config.home_offset.0 > 0.0 {
            Direction::Clockwise
        } else {
            Direction::CounterClockwise
        };

        // Calculate offset steps
        self.max_steps_in_phase =
            (self.config.home_offset.0.abs() * self.steps_per_degree) as i64;

        // Use slow interval for offset move
        self.current_interval_ns = self.slow_interval_ns;
    }

    /// Get the home position to set after successful homing.
    pub fn home_position(&self) -> Degrees {
        self.config.home_position
    }
}

/// Perform blocking homing sequence.
///
/// This function executes a complete homing sequence synchronously,
/// blocking until homing is complete or fails.
///
/// # Type Parameters
///
/// - `STEP`: Step pin type
/// - `DIR`: Direction pin type  
/// - `DELAY`: Delay provider type
/// - `HOME`: Home switch pin type
/// - `MIN`: Min limit switch pin type (can be same as HOME if not used)
/// - `MAX`: Max limit switch pin type (can be same as HOME if not used)
///
/// # Arguments
///
/// - `step_pin`: Mutable reference to step output pin
/// - `dir_pin`: Mutable reference to direction output pin
/// - `delay`: Mutable reference to delay provider
/// - `switches`: Switch inputs for homing
/// - `config`: Homing configuration
/// - `constraints`: Mechanical constraints
/// - `invert_direction`: Whether direction is inverted
///
/// # Returns
///
/// The number of steps taken during homing, or an error.
pub fn execute_homing_blocking<STEP, DIR, DELAY, HOME, MIN, MAX>(
    step_pin: &mut STEP,
    dir_pin: &mut DIR,
    delay: &mut DELAY,
    switches: &mut HomingSwitches<'_, HOME, MIN, MAX>,
    config: HomingConfig,
    constraints: &MechanicalConstraints,
    invert_direction: bool,
) -> Result<i64>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: DelayNs,
    HOME: InputPin,
    MIN: InputPin,
    MAX: InputPin,
{
    // Verify we have a home switch for strategies that require it
    match config.strategy {
        HomingStrategy::HomeSwitch | HomingStrategy::HomeSwitchFast => {
            if switches.home.is_none() {
                return Err(HomingError::NoHomeSwitchConfigured.into());
            }
        }
        HomingStrategy::MinLimitOffset => {
            if switches.limit_min.is_none() && switches.home.is_none() {
                return Err(HomingError::NoHomeSwitchConfigured.into());
            }
        }
        HomingStrategy::MaxLimitOffset => {
            if switches.limit_max.is_none() && switches.home.is_none() {
                return Err(HomingError::NoHomeSwitchConfigured.into());
            }
        }
        _ => {}
    }

    let mut executor = HomingExecutor::new(config, constraints);

    // Set initial direction
    set_direction(dir_pin, executor.direction(), invert_direction)?;

    // Check if we're already on the switch
    let already_on_switch = switches.is_home_triggered()?;
    if already_on_switch {
        // Back off first
        executor.on_home_switch_triggered();
        // Immediately transition to backoff since we're starting on switch
        set_direction(dir_pin, executor.direction(), invert_direction)?;
    }

    let mut prev_switch_state = already_on_switch;

    while !executor.is_complete() {
        // Check switches
        let home_triggered = switches.is_home_triggered()?;

        // Detect switch transitions
        if home_triggered && !prev_switch_state {
            executor.on_home_switch_triggered();
            // Direction may have changed
            set_direction(dir_pin, executor.direction(), invert_direction)?;
        } else if !home_triggered && prev_switch_state {
            executor.on_home_switch_released();
        }

        prev_switch_state = home_triggered;

        // Check for unexpected limits
        let limit_triggered =
            switches.is_limit_triggered_in_direction(executor.config().direction)?;
        if limit_triggered && executor.phase() == HomingPhase::FastApproach {
            executor.on_unexpected_limit();
            return Err(HomingError::UnexpectedLimitTriggered.into());
        }

        if executor.is_complete() {
            break;
        }

        // Generate step pulse
        step_pin.set_high().map_err(|_| MotorError::PinError)?;
        delay.delay_us(2);
        step_pin.set_low().map_err(|_| MotorError::PinError)?;

        // Advance executor
        if !executor.step() {
            if executor.phase() == HomingPhase::Failed {
                return Err(HomingError::SwitchNotFound {
                    distance_traveled: executor.total_steps() as f32 / constraints.steps_per_degree,
                    max_travel: executor.config().max_travel.0,
                }
                .into());
            }
            // Phase changed, update direction if needed
            set_direction(dir_pin, executor.direction(), invert_direction)?;
        }

        // Delay until next step
        let delay_ns = executor.current_interval_ns().saturating_sub(2000);
        if delay_ns > 0 {
            delay.delay_ns(delay_ns);
        }
    }

    if executor.phase() == HomingPhase::Failed {
        return Err(HomingError::SwitchNotFound {
            distance_traveled: executor.total_steps() as f32 / constraints.steps_per_degree,
            max_travel: executor.config().max_travel.0,
        }
        .into());
    }

    Ok(executor.total_steps())
}

fn set_direction<DIR: OutputPin>(
    dir_pin: &mut DIR,
    direction: Direction,
    invert: bool,
) -> Result<()> {
    let pin_high = match direction {
        Direction::Clockwise => !invert,
        Direction::CounterClockwise => invert,
    };

    if pin_high {
        dir_pin.set_high().map_err(|_| MotorError::PinError)?;
    } else {
        dir_pin.set_low().map_err(|_| MotorError::PinError)?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::HomingConfig;
    use crate::config::units::{DegreesPerSec, DegreesPerSecSquared, Microsteps};
    use crate::config::MotorConfig;

    fn make_test_constraints() -> MechanicalConstraints {
        let config = MotorConfig {
            name: heapless::String::try_from("test").unwrap(),
            steps_per_revolution: 200,
            microsteps: Microsteps::SIXTEENTH,
            gear_ratio: 1.0,
            max_velocity: DegreesPerSec(360.0),
            max_acceleration: DegreesPerSecSquared(720.0),
            invert_direction: false,
            limits: None,
            backlash_compensation: None,
            switches: None,
            homing: None,
        };
        MechanicalConstraints::from_config(&config)
    }

    #[test]
    fn test_homing_executor_creation() {
        let config = HomingConfig::default();
        let constraints = make_test_constraints();
        let executor = HomingExecutor::new(config, &constraints);

        assert_eq!(executor.phase(), HomingPhase::FastApproach);
        assert!(!executor.is_complete());
    }

    #[test]
    fn test_homing_executor_switch_trigger() {
        let config = HomingConfig::default();
        let constraints = make_test_constraints();
        let mut executor = HomingExecutor::new(config, &constraints);

        // Simulate stepping until switch triggered
        for _ in 0..100 {
            executor.step();
        }

        // Trigger switch
        executor.on_home_switch_triggered();
        assert_eq!(executor.phase(), HomingPhase::Backoff);
    }
}
