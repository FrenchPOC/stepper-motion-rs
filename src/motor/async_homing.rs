//! Async homing execution for stepper motors with physical switches.
//!
//! This module provides async homing functionality using embedded-hal-async traits
//! and embassy-time for non-blocking delays.

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::delay::DelayNs as AsyncDelayNs;

use crate::config::MechanicalConstraints;
use crate::config::{HomingConfig, HomingPhase, HomingStrategy};
use crate::error::{HomingError, MotorError, Result};
use crate::motion::Direction;

use super::homing::{HomingExecutor, HomingSwitches};

/// Perform async homing sequence.
///
/// This function executes a complete homing sequence asynchronously,
/// yielding during delays to allow other tasks to run.
///
/// # Type Parameters
///
/// - `STEP`: Step pin type
/// - `DIR`: Direction pin type
/// - `DELAY`: Async delay provider type
/// - `HOME`: Home switch pin type
/// - `MIN`: Min limit switch pin type
/// - `MAX`: Max limit switch pin type
///
/// # Arguments
///
/// - `step_pin`: Mutable reference to step output pin
/// - `dir_pin`: Mutable reference to direction output pin
/// - `delay`: Mutable reference to async delay provider
/// - `switches`: Switch inputs for homing
/// - `config`: Homing configuration
/// - `constraints`: Mechanical constraints
/// - `invert_direction`: Whether direction is inverted
///
/// # Returns
///
/// The number of steps taken during homing, or an error.
pub async fn execute_homing_async<STEP, DIR, DELAY, HOME, MIN, MAX>(
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
    DELAY: AsyncDelayNs,
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
    set_direction_async(dir_pin, executor.direction(), invert_direction)?;

    // Check if we're already on the switch
    let already_on_switch = switches.is_home_triggered()?;
    if already_on_switch {
        // Back off first
        executor.on_home_switch_triggered();
        // Immediately transition to backoff since we're starting on switch
        set_direction_async(dir_pin, executor.direction(), invert_direction)?;
    }

    let mut prev_switch_state = already_on_switch;

    while !executor.is_complete() {
        // Check switches
        let home_triggered = switches.is_home_triggered()?;

        // Detect switch transitions
        if home_triggered && !prev_switch_state {
            executor.on_home_switch_triggered();
            // Direction may have changed
            set_direction_async(dir_pin, executor.direction(), invert_direction)?;
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
        delay.delay_us(2).await;
        step_pin.set_low().map_err(|_| MotorError::PinError)?;

        // Advance executor
        let direction_before = executor.direction();
        let keep_stepping = executor.step();

        // Direction can change while step() still returns true
        // (Backoff -> SlowApproach), so update DIR immediately.
        if executor.direction() != direction_before {
            set_direction_async(dir_pin, executor.direction(), invert_direction)?;
        }

        if !keep_stepping {
            if executor.phase() == HomingPhase::Failed {
                return Err(HomingError::SwitchNotFound {
                    distance_traveled: executor.total_steps() as f32 / constraints.steps_per_degree,
                    max_travel: executor.config().max_travel.0,
                }
                .into());
            }
            // Phase changed, update direction if needed
            set_direction_async(dir_pin, executor.direction(), invert_direction)?;
        }

        // Async delay until next step
        let delay_ns = executor.current_interval_ns().saturating_sub(2000);
        if delay_ns > 0 {
            delay.delay_ns(delay_ns).await;
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

fn set_direction_async<DIR: OutputPin>(
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
