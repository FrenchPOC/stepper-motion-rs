//! Builder pattern for AsyncStepperMotor.

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs as AsyncDelayNs;

use crate::config::units::{DegreesPerSec, DegreesPerSecSquared, Microsteps};
use crate::config::{MechanicalConstraints, MotorConfig, SystemConfig};
use crate::error::{ConfigError, Error, Result};

use super::async_driver::AsyncStepperMotor;
use super::state::Idle;

/// Builder for creating AsyncStepperMotor instances.
pub struct AsyncStepperMotorBuilder<STEP, DIR, DELAY>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: AsyncDelayNs,
{
    step_pin: Option<STEP>,
    dir_pin: Option<DIR>,
    delay: Option<DELAY>,
    name: Option<heapless::String<32>>,
    steps_per_revolution: Option<u16>,
    microsteps: Option<Microsteps>,
    gear_ratio: f32,
    max_velocity: Option<DegreesPerSec>,
    max_acceleration: Option<DegreesPerSecSquared>,
    invert_direction: bool,
    constraints: Option<MechanicalConstraints>,
    backlash_steps: i64,
}

impl<STEP, DIR, DELAY> Default for AsyncStepperMotorBuilder<STEP, DIR, DELAY>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: AsyncDelayNs,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<STEP, DIR, DELAY> AsyncStepperMotorBuilder<STEP, DIR, DELAY>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: AsyncDelayNs,
{
    /// Create a new builder.
    pub fn new() -> Self {
        Self {
            step_pin: None,
            dir_pin: None,
            delay: None,
            name: None,
            steps_per_revolution: None,
            microsteps: None,
            gear_ratio: 1.0,
            max_velocity: None,
            max_acceleration: None,
            invert_direction: false,
            constraints: None,
            backlash_steps: 0,
        }
    }

    /// Set the STEP pin.
    pub fn step_pin(mut self, pin: STEP) -> Self {
        self.step_pin = Some(pin);
        self
    }

    /// Set the DIR pin.
    pub fn dir_pin(mut self, pin: DIR) -> Self {
        self.dir_pin = Some(pin);
        self
    }

    /// Set the async delay provider.
    pub fn delay(mut self, delay: DELAY) -> Self {
        self.delay = Some(delay);
        self
    }

    /// Set the motor name.
    pub fn name(mut self, name: &str) -> Self {
        self.name = heapless::String::try_from(name).ok();
        self
    }

    /// Set steps per revolution (base motor steps before microstepping).
    pub fn steps_per_revolution(mut self, steps: u16) -> Self {
        self.steps_per_revolution = Some(steps);
        self
    }

    /// Set microstep configuration.
    pub fn microsteps(mut self, microsteps: Microsteps) -> Self {
        self.microsteps = Some(microsteps);
        self
    }

    /// Set gear ratio (> 1.0 means gear reduction).
    pub fn gear_ratio(mut self, ratio: f32) -> Self {
        self.gear_ratio = ratio;
        self
    }

    /// Set maximum velocity in degrees per second.
    pub fn max_velocity(mut self, velocity: DegreesPerSec) -> Self {
        self.max_velocity = Some(velocity);
        self
    }

    /// Set maximum acceleration in degrees per second squared.
    pub fn max_acceleration(mut self, acceleration: DegreesPerSecSquared) -> Self {
        self.max_acceleration = Some(acceleration);
        self
    }

    /// Set whether direction pin logic is inverted.
    pub fn invert_direction(mut self, invert: bool) -> Self {
        self.invert_direction = invert;
        self
    }

    /// Set pre-computed mechanical constraints directly.
    pub fn constraints(mut self, constraints: MechanicalConstraints) -> Self {
        self.constraints = Some(constraints);
        self
    }

    /// Set backlash compensation in steps.
    pub fn backlash_steps(mut self, steps: i64) -> Self {
        self.backlash_steps = steps;
        self
    }

    /// Configure from a MotorConfig.
    ///
    /// This extracts all relevant parameters from the config.
    pub fn from_motor_config(mut self, config: &MotorConfig) -> Self {
        self.name = Some(config.name.clone());
        self.steps_per_revolution = Some(config.steps_per_revolution);
        self.microsteps = Some(config.microsteps);
        self.gear_ratio = config.gear_ratio;
        self.max_velocity = Some(config.max_velocity);
        self.max_acceleration = Some(config.max_acceleration);
        self.invert_direction = config.invert_direction;
        self.constraints = Some(MechanicalConstraints::from_config(config));
        self
    }

    /// Configure from a named motor in a SystemConfig.
    ///
    /// # Errors
    ///
    /// Returns an error if the motor name is not found in the configuration.
    pub fn from_config(self, config: &SystemConfig, motor_name: &str) -> Result<Self> {
        let motor_config = config.motor(motor_name).ok_or_else(|| {
            Error::Config(ConfigError::MotorNotFound(
                heapless::String::try_from(motor_name).unwrap_or_default(),
            ))
        })?;

        Ok(self.from_motor_config(motor_config))
    }

    /// Build the AsyncStepperMotor.
    ///
    /// # Errors
    ///
    /// Returns an error if required fields are missing or invalid.
    pub fn build(self) -> Result<AsyncStepperMotor<STEP, DIR, DELAY, Idle>> {
        let step_pin = self.step_pin.ok_or_else(|| {
            Error::Config(ConfigError::ParseError(
                heapless::String::try_from("missing step_pin").unwrap_or_default(),
            ))
        })?;

        let dir_pin = self.dir_pin.ok_or_else(|| {
            Error::Config(ConfigError::ParseError(
                heapless::String::try_from("missing dir_pin").unwrap_or_default(),
            ))
        })?;

        let delay = self.delay.ok_or_else(|| {
            Error::Config(ConfigError::ParseError(
                heapless::String::try_from("missing delay").unwrap_or_default(),
            ))
        })?;

        // If constraints were set directly, use them
        let constraints = if let Some(c) = self.constraints {
            c
        } else {
            // Otherwise, build from individual parameters
            let steps_per_revolution = self.steps_per_revolution.ok_or_else(|| {
                Error::Config(ConfigError::ParseError(
                    heapless::String::try_from("missing steps_per_revolution").unwrap_or_default(),
                ))
            })?;

            let microsteps = self.microsteps.ok_or_else(|| {
                Error::Config(ConfigError::ParseError(
                    heapless::String::try_from("missing microsteps").unwrap_or_default(),
                ))
            })?;

            let max_velocity = self.max_velocity.ok_or_else(|| {
                Error::Config(ConfigError::ParseError(
                    heapless::String::try_from("missing max_velocity").unwrap_or_default(),
                ))
            })?;

            let max_acceleration = self.max_acceleration.ok_or_else(|| {
                Error::Config(ConfigError::ParseError(
                    heapless::String::try_from("missing max_acceleration").unwrap_or_default(),
                ))
            })?;

            // Create a temporary MotorConfig to use from_config
            let temp_config = MotorConfig {
                name: self.name.clone().unwrap_or_else(|| {
                    heapless::String::try_from("unnamed").unwrap_or_default()
                }),
                steps_per_revolution,
                microsteps,
                gear_ratio: self.gear_ratio,
                max_velocity,
                max_acceleration,
                invert_direction: self.invert_direction,
                limits: None,
                backlash_compensation: None,
                switches: None,
                homing: None,
            };

            MechanicalConstraints::from_config(&temp_config)
        };

        let name = self.name.unwrap_or_else(|| {
            heapless::String::try_from("unnamed").unwrap_or_default()
        });

        Ok(AsyncStepperMotor::new(
            step_pin,
            dir_pin,
            delay,
            constraints,
            name,
            self.invert_direction,
            self.backlash_steps,
        ))
    }
}

#[cfg(test)]
mod tests {
    // Tests require embedded-hal-mock with async support
}
