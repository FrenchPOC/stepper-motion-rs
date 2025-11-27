//! Builder pattern for StepperMotor.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

use crate::config::units::{DegreesPerSec, DegreesPerSecSquared, Microsteps};
use crate::config::{MechanicalConstraints, MotorConfig, SystemConfig};
use crate::error::{ConfigError, Error, Result};

use super::driver::StepperMotor;
use super::state::Idle;

/// Builder for creating StepperMotor instances.
pub struct StepperMotorBuilder<STEP, DIR, DELAY>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: DelayNs,
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

impl<STEP, DIR, DELAY> Default for StepperMotorBuilder<STEP, DIR, DELAY>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: DelayNs,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<STEP, DIR, DELAY> StepperMotorBuilder<STEP, DIR, DELAY>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: DelayNs,
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

    /// Set the delay provider.
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

    /// Set gear ratio.
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

    /// Set direction inversion.
    pub fn invert_direction(mut self, invert: bool) -> Self {
        self.invert_direction = invert;
        self
    }

    /// Set backlash compensation in steps.
    ///
    /// Backlash is applied on direction changes to compensate for mechanical play.
    pub fn backlash_steps(mut self, steps: i64) -> Self {
        self.backlash_steps = steps;
        self
    }

    /// Configure from a MotorConfig.
    pub fn from_motor_config(mut self, config: &MotorConfig) -> Self {
        self.name = Some(config.name.clone());
        self.steps_per_revolution = Some(config.steps_per_revolution);
        self.microsteps = Some(config.microsteps);
        self.gear_ratio = config.gear_ratio;
        self.max_velocity = Some(config.max_velocity);
        self.max_acceleration = Some(config.max_acceleration);
        self.invert_direction = config.invert_direction;
        self.constraints = Some(MechanicalConstraints::from_config(config));
        // Extract backlash compensation if configured (convert degrees to steps)
        if let Some(backlash_deg) = config.backlash_compensation {
            let steps_per_degree = config.steps_per_degree();
            self.backlash_steps = (backlash_deg.0 * steps_per_degree) as i64;
        }
        self
    }

    /// Configure from SystemConfig by motor name.
    pub fn from_config(self, config: &SystemConfig, motor_name: &str) -> Result<Self> {
        let motor_config = config
            .motor(motor_name)
            .ok_or_else(|| Error::Config(ConfigError::MotorNotFound(
                heapless::String::try_from(motor_name).unwrap_or_default(),
            )))?;

        Ok(self.from_motor_config(motor_config))
    }

    /// Build the StepperMotor.
    ///
    /// # Errors
    ///
    /// Returns an error if required fields are missing.
    pub fn build(self) -> Result<StepperMotor<STEP, DIR, DELAY, Idle>> {
        let step_pin = self.step_pin.ok_or_else(|| {
            Error::Config(ConfigError::ParseError(
                heapless::String::try_from("step_pin is required").unwrap(),
            ))
        })?;

        let dir_pin = self.dir_pin.ok_or_else(|| {
            Error::Config(ConfigError::ParseError(
                heapless::String::try_from("dir_pin is required").unwrap(),
            ))
        })?;

        let delay = self.delay.ok_or_else(|| {
            Error::Config(ConfigError::ParseError(
                heapless::String::try_from("delay is required").unwrap(),
            ))
        })?;

        let name = self.name.unwrap_or_else(|| {
            heapless::String::try_from("motor").unwrap()
        });

        let constraints = if let Some(c) = self.constraints {
            c
        } else {
            // Build constraints from individual fields
            let steps = self.steps_per_revolution.ok_or_else(|| {
                Error::Config(ConfigError::ParseError(
                    heapless::String::try_from("steps_per_revolution is required").unwrap(),
                ))
            })?;

            let microsteps = self.microsteps.unwrap_or(Microsteps::FULL);
            let max_velocity = self.max_velocity.ok_or_else(|| {
                Error::Config(ConfigError::ParseError(
                    heapless::String::try_from("max_velocity is required").unwrap(),
                ))
            })?;

            let max_acceleration = self.max_acceleration.ok_or_else(|| {
                Error::Config(ConfigError::ParseError(
                    heapless::String::try_from("max_acceleration is required").unwrap(),
                ))
            })?;

            // Create a temporary MotorConfig to compute constraints
            let config = MotorConfig {
                name: name.clone(),
                steps_per_revolution: steps,
                microsteps,
                gear_ratio: self.gear_ratio,
                max_velocity,
                max_acceleration,
                invert_direction: self.invert_direction,
                limits: None,
                backlash_compensation: None,
            };

            MechanicalConstraints::from_config(&config)
        };

        Ok(StepperMotor::new(
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
