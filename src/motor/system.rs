//! Motor system facade for multi-motor configuration.
//!
//! Provides a high-level API for managing multiple motors from a single configuration.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use heapless::{FnvIndexMap, String};

use crate::config::{MechanicalConstraints, MotorConfig, SystemConfig};
use crate::error::{ConfigError, Error, Result};
use crate::motor::state::Idle;
use crate::motor::{StepperMotor, StepperMotorBuilder};
use crate::trajectory::TrajectoryRegistry;

/// A facade for managing multiple stepper motors from configuration.
///
/// `MotorSystem` provides a high-level API for:
/// - Creating motors from named configurations
/// - Accessing motors by name
/// - Managing trajectory registries
///
/// # Example
///
/// ```rust,ignore
/// use stepper_motion::motor::MotorSystem;
///
/// let config: SystemConfig = toml::from_str(CONFIG_TOML)?;
/// let mut system = MotorSystem::from_config(config);
///
/// // Register motors with their hardware pins
/// system.register_motor("x_axis", step_pin_x, dir_pin_x, delay_x)?;
/// system.register_motor("y_axis", step_pin_y, dir_pin_y, delay_y)?;
///
/// // Access motors by name
/// if let Some(motor) = system.motor("x_axis") {
///     println!("X position: {}", motor.position_degrees().0);
/// }
/// ```
pub struct MotorSystem {
    /// The system configuration.
    config: SystemConfig,
    /// Trajectory registry for named lookups.
    registry: TrajectoryRegistry,
    /// Registered motor names (actual motors are owned by user due to generic types).
    registered_motors: FnvIndexMap<String<32>, MechanicalConstraints, 8>,
}

impl MotorSystem {
    /// Create a new motor system from configuration.
    ///
    /// This initializes the trajectory registry but does not create any motors.
    /// Motors must be registered individually using `register_motor()` or
    /// created using `build_motor()`.
    pub fn from_config(config: SystemConfig) -> Self {
        let registry = TrajectoryRegistry::from_config(&config);
        Self {
            config,
            registry,
            registered_motors: FnvIndexMap::new(),
        }
    }

    /// Get the system configuration.
    pub fn config(&self) -> &SystemConfig {
        &self.config
    }

    /// Get the trajectory registry.
    pub fn trajectories(&self) -> &TrajectoryRegistry {
        &self.registry
    }

    /// Get a motor configuration by name.
    ///
    /// Returns `None` if no motor with that name exists in the configuration.
    pub fn motor_config(&self, name: &str) -> Option<&MotorConfig> {
        self.config.motor(name)
    }

    /// Get mechanical constraints for a motor by name.
    ///
    /// Returns `None` if no motor with that name exists.
    pub fn constraints(&self, name: &str) -> Option<MechanicalConstraints> {
        self.config
            .motor(name)
            .map(MechanicalConstraints::from_config)
    }

    /// Check if a motor name exists in the configuration.
    pub fn has_motor(&self, name: &str) -> bool {
        self.config.motor(name).is_some()
    }

    /// List all configured motor names.
    pub fn motor_names(&self) -> impl Iterator<Item = &str> {
        self.config.motor_names()
    }

    /// Register a motor as active in the system.
    ///
    /// This marks the motor as registered and stores its constraints.
    /// The actual motor instance is returned to the caller.
    ///
    /// # Errors
    ///
    /// Returns an error if the motor name doesn't exist in the configuration.
    pub fn register_motor<STEP, DIR, DELAY>(
        &mut self,
        name: &str,
        step_pin: STEP,
        dir_pin: DIR,
        delay: DELAY,
    ) -> Result<StepperMotor<STEP, DIR, DELAY, Idle>>
    where
        STEP: OutputPin,
        DIR: OutputPin,
        DELAY: DelayNs,
    {
        let motor_config = self.config.motor(name).ok_or_else(|| {
            Error::Config(ConfigError::MotorNotFound(
                String::try_from(name).unwrap_or_default(),
            ))
        })?;

        let constraints = MechanicalConstraints::from_config(motor_config);

        // Store the constraints for this motor
        let motor_name: String<32> = String::try_from(name).unwrap_or_default();
        let _ = self.registered_motors.insert(motor_name, constraints);

        // Build and return the motor
        StepperMotorBuilder::new()
            .step_pin(step_pin)
            .dir_pin(dir_pin)
            .delay(delay)
            .from_motor_config(motor_config)
            .build()
    }

    /// Build a motor from configuration without registering it.
    ///
    /// Use this when you need a motor but don't need system-level tracking.
    ///
    /// # Errors
    ///
    /// Returns an error if the motor name doesn't exist or building fails.
    pub fn build_motor<STEP, DIR, DELAY>(
        &self,
        name: &str,
        step_pin: STEP,
        dir_pin: DIR,
        delay: DELAY,
    ) -> Result<StepperMotor<STEP, DIR, DELAY, Idle>>
    where
        STEP: OutputPin,
        DIR: OutputPin,
        DELAY: DelayNs,
    {
        let motor_config = self.config.motor(name).ok_or_else(|| {
            Error::Config(ConfigError::MotorNotFound(
                String::try_from(name).unwrap_or_default(),
            ))
        })?;

        StepperMotorBuilder::new()
            .step_pin(step_pin)
            .dir_pin(dir_pin)
            .delay(delay)
            .from_motor_config(motor_config)
            .build()
    }

    /// Check if a motor has been registered.
    pub fn is_registered(&self, name: &str) -> bool {
        self.registered_motors
            .iter()
            .any(|(k, _)| k.as_str() == name)
    }

    /// Get the number of registered motors.
    pub fn registered_count(&self) -> usize {
        self.registered_motors.len()
    }

    /// Get constraints for a registered motor.
    ///
    /// Returns `None` if the motor is not registered.
    pub fn registered_constraints(&self, name: &str) -> Option<&MechanicalConstraints> {
        self.registered_motors
            .iter()
            .find(|(k, _)| k.as_str() == name)
            .map(|(_, v)| v)
    }

    /// Get a trajectory by name, with error if not found.
    ///
    /// This is a convenience method that delegates to the registry.
    pub fn trajectory(&self, name: &str) -> Result<&crate::config::TrajectoryConfig> {
        self.registry.get_or_error(name)
    }

    /// Get all trajectory names for a specific motor.
    pub fn trajectories_for_motor<'a>(
        &'a self,
        motor_name: &'a str,
    ) -> impl Iterator<Item = &'a str> + 'a {
        self.registry
            .iter()
            .filter(move |(_, traj)| traj.motor.as_str() == motor_name)
            .map(|(name, _)| name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> SystemConfig {
        let toml = r#"
[motors.x_axis]
name = "X Axis"
steps_per_revolution = 200
microsteps = 16
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0

[motors.y_axis]
name = "Y Axis"
steps_per_revolution = 400
microsteps = 8
max_velocity_deg_per_sec = 180.0
max_acceleration_deg_per_sec2 = 360.0

[trajectories.home_x]
motor = "x_axis"
target_degrees = 0.0
velocity_percent = 50

[trajectories.home_y]
motor = "y_axis"
target_degrees = 0.0
velocity_percent = 50
"#;
        toml::from_str(toml).unwrap()
    }

    #[test]
    fn test_motor_system_creation() {
        let config = test_config();
        let system = MotorSystem::from_config(config);

        assert!(system.has_motor("x_axis"));
        assert!(system.has_motor("y_axis"));
        assert!(!system.has_motor("z_axis"));
    }

    #[test]
    fn test_motor_names() {
        let config = test_config();
        let system = MotorSystem::from_config(config);

        let names: Vec<_> = system.motor_names().collect();
        assert!(names.contains(&"x_axis"));
        assert!(names.contains(&"y_axis"));
    }

    #[test]
    fn test_constraints_lookup() {
        let config = test_config();
        let system = MotorSystem::from_config(config);

        let constraints = system.constraints("x_axis").unwrap();
        // 200 * 16 = 3200 steps/rev
        assert_eq!(constraints.steps_per_revolution, 3200);

        let constraints = system.constraints("y_axis").unwrap();
        // 400 * 8 = 3200 steps/rev
        assert_eq!(constraints.steps_per_revolution, 3200);
    }

    #[test]
    fn test_trajectories_for_motor() {
        let config = test_config();
        let system = MotorSystem::from_config(config);

        let x_trajectories: Vec<_> = system.trajectories_for_motor("x_axis").collect();
        assert!(x_trajectories.contains(&"home_x"));
        assert!(!x_trajectories.contains(&"home_y"));

        let y_trajectories: Vec<_> = system.trajectories_for_motor("y_axis").collect();
        assert!(y_trajectories.contains(&"home_y"));
        assert!(!y_trajectories.contains(&"home_x"));
    }

    #[test]
    fn test_trajectory_lookup() {
        let config = test_config();
        let system = MotorSystem::from_config(config);

        let traj = system.trajectory("home_x");
        assert!(traj.is_ok());
        assert_eq!(traj.unwrap().motor.as_str(), "x_axis");

        let traj = system.trajectory("nonexistent");
        assert!(traj.is_err());
    }
}
