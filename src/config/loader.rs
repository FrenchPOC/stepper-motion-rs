//! Configuration loading from files (std only).

use std::fs;
use std::path::Path;

use crate::error::{ConfigError, Error, Result};

use super::SystemConfig;

/// Load configuration from a TOML file.
///
/// # Errors
///
/// Returns an error if the file cannot be read or parsed.
///
/// # Example
///
/// ```rust,ignore
/// use stepper_motion::load_config;
///
/// let config = load_config("motion.toml")?;
/// ```
pub fn load_config<P: AsRef<Path>>(path: P) -> Result<SystemConfig> {
    let content = fs::read_to_string(path.as_ref()).map_err(|e| {
        let msg = heapless::String::try_from(e.to_string().as_str()).unwrap_or_default();
        Error::Config(ConfigError::IoError(msg))
    })?;

    parse_config(&content)
}

/// Parse configuration from a TOML string.
///
/// # Errors
///
/// Returns an error if the TOML is invalid or fails validation.
pub fn parse_config(content: &str) -> Result<SystemConfig> {
    let config: SystemConfig = toml::from_str(content).map_err(|e| {
        let msg = heapless::String::try_from(e.message()).unwrap_or_default();
        Error::Config(ConfigError::ParseError(msg))
    })?;

    // Validate the configuration
    super::validation::validate_config(&config)?;

    Ok(config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_minimal_config() {
        let toml = r#"
[motors.x_axis]
name = "X-Axis"
steps_per_revolution = 200
microsteps = 16
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0
"#;

        let config = parse_config(toml).unwrap();
        assert!(config.motor("x_axis").is_some());
    }

    #[test]
    fn test_parse_with_trajectory() {
        let toml = r#"
[motors.x_axis]
name = "X-Axis"
steps_per_revolution = 200
microsteps = 16
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0

[trajectories.home]
motor = "x_axis"
target_degrees = 0.0
velocity_percent = 50
"#;

        let config = parse_config(toml).unwrap();
        assert!(config.trajectory("home").is_some());
    }

    #[test]
    fn test_parse_asymmetric_trajectory() {
        let toml = r#"
[motors.x_axis]
name = "X-Axis"
steps_per_revolution = 200
microsteps = 16
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0

[trajectories.gentle_stop]
motor = "x_axis"
target_degrees = 90.0
velocity_percent = 100
acceleration_deg_per_sec2 = 500.0
deceleration_deg_per_sec2 = 200.0
"#;

        let config = parse_config(toml).unwrap();
        let traj = config.trajectory("gentle_stop").unwrap();
        assert!(traj.is_asymmetric());
    }
}
