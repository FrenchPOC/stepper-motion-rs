//! Unit tests for configuration validation.

use stepper_motion::config::{validate_config, SystemConfig};
use stepper_motion::error::{Error, ConfigError};

/// Test validation of a valid configuration.
#[test]
fn test_valid_config_passes_validation() {
    let toml_str = r#"
[motors.stepper1]
name = "main_axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity = 360.0
max_acceleration = 720.0

[trajectories.move_90]
motor = "stepper1"
target_degrees = 90.0
velocity_percent = 100
"#;

    let config: SystemConfig = toml::from_str(toml_str).expect("Failed to parse TOML");
    assert!(validate_config(&config).is_ok());
}

/// Test validation fails for trajectory referencing non-existent motor.
#[test]
fn test_trajectory_invalid_motor_reference() {
    let toml_str = r#"
[motors.stepper1]
name = "main_axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity = 360.0
max_acceleration = 720.0

[trajectories.bad_ref]
motor = "nonexistent_motor"
target_degrees = 90.0
"#;

    let config: SystemConfig = toml::from_str(toml_str).expect("Failed to parse TOML");
    let result = validate_config(&config);
    assert!(result.is_err());
}

/// Test validation fails for invalid velocity percent.
#[test]
fn test_invalid_velocity_percent() {
    let toml_str = r#"
[motors.stepper1]
name = "main_axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity = 360.0
max_acceleration = 720.0

[trajectories.too_fast]
motor = "stepper1"
target_degrees = 90.0
velocity_percent = 250
"#;

    let config: SystemConfig = toml::from_str(toml_str).expect("Failed to parse TOML");
    let result = validate_config(&config);
    assert!(result.is_err());
}

/// Test validation fails for invalid soft limits (min >= max).
#[test]
fn test_invalid_soft_limits() {
    let toml_str = r#"
[motors.stepper1]
name = "bad_limits"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity = 360.0
max_acceleration = 720.0

[motors.stepper1.limits]
min_degrees = 90.0
max_degrees = -90.0
"#;

    let config: SystemConfig = toml::from_str(toml_str).expect("Failed to parse TOML");
    let result = validate_config(&config);
    assert!(result.is_err());
}

/// Test that empty configuration is valid.
#[test]
fn test_empty_config_is_valid() {
    let config = SystemConfig::default();
    assert!(validate_config(&config).is_ok());
}
