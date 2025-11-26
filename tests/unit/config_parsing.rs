//! Unit tests for TOML configuration parsing.

use stepper_motion::config::{load_config, SystemConfig};

/// Test parsing a valid motor configuration from TOML.
#[test]
fn test_parse_motor_config() {
    let toml_str = r#"
[motors.stepper1]
name = "main_axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity = 360.0
max_acceleration = 720.0
invert_direction = false
"#;

    let config: SystemConfig = toml::from_str(toml_str).expect("Failed to parse TOML");
    let motor = config.motor("stepper1").expect("Motor not found");
    
    assert_eq!(motor.name.as_str(), "main_axis");
    assert_eq!(motor.steps_per_revolution, 200);
    assert_eq!(motor.microsteps.value(), 16);
    assert_eq!(motor.gear_ratio, 1.0);
    assert_eq!(motor.max_velocity.0, 360.0);
    assert_eq!(motor.max_acceleration.0, 720.0);
    assert!(!motor.invert_direction);
}

/// Test parsing trajectory configuration with asymmetric acceleration.
#[test]
fn test_parse_trajectory_with_asymmetric_accel() {
    let toml_str = r#"
[motors.stepper1]
name = "main_axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity = 360.0
max_acceleration = 720.0

[trajectories.home]
motor = "stepper1"
target_degrees = 0.0
velocity_percent = 50
acceleration_deg_per_sec2 = 500.0
deceleration_deg_per_sec2 = 200.0
"#;

    let config: SystemConfig = toml::from_str(toml_str).expect("Failed to parse TOML");
    let trajectory = config.trajectory("home").expect("Trajectory not found");
    
    assert_eq!(trajectory.motor.as_str(), "stepper1");
    assert_eq!(trajectory.target_degrees.0, 0.0);
    assert_eq!(trajectory.velocity_percent, 50);
    assert_eq!(trajectory.acceleration.unwrap().0, 500.0);
    assert_eq!(trajectory.deceleration.unwrap().0, 200.0);
    assert!(trajectory.is_asymmetric());
}

/// Test parsing motor with soft limits.
#[test]
fn test_parse_motor_with_limits() {
    let toml_str = r#"
[motors.stepper1]
name = "limited_axis"
steps_per_revolution = 200
microsteps = 8
gear_ratio = 2.0
max_velocity = 180.0
max_acceleration = 360.0

[motors.stepper1.limits]
min_degrees = -90.0
max_degrees = 90.0
policy = "reject"
"#;

    let config: SystemConfig = toml::from_str(toml_str).expect("Failed to parse TOML");
    let motor = config.motor("stepper1").expect("Motor not found");
    
    let limits = motor.limits.as_ref().expect("Limits not found");
    assert_eq!(limits.min_degrees.0, -90.0);
    assert_eq!(limits.max_degrees.0, 90.0);
}

/// Test parsing waypoint trajectory (sequence).
#[test]
fn test_parse_waypoint_trajectory() {
    let toml_str = r#"
[motors.stepper1]
name = "main_axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity = 360.0
max_acceleration = 720.0

[sequences.scan]
motor = "stepper1"
waypoints = [0.0, 45.0, 90.0, 135.0, 180.0]
velocity_percent = 75
dwell_ms = 100
"#;

    let config: SystemConfig = toml::from_str(toml_str).expect("Failed to parse TOML");
    let sequence = config.sequence("scan").expect("Sequence not found");
    
    assert_eq!(sequence.motor.as_str(), "stepper1");
    assert_eq!(sequence.waypoints.len(), 5);
    assert_eq!(sequence.velocity_percent, 75);
    assert_eq!(sequence.dwell_ms, 100);
}

/// Test that invalid microstep values are rejected during parsing.
#[test]
fn test_invalid_microsteps_rejected() {
    let toml_str = r#"
[motors.stepper1]
name = "bad_config"
steps_per_revolution = 200
microsteps = 12
gear_ratio = 1.0
max_velocity = 360.0
max_acceleration = 720.0
"#;

    let result: Result<SystemConfig, _> = toml::from_str(toml_str);
    assert!(result.is_err(), "Should reject non-power-of-2 microsteps");
}
