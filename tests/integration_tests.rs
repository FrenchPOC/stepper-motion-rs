//! Integration tests for stepper-motion library (T018-T021, T036-T038, T049-T051)
//!
//! These tests verify the complete workflow from TOML parsing to trajectory execution.

use stepper_motion::config::{
    LimitPolicy, MechanicalConstraints, SoftLimits, SystemConfig,
};
use stepper_motion::config::units::{Degrees, Microsteps};
use stepper_motion::trajectory::TrajectoryRegistry;

// =============================================================================
// Test configuration data
// =============================================================================

const MINIMAL_CONFIG: &str = r#"
[motors.test_motor]
name = "Test Motor"
steps_per_revolution = 200
microsteps = 16
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0
"#;

const FULL_CONFIG: &str = r#"
[motors.pan]
name = "Pan Axis"
steps_per_revolution = 200
microsteps = 32
gear_ratio = 4.0
max_velocity_deg_per_sec = 180.0
max_acceleration_deg_per_sec2 = 360.0
invert_direction = true

[motors.pan.limits]
min_degrees = -90.0
max_degrees = 90.0
policy = "reject"

[trajectories.home]
motor = "pan"
target_degrees = 0.0
velocity_percent = 50

[trajectories.asymmetric]
motor = "pan"
target_degrees = 45.0
velocity_percent = 100
acceleration_deg_per_sec2 = 288.0
deceleration_deg_per_sec2 = 180.0
"#;

// Helper to parse config using toml crate directly
fn parse_config(toml_str: &str) -> Result<SystemConfig, toml::de::Error> {
    toml::from_str(toml_str)
}

// =============================================================================
// T018: Unit test for TOML parsing
// =============================================================================

#[test]
fn t018_parse_minimal_motor_config() {
    let config = parse_config(MINIMAL_CONFIG).expect("Should parse minimal config");
    
    let motor = config.motor("test_motor").expect("Motor should exist");
    assert_eq!(motor.name.as_str(), "Test Motor");
    assert_eq!(motor.steps_per_revolution, 200);
    assert_eq!(motor.microsteps, Microsteps::SIXTEENTH);
    assert!((motor.max_velocity.0 - 360.0).abs() < 0.01);
    assert!((motor.max_acceleration.0 - 720.0).abs() < 0.01);
}

#[test]
fn t018_parse_full_motor_config() {
    let config = parse_config(FULL_CONFIG).expect("Should parse full config");
    
    let motor = config.motor("pan").expect("Pan motor should exist");
    assert_eq!(motor.name.as_str(), "Pan Axis");
    assert_eq!(motor.microsteps, Microsteps::THIRTY_SECOND);
    assert!((motor.gear_ratio - 4.0).abs() < 0.001);
    assert!(motor.invert_direction);
    
    let limits = motor.limits.as_ref().expect("Should have limits");
    assert!((limits.min.0 - (-90.0)).abs() < 0.01);
    assert_eq!(limits.policy, LimitPolicy::Reject);
}

#[test]
fn t018_parse_trajectory_config() {
    let config = parse_config(FULL_CONFIG).expect("Should parse config");
    
    let home = config.trajectory("home").expect("Home trajectory should exist");
    assert_eq!(home.motor.as_str(), "pan");
    assert_eq!(home.velocity_percent, 50);
}

// =============================================================================
// T019: Unit test for configuration validation
// =============================================================================

#[test]
fn t019_validate_microstep_values() {
    let valid_microsteps = [
        (1, Microsteps::FULL),
        (2, Microsteps::HALF),
        (4, Microsteps::QUARTER),
        (8, Microsteps::EIGHTH),
        (16, Microsteps::SIXTEENTH),
        (32, Microsteps::THIRTY_SECOND),
    ];
    
    for (ms_value, expected) in valid_microsteps {
        let toml = format!(
            r#"
[motors.m1]
name = "Motor"
steps_per_revolution = 200
microsteps = {ms_value}
max_velocity_deg_per_sec = 100.0
max_acceleration_deg_per_sec2 = 200.0
"#
        );
        
        let config = parse_config(&toml).expect(&format!("Microsteps {} should parse", ms_value));
        let motor = config.motor("m1").unwrap();
        assert_eq!(motor.microsteps, expected);
    }
}

#[test]
fn t019_validate_limit_policies() {
    // Note: The TOML field is "mode" but it maps to struct field "policy"
    // Actually checking the config/limits.rs - the serde field is just "policy", no rename
    for (policy_str, expected) in [("clamp", LimitPolicy::Clamp), ("reject", LimitPolicy::Reject)] {
        let toml = format!(
            r#"
[motors.m1]
name = "Motor"
steps_per_revolution = 200
microsteps = 16
max_velocity_deg_per_sec = 100.0
max_acceleration_deg_per_sec2 = 200.0

[motors.m1.limits]
min_degrees = 0.0
max_degrees = 360.0
policy = "{policy_str}"
"#
        );
        
        let config = parse_config(&toml).expect(&format!("Policy '{}' should parse", policy_str));
        let motor = config.motor("m1").unwrap();
        let limits = motor.limits.as_ref().unwrap();
        assert_eq!(limits.policy, expected);
    }
}

// =============================================================================
// T020: Integration test for config loading workflow
// =============================================================================

#[test]
fn t020_config_loading_workflow() {
    // Step 1: Parse configuration
    let config = parse_config(FULL_CONFIG).expect("Config should parse");
    
    // Step 2: Access motor configuration
    let motor_config = config.motor("pan").expect("Motor should exist");
    
    // Step 3: Derive mechanical constraints
    let constraints = MechanicalConstraints::from_config(motor_config);
    
    // Step 4: Verify constraints are correctly calculated
    // 200 base * 32 microsteps * 4.0 gear = 25600 steps/rev
    assert_eq!(constraints.steps_per_revolution, 25600);
    
    // 25600 / 360 ≈ 71.11 steps/degree
    assert!((constraints.steps_per_degree - 71.11).abs() < 0.1);
    
    // Step 5: Access trajectory configuration
    let trajectory = config.trajectory("home").expect("Trajectory should exist");
    assert_eq!(trajectory.motor.as_str(), "pan");
}

#[test]
fn t020_complete_system_config() {
    let config = parse_config(FULL_CONFIG).expect("Config should parse");
    
    // Verify we can iterate motor names
    let motor_names: Vec<_> = config.motor_names().collect();
    assert!(motor_names.contains(&"pan"));
    
    // Verify we can iterate trajectory names
    let trajectory_names: Vec<_> = config.trajectory_names().collect();
    assert!(trajectory_names.contains(&"home"));
    assert!(trajectory_names.contains(&"asymmetric"));
}

// =============================================================================
// T021: Contract test - valid config → parsed struct
// =============================================================================

#[test]
fn t021_contract_valid_config_produces_struct() {
    // Contract: Any valid TOML configuration following our schema
    // MUST produce a valid SystemConfig struct
    
    let config = parse_config(FULL_CONFIG);
    
    // Contract assertion 1: Parsing succeeds
    assert!(config.is_ok(), "Valid config MUST parse successfully");
    
    let config = config.unwrap();
    
    // Contract assertion 2: All declared motors are accessible
    assert!(config.motor("pan").is_some(), "Declared motor MUST be accessible");
    
    // Contract assertion 3: All declared trajectories are accessible
    assert!(config.trajectory("home").is_some(), "Declared trajectory MUST be accessible");
    assert!(config.trajectory("asymmetric").is_some(), "Declared trajectory MUST be accessible");
    
    // Contract assertion 4: Non-existent names return None
    assert!(config.motor("nonexistent").is_none(), "Non-existent motor MUST return None");
    assert!(config.trajectory("nonexistent").is_none(), "Non-existent trajectory MUST return None");
}

// =============================================================================
// T036: Unit test for MechanicalConstraints derivation
// =============================================================================

#[test]
fn t036_mechanical_constraints_derivation() {
    let config = parse_config(MINIMAL_CONFIG).unwrap();
    let motor = config.motor("test_motor").unwrap();
    let constraints = MechanicalConstraints::from_config(motor);
    
    // 200 steps * 16 microsteps = 3200 steps/rev
    assert_eq!(constraints.steps_per_revolution, 3200);
    
    // 3200 / 360 ≈ 8.89 steps/degree
    assert!((constraints.steps_per_degree - 8.889).abs() < 0.01);
    
    // Max velocity: 360 deg/s * 8.889 steps/deg ≈ 3200 steps/s
    assert!((constraints.max_velocity_steps_per_sec - 3200.0).abs() < 1.0);
    
    // Max acceleration: 720 deg/s² * 8.889 steps/deg ≈ 6400 steps/s²
    assert!((constraints.max_acceleration_steps_per_sec2 - 6400.0).abs() < 1.0);
}

#[test]
fn t036_constraints_with_gear_ratio() {
    let config = parse_config(FULL_CONFIG).unwrap();
    let motor = config.motor("pan").unwrap();
    let constraints = MechanicalConstraints::from_config(motor);
    
    // 200 * 32 * 4.0 = 25600 steps/rev
    assert_eq!(constraints.steps_per_revolution, 25600);
}

// =============================================================================
// T037: Unit test for soft limit enforcement
// =============================================================================

#[test]
fn t037_soft_limit_clamp() {
    let limits = SoftLimits::new(Degrees(-90.0), Degrees(90.0), LimitPolicy::Clamp);
    
    // Within range: unchanged
    let result = limits.apply(Degrees(0.0));
    assert!(result.is_some());
    assert!((result.unwrap().0).abs() < 0.01);
    
    let result = limits.apply(Degrees(45.0));
    assert!(result.is_some());
    assert!((result.unwrap().0 - 45.0).abs() < 0.01);
    
    // Below minimum: clamped
    let result = limits.apply(Degrees(-180.0));
    assert!(result.is_some());
    assert!((result.unwrap().0 - (-90.0)).abs() < 0.01);
    
    // Above maximum: clamped
    let result = limits.apply(Degrees(180.0));
    assert!(result.is_some());
    assert!((result.unwrap().0 - 90.0).abs() < 0.01);
}

#[test]
fn t037_soft_limit_reject() {
    let limits = SoftLimits::new(Degrees(-90.0), Degrees(90.0), LimitPolicy::Reject);
    
    // Within range: valid (returns Some)
    assert!(limits.apply(Degrees(0.0)).is_some());
    assert!(limits.apply(Degrees(90.0)).is_some());
    assert!(limits.apply(Degrees(-90.0)).is_some());
    
    // Outside range: rejected (returns None)
    assert!(limits.apply(Degrees(-91.0)).is_none());
    assert!(limits.apply(Degrees(91.0)).is_none());
}

#[test]
fn t037_soft_limit_contains() {
    let limits = SoftLimits::new(Degrees(-90.0), Degrees(90.0), LimitPolicy::Reject);
    
    assert!(limits.contains(Degrees(0.0)));
    assert!(limits.contains(Degrees(-90.0)));
    assert!(limits.contains(Degrees(90.0)));
    assert!(!limits.contains(Degrees(-91.0)));
    assert!(!limits.contains(Degrees(91.0)));
}

// =============================================================================
// T038: Integration test for constraint validation
// =============================================================================

#[test]
fn t038_trajectory_constraint_validation() {
    let config = parse_config(FULL_CONFIG).unwrap();
    let motor = config.motor("pan").unwrap();
    let constraints = MechanicalConstraints::from_config(motor);
    let trajectory = config.trajectory("home").unwrap();
    
    // Validate trajectory against constraints
    let result = trajectory.check_feasibility(&constraints);
    assert!(result.is_ok(), "Home trajectory should be feasible");
}

#[test]
fn t038_velocity_percent_in_bounds() {
    // Test that velocity_percent creates valid velocities
    let toml = r#"
[motors.m1]
name = "Motor"
steps_per_revolution = 200
microsteps = 16
max_velocity_deg_per_sec = 100.0
max_acceleration_deg_per_sec2 = 200.0

[trajectories.t1]
motor = "m1"
target_degrees = 90.0
velocity_percent = 50
"#;
    
    let config = parse_config(toml).unwrap();
    let motor = config.motor("m1").unwrap();
    let constraints = MechanicalConstraints::from_config(motor);
    let trajectory = config.trajectory("t1").unwrap();
    
    // 50% of 100 deg/s = 50 deg/s
    let effective_velocity = trajectory.effective_velocity(&constraints);
    assert!((effective_velocity - 50.0).abs() < 0.01);
    
    // Should be feasible
    assert!(trajectory.check_feasibility(&constraints).is_ok());
}

// =============================================================================
// T049: Unit test for TrajectoryRegistry
// =============================================================================

#[test]
fn t049_trajectory_registry_creation() {
    let config = parse_config(FULL_CONFIG).unwrap();
    let registry = TrajectoryRegistry::from_config(&config);
    
    // Should have 2 trajectories
    assert_eq!(registry.len(), 2);
    assert!(!registry.is_empty());
}

#[test]
fn t049_registry_get_by_name() {
    let config = parse_config(FULL_CONFIG).unwrap();
    let registry = TrajectoryRegistry::from_config(&config);
    
    // Get existing trajectory
    let home = registry.get("home");
    assert!(home.is_some());
    assert_eq!(home.unwrap().motor.as_str(), "pan");
    
    // Get non-existing trajectory
    let missing = registry.get("nonexistent");
    assert!(missing.is_none());
}

// =============================================================================
// T050: Unit test for trajectory lookup by name
// =============================================================================

#[test]
fn t050_lookup_returns_correct_trajectory() {
    let config = parse_config(FULL_CONFIG).unwrap();
    let registry = TrajectoryRegistry::from_config(&config);
    
    let home = registry.get("home").unwrap();
    assert!((home.target_degrees.0).abs() < 0.01);
    assert_eq!(home.velocity_percent, 50);
    
    let asymmetric = registry.get("asymmetric").unwrap();
    assert!((asymmetric.target_degrees.0 - 45.0).abs() < 0.01);
    // Asymmetric uses absolute acceleration/deceleration values
    assert!(asymmetric.acceleration.is_some());
    assert!(asymmetric.deceleration.is_some());
    assert!(asymmetric.is_asymmetric());
}

#[test]
fn t050_get_or_error_with_available_names() {
    let config = parse_config(FULL_CONFIG).unwrap();
    let registry = TrajectoryRegistry::from_config(&config);
    
    // Success case
    let result = registry.get_or_error("home");
    assert!(result.is_ok());
    
    // Error case: should include available names in error
    let result = registry.get_or_error("nonexistent");
    assert!(result.is_err());
    
    // The error message should mention available trajectories
    let err = result.unwrap_err();
    let err_str = format!("{:?}", err);
    // It should list available names (home and/or asymmetric)
    assert!(
        err_str.contains("home") || err_str.contains("asymmetric") || err_str.contains("Available"),
        "Error should list available names: {}", 
        err_str
    );
}

// =============================================================================
// T051: Integration test for named trajectory execution
// =============================================================================

#[test]
fn t051_named_trajectory_execution_flow() {
    let config = parse_config(FULL_CONFIG).unwrap();
    let registry = TrajectoryRegistry::from_config(&config);
    
    // Step 1: Get trajectory by name
    let trajectory = registry.get("asymmetric").unwrap();
    
    // Step 2: Get motor config
    let motor = config.motor(&trajectory.motor).unwrap();
    
    // Step 3: Derive constraints
    let constraints = MechanicalConstraints::from_config(motor);
    
    // Step 4: Check feasibility
    let feasibility = trajectory.check_feasibility(&constraints);
    assert!(feasibility.is_ok());
    
    // Step 5: Calculate effective values for the profile
    let effective_velocity = trajectory.effective_velocity(&constraints);
    let effective_accel = trajectory.effective_acceleration(&constraints);
    let effective_decel = trajectory.effective_deceleration(&constraints);
    
    // Verify asymmetric profile - deceleration should be different from acceleration
    assert!(
        (effective_accel - effective_decel).abs() > 0.01,
        "Asymmetric trajectory should have different accel ({}) vs decel ({})",
        effective_accel,
        effective_decel
    );
    
    // 100% velocity of 180 deg/s
    assert!((effective_velocity - 180.0).abs() < 0.01);
    
    // 80% acceleration of 360 deg/s² = 288 deg/s²
    assert!((effective_accel - 288.0).abs() < 0.1);
    
    // 50% deceleration of 360 deg/s² = 180 deg/s²
    assert!((effective_decel - 180.0).abs() < 0.1);
}

#[test]
fn t051_complete_execution_workflow() {
    let config = parse_config(FULL_CONFIG).unwrap();
    let registry = TrajectoryRegistry::from_config(&config);
    
    // This test verifies the complete workflow from config to execution-ready
    for (name, trajectory) in registry.iter() {
        // Each trajectory must reference a valid motor
        let motor = config.motor(&trajectory.motor);
        assert!(motor.is_some(), "Trajectory '{}' references invalid motor '{}'", name, trajectory.motor);
        
        let motor = motor.unwrap();
        let constraints = MechanicalConstraints::from_config(motor);
        
        // Each trajectory must be feasible with its motor's constraints
        let feasibility = trajectory.check_feasibility(&constraints);
        assert!(
            feasibility.is_ok(),
            "Trajectory '{}' should be feasible: {:?}",
            name,
            feasibility.err()
        );
    }
}
