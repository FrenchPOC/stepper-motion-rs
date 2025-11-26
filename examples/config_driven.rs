//! Example: Configuration-driven trajectory execution.
//!
//! This example demonstrates how to:
//! - Load motor and trajectory configuration from TOML
//! - Use the TrajectoryRegistry for named lookups
//! - Calculate motion profiles from configuration
//!
//! Run with: `cargo run --example config_driven --features std`

use stepper_motion::{
    config::{MechanicalConstraints, SystemConfig},
    error::{ConfigError, Error, Result},
    motion::MotionProfile,
    trajectory::TrajectoryRegistry,
};

/// Mock STEP pin for demonstration.
struct MockStepPin;

impl embedded_hal::digital::ErrorType for MockStepPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for MockStepPin {
    fn set_low(&mut self) -> core::result::Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> core::result::Result<(), Self::Error> {
        Ok(())
    }
}

/// Mock DIR pin for demonstration.
struct MockDirPin;

impl embedded_hal::digital::ErrorType for MockDirPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for MockDirPin {
    fn set_low(&mut self) -> core::result::Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> core::result::Result<(), Self::Error> {
        Ok(())
    }
}

/// Mock delay for demonstration.
struct MockDelay;

impl embedded_hal::delay::DelayNs for MockDelay {
    fn delay_ns(&mut self, _ns: u32) {
        // In real code, this would actually delay
    }
}

fn main() -> Result<()> {
    println!("=== Configuration-Driven Trajectory Example ===\n");

    // Sample TOML configuration using the actual SystemConfig structure.
    // Motors and trajectories are indexed by name in FnvIndexMap.
    let toml_content = r#"
[motors.pan_axis]
name = "pan_axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 4.0
max_velocity_deg_per_sec = 180.0
max_acceleration_deg_per_sec2 = 360.0
invert_direction = false

[motors.pan_axis.limits]
min_degrees = -180.0
max_degrees = 180.0
policy = "reject"

# Trajectory configurations use motor name, target_degrees, and percentages
# or absolute acceleration values

[trajectories.home]
motor = "pan_axis"
target_degrees = 0.0
velocity_percent = 50
acceleration_percent = 50

[trajectories.left_90]
motor = "pan_axis"
target_degrees = -90.0
velocity_percent = 100
acceleration_deg_per_sec2 = 180.0
deceleration_deg_per_sec2 = 90.0

[trajectories.right_90]
motor = "pan_axis"
target_degrees = 90.0
velocity_percent = 100
acceleration_deg_per_sec2 = 180.0
deceleration_deg_per_sec2 = 90.0

[trajectories.quick_sweep]
motor = "pan_axis"
target_degrees = 180.0
velocity_percent = 100
acceleration_percent = 100

# Waypoint trajectories list positions to visit
[sequences.demo_pattern]
motor = "pan_axis"
waypoints = [0.0, 90.0, -90.0, 0.0]
dwell_ms = 500
velocity_percent = 75
"#;

    // Parse configuration
    let config: SystemConfig = toml::from_str(toml_content).map_err(|e| {
        // Print full error for debugging
        eprintln!("TOML parse error: {}", e);
        let msg: heapless::String<128> = heapless::String::try_from(e.to_string().as_str())
            .unwrap_or_else(|_| heapless::String::try_from("Parse error").unwrap());
        Error::Config(ConfigError::ParseError(msg))
    })?;

    // Get the first motor for demonstration
    let motor_name = config.motor_names().next().expect("No motors in config");
    let motor_config = config.motor(motor_name).expect("Motor not found");

    println!("Motor Configuration:");
    println!("  Name: {}", motor_config.name);
    println!("  Steps/rev: {}", motor_config.steps_per_revolution);
    println!("  Microsteps: {:?}", motor_config.microsteps);
    println!("  Gear ratio: {}", motor_config.gear_ratio);
    println!(
        "  Max velocity: {} °/s",
        motor_config.max_velocity.value()
    );
    println!(
        "  Max acceleration: {} °/s²",
        motor_config.max_acceleration.value()
    );
    println!("  Total steps/rev: {}", motor_config.total_steps_per_revolution());
    println!("  Steps/degree: {:.2}", motor_config.steps_per_degree());
    println!();

    // Create mechanical constraints from motor config
    let constraints = MechanicalConstraints::from_config(motor_config);

    // Build trajectory registry
    let registry = TrajectoryRegistry::from_config(&config);

    println!("Available Trajectories:");
    for name in registry.names() {
        if let Some(traj) = registry.get(name) {
            let accel = traj.effective_acceleration(&constraints);
            let decel = traj.effective_deceleration(&constraints);
            let velocity = traj.effective_velocity(&constraints);
            let profile_type = if traj.is_asymmetric() {
                "asymmetric"
            } else {
                "symmetric"
            };

            println!(
                "  - {} → {}° @ {:.1}°/s (accel: {:.1}°/s², decel: {:.1}°/s²) [{}]",
                name,
                traj.target_degrees.value(),
                velocity,
                accel,
                decel,
                profile_type
            );
        }
    }
    println!();

    // Demonstrate trajectory lookup and motion profile calculation
    let trajectory_names = ["home", "left_90", "right_90", "quick_sweep"];

    println!("Motion Profile Calculations:");
    println!("{}", "-".repeat(60));

    for name in trajectory_names {
        if let Some(traj) = registry.get(name) {
            // Calculate steps for this move (from position 0)
            let target_steps = constraints.degrees_to_steps(traj.target_degrees.value()).abs() as u32;

            // Get motion parameters
            let velocity_steps = constraints.velocity_to_steps(
                traj.effective_velocity(&constraints),
            );
            let accel_steps = constraints.acceleration_to_steps(
                traj.effective_acceleration(&constraints),
            );
            let decel_steps = constraints.acceleration_to_steps(
                traj.effective_deceleration(&constraints),
            );

            let profile = MotionProfile::asymmetric_trapezoidal(
                target_steps as i64,
                velocity_steps,
                accel_steps,
                decel_steps,
            );

            println!("Trajectory: {}", name);
            println!(
                "  Target: {}° ({} steps)",
                traj.target_degrees.value(),
                target_steps
            );
            println!("  Direction: {:?}", profile.direction);
            println!("  Phases:");
            println!("    - Acceleration: {} steps", profile.accel_steps);
            println!("    - Cruise: {} steps", profile.cruise_steps);
            println!("    - Deceleration: {} steps", profile.decel_steps);
            println!("  Timing:");
            println!(
                "    - Cruise interval: {} µs",
                profile.cruise_interval_ns / 1000
            );
            println!(
                "    - Est. duration: {:.3}s",
                profile.estimated_duration_secs()
            );
            println!();
        }
    }

    // Demonstrate sequence lookup
    println!("Sequences:");
    if !config.sequences.is_empty() {
        for (name, seq) in &config.sequences {
            println!("  Sequence: {}", name);
            println!("  Motor: {}", seq.motor);
            println!("  Waypoints: {} positions", seq.waypoints.len());
            println!("  Dwell time: {}ms between each", seq.dwell_ms);
            println!("  Velocity: {}% of max", seq.velocity_percent);
            print!("  Path: ");
            for (i, wp) in seq.waypoints.iter().enumerate() {
                if i > 0 {
                    print!(" → ");
                }
                print!("{}°", wp.value());
            }
            println!();
        }
    } else {
        println!("  No sequences defined");
    }
    println!();

    // Demonstrate looking up a non-existent trajectory
    let missing_name = "nonexistent";
    match registry.get(missing_name) {
        Some(_) => println!("Found trajectory: {}", missing_name),
        None => println!("Trajectory '{}' not found (expected)", missing_name),
    }

    println!("\n=== Example Complete ===");

    Ok(())
}
