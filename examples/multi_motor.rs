//! Example: Multi-motor configuration.
//!
//! This example demonstrates how to:
//! - Configure multiple motors in a single TOML file
//! - Access motors by name from the configuration
//! - Use different trajectories for different motors
//!
//! Run with: `cargo run --example multi_motor --features std`

use stepper_motion::{
    config::{MechanicalConstraints, SystemConfig},
    error::{ConfigError, Error, Result},
    motion::MotionProfile,
    trajectory::TrajectoryRegistry,
};

fn main() -> Result<()> {
    println!("=== Multi-Motor Configuration Example ===\n");

    // Multi-motor TOML configuration with two axes
    let toml_content = r#"
# X-axis: fast linear motion
[motors.x_axis]
name = "x_axis"
steps_per_revolution = 200
microsteps = 32
gear_ratio = 1.0
max_velocity_deg_per_sec = 720.0
max_acceleration_deg_per_sec2 = 1440.0

[motors.x_axis.limits]
min_degrees = 0.0
max_degrees = 360.0
policy = "clamp"

# Y-axis: precise rotational motion with gearing
[motors.y_axis]
name = "y_axis"
steps_per_revolution = 400
microsteps = 16
gear_ratio = 5.0
max_velocity_deg_per_sec = 90.0
max_acceleration_deg_per_sec2 = 180.0
invert_direction = true

[motors.y_axis.limits]
min_degrees = -45.0
max_degrees = 45.0
policy = "reject"

# Z-axis: vertical motion with backlash compensation
[motors.z_axis]
name = "z_axis"
steps_per_revolution = 200
microsteps = 8
gear_ratio = 2.5
max_velocity_deg_per_sec = 180.0
max_acceleration_deg_per_sec2 = 360.0
backlash_compensation_deg = 0.5

# Trajectories for X-axis
[trajectories.x_home]
motor = "x_axis"
target_degrees = 0.0
velocity_percent = 50
acceleration_percent = 50

[trajectories.x_full_sweep]
motor = "x_axis"
target_degrees = 360.0
velocity_percent = 100
acceleration_deg_per_sec2 = 2880.0
deceleration_deg_per_sec2 = 720.0

# Trajectories for Y-axis (slow, precise)
[trajectories.y_home]
motor = "y_axis"
target_degrees = 0.0
velocity_percent = 25
acceleration_percent = 25

[trajectories.y_tilt_up]
motor = "y_axis"
target_degrees = 30.0
velocity_percent = 50
acceleration_percent = 50

[trajectories.y_tilt_down]
motor = "y_axis"
target_degrees = -30.0
velocity_percent = 50
acceleration_percent = 50

# Trajectories for Z-axis
[trajectories.z_home]
motor = "z_axis"
target_degrees = 0.0
velocity_percent = 100
acceleration_percent = 100

[trajectories.z_up]
motor = "z_axis"
target_degrees = 90.0
velocity_percent = 75
acceleration_deg_per_sec2 = 180.0
deceleration_deg_per_sec2 = 360.0

# Multi-motor sequences
[sequences.all_home]
motor = "x_axis"
waypoints = [0.0]
velocity_percent = 50

[sequences.scan_pattern]
motor = "x_axis"
waypoints = [0.0, 90.0, 180.0, 270.0, 360.0, 270.0, 180.0, 90.0, 0.0]
dwell_ms = 100
velocity_percent = 100
"#;

    // Parse configuration
    let config: SystemConfig = toml::from_str(toml_content).map_err(|e| {
        eprintln!("TOML parse error: {}", e);
        let msg: heapless::String<128> = heapless::String::try_from(e.to_string().as_str())
            .unwrap_or_else(|_| heapless::String::try_from("Parse error").unwrap());
        Error::Config(ConfigError::ParseError(msg))
    })?;

    // List all configured motors
    println!("Configured Motors:");
    println!("{}", "=".repeat(70));

    for motor_name in config.motor_names() {
        if let Some(motor_config) = config.motor(motor_name) {
            let constraints = MechanicalConstraints::from_config(motor_config);

            println!("\n{}: {}", motor_name.to_uppercase(), motor_config.name);
            println!("  Steps/rev: {} × {} microsteps × {} gear = {} total steps/rev",
                motor_config.steps_per_revolution,
                motor_config.microsteps.value(),
                motor_config.gear_ratio,
                motor_config.total_steps_per_revolution()
            );
            println!("  Resolution: {:.4}° per step ({:.2} steps/degree)",
                360.0 / motor_config.total_steps_per_revolution() as f32,
                motor_config.steps_per_degree()
            );
            println!("  Max velocity: {}°/s ({:.0} steps/s)",
                motor_config.max_velocity.value(),
                constraints.max_velocity_steps_per_sec
            );
            println!("  Max accel: {}°/s² ({:.0} steps/s²)",
                motor_config.max_acceleration.value(),
                constraints.max_acceleration_steps_per_sec2
            );
            println!("  Direction inverted: {}", motor_config.invert_direction);

            if let Some(limits) = &motor_config.limits {
                println!("  Limits: {}° to {}° ({:?})",
                    limits.min.value(),
                    limits.max.value(),
                    limits.policy
                );
            }

            if let Some(backlash) = motor_config.backlash_compensation {
                println!("  Backlash compensation: {}°", backlash.value());
            }
        }
    }

    // List trajectories grouped by motor
    println!("\n\nTrajectories by Motor:");
    println!("{}", "=".repeat(70));

    let registry = TrajectoryRegistry::from_config(&config);

    for motor_name in config.motor_names() {
        let motor_config = config.motor(motor_name).unwrap();
        let constraints = MechanicalConstraints::from_config(motor_config);

        println!("\n{}:", motor_name.to_uppercase());

        for traj_name in registry.names() {
            if let Some(traj) = registry.get(traj_name) {
                if traj.motor.as_str() == motor_name {
                    let target_steps = constraints.degrees_to_steps(traj.target_degrees.value()).abs() as u32;
                    let velocity = traj.effective_velocity(&constraints);
                    let accel = traj.effective_acceleration(&constraints);
                    let decel = traj.effective_deceleration(&constraints);

                    let profile = MotionProfile::asymmetric_trapezoidal(
                        target_steps as i64,
                        constraints.velocity_to_steps(velocity),
                        constraints.acceleration_to_steps(accel),
                        constraints.acceleration_to_steps(decel),
                    );

                    let profile_type = if traj.is_asymmetric() { "A" } else { "S" };

                    println!("  {} [{}]: {}° → {:.3}s",
                        traj_name,
                        profile_type,
                        traj.target_degrees.value(),
                        profile.estimated_duration_secs()
                    );
                }
            }
        }
    }

    // Show sequences
    println!("\n\nSequences:");
    println!("{}", "=".repeat(70));

    for (name, seq) in &config.sequences {
        print!("\n{}: {} waypoints on {} | ", name, seq.waypoints.len(), seq.motor);
        for (i, wp) in seq.waypoints.iter().enumerate() {
            if i > 0 {
                print!("→");
            }
            print!("{}°", wp.value());
        }
        println!();
        println!("  Dwell: {}ms, Velocity: {}%", seq.dwell_ms, seq.velocity_percent);
    }

    // Demonstrate motor selection for a coordinated move
    println!("\n\nCoordinated Move Planning:");
    println!("{}", "=".repeat(70));

    // Plan moves for all motors to their home positions
    println!("\nPlanning 'all_home' sequence:");

    let home_trajectories = ["x_home", "y_home", "z_home"];
    let mut total_duration = 0.0f32;

    for traj_name in home_trajectories {
        if let Some(traj) = registry.get(traj_name) {
            if let Some(motor_config) = config.motor(&traj.motor) {
                let constraints = MechanicalConstraints::from_config(motor_config);
                let target_steps = constraints.degrees_to_steps(traj.target_degrees.value()).abs() as u32;

                let profile = MotionProfile::asymmetric_trapezoidal(
                    target_steps as i64,
                    constraints.velocity_to_steps(traj.effective_velocity(&constraints)),
                    constraints.acceleration_to_steps(traj.effective_acceleration(&constraints)),
                    constraints.acceleration_to_steps(traj.effective_deceleration(&constraints)),
                );

                let duration = profile.estimated_duration_secs();
                total_duration = total_duration.max(duration);

                println!("  {} → {} home: {:.3}s ({} steps)",
                    traj_name, traj.motor, duration, target_steps);
            }
        }
    }

    println!("\n  Parallel execution time: {:.3}s", total_duration);
    println!("  (All motors move simultaneously)");

    println!("\n=== Example Complete ===");

    Ok(())
}
