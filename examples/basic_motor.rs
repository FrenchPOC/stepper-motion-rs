//! Basic motor control example.
//!
//! Demonstrates creating a stepper motor from configuration and executing
//! a simple trajectory with asymmetric acceleration/deceleration.
//!
//! This example uses embedded-hal-mock for testing without real hardware.

use stepper_motion::{
    config::units::{DegreesPerSec, DegreesPerSecSquared, Microsteps},
    motor::StepperMotorBuilder,
    motion::MotionProfile,
};

/// Mock delay provider for demonstration.
struct MockDelay;

impl embedded_hal::delay::DelayNs for MockDelay {
    fn delay_ns(&mut self, ns: u32) {
        // In real code, this would use hardware timer
        std::thread::sleep(std::time::Duration::from_nanos(ns as u64));
    }
}

/// Mock output pin for demonstration.
struct MockPin {
    state: bool,
}

impl MockPin {
    fn new() -> Self {
        Self { state: false }
    }
}

impl embedded_hal::digital::OutputPin for MockPin {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        Ok(())
    }
}

impl embedded_hal::digital::ErrorType for MockPin {
    type Error = core::convert::Infallible;
}

fn main() {
    println!("=== Basic Motor Control Example ===\n");

    // Create mock hardware
    let step_pin = MockPin::new();
    let dir_pin = MockPin::new();
    let delay = MockDelay;

    // Build motor from manual configuration
    let motor = StepperMotorBuilder::new()
        .name("demo_motor")
        .step_pin(step_pin)
        .dir_pin(dir_pin)
        .delay(delay)
        .steps_per_revolution(200)
        .microsteps(Microsteps::SIXTEENTH)
        .gear_ratio(1.0)
        .max_velocity(DegreesPerSec(360.0))
        .max_acceleration(DegreesPerSecSquared(720.0))
        .build()
        .expect("Failed to build motor");

    println!("Motor created: {}", motor.name());
    println!("Initial position: {} steps ({} degrees)",
        motor.position_steps().0,
        motor.position_degrees().0
    );
    println!("State: {}", motor.state_name());

    // Demonstrate motion profile calculation
    let profile = MotionProfile::asymmetric_trapezoidal(
        3200,  // steps (1 full revolution at 16x microstepping)
        3200.0, // max velocity (steps/sec)
        6400.0, // acceleration (steps/sec²)
        3200.0, // deceleration (steps/sec²) - slower decel for smooth stop
    );

    println!("\n=== Motion Profile ===");
    println!("Total steps: {}", profile.total_steps);
    println!("Direction: {:?}", profile.direction);
    println!("Acceleration phase: {} steps", profile.accel_steps);
    println!("Cruise phase: {} steps", profile.cruise_steps);
    println!("Deceleration phase: {} steps", profile.decel_steps);
    println!("Initial interval: {} ns", profile.initial_interval_ns);
    println!("Cruise interval: {} ns", profile.cruise_interval_ns);
    println!("Estimated duration: {:.3} seconds", profile.estimated_duration_secs());

    // Load configuration from TOML (if available)
    println!("\n=== Configuration Loading ===");
    
    let toml_content = r#"
[motors.demo]
name = "demo_motor"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0

[motors.demo.limits]
min_degrees = -180.0
max_degrees = 180.0
policy = "reject"

[trajectories.home]
motor = "demo"
target_degrees = 0.0
velocity_percent = 50
acceleration_deg_per_sec2 = 500.0
deceleration_deg_per_sec2 = 200.0

[trajectories.quarter_turn]
motor = "demo"
target_degrees = 90.0
velocity_percent = 100
"#;

    let config: stepper_motion::SystemConfig = toml::from_str(toml_content)
        .expect("Failed to parse config");

    println!("Loaded configuration with {} motor(s) and {} trajectory(ies)",
        config.motors.len(),
        config.trajectories.len()
    );

    // Validate configuration
    stepper_motion::validate_config(&config).expect("Configuration validation failed");
    println!("Configuration validated successfully!");

    // Display trajectory info
    if let Some(home_traj) = config.trajectory("home") {
        println!("\nTrajectory 'home':");
        println!("  Target: {} degrees", home_traj.target_degrees.0);
        println!("  Velocity: {}% of max", home_traj.velocity_percent);
        println!("  Asymmetric: {}", if home_traj.is_asymmetric() { "yes" } else { "no" });
        if let Some(accel) = home_traj.acceleration {
            println!("  Acceleration: {} deg/s²", accel.0);
        }
        if let Some(decel) = home_traj.deceleration {
            println!("  Deceleration: {} deg/s²", decel.0);
        }
    }

    println!("\n=== Example Complete ===");
    println!("In production code, call motor.move_to(Degrees(90.0)) to execute motion.");
}
