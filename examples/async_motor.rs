//! Async motor control example.
//!
//! This example demonstrates how to use the async API for stepper motor control
//! with `embassy-time` for non-blocking delays.
//!
//! # Features Required
//!
//! This example requires the `async` feature:
//! ```toml
//! stepper-motion = { version = "0.1", features = ["async"] }
//! ```
//!
//! # Note
//!
//! This is a conceptual example that won't run directly without:
//! - An async executor (like `embassy-executor`)
//! - Real hardware or mocks implementing `embedded-hal` traits
//! - Embassy time driver configuration

// This example requires the async feature
#![cfg(feature = "async")]

use stepper_motion::{
    AsyncMotorSystem, AsyncStepperMotor, AsyncStepperMotorBuilder,
    config::units::{Degrees, DegreesPerSec, DegreesPerSecSquared, Microsteps},
};

// Mock types for demonstration (in real code, use your HAL's types)
mod mock {
    use embedded_hal::digital::{ErrorType, OutputPin};
    use embedded_hal_async::delay::DelayNs;

    /// Mock output pin for demonstration
    pub struct MockPin;

    impl ErrorType for MockPin {
        type Error = core::convert::Infallible;
    }

    impl OutputPin for MockPin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    /// Mock async delay using embassy-time
    pub struct MockDelay;

    impl DelayNs for MockDelay {
        async fn delay_ns(&mut self, ns: u32) {
            embassy_time::Timer::after(embassy_time::Duration::from_nanos(ns as u64)).await;
        }
    }
}

/// Example of creating an async motor directly with builder pattern
async fn direct_motor_example() -> stepper_motion::Result<()> {
    use mock::{MockDelay, MockPin};

    // Build async motor using the builder pattern
    let motor: AsyncStepperMotor<MockPin, MockPin, MockDelay, _> = AsyncStepperMotorBuilder::new()
        .name("demo_motor")
        .step_pin(MockPin)
        .dir_pin(MockPin)
        .delay(MockDelay)
        .steps_per_revolution(200)
        .microsteps(Microsteps::SIXTEENTH)
        .gear_ratio(1.0)
        .max_velocity(DegreesPerSec(360.0))
        .max_acceleration(DegreesPerSecSquared(720.0))
        .build()?;

    // Move to position asynchronously
    // Other async tasks can run during the step delays!
    let motor = match motor.move_to_async(Degrees(90.0)).await {
        Ok(m) => m,
        Err((m, e)) => {
            // Handle error, motor is returned for recovery
            println!("Move failed: {:?}", e);
            m
        }
    };

    // Move to another position
    let motor = match motor.move_to_async(Degrees(180.0)).await {
        Ok(m) => m,
        Err((m, e)) => {
            println!("Move failed: {:?}", e);
            m
        }
    };

    println!("Final position: {:?}", motor.position_degrees());

    Ok(())
}

/// Example of using AsyncMotorSystem with configuration
#[cfg(feature = "std")]
async fn system_motor_example() -> stepper_motion::Result<()> {
    use mock::{MockDelay, MockPin};

    // Configuration TOML
    let config_toml = r#"
[motors.x_axis]
name = "X Axis"
steps_per_revolution = 200
microsteps = 16
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0

[motors.y_axis]
name = "Y Axis"  
steps_per_revolution = 200
microsteps = 8
max_velocity_deg_per_sec = 180.0
max_acceleration_deg_per_sec2 = 360.0

[trajectories.home_x]
motor = "x_axis"
target_degrees = 0.0
velocity_percent = 50

[trajectories.scan_x]
motor = "x_axis"
target_degrees = 180.0
velocity_percent = 100
"#;

    // Parse configuration
    let config: stepper_motion::SystemConfig = toml::from_str(config_toml)
        .map_err(|e| stepper_motion::Error::Config(
            stepper_motion::error::ConfigError::ParseError(
                heapless::String::try_from(e.to_string().as_str()).unwrap_or_default()
            )
        ))?;

    // Create async motor system
    let mut system = AsyncMotorSystem::from_config(config);

    // Register motor with async delay
    let motor = system.register_motor("x_axis", MockPin, MockPin, MockDelay)?;

    // Execute trajectory asynchronously
    let motor = motor.execute_async("home_x", system.trajectories()).await
        .map_err(|(_, e)| e)?;

    println!("After home: {:?}", motor.position_degrees());

    // Execute another trajectory
    let motor = motor.execute_async("scan_x", system.trajectories()).await
        .map_err(|(_, e)| e)?;

    println!("After scan: {:?}", motor.position_degrees());

    Ok(())
}

/// Example showing cooperative multitasking with async motor
async fn cooperative_example() {
    use mock::{MockDelay, MockPin};

    // Build motor using builder pattern
    let motor: AsyncStepperMotor<MockPin, MockPin, MockDelay, _> = AsyncStepperMotorBuilder::new()
        .name("cooperative_motor")
        .step_pin(MockPin)
        .dir_pin(MockPin)
        .delay(MockDelay)
        .steps_per_revolution(200)
        .microsteps(Microsteps::SIXTEENTH)
        .max_velocity(DegreesPerSec(360.0))
        .max_acceleration(DegreesPerSecSquared(720.0))
        .build()
        .unwrap();

    // Start move - returns immediately with Moving state motor
    let moving_motor = match motor.move_to(Degrees(90.0)) {
        Ok(m) => m,
        Err((_, e)) => {
            println!("Failed to start move: {:?}", e);
            return;
        }
    };

    // Run to completion asynchronously
    // During each step's delay, other async tasks can execute
    match moving_motor.run_to_completion_async().await {
        Ok(idle_motor) => {
            println!("Move complete! Position: {:?}", idle_motor.position_degrees());
        }
        Err(e) => {
            println!("Move failed: {:?}", e);
        }
    }
}

// Main function - in real Embassy applications, this would use #[embassy_executor::main]
fn main() {
    println!("Async motor example");
    println!("This example demonstrates the async API.");
    println!("In a real application, you would:");
    println!("1. Use #[embassy_executor::main] as entry point");
    println!("2. Configure the embassy-time driver for your platform");
    println!("3. Use real hardware pins from your HAL");
    println!();
    println!("Example code structure:");
    println!();
    println!(r#"
#[embassy_executor::main]
async fn main(_spawner: Spawner) {{
    let p = embassy_stm32::init(Default::default());
    
    let step_pin = Output::new(p.PA0, Level::Low, Speed::Medium);
    let dir_pin = Output::new(p.PA1, Level::Low, Speed::Medium);
    
    let motor = AsyncStepperMotorBuilder::new()
        .step_pin(step_pin)
        .dir_pin(dir_pin)
        .delay(embassy_time::Delay)
        .steps_per_revolution(200)
        .microsteps(Microsteps::M16)
        .max_velocity(DegreesPerSec(360.0))
        .max_acceleration(DegreesPerSecSquared(720.0))
        .build()
        .unwrap();
    
    // Non-blocking move - other tasks can run during delays
    let motor = motor.move_to_async(Degrees(90.0)).await.unwrap();
}}
"#);
}
