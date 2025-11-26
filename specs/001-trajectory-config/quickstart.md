# Quickstart: stepper-motion-rs

**Feature**: 001-trajectory-config  
**Date**: 2025-11-26

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
stepper-motion = "0.1"

# For embedded (no_std):
# stepper-motion = { version = "0.1", default-features = false, features = ["alloc"] }
```

## Example 1: Manual Motor Control (No Configuration)

The simplest usage — directly control a motor without configuration files:

```rust
use stepper_motion::{StepperMotor, Direction, Degrees};
use your_hal::{Delay, OutputPin};  // Your platform's HAL

fn main() -> Result<(), stepper_motion::Error> {
    // Get pins from your HAL
    let step_pin = /* ... */;
    let dir_pin = /* ... */;
    let delay = Delay::new();

    // Create motor with mechanical parameters
    let mut motor = StepperMotor::builder()
        .step_pin(step_pin)
        .dir_pin(dir_pin)
        .delay(delay)
        .steps_per_revolution(200)
        .microsteps(16)
        .max_velocity(360.degrees_per_sec())
        .max_acceleration(720.degrees_per_sec_squared())
        .build()?;

    // Move to absolute position
    motor.move_to(Degrees(90.0))?;
    
    // Execute the move (blocking)
    while motor.is_moving() {
        motor.step()?;
    }

    println!("Current position: {}°", motor.position_degrees());
    Ok(())
}
```

## Example 2: Configuration-Driven (Recommended)

Define motors and trajectories in `motion.toml`:

```toml
[motors.x_axis]
name = "X-Axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0

[motors.x_axis.limits]
min_degrees = -180.0
max_degrees = 180.0

[trajectories.home]
motor = "x_axis"
target_degrees = 0.0
velocity_percent = 50

[trajectories.work_position]
motor = "x_axis"
target_degrees = 90.0
velocity_percent = 100

[trajectories.far_end]
motor = "x_axis"
target_degrees = 568.0  # Will be validated against limits!
```

Use in code:

```rust
use stepper_motion::{MotorSystem, SystemConfig};

fn main() -> Result<(), stepper_motion::Error> {
    // Load configuration
    let config: SystemConfig = stepper_motion::load_config("motion.toml")?;
    
    // Create motor system with your pins
    let mut system = MotorSystem::from_config(config)
        .with_motor("x_axis", step_pin, dir_pin)?
        .build(delay)?;

    // Execute named trajectory — motor moves to 0° at 50% speed
    system.motor("x_axis")?.execute("home")?;
    
    // Execute another trajectory — motor moves to 90° at full speed
    system.motor("x_axis")?.execute("work_position")?;

    // Query current state
    let motor = system.motor("x_axis")?;
    println!("Position: {}°", motor.position_degrees());
    println!("State: {:?}", motor.state());

    Ok(())
}
```

## Example 3: Embedded (no_std)

For microcontrollers without a filesystem:

```rust
#![no_std]
#![no_main]

use stepper_motion::{StepperMotor, SystemConfig};
use cortex_m_rt::entry;
use stm32f4xx_hal::{pac, prelude::*, gpio::*, timer::*};

// Embed configuration at compile time
const CONFIG_TOML: &str = include_str!("../motion.toml");

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let gpioa = dp.GPIOA.split();
    
    let step_pin = gpioa.pa0.into_push_pull_output();
    let dir_pin = gpioa.pa1.into_push_pull_output();
    let delay = dp.TIM2.delay_us(&clocks);

    // Parse embedded configuration
    let config: SystemConfig = stepper_motion::parse_config(CONFIG_TOML).unwrap();
    
    let mut motor = StepperMotor::from_config(
        &config.motors["x_axis"],
        step_pin,
        dir_pin,
        delay,
    ).unwrap();

    loop {
        // Execute trajectory
        motor.execute_trajectory(&config.trajectories["home"]).unwrap();
        delay.delay_ms(1000u32);
        
        motor.execute_trajectory(&config.trajectories["work_position"]).unwrap();
        delay.delay_ms(1000u32);
    }
}
```

## Example 4: Multiple Motors

```toml
# motion.toml
[motors.x_axis]
name = "X-Axis"
steps_per_revolution = 200
microsteps = 16
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0

[motors.y_axis]
name = "Y-Axis"
steps_per_revolution = 400  # Different motor
microsteps = 8
max_velocity_deg_per_sec = 180.0
max_acceleration_deg_per_sec2 = 360.0

[trajectories.origin]
motor = "x_axis"
target_degrees = 0.0

[trajectories.y_home]
motor = "y_axis"
target_degrees = 0.0

[trajectories.work_xy]
motor = "x_axis"
target_degrees = 45.0
# Note: For coordinated multi-axis, use sequences
```

```rust
use stepper_motion::MotorSystem;

fn main() -> Result<(), stepper_motion::Error> {
    let config = stepper_motion::load_config("motion.toml")?;
    
    let mut system = MotorSystem::from_config(config)
        .with_motor("x_axis", x_step, x_dir)?
        .with_motor("y_axis", y_step, y_dir)?
        .build(delay)?;

    // Move X axis
    system.motor("x_axis")?.execute("origin")?;
    
    // Move Y axis
    system.motor("y_axis")?.execute("y_home")?;

    Ok(())
}
```

## Example 5: Runtime Position Queries

```rust
let motor = system.motor("x_axis")?;

// Get current position in various units
let degrees = motor.position_degrees();      // f32
let steps = motor.position_steps();          // i64
let revolutions = motor.position_revolutions(); // f32

// Get motor state
match motor.state() {
    MotorState::Idle => println!("Ready for commands"),
    MotorState::Moving { remaining_steps } => {
        println!("Moving, {} steps remaining", remaining_steps);
    }
    MotorState::Fault(err) => println!("Error: {:?}", err),
}

// Check if at a limit
if motor.at_limit() {
    println!("Motor is at a soft limit");
}
```

## Example 6: Emergency Stop

```rust
// During motion, you can emergency stop:
motor.emergency_stop()?;

// Position is preserved, but motor enters Idle state immediately
// Motion profile is discarded
```

## Common Patterns

### Converting Between Units

```rust
use stepper_motion::units::*;

// From configuration
let velocity = 360.degrees_per_sec();
let accel = 720.degrees_per_sec_squared();

// Positions
let target = Degrees(90.0);
let relative = Degrees(-45.0);  // Negative for reverse

// The motor handles all conversions internally
motor.move_to(Degrees(568.0))?;  // Absolute positioning
motor.move_by(Degrees(-180.0))?; // Relative move
```

### Error Handling

```rust
match motor.execute("unknown_trajectory") {
    Ok(()) => println!("Move complete"),
    Err(MotorError::TrajectoryNotFound(name)) => {
        println!("No trajectory named '{}'", name);
    }
    Err(MotorError::LimitExceeded { requested, limit }) => {
        println!("Cannot move to {}°, limit is {}°", requested, limit);
    }
    Err(e) => println!("Motor error: {:?}", e),
}
```

## Next Steps

1. See [Configuration Reference](./contracts/config-schema.md) for full TOML options
2. See [Data Model](./data-model.md) for type details
3. See [Research](./research.md) for implementation notes
