# stepper-motion

A configuration-driven stepper motor motion control library for Rust, designed for embedded systems with `no_std` support.

[![Crates.io](https://img.shields.io/crates/v/stepper-motion.svg)](https://crates.io/crates/stepper-motion)
[![Documentation](https://docs.rs/stepper-motion/badge.svg)](https://docs.rs/stepper-motion)
[![License](https://img.shields.io/crates/l/stepper-motion.svg)](LICENSE)

## Features

- **📁 Configuration-Driven**: Define motor parameters and trajectories in TOML files
- **🔧 Embedded-Ready**: Full `no_std` support with `embedded-hal 1.0` integration
- **⚡ Asymmetric Motion Profiles**: Independent acceleration and deceleration rates
- **🎯 Named Trajectories**: Execute movements by name with registry-based lookup
- **📐 Type-Safe Units**: Physical quantities with compile-time unit checking
- **🛡️ Mechanical Constraints**: Automatic validation against hardware limits
- **📍 Absolute Position Tracking**: i64 step-based position management
- **🔄 Backlash Compensation**: Configurable mechanical play compensation

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
stepper-motion = "0.1"
```

### Feature Flags

| Feature | Default | Description |
|---------|---------|-------------|
| `std` | ✓ | Standard library support, TOML file loading |
| `alloc` | | Heap allocation without full std |
| `defmt` | | `defmt` formatting for embedded debugging |
| `async` | | Async executor support (planned) |

For `no_std` environments:

```toml
[dependencies]
stepper-motion = { version = "0.1", default-features = false, features = ["alloc"] }
```

## Quick Start

### 1. Create a Configuration File

```toml
# motion.toml
[motors.pan_axis]
name = "Pan Axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 4.0
max_velocity_deg_per_sec = 180.0
max_acceleration_deg_per_sec2 = 360.0
invert_direction = false
backlash_compensation_deg = 0.5

[motors.pan_axis.limits]
min_degrees = -180.0
max_degrees = 180.0
policy = "reject"

[trajectories.home]
motor = "pan_axis"
target_degrees = 0.0
velocity_percent = 50

[trajectories.quarter_turn]
motor = "pan_axis"
target_degrees = 90.0
velocity_percent = 100
acceleration_deg_per_sec2 = 360.0
deceleration_deg_per_sec2 = 180.0  # Asymmetric: slower decel
```

### 2. Load and Use in Your Application

```rust
use stepper_motion::{
    SystemConfig, 
    motor::StepperMotorBuilder,
    trajectory::TrajectoryRegistry,
    config::units::Degrees,
};

// Load configuration (requires `std` feature)
fn main() -> Result<(), stepper_motion::Error> {
    // Parse TOML configuration
    let config: SystemConfig = toml::from_str(include_str!("motion.toml"))?;
    
    // Get motor configuration
    let motor_config = config.motor("pan_axis").expect("Motor not found");
    
    // Your hardware pins (implement embedded-hal 1.0 traits)
    let step_pin = MyStepPin::new();
    let dir_pin = MyDirPin::new();
    let delay = MyDelay::new();
    
    // Build motor from configuration
    let motor = StepperMotorBuilder::new()
        .step_pin(step_pin)
        .dir_pin(dir_pin)
        .delay(delay)
        .from_motor_config(motor_config)
        .build()?;
    
    // Load trajectory registry for named lookups
    let registry = TrajectoryRegistry::from_config(&config);
    
    // Get trajectory by name
    let trajectory = registry.get_or_error("quarter_turn")?;
    println!("Target: {}°", trajectory.target_degrees.0);
    
    Ok(())
}
```

### 3. Manual Motor Control (Builder Pattern)

```rust
use stepper_motion::{
    motor::StepperMotorBuilder,
    config::units::{Degrees, DegreesPerSec, DegreesPerSecSquared, Microsteps},
};

// Create motor with explicit parameters
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
    .backlash_steps(10)  // Optional: backlash compensation
    .build()?;

println!("Motor: {}", motor.name());
println!("Position: {} steps ({} degrees)", 
    motor.position_steps().0, 
    motor.position_degrees().0);

// Move to absolute position
let moving_motor = motor.move_to(Degrees(90.0))?;

// Execute step-by-step
while moving_motor.is_moving() {
    moving_motor.step()?;
}

let idle_motor = moving_motor.finish();
```

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                    stepper-motion                   │
├─────────────────────────────────────────────────────┤
│  config/          │ TOML parsing, validation        │
│  ├── motor.rs     │ MotorConfig, limits             │
│  ├── trajectory.rs│ TrajectoryConfig (asymmetric)   │
│  ├── mechanical.rs│ MechanicalConstraints           │
│  ├── limits.rs    │ SoftLimits, LimitPolicy         │
│  └── units.rs     │ Degrees, Steps, Microsteps      │
├─────────────────────────────────────────────────────┤
│  motor/           │ Hardware abstraction            │
│  ├── driver.rs    │ StepperMotor<STEP,DIR,DELAY,S>  │
│  ├── builder.rs   │ Builder pattern construction    │
│  ├── state.rs     │ Type-state: Idle, Moving, etc.  │
│  └── position.rs  │ Position tracking (i64 steps)   │
├─────────────────────────────────────────────────────┤
│  motion/          │ Motion planning                 │
│  ├── profile.rs   │ MotionProfile (trapezoidal)     │
│  └── executor.rs  │ Step pulse generation           │
├─────────────────────────────────────────────────────┤
│  trajectory/      │ Named trajectory management     │
│  └── registry.rs  │ TrajectoryRegistry              │
└─────────────────────────────────────────────────────┘
```

## Motion Profiles

### Symmetric Trapezoidal

```
velocity
    ▲
max ┤    ┌───────┐
    │   /         \
    │  /           \
    └─/─────────────\────► time
      accel  cruise  decel
      (same rate for both)
```

### Asymmetric Trapezoidal

```
velocity
    ▲
max ┤    ┌───────┐
    │   /│       │\
    │  / │       │ \
    └─/──┴───────┴──\───► time
     fast         slow
     accel        decel
```

Set different rates in TOML:

```toml
[trajectories.gentle_stop]
motor = "pan_axis"
target_degrees = 90.0
velocity_percent = 100
acceleration_deg_per_sec2 = 720.0   # Fast acceleration
deceleration_deg_per_sec2 = 180.0   # Gentle deceleration
```

Or using percent-based values (relative to motor max):

```toml
[trajectories.smooth_move]
motor = "pan_axis"
target_degrees = 45.0
velocity_percent = 75      # 75% of motor's max velocity
acceleration_percent = 100 # 100% of motor's max acceleration
```

## Mechanical Constraints

Define hardware limits to prevent damage:

```toml
[motors.servo]
name = "Servo Axis"
steps_per_revolution = 200
microsteps = 32
gear_ratio = 5.0  # 5:1 reduction gearbox
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0
backlash_compensation_deg = 0.5  # Compensate 0.5° backlash on reversal

[motors.servo.limits]
min_degrees = -360.0
max_degrees = 360.0
policy = "reject"  # or "clamp"
```

### Limit Policies

- **`reject`**: Return error if target position exceeds limits
- **`clamp`**: Automatically constrain target to nearest limit

### Unit Conversions

The library automatically handles conversions:

```rust
let constraints = motor.constraints();

// Configuration values → internal steps
println!("Steps/revolution: {}", constraints.steps_per_revolution);
println!("Steps/degree: {:.4}", constraints.steps_per_degree);
println!("Max velocity: {:.0} steps/s", constraints.max_velocity_steps_per_sec);
```

## Examples

Run the included examples:

```bash
# Basic motor control with mechanical constraints demonstration
cargo run --example basic_motor

# Configuration-driven operation with named trajectories
cargo run --example config_driven

# Multi-motor system demonstration
cargo run --example multi_motor
```

## Type-State Safety

The motor uses Rust's type system to enforce valid state transitions:

```rust
// Motor starts in Idle state
let motor: StepperMotor<_, _, _, Idle> = builder.build()?;

// move_to() transitions to Moving state
let moving: StepperMotor<_, _, _, Moving> = motor.move_to(Degrees(90.0))?;

// Can only call step() or finish() on Moving motor
while moving.is_moving() {
    moving.step()?;
}

// finish() transitions back to Idle
let motor: StepperMotor<_, _, _, Idle> = moving.finish();
```

## Minimum Supported Rust Version (MSRV)

Rust 1.70.0 or later (required for `embedded-hal 1.0`).

## no_std Usage

For embedded systems without standard library:

```rust
#![no_std]
#![no_main]

use stepper_motion::{SystemConfig, motor::StepperMotorBuilder};

// Embed configuration at compile time
const CONFIG_TOML: &str = include_str!("../motion.toml");

fn setup() {
    // Parse embedded configuration
    let config: SystemConfig = toml::from_str(CONFIG_TOML).unwrap();
    
    let motor_config = config.motor("servo").unwrap();
    
    // Build motor with your embedded-hal pins
    let motor = StepperMotorBuilder::new()
        .step_pin(gpioa.pa0.into_push_pull_output())
        .dir_pin(gpioa.pa1.into_push_pull_output())
        .delay(timer.delay_us())
        .from_motor_config(motor_config)
        .build()
        .unwrap();
}
```

## Contributing

Contributions are welcome! Please read the [CHANGELOG](CHANGELOG.md) for version history.

### Development

```bash
# Run all tests (46 tests: 25 unit + 21 integration)
cargo test --all-features

# Check no_std compatibility
cargo build --no-default-features
cargo build --no-default-features --features alloc

# Run clippy
cargo clippy --all-features

# Format code
cargo fmt

# Run examples
cargo run --example basic_motor
cargo run --example config_driven
cargo run --example multi_motor
```

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
