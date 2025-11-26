# stepper-motion-rs

A configuration-driven stepper motor motion control library for Rust, designed for embedded systems with `no_std` support.

[![Crates.io](https://img.shields.io/crates/v/stepper-motion-rs.svg)](https://crates.io/crates/stepper-motion-rs)
[![Documentation](https://docs.rs/stepper-motion-rs/badge.svg)](https://docs.rs/stepper-motion-rs)
[![License](https://img.shields.io/crates/l/stepper-motion-rs.svg)](LICENSE)

## Features

- **📁 Configuration-Driven**: Define motor parameters and trajectories in TOML files
- **🔧 Embedded-Ready**: Full `no_std` support with `embedded-hal 1.0` integration
- **⚡ Asymmetric Motion Profiles**: Independent acceleration and deceleration rates
- **🎯 Named Trajectories**: Execute movements by name with registry-based lookup
- **📐 Type-Safe Units**: Physical quantities with compile-time unit checking
- **🛡️ Mechanical Constraints**: Automatic validation against hardware limits
- **📍 Absolute Position Tracking**: i64 step-based position management

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
stepper-motion-rs = "0.1"
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
stepper-motion-rs = { version = "0.1", default-features = false, features = ["alloc"] }
```

## Quick Start

### 1. Create a Configuration File

```toml
# motion.toml
[motor]
name = "main_axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0
invert_direction = false

[motor.limits]
min_position = -180.0
max_position = 180.0
policy = "reject"

[[trajectories]]
name = "home"
target_position_deg = 0.0
velocity_deg_per_sec = 90.0
acceleration_deg_per_sec2 = 180.0

[[trajectories]]
name = "quarter_turn"
target_position_deg = 90.0
velocity_deg_per_sec = 180.0
acceleration_deg_per_sec2 = 360.0
deceleration_deg_per_sec2 = 180.0  # Asymmetric: slower decel
```

### 2. Load and Use in Your Application

```rust
use stepper_motion_rs::{
    config::{SystemConfig, MechanicalConstraints},
    motor::StepperMotorBuilder,
    trajectory::TrajectoryRegistry,
    units::Degrees,
};
use embedded_hal::digital::OutputPin;
use embedded_hal::delay::DelayNs;

// Load configuration (requires `std` feature)
#[cfg(feature = "std")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = SystemConfig::from_toml_file("motion.toml")?;
    
    // Your hardware pins (implement embedded-hal traits)
    let step_pin = MyStepPin::new();
    let dir_pin = MyDirPin::new();
    let delay = MyDelay::new();
    
    // Build motor from configuration
    let motor = StepperMotorBuilder::new(step_pin, dir_pin, delay)
        .from_motor_config(&config.motor)?
        .build();
    
    // Load trajectory registry
    let registry = TrajectoryRegistry::from_config(&config)?;
    
    // Get named trajectory
    if let Some(trajectory) = registry.get("quarter_turn") {
        println!("Executing: {} to {}°", 
            trajectory.name, 
            trajectory.target_position_deg);
    }
    
    Ok(())
}
```

### 3. Manual Motor Control

```rust
use stepper_motion_rs::motor::{StepperMotor, Idle};
use stepper_motion_rs::units::{Degrees, Microsteps};

// Create motor with explicit parameters
let mut motor = StepperMotor::new(step_pin, dir_pin, delay)
    .with_steps_per_rev(200)
    .with_microsteps(Microsteps::X16)
    .with_gear_ratio(1.0);

// Move to absolute position
motor.move_to(Degrees::new(90.0))?;

// Check current position
let position = motor.position();
println!("Current position: {} steps", position.steps());
```

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                    stepper-motion-rs                │
├─────────────────────────────────────────────────────┤
│  config/          │ TOML parsing, validation        │
│  ├── motor.rs     │ MotorConfig, limits             │
│  ├── trajectory.rs│ TrajectoryConfig (asymmetric)   │
│  ├── mechanical.rs│ MechanicalConstraints           │
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
│  ├── registry.rs  │ TrajectoryRegistry              │
│  └── builder.rs   │ Trajectory builder API          │
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

Set different rates:

```toml
[[trajectories]]
name = "gentle_stop"
target_position_deg = 90.0
velocity_deg_per_sec = 180.0
acceleration_deg_per_sec2 = 720.0   # Fast acceleration
deceleration_deg_per_sec2 = 180.0   # Gentle deceleration
```

## Mechanical Constraints

Define hardware limits to prevent damage:

```toml
[motor]
steps_per_revolution = 200
microsteps = 16
gear_ratio = 5.0  # 5:1 reduction
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0

[motor.limits]
min_position = -360.0  # degrees
max_position = 360.0
policy = "reject"      # or "clamp"
```

### Limit Policies

- **`reject`**: Return error if trajectory exceeds limits
- **`clamp`**: Automatically constrain values to limits

## Examples

Run the examples:

```bash
# Basic motor control
cargo run --example basic_motor --features std

# Configuration-driven operation
cargo run --example config_driven --features std
```

## Minimum Supported Rust Version (MSRV)

Rust 1.70.0 or later (required for `embedded-hal 1.0`).

## no_std Usage

For embedded systems without standard library:

```rust
#![no_std]
#![no_main]

use stepper_motion_rs::{
    config::SystemConfig,
    motor::StepperMotorBuilder,
};

// Parse embedded configuration (no file loading)
const CONFIG_TOML: &str = include_str!("../motion.toml");

fn setup() {
    // Manual parsing required in no_std
    // Use serde with heapless strings
}
```

## Contributing

Contributions are welcome! Please read the [CHANGELOG](CHANGELOG.md) for version history and the development guidelines.

### Development

```bash
# Run tests
cargo test --all-features

# Check no_std compatibility
cargo check --no-default-features --features alloc

# Run clippy
cargo clippy --all-features

# Format code
cargo fmt
```

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
