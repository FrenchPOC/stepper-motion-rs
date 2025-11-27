# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

#### Motor System Facade
- `MotorSystem` facade for multi-motor management
- `system.motor("name")` accessor for named motor lookup
- `system.motor_mut("name")` for mutable access
- `motor_names()` iterator for listing all motors
- `trajectories_for_motor()` to filter trajectories by motor
- Constraint and trajectory lookup methods

#### Trajectory Registry Enhancements
- `get_or_error()` method with helpful error messages listing available trajectories
- `check_feasibility()` method on `TrajectoryConfig` for constraint validation

#### Motor Driver Enhancements
- `execute()` method for trajectory execution
- `move_to_blocking()` for synchronous position moves
- `backlash_steps` field with automatic conversion from config degrees

#### Integration Tests
- 21 integration tests covering US1, US2, US3 scenarios
- TOML parsing validation tests (T018-T021)
- Mechanical constraints tests (T036-T038)
- Trajectory registry tests (T049-T051)

## [0.1.0] - 2025-01-13

### Added

#### Configuration System
- TOML-based configuration for motors, trajectories, and sequences
- `SystemConfig` root structure with named motor and trajectory maps
- `MotorConfig` with steps/rev, microsteps, gear ratio, and limits
- `TrajectoryConfig` with target position and motion parameters
- `WaypointTrajectory` for multi-point sequences
- `SoftLimits` with `reject` and `clamp` policies
- Validation for all configuration values

#### Motor Control
- `StepperMotor` driver generic over embedded-hal 1.0 `OutputPin` and `DelayNs`
- Type-state pattern for compile-time state safety: `Idle`, `Moving`, `Homing`, `Fault`
- Builder pattern with `StepperMotorBuilder` for ergonomic construction
- Absolute position tracking in i64 steps (unlimited range)
- Direction inversion support via `invert_direction` config
- Backlash compensation field (placeholder for future implementation)

#### Motion Profiles
- Asymmetric trapezoidal motion with independent acceleration/deceleration rates
- Symmetric trapezoidal motion (equal accel/decel)
- `MotionProfile` struct with phase breakdown (accel, cruise, decel)
- Duration estimation for motion planning
- Step interval calculation for timing

#### Trajectory Management
- `TrajectoryRegistry` for named trajectory lookup
- `TrajectoryBuilder` for programmatic trajectory construction
- Velocity and acceleration as percentage or absolute values
- Dwell time support for sequences

#### Unit Types
- `Degrees` - angular position
- `DegreesPerSec` - angular velocity
- `DegreesPerSecSquared` - angular acceleration
- `Steps` - discrete motor steps
- `Microsteps` - validated power-of-2 microstepping (1-256)
- `MechanicalConstraints` for unit conversions

#### Platform Support
- `no_std` compatible core library
- `std` feature for file I/O and TOML loading
- `alloc` feature for heap allocation without full std
- `defmt` feature for embedded debugging (optional)
- Minimum Rust version: 1.70.0 (embedded-hal 1.0)

#### Documentation & Examples
- Comprehensive rustdoc for all public types
- `basic_motor.rs` example for manual motor control
- `config_driven.rs` example for configuration-based workflow
- `multi_motor.rs` example for multi-axis systems
- `motion.toml` example configuration file
- README with quick start guide

### Dependencies
- `embedded-hal` 1.0 - Hardware abstraction traits
- `serde` 1.0 - Serialization (always with `derive` feature)
- `heapless` 0.8 - Stack-allocated collections for `no_std`
- `toml` 0.8 - TOML parsing (std feature only)
- `libm` 0.2 - Math functions for `no_std` environments

[Unreleased]: https://github.com/your-org/stepper-motion-rs/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/your-org/stepper-motion-rs/releases/tag/v0.1.0
