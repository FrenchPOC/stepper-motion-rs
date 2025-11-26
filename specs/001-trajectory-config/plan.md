# Implementation Plan: Trajectory Configuration System

**Branch**: `001-trajectory-config` | **Date**: 2025-11-26 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-trajectory-config/spec.md`

## Summary

A `no_std`-compatible Rust library for stepper motor motion control that:
1. Loads motor configurations and named trajectories from TOML files
2. Uses `embedded-hal` 1.0 `OutputPin` traits for STEP and DIR pin control
3. Supports microstep configuration per motor
4. Tracks motor state and absolute position at all times
5. Enables calling named trajectories (e.g., `motor1.execute("home")` → moves to configured position)
6. **Supports independent acceleration and deceleration rates per trajectory** for asymmetric motion profiles

**Core Innovation**: Configuration-driven motion profiles with compile-time type safety and zero-cost abstractions.

## Technical Context

**Language/Version**: Rust 1.70+ (MSRV for embedded-hal 1.0 compatibility)  
**Primary Dependencies**:
- `embedded-hal` 1.0 — GPIO traits (OutputPin for STEP/DIR)
- `embedded-hal` 1.0 — DelayNs trait for step timing
- `serde` 1.0 (no_std, derive feature) — Configuration deserialization
- `toml` or `serde-json-core` — TOML/JSON parsing (std feature gate)
- `heapless` — no_std collections (Vec, String alternatives)

**Storage**: TOML configuration files (loaded at init, not runtime)  
**Testing**: `cargo test --all-features` + embedded-hal-mock for hardware simulation  
**Target Platform**: `no_std` + `alloc` (embedded MCUs), optional `std` for file I/O  
**Project Type**: Single library crate with optional feature flags  
**Performance Goals**: <1µs per step pulse generation, <10ms config parsing  
**Constraints**: Zero heap allocation in motion-critical paths, deterministic timing  
**Scale/Scope**: 1-8 motors typical, 100+ named trajectories per config

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Ergonomic API Design | ✅ PASS | Builder patterns for Motor/Trajectory, type-states for motor state |
| II. Zero-Cost Abstractions | ✅ PASS | Generics over embedded-hal traits, `#[inline]` on hot paths, no_std core |
| III. Fearless Concurrency & Safety | ✅ PASS | Motor owns pins exclusively, no unsafe in public API |
| IV. Test-Driven Development | ✅ PASS | TDD with embedded-hal-mock, property tests for motion math |
| V. Semantic Versioning | ✅ PASS | 0.1.0 initial release, CHANGELOG.md from day one |

## Project Structure

### Documentation (this feature)

```text
specs/001-trajectory-config/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── config-schema.md # TOML configuration contract
└── tasks.md             # Phase 2 output (/speckit.tasks command)
```

### Source Code (repository root)

```text
src/
├── lib.rs               # Public API re-exports, feature gates
├── config/
│   ├── mod.rs           # Configuration module
│   ├── motor.rs         # MotorConfig deserialization
│   ├── trajectory.rs    # TrajectoryConfig deserialization
│   ├── mechanical.rs    # MechanicalConstraints
│   └── units.rs         # Unit types (Degrees, Steps, etc.)
├── motor/
│   ├── mod.rs           # Motor module
│   ├── driver.rs        # StepperMotor<STEP, DIR, State> driver
│   ├── state.rs         # Type-state definitions (Idle, Moving, Error)
│   └── position.rs      # Position tracking (absolute/relative)
├── motion/
│   ├── mod.rs           # Motion module
│   ├── profile.rs       # Asymmetric trapezoidal motion profiles
│   ├── planner.rs       # Step timing calculation
│   └── executor.rs      # Step pulse generation
├── trajectory/
│   ├── mod.rs           # Trajectory module
│   ├── registry.rs      # Named trajectory storage
│   └── builder.rs       # Trajectory builder API
└── error.rs             # Error types

tests/
├── integration/
│   ├── config_loading.rs
│   ├── motor_movement.rs
│   └── trajectory_execution.rs
└── unit/
    ├── motion_math.rs
    ├── position_tracking.rs
    └── config_validation.rs

examples/
├── basic_motor.rs       # Minimal motor setup
├── config_driven.rs     # Full config file usage
└── multi_motor.rs       # Multiple motors from config
```

**Structure Decision**: Single library crate with modular internal organization. Feature flags control `std` vs `no_std` compilation.

## Feature Flags

```toml
[features]
default = ["std"]
std = ["serde/std", "toml"]           # File I/O, full TOML parsing
alloc = ["serde/alloc"]               # Heap for no_std with allocator
defmt = ["dep:defmt"]                 # Embedded logging
async = ["embedded-hal-async"]        # Async step generation
```

## Complexity Tracking

> No constitution violations. Design follows all principles.

---

## Phase 0: Research Complete

See [research.md](./research.md) for detailed findings.

### Key Technical Decisions

1. **embedded-hal 1.0**: Use `OutputPin::set_high()`/`set_low()` for STEP pulse, `OutputPin::set_state()` for DIR
2. **Delay Trait**: Use `DelayNs::delay_ns()` for precise step timing (nanosecond resolution)
3. **Configuration**: TOML with `serde` derive, `heapless::String` for no_std names
4. **Motion Profile**: **Asymmetric trapezoidal** — independent acceleration and deceleration rates
5. **Position Tracking**: i64 steps from origin, converted to user units on demand

### Asymmetric Acceleration/Deceleration

Each trajectory can specify independent acceleration and deceleration rates:

```toml
[trajectories.gentle_stop]
motor = "x_axis"
target_degrees = 90.0
velocity_percent = 100
acceleration_deg_per_sec2 = 500.0   # Fast acceleration
deceleration_deg_per_sec2 = 200.0   # Gentle deceleration
```

**Use Cases**:
- **Gentle stops**: Fast acceleration, slow deceleration to reduce vibration at target
- **Quick starts**: Slow acceleration (reduce torque), fast deceleration (emergency-like)
- **Asymmetric loads**: Different inertia in each direction (gravity, counterweights)

### Suggested Additional Features

Based on the user's requirements and common stepper motor use cases:

1. **Homing Sequence**: Optional limit switch support for finding origin
2. **Soft Limits**: Configurable min/max position bounds with rejection policy
3. **Speed Override**: Runtime velocity scaling (0-200%) without reconfiguration  
4. **Emergency Stop**: Immediate halt with position preservation
5. **Position Persistence**: Optional save/restore of position across reboots (requires external storage trait)
6. **Backlash Compensation**: Configurable backlash correction on direction reversal
7. **Jog Mode**: Manual step-by-step movement for setup/calibration
