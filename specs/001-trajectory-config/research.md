# Research: Trajectory Configuration System

**Feature**: 001-trajectory-config  
**Date**: 2025-11-26

## 1. embedded-hal 1.0 Integration

### OutputPin Trait (STEP/DIR Control)

From `embedded-hal` 1.0, the `OutputPin` trait provides:

```rust
pub trait OutputPin: ErrorType {
    fn set_low(&mut self) -> Result<(), Self::Error>;
    fn set_high(&mut self) -> Result<(), Self::Error>;
    fn set_state(&mut self, state: PinState) -> Result<(), Self::Error>;
}
```

**Usage for Stepper Motors**:
- **STEP pin**: Pulse generation via `set_high()` → delay → `set_low()`
- **DIR pin**: Direction control via `set_state(PinState::High)` for CW, `set_state(PinState::Low)` for CCW

### DelayNs Trait (Step Timing)

```rust
pub trait DelayNs {
    fn delay_ns(&mut self, ns: u32);
    fn delay_us(&mut self, us: u32) { self.delay_ns(us * 1000) }
    fn delay_ms(&mut self, ms: u32) { self.delay_ns(ms * 1_000_000) }
}
```

**Key Insight**: Nanosecond precision is critical for high-speed stepping. At 100,000 steps/sec, each step period is 10µs = 10,000ns.

### Generic Driver Pattern

```rust
pub struct StepperMotor<STEP, DIR, DELAY, STATE>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: DelayNs,
{
    step_pin: STEP,
    dir_pin: DIR,
    delay: DELAY,
    _state: PhantomData<STATE>,
}
```

## 2. Serde no_std Configuration

### Cargo.toml Setup

```toml
[dependencies]
serde = { version = "1.0", default-features = false, features = ["derive"] }

# For std builds only:
[target.'cfg(feature = "std")'.dependencies]
toml = "0.8"

# For no_std with alloc:
[target.'cfg(not(feature = "std"))'.dependencies]
serde-json-core = "0.5"  # Alternative: parse pre-compiled config
```

### heapless for no_std Strings

```rust
use heapless::String;

#[derive(Deserialize)]
pub struct TrajectoryConfig {
    pub name: String<32>,  // Max 32 chars, stack-allocated
    pub target_degrees: f32,
    pub max_velocity: f32,
}
```

## 3. Motion Profile Mathematics

### Trapezoidal Profile

Three phases: acceleration, cruise, deceleration.

```
velocity
    ^
    |     ___________
    |    /           \
    |   /             \
    |  /               \
    | /                 \
    +-----------------------> time
      t_acc   t_cruise  t_dec
```

**Equations**:
- Distance during acceleration: `d_acc = 0.5 * a * t_acc²`
- Distance during cruise: `d_cruise = v_max * t_cruise`
- Distance during deceleration: `d_dec = v_max * t_dec - 0.5 * a * t_dec²`

### Step Timing Calculation

For trapezoidal acceleration, step intervals decrease during acceleration:

```rust
fn step_interval_us(step_number: u32, acceleration: f32) -> u32 {
    // Simplified: t_n = t_0 * sqrt(n) / sqrt(n+1)
    // More accurate: use Bresenham-style integer math
}
```

## 4. Microstep Configuration

Common microstep modes and their step multipliers:

| Mode | Multiplier | Typical Use |
|------|------------|-------------|
| Full Step | 1× | High torque, low resolution |
| Half Step | 2× | Balance |
| 1/4 Step | 4× | Smoother motion |
| 1/8 Step | 8× | Standard precision |
| 1/16 Step | 16× | High precision |
| 1/32 Step | 32× | Ultra-smooth |
| 1/256 Step | 256× | Maximum resolution |

**Configuration Impact**:
- `steps_per_revolution = base_steps * microstep_multiplier`
- Example: 200-step motor with 1/16 microstepping = 3200 steps/rev

## 5. Position Tracking Strategy

### Absolute vs Relative

```rust
pub struct Position {
    /// Absolute position in microsteps from origin
    steps: i64,
    /// Cached conversion factors
    steps_per_degree: f32,
    steps_per_mm: Option<f32>,
}

impl Position {
    pub fn as_degrees(&self) -> f32 {
        self.steps as f32 / self.steps_per_degree
    }
    
    pub fn move_to_absolute(&mut self, target_degrees: f32) -> i64 {
        let target_steps = (target_degrees * self.steps_per_degree) as i64;
        let delta = target_steps - self.steps;
        self.steps = target_steps;
        delta  // Steps to move (negative = reverse)
    }
}
```

### State Machine

```rust
pub struct Idle;
pub struct Moving;
pub struct Homing;
pub struct Error;

impl<STEP, DIR, DELAY> StepperMotor<STEP, DIR, DELAY, Idle> {
    pub fn start_move(self, steps: i64) -> StepperMotor<STEP, DIR, DELAY, Moving> {
        // Transition to Moving state
    }
}

impl<STEP, DIR, DELAY> StepperMotor<STEP, DIR, DELAY, Moving> {
    pub fn step(&mut self) -> Result<bool, MotorError> {
        // Returns Ok(true) if more steps remain, Ok(false) if complete
    }
    
    pub fn complete(self) -> StepperMotor<STEP, DIR, DELAY, Idle> {
        // Transition back to Idle
    }
}
```

## 6. Configuration File Format

### TOML Structure (Recommended)

```toml
[motors.motor1]
name = "X-Axis"
steps_per_revolution = 200
microsteps = 16
gear_ratio = 1.0
max_velocity_deg_per_sec = 360.0
max_acceleration_deg_per_sec2 = 720.0
invert_direction = false

[motors.motor1.limits]
min_degrees = -180.0
max_degrees = 180.0
soft_limit_policy = "reject"  # or "clamp"

[trajectories.home]
motor = "motor1"
target_degrees = 0.0
velocity_percent = 50

[trajectories.work_position]
motor = "motor1"
target_degrees = 90.0
velocity_percent = 100
acceleration_percent = 80

[trajectories.scan_sequence]
motor = "motor1"
waypoints = [0.0, 45.0, 90.0, 135.0, 180.0]
dwell_ms = 500
```

## 7. Error Handling Strategy

```rust
#[derive(Debug)]
pub enum MotorError<E> {
    /// Pin operation failed
    Pin(E),
    /// Trajectory exceeds mechanical limits
    LimitExceeded { requested: f32, limit: f32 },
    /// Configuration validation failed
    Config(ConfigError),
    /// Motor is in wrong state for operation
    InvalidState(&'static str),
    /// Trajectory not found in registry
    TrajectoryNotFound(heapless::String<32>),
}

#[derive(Debug)]
pub enum ConfigError {
    Parse(ParseErrorKind),
    Validation(ValidationError),
    MissingField(&'static str),
}
```

## 8. Testing Strategy

### embedded-hal-mock

```rust
#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::pin::{Mock as PinMock, State, Transaction};
    use embedded_hal_mock::eh1::delay::NoopDelay;

    #[test]
    fn test_single_step() {
        let expectations = [
            Transaction::set(State::High),
            Transaction::set(State::Low),
        ];
        let step_pin = PinMock::new(&expectations);
        let dir_pin = PinMock::new(&[]);
        let delay = NoopDelay::new();
        
        let mut motor = StepperMotor::new(step_pin, dir_pin, delay);
        motor.step().unwrap();
        
        motor.step_pin.done();
    }
}
```

### Property-Based Testing

```rust
use proptest::prelude::*;

proptest! {
    #[test]
    fn position_roundtrip(degrees in -3600.0f32..3600.0f32) {
        let mut pos = Position::new(3200.0); // steps per degree
        let target = pos.move_to_absolute(degrees);
        pos.apply_steps(target);
        prop_assert!((pos.as_degrees() - degrees).abs() < 0.001);
    }
}
```

## 9. Performance Considerations

### Critical Path Optimization

```rust
#[inline(always)]
pub fn generate_step_pulse<STEP: OutputPin, DELAY: DelayNs>(
    step: &mut STEP,
    delay: &mut DELAY,
    pulse_width_ns: u32,
) -> Result<(), STEP::Error> {
    step.set_high()?;
    delay.delay_ns(pulse_width_ns);
    step.set_low()?;
    Ok(())
}
```

### Precomputed Step Tables

For deterministic timing, precompute step intervals during trajectory planning:

```rust
pub struct PrecomputedTrajectory {
    /// Step intervals in nanoseconds
    intervals: heapless::Vec<u32, 1024>,
    /// Total steps
    total_steps: u32,
    /// Direction
    direction: Direction,
}
```

## 10. Conclusion

The design leverages:
- **embedded-hal 1.0** for hardware abstraction
- **Type-states** for compile-time safety
- **Serde** for configuration with `no_std` support
- **Trapezoidal profiles** for smooth acceleration
- **i64 position tracking** for unlimited range

All decisions align with constitution principles (ergonomic API, zero-cost abstractions, safety).
