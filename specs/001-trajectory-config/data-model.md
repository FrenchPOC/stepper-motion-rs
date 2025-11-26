# Data Model: Trajectory Configuration System

**Feature**: 001-trajectory-config  
**Date**: 2025-11-26

## Core Types

### Units & Quantities

```rust
/// Angular position in degrees (f32 for configuration, i64 steps internally)
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct Degrees(pub f32);

/// Angular velocity in degrees per second
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct DegreesPerSec(pub f32);

/// Angular acceleration in degrees per second squared
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct DegreesPerSecSquared(pub f32);

/// Motor steps (absolute position from origin)
#[derive(Debug, Clone, Copy, Default)]
pub struct Steps(pub i64);

/// Microstep divisor (1, 2, 4, 8, 16, 32, 64, 128, 256)
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(try_from = "u16")]
pub struct Microsteps(u16);

impl Microsteps {
    pub const FULL: Self = Self(1);
    pub const HALF: Self = Self(2);
    pub const QUARTER: Self = Self(4);
    pub const EIGHTH: Self = Self(8);
    pub const SIXTEENTH: Self = Self(16);
    pub const THIRTY_SECOND: Self = Self(32);
}
```

### Motor Configuration

```rust
/// Complete motor configuration from TOML
#[derive(Debug, Clone, Deserialize)]
pub struct MotorConfig {
    /// Human-readable name
    pub name: heapless::String<32>,
    
    /// Base steps per revolution (typically 200 for 1.8° motors)
    pub steps_per_revolution: u16,
    
    /// Microstep setting (1, 2, 4, 8, 16, 32, etc.)
    pub microsteps: Microsteps,
    
    /// Gear ratio (output:input, e.g., 5.0 means 5:1 reduction)
    #[serde(default = "default_gear_ratio")]
    pub gear_ratio: f32,
    
    /// Maximum angular velocity
    pub max_velocity: DegreesPerSec,
    
    /// Maximum angular acceleration
    pub max_acceleration: DegreesPerSecSquared,
    
    /// Invert direction pin logic
    #[serde(default)]
    pub invert_direction: bool,
    
    /// Optional soft limits
    #[serde(default)]
    pub limits: Option<SoftLimits>,
    
    /// Optional backlash compensation in degrees
    #[serde(default)]
    pub backlash_compensation: Option<Degrees>,
}

fn default_gear_ratio() -> f32 { 1.0 }
```

### Mechanical Constraints

```rust
/// Derived mechanical parameters (computed from MotorConfig)
#[derive(Debug, Clone)]
pub struct MechanicalConstraints {
    /// Total steps per output revolution (steps × microsteps × gear_ratio)
    pub steps_per_revolution: u32,
    
    /// Steps per degree of output rotation
    pub steps_per_degree: f32,
    
    /// Maximum velocity in steps per second
    pub max_velocity_steps_per_sec: f32,
    
    /// Maximum acceleration in steps per second squared
    pub max_acceleration_steps_per_sec2: f32,
    
    /// Minimum step interval in nanoseconds (at max velocity)
    pub min_step_interval_ns: u32,
    
    /// Soft limits in steps (if configured)
    pub limits: Option<StepLimits>,
}

#[derive(Debug, Clone)]
pub struct StepLimits {
    pub min_steps: i64,
    pub max_steps: i64,
    pub policy: LimitPolicy,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LimitPolicy {
    /// Reject moves that would exceed limits
    Reject,
    /// Clamp target to nearest limit
    Clamp,
}
```

### Soft Limits (Config)

```rust
#[derive(Debug, Clone, Deserialize)]
pub struct SoftLimits {
    /// Minimum allowed position in degrees
    pub min_degrees: Degrees,
    
    /// Maximum allowed position in degrees  
    pub max_degrees: Degrees,
    
    /// What to do when limit is exceeded
    #[serde(default)]
    pub policy: LimitPolicy,
}

impl Default for LimitPolicy {
    fn default() -> Self { Self::Reject }
}
```

### Trajectory Definition

```rust
/// A named trajectory from configuration
#[derive(Debug, Clone, Deserialize)]
pub struct TrajectoryConfig {
    /// Target motor name (must match a motor in config)
    pub motor: heapless::String<32>,
    
    /// Target position in degrees (absolute from origin)
    pub target_degrees: Degrees,
    
    /// Velocity as percentage of motor's max (0-100)
    #[serde(default = "default_velocity_percent")]
    pub velocity_percent: u8,
    
    /// Acceleration as percentage of motor's max (0-100)
    /// Used when both accel and decel are symmetric
    /// Ignored if acceleration_deg_per_sec2 or deceleration_deg_per_sec2 is set
    #[serde(default = "default_acceleration_percent")]
    pub acceleration_percent: u8,
    
    /// Absolute acceleration rate in degrees/sec² (optional)
    /// Overrides acceleration_percent for the acceleration phase
    /// If not set, uses motor's max_acceleration * acceleration_percent
    #[serde(default)]
    pub acceleration_deg_per_sec2: Option<DegreesPerSecSquared>,
    
    /// Absolute deceleration rate in degrees/sec² (optional)
    /// Overrides acceleration_percent for the deceleration phase
    /// If not set, uses acceleration_deg_per_sec2 (symmetric) or motor's max
    #[serde(default)]
    pub deceleration_deg_per_sec2: Option<DegreesPerSecSquared>,
    
    /// Optional dwell time at target (milliseconds)
    #[serde(default)]
    pub dwell_ms: Option<u32>,
}

fn default_velocity_percent() -> u8 { 100 }
fn default_acceleration_percent() -> u8 { 100 }

impl TrajectoryConfig {
    /// Get effective acceleration rate for this trajectory
    pub fn effective_acceleration(&self, motor: &MotorConstraints) -> DegreesPerSecSquared {
        self.acceleration_deg_per_sec2.unwrap_or_else(|| {
            motor.max_acceleration * (self.acceleration_percent as f32 / 100.0)
        })
    }
    
    /// Get effective deceleration rate for this trajectory
    /// Falls back to acceleration if not specified (symmetric profile)
    pub fn effective_deceleration(&self, motor: &MotorConstraints) -> DegreesPerSecSquared {
        self.deceleration_deg_per_sec2
            .or(self.acceleration_deg_per_sec2)
            .unwrap_or_else(|| {
                motor.max_acceleration * (self.acceleration_percent as f32 / 100.0)
            })
    }
}
```

### Multi-Waypoint Trajectory

```rust
/// Trajectory with multiple waypoints
#[derive(Debug, Clone, Deserialize)]
pub struct WaypointTrajectory {
    /// Target motor name
    pub motor: heapless::String<32>,
    
    /// Ordered list of waypoint positions in degrees
    pub waypoints: heapless::Vec<Degrees, 32>,
    
    /// Dwell time at each waypoint (milliseconds)
    #[serde(default)]
    pub dwell_ms: u32,
    
    /// Velocity percent for all moves
    #[serde(default = "default_velocity_percent")]
    pub velocity_percent: u8,
}
```

### Complete Configuration File

```rust
/// Root configuration structure
#[derive(Debug, Clone, Deserialize)]
pub struct SystemConfig {
    /// Named motor configurations
    pub motors: heapless::FnvIndexMap<heapless::String<32>, MotorConfig, 8>,
    
    /// Named trajectory configurations
    #[serde(default)]
    pub trajectories: heapless::FnvIndexMap<heapless::String<32>, TrajectoryConfig, 64>,
    
    /// Named waypoint trajectories (optional)
    #[serde(default)]
    pub sequences: heapless::FnvIndexMap<heapless::String<32>, WaypointTrajectory, 16>,
}
```

## Runtime Types

### Motor State Machine

```rust
/// Type-state markers for motor states
pub mod state {
    /// Motor is idle and ready for commands
    pub struct Idle;
    
    /// Motor is currently executing a move
    pub struct Moving;
    
    /// Motor is executing a homing sequence
    pub struct Homing;
    
    /// Motor encountered an error and needs recovery
    pub struct Fault;
}
```

### Motor Driver

```rust
/// The main motor driver, generic over pin types and state
pub struct StepperMotor<STEP, DIR, DELAY, STATE = state::Idle>
where
    STEP: OutputPin,
    DIR: OutputPin,
    DELAY: DelayNs,
{
    /// STEP pin (pulse to move one step)
    step_pin: STEP,
    
    /// DIR pin (high = CW, low = CCW, or inverted)
    dir_pin: DIR,
    
    /// Delay provider for step timing
    delay: DELAY,
    
    /// Current absolute position in steps
    position: Steps,
    
    /// Current direction (cached to avoid unnecessary pin writes)
    current_direction: Direction,
    
    /// Mechanical constraints from configuration
    constraints: MechanicalConstraints,
    
    /// Motor name for trajectory lookup
    name: heapless::String<32>,
    
    /// Type-state marker
    _state: PhantomData<STATE>,
}
```

### Motion Profile

```rust
/// Computed motion profile for a move (asymmetric trapezoidal)
#[derive(Debug, Clone)]
pub struct MotionProfile {
    /// Total steps to move (signed: positive = CW, negative = CCW)
    pub total_steps: i64,
    
    /// Direction of motion
    pub direction: Direction,
    
    /// Steps in acceleration phase
    pub accel_steps: u32,
    
    /// Steps in cruise phase (constant velocity)
    pub cruise_steps: u32,
    
    /// Steps in deceleration phase
    pub decel_steps: u32,
    
    /// Initial step interval (nanoseconds) - start of acceleration
    pub initial_interval_ns: u32,
    
    /// Cruise step interval (nanoseconds, at max velocity)
    pub cruise_interval_ns: u32,
    
    /// Acceleration rate in steps/sec²
    pub accel_rate: u32,
    
    /// Deceleration rate in steps/sec² (can differ from accel)
    pub decel_rate: u32,
}

impl MotionProfile {
    /// Create an asymmetric trapezoidal profile
    pub fn asymmetric_trapezoidal(
        total_steps: i64,
        max_velocity: f32,      // steps/sec
        acceleration: f32,      // steps/sec² 
        deceleration: f32,      // steps/sec²
    ) -> Self {
        let direction = if total_steps >= 0 {
            Direction::Clockwise
        } else {
            Direction::CounterClockwise
        };
        
        let steps = total_steps.unsigned_abs() as u32;
        
        // Calculate phase lengths for asymmetric profile
        // Time to reach max velocity: t_a = v_max / a
        // Distance during acceleration: d_a = 0.5 * a * t_a²
        // Similar for deceleration with different rate
        
        let t_accel = max_velocity / acceleration;
        let t_decel = max_velocity / deceleration;
        
        let accel_distance = 0.5 * acceleration * t_accel * t_accel;
        let decel_distance = 0.5 * deceleration * t_decel * t_decel;
        
        let (accel_steps, cruise_steps, decel_steps) = 
            if accel_distance + decel_distance >= steps as f32 {
                // Triangle profile: can't reach max velocity
                // Scale down proportionally
                let ratio = acceleration / (acceleration + deceleration);
                let accel_steps = (steps as f32 * ratio) as u32;
                let decel_steps = steps - accel_steps;
                (accel_steps, 0u32, decel_steps)
            } else {
                // Full trapezoidal profile
                let accel_steps = accel_distance as u32;
                let decel_steps = decel_distance as u32;
                let cruise_steps = steps - accel_steps - decel_steps;
                (accel_steps, cruise_steps, decel_steps)
            };
        
        Self {
            total_steps,
            direction,
            accel_steps,
            cruise_steps,
            decel_steps,
            initial_interval_ns: (1_000_000_000.0 / (acceleration.sqrt())) as u32,
            cruise_interval_ns: (1_000_000_000.0 / max_velocity) as u32,
            accel_rate: acceleration as u32,
            decel_rate: deceleration as u32,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}
```

### Motion Executor State

```rust
/// Runtime state during motion execution
pub struct MotionExecutor {
    /// The computed profile being executed
    profile: MotionProfile,
    
    /// Current step number (0 to total_steps - 1)
    current_step: u32,
    
    /// Current step interval in nanoseconds
    current_interval_ns: u32,
    
    /// Current phase of motion
    phase: MotionPhase,
}

#[derive(Debug, Clone, Copy)]
pub enum MotionPhase {
    Accelerating,
    Cruising,
    Decelerating,
    Complete,
}
```

### Trajectory Registry

```rust
/// Runtime registry of named trajectories
pub struct TrajectoryRegistry {
    /// Parsed trajectory configurations indexed by name
    trajectories: heapless::FnvIndexMap<heapless::String<32>, TrajectoryConfig, 64>,
    
    /// Parsed sequence configurations indexed by name
    sequences: heapless::FnvIndexMap<heapless::String<32>, WaypointTrajectory, 16>,
}
```

## Entity Relationships

```
┌─────────────────────────────────────────────────────────────────┐
│                        SystemConfig                             │
│  (Root configuration, parsed from TOML)                         │
└─────────────────────────────────────────────────────────────────┘
         │                            │                    │
         ▼                            ▼                    ▼
┌─────────────────┐    ┌─────────────────────┐   ┌──────────────────┐
│   MotorConfig   │    │  TrajectoryConfig   │   │ WaypointTrajectory│
│  (per motor)    │    │  (named trajectory) │   │  (multi-waypoint) │
└─────────────────┘    └─────────────────────┘   └──────────────────┘
         │                       │
         ▼                       │
┌─────────────────────┐          │
│MechanicalConstraints│          │
│ (computed at init)  │          │
└─────────────────────┘          │
         │                       │
         ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    StepperMotor<STEP, DIR, DELAY, STATE>        │
│  - Owns GPIO pins (STEP, DIR)                                   │
│  - Tracks position (Steps)                                      │
│  - Enforces mechanical constraints                              │
│  - Executes trajectories by name                                │
└─────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                        MotionProfile                             │
│  - Computed for each move                                       │
│  - Defines accel/cruise/decel phases                            │
└─────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                       MotionExecutor                             │
│  - Generates step pulses with correct timing                    │
│  - Tracks current phase and step count                          │
└─────────────────────────────────────────────────────────────────┘
```

## Invariants

1. **Position Always Valid**: `StepperMotor::position` is always the true absolute position
2. **Constraints Immutable**: Once computed, `MechanicalConstraints` never change
3. **Type-State Safety**: Cannot call `step()` on `StepperMotor<_, _, _, Idle>`
4. **Name Uniqueness**: Motor and trajectory names are unique within their respective maps
5. **Limit Enforcement**: Moves that exceed soft limits are rejected/clamped before execution
