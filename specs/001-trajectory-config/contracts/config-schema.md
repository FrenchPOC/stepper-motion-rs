# Configuration Schema: stepper-motion-rs

**Feature**: 001-trajectory-config  
**Date**: 2025-11-26

## Overview

Configuration files use TOML format. The schema defines motors and their mechanical properties, plus named trajectories that can be executed by name at runtime.

## Complete Example

```toml
# motion.toml - Complete configuration example

#═══════════════════════════════════════════════════════════════════════════════
# MOTORS
# Define each motor with its mechanical properties
#═══════════════════════════════════════════════════════════════════════════════

[motors.x_axis]
name = "X-Axis Stepper"           # Human-readable name (max 32 chars)
steps_per_revolution = 200        # Base steps (before microstepping)
microsteps = 16                   # Microstep divisor: 1, 2, 4, 8, 16, 32, 64, 128, 256
gear_ratio = 1.0                  # Output:Input ratio (e.g., 5.0 = 5:1 reduction)
max_velocity_deg_per_sec = 360.0  # Maximum angular velocity
max_acceleration_deg_per_sec2 = 720.0  # Maximum angular acceleration
invert_direction = false          # Swap CW/CCW if motor wired backwards

[motors.x_axis.limits]
min_degrees = -180.0              # Minimum allowed position
max_degrees = 180.0               # Maximum allowed position
policy = "reject"                 # "reject" or "clamp"

[motors.y_axis]
name = "Y-Axis Stepper"
steps_per_revolution = 400        # 0.9° motor
microsteps = 8
gear_ratio = 2.5                  # 2.5:1 gearbox
max_velocity_deg_per_sec = 180.0
max_acceleration_deg_per_sec2 = 360.0
invert_direction = true
backlash_compensation_deg = 0.5   # Compensate 0.5° backlash on reversal

[motors.z_axis]
name = "Z-Axis (Lead Screw)"
steps_per_revolution = 200
microsteps = 32
gear_ratio = 1.0
max_velocity_deg_per_sec = 720.0
max_acceleration_deg_per_sec2 = 1440.0
# No limits defined = unlimited travel

#═══════════════════════════════════════════════════════════════════════════════
# TRAJECTORIES
# Named motion profiles that can be executed by name
#═══════════════════════════════════════════════════════════════════════════════

[trajectories.x_home]
motor = "x_axis"                  # Must match a motor name
target_degrees = 0.0              # Absolute target position
velocity_percent = 50             # 50% of motor's max velocity
acceleration_percent = 100        # 100% of motor's max acceleration

[trajectories.x_work_position]
motor = "x_axis"
target_degrees = 90.0
velocity_percent = 100
acceleration_percent = 80
dwell_ms = 500                    # Wait 500ms after reaching target

# Asymmetric acceleration example: fast start, gentle stop
[trajectories.x_gentle_stop]
motor = "x_axis"
target_degrees = 180.0
velocity_percent = 100
acceleration_deg_per_sec2 = 1000.0   # Fast acceleration (absolute value)
deceleration_deg_per_sec2 = 300.0    # Gentle deceleration (asymmetric)

# Quick stop example: slow start, emergency stop capable
[trajectories.x_emergency_ready]
motor = "x_axis"
target_degrees = 45.0
velocity_percent = 80
acceleration_deg_per_sec2 = 200.0    # Gradual acceleration
deceleration_deg_per_sec2 = 2000.0   # Aggressive deceleration

[trajectories.x_far_position]
motor = "x_axis"
target_degrees = 568.0            # Note: Will be clamped/rejected per limits!
velocity_percent = 75

[trajectories.y_home]
motor = "y_axis"
target_degrees = 0.0
velocity_percent = 30             # Slow homing

[trajectories.z_up]
motor = "z_axis"
target_degrees = 3600.0           # 10 full rotations up
velocity_percent = 100

#═══════════════════════════════════════════════════════════════════════════════
# SEQUENCES (Optional)
# Multi-waypoint trajectories for scanning/inspection patterns
#═══════════════════════════════════════════════════════════════════════════════

[sequences.x_scan]
motor = "x_axis"
waypoints = [0.0, 45.0, 90.0, 135.0, 180.0]  # Visit each position in order
dwell_ms = 1000                   # Wait 1 second at each waypoint
velocity_percent = 50

[sequences.y_calibration]
motor = "y_axis"
waypoints = [-90.0, 0.0, 90.0, 0.0]
dwell_ms = 500
velocity_percent = 25
```

## Field Reference

### Motor Configuration

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `name` | string | ✅ | — | Human-readable motor name (max 32 chars) |
| `steps_per_revolution` | u16 | ✅ | — | Base motor steps per revolution (before microstepping) |
| `microsteps` | u16 | ✅ | — | Microstep divisor. Valid: 1, 2, 4, 8, 16, 32, 64, 128, 256 |
| `gear_ratio` | f32 | ❌ | 1.0 | Output:Input gear ratio |
| `max_velocity_deg_per_sec` | f32 | ✅ | — | Maximum angular velocity (degrees/second) |
| `max_acceleration_deg_per_sec2` | f32 | ✅ | — | Maximum angular acceleration (degrees/second²) |
| `invert_direction` | bool | ❌ | false | Invert direction pin logic |
| `backlash_compensation_deg` | f32 | ❌ | 0.0 | Backlash compensation in degrees |
| `limits` | table | ❌ | None | Soft limit configuration |

### Soft Limits

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `min_degrees` | f32 | ✅ | — | Minimum allowed position |
| `max_degrees` | f32 | ✅ | — | Maximum allowed position |
| `policy` | string | ❌ | "reject" | How to handle limit violations: "reject" or "clamp" |

### Trajectory Configuration

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `motor` | string | ✅ | — | Motor name to execute on (must exist in motors) |
| `target_degrees` | f32 | ✅ | — | Absolute target position in degrees |
| `velocity_percent` | u8 | ❌ | 100 | Velocity as percentage of motor's max (1-200) |
| `acceleration_percent` | u8 | ❌ | 100 | Acceleration as percentage of motor's max (1-200), used when absolute rates not specified |
| `acceleration_deg_per_sec2` | f32 | ❌ | — | Absolute acceleration rate in deg/s² (overrides acceleration_percent for accel phase) |
| `deceleration_deg_per_sec2` | f32 | ❌ | — | Absolute deceleration rate in deg/s² (if not set, uses acceleration value for symmetric profile) |
| `dwell_ms` | u32 | ❌ | 0 | Time to wait at target (milliseconds) |

#### Acceleration/Deceleration Priority

1. If `deceleration_deg_per_sec2` is set → use it for decel phase
2. Else if `acceleration_deg_per_sec2` is set → use it for both phases (symmetric)
3. Else → use `motor.max_acceleration * acceleration_percent / 100`

### Sequence Configuration

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `motor` | string | ✅ | — | Motor name to execute on |
| `waypoints` | [f32] | ✅ | — | Array of positions to visit in order (max 32) |
| `dwell_ms` | u32 | ❌ | 0 | Dwell time at each waypoint |
| `velocity_percent` | u8 | ❌ | 100 | Velocity for all moves |

## Validation Rules

### Motor Validation

1. `name` must be 1-32 characters, unique across all motors
2. `steps_per_revolution` must be > 0 and ≤ 65535
3. `microsteps` must be a power of 2 from 1 to 256
4. `gear_ratio` must be > 0
5. `max_velocity_deg_per_sec` must be > 0
6. `max_acceleration_deg_per_sec2` must be > 0
7. If `limits` is specified, `min_degrees` must be < `max_degrees`

### Trajectory Validation

1. `motor` must reference an existing motor name
2. `velocity_percent` must be 1-200 (allows overdrive)
3. `acceleration_percent` must be 1-200
4. `acceleration_deg_per_sec2` if specified, must be > 0 and ≤ motor's max × 2
5. `deceleration_deg_per_sec2` if specified, must be > 0 and ≤ motor's max × 2
6. `target_degrees` is validated against motor limits (if configured)

### Sequence Validation

1. `motor` must reference an existing motor name
2. `waypoints` must have at least 1 and at most 32 entries
3. Each waypoint is validated against motor limits

## Computed Values

The library computes these derived values from configuration:

```rust
// Total steps per output shaft revolution
total_steps = steps_per_revolution × microsteps × gear_ratio

// Example: 200 steps × 16 microsteps × 1.0 ratio = 3200 steps/rev

// Steps per degree
steps_per_degree = total_steps / 360.0

// Example: 3200 / 360 = 8.889 steps/degree
```

## Error Messages

Configuration errors include line numbers and specific guidance:

```
Error at line 15, column 12:
  motors.x_axis.microsteps = 17
                             ^^
  Invalid microstep value: 17
  Valid values are: 1, 2, 4, 8, 16, 32, 64, 128, 256

Error at line 28:
  trajectories.unknown_motor.motor = "z_axis"
  Motor 'z_axis' not found in [motors] section.
  Available motors: x_axis, y_axis
```

## Best Practices

1. **Name motors by axis**: `x_axis`, `y_axis`, `z_axis` or `pan`, `tilt`, `zoom`
2. **Start conservative**: Use lower velocity/acceleration initially
3. **Define home trajectories**: Every motor should have a `*_home` trajectory
4. **Set soft limits**: Prevent mechanical damage
5. **Use velocity_percent < 100**: Leave headroom for acceleration ramps
6. **Document backlash**: Measure and configure compensation if needed
