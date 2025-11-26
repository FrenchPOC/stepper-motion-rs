# Feature Specification: Trajectory Configuration System

**Feature Branch**: `001-trajectory-config`  
**Created**: 2025-11-26  
**Status**: Ready for Implementation  
**Input**: User description: "I want this library to facilitate the integration of stepper motors. I want it to be possible to assign trajectories to a motor from a configuration file. The motor initialization side remains manual, but the creation of trajectories based on the mechanics is done by configuration outside the main code. Need `no_std` compatibility, embedded-hal 1.0 for STEP/DIR pins, microstep configuration, absolute position tracking, and named trajectory execution."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Load Trajectory from Configuration File (Priority: P1)

As a developer integrating stepper motors, I want to define motion trajectories in an external configuration file so that I can modify motion profiles without recompiling my application.

**Why this priority**: This is the core value proposition — separating trajectory definition from code enables rapid iteration, non-programmer configuration, and cleaner code architecture.

**Independent Test**: Can be fully tested by creating a configuration file with trajectory parameters and loading it into a manually-initialized motor, then verifying the motor executes the expected motion profile.

**Acceptance Scenarios**:

1. **Given** a valid configuration file with trajectory parameters, **When** I load the configuration and apply it to an initialized motor, **Then** the motor receives a trajectory matching the configuration values
2. **Given** a configuration file with mechanical constraints (max velocity, acceleration limits), **When** I load the trajectory, **Then** the system respects these constraints and generates a safe motion profile
3. **Given** a configuration file with invalid or missing required fields, **When** I attempt to load it, **Then** the system returns a clear error indicating what is wrong and where

---

### User Story 2 - Define Mechanical Constraints in Configuration (Priority: P2)

As a developer, I want to specify mechanical parameters (gear ratios, steps per revolution, microsteps, maximum speeds, acceleration limits) in the configuration file so that trajectories are automatically validated against hardware capabilities.

**Why this priority**: Mechanical constraints are fundamental to safe operation — this builds on P1 by adding the "mechanics-based" aspect that ensures trajectories are physically achievable.

**Independent Test**: Can be tested by defining mechanical parameters in configuration, then attempting to create trajectories that exceed those parameters and verifying they are rejected or clamped.

**Acceptance Scenarios**:

1. **Given** a configuration with mechanical limits defined, **When** I request a trajectory exceeding max velocity, **Then** the system either rejects it with a clear error or clamps to the safe limit (based on configuration policy)
2. **Given** a configuration with gear ratio and steps-per-revolution, **When** I specify trajectory in real-world units (degrees, mm), **Then** the system correctly converts to motor steps
3. **Given** a configuration with acceleration limits, **When** I generate a motion profile, **Then** the acceleration and deceleration phases respect the configured limits

---

### User Story 3 - Multiple Named Trajectories in Single Configuration (Priority: P3)

As a developer with multiple motion sequences, I want to define several named trajectories in one configuration file so that I can reference them by name in my code and switch between profiles easily.

**Why this priority**: Many applications need multiple motion profiles (home position, work position, calibration sequence). This adds significant ergonomic value once the core loading mechanism exists.

**Independent Test**: Can be tested by creating a configuration with multiple named trajectories, loading it once, then applying different trajectories by name to verify each executes correctly.

**Acceptance Scenarios**:

1. **Given** a configuration file with multiple named trajectories, **When** I load the configuration, **Then** I can access each trajectory by its string name
2. **Given** a loaded configuration, **When** I request a trajectory name that doesn't exist, **Then** the system returns a clear "not found" error with available names listed
3. **Given** multiple trajectories sharing mechanical constraints, **When** I define constraints once at the top level, **Then** all trajectories inherit those constraints without repetition

---

### Edge Cases

- What happens when the configuration file doesn't exist or is unreadable? → Clear error with file path
- What happens when configuration format is malformed (syntax error)? → Error with line number and description
- What happens when trajectory values are negative where they shouldn't be? → Validation error
- How does the system handle extremely long trajectories or very small time steps? → Graceful handling with warnings if near limits
- What happens when units are ambiguous or missing? → Require explicit units or fail with clear message

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST load trajectory definitions from external configuration files
- **FR-002**: System MUST support at minimum one human-readable configuration format (TOML assumed as sensible Rust default)
- **FR-003**: System MUST validate configuration against a schema and report all validation errors, not just the first one
- **FR-004**: System MUST allow mechanical parameters to be defined in configuration (steps per revolution, gear ratios, velocity limits, acceleration limits)
- **FR-005**: System MUST convert real-world units (degrees, radians, millimeters) to motor steps using mechanical parameters
- **FR-006**: System MUST support multiple named trajectories within a single configuration file
- **FR-007**: System MUST provide clear, actionable error messages that include file location and what was expected
- **FR-008**: System MUST allow trajectory application to manually-initialized motors (no automatic motor initialization)
- **FR-009**: System MUST validate that requested trajectories are achievable within mechanical constraints before execution
- **FR-010**: System MUST support trajectory inheritance or defaults to reduce configuration repetition

### Key Entities

- **TrajectoryConfig**: The parsed representation of a configuration file, containing mechanical parameters and named trajectories
- **MechanicalConstraints**: Physical limits of the motor system — max velocity, max acceleration, steps per revolution, microsteps, gear ratio, travel limits
- **StepperMotor**: The runtime motor driver using embedded-hal 1.0 `OutputPin` for STEP/DIR control, tracking state and absolute position
- **Trajectory**: A motion profile with target position, velocity profile, and timing; always absolute (relative to origin)
- **MotionProfile**: Computed step timing for trapezoidal acceleration/deceleration
- **MotionSegment**: A single phase of motion (acceleration, cruise, deceleration) within a trajectory
- **Units**: Representation of physical units (steps, degrees, radians, mm) enabling safe conversions
- **Position**: Absolute position in steps from origin, with unit conversion methods

## Technical Requirements

### Hardware Abstraction

- **FR-011**: System MUST use `embedded-hal` 1.0 `OutputPin` trait for STEP and DIR pin control
- **FR-012**: System MUST use `embedded-hal` 1.0 `DelayNs` trait for step timing
- **FR-013**: System MUST be `no_std` compatible with optional `std` feature for file I/O
- **FR-014**: System MUST support `alloc` feature for heap allocation in no_std environments

### Motor State & Position Tracking

- **FR-015**: Each motor MUST track its absolute position in steps from origin at all times
- **FR-016**: Each motor MUST know its current state (Idle, Moving, Homing, Fault)
- **FR-017**: Position MUST be preserved across trajectory executions
- **FR-018**: System MUST support querying current position in degrees (absolute from origin)

### Microstep Configuration

- **FR-019**: Motor configuration MUST include microstep setting (1, 2, 4, 8, 16, 32, 64, 128, 256)
- **FR-020**: Step calculations MUST account for microsteps × steps_per_revolution × gear_ratio
- **FR-021**: Microstep setting MUST be validated as power of 2

### Asymmetric Motion Profiles

- **FR-022**: Trajectory configuration MAY specify independent acceleration and deceleration rates
- **FR-023**: When only acceleration is specified, deceleration MUST default to the same value (symmetric profile)
- **FR-024**: When neither is specified, MUST use motor's max acceleration × acceleration_percent
- **FR-025**: Asymmetric profiles MUST correctly compute step counts for triangular profiles (when max velocity cannot be reached)

## Assumptions

- Configuration format will be TOML (standard Rust ecosystem choice, human-readable)
- Motors are initialized manually by user code before trajectories are applied
- The library will be `no_std` compatible for embedded use, with optional `std` features for file I/O
- Trajectories are point-to-point moves; complex multi-segment paths can be built from sequences
- Real-time execution timing is handled by user code; this library generates the motion profile
- embedded-hal 1.0 is used (not 0.2.x) for OutputPin and DelayNs traits
- Position tracking uses i64 steps for unlimited range

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Developers can define a complete trajectory in configuration and apply it to a motor in under 10 lines of code
- **SC-002**: Configuration changes take effect without recompilation — users can modify and reload at runtime
- **SC-003**: 100% of validation errors include actionable guidance (what's wrong, where, and how to fix)
- **SC-004**: Trajectory loading from file completes in under 10 milliseconds for typical configurations
- **SC-005**: Library compiles and runs on embedded targets (`no_std` + `alloc`)
- **SC-006**: Documentation includes working examples for each user story that compile and run
- **SC-007**: Motor position is always accurate within 1 step of expected value
- **SC-008**: Named trajectories can be called by string name (e.g., `motor.execute("trajectory1")`)
