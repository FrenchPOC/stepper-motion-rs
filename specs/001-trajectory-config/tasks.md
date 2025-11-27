# Tasks: Trajectory Configuration System

**Input**: Design documents from `/specs/001-trajectory-config/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/config-schema.md ✅

**Tests**: TDD approach enabled per constitution (Principle IV). Tests written first, must fail before implementation.

**Organization**: Tasks grouped by user story for independent implementation and MVP delivery.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- All paths are absolute from repository root

---

## Phase 1: Setup

**Purpose**: Project initialization and Rust crate structure

- [X] T001 Create Rust library crate with `cargo init --lib` at repository root
- [X] T002 Configure Cargo.toml with dependencies: embedded-hal 1.0, serde (no_std), heapless, toml (std feature)
- [X] T003 [P] Create feature flags in Cargo.toml: default=["std"], std, alloc, defmt, async
- [X] T004 [P] Create directory structure: src/config/, src/motor/, src/motion/, src/trajectory/
- [X] T005 [P] Create src/lib.rs with module declarations and public API re-exports
- [X] T006 [P] Create src/error.rs with unified Error type and ConfigError variants
- [X] T007 [P] Create CHANGELOG.md with v0.1.0 placeholder (constitution requirement)
- [X] T008 [P] Create rustfmt.toml and clippy.toml for code style enforcement

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core types and traits that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T009 Create src/config/units.rs with Degrees, DegreesPerSec, DegreesPerSecSquared, Steps types
- [X] T010 [P] Create src/config/mod.rs with module re-exports
- [X] T011 Implement Microsteps type with power-of-2 validation in src/config/units.rs
- [X] T012 [P] Create src/motor/state.rs with type-state markers: Idle, Moving, Homing, Fault
- [X] T013 [P] Create src/motor/position.rs with Position struct (i64 steps, unit conversions)
- [X] T014 Create src/motor/mod.rs with module structure
- [X] T015 Create src/motion/mod.rs with module structure
- [X] T016 [P] Create src/trajectory/mod.rs with module structure
- [X] T017 Create tests/unit/mod.rs test harness

**Checkpoint**: Foundation ready — user story implementation can now begin

---

## Phase 3: User Story 1 — Load Trajectory from Configuration File (Priority: P1) 🎯 MVP

**Goal**: Define motion trajectories in external TOML file and apply to initialized motor

**Independent Test**: Create config file, load it, apply to mock motor, verify motion profile matches config

### Tests for User Story 1

> **TDD: Write tests FIRST, ensure they FAIL before implementation**

- [X] T018 [P] [US1] Unit test for TOML parsing in tests/unit/config_parsing.rs
- [X] T019 [P] [US1] Unit test for configuration validation in tests/unit/config_validation.rs
- [X] T020 [P] [US1] Integration test for config loading workflow in tests/integration/config_loading.rs
- [X] T021 [P] [US1] Contract test: valid config → parsed struct in tests/contract/config_contract.rs

### Implementation for User Story 1

- [X] T022 [P] [US1] Create src/config/motor.rs with MotorConfig struct and serde derives
- [X] T023 [P] [US1] Create src/config/trajectory.rs with TrajectoryConfig struct (including asymmetric accel/decel)
- [X] T024 [US1] Create src/config/system.rs with SystemConfig root struct
- [X] T025 [US1] Implement TOML loading with toml crate (std feature) in src/config/loader.rs
- [X] T026 [US1] Implement configuration validation in src/config/validation.rs
- [X] T027 [US1] Create src/motor/driver.rs with StepperMotor<STEP, DIR, DELAY, STATE> generic struct
- [X] T028 [US1] Implement motor builder pattern in src/motor/builder.rs
- [X] T029 [US1] Create src/motion/profile.rs with MotionProfile struct (asymmetric trapezoidal)
- [X] T030 [US1] Implement asymmetric_trapezoidal() constructor for MotionProfile
- [X] T031 [US1] Create src/motion/executor.rs with step pulse generation logic
- [X] T032 [US1] Implement motor.move_to(Degrees) method in src/motor/driver.rs
- [X] T033 [US1] Implement motor.step() blocking step execution in src/motor/driver.rs
- [X] T034 [US1] Add position tracking in motor driver (i64 steps from origin)
- [X] T035 [US1] Create examples/basic_motor.rs demonstrating manual motor control

**Checkpoint**: User Story 1 complete — can load config and execute single trajectory on motor

---

## Phase 4: User Story 2 — Define Mechanical Constraints in Configuration (Priority: P2)

**Goal**: Specify mechanical parameters in config, auto-validate trajectories against hardware limits

**Independent Test**: Define mechanical limits in config, attempt to create trajectory exceeding limits, verify rejection/clamping

### Tests for User Story 2

- [X] T036 [P] [US2] Unit test for MechanicalConstraints derivation in tests/unit/mechanical_constraints.rs
- [X] T037 [P] [US2] Unit test for soft limit enforcement in tests/unit/limits.rs
- [X] T038 [P] [US2] Integration test for constraint validation in tests/integration/constraint_validation.rs

### Implementation for User Story 2

- [X] T039 [P] [US2] Create src/config/mechanical.rs with MechanicalConstraints struct
- [X] T040 [P] [US2] Create src/config/limits.rs with SoftLimits, StepLimits, LimitPolicy types
- [X] T041 [US2] Implement MechanicalConstraints::from_config() derivation logic
- [X] T042 [US2] Implement steps_per_degree calculation (steps × microsteps × gear_ratio / 360)
- [X] T043 [US2] Implement velocity conversion (deg/sec → steps/sec) in src/config/mechanical.rs
- [X] T044 [US2] Add limit validation in trajectory planning (reject vs clamp policy)
- [X] T045 [US2] Implement unit conversions: degrees ↔ steps, deg/sec ↔ steps/sec
- [X] T046 [US2] Add trajectory feasibility check before execution
- [X] T047 [US2] Implement effective_acceleration() and effective_deceleration() for asymmetric profiles
- [X] T048 [US2] Update examples/basic_motor.rs with mechanical constraint demonstration

**Checkpoint**: User Story 2 complete — trajectories validated against mechanical limits

---

## Phase 5: User Story 3 — Multiple Named Trajectories (Priority: P3)

**Goal**: Define multiple named trajectories in one config, reference by string name

**Independent Test**: Create config with multiple trajectories, load once, execute different trajectories by name

### Tests for User Story 3

- [X] T049 [P] [US3] Unit test for TrajectoryRegistry in tests/unit/trajectory_registry.rs
- [X] T050 [P] [US3] Unit test for trajectory lookup by name in tests/unit/trajectory_lookup.rs
- [X] T051 [P] [US3] Integration test for named trajectory execution in tests/integration/named_trajectories.rs

### Implementation for User Story 3

- [X] T052 [P] [US3] Create src/trajectory/registry.rs with TrajectoryRegistry struct
- [X] T053 [US3] Implement registry.get("name") → Option<&TrajectoryConfig>
- [X] T054 [US3] Implement trajectory not found error with available names list
- [X] T055 [US3] Create src/trajectory/builder.rs with Trajectory builder API
- [X] T056 [US3] Implement motor.execute("trajectory_name") method
- [ ] T057 [US3] Create MotorSystem facade for multi-motor configuration
- [ ] T058 [US3] Implement system.motor("name") accessor
- [X] T059 [US3] Add WaypointTrajectory (sequences) support in src/config/trajectory.rs
- [X] T060 [US3] Create examples/config_driven.rs demonstrating named trajectory execution
- [X] T061 [US3] Create examples/multi_motor.rs demonstrating multiple motors from config

**Checkpoint**: User Story 3 complete — full configuration-driven workflow operational

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Documentation, optimization, hardening

- [X] T062 [P] Create README.md with installation, usage, and feature overview
- [X] T063 [P] Add rustdoc documentation to all public types and methods
- [X] T064 [P] Create motion.toml example configuration file at repository root
- [X] T065 Run examples from quickstart.md and verify they compile and execute
- [X] T066 [P] Add backlash_compensation field support in motor driver
- [X] T067 [P] Add invert_direction field support in motor driver
- [X] T068 Verify no_std compilation: `cargo build --no-default-features --features alloc`
- [X] T069 Run `cargo clippy --all-features` and fix all warnings
- [X] T070 Run `cargo test --all-features` and ensure 100% pass
- [X] T071 Update CHANGELOG.md with v0.1.0 release notes

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1: Setup ──────────────┐
                             │
                             ▼
Phase 2: Foundational ───────┤ (BLOCKS all user stories)
                             │
         ┌───────────────────┼───────────────────┐
         ▼                   ▼                   ▼
Phase 3: US1 (P1)    Phase 4: US2 (P2)   Phase 5: US3 (P3)
    MVP 🎯               └──depends on US1 models
                             └──depends on US1 & US2
                                               │
                             ┌─────────────────┘
                             ▼
                    Phase 6: Polish
```

### User Story Dependencies

| Story | Depends On | Can Start After |
|-------|------------|-----------------|
| **US1** (Load Trajectory) | Phase 2 | Foundational complete |
| **US2** (Mechanical Constraints) | US1 models | T024 (SystemConfig) |
| **US3** (Named Trajectories) | US1 + US2 | T032 (motor.move_to) |

### Parallel Opportunities

**Phase 1** (all [P] tasks run together):
```
T003 ─┬─ T004 ─┬─ T005 ─┬─ T006 ─┬─ T007 ─┬─ T008
```

**Phase 2** (after T009):
```
T010 ─┬─ T012 ─┬─ T013 ─┬─ T016
```

**Phase 3 Tests** (all parallel):
```
T018 ─┬─ T019 ─┬─ T020 ─┬─ T021
```

**Phase 3 Implementation** (parallel models):
```
T022 ─┬─ T023    → T024 → T025 → ...
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. ✅ Complete Phase 1: Setup
2. ✅ Complete Phase 2: Foundational
3. ✅ Complete Phase 3: User Story 1
4. **STOP & VALIDATE**: Run tests, verify basic_motor.rs example works
5. **MVP READY**: Can load TOML config and execute trajectory on motor

### Incremental Delivery

| Increment | Delivers | Test With |
|-----------|----------|-----------|
| **MVP** | Config loading, single trajectory | `examples/basic_motor.rs` |
| **+US2** | Mechanical constraints, validation | Exceed limits → error |
| **+US3** | Named trajectories, multi-motor | `examples/config_driven.rs` |

### Suggested MVP Scope

**Just User Story 1 (P1)** — Tasks T001–T035:
- Load motor config from TOML
- Create motor with embedded-hal pins
- Execute single trajectory with asymmetric acceleration
- Track absolute position

This is a fully functional, independently testable increment.

---

## Notes

- **[P] tasks**: Different files, no dependencies — run in parallel
- **[Story] label**: Maps task to user story for traceability
- **TDD**: Write failing tests before implementation (constitution requirement)
- **Commit strategy**: Commit after each task or logical group
- **no_std**: Verify with `--no-default-features --features alloc` regularly
- **Position**: Always track in i64 steps — unlimited range
