<!--
  SYNC IMPACT REPORT
  ==================
  Version change: N/A → 1.0.0 (initial ratification)
  
  Added sections:
  - Core Principles (5 principles)
  - API Design Standards
  - Quality Gates
  - Governance
  
  Modified principles: N/A (initial version)
  Removed sections: N/A (initial version)
  
  Templates requiring updates:
  - plan-template.md: ✅ Compatible (Constitution Check section exists)
  - spec-template.md: ✅ Compatible (Requirements section aligns with principles)
  - tasks-template.md: ✅ Compatible (Phase structure supports TDD workflow)
  - checklist-template.md: ✅ Compatible (Generic structure)
  - agent-file-template.md: ✅ Compatible (Generic structure)
  
  Follow-up TODOs: None
-->

# stepper-motion-rs Constitution

A Rust library for stepper motor motion control — designed for elegance, safety, and developer joy.

## Core Principles

### I. Ergonomic API Design

The public API MUST prioritize developer experience and code readability above all else.

- Method chaining and builder patterns MUST be used where they improve expressiveness
- Type states MUST encode invalid states as unrepresentable at compile time
- Default behaviors MUST be sensible; configuration MUST be optional, not required
- Error messages MUST be actionable and guide users toward correct usage
- Documentation examples MUST compile and demonstrate idiomatic usage

**Rationale**: An elegant API reduces cognitive load, prevents bugs at compile time, and makes the library a pleasure to use. Users should fall into the "pit of success."

### II. Zero-Cost Abstractions

High-level ergonomic APIs MUST NOT sacrifice runtime performance.

- Abstractions MUST compile down to equivalent hand-written code (no hidden allocations)
- `#[inline]` hints MUST be applied to performance-critical paths
- Generic parameters MUST be used to enable monomorphization where beneficial
- Benchmarks MUST validate that abstraction layers add zero overhead
- `no_std` compatibility MUST be maintained for embedded targets

**Rationale**: Motion control often runs on resource-constrained embedded systems. Elegance must not come at the cost of microseconds.

### III. Fearless Concurrency & Safety

All public APIs MUST leverage Rust's type system to guarantee memory and thread safety.

- `unsafe` code MUST be encapsulated behind safe abstractions with documented invariants
- Send/Sync bounds MUST be correct and explicit
- Interior mutability patterns MUST use appropriate synchronization primitives
- Panic paths MUST be documented; recoverable errors MUST use `Result<T, E>`
- Hardware access MUST be abstracted to allow safe mocking in tests

**Rationale**: Users must trust the library to behave correctly in concurrent and real-time contexts without fear of undefined behavior.

### IV. Test-Driven Development

All functionality MUST be developed using TDD methodology.

- Tests MUST be written before implementation; failing tests MUST be observed
- Unit tests MUST cover edge cases, boundary conditions, and error paths
- Integration tests MUST validate real-world motion profiles and timing constraints
- Property-based tests SHOULD be used for mathematical invariants
- CI MUST enforce test passage with `cargo test --all-features`

**Rationale**: TDD ensures correctness from the start and provides living documentation of expected behavior.

### V. Semantic Versioning & Stability

The public API MUST follow semver strictly to protect downstream users.

- Breaking changes MUST increment MAJOR version
- New features MUST increment MINOR version
- Bug fixes and documentation MUST increment PATCH version
- Deprecated items MUST include `#[deprecated]` with migration guidance
- CHANGELOG.md MUST document all user-facing changes

**Rationale**: Users depend on stable APIs. Predictable versioning enables confident upgrades.

## API Design Standards

These standards codify the ergonomic patterns that MUST be followed.

### Builder Pattern Requirements

```rust
// ✅ Correct: Fluent, chainable, with sensible defaults
let motion = MotionProfile::new()
    .max_velocity(1000.steps_per_sec())
    .acceleration(500.steps_per_sec_squared())
    .build()?;

// ❌ Wrong: Positional arguments, no defaults
let motion = MotionProfile::new(1000, 500, 0, 0, true, false);
```

### Type-State Enforcement

States that cannot coexist MUST be encoded in the type system:

```rust
// ✅ Correct: Motor<Stopped> cannot call step()
motor.enable()?.set_direction(CW).step();

// ❌ Wrong: Runtime check for motor state
if motor.is_enabled() { motor.step(); }
```

### Error Handling

- Custom error types MUST implement `std::error::Error`
- Errors MUST be categorized (configuration, hardware, motion)
- `thiserror` MAY be used for derive convenience
- Panics MUST only occur for programmer errors, never for user input

## Quality Gates

All code MUST pass these gates before merge:

| Gate | Command | Requirement |
|------|---------|-------------|
| Format | `cargo fmt --check` | Zero diff |
| Lint | `cargo clippy -- -D warnings` | Zero warnings |
| Test | `cargo test --all-features` | All pass |
| Docs | `cargo doc --no-deps` | Zero warnings |
| MSRV | `cargo +1.70.0 check` | Compiles on MSRV |

## Governance

This constitution supersedes all informal practices. Amendments require:

1. Proposal with rationale documenting the change
2. Review of impact on existing code and downstream users
3. Version increment following semver (constitution changes = MINOR or MAJOR)
4. Update to CHANGELOG.md with migration guidance if applicable

All pull requests MUST include a Constitution Check confirming:

- [ ] API additions follow ergonomic patterns (Principle I)
- [ ] No hidden performance costs introduced (Principle II)
- [ ] Safety guarantees maintained (Principle III)
- [ ] Tests written before implementation (Principle IV)
- [ ] Semver correctly applied (Principle V)

**Version**: 1.0.0 | **Ratified**: 2025-11-26 | **Last Amended**: 2025-11-26
