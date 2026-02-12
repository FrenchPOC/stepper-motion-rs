# Agent Prompt: Integrate Continuous Forward Motion

Use this prompt to instruct an agent to add continuous forward motion in an application that uses `stepper-motion`.

## Copy/Paste Prompt

```text
Integrate continuous forward stepper motion using the stepper-motion library.

Requirements:
1. Start motion with `start_continuous_forward(DegreesPerSec(...))`.
2. Keep stepping in a loop using:
   - sync: `step_with_switch_checks(&mut switches)`
   - async: `step_async_with_switch_checks(&mut switches).await`
3. Enforce stop conditions:
   - application stop signal
   - `MotorError::LimitExceeded` (soft limit)
   - `MotorError::HardwareLimitTriggered` (home/min/max switch)
4. Stop motion by calling `stop()` and keep the returned Idle motor for future commands.
5. Do not use `run_to_completion()` for continuous mode.
6. Wire switches using `HomingSwitches::new(...)` with correct `SwitchConfig` polarity for home/min/max.
7. Preserve existing error handling and log which limit caused the stop.

Implementation details:
- Forward means positive/clockwise motor coordinates.
- Validate requested speed is > 0 and <= configured max velocity.
- Keep the code no_std-friendly (no heap allocations in control loop).
- Add one test that verifies:
  - motion starts,
  - stepping updates position,
  - motion stops on a limit condition.
```

## API Reference

- Sync:
  - `StepperMotor<..., Idle>::start_continuous_forward(speed)`
  - `StepperMotor<..., Moving>::step_with_switch_checks(&mut switches)`
  - `StepperMotor<..., Moving>::stop()`
- Async (`async` feature):
  - `AsyncStepperMotor<..., Idle>::start_continuous_forward(speed)`
  - `AsyncStepperMotor<..., Moving>::step_async_with_switch_checks(&mut switches).await`
  - `AsyncStepperMotor<..., Moving>::stop()`

## Minimal Sync Skeleton

```rust
use stepper_motion::{config::units::DegreesPerSec, error::MotorError, Error};

let mut moving = motor.start_continuous_forward(DegreesPerSec(120.0))?;

loop {
    match moving.step_with_switch_checks(&mut switches) {
        Ok(_) => {}
        Err(Error::Motor(MotorError::LimitExceeded { .. }))
        | Err(Error::Motor(MotorError::HardwareLimitTriggered { .. })) => break,
        Err(e) => return Err(e),
    }

    if app_stop_requested() {
        break;
    }
}

let motor = moving.stop();
```
