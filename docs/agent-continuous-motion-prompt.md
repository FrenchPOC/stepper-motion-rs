# Agent Prompt: Integrate Continuous Motion

Use this prompt to instruct an agent to add continuous motion in an application that uses `stepper-motion`.

## Copy/Paste Prompt

```text
Integrate continuous stepper motion using the stepper-motion library.

Requirements:
1. Start motion with either `start_continuous_forward(DegreesPerSec(...))` or `start_continuous_backward(DegreesPerSec(...))`.
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
8. Optional convenience: use `run_continuous_forward_until_home(...)` / `run_continuous_backward_until_home(...)` when you want a one-call homing sweep.

Implementation details:
- Forward means positive/clockwise motor coordinates.
- Backward means negative/counter-clockwise motor coordinates.
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
  - `StepperMotor<..., Idle>::start_continuous_backward(speed)`
  - `StepperMotor<..., Idle>::run_continuous_forward_until_home(speed, switches)`
  - `StepperMotor<..., Idle>::run_continuous_backward_until_home(speed, switches)`
  - `StepperMotor<..., Moving>::step_with_switch_checks(&mut switches)`
  - `StepperMotor<..., Moving>::stop()`
- Async (`async` feature):
  - `AsyncStepperMotor<..., Idle>::start_continuous_forward(speed)`
  - `AsyncStepperMotor<..., Idle>::start_continuous_backward(speed)`
  - `AsyncStepperMotor<..., Idle>::run_continuous_forward_until_home(speed, switches).await`
  - `AsyncStepperMotor<..., Idle>::run_continuous_backward_until_home(speed, switches).await`
  - `AsyncStepperMotor<..., Moving>::step_async_with_switch_checks(&mut switches).await`
  - `AsyncStepperMotor<..., Moving>::stop()`

## Minimal Sync Skeleton

```rust
use stepper_motion::{config::units::DegreesPerSec, error::MotorError, Error};

let mut moving = motor.start_continuous_backward(DegreesPerSec(120.0))?;

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
