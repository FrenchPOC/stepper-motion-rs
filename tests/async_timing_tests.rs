#![cfg(feature = "async")]

use std::cell::RefCell;
use std::convert::Infallible;
use std::future::Future;
use std::rc::Rc;
use std::sync::Arc;
use std::task::{Context, Poll, Wake, Waker};

use embassy_time::{Duration, Instant, MockDriver};
use embedded_hal::digital::{ErrorType, OutputPin};
use stepper_motion::{
    AsyncStepperMotorBuilder, Degrees, DegreesPerSec, DegreesPerSecSquared, Microsteps,
    MotionProfile,
};

#[derive(Clone)]
struct Pin(Rc<RefCell<Vec<(bool, u64)>>>);

impl ErrorType for Pin {
    type Error = Infallible;
}

impl OutputPin for Pin {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.borrow_mut().push((true, Instant::now().as_ticks()));
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.borrow_mut().push((false, Instant::now().as_ticks()));
        Ok(())
    }
}

struct NoopWake;
impl Wake for NoopWake {
    fn wake(self: Arc<Self>) {}
}

#[test]
fn profiled_pulses_include_both_endpoints_and_do_not_accumulate_task_delays() {
    let clock = MockDriver::get();
    clock.reset();
    let events = Rc::new(RefCell::new(Vec::new()));
    let motor = AsyncStepperMotorBuilder::new()
        .name("test")
        .step_pin(Pin(events.clone()))
        .dir_pin(Pin(Rc::new(RefCell::new(Vec::new()))))
        .delay(embassy_time::Delay)
        .steps_per_revolution(360)
        .microsteps(Microsteps::FULL)
        .gear_ratio(1.0)
        .max_velocity(DegreesPerSec::new(1000.0))
        .max_acceleration(DegreesPerSecSquared::new(20000.0))
        .build()
        .unwrap();
    let mut moving = match motor.move_to_with_acceleration(
        Degrees::new(100.0),
        DegreesPerSecSquared::new(20000.0),
        DegreesPerSecSquared::new(10000.0),
    ) {
        Ok(moving) => moving,
        Err((_, err)) => panic!("move failed: {err}"),
    };
    let profile = MotionProfile::asymmetric_trapezoidal(100, 1000.0, 20000.0, 10000.0);
    let waker = Waker::from(Arc::new(NoopWake));
    let mut cx = Context::from_waker(&waker);
    let mut elapsed_ns = 0_u64;

    for step in 0..100 {
        elapsed_ns += profile.interval_at(step) as u64;
        let deadline = Instant::from_ticks(0) + Duration::from_nanos(elapsed_ns);
        let mut future = Box::pin(moving.step_async());
        assert!(future.as_mut().poll(&mut cx).is_pending());
        clock.advance(deadline - Instant::now());
        assert!(future.as_mut().poll(&mut cx).is_pending()); // STEP is high
        assert_eq!(events.borrow().last(), Some(&(true, deadline.as_ticks())));
        clock.advance(Duration::from_micros(2));
        assert!(
            matches!(future.as_mut().poll(&mut cx), Poll::Ready(Ok(done)) if done == (step == 99))
        );
        drop(future);
        if step < 99 {
            // Simulate time spent on other tasks after every pulse.
            clock.advance(Duration::from_micros(37));
        }
    }

    assert_eq!(moving.position_steps().0, 100);
    let pulses = events.borrow();
    assert_eq!(pulses.len(), 200);
    for pulse in pulses.chunks_exact(2) {
        assert!(pulse[0].0);
        assert!(!pulse[1].0);
        assert_eq!(pulse[1].1 - pulse[0].1, Duration::from_micros(2).as_ticks());
    }
    let last_edge = pulses[198].1 as f64 / embassy_time::TICK_HZ as f64;
    assert!((last_edge - 0.175).abs() < 2e-6);
}
