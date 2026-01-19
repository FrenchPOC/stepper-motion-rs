//! Complete Embassy MCU Example with Interrupt-Driven Limit Switches
//!
//! This example demonstrates a complete stepper motor control system using:
//! - Embassy async runtime
//! - Hardware GPIO interrupts for limit switches
//! - Async motor stepping with immediate stop on limit trigger
//!
//! # Target Hardware
//! This example is written for STM32 but can be adapted to other Embassy-supported MCUs.
//!
//! # Pin Configuration (example for STM32F4)
//! - PA0: Step output
//! - PA1: Direction output
//! - PA2: Enable output
//! - PB0: Home switch input (with interrupt)
//! - PB1: Min limit switch input (with interrupt)
//! - PB2: Max limit switch input (with interrupt)
//!
//! # Build
//! ```bash
//! cargo build --example embassy_mcu --target thumbv7em-none-eabihf --features embassy
//! ```

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, warn, error};
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{PA0, PA1, PA2, PB0, PB1, PB2};
use embassy_stm32::{bind_interrupts, exti};
use embassy_time::{Duration, Ticker, Timer};

use stepper_motion::{
    Degrees, DegreesPerSec,
    LimitEvent, LimitFlags, LimitHandler, LimitType,
    SwitchPolarity, HomingConfig, HomingStrategy, HomingDirection,
};

// =============================================================================
// STATIC FLAGS - Accessible from interrupt context
// =============================================================================

/// Global limit switch flags for interrupt-safe communication
static LIMIT_FLAGS: LimitFlags = LimitFlags::new();

/// Flag to signal that homing is complete
static HOMING_COMPLETE: AtomicBool = AtomicBool::new(false);

/// Flag to signal motor should be running
static MOTOR_RUNNING: AtomicBool = AtomicBool::new(false);

// =============================================================================
// LIMIT SWITCH CALLBACKS - Called from interrupt context
// =============================================================================

/// Callback for min limit switch events
/// 
/// This is called directly from the GPIO interrupt handler.
/// Keep it fast and simple - only set flags!
fn on_min_limit_triggered(event: LimitEvent) {
    if event.is_activated() {
        defmt::warn!("MIN LIMIT TRIGGERED - Emergency stop!");
        // The LimitHandler already sets emergency_stop flag
        // We can add additional logging or actions here
    } else {
        defmt::info!("Min limit released");
    }
}

/// Callback for max limit switch events
fn on_max_limit_triggered(event: LimitEvent) {
    if event.is_activated() {
        defmt::warn!("MAX LIMIT TRIGGERED - Emergency stop!");
    } else {
        defmt::info!("Max limit released");
    }
}

/// Callback for home switch events (used during homing sequence)
fn on_home_switch_triggered(event: LimitEvent) {
    if event.is_activated() {
        defmt::info!("Home switch activated");
        HOMING_COMPLETE.store(true, Ordering::SeqCst);
    }
}

// =============================================================================
// MOTOR DRIVER - Async stepper control
// =============================================================================

/// Simple async stepper motor driver
struct AsyncMotor<'d> {
    step_pin: Output<'d>,
    dir_pin: Output<'d>,
    enable_pin: Output<'d>,
    step_delay_us: u64,
    current_position: i32,
}

impl<'d> AsyncMotor<'d> {
    fn new(
        step_pin: Output<'d>,
        dir_pin: Output<'d>,
        enable_pin: Output<'d>,
    ) -> Self {
        Self {
            step_pin,
            dir_pin,
            enable_pin,
            step_delay_us: 500, // 1kHz stepping
            current_position: 0,
        }
    }

    fn enable(&mut self) {
        self.enable_pin.set_low(); // Active low enable
        defmt::info!("Motor enabled");
    }

    fn disable(&mut self) {
        self.enable_pin.set_high();
        defmt::info!("Motor disabled");
    }

    fn set_direction_forward(&mut self) {
        self.dir_pin.set_high();
    }

    fn set_direction_reverse(&mut self) {
        self.dir_pin.set_low();
    }

    fn set_speed_hz(&mut self, hz: u32) {
        if hz > 0 {
            self.step_delay_us = 1_000_000 / (hz as u64 * 2);
        }
    }

    /// Perform a single step (non-blocking)
    async fn step(&mut self) {
        self.step_pin.set_high();
        Timer::after(Duration::from_micros(self.step_delay_us)).await;
        self.step_pin.set_low();
        Timer::after(Duration::from_micros(self.step_delay_us)).await;
    }

    /// Move a number of steps, checking limit flags between each step
    /// Returns the actual number of steps completed
    async fn move_steps(&mut self, steps: i32, flags: &LimitFlags) -> i32 {
        let direction = if steps >= 0 { 1 } else { -1 };
        let abs_steps = steps.abs() as u32;

        if direction > 0 {
            self.set_direction_forward();
        } else {
            self.set_direction_reverse();
        }

        let mut completed = 0i32;

        for _ in 0..abs_steps {
            // Check for emergency stop before each step
            if flags.is_emergency_stop() {
                defmt::warn!("Emergency stop detected! Halting at step {}", completed);
                break;
            }

            // Check specific limits based on direction
            if direction > 0 && flags.is_max_triggered() {
                defmt::warn!("Max limit active, cannot move forward");
                break;
            }
            if direction < 0 && flags.is_min_triggered() {
                defmt::warn!("Min limit active, cannot move reverse");
                break;
            }

            self.step().await;
            completed += direction;
            self.current_position += direction;
        }

        completed
    }

    /// Run continuously in one direction until limit or stop flag
    async fn run_until_limit(&mut self, forward: bool, flags: &LimitFlags) {
        if forward {
            self.set_direction_forward();
        } else {
            self.set_direction_reverse();
        }

        loop {
            if flags.is_emergency_stop() {
                break;
            }
            if forward && flags.is_max_triggered() {
                break;
            }
            if !forward && flags.is_min_triggered() {
                break;
            }

            self.step().await;
            self.current_position += if forward { 1 } else { -1 };
        }
    }

    fn position(&self) -> i32 {
        self.current_position
    }

    fn set_position(&mut self, pos: i32) {
        self.current_position = pos;
    }
}

// =============================================================================
// HOMING SEQUENCE - Async homing with interrupt-driven home switch
// =============================================================================

/// Perform homing sequence using home switch interrupt
async fn perform_homing<'d>(
    motor: &mut AsyncMotor<'d>,
    config: &HomingConfig,
    flags: &LimitFlags,
) -> Result<(), &'static str> {
    defmt::info!("Starting homing sequence...");

    // Reset flags
    flags.clear_all();
    HOMING_COMPLETE.store(false, Ordering::SeqCst);

    // Determine direction
    let forward = match config.direction {
        HomingDirection::ToMin => false,
        HomingDirection::ToMax => true,
    };

    // Set homing speed (fast approach)
    let fast_hz = (config.fast_velocity.0 * 1000.0) as u32;
    motor.set_speed_hz(fast_hz);
    motor.enable();

    defmt::info!("Phase 1: Fast approach at {} Hz", fast_hz);

    // Move towards home switch
    if forward {
        motor.set_direction_forward();
    } else {
        motor.set_direction_reverse();
    }

    // Step until home switch triggers (set by interrupt callback)
    let max_steps = (config.max_travel.0 * 1000.0) as u32; // deg to steps (assuming 1000 steps/deg)
    let mut steps = 0u32;

    while !HOMING_COMPLETE.load(Ordering::SeqCst) {
        if flags.is_emergency_stop() {
            motor.disable();
            return Err("Emergency stop during homing");
        }

        motor.step().await;
        steps += 1;

        if steps > max_steps {
            motor.disable();
            return Err("Homing timeout - max travel exceeded");
        }
    }

    defmt::info!("Home switch found after {} steps", steps);

    // Phase 2: Back off from switch
    let backoff_steps = (config.backoff_distance.0 * 1000.0) as u32;
    defmt::info!("Phase 2: Backing off {} steps", backoff_steps);

    HOMING_COMPLETE.store(false, Ordering::SeqCst);

    // Reverse direction
    if forward {
        motor.set_direction_reverse();
    } else {
        motor.set_direction_forward();
    }

    for _ in 0..backoff_steps {
        motor.step().await;
    }

    // Phase 3: Slow approach
    let slow_hz = (config.slow_velocity.0 * 1000.0) as u32;
    motor.set_speed_hz(slow_hz);
    defmt::info!("Phase 3: Slow approach at {} Hz", slow_hz);

    // Reverse direction again (towards home)
    if forward {
        motor.set_direction_forward();
    } else {
        motor.set_direction_reverse();
    }

    while !HOMING_COMPLETE.load(Ordering::SeqCst) {
        motor.step().await;
    }

    // Set position to home offset
    let home_position = (config.home_offset.0 * 1000.0) as i32;
    motor.set_position(home_position);

    defmt::info!("Homing complete! Position set to {}", home_position);
    Ok(())
}

// =============================================================================
// INTERRUPT HANDLERS - GPIO EXTI tasks
// =============================================================================

/// Task that handles min limit switch interrupts
#[embassy_executor::task]
async fn min_limit_task(mut pin: ExtiInput<'static>) {
    // Create handler with callback
    let handler = LimitHandler::interrupt()
        .with_flags(&LIMIT_FLAGS)
        .with_min_callback(on_min_limit_triggered)
        .with_min_polarity(SwitchPolarity::NO); // Normally Open switch

    loop {
        // Wait for any edge (both press and release)
        pin.wait_for_any_edge().await;
        
        // Read current pin state and invoke callback
        let pin_high = pin.is_high();
        handler.on_min_limit_interrupt(pin_high);
        
        // Small debounce delay
        Timer::after(Duration::from_millis(5)).await;
    }
}

/// Task that handles max limit switch interrupts
#[embassy_executor::task]
async fn max_limit_task(mut pin: ExtiInput<'static>) {
    let handler = LimitHandler::interrupt()
        .with_flags(&LIMIT_FLAGS)
        .with_max_callback(on_max_limit_triggered)
        .with_max_polarity(SwitchPolarity::NO);

    loop {
        pin.wait_for_any_edge().await;
        
        let pin_high = pin.is_high();
        handler.on_max_limit_interrupt(pin_high);
        
        Timer::after(Duration::from_millis(5)).await;
    }
}

/// Task that handles home switch interrupts
#[embassy_executor::task]
async fn home_switch_task(mut pin: ExtiInput<'static>) {
    // Home switch just sets a flag, doesn't trigger emergency stop
    loop {
        pin.wait_for_any_edge().await;
        
        let is_active = !pin.is_high(); // NO switch: LOW = active
        if is_active {
            HOMING_COMPLETE.store(true, Ordering::SeqCst);
            on_home_switch_triggered(LimitEvent {
                limit_type: LimitType::Min, // Using Min as placeholder
                pin_state: false,
                is_activated: true,
            });
        }
        
        Timer::after(Duration::from_millis(5)).await;
    }
}

// =============================================================================
// STATUS REPORTING TASK
// =============================================================================

/// Task that periodically reports motor status
#[embassy_executor::task]
async fn status_task() {
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;

        defmt::info!(
            "Status: running={}, emergency={}, min={}, max={}",
            MOTOR_RUNNING.load(Ordering::Relaxed),
            LIMIT_FLAGS.is_emergency_stop(),
            LIMIT_FLAGS.is_min_triggered(),
            LIMIT_FLAGS.is_max_triggered(),
        );
    }
}

// =============================================================================
// MAIN ENTRY POINT
// =============================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("Embassy Stepper Motor Example Starting...");

    // Initialize peripherals
    let p = embassy_stm32::init(Default::default());

    // Configure motor output pins
    let step_pin = Output::new(p.PA0, Level::Low, Speed::High);
    let dir_pin = Output::new(p.PA1, Level::Low, Speed::High);
    let enable_pin = Output::new(p.PA2, Level::High, Speed::High); // Active low

    // Configure limit switch input pins with pull-ups and interrupts
    let home_input = Input::new(p.PB0, Pull::Up);
    let min_input = Input::new(p.PB1, Pull::Up);
    let max_input = Input::new(p.PB2, Pull::Up);

    // Wrap inputs with EXTI for interrupt capability
    let home_exti = ExtiInput::new(home_input, p.EXTI0);
    let min_exti = ExtiInput::new(min_input, p.EXTI1);
    let max_exti = ExtiInput::new(max_input, p.EXTI2);

    // Spawn interrupt handler tasks
    spawner.spawn(home_switch_task(home_exti)).unwrap();
    spawner.spawn(min_limit_task(min_exti)).unwrap();
    spawner.spawn(max_limit_task(max_exti)).unwrap();
    spawner.spawn(status_task()).unwrap();

    defmt::info!("Interrupt handlers spawned");

    // Create motor driver
    let mut motor = AsyncMotor::new(step_pin, dir_pin, enable_pin);
    motor.set_speed_hz(2000); // 2kHz stepping

    // Configure homing
    let homing_config = HomingConfig {
        strategy: HomingStrategy::HomeSwitchFast,
        direction: HomingDirection::ToMin,
        fast_velocity: DegreesPerSec(10.0),    // 10 deg/s
        slow_velocity: DegreesPerSec(2.0),     // 2 deg/s
        backoff_distance: Degrees(5.0),        // 5 degrees
        home_offset: Degrees(0.0),             // Home = 0
        max_travel: Degrees(200.0),            // 200 degrees max
        home_position: Degrees(0.0),
    };

    // Wait a moment for everything to initialize
    Timer::after(Duration::from_millis(100)).await;

    // Perform homing sequence
    match perform_homing(&mut motor, &homing_config, &LIMIT_FLAGS).await {
        Ok(()) => defmt::info!("Homing successful!"),
        Err(e) => {
            defmt::error!("Homing failed: {}", e);
            // In production, you might want to retry or alert the user
        }
    }

    // Clear any flags from homing
    LIMIT_FLAGS.clear_all();

    // Main control loop
    defmt::info!("Entering main control loop");
    MOTOR_RUNNING.store(true, Ordering::SeqCst);

    loop {
        // Example: Move forward 1000 steps
        defmt::info!("Moving forward 1000 steps...");
        let completed = motor.move_steps(1000, &LIMIT_FLAGS).await;
        defmt::info!("Completed {} steps, position: {}", completed, motor.position());

        if LIMIT_FLAGS.is_emergency_stop() {
            defmt::warn!("Limit triggered! Waiting for clear...");
            
            // Wait for limit to be cleared (switch released)
            while LIMIT_FLAGS.is_emergency_stop() {
                Timer::after(Duration::from_millis(100)).await;
            }
            defmt::info!("Limit cleared, resuming...");
            continue;
        }

        // Wait before reversing
        Timer::after(Duration::from_secs(1)).await;

        // Example: Move backward 1000 steps
        defmt::info!("Moving backward 1000 steps...");
        let completed = motor.move_steps(-1000, &LIMIT_FLAGS).await;
        defmt::info!("Completed {} steps, position: {}", completed, motor.position());

        if LIMIT_FLAGS.is_emergency_stop() {
            defmt::warn!("Limit triggered! Waiting for clear...");
            while LIMIT_FLAGS.is_emergency_stop() {
                Timer::after(Duration::from_millis(100)).await;
            }
            defmt::info!("Limit cleared, resuming...");
            continue;
        }

        // Wait before next cycle
        Timer::after(Duration::from_secs(1)).await;
    }
}

// =============================================================================
// CONFIGURATION NOTES
// =============================================================================
//
// To build this example, you need to:
//
// 1. Add embassy dependencies to Cargo.toml:
//    ```toml
//    [dependencies]
//    embassy-executor = { version = "0.5", features = ["arch-cortex-m", "executor-thread"] }
//    embassy-stm32 = { version = "0.1", features = ["stm32f411ce", "time-driver-any"] }
//    embassy-time = { version = "0.3" }
//    defmt = "0.3"
//    defmt-rtt = "0.4"
//    panic-probe = { version = "0.3", features = ["print-defmt"] }
//    cortex-m = { version = "0.7", features = ["critical-section-single-core"] }
//    cortex-m-rt = "0.7"
//    ```
//
// 2. Create .cargo/config.toml:
//    ```toml
//    [target.thumbv7em-none-eabihf]
//    runner = "probe-rs run --chip STM32F411CEUx"
//
//    [build]
//    target = "thumbv7em-none-eabihf"
//    ```
//
// 3. Create memory.x for your specific MCU
//
// 4. Build and flash:
//    ```bash
//    cargo build --example embassy_mcu --release
//    probe-rs run --chip STM32F411CEUx target/thumbv7em-none-eabihf/release/examples/embassy_mcu
//    ```
