//! Embassy-Compatible Pattern Example (Testable without hardware)
//!
//! This example demonstrates the interrupt callback pattern used with Embassy
//! but in a way that can be compiled and tested on a host machine.
//!
//! For actual MCU deployment, see `embassy_mcu.rs`.
//!
//! # Key Concepts
//!
//! 1. **Static LimitFlags**: Atomic flags shared between "interrupt" context and main
//! 2. **Function pointer callbacks**: `fn(LimitEvent)` - no closures, interrupt-safe
//! 3. **Polarity handling**: Callbacks auto-apply NO/NC logic
//! 4. **Emergency stop**: Automatic flag set on limit activation

use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::thread;
use std::time::Duration;

use stepper_motion::{
    Degrees, DegreesPerSec, HomingConfig, HomingDirection, HomingStrategy, LimitEvent, LimitFlags,
    LimitHandler, LimitType, SwitchPolarity,
};

// =============================================================================
// GLOBAL STATE - Simulates what would be static in MCU firmware
// =============================================================================

/// Limit switch flags - shared between "interrupt" thread and main
static LIMIT_FLAGS: LimitFlags = LimitFlags::new();

/// Home switch found flag (separate from emergency stop)
static HOME_FOUND: AtomicBool = AtomicBool::new(false);

/// Simulated motor position
static MOTOR_POSITION: AtomicI32 = AtomicI32::new(0);

/// Motor running flag
static MOTOR_RUNNING: AtomicBool = AtomicBool::new(false);

// =============================================================================
// INTERRUPT CALLBACKS - Would be called from GPIO ISR on MCU
// =============================================================================

/// Min limit callback - called when min limit switch changes state
fn on_min_limit(event: LimitEvent) {
    println!(
        "[ISR] Min limit: pin={}, activated={}",
        event.pin_state,
        event.is_activated()
    );

    if event.is_activated() {
        println!("[ISR] ⛔ MIN LIMIT HIT - Emergency stop!");
    }
}

/// Max limit callback - called when max limit switch changes state  
fn on_max_limit(event: LimitEvent) {
    println!(
        "[ISR] Max limit: pin={}, activated={}",
        event.pin_state,
        event.is_activated()
    );

    if event.is_activated() {
        println!("[ISR] ⛔ MAX LIMIT HIT - Emergency stop!");
    }
}

/// Home switch callback - doesn't trigger emergency stop
fn on_home_switch(_event: LimitEvent) {
    println!("[ISR] 🏠 Home switch triggered!");
    HOME_FOUND.store(true, Ordering::SeqCst);
}

// =============================================================================
// SIMULATED HARDWARE - Mimics GPIO interrupt behavior
// =============================================================================

/// Simulates GPIO interrupt when position reaches limits
fn simulate_limit_interrupts(min_pos: i32, max_pos: i32, home_pos: i32) {
    // Create handlers (would be done once in firmware init)
    let min_handler = LimitHandler::interrupt()
        .with_flags(&LIMIT_FLAGS)
        .with_min_callback(on_min_limit)
        .with_min_polarity(SwitchPolarity::NO);

    let max_handler = LimitHandler::interrupt()
        .with_flags(&LIMIT_FLAGS)
        .with_max_callback(on_max_limit)
        .with_max_polarity(SwitchPolarity::NO);

    println!("🔌 [HW SIM] Interrupt simulator started");
    println!(
        "          Min limit at: {}, Max limit at: {}, Home at: {}",
        min_pos, max_pos, home_pos
    );

    let mut last_min_active = false;
    let mut last_max_active = false;
    let mut last_home_active = false;

    loop {
        let pos = MOTOR_POSITION.load(Ordering::SeqCst);

        // Check min limit (active when pos <= min_pos)
        let min_active = pos <= min_pos;
        if min_active != last_min_active {
            // Simulate GPIO interrupt: NO switch, active = LOW
            let pin_high = !min_active;
            min_handler.on_min_limit_interrupt(pin_high);
            last_min_active = min_active;
        }

        // Check max limit (active when pos >= max_pos)
        let max_active = pos >= max_pos;
        if max_active != last_max_active {
            let pin_high = !max_active;
            max_handler.on_max_limit_interrupt(pin_high);
            last_max_active = max_active;
        }

        // Check home switch (active when at home position ±10)
        let home_active = (pos - home_pos).abs() <= 10;
        if home_active && !last_home_active {
            on_home_switch(LimitEvent::new(
                LimitType::Min, // Using Min as placeholder
                SwitchPolarity::NO,
                false, // pin_state LOW = active for NO
            ));
        }
        last_home_active = home_active;

        // Exit when motor stops
        if !MOTOR_RUNNING.load(Ordering::SeqCst) {
            break;
        }

        thread::sleep(Duration::from_micros(100));
    }

    println!("🔌 [HW SIM] Interrupt simulator stopped");
}

// =============================================================================
// MOTOR CONTROL - Simulates async motor stepping
// =============================================================================

/// Move motor by specified number of steps, checking limits
fn move_steps(steps: i32) -> i32 {
    let direction = if steps >= 0 { 1 } else { -1 };
    let mut completed = 0;

    println!(
        "🔄 Moving {} steps (direction: {})",
        steps.abs(),
        if direction > 0 { "forward" } else { "reverse" }
    );

    for _ in 0..steps.abs() {
        // Check emergency stop (set by interrupt callback)
        if LIMIT_FLAGS.is_emergency_stop() {
            println!("⚠️  Emergency stop detected!");
            break;
        }

        // Direction-specific limit checks
        if direction > 0 && LIMIT_FLAGS.is_max_triggered() {
            println!("⚠️  Max limit active, cannot move forward");
            break;
        }
        if direction < 0 && LIMIT_FLAGS.is_min_triggered() {
            println!("⚠️  Min limit active, cannot move reverse");
            break;
        }

        // Perform step
        MOTOR_POSITION.fetch_add(direction, Ordering::SeqCst);
        completed += 1;

        // Simulate step timing (would be async in real code)
        thread::sleep(Duration::from_micros(50));
    }

    println!(
        "   Completed {} of {} steps. Position: {}",
        completed,
        steps.abs(),
        MOTOR_POSITION.load(Ordering::SeqCst)
    );

    completed * direction
}

/// Perform homing sequence
fn perform_homing(config: &HomingConfig) -> Result<(), &'static str> {
    println!("\n🏠 Starting homing sequence...");
    println!("   Strategy: {:?}", config.strategy);
    println!("   Direction: {:?}", config.direction);

    // Clear flags
    LIMIT_FLAGS.clear_all();
    HOME_FOUND.store(false, Ordering::SeqCst);

    let direction = match config.direction {
        HomingDirection::ToMin => -1,
        HomingDirection::ToMax => 1,
    };

    // Phase 1: Fast approach to home switch
    println!("\n   Phase 1: Fast approach");
    let max_steps = (config.max_travel.0 * 100.0) as i32;

    for step in 0..max_steps {
        if LIMIT_FLAGS.is_emergency_stop() {
            return Err("Emergency stop during homing");
        }

        if HOME_FOUND.load(Ordering::SeqCst) {
            println!("   Home switch found after {} steps", step);
            break;
        }

        MOTOR_POSITION.fetch_add(direction, Ordering::SeqCst);
        thread::sleep(Duration::from_micros(20));
    }

    if !HOME_FOUND.load(Ordering::SeqCst) {
        return Err("Home switch not found within max travel");
    }

    // Phase 2: Back off
    println!("   Phase 2: Backing off {:?}", config.backoff_distance);
    HOME_FOUND.store(false, Ordering::SeqCst);

    let backoff_steps = (config.backoff_distance.0 * 100.0) as i32;
    for _ in 0..backoff_steps {
        MOTOR_POSITION.fetch_add(-direction, Ordering::SeqCst);
        thread::sleep(Duration::from_micros(20));
    }

    // Phase 3: Slow approach
    println!("   Phase 3: Slow approach");
    for _ in 0..backoff_steps * 2 {
        if HOME_FOUND.load(Ordering::SeqCst) {
            break;
        }
        MOTOR_POSITION.fetch_add(direction, Ordering::SeqCst);
        thread::sleep(Duration::from_micros(100)); // Slower
    }

    // Set home position
    let home_offset = (config.home_offset.0 * 100.0) as i32;
    MOTOR_POSITION.store(home_offset, Ordering::SeqCst);

    println!("✅ Homing complete! Position set to {}", home_offset);
    Ok(())
}

// =============================================================================
// MAIN
// =============================================================================

fn main() {
    println!("Embassy-Compatible Interrupt Pattern Example");
    println!("============================================\n");

    // Define virtual limit switch positions
    let min_limit_pos = -1000;
    let max_limit_pos = 1000;
    let home_pos = 0;

    // Start motor at center
    MOTOR_POSITION.store(500, Ordering::SeqCst);
    MOTOR_RUNNING.store(true, Ordering::SeqCst);

    // Spawn "interrupt simulator" thread (mimics GPIO EXTI on MCU)
    let interrupt_thread = thread::spawn(move || {
        simulate_limit_interrupts(min_limit_pos, max_limit_pos, home_pos);
    });

    // Wait for simulator to start
    thread::sleep(Duration::from_millis(10));

    // Configure homing
    let homing_config = HomingConfig {
        strategy: HomingStrategy::HomeSwitchFast,
        direction: HomingDirection::ToMin,
        fast_velocity: DegreesPerSec(10.0),
        slow_velocity: DegreesPerSec(2.0),
        backoff_distance: Degrees(2.0), // 200 steps at 100 steps/mm
        home_offset: Degrees(0.0),
        max_travel: Degrees(50.0),
        home_position: Degrees(0.0),
    };

    // Perform homing
    match perform_homing(&homing_config) {
        Ok(()) => println!("\n✅ Homing successful!"),
        Err(e) => println!("\n❌ Homing failed: {}", e),
    }

    // Clear any flags from homing
    LIMIT_FLAGS.clear_all();
    println!("\n--- Beginning motion tests ---\n");

    // Test 1: Normal motion
    println!("Test 1: Normal motion (should complete)");
    move_steps(300);

    // Test 2: Motion that hits max limit
    println!("\nTest 2: Motion towards max limit (should hit limit)");
    move_steps(1000); // Will hit max at 1000

    // Clear flags and try reverse
    println!("\n   Clearing flags...");
    LIMIT_FLAGS.clear_all();
    thread::sleep(Duration::from_millis(10));

    // Test 3: Motion that hits min limit
    println!("\nTest 3: Motion towards min limit (should hit limit)");
    move_steps(-3000); // Will hit min at -1000

    // Done
    MOTOR_RUNNING.store(false, Ordering::SeqCst);
    interrupt_thread.join().unwrap();

    println!("\n============================================");
    println!("Example complete!");
    println!("Final position: {}", MOTOR_POSITION.load(Ordering::SeqCst));
}
