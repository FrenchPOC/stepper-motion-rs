//! Example: Limit switch interrupts with callbacks
//!
//! This example demonstrates how to use limit switch callbacks that can be
//! triggered from hardware interrupts for immediate motor stop.
//!
//! # Interrupt vs Polling
//!
//! - **Polling Mode**: Switches checked during stepping (simpler, some latency)
//! - **Interrupt Mode**: Callbacks invoked immediately via hardware (faster response)
//!
//! # Typical Usage Pattern
//!
//! 1. Create static `LimitFlags` for interrupt-safe communication
//! 2. Create `LimitHandler` with callbacks
//! 3. In interrupt handler, call `on_min_limit_interrupt()` or `on_max_limit_interrupt()`
//! 4. In main loop, check `is_emergency_stop()` to halt motor

use core::sync::atomic::{AtomicU32, Ordering};

use stepper_motion::{
    LimitEvent, LimitFlags, LimitHandler, LimitTriggerMode, LimitType, SwitchConfig, SwitchPolarity,
};

// Static flags for interrupt-safe communication between ISR and main code
// In real embedded code, this would be accessed from interrupt handlers
static MOTOR_LIMIT_FLAGS: LimitFlags = LimitFlags::new();

// Counter to track callback invocations (for demonstration)
static CALLBACK_COUNT: AtomicU32 = AtomicU32::new(0);

/// Callback function for limit switch events.
///
/// This function is designed to be called from interrupt context:
/// - No heap allocation
/// - Quick execution
/// - Only sets atomic flags
fn on_limit_triggered(event: LimitEvent) {
    // Increment callback counter
    CALLBACK_COUNT.fetch_add(1, Ordering::SeqCst);

    // Log which limit was hit (in real code, use defmt or similar)
    match event.limit_type {
        LimitType::Min => {
            println!(
                "  [ISR] Min limit triggered! Pin state: {}",
                event.pin_state
            );
        }
        LimitType::Max => {
            println!(
                "  [ISR] Max limit triggered! Pin state: {}",
                event.pin_state
            );
        }
    }

    // Check if this is an activation (not release)
    if event.is_activated() {
        println!("  [ISR] Switch ACTIVATED - emergency stop!");
    } else {
        println!("  [ISR] Switch released");
    }
}

/// Separate callback for min limit only
fn on_min_limit(event: LimitEvent) {
    if event.is_activated() {
        println!("  [ISR] MIN LIMIT HIT!");
    }
}

/// Separate callback for max limit only
fn on_max_limit(event: LimitEvent) {
    if event.is_activated() {
        println!("  [ISR] MAX LIMIT HIT!");
    }
}

fn main() {
    println!("Limit Switch Interrupt Example");
    println!("===============================\n");

    // Example 1: Basic limit handler with shared callback
    println!("1. Shared callback for both limits:");
    {
        let handler = LimitHandler::interrupt()
            .with_flags(&MOTOR_LIMIT_FLAGS)
            .with_callback(on_limit_triggered)
            .with_min_polarity(SwitchPolarity::NO)
            .with_max_polarity(SwitchPolarity::NO);

        // Simulate interrupt: min limit pin goes LOW (active for NO switch)
        println!("\n   Simulating min limit interrupt (pin LOW):");
        handler.on_min_limit_interrupt(false);

        // Check flags from main code
        println!("   Emergency stop active: {}", handler.is_emergency_stop());
        println!("   Triggered limit: {:?}", handler.check_limits_triggered());

        // Clear flags after handling
        handler.clear_flags();
        println!(
            "   After clearing: emergency_stop = {}",
            handler.is_emergency_stop()
        );
    }

    // Example 2: Separate callbacks for each limit
    println!("\n2. Separate callbacks for min/max:");
    {
        MOTOR_LIMIT_FLAGS.clear_all();

        let handler = LimitHandler::interrupt()
            .with_flags(&MOTOR_LIMIT_FLAGS)
            .with_min_callback(on_min_limit)
            .with_max_callback(on_max_limit)
            .with_min_polarity(SwitchPolarity::NO)
            .with_max_polarity(SwitchPolarity::NC); // NC switch for max

        // Simulate min limit (NO switch, LOW = active)
        println!("\n   Min limit (NO switch, pin LOW = active):");
        handler.on_min_limit_interrupt(false);

        handler.clear_flags();

        // Simulate max limit (NC switch, HIGH = active)
        println!("\n   Max limit (NC switch, pin HIGH = active):");
        handler.on_max_limit_interrupt(true);
    }

    // Example 3: Polling mode (no interrupts)
    println!("\n3. Polling mode (for simpler setups):");
    {
        let handler = LimitHandler::polling()
            .with_min_polarity(SwitchPolarity::NO)
            .with_max_polarity(SwitchPolarity::NO);

        println!("   Mode: {:?}", handler.mode());
        println!("   Is interrupt mode: {}", handler.is_interrupt_mode());
    }

    // Example 4: Switch configuration with trigger mode
    println!("\n4. Switch configuration with trigger mode:");
    {
        // Polling mode (default)
        let switch_polling = SwitchConfig::new(SwitchPolarity::NO);
        println!(
            "   Polling switch: polarity={:?}, interrupt={}",
            switch_polling.polarity,
            switch_polling.is_interrupt_mode()
        );

        // Interrupt mode
        let switch_interrupt = SwitchConfig::with_interrupt(SwitchPolarity::NC);
        println!(
            "   Interrupt switch: polarity={:?}, interrupt={}",
            switch_interrupt.polarity,
            switch_interrupt.is_interrupt_mode()
        );

        // Builder pattern
        let switch_custom =
            SwitchConfig::new(SwitchPolarity::NO).with_trigger_mode(LimitTriggerMode::Interrupt);
        println!(
            "   Custom switch: polarity={:?}, interrupt={}",
            switch_custom.polarity,
            switch_custom.is_interrupt_mode()
        );
    }

    // Example 5: Typical interrupt handler pattern
    println!("\n5. Typical interrupt handler pattern:");
    println!(
        r#"
   // In your embedded code:

   // 1. Define static flags (accessible from ISR)
   static LIMIT_FLAGS: LimitFlags = LimitFlags::new();

   // 2. Create handler (in main, before enabling interrupts)
   let handler = LimitHandler::interrupt()
       .with_flags(&LIMIT_FLAGS)
       .with_min_polarity(SwitchPolarity::NO)
       .with_max_polarity(SwitchPolarity::NO);

   // 3. In your GPIO interrupt handler:
   #[interrupt]
   fn EXTI0() {{
       let pin_state = limit_min_pin.is_high().unwrap();
       handler.on_min_limit_interrupt(pin_state);
       // Clear interrupt flag...
   }}

   // 4. In your motor stepping loop:
   loop {{
       if LIMIT_FLAGS.is_emergency_stop() {{
           motor.stop();
           break;
       }}
       motor.step();
   }}
"#
    );

    // Example 6: TOML configuration
    println!("6. TOML Configuration Example:");
    println!(
        r#"
   [motors.switches.limit_min]
   polarity = "NO"
   enabled = true
   trigger_mode = "interrupt"

   [motors.switches.limit_max]
   polarity = "NC"
   enabled = true
   trigger_mode = "polling"
"#
    );

    println!(
        "Total callbacks invoked: {}",
        CALLBACK_COUNT.load(Ordering::SeqCst)
    );
    println!("\nLimit switch interrupt example complete!");
}
