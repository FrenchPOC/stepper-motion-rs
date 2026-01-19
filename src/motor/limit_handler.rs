//! Limit switch handler with interrupt and callback support.
//!
//! This module provides interrupt-safe limit switch handling with callbacks
//! that can be triggered from hardware interrupt handlers.
//!
//! # Interrupt Safety
//!
//! The callbacks are designed to be called from interrupt context:
//! - Use function pointers (no heap allocation)
//! - Quick execution (just set flags or minimal work)
//! - Atomic flag updates for thread-safe communication
//!
//! # Usage Modes
//!
//! - **Polling Mode**: Switches are checked during motor stepping (default)
//! - **Interrupt Mode**: Callbacks are invoked from hardware interrupts

use core::sync::atomic::{AtomicBool, Ordering};

use crate::config::{LimitTriggerMode, SwitchPolarity};

/// Which limit switch was triggered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LimitType {
    /// Minimum travel limit.
    Min,
    /// Maximum travel limit.
    Max,
}

/// Limit switch event passed to callbacks.
#[derive(Debug, Clone, Copy)]
pub struct LimitEvent {
    /// Which limit was triggered.
    pub limit_type: LimitType,
    /// The polarity configuration of the switch.
    pub polarity: SwitchPolarity,
    /// Raw pin state when triggered.
    pub pin_state: bool,
}

impl LimitEvent {
    /// Create a new limit event.
    pub fn new(limit_type: LimitType, polarity: SwitchPolarity, pin_state: bool) -> Self {
        Self {
            limit_type,
            polarity,
            pin_state,
        }
    }

    /// Check if this event represents the switch being activated (not released).
    #[inline]
    pub fn is_activated(&self) -> bool {
        self.polarity.is_active(self.pin_state)
    }
}

/// Function pointer type for limit switch callbacks.
///
/// This is interrupt-safe as it uses a simple function pointer.
/// The callback should be quick and not allocate memory.
///
/// # Arguments
///
/// * `event` - Information about which limit was triggered
///
/// # Safety
///
/// This callback may be called from interrupt context. It must:
/// - Not allocate memory
/// - Not block or wait
/// - Complete quickly
/// - Only use interrupt-safe operations (atomics, etc.)
pub type LimitCallback = fn(LimitEvent);

/// Atomic flags for limit switch state.
///
/// These can be safely read/written from interrupt context and main code.
#[derive(Debug)]
pub struct LimitFlags {
    /// Minimum limit triggered flag.
    min_triggered: AtomicBool,
    /// Maximum limit triggered flag.
    max_triggered: AtomicBool,
    /// Emergency stop flag (set when any limit triggers in interrupt mode).
    emergency_stop: AtomicBool,
}

impl Default for LimitFlags {
    fn default() -> Self {
        Self::new()
    }
}

impl LimitFlags {
    /// Create new limit flags (all cleared).
    pub const fn new() -> Self {
        Self {
            min_triggered: AtomicBool::new(false),
            max_triggered: AtomicBool::new(false),
            emergency_stop: AtomicBool::new(false),
        }
    }

    /// Set the minimum limit triggered flag.
    #[inline]
    pub fn set_min_triggered(&self) {
        self.min_triggered.store(true, Ordering::SeqCst);
        self.emergency_stop.store(true, Ordering::SeqCst);
    }

    /// Set the maximum limit triggered flag.
    #[inline]
    pub fn set_max_triggered(&self) {
        self.max_triggered.store(true, Ordering::SeqCst);
        self.emergency_stop.store(true, Ordering::SeqCst);
    }

    /// Check and clear the minimum limit flag.
    #[inline]
    pub fn take_min_triggered(&self) -> bool {
        self.min_triggered.swap(false, Ordering::SeqCst)
    }

    /// Check and clear the maximum limit flag.
    #[inline]
    pub fn take_max_triggered(&self) -> bool {
        self.max_triggered.swap(false, Ordering::SeqCst)
    }

    /// Check if emergency stop is active.
    #[inline]
    pub fn is_emergency_stop(&self) -> bool {
        self.emergency_stop.load(Ordering::SeqCst)
    }

    /// Clear the emergency stop flag.
    #[inline]
    pub fn clear_emergency_stop(&self) {
        self.emergency_stop.store(false, Ordering::SeqCst);
    }

    /// Clear all flags.
    #[inline]
    pub fn clear_all(&self) {
        self.min_triggered.store(false, Ordering::SeqCst);
        self.max_triggered.store(false, Ordering::SeqCst);
        self.emergency_stop.store(false, Ordering::SeqCst);
    }

    /// Check minimum limit flag without clearing.
    #[inline]
    pub fn is_min_triggered(&self) -> bool {
        self.min_triggered.load(Ordering::SeqCst)
    }

    /// Check maximum limit flag without clearing.
    #[inline]
    pub fn is_max_triggered(&self) -> bool {
        self.max_triggered.load(Ordering::SeqCst)
    }
}

/// Limit switch handler with optional interrupt callbacks.
///
/// This handler can work in two modes:
/// - Polling: Switches checked during motor stepping
/// - Interrupt: Callbacks invoked from hardware interrupts
///
/// # Example
///
/// ```rust,ignore
/// use stepper_motion::motor::{LimitHandler, LimitFlags, LimitEvent, LimitTriggerMode};
///
/// // Create shared flags (typically static for interrupt access)
/// static LIMIT_FLAGS: LimitFlags = LimitFlags::new();
///
/// // Create handler with interrupt mode
/// let handler = LimitHandler::new(LimitTriggerMode::Interrupt)
///     .with_min_callback(on_min_limit)
///     .with_max_callback(on_max_limit)
///     .with_flags(&LIMIT_FLAGS);
///
/// // Callback function (called from interrupt)
/// fn on_min_limit(event: LimitEvent) {
///     if event.is_activated() {
///         // Set flag, don't do heavy work here
///         LIMIT_FLAGS.set_min_triggered();
///     }
/// }
/// ```
pub struct LimitHandler<'a> {
    /// Trigger mode (polling or interrupt).
    mode: LimitTriggerMode,
    /// Callback for minimum limit switch.
    min_callback: Option<LimitCallback>,
    /// Callback for maximum limit switch.
    max_callback: Option<LimitCallback>,
    /// Optional reference to atomic flags for interrupt communication.
    flags: Option<&'a LimitFlags>,
    /// Minimum limit switch polarity.
    min_polarity: SwitchPolarity,
    /// Maximum limit switch polarity.
    max_polarity: SwitchPolarity,
}

impl<'a> LimitHandler<'a> {
    /// Create a new limit handler with the specified trigger mode.
    pub fn new(mode: LimitTriggerMode) -> Self {
        Self {
            mode,
            min_callback: None,
            max_callback: None,
            flags: None,
            min_polarity: SwitchPolarity::NO,
            max_polarity: SwitchPolarity::NO,
        }
    }

    /// Create a handler in polling mode.
    pub fn polling() -> Self {
        Self::new(LimitTriggerMode::Polling)
    }

    /// Create a handler in interrupt mode.
    pub fn interrupt() -> Self {
        Self::new(LimitTriggerMode::Interrupt)
    }

    /// Set the minimum limit callback.
    pub fn with_min_callback(mut self, callback: LimitCallback) -> Self {
        self.min_callback = Some(callback);
        self
    }

    /// Set the maximum limit callback.
    pub fn with_max_callback(mut self, callback: LimitCallback) -> Self {
        self.max_callback = Some(callback);
        self
    }

    /// Set both limit callbacks to the same function.
    pub fn with_callback(mut self, callback: LimitCallback) -> Self {
        self.min_callback = Some(callback);
        self.max_callback = Some(callback);
        self
    }

    /// Set the atomic flags reference for interrupt communication.
    pub fn with_flags(mut self, flags: &'a LimitFlags) -> Self {
        self.flags = Some(flags);
        self
    }

    /// Set the minimum limit switch polarity.
    pub fn with_min_polarity(mut self, polarity: SwitchPolarity) -> Self {
        self.min_polarity = polarity;
        self
    }

    /// Set the maximum limit switch polarity.
    pub fn with_max_polarity(mut self, polarity: SwitchPolarity) -> Self {
        self.max_polarity = polarity;
        self
    }

    /// Get the trigger mode.
    #[inline]
    pub fn mode(&self) -> LimitTriggerMode {
        self.mode
    }

    /// Check if using interrupt mode.
    #[inline]
    pub fn is_interrupt_mode(&self) -> bool {
        self.mode == LimitTriggerMode::Interrupt
    }

    /// Get the atomic flags reference.
    #[inline]
    pub fn flags(&self) -> Option<&LimitFlags> {
        self.flags
    }

    /// Handle minimum limit switch trigger.
    ///
    /// This should be called from the interrupt handler when the min limit
    /// switch state changes.
    ///
    /// # Arguments
    ///
    /// * `pin_state` - Current state of the pin (true = high, false = low)
    #[inline]
    pub fn on_min_limit_interrupt(&self, pin_state: bool) {
        let event = LimitEvent::new(LimitType::Min, self.min_polarity, pin_state);

        // Set atomic flag if available
        if event.is_activated() {
            if let Some(flags) = self.flags {
                flags.set_min_triggered();
            }
        }

        // Call user callback if registered
        if let Some(callback) = self.min_callback {
            callback(event);
        }
    }

    /// Handle maximum limit switch trigger.
    ///
    /// This should be called from the interrupt handler when the max limit
    /// switch state changes.
    ///
    /// # Arguments
    ///
    /// * `pin_state` - Current state of the pin (true = high, false = low)
    #[inline]
    pub fn on_max_limit_interrupt(&self, pin_state: bool) {
        let event = LimitEvent::new(LimitType::Max, self.max_polarity, pin_state);

        // Set atomic flag if available
        if event.is_activated() {
            if let Some(flags) = self.flags {
                flags.set_max_triggered();
            }
        }

        // Call user callback if registered
        if let Some(callback) = self.max_callback {
            callback(event);
        }
    }

    /// Check if any limit has been triggered (from flags).
    ///
    /// This checks the atomic flags, useful for polling the interrupt state
    /// from main code.
    #[inline]
    pub fn check_limits_triggered(&self) -> Option<LimitType> {
        if let Some(flags) = self.flags {
            if flags.is_min_triggered() {
                return Some(LimitType::Min);
            }
            if flags.is_max_triggered() {
                return Some(LimitType::Max);
            }
        }
        None
    }

    /// Check if emergency stop is active.
    #[inline]
    pub fn is_emergency_stop(&self) -> bool {
        self.flags.map(|f| f.is_emergency_stop()).unwrap_or(false)
    }

    /// Clear all triggered flags.
    #[inline]
    pub fn clear_flags(&self) {
        if let Some(flags) = self.flags {
            flags.clear_all();
        }
    }
}

/// Default handler with polling mode.
impl Default for LimitHandler<'_> {
    fn default() -> Self {
        Self::polling()
    }
}

/// Helper macro to create a static limit flags instance.
///
/// This is useful for interrupt handlers that need static access to flags.
///
/// # Example
///
/// ```rust,ignore
/// use stepper_motion::limit_flags;
///
/// limit_flags!(MOTOR_X_LIMITS);
/// limit_flags!(MOTOR_Y_LIMITS);
/// ```
#[macro_export]
macro_rules! limit_flags {
    ($name:ident) => {
        static $name: $crate::motor::LimitFlags = $crate::motor::LimitFlags::new();
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_limit_flags() {
        let flags = LimitFlags::new();

        assert!(!flags.is_min_triggered());
        assert!(!flags.is_max_triggered());
        assert!(!flags.is_emergency_stop());

        flags.set_min_triggered();
        assert!(flags.is_min_triggered());
        assert!(flags.is_emergency_stop());

        // Take clears the flag
        assert!(flags.take_min_triggered());
        assert!(!flags.is_min_triggered());
        // But emergency stop remains
        assert!(flags.is_emergency_stop());

        flags.clear_all();
        assert!(!flags.is_emergency_stop());
    }

    #[test]
    fn test_limit_event() {
        // NO switch: low = active
        let event = LimitEvent::new(LimitType::Min, SwitchPolarity::NO, false);
        assert!(event.is_activated());

        let event = LimitEvent::new(LimitType::Min, SwitchPolarity::NO, true);
        assert!(!event.is_activated());

        // NC switch: high = active
        let event = LimitEvent::new(LimitType::Max, SwitchPolarity::NC, true);
        assert!(event.is_activated());
    }

    #[test]
    fn test_limit_handler_callback() {
        use core::sync::atomic::AtomicU32;

        static CALL_COUNT: AtomicU32 = AtomicU32::new(0);

        fn test_callback(_event: LimitEvent) {
            CALL_COUNT.fetch_add(1, Ordering::SeqCst);
        }

        let handler = LimitHandler::interrupt()
            .with_min_callback(test_callback)
            .with_min_polarity(SwitchPolarity::NO);

        // Simulate interrupt with pin going low (active for NO switch)
        handler.on_min_limit_interrupt(false);

        assert_eq!(CALL_COUNT.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn test_limit_handler_with_flags() {
        let flags = LimitFlags::new();

        let handler = LimitHandler::interrupt()
            .with_flags(&flags)
            .with_min_polarity(SwitchPolarity::NO)
            .with_max_polarity(SwitchPolarity::NC);

        // Trigger min limit (NO switch, low = active)
        handler.on_min_limit_interrupt(false);
        assert!(handler.is_emergency_stop());
        assert_eq!(handler.check_limits_triggered(), Some(LimitType::Min));

        handler.clear_flags();
        assert!(!handler.is_emergency_stop());
    }
}
