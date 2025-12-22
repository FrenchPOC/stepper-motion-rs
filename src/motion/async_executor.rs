//! Async motion execution using embassy-time.
//!
//! This module provides async motion execution compatible with both std and no_std
//! environments using the embassy-time crate for async delays.

use embassy_time::{Duration, Timer};

use super::profile::{MotionPhase, MotionProfile};

/// Async runtime state during motion execution.
///
/// Similar to `MotionExecutor` but uses async delays instead of blocking.
#[derive(Debug, Clone)]
pub struct AsyncMotionExecutor {
    /// The computed profile being executed.
    profile: MotionProfile,

    /// Current step number (0 to total_steps - 1).
    current_step: u32,

    /// Current step interval in nanoseconds.
    current_interval_ns: u32,

    /// Current phase of motion.
    phase: MotionPhase,
}

impl AsyncMotionExecutor {
    /// Create a new async executor for a motion profile.
    pub fn new(profile: MotionProfile) -> Self {
        let phase = if profile.is_zero() {
            MotionPhase::Complete
        } else {
            MotionPhase::Accelerating
        };

        let interval = if profile.is_zero() {
            u32::MAX
        } else {
            profile.initial_interval_ns
        };

        Self {
            profile,
            current_step: 0,
            current_interval_ns: interval,
            phase,
        }
    }

    /// Check if motion is complete.
    #[inline]
    pub fn is_complete(&self) -> bool {
        self.phase == MotionPhase::Complete
    }

    /// Get the current step number.
    #[inline]
    pub fn current_step(&self) -> u32 {
        self.current_step
    }

    /// Get the total number of steps.
    #[inline]
    pub fn total_steps(&self) -> u32 {
        self.profile.total_steps
    }

    /// Get steps remaining.
    #[inline]
    pub fn steps_remaining(&self) -> u32 {
        self.profile.total_steps.saturating_sub(self.current_step)
    }

    /// Get the current phase.
    #[inline]
    pub fn phase(&self) -> MotionPhase {
        self.phase
    }

    /// Get the current step interval in nanoseconds.
    #[inline]
    pub fn current_interval_ns(&self) -> u32 {
        self.current_interval_ns
    }

    /// Get the motion profile.
    #[inline]
    pub fn profile(&self) -> &MotionProfile {
        &self.profile
    }

    /// Advance to the next step.
    ///
    /// Returns `true` if a step should be executed, `false` if complete.
    pub fn advance(&mut self) -> bool {
        if self.is_complete() {
            return false;
        }

        self.current_step += 1;

        if self.current_step >= self.profile.total_steps {
            self.phase = MotionPhase::Complete;
            self.current_interval_ns = u32::MAX;
            return false;
        }

        // Update phase and interval
        self.phase = self.profile.phase_at(self.current_step);
        self.current_interval_ns = self.profile.interval_at(self.current_step);

        true
    }

    /// Wait for the current step interval asynchronously.
    ///
    /// This method uses embassy-time for non-blocking delays, allowing
    /// other tasks to run while waiting.
    pub async fn wait_step_interval(&self) {
        if self.current_interval_ns > 0 && self.current_interval_ns < u32::MAX {
            // Subtract pulse width (2us = 2000ns) as done in blocking version
            let delay_ns = self.current_interval_ns.saturating_sub(2000);
            if delay_ns > 0 {
                // Convert to Duration - embassy-time supports nanosecond precision
                let duration = Duration::from_nanos(delay_ns as u64);
                Timer::after(duration).await;
            }
        }
    }

    /// Reset the executor to the beginning.
    pub fn reset(&mut self) {
        self.current_step = 0;
        self.phase = if self.profile.is_zero() {
            MotionPhase::Complete
        } else {
            MotionPhase::Accelerating
        };
        self.current_interval_ns = if self.profile.is_zero() {
            u32::MAX
        } else {
            self.profile.initial_interval_ns
        };
    }

    /// Get progress as a percentage (0.0 to 1.0).
    #[inline]
    pub fn progress(&self) -> f32 {
        if self.profile.total_steps == 0 {
            1.0
        } else {
            self.current_step as f32 / self.profile.total_steps as f32
        }
    }
}

/// Async delay helper for step pulse timing.
///
/// Uses embassy-time for cooperative async delays.
pub async fn async_delay_us(microseconds: u32) {
    Timer::after(Duration::from_micros(microseconds as u64)).await;
}

/// Async delay helper for nanosecond precision.
pub async fn async_delay_ns(nanoseconds: u32) {
    Timer::after(Duration::from_nanos(nanoseconds as u64)).await;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_async_executor_creation() {
        let profile = MotionProfile::symmetric_trapezoidal(10, 1000.0, 2000.0);
        let executor = AsyncMotionExecutor::new(profile);

        assert!(!executor.is_complete());
        assert_eq!(executor.current_step(), 0);
        assert_eq!(executor.total_steps(), 10);
    }

    #[test]
    fn test_async_executor_advance() {
        let profile = MotionProfile::symmetric_trapezoidal(10, 1000.0, 2000.0);
        let mut executor = AsyncMotionExecutor::new(profile);

        // Advance through all steps
        while executor.advance() {}

        assert!(executor.is_complete());
        assert_eq!(executor.current_step(), 10);
    }

    #[test]
    fn test_async_zero_profile() {
        let profile = MotionProfile::zero();
        let executor = AsyncMotionExecutor::new(profile);

        assert!(executor.is_complete());
        assert_eq!(executor.steps_remaining(), 0);
    }

    #[test]
    fn test_async_reset() {
        let profile = MotionProfile::symmetric_trapezoidal(10, 1000.0, 2000.0);
        let mut executor = AsyncMotionExecutor::new(profile);

        // Advance a few steps
        executor.advance();
        executor.advance();
        assert_eq!(executor.current_step(), 2);

        // Reset
        executor.reset();
        assert_eq!(executor.current_step(), 0);
        assert!(!executor.is_complete());
    }
}
