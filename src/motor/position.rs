//! Position tracking for stepper motors.
//!
//! Provides absolute position tracking in steps with unit conversions.

use crate::config::units::{Degrees, Steps};

/// Motor position tracker.
///
/// Maintains absolute position in steps and provides unit conversions.
#[derive(Debug, Clone, Copy, Default)]
pub struct Position {
    /// Current position in steps (from origin)
    steps: Steps,
    /// Steps per degree for conversions
    steps_per_degree: f32,
}

impl Position {
    /// Create a new position tracker.
    #[inline]
    pub fn new(steps_per_degree: f32) -> Self {
        Self {
            steps: Steps::default(),
            steps_per_degree,
        }
    }

    /// Create a position tracker at a specific position.
    #[inline]
    pub fn at(steps: Steps, steps_per_degree: f32) -> Self {
        Self {
            steps,
            steps_per_degree,
        }
    }

    /// Get current position in steps.
    #[inline]
    pub fn steps(&self) -> Steps {
        self.steps
    }

    /// Get current position in degrees.
    #[inline]
    pub fn degrees(&self) -> Degrees {
        self.steps.to_degrees(self.steps_per_degree)
    }

    /// Set position in steps.
    #[inline]
    pub fn set_steps(&mut self, steps: Steps) {
        self.steps = steps;
    }

    /// Set position in degrees.
    #[inline]
    pub fn set_degrees(&mut self, degrees: Degrees) {
        self.steps = Steps::from_degrees(degrees, self.steps_per_degree);
    }

    /// Move by a number of steps.
    #[inline]
    pub fn move_steps(&mut self, delta: i64) {
        self.steps = Steps(self.steps.0 + delta);
    }

    /// Move by an amount in degrees.
    #[inline]
    pub fn move_degrees(&mut self, delta: Degrees) {
        let delta_steps = (delta.0 * self.steps_per_degree) as i64;
        self.move_steps(delta_steps);
    }

    /// Reset position to origin (0 steps).
    #[inline]
    pub fn reset(&mut self) {
        self.steps = Steps::default();
    }

    /// Set current position as the new origin.
    #[inline]
    pub fn set_origin(&mut self) {
        self.steps = Steps::default();
    }

    /// Get steps per degree conversion factor.
    #[inline]
    pub fn steps_per_degree(&self) -> f32 {
        self.steps_per_degree
    }

    /// Calculate steps needed to reach a target position in degrees.
    #[inline]
    pub fn steps_to(&self, target: Degrees) -> i64 {
        let target_steps = Steps::from_degrees(target, self.steps_per_degree);
        target_steps.0 - self.steps.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_position_tracking() {
        // 200 steps/rev * 16 microsteps = 3200 steps/rev
        // 3200 / 360 = 8.889 steps/degree
        let steps_per_degree = 3200.0 / 360.0;
        let mut pos = Position::new(steps_per_degree);

        assert_eq!(pos.steps().value(), 0);

        pos.move_degrees(Degrees(90.0));
        assert!((pos.degrees().value() - 90.0).abs() < 0.1);

        pos.move_degrees(Degrees(90.0));
        assert!((pos.degrees().value() - 180.0).abs() < 0.1);

        pos.move_degrees(Degrees(-180.0));
        assert!(pos.degrees().value().abs() < 0.1);
    }

    #[test]
    fn test_steps_to_target() {
        let steps_per_degree = 10.0;
        let pos = Position::at(Steps(900), steps_per_degree);

        let steps = pos.steps_to(Degrees(180.0));
        assert_eq!(steps, 900); // 1800 - 900 = 900
    }
}
