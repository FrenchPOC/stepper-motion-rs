//! Motion profile calculation.
//!
//! Provides asymmetric trapezoidal motion profiles with independent
//! acceleration and deceleration rates.

use libm::sqrtf;

/// Direction of motor motion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Clockwise (positive step count).
    Clockwise,
    /// Counter-clockwise (negative step count).
    CounterClockwise,
}

impl Direction {
    /// Get direction from signed step count.
    #[inline]
    pub fn from_steps(steps: i64) -> Self {
        if steps >= 0 {
            Direction::Clockwise
        } else {
            Direction::CounterClockwise
        }
    }

    /// Get the sign multiplier.
    #[inline]
    pub fn sign(self) -> i64 {
        match self {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        }
    }
}

/// Current phase of motion execution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionPhase {
    /// Accelerating from rest toward cruise velocity.
    Accelerating,
    /// Moving at constant cruise velocity.
    Cruising,
    /// Decelerating from cruise velocity to rest.
    Decelerating,
    /// Motion complete.
    Complete,
}

/// Computed motion profile for a move (asymmetric trapezoidal).
#[derive(Debug, Clone)]
pub struct MotionProfile {
    /// Total steps to move (absolute value).
    pub total_steps: u32,

    /// Direction of motion.
    pub direction: Direction,

    /// Steps in acceleration phase.
    pub accel_steps: u32,

    /// Steps in cruise phase (constant velocity).
    pub cruise_steps: u32,

    /// Steps in deceleration phase.
    pub decel_steps: u32,

    /// Initial step interval (nanoseconds) - at start of acceleration.
    pub initial_interval_ns: u32,

    /// Cruise step interval (nanoseconds) - at max velocity.
    pub cruise_interval_ns: u32,

    /// Acceleration rate in steps/sec².
    pub accel_rate: f32,

    /// Deceleration rate in steps/sec².
    pub decel_rate: f32,
}

impl MotionProfile {
    /// Create an asymmetric trapezoidal motion profile.
    ///
    /// # Arguments
    ///
    /// * `total_steps` - Signed step count (positive = CW, negative = CCW)
    /// * `max_velocity` - Maximum velocity in steps/sec
    /// * `acceleration` - Acceleration rate in steps/sec²
    /// * `deceleration` - Deceleration rate in steps/sec²
    pub fn asymmetric_trapezoidal(
        total_steps: i64,
        max_velocity: f32,
        acceleration: f32,
        deceleration: f32,
    ) -> Self {
        let direction = Direction::from_steps(total_steps);
        let steps = total_steps.unsigned_abs() as u32;

        if steps == 0 || max_velocity <= 0.0 || acceleration <= 0.0 || deceleration <= 0.0 {
            return Self::zero();
        }

        // Calculate phase lengths for asymmetric profile
        // Time to reach max velocity: t = v_max / a
        // Distance during acceleration: d = 0.5 * a * t²
        let t_accel = max_velocity / acceleration;
        let t_decel = max_velocity / deceleration;

        let accel_distance = 0.5 * acceleration * t_accel * t_accel;
        let decel_distance = 0.5 * deceleration * t_decel * t_decel;

        let (accel_steps, cruise_steps, decel_steps) =
            if accel_distance + decel_distance >= steps as f32 {
                // Triangle profile: can't reach max velocity
                // Scale down proportionally based on acceleration rates
                let ratio = acceleration / (acceleration + deceleration);
                let accel_steps = (steps as f32 * ratio) as u32;
                let decel_steps = steps.saturating_sub(accel_steps);
                (accel_steps, 0u32, decel_steps)
            } else {
                // Full trapezoidal profile
                let accel_steps = accel_distance as u32;
                let decel_steps = decel_distance as u32;
                let cruise_steps = steps.saturating_sub(accel_steps + decel_steps);
                (accel_steps, cruise_steps, decel_steps)
            };

        // Calculate step intervals
        // Initial interval is very long (starting from rest)
        // We use a practical minimum initial velocity
        let initial_velocity = sqrtf(2.0 * acceleration);
        let initial_interval_ns = (1_000_000_000.0 / initial_velocity) as u32;
        let cruise_interval_ns = (1_000_000_000.0 / max_velocity) as u32;

        Self {
            total_steps: steps,
            direction,
            accel_steps,
            cruise_steps,
            decel_steps,
            initial_interval_ns,
            cruise_interval_ns,
            accel_rate: acceleration,
            decel_rate: deceleration,
        }
    }

    /// Create a symmetric trapezoidal profile (same accel and decel).
    pub fn symmetric_trapezoidal(
        total_steps: i64,
        max_velocity: f32,
        acceleration: f32,
    ) -> Self {
        Self::asymmetric_trapezoidal(total_steps, max_velocity, acceleration, acceleration)
    }

    /// Create a zero-length profile (no motion).
    pub fn zero() -> Self {
        Self {
            total_steps: 0,
            direction: Direction::Clockwise,
            accel_steps: 0,
            cruise_steps: 0,
            decel_steps: 0,
            initial_interval_ns: u32::MAX,
            cruise_interval_ns: u32::MAX,
            accel_rate: 0.0,
            decel_rate: 0.0,
        }
    }

    /// Check if this is a zero-length profile.
    #[inline]
    pub fn is_zero(&self) -> bool {
        self.total_steps == 0
    }

    /// Get the phase at a given step number.
    pub fn phase_at(&self, step: u32) -> MotionPhase {
        if step >= self.total_steps {
            MotionPhase::Complete
        } else if step < self.accel_steps {
            MotionPhase::Accelerating
        } else if step < self.accel_steps + self.cruise_steps {
            MotionPhase::Cruising
        } else {
            MotionPhase::Decelerating
        }
    }

    /// Calculate step interval for a given step number.
    ///
    /// Uses the step timing formula for trapezoidal acceleration.
    pub fn interval_at(&self, step: u32) -> u32 {
        let phase = self.phase_at(step);

        match phase {
            MotionPhase::Complete => u32::MAX,
            MotionPhase::Cruising => self.cruise_interval_ns,
            MotionPhase::Accelerating => {
                // During acceleration: interval decreases
                // t_n = t_0 * sqrt(n / (n + 1)) approximately
                // We use a simplified linear interpolation for now
                let progress = step as f32 / self.accel_steps.max(1) as f32;
                let interval = self.initial_interval_ns as f32
                    - (self.initial_interval_ns as f32 - self.cruise_interval_ns as f32) * progress;
                interval as u32
            }
            MotionPhase::Decelerating => {
                // During deceleration: interval increases
                let decel_step = step - self.accel_steps - self.cruise_steps;
                let progress = decel_step as f32 / self.decel_steps.max(1) as f32;
                let interval = self.cruise_interval_ns as f32
                    + (self.initial_interval_ns as f32 - self.cruise_interval_ns as f32) * progress;
                interval as u32
            }
        }
    }

    /// Estimate total duration of the motion profile in seconds.
    ///
    /// This is an approximation based on the trapezoidal profile phases.
    pub fn estimated_duration_secs(&self) -> f32 {
        if self.total_steps == 0 {
            return 0.0;
        }

        let cruise_velocity = 1_000_000_000.0 / self.cruise_interval_ns as f32;

        // Time for each phase
        // Acceleration: v = a*t, so t = v/a
        let accel_time = if self.accel_rate > 0.0 {
            cruise_velocity / self.accel_rate
        } else {
            0.0
        };

        // Cruise: t = distance / velocity
        let cruise_time = self.cruise_steps as f32 / cruise_velocity;

        // Deceleration: t = v/d
        let decel_time = if self.decel_rate > 0.0 {
            cruise_velocity / self.decel_rate
        } else {
            0.0
        };

        accel_time + cruise_time + decel_time
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_symmetric_profile() {
        let profile = MotionProfile::symmetric_trapezoidal(
            1000,   // steps
            1000.0, // steps/sec
            2000.0, // steps/sec²
        );

        assert_eq!(profile.total_steps, 1000);
        assert_eq!(profile.direction, Direction::Clockwise);
        assert!(profile.accel_steps > 0);
        assert!(profile.cruise_steps > 0);
        assert_eq!(profile.accel_steps, profile.decel_steps);
    }

    #[test]
    fn test_asymmetric_profile() {
        let profile = MotionProfile::asymmetric_trapezoidal(
            1000,   // steps
            1000.0, // steps/sec
            2000.0, // accel steps/sec²
            1000.0, // decel steps/sec² (slower)
        );

        assert!(profile.decel_steps > profile.accel_steps);
    }

    #[test]
    fn test_triangle_profile() {
        // Very short move that can't reach max velocity
        let profile = MotionProfile::symmetric_trapezoidal(
            100,     // only 100 steps
            10000.0, // very high max velocity
            1000.0,  // moderate acceleration
        );

        // Should be a triangle (no cruise phase)
        assert_eq!(profile.cruise_steps, 0);
    }

    #[test]
    fn test_direction() {
        let cw = MotionProfile::symmetric_trapezoidal(100, 1000.0, 2000.0);
        let ccw = MotionProfile::symmetric_trapezoidal(-100, 1000.0, 2000.0);

        assert_eq!(cw.direction, Direction::Clockwise);
        assert_eq!(ccw.direction, Direction::CounterClockwise);
        assert_eq!(cw.total_steps, ccw.total_steps);
    }
}
