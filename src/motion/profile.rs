//! Motion profile calculation.
//!
//! Provides asymmetric trapezoidal motion profiles with independent
//! acceleration and deceleration rates.

use libm::{ceilf, sqrt, sqrtf};

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

    /// Get the opposite direction.
    #[inline]
    pub fn opposite(self) -> Self {
        match self {
            Direction::Clockwise => Direction::CounterClockwise,
            Direction::CounterClockwise => Direction::Clockwise,
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

    /// Cruise step interval (nanoseconds), using the reachable peak velocity.
    pub cruise_interval_ns: u32,

    /// Acceleration rate in steps/sec².
    pub accel_rate: f32,

    /// Deceleration rate in steps/sec².
    pub decel_rate: f32,

    // Keep fractional phase boundaries: a single step can cross two phases.
    accel_distance: f64,
    decel_distance: f64,
    peak_velocity: f32,
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

        if steps == 0
            || !max_velocity.is_finite()
            || !acceleration.is_finite()
            || !deceleration.is_finite()
            || max_velocity <= 0.0
            || acceleration <= 0.0
            || deceleration <= 0.0
        {
            return Self::zero();
        }

        // v_peak² = 2*N / (1/a + 1/d). This also handles short moves and
        // asymmetric ramps (the slower ramp must occupy more distance).
        let a = acceleration as f64;
        let d = deceleration as f64;
        let peak_velocity =
            (max_velocity as f64).min(sqrt(2.0 * steps as f64 / (1.0 / a + 1.0 / d)));
        let accel_distance = peak_velocity * peak_velocity / (2.0 * a);
        let decel_distance = peak_velocity * peak_velocity / (2.0 * d);

        let (accel_steps, cruise_steps, decel_steps) = if peak_velocity < max_velocity as f64 {
            let accel_steps = (steps as f64 * d / (a + d)) as u32;
            let decel_steps = steps.saturating_sub(accel_steps);
            (accel_steps, 0u32, decel_steps)
        } else {
            // Full trapezoidal profile
            let accel_steps = accel_distance as u32;
            let decel_steps = decel_distance as u32;
            let cruise_steps = steps.saturating_sub(accel_steps + decel_steps);
            (accel_steps, cruise_steps, decel_steps)
        };

        let mut profile = Self {
            total_steps: steps,
            direction,
            accel_steps,
            cruise_steps,
            decel_steps,
            initial_interval_ns: 0,
            cruise_interval_ns: ceilf((1_000_000_000.0 / peak_velocity) as f32).max(1.0) as u32,
            accel_rate: acceleration,
            decel_rate: deceleration,
            accel_distance,
            decel_distance,
            peak_velocity: peak_velocity as f32,
        };
        profile.initial_interval_ns = profile.interval_at(0);
        profile
    }

    /// Create a symmetric trapezoidal profile (same accel and decel).
    pub fn symmetric_trapezoidal(total_steps: i64, max_velocity: f32, acceleration: f32) -> Self {
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
            accel_distance: 0.0,
            decel_distance: 0.0,
            peak_velocity: 0.0,
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
    /// Time to travel from position `step` to `step + 1`, starting and ending
    /// at rest over the whole move. Integrate each phase crossed by this step.
    /// The rationalized expression 2*distance/(v0+v1) avoids subtracting nearly
    /// equal square roots, which would lose precision on long ramps.
    pub fn interval_at(&self, step: u32) -> u32 {
        if step >= self.total_steps {
            return u32::MAX;
        }

        let start = step as f64;
        let end = start + 1.0;
        let decel_start = (self.total_steps as f64 - self.decel_distance).max(self.accel_distance);
        let mut seconds = 0.0_f32;

        let accel_end = end.min(self.accel_distance);
        if start < accel_end {
            let v0 = sqrtf(2.0 * self.accel_rate * start as f32);
            let v1 = sqrtf(2.0 * self.accel_rate * accel_end as f32);
            seconds += 2.0 * (accel_end - start) as f32 / (v0 + v1);
        }

        let cruise_start = start.max(self.accel_distance);
        let cruise_end = end.min(decel_start);
        if cruise_start < cruise_end {
            seconds += (cruise_end - cruise_start) as f32 / self.peak_velocity;
        }

        let decel_begin = start.max(decel_start);
        if decel_begin < end {
            let remaining = (self.total_steps - step) as f64;
            let v0 = sqrtf(2.0 * self.decel_rate * remaining.min(self.decel_distance) as f32);
            let v1 = sqrtf(2.0 * self.decel_rate * (remaining - 1.0) as f32);
            seconds += 2.0 * (end - decel_begin) as f32 / (v0 + v1);
        }

        ceilf(seconds * 1_000_000_000.0).max(1.0) as u32
    }

    /// Estimate total duration of the motion profile in seconds.
    ///
    /// This is an approximation based on the trapezoidal profile phases.
    pub fn estimated_duration_secs(&self) -> f32 {
        if self.total_steps == 0 {
            return 0.0;
        }

        let cruise_distance =
            (self.total_steps as f64 - self.accel_distance - self.decel_distance).max(0.0);
        self.peak_velocity / self.accel_rate
            + self.peak_velocity / self.decel_rate
            + (cruise_distance / self.peak_velocity as f64) as f32
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

    fn duration_from_intervals(profile: &MotionProfile) -> f64 {
        (0..profile.total_steps)
            .map(|step| profile.interval_at(step) as f64 * 1e-9)
            .sum()
    }

    #[test]
    fn step_timestamps_follow_constant_acceleration() {
        // Same step-domain parameters as Y/Z at full speed.
        let a = 60000.0 * 3200.0 / 360.0;
        let v = 4000.0 * 3200.0 / 360.0;
        let profile = MotionProfile::symmetric_trapezoidal(3200, v, a);
        let mut elapsed = 0.0;
        for step in 0..profile.accel_steps {
            elapsed += profile.interval_at(step) as f64 * 1e-9;
            // x(t) = a*t²/2, within nanosecond interval rounding.
            let expected = sqrt(2.0 * (step + 1) as f64 / a as f64);
            assert!((elapsed - expected).abs() < 2e-6);
        }
        assert!((elapsed - 0.06666).abs() < 2e-5);
        assert!((duration_from_intervals(&profile) - 0.1566667).abs() < 1e-5);
    }

    #[test]
    fn short_move_reduces_peak_velocity_and_duration() {
        let profile = MotionProfile::symmetric_trapezoidal(100, 10000.0, 1000.0);
        let expected_peak = sqrtf(1000.0 * 100.0);
        assert!((profile.peak_velocity - expected_peak).abs() < 0.001);
        assert_eq!(profile.cruise_steps, 0);
        let duration = 2.0 * sqrt(100.0 / 1000.0);
        assert!((duration_from_intervals(&profile) - duration).abs() < 2e-6);
        assert!((profile.estimated_duration_secs() as f64 - duration).abs() < 1e-6);
        for step in 0..100 {
            assert!(profile.interval_at(step) >= profile.cruise_interval_ns);
            assert_eq!(profile.interval_at(step), profile.interval_at(99 - step));
        }
    }

    #[test]
    fn asymmetric_triangle_gives_more_distance_to_the_slower_ramp() {
        let profile = MotionProfile::asymmetric_trapezoidal(90, 10000.0, 2000.0, 1000.0);
        assert_eq!(profile.accel_steps, 30);
        assert_eq!(profile.decel_steps, 60);
        assert_eq!(profile.cruise_steps, 0);
        let peak = sqrt(120000.0);
        let expected = peak / 2000.0 + peak / 1000.0;
        assert!((duration_from_intervals(&profile) - expected).abs() < 2e-6);
        assert!(profile.interval_at(89) > profile.interval_at(0));
    }

    #[test]
    fn fractional_phase_boundaries_and_single_steps_preserve_duration() {
        for (steps, velocity, accel, decel) in [
            (1, 1000.0, 2000.0, 1000.0),
            (2, 1000.0, 2000.0, 1000.0),
            (99, 1000.0, 2000.0, 1000.0),
            (30, 1.0, 2000.0, 1000.0),
            (1000, 101.0, 2000.0, 1000.0),
        ] {
            let profile = MotionProfile::asymmetric_trapezoidal(steps, velocity, accel, decel);
            let expected = profile.estimated_duration_secs() as f64;
            assert!((duration_from_intervals(&profile) - expected).abs() < expected * 2e-6);
            assert_eq!(profile.initial_interval_ns, profile.interval_at(0));
            assert_eq!(profile.interval_at(steps as u32), u32::MAX);
            assert_eq!(
                profile.accel_steps + profile.cruise_steps + profile.decel_steps,
                steps as u32
            );
        }
    }

    #[test]
    fn non_finite_rates_do_not_create_a_profile() {
        for invalid in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(MotionProfile::symmetric_trapezoidal(100, invalid, 1000.0).is_zero());
            assert!(MotionProfile::asymmetric_trapezoidal(100, 100.0, invalid, 1000.0).is_zero());
            assert!(MotionProfile::asymmetric_trapezoidal(100, 100.0, 1000.0, invalid).is_zero());
        }
    }
}
