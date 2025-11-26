//! Mechanical constraints derived from motor configuration.

use super::limits::StepLimits;
use super::motor::MotorConfig;
use super::units::{DegreesPerSec, DegreesPerSecSquared};

/// Derived mechanical parameters computed from motor configuration.
///
/// These are computed once at initialization and used for all motion planning.
#[derive(Debug, Clone)]
pub struct MechanicalConstraints {
    /// Total steps per output revolution (steps × microsteps × gear_ratio).
    pub steps_per_revolution: u32,

    /// Steps per degree of output rotation.
    pub steps_per_degree: f32,

    /// Maximum velocity in steps per second.
    pub max_velocity_steps_per_sec: f32,

    /// Maximum acceleration in steps per second squared.
    pub max_acceleration_steps_per_sec2: f32,

    /// Minimum step interval in nanoseconds (at max velocity).
    pub min_step_interval_ns: u32,

    /// Soft limits in steps (if configured).
    pub limits: Option<StepLimits>,

    /// Maximum velocity in degrees per second.
    pub max_velocity: DegreesPerSec,

    /// Maximum acceleration in degrees per second squared.
    pub max_acceleration: DegreesPerSecSquared,
}

impl MechanicalConstraints {
    /// Compute mechanical constraints from motor configuration.
    pub fn from_config(config: &MotorConfig) -> Self {
        // Total steps per output shaft revolution
        let steps_per_revolution = (config.steps_per_revolution as f32
            * config.microsteps.value() as f32
            * config.gear_ratio) as u32;

        // Steps per degree
        let steps_per_degree = steps_per_revolution as f32 / 360.0;

        // Convert velocity from deg/sec to steps/sec
        let max_velocity_steps_per_sec = config.max_velocity.0 * steps_per_degree;

        // Convert acceleration from deg/sec² to steps/sec²
        let max_acceleration_steps_per_sec2 = config.max_acceleration.0 * steps_per_degree;

        // Minimum step interval at max velocity (nanoseconds)
        let min_step_interval_ns = if max_velocity_steps_per_sec > 0.0 {
            (1_000_000_000.0 / max_velocity_steps_per_sec) as u32
        } else {
            u32::MAX
        };

        // Convert soft limits to step limits
        let limits = config
            .limits
            .as_ref()
            .map(|l| StepLimits::from_soft_limits(l, steps_per_degree));

        Self {
            steps_per_revolution,
            steps_per_degree,
            max_velocity_steps_per_sec,
            max_acceleration_steps_per_sec2,
            min_step_interval_ns,
            limits,
            max_velocity: config.max_velocity,
            max_acceleration: config.max_acceleration,
        }
    }

    /// Convert degrees to steps.
    #[inline]
    pub fn degrees_to_steps(&self, degrees: f32) -> i64 {
        (degrees * self.steps_per_degree) as i64
    }

    /// Convert steps to degrees.
    #[inline]
    pub fn steps_to_degrees(&self, steps: i64) -> f32 {
        steps as f32 / self.steps_per_degree
    }

    /// Convert deg/sec to steps/sec.
    #[inline]
    pub fn velocity_to_steps(&self, deg_per_sec: f32) -> f32 {
        deg_per_sec * self.steps_per_degree
    }

    /// Convert deg/sec² to steps/sec².
    #[inline]
    pub fn acceleration_to_steps(&self, deg_per_sec2: f32) -> f32 {
        deg_per_sec2 * self.steps_per_degree
    }

    /// Calculate step interval for a given velocity in steps/sec.
    #[inline]
    pub fn velocity_to_interval_ns(&self, velocity_steps_per_sec: f32) -> u32 {
        if velocity_steps_per_sec > 0.0 {
            (1_000_000_000.0 / velocity_steps_per_sec) as u32
        } else {
            u32::MAX
        }
    }

    /// Check if a position is within soft limits.
    pub fn check_limits(&self, steps: i64) -> Option<i64> {
        match &self.limits {
            Some(limits) => limits.apply(steps),
            None => Some(steps), // No limits = always valid
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::units::Microsteps;

    fn make_test_config() -> MotorConfig {
        MotorConfig {
            name: heapless::String::try_from("test").unwrap(),
            steps_per_revolution: 200,
            microsteps: Microsteps::SIXTEENTH,
            gear_ratio: 1.0,
            max_velocity: DegreesPerSec(360.0),
            max_acceleration: DegreesPerSecSquared(720.0),
            invert_direction: false,
            limits: None,
            backlash_compensation: None,
        }
    }

    #[test]
    fn test_steps_per_revolution() {
        let config = make_test_config();
        let constraints = MechanicalConstraints::from_config(&config);

        // 200 * 16 * 1.0 = 3200
        assert_eq!(constraints.steps_per_revolution, 3200);
    }

    #[test]
    fn test_steps_per_degree() {
        let config = make_test_config();
        let constraints = MechanicalConstraints::from_config(&config);

        // 3200 / 360 = 8.889
        assert!((constraints.steps_per_degree - 8.889).abs() < 0.01);
    }

    #[test]
    fn test_velocity_conversion() {
        let config = make_test_config();
        let constraints = MechanicalConstraints::from_config(&config);

        // 360 deg/sec * 8.889 steps/deg = 3200 steps/sec
        assert!((constraints.max_velocity_steps_per_sec - 3200.0).abs() < 1.0);
    }
}
