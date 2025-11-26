//! Trajectory configuration from TOML.

use heapless::{String, Vec};
use serde::Deserialize;

use super::mechanical::MechanicalConstraints;
use super::units::{Degrees, DegreesPerSecSquared};

/// A named trajectory from configuration.
#[derive(Debug, Clone, Deserialize)]
pub struct TrajectoryConfig {
    /// Target motor name (must match a motor in config).
    pub motor: String<32>,

    /// Target position in degrees (absolute from origin).
    pub target_degrees: Degrees,

    /// Velocity as percentage of motor's max (1-200).
    #[serde(default = "default_velocity_percent")]
    pub velocity_percent: u8,

    /// Acceleration as percentage of motor's max (1-200).
    /// Used when absolute rates are not specified.
    #[serde(default = "default_acceleration_percent")]
    pub acceleration_percent: u8,

    /// Absolute acceleration rate in degrees/sec² (optional).
    /// Overrides acceleration_percent for the acceleration phase.
    #[serde(default, rename = "acceleration_deg_per_sec2")]
    pub acceleration: Option<DegreesPerSecSquared>,

    /// Absolute deceleration rate in degrees/sec² (optional).
    /// If not set, uses acceleration value (symmetric profile).
    #[serde(default, rename = "deceleration_deg_per_sec2")]
    pub deceleration: Option<DegreesPerSecSquared>,

    /// Optional dwell time at target (milliseconds).
    #[serde(default)]
    pub dwell_ms: Option<u32>,
}

fn default_velocity_percent() -> u8 {
    100
}

fn default_acceleration_percent() -> u8 {
    100
}

impl TrajectoryConfig {
    /// Get effective acceleration rate for this trajectory.
    pub fn effective_acceleration(&self, constraints: &MechanicalConstraints) -> f32 {
        self.acceleration.map(|a| a.0).unwrap_or_else(|| {
            constraints.max_acceleration.0 * (self.acceleration_percent as f32 / 100.0)
        })
    }

    /// Get effective deceleration rate for this trajectory.
    /// Falls back to acceleration if not specified (symmetric profile).
    pub fn effective_deceleration(&self, constraints: &MechanicalConstraints) -> f32 {
        self.deceleration
            .map(|d| d.0)
            .or_else(|| self.acceleration.map(|a| a.0))
            .unwrap_or_else(|| {
                constraints.max_acceleration.0 * (self.acceleration_percent as f32 / 100.0)
            })
    }

    /// Get effective velocity for this trajectory.
    pub fn effective_velocity(&self, constraints: &MechanicalConstraints) -> f32 {
        constraints.max_velocity.0 * (self.velocity_percent as f32 / 100.0)
    }

    /// Check if this trajectory uses asymmetric acceleration.
    pub fn is_asymmetric(&self) -> bool {
        self.deceleration.is_some()
            && self.acceleration.is_some()
            && self.acceleration != self.deceleration
    }
}

/// Trajectory with multiple waypoints.
#[derive(Debug, Clone, Deserialize)]
pub struct WaypointTrajectory {
    /// Target motor name.
    pub motor: String<32>,

    /// Ordered list of waypoint positions in degrees (max 32).
    pub waypoints: Vec<Degrees, 32>,

    /// Dwell time at each waypoint (milliseconds).
    #[serde(default)]
    pub dwell_ms: u32,

    /// Velocity percent for all moves.
    #[serde(default = "default_velocity_percent")]
    pub velocity_percent: u8,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::units::{DegreesPerSec, Microsteps};
    use crate::config::MotorConfig;

    fn make_test_constraints() -> MechanicalConstraints {
        let config = MotorConfig {
            name: String::try_from("test").unwrap(),
            steps_per_revolution: 200,
            microsteps: Microsteps::SIXTEENTH,
            gear_ratio: 1.0,
            max_velocity: DegreesPerSec(360.0),
            max_acceleration: DegreesPerSecSquared(720.0),
            invert_direction: false,
            limits: None,
            backlash_compensation: None,
        };
        MechanicalConstraints::from_config(&config)
    }

    #[test]
    fn test_symmetric_profile() {
        let traj = TrajectoryConfig {
            motor: String::try_from("test").unwrap(),
            target_degrees: Degrees(90.0),
            velocity_percent: 100,
            acceleration_percent: 50,
            acceleration: None,
            deceleration: None,
            dwell_ms: None,
        };

        let constraints = make_test_constraints();
        let accel = traj.effective_acceleration(&constraints);
        let decel = traj.effective_deceleration(&constraints);

        assert!((accel - 360.0).abs() < 0.1); // 720 * 50% = 360
        assert!((decel - 360.0).abs() < 0.1);
        assert!(!traj.is_asymmetric());
    }

    #[test]
    fn test_asymmetric_profile() {
        let traj = TrajectoryConfig {
            motor: String::try_from("test").unwrap(),
            target_degrees: Degrees(90.0),
            velocity_percent: 100,
            acceleration_percent: 100,
            acceleration: Some(DegreesPerSecSquared(500.0)),
            deceleration: Some(DegreesPerSecSquared(200.0)),
            dwell_ms: None,
        };

        let constraints = make_test_constraints();
        let accel = traj.effective_acceleration(&constraints);
        let decel = traj.effective_deceleration(&constraints);

        assert!((accel - 500.0).abs() < 0.1);
        assert!((decel - 200.0).abs() < 0.1);
        assert!(traj.is_asymmetric());
    }
}
