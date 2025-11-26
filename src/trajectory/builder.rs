//! Trajectory builder for programmatic trajectory creation.

use heapless::String;

use crate::config::{TrajectoryConfig, WaypointTrajectory};
use crate::config::units::{Degrees, DegreesPerSecSquared};
use crate::error::{Error, Result, TrajectoryError};

/// Builder for creating single-target trajectories.
#[derive(Debug, Clone)]
pub struct TrajectoryBuilder {
    motor: Option<String<32>>,
    target_degrees: Option<Degrees>,
    velocity_percent: u8,
    acceleration_percent: u8,
    acceleration: Option<DegreesPerSecSquared>,
    deceleration: Option<DegreesPerSecSquared>,
    dwell_ms: Option<u32>,
}

impl Default for TrajectoryBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl TrajectoryBuilder {
    /// Create a new trajectory builder.
    pub fn new() -> Self {
        Self {
            motor: None,
            target_degrees: None,
            velocity_percent: 100,
            acceleration_percent: 100,
            acceleration: None,
            deceleration: None,
            dwell_ms: None,
        }
    }

    /// Set the target motor name.
    pub fn motor(mut self, name: &str) -> Self {
        self.motor = String::try_from(name).ok();
        self
    }

    /// Set the target position in degrees.
    pub fn target(mut self, position: Degrees) -> Self {
        self.target_degrees = Some(position);
        self
    }

    /// Set velocity as percentage of motor's max (1-200).
    pub fn velocity_percent(mut self, percent: u8) -> Self {
        self.velocity_percent = percent.clamp(1, 200);
        self
    }

    /// Set acceleration as percentage of motor's max (1-200).
    pub fn acceleration_percent(mut self, percent: u8) -> Self {
        self.acceleration_percent = percent.clamp(1, 200);
        self
    }

    /// Set absolute acceleration rate in degrees/sec².
    pub fn acceleration(mut self, accel: DegreesPerSecSquared) -> Self {
        self.acceleration = Some(accel);
        self
    }

    /// Set absolute deceleration rate in degrees/sec².
    pub fn deceleration(mut self, decel: DegreesPerSecSquared) -> Self {
        self.deceleration = Some(decel);
        self
    }

    /// Set asymmetric acceleration/deceleration rates.
    pub fn asymmetric(mut self, accel: DegreesPerSecSquared, decel: DegreesPerSecSquared) -> Self {
        self.acceleration = Some(accel);
        self.deceleration = Some(decel);
        self
    }

    /// Set dwell time at target in milliseconds.
    pub fn dwell(mut self, dwell_ms: u32) -> Self {
        self.dwell_ms = Some(dwell_ms);
        self
    }

    /// Build the trajectory configuration.
    ///
    /// # Errors
    ///
    /// Returns an error if required fields are missing.
    pub fn build(self) -> Result<TrajectoryConfig> {
        let motor = self.motor.ok_or_else(|| {
            Error::Trajectory(TrajectoryError::InvalidName(
                String::try_from("motor not specified").unwrap(),
            ))
        })?;

        let target_degrees = self.target_degrees.ok_or_else(|| {
            Error::Trajectory(TrajectoryError::InvalidName(
                String::try_from("target not specified").unwrap(),
            ))
        })?;

        Ok(TrajectoryConfig {
            motor,
            target_degrees,
            velocity_percent: self.velocity_percent,
            acceleration_percent: self.acceleration_percent,
            acceleration: self.acceleration,
            deceleration: self.deceleration,
            dwell_ms: self.dwell_ms,
        })
    }
}

/// Maximum number of waypoints in a trajectory.
pub const MAX_WAYPOINTS: usize = 32;

/// Builder for creating waypoint trajectories.
#[derive(Debug, Clone)]
pub struct WaypointTrajectoryBuilder {
    motor: Option<String<32>>,
    waypoints: heapless::Vec<Degrees, MAX_WAYPOINTS>,
    velocity_percent: u8,
    dwell_ms: u32,
}

impl Default for WaypointTrajectoryBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl WaypointTrajectoryBuilder {
    /// Create a new waypoint trajectory builder.
    pub fn new() -> Self {
        Self {
            motor: None,
            waypoints: heapless::Vec::new(),
            velocity_percent: 100,
            dwell_ms: 0,
        }
    }

    /// Set the target motor name.
    pub fn motor(mut self, name: &str) -> Self {
        self.motor = String::try_from(name).ok();
        self
    }

    /// Add a waypoint at the given position.
    pub fn waypoint(mut self, position: Degrees) -> Self {
        let _ = self.waypoints.push(position);
        self
    }

    /// Add multiple waypoints.
    pub fn waypoints(mut self, positions: &[Degrees]) -> Self {
        for pos in positions {
            let _ = self.waypoints.push(*pos);
        }
        self
    }

    /// Set velocity as percentage of motor's max (1-200).
    pub fn velocity_percent(mut self, percent: u8) -> Self {
        self.velocity_percent = percent.clamp(1, 200);
        self
    }

    /// Set dwell time at each waypoint in milliseconds.
    pub fn dwell(mut self, dwell_ms: u32) -> Self {
        self.dwell_ms = dwell_ms;
        self
    }

    /// Build the waypoint trajectory configuration.
    ///
    /// # Errors
    ///
    /// Returns an error if required fields are missing or waypoints are empty.
    pub fn build(self) -> Result<WaypointTrajectory> {
        let motor = self.motor.ok_or_else(|| {
            Error::Trajectory(TrajectoryError::InvalidName(
                String::try_from("motor not specified").unwrap(),
            ))
        })?;

        if self.waypoints.is_empty() {
            return Err(Error::Trajectory(TrajectoryError::Empty));
        }

        Ok(WaypointTrajectory {
            motor,
            waypoints: self.waypoints,
            velocity_percent: self.velocity_percent,
            dwell_ms: self.dwell_ms,
        })
    }
}
