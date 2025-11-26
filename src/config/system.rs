//! System configuration - root configuration structure.

use heapless::{FnvIndexMap, String};
use serde::Deserialize;

use super::motor::MotorConfig;
use super::trajectory::{TrajectoryConfig, WaypointTrajectory};

/// Root configuration structure from TOML.
#[derive(Debug, Clone, Deserialize)]
pub struct SystemConfig {
    /// Named motor configurations.
    pub motors: FnvIndexMap<String<32>, MotorConfig, 8>,

    /// Named trajectory configurations.
    #[serde(default)]
    pub trajectories: FnvIndexMap<String<32>, TrajectoryConfig, 64>,

    /// Named waypoint trajectories (sequences).
    #[serde(default)]
    pub sequences: FnvIndexMap<String<32>, WaypointTrajectory, 16>,
}

impl SystemConfig {
    /// Get a motor configuration by name.
    pub fn motor(&self, name: &str) -> Option<&MotorConfig> {
        self.motors
            .iter()
            .find(|(k, _)| k.as_str() == name)
            .map(|(_, v)| v)
    }

    /// Get a trajectory configuration by name.
    pub fn trajectory(&self, name: &str) -> Option<&TrajectoryConfig> {
        self.trajectories
            .iter()
            .find(|(k, _)| k.as_str() == name)
            .map(|(_, v)| v)
    }

    /// Get a waypoint trajectory by name.
    pub fn sequence(&self, name: &str) -> Option<&WaypointTrajectory> {
        self.sequences
            .iter()
            .find(|(k, _)| k.as_str() == name)
            .map(|(_, v)| v)
    }

    /// List all motor names.
    pub fn motor_names(&self) -> impl Iterator<Item = &str> {
        self.motors.keys().map(|s| s.as_str())
    }

    /// List all trajectory names.
    pub fn trajectory_names(&self) -> impl Iterator<Item = &str> {
        self.trajectories.keys().map(|s| s.as_str())
    }

    /// List all sequence names.
    pub fn sequence_names(&self) -> impl Iterator<Item = &str> {
        self.sequences.keys().map(|s| s.as_str())
    }
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self {
            motors: FnvIndexMap::new(),
            trajectories: FnvIndexMap::new(),
            sequences: FnvIndexMap::new(),
        }
    }
}
