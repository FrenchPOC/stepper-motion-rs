//! Trajectory registry for named trajectory lookup.

use heapless::{FnvIndexMap, String};

use crate::config::TrajectoryConfig;
use crate::error::{Error, Result, TrajectoryError};

/// Maximum number of trajectories in the registry.
pub const MAX_TRAJECTORIES: usize = 32;

/// Registry for named trajectories.
#[derive(Debug)]
pub struct TrajectoryRegistry {
    trajectories: FnvIndexMap<String<32>, TrajectoryConfig, MAX_TRAJECTORIES>,
}

impl Default for TrajectoryRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl TrajectoryRegistry {
    /// Create a new empty registry.
    pub fn new() -> Self {
        Self {
            trajectories: FnvIndexMap::new(),
        }
    }

    /// Register a trajectory with a name.
    ///
    /// # Errors
    ///
    /// Returns an error if the registry is full.
    pub fn register(&mut self, name: &str, trajectory: TrajectoryConfig) -> Result<()> {
        let name_str = String::try_from(name).map_err(|_| {
            Error::Trajectory(TrajectoryError::InvalidName(
                String::try_from("name too long").unwrap(),
            ))
        })?;

        self.trajectories
            .insert(name_str, trajectory)
            .map_err(|_| {
                Error::Trajectory(TrajectoryError::InvalidName(
                    String::try_from("registry full").unwrap(),
                ))
            })?;

        Ok(())
    }

    /// Get a trajectory by name.
    pub fn get(&self, name: &str) -> Option<&TrajectoryConfig> {
        let name_str = String::try_from(name).ok()?;
        self.trajectories.get(&name_str)
    }

    /// Check if a trajectory exists.
    pub fn contains(&self, name: &str) -> bool {
        if let Ok(name_str) = String::try_from(name) {
            self.trajectories.contains_key(&name_str)
        } else {
            false
        }
    }

    /// Remove a trajectory by name.
    pub fn remove(&mut self, name: &str) -> Option<TrajectoryConfig> {
        let name_str = String::try_from(name).ok()?;
        self.trajectories.remove(&name_str)
    }

    /// Get the number of registered trajectories.
    pub fn len(&self) -> usize {
        self.trajectories.len()
    }

    /// Check if the registry is empty.
    pub fn is_empty(&self) -> bool {
        self.trajectories.is_empty()
    }

    /// Get an iterator over trajectory names.
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.trajectories.keys().map(|s| s.as_str())
    }

    /// Get an iterator over trajectories.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &TrajectoryConfig)> {
        self.trajectories
            .iter()
            .map(|(k, v)| (k.as_str(), v))
    }

    /// Clear all trajectories.
    pub fn clear(&mut self) {
        self.trajectories.clear();
    }
}

#[cfg(feature = "std")]
impl TrajectoryRegistry {
    /// Load trajectories from a SystemConfig.
    pub fn from_config(config: &crate::config::SystemConfig) -> Self {
        let mut registry = Self::new();
        for (name, trajectory) in &config.trajectories {
            let _ = registry.register(name.as_str(), trajectory.clone());
        }
        registry
    }
}
