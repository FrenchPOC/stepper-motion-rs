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

    /// Get a trajectory by name, returning an error with available names if not found.
    ///
    /// # Errors
    ///
    /// Returns `TrajectoryError::NotFoundWithNames` if the trajectory doesn't exist,
    /// including a list of available trajectory names for debugging.
    pub fn get_or_error(&self, name: &str) -> Result<&TrajectoryConfig> {
        self.get(name).ok_or_else(|| {
            // Build list of available names for the error message
            let mut available: heapless::String<256> = heapless::String::new();
            let mut first = true;
            for traj_name in self.names() {
                if !first {
                    let _ = available.push_str(", ");
                }
                let _ = available.push_str(traj_name);
                first = false;
            }
            
            let mut msg: heapless::String<64> = heapless::String::new();
            let _ = msg.push_str("'");
            let _ = msg.push_str(name);
            let _ = msg.push_str("' not found. Available: ");
            let _ = msg.push_str(&available);
            
            Error::Trajectory(TrajectoryError::InvalidName(msg))
        })
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

    /// Load trajectories from a SystemConfig.
    pub fn from_config(config: &crate::config::SystemConfig) -> Self {
        let mut registry = Self::new();
        for (name, trajectory) in &config.trajectories {
            let _ = registry.register(name.as_str(), trajectory.clone());
        }
        registry
    }
}
