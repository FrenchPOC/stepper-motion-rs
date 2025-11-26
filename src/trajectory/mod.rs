//! Trajectory module for stepper-motion.
//!
//! Provides named trajectory storage, lookup, and building.

mod builder;
mod registry;

pub use builder::{TrajectoryBuilder, WaypointTrajectoryBuilder, MAX_WAYPOINTS};
pub use registry::{TrajectoryRegistry, MAX_TRAJECTORIES};
