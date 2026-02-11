//! Homing configuration and strategies.
//!
//! This module provides types for configuring and executing homing sequences
//! for stepper motors with physical switches.

use serde::Deserialize;

use super::units::{Degrees, DegreesPerSec};

/// Homing direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HomingDirection {
    /// Move towards minimum (negative direction).
    #[default]
    ToMin,
    /// Move towards maximum (positive direction).
    ToMax,
}

impl HomingDirection {
    /// Convert to direction sign (-1 for ToMin, +1 for ToMax).
    #[inline]
    pub fn sign(&self) -> i64 {
        match self {
            HomingDirection::ToMin => -1,
            HomingDirection::ToMax => 1,
        }
    }

    /// Get the opposite direction.
    #[inline]
    pub fn opposite(&self) -> Self {
        match self {
            HomingDirection::ToMin => HomingDirection::ToMax,
            HomingDirection::ToMax => HomingDirection::ToMin,
        }
    }
}

/// Homing strategy types.
///
/// Different strategies for finding the home position based on available switches
/// and mechanical configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HomingStrategy {
    /// Move until home switch is triggered, then back off and approach slowly.
    /// This is the most common and accurate strategy.
    #[default]
    HomeSwitch,

    /// Move until home switch is triggered (fast approach only).
    /// Faster but less accurate than HomeSwitch strategy.
    HomeSwitchFast,

    /// Move to limit switch, then offset by specified distance to home position.
    /// Useful when home switch is not at the mechanical limit.
    LimitThenOffset,

    /// Move to minimum limit switch, then offset.
    /// Home position is defined as offset from minimum limit.
    MinLimitOffset,

    /// Move to maximum limit switch, then offset.
    /// Home position is defined as offset from maximum limit.
    MaxLimitOffset,

    /// Stall detection homing (requires motor with stall detection capability).
    /// Motor moves until stall is detected, indicating mechanical limit.
    StallDetect,

    /// Move to hard stop (mechanical limit) at reduced current/speed.
    /// Use with caution - can cause mechanical wear.
    HardStop,
}

/// Complete homing configuration.
#[derive(Debug, Clone, Deserialize)]
pub struct HomingConfig {
    /// Homing strategy to use.
    #[serde(default)]
    pub strategy: HomingStrategy,

    /// Direction to move during initial homing approach.
    #[serde(default)]
    pub direction: HomingDirection,

    /// Fast approach velocity (deg/sec).
    /// Used for initial approach to switch.
    #[serde(rename = "fast_velocity_deg_per_sec")]
    pub fast_velocity: DegreesPerSec,

    /// Slow approach velocity (deg/sec).
    /// Used for accurate final approach after backing off.
    #[serde(rename = "slow_velocity_deg_per_sec")]
    pub slow_velocity: DegreesPerSec,

    /// Back-off distance after triggering switch (degrees).
    /// Motor backs off this amount before slow approach.
    #[serde(default, rename = "backoff_degrees")]
    pub backoff_distance: Degrees,

    /// Offset from switch to define home position (degrees).
    /// Applied after homing sequence completes.
    #[serde(default, rename = "offset_degrees")]
    pub home_offset: Degrees,

    /// Maximum travel distance during homing (degrees).
    /// Homing fails if switch not found within this distance.
    #[serde(rename = "max_travel_degrees")]
    pub max_travel: Degrees,

    /// Position to set after successful homing (degrees).
    /// Typically 0.0 for home position.
    #[serde(default, rename = "home_position_degrees")]
    pub home_position: Degrees,
}

impl Default for HomingConfig {
    fn default() -> Self {
        Self {
            strategy: HomingStrategy::HomeSwitch,
            direction: HomingDirection::ToMin,
            fast_velocity: DegreesPerSec(90.0), // 90 deg/sec
            slow_velocity: DegreesPerSec(10.0), // 10 deg/sec
            backoff_distance: Degrees(5.0),     // 5 degrees
            home_offset: Degrees(0.0),
            max_travel: Degrees(400.0), // Slightly more than one revolution
            home_position: Degrees(0.0),
        }
    }
}

impl HomingConfig {
    /// Create a new homing configuration with default settings.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a homing configuration using home switch strategy.
    pub fn home_switch(direction: HomingDirection) -> Self {
        Self {
            strategy: HomingStrategy::HomeSwitch,
            direction,
            ..Default::default()
        }
    }

    /// Create a homing configuration using limit switch with offset.
    pub fn limit_with_offset(direction: HomingDirection, offset: Degrees) -> Self {
        let strategy = match direction {
            HomingDirection::ToMin => HomingStrategy::MinLimitOffset,
            HomingDirection::ToMax => HomingStrategy::MaxLimitOffset,
        };
        Self {
            strategy,
            direction,
            home_offset: offset,
            ..Default::default()
        }
    }

    /// Set the fast approach velocity.
    pub fn with_fast_velocity(mut self, velocity: DegreesPerSec) -> Self {
        self.fast_velocity = velocity;
        self
    }

    /// Set the slow approach velocity.
    pub fn with_slow_velocity(mut self, velocity: DegreesPerSec) -> Self {
        self.slow_velocity = velocity;
        self
    }

    /// Set the backoff distance.
    pub fn with_backoff(mut self, distance: Degrees) -> Self {
        self.backoff_distance = distance;
        self
    }

    /// Set the maximum travel distance.
    pub fn with_max_travel(mut self, distance: Degrees) -> Self {
        self.max_travel = distance;
        self
    }

    /// Set the home offset.
    pub fn with_offset(mut self, offset: Degrees) -> Self {
        self.home_offset = offset;
        self
    }

    /// Set the home position value.
    pub fn with_home_position(mut self, position: Degrees) -> Self {
        self.home_position = position;
        self
    }
}

/// Homing state machine phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HomingPhase {
    /// Not homing / idle.
    Idle,
    /// Fast approach towards switch.
    FastApproach,
    /// Backing off after triggering switch.
    Backoff,
    /// Slow approach for accurate positioning.
    SlowApproach,
    /// Moving to final home position with offset.
    MovingToOffset,
    /// Homing complete.
    Complete,
    /// Homing failed.
    Failed,
}

impl HomingPhase {
    /// Check if homing is in progress.
    pub fn is_active(&self) -> bool {
        matches!(
            self,
            HomingPhase::FastApproach
                | HomingPhase::Backoff
                | HomingPhase::SlowApproach
                | HomingPhase::MovingToOffset
        )
    }

    /// Check if homing is complete (successfully or failed).
    pub fn is_finished(&self) -> bool {
        matches!(self, HomingPhase::Complete | HomingPhase::Failed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_homing_direction() {
        assert_eq!(HomingDirection::ToMin.sign(), -1);
        assert_eq!(HomingDirection::ToMax.sign(), 1);
        assert_eq!(HomingDirection::ToMin.opposite(), HomingDirection::ToMax);
        assert_eq!(HomingDirection::ToMax.opposite(), HomingDirection::ToMin);
    }

    #[test]
    fn test_homing_config_default() {
        let config = HomingConfig::default();
        assert_eq!(config.strategy, HomingStrategy::HomeSwitch);
        assert_eq!(config.direction, HomingDirection::ToMin);
    }

    #[test]
    fn test_homing_phase() {
        assert!(!HomingPhase::Idle.is_active());
        assert!(HomingPhase::FastApproach.is_active());
        assert!(HomingPhase::SlowApproach.is_active());
        assert!(!HomingPhase::Complete.is_active());
        assert!(HomingPhase::Complete.is_finished());
        assert!(HomingPhase::Failed.is_finished());
    }
}
