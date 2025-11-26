//! Soft limit configuration and types.

use serde::Deserialize;

use super::units::Degrees;

/// Policy for handling limit violations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LimitPolicy {
    /// Reject moves that would exceed limits.
    #[default]
    Reject,
    /// Clamp target to nearest limit.
    Clamp,
}

/// Soft limits in degrees (from configuration).
#[derive(Debug, Clone, Deserialize)]
pub struct SoftLimits {
    /// Minimum allowed position in degrees.
    #[serde(rename = "min_degrees")]
    pub min: Degrees,

    /// Maximum allowed position in degrees.
    #[serde(rename = "max_degrees")]
    pub max: Degrees,

    /// What to do when limit is exceeded.
    #[serde(default)]
    pub policy: LimitPolicy,
}

impl SoftLimits {
    /// Create new soft limits.
    pub fn new(min: Degrees, max: Degrees, policy: LimitPolicy) -> Self {
        Self { min, max, policy }
    }

    /// Check if limits are valid (min < max).
    pub fn is_valid(&self) -> bool {
        self.min.0 < self.max.0
    }

    /// Check if a position is within limits.
    pub fn contains(&self, position: Degrees) -> bool {
        position.0 >= self.min.0 && position.0 <= self.max.0
    }

    /// Apply limit policy to a target position.
    ///
    /// Returns `Some(position)` if valid or clamped, `None` if rejected.
    pub fn apply(&self, target: Degrees) -> Option<Degrees> {
        if self.contains(target) {
            Some(target)
        } else {
            match self.policy {
                LimitPolicy::Reject => None,
                LimitPolicy::Clamp => {
                    if target.0 < self.min.0 {
                        Some(self.min)
                    } else {
                        Some(self.max)
                    }
                }
            }
        }
    }
}

/// Soft limits converted to steps (for runtime use).
#[derive(Debug, Clone)]
pub struct StepLimits {
    /// Minimum position in steps.
    pub min_steps: i64,
    /// Maximum position in steps.
    pub max_steps: i64,
    /// Limit policy.
    pub policy: LimitPolicy,
}

impl StepLimits {
    /// Create step limits from soft limits and steps per degree.
    pub fn from_soft_limits(soft: &SoftLimits, steps_per_degree: f32) -> Self {
        Self {
            min_steps: (soft.min.0 * steps_per_degree) as i64,
            max_steps: (soft.max.0 * steps_per_degree) as i64,
            policy: soft.policy,
        }
    }

    /// Check if a position is within limits.
    pub fn contains(&self, steps: i64) -> bool {
        steps >= self.min_steps && steps <= self.max_steps
    }

    /// Apply limit policy to a target position.
    ///
    /// Returns `Some(steps)` if valid or clamped, `None` if rejected.
    pub fn apply(&self, target: i64) -> Option<i64> {
        if self.contains(target) {
            Some(target)
        } else {
            match self.policy {
                LimitPolicy::Reject => None,
                LimitPolicy::Clamp => {
                    if target < self.min_steps {
                        Some(self.min_steps)
                    } else {
                        Some(self.max_steps)
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_soft_limits_reject() {
        let limits = SoftLimits::new(Degrees(-180.0), Degrees(180.0), LimitPolicy::Reject);

        assert!(limits.apply(Degrees(0.0)).is_some());
        assert!(limits.apply(Degrees(180.0)).is_some());
        assert!(limits.apply(Degrees(-180.0)).is_some());
        assert!(limits.apply(Degrees(181.0)).is_none());
        assert!(limits.apply(Degrees(-181.0)).is_none());
    }

    #[test]
    fn test_soft_limits_clamp() {
        let limits = SoftLimits::new(Degrees(-180.0), Degrees(180.0), LimitPolicy::Clamp);

        assert_eq!(limits.apply(Degrees(0.0)).unwrap().0, 0.0);
        assert_eq!(limits.apply(Degrees(360.0)).unwrap().0, 180.0);
        assert_eq!(limits.apply(Degrees(-360.0)).unwrap().0, -180.0);
    }
}
