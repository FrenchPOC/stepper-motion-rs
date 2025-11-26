//! Unit types for physical quantities.
//!
//! Provides type-safe representations of angles, velocities, accelerations,
//! and motor steps to prevent unit confusion at compile time.

use core::ops::{Add, Mul, Sub};

use serde::Deserialize;

use crate::error::ConfigError;

/// Angular position in degrees.
///
/// Used for configuration and user-facing API. Internally converted to [`Steps`].
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deserialize)]
#[serde(transparent)]
pub struct Degrees(pub f32);

impl Degrees {
    /// Create a new Degrees value.
    #[inline]
    pub const fn new(value: f32) -> Self {
        Self(value)
    }

    /// Get the raw value.
    #[inline]
    pub const fn value(self) -> f32 {
        self.0
    }

    /// Convert to radians.
    #[inline]
    pub fn to_radians(self) -> f32 {
        self.0.to_radians()
    }

    /// Create from radians.
    #[inline]
    pub fn from_radians(radians: f32) -> Self {
        Self(radians.to_degrees())
    }
}

impl Add for Degrees {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Degrees {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

/// Angular velocity in degrees per second.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deserialize)]
#[serde(transparent)]
pub struct DegreesPerSec(pub f32);

impl DegreesPerSec {
    /// Create a new DegreesPerSec value.
    #[inline]
    pub const fn new(value: f32) -> Self {
        Self(value)
    }

    /// Get the raw value.
    #[inline]
    pub const fn value(self) -> f32 {
        self.0
    }
}

impl Mul<f32> for DegreesPerSec {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self(self.0 * rhs)
    }
}

/// Angular acceleration in degrees per second squared.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Deserialize)]
#[serde(transparent)]
pub struct DegreesPerSecSquared(pub f32);

impl DegreesPerSecSquared {
    /// Create a new DegreesPerSecSquared value.
    #[inline]
    pub const fn new(value: f32) -> Self {
        Self(value)
    }

    /// Get the raw value.
    #[inline]
    pub const fn value(self) -> f32 {
        self.0
    }
}

impl Mul<f32> for DegreesPerSecSquared {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self(self.0 * rhs)
    }
}

/// Motor position in steps (absolute from origin).
///
/// Uses i64 for unlimited range in either direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Steps(pub i64);

impl Steps {
    /// Create a new Steps value.
    #[inline]
    pub const fn new(value: i64) -> Self {
        Self(value)
    }

    /// Get the raw value.
    #[inline]
    pub const fn value(self) -> i64 {
        self.0
    }

    /// Get absolute value as u64.
    #[inline]
    pub fn abs(self) -> u64 {
        self.0.unsigned_abs()
    }

    /// Convert to degrees using steps per degree ratio.
    #[inline]
    pub fn to_degrees(self, steps_per_degree: f32) -> Degrees {
        Degrees(self.0 as f32 / steps_per_degree)
    }

    /// Create from degrees using steps per degree ratio.
    #[inline]
    pub fn from_degrees(degrees: Degrees, steps_per_degree: f32) -> Self {
        Self((degrees.0 * steps_per_degree) as i64)
    }
}

impl Add for Steps {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Sub for Steps {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

/// Microstep divisor (1, 2, 4, 8, 16, 32, 64, 128, 256).
///
/// Validated at construction to be a power of 2 within the valid range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Microsteps(u16);

impl Microsteps {
    /// Full step (no microstepping).
    pub const FULL: Self = Self(1);
    /// Half step.
    pub const HALF: Self = Self(2);
    /// Quarter step.
    pub const QUARTER: Self = Self(4);
    /// Eighth step.
    pub const EIGHTH: Self = Self(8);
    /// Sixteenth step.
    pub const SIXTEENTH: Self = Self(16);
    /// Thirty-second step.
    pub const THIRTY_SECOND: Self = Self(32);
    /// Sixty-fourth step.
    pub const SIXTY_FOURTH: Self = Self(64);
    /// 128th step.
    pub const ONE_TWENTY_EIGHTH: Self = Self(128);
    /// 256th step (maximum resolution).
    pub const TWO_FIFTY_SIXTH: Self = Self(256);

    /// Valid microstep values.
    const VALID_VALUES: [u16; 9] = [1, 2, 4, 8, 16, 32, 64, 128, 256];

    /// Create a new Microsteps value with validation.
    ///
    /// # Errors
    ///
    /// Returns `ConfigError::InvalidMicrosteps` if the value is not a valid power of 2.
    pub fn new(value: u16) -> Result<Self, ConfigError> {
        if Self::VALID_VALUES.contains(&value) {
            Ok(Self(value))
        } else {
            Err(ConfigError::InvalidMicrosteps(value))
        }
    }

    /// Get the raw divisor value.
    #[inline]
    pub const fn value(self) -> u16 {
        self.0
    }

    /// Check if a value is valid.
    #[inline]
    pub fn is_valid(value: u16) -> bool {
        Self::VALID_VALUES.contains(&value)
    }
}

impl Default for Microsteps {
    fn default() -> Self {
        Self::FULL
    }
}

impl TryFrom<u16> for Microsteps {
    type Error = ConfigError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

impl<'de> Deserialize<'de> for Microsteps {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        use core::fmt::Write;
        let value = u16::deserialize(deserializer)?;
        Microsteps::new(value).map_err(|e| {
            let mut buf = heapless::String::<128>::new();
            let _ = write!(buf, "{}", e);
            serde::de::Error::custom(buf.as_str())
        })
    }
}

/// Extension trait for creating unit types from primitives.
pub trait UnitExt {
    /// Convert to Degrees.
    fn degrees(self) -> Degrees;
    /// Convert to DegreesPerSec.
    fn degrees_per_sec(self) -> DegreesPerSec;
    /// Convert to DegreesPerSecSquared.
    fn degrees_per_sec_squared(self) -> DegreesPerSecSquared;
}

impl UnitExt for f32 {
    #[inline]
    fn degrees(self) -> Degrees {
        Degrees(self)
    }

    #[inline]
    fn degrees_per_sec(self) -> DegreesPerSec {
        DegreesPerSec(self)
    }

    #[inline]
    fn degrees_per_sec_squared(self) -> DegreesPerSecSquared {
        DegreesPerSecSquared(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_microsteps_valid_values() {
        for &v in &Microsteps::VALID_VALUES {
            assert!(Microsteps::new(v).is_ok());
        }
    }

    #[test]
    fn test_microsteps_invalid_values() {
        assert!(Microsteps::new(0).is_err());
        assert!(Microsteps::new(3).is_err());
        assert!(Microsteps::new(17).is_err());
        assert!(Microsteps::new(512).is_err());
    }

    #[test]
    fn test_degrees_conversion() {
        let d = Degrees::new(180.0);
        assert!((d.to_radians() - core::f32::consts::PI).abs() < 0.0001);
    }

    #[test]
    fn test_steps_to_degrees() {
        let steps = Steps::new(3200);
        let steps_per_degree = 3200.0 / 360.0;
        let degrees = steps.to_degrees(steps_per_degree);
        assert!((degrees.value() - 360.0).abs() < 0.01);
    }
}
