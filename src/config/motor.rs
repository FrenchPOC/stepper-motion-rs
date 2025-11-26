//! Motor configuration from TOML.

use heapless::String;
use serde::Deserialize;

use super::limits::SoftLimits;
use super::units::{Degrees, DegreesPerSec, DegreesPerSecSquared, Microsteps};

/// Complete motor configuration from TOML.
#[derive(Debug, Clone, Deserialize)]
pub struct MotorConfig {
    /// Human-readable name (max 32 chars).
    pub name: String<32>,

    /// Base steps per revolution (typically 200 for 1.8° motors).
    pub steps_per_revolution: u16,

    /// Microstep setting (1, 2, 4, 8, 16, 32, etc.).
    pub microsteps: Microsteps,

    /// Gear ratio (output:input, e.g., 5.0 means 5:1 reduction).
    #[serde(default = "default_gear_ratio")]
    pub gear_ratio: f32,

    /// Maximum angular velocity in degrees per second.
    #[serde(rename = "max_velocity_deg_per_sec")]
    pub max_velocity: DegreesPerSec,

    /// Maximum angular acceleration in degrees per second squared.
    #[serde(rename = "max_acceleration_deg_per_sec2")]
    pub max_acceleration: DegreesPerSecSquared,

    /// Invert direction pin logic.
    #[serde(default)]
    pub invert_direction: bool,

    /// Optional soft limits.
    #[serde(default)]
    pub limits: Option<SoftLimits>,

    /// Optional backlash compensation in degrees.
    #[serde(default, rename = "backlash_compensation_deg")]
    pub backlash_compensation: Option<Degrees>,
}

fn default_gear_ratio() -> f32 {
    1.0
}

impl MotorConfig {
    /// Calculate total steps per output shaft revolution.
    pub fn total_steps_per_revolution(&self) -> u32 {
        (self.steps_per_revolution as f32 * self.microsteps.value() as f32 * self.gear_ratio)
            as u32
    }

    /// Calculate steps per degree of output rotation.
    pub fn steps_per_degree(&self) -> f32 {
        self.total_steps_per_revolution() as f32 / 360.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_total_steps() {
        let config = MotorConfig {
            name: String::try_from("test").unwrap(),
            steps_per_revolution: 200,
            microsteps: Microsteps::SIXTEENTH,
            gear_ratio: 2.0,
            max_velocity: DegreesPerSec(360.0),
            max_acceleration: DegreesPerSecSquared(720.0),
            invert_direction: false,
            limits: None,
            backlash_compensation: None,
        };

        // 200 * 16 * 2.0 = 6400
        assert_eq!(config.total_steps_per_revolution(), 6400);
    }
}
