//! Configuration validation.

use crate::error::{ConfigError, Error, Result, TrajectoryError};

use super::SystemConfig;

/// Validate a system configuration.
///
/// Checks:
/// - Motor configurations are valid
/// - Trajectory references existing motors
/// - Velocity/acceleration percentages are in range
/// - Soft limits are valid (min < max)
pub fn validate_config(config: &SystemConfig) -> Result<()> {
    // Validate motors
    for (name, motor) in config.motors.iter() {
        validate_motor(name.as_str(), motor)?;
    }

    // Validate trajectories
    for (name, traj) in config.trajectories.iter() {
        validate_trajectory(name.as_str(), traj, config)?;
    }

    // Validate sequences
    for (name, seq) in config.sequences.iter() {
        validate_sequence(name.as_str(), seq, config)?;
    }

    Ok(())
}

fn validate_motor(_name: &str, config: &super::MotorConfig) -> Result<()> {
    // Gear ratio must be positive
    if config.gear_ratio <= 0.0 {
        return Err(Error::Config(ConfigError::InvalidGearRatio(config.gear_ratio)));
    }

    // Max velocity must be positive
    if config.max_velocity.0 <= 0.0 {
        return Err(Error::Config(ConfigError::InvalidMaxVelocity(
            config.max_velocity.0,
        )));
    }

    // Max acceleration must be positive
    if config.max_acceleration.0 <= 0.0 {
        return Err(Error::Config(ConfigError::InvalidMaxAcceleration(
            config.max_acceleration.0,
        )));
    }

    // Soft limits: min must be < max
    if let Some(ref limits) = config.limits {
        if !limits.is_valid() {
            return Err(Error::Config(ConfigError::InvalidSoftLimits {
                min: limits.min.0,
                max: limits.max.0,
            }));
        }
    }

    Ok(())
}

fn validate_trajectory(
    name: &str,
    traj: &super::TrajectoryConfig,
    config: &SystemConfig,
) -> Result<()> {
    // Motor must exist
    if config.motor(traj.motor.as_str()).is_none() {
        return Err(Error::Trajectory(TrajectoryError::MotorNotFound {
            trajectory: heapless::String::try_from(name).unwrap_or_default(),
            motor: traj.motor.clone(),
        }));
    }

    // Velocity percent must be 1-200
    if traj.velocity_percent == 0 || traj.velocity_percent > 200 {
        return Err(Error::Config(ConfigError::InvalidVelocityPercent(
            traj.velocity_percent,
        )));
    }

    // Acceleration percent must be 1-200
    if traj.acceleration_percent == 0 || traj.acceleration_percent > 200 {
        return Err(Error::Config(ConfigError::InvalidAccelerationPercent(
            traj.acceleration_percent,
        )));
    }

    // Check target against limits if motor has them
    if let Some(motor) = config.motor(traj.motor.as_str()) {
        if let Some(ref limits) = motor.limits {
            if !limits.contains(traj.target_degrees) {
                // Note: This is a warning, not an error, if policy is Clamp
                // For now, we only error on Reject policy
                if limits.policy == super::LimitPolicy::Reject {
                    return Err(Error::Trajectory(TrajectoryError::TargetExceedsLimits {
                        target: traj.target_degrees.0,
                        min: limits.min.0,
                        max: limits.max.0,
                    }));
                }
            }
        }
    }

    Ok(())
}

fn validate_sequence(
    name: &str,
    seq: &super::WaypointTrajectory,
    config: &SystemConfig,
) -> Result<()> {
    // Motor must exist
    if config.motor(seq.motor.as_str()).is_none() {
        return Err(Error::Trajectory(TrajectoryError::MotorNotFound {
            trajectory: heapless::String::try_from(name).unwrap_or_default(),
            motor: seq.motor.clone(),
        }));
    }

    // Must have at least one waypoint
    if seq.waypoints.is_empty() {
        return Err(Error::Trajectory(TrajectoryError::EmptyWaypoints));
    }

    // Velocity percent must be 1-200
    if seq.velocity_percent == 0 || seq.velocity_percent > 200 {
        return Err(Error::Config(ConfigError::InvalidVelocityPercent(
            seq.velocity_percent,
        )));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_invalid_gear_ratio() {
        use crate::config::units::{DegreesPerSec, DegreesPerSecSquared, Microsteps};
        use crate::config::MotorConfig;

        let config = MotorConfig {
            name: heapless::String::try_from("test").unwrap(),
            steps_per_revolution: 200,
            microsteps: Microsteps::SIXTEENTH,
            gear_ratio: -1.0, // Invalid!
            max_velocity: DegreesPerSec(360.0),
            max_acceleration: DegreesPerSecSquared(720.0),
            invert_direction: false,
            limits: None,
            backlash_compensation: None,
        };

        let result = validate_motor("test", &config);
        assert!(matches!(
            result,
            Err(Error::Config(ConfigError::InvalidGearRatio(_)))
        ));
    }
}
