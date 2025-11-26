//! Error types for stepper-motion library.
//!
//! Provides unified error handling across configuration, motor control, and motion execution.

use core::fmt;

/// Result type alias using the library's Error type.
pub type Result<T> = core::result::Result<T, Error>;

/// Unified error type for all stepper-motion operations.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// Configuration parsing or validation error
    Config(ConfigError),
    /// Motor operation error
    Motor(MotorError),
    /// Motion profile or execution error
    Motion(MotionError),
    /// Trajectory lookup or execution error
    Trajectory(TrajectoryError),
}

/// Configuration-related errors.
#[derive(Debug, Clone, PartialEq)]
pub enum ConfigError {
    /// Failed to parse TOML configuration
    ParseError(heapless::String<128>),
    /// Invalid microstep value (must be power of 2: 1, 2, 4, 8, 16, 32, 64, 128, 256)
    InvalidMicrosteps(u16),
    /// Motor name not found in configuration
    MotorNotFound(heapless::String<32>),
    /// Trajectory name not found in configuration
    TrajectoryNotFound(heapless::String<32>),
    /// Duplicate motor name in configuration
    DuplicateMotorName(heapless::String<32>),
    /// Duplicate trajectory name in configuration
    DuplicateTrajectoryName(heapless::String<32>),
    /// Invalid velocity percent (must be 1-200)
    InvalidVelocityPercent(u8),
    /// Invalid acceleration percent (must be 1-200)
    InvalidAccelerationPercent(u8),
    /// Invalid gear ratio (must be > 0)
    InvalidGearRatio(f32),
    /// Invalid max velocity (must be > 0)
    InvalidMaxVelocity(f32),
    /// Invalid max acceleration (must be > 0)
    InvalidMaxAcceleration(f32),
    /// Invalid soft limits (min must be < max)
    InvalidSoftLimits {
        /// Minimum limit value
        min: f32,
        /// Maximum limit value
        max: f32,
    },
    /// File I/O error (std only)
    #[cfg(feature = "std")]
    IoError(heapless::String<128>),
}

/// Motor operation errors.
#[derive(Debug, Clone, PartialEq)]
pub enum MotorError {
    /// Pin operation failed
    PinError,
    /// Motor is in wrong state for requested operation
    InvalidState(heapless::String<32>),
    /// Motor not initialized
    NotInitialized,
    /// Position exceeds soft limits
    LimitExceeded {
        /// Current or requested position
        position: i64,
        /// Limit that was exceeded (min or max)
        limit: i64,
    },
}

/// Motion profile and execution errors.
#[derive(Debug, Clone, PartialEq)]
pub enum MotionError {
    /// Requested velocity exceeds motor's maximum
    VelocityExceedsLimit {
        /// Requested velocity
        requested: f32,
        /// Maximum allowed velocity
        max: f32,
    },
    /// Requested acceleration exceeds motor's maximum
    AccelerationExceedsLimit {
        /// Requested acceleration
        requested: f32,
        /// Maximum allowed acceleration
        max: f32,
    },
    /// Move distance too short for requested acceleration profile
    MoveTooShort {
        /// Requested move in steps
        steps: i64,
        /// Minimum required steps
        minimum: i64,
    },
    /// Motion profile computation overflow
    Overflow,
}

/// Trajectory-related errors.
#[derive(Debug, Clone, PartialEq)]
pub enum TrajectoryError {
    /// Trajectory references non-existent motor
    MotorNotFound {
        /// Trajectory name
        trajectory: heapless::String<32>,
        /// Referenced motor name
        motor: heapless::String<32>,
    },
    /// Trajectory target exceeds motor limits
    TargetExceedsLimits {
        /// Target position in degrees
        target: f32,
        /// Motor's min limit
        min: f32,
        /// Motor's max limit
        max: f32,
    },
    /// Waypoint list is empty
    EmptyWaypoints,
    /// Too many waypoints
    TooManyWaypoints,
    /// Invalid trajectory name or configuration
    InvalidName(heapless::String<64>),
    /// Empty trajectory (no waypoints or target)
    Empty,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Config(e) => write!(f, "Configuration error: {}", e),
            Error::Motor(e) => write!(f, "Motor error: {}", e),
            Error::Motion(e) => write!(f, "Motion error: {}", e),
            Error::Trajectory(e) => write!(f, "Trajectory error: {}", e),
        }
    }
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::ParseError(msg) => write!(f, "Parse error: {}", msg),
            ConfigError::InvalidMicrosteps(v) => {
                write!(f, "Invalid microsteps: {}. Valid values: 1, 2, 4, 8, 16, 32, 64, 128, 256", v)
            }
            ConfigError::MotorNotFound(name) => write!(f, "Motor '{}' not found", name),
            ConfigError::TrajectoryNotFound(name) => write!(f, "Trajectory '{}' not found", name),
            ConfigError::DuplicateMotorName(name) => write!(f, "Duplicate motor name: '{}'", name),
            ConfigError::DuplicateTrajectoryName(name) => write!(f, "Duplicate trajectory name: '{}'", name),
            ConfigError::InvalidVelocityPercent(v) => write!(f, "Invalid velocity percent: {}. Must be 1-200", v),
            ConfigError::InvalidAccelerationPercent(v) => write!(f, "Invalid acceleration percent: {}. Must be 1-200", v),
            ConfigError::InvalidGearRatio(v) => write!(f, "Invalid gear ratio: {}. Must be > 0", v),
            ConfigError::InvalidMaxVelocity(v) => write!(f, "Invalid max velocity: {}. Must be > 0", v),
            ConfigError::InvalidMaxAcceleration(v) => write!(f, "Invalid max acceleration: {}. Must be > 0", v),
            ConfigError::InvalidSoftLimits { min, max } => {
                write!(f, "Invalid soft limits: min ({}) must be < max ({})", min, max)
            }
            #[cfg(feature = "std")]
            ConfigError::IoError(msg) => write!(f, "I/O error: {}", msg),
        }
    }
}

impl fmt::Display for MotorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MotorError::PinError => write!(f, "GPIO pin operation failed"),
            MotorError::InvalidState(state) => write!(f, "Invalid motor state: {}", state),
            MotorError::NotInitialized => write!(f, "Motor not initialized"),
            MotorError::LimitExceeded { position, limit } => {
                write!(f, "Position {} exceeds limit {}", position, limit)
            }
        }
    }
}

impl fmt::Display for MotionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MotionError::VelocityExceedsLimit { requested, max } => {
                write!(f, "Requested velocity {} exceeds maximum {}", requested, max)
            }
            MotionError::AccelerationExceedsLimit { requested, max } => {
                write!(f, "Requested acceleration {} exceeds maximum {}", requested, max)
            }
            MotionError::MoveTooShort { steps, minimum } => {
                write!(f, "Move of {} steps too short, minimum is {}", steps, minimum)
            }
            MotionError::Overflow => write!(f, "Motion profile computation overflow"),
        }
    }
}

impl fmt::Display for TrajectoryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TrajectoryError::MotorNotFound { trajectory, motor } => {
                write!(f, "Trajectory '{}' references unknown motor '{}'", trajectory, motor)
            }
            TrajectoryError::TargetExceedsLimits { target, min, max } => {
                write!(f, "Target position {} exceeds limits [{}, {}]", target, min, max)
            }
            TrajectoryError::EmptyWaypoints => write!(f, "Waypoint list is empty"),
            TrajectoryError::TooManyWaypoints => {
                write!(f, "Too many waypoints (max 32)")
            }
            TrajectoryError::InvalidName(name) => {
                write!(f, "Invalid trajectory name or configuration: {}", name)
            }
            TrajectoryError::Empty => write!(f, "Trajectory is empty (no waypoints or target)"),
        }
    }
}

// Conversion impls
impl From<ConfigError> for Error {
    fn from(e: ConfigError) -> Self {
        Error::Config(e)
    }
}

impl From<MotorError> for Error {
    fn from(e: MotorError) -> Self {
        Error::Motor(e)
    }
}

impl From<MotionError> for Error {
    fn from(e: MotionError) -> Self {
        Error::Motion(e)
    }
}

impl From<TrajectoryError> for Error {
    fn from(e: TrajectoryError) -> Self {
        Error::Trajectory(e)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

#[cfg(feature = "std")]
impl std::error::Error for ConfigError {}

#[cfg(feature = "std")]
impl std::error::Error for MotorError {}

#[cfg(feature = "std")]
impl std::error::Error for MotionError {}

#[cfg(feature = "std")]
impl std::error::Error for TrajectoryError {}
