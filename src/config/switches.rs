//! Physical switch configuration for limit and home switches.
//!
//! This module provides configuration types for physical switches used
//! in stepper motor systems, including home switches and travel limit switches.

use serde::Deserialize;

/// Switch polarity configuration.
///
/// Determines whether the switch is normally open (NO) or normally closed (NC).
/// This affects how the switch state is interpreted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum SwitchPolarity {
    /// Normally Open: switch is active (triggered) when circuit is closed (pin reads low with pull-up).
    #[default]
    NO,
    /// Normally Closed: switch is active (triggered) when circuit is open (pin reads high with pull-up).
    NC,
}

impl SwitchPolarity {
    /// Check if a raw pin state indicates the switch is active/triggered.
    ///
    /// For NO switches with pull-up resistors, active = pin low (false).
    /// For NC switches with pull-up resistors, active = pin high (true).
    #[inline]
    pub fn is_active(&self, pin_state: bool) -> bool {
        match self {
            SwitchPolarity::NO => !pin_state, // NO with pull-up: low = active
            SwitchPolarity::NC => pin_state,  // NC with pull-up: high = active
        }
    }
}

/// Trigger mode for limit switches.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LimitTriggerMode {
    /// Switches are polled during motor stepping.
    /// Simpler but has latency equal to step interval.
    #[default]
    Polling,
    /// Switches trigger hardware interrupts with callbacks.
    /// Lower latency, immediate response.
    Interrupt,
}

/// Configuration for a single switch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub struct SwitchConfig {
    /// Switch polarity (NO or NC).
    #[serde(default)]
    pub polarity: SwitchPolarity,

    /// Whether this switch is enabled/connected.
    #[serde(default = "default_enabled")]
    pub enabled: bool,

    /// Trigger mode (polling or interrupt).
    #[serde(default)]
    pub trigger_mode: LimitTriggerMode,
}

fn default_enabled() -> bool {
    true
}

impl Default for SwitchConfig {
    fn default() -> Self {
        Self {
            polarity: SwitchPolarity::NO,
            enabled: true,
            trigger_mode: LimitTriggerMode::Polling,
        }
    }
}

impl SwitchConfig {
    /// Create a new switch configuration.
    pub fn new(polarity: SwitchPolarity) -> Self {
        Self {
            polarity,
            enabled: true,
            trigger_mode: LimitTriggerMode::Polling,
        }
    }

    /// Create a switch configuration with interrupt mode.
    pub fn with_interrupt(polarity: SwitchPolarity) -> Self {
        Self {
            polarity,
            enabled: true,
            trigger_mode: LimitTriggerMode::Interrupt,
        }
    }

    /// Create a disabled switch configuration.
    pub fn disabled() -> Self {
        Self {
            polarity: SwitchPolarity::NO,
            enabled: false,
            trigger_mode: LimitTriggerMode::Polling,
        }
    }

    /// Set the trigger mode.
    pub fn with_trigger_mode(mut self, mode: LimitTriggerMode) -> Self {
        self.trigger_mode = mode;
        self
    }

    /// Check if this switch uses interrupt mode.
    #[inline]
    pub fn is_interrupt_mode(&self) -> bool {
        self.trigger_mode == LimitTriggerMode::Interrupt
    }

    /// Check if a raw pin state indicates the switch is active/triggered.
    #[inline]
    pub fn is_active(&self, pin_state: bool) -> bool {
        self.enabled && self.polarity.is_active(pin_state)
    }
}

/// Configuration for all switches on a motor axis.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct SwitchesConfig {
    /// Home switch configuration.
    #[serde(default)]
    pub home: Option<SwitchConfig>,

    /// Minimum travel limit switch configuration.
    #[serde(default)]
    pub limit_min: Option<SwitchConfig>,

    /// Maximum travel limit switch configuration.
    #[serde(default)]
    pub limit_max: Option<SwitchConfig>,
}

impl SwitchesConfig {
    /// Create a configuration with only a home switch.
    pub fn home_only(polarity: SwitchPolarity) -> Self {
        Self {
            home: Some(SwitchConfig::new(polarity)),
            limit_min: None,
            limit_max: None,
        }
    }

    /// Create a configuration with home and both limit switches.
    pub fn with_limits(
        home_polarity: SwitchPolarity,
        limit_min_polarity: SwitchPolarity,
        limit_max_polarity: SwitchPolarity,
    ) -> Self {
        Self {
            home: Some(SwitchConfig::new(home_polarity)),
            limit_min: Some(SwitchConfig::new(limit_min_polarity)),
            limit_max: Some(SwitchConfig::new(limit_max_polarity)),
        }
    }

    /// Check if any switches are configured.
    pub fn has_switches(&self) -> bool {
        self.home.as_ref().map(|s| s.enabled).unwrap_or(false)
            || self.limit_min.as_ref().map(|s| s.enabled).unwrap_or(false)
            || self.limit_max.as_ref().map(|s| s.enabled).unwrap_or(false)
    }

    /// Check if a home switch is configured and enabled.
    pub fn has_home_switch(&self) -> bool {
        self.home.as_ref().map(|s| s.enabled).unwrap_or(false)
    }

    /// Check if limit switches are configured and enabled.
    pub fn has_limit_switches(&self) -> bool {
        self.limit_min.as_ref().map(|s| s.enabled).unwrap_or(false)
            || self.limit_max.as_ref().map(|s| s.enabled).unwrap_or(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_switch_polarity() {
        let polarity = SwitchPolarity::NO;
        // NO switch with pull-up: low (false) = active, high (true) = inactive
        assert!(polarity.is_active(false));
        assert!(!polarity.is_active(true));
    }

    #[test]
    fn test_nc_switch_polarity() {
        let polarity = SwitchPolarity::NC;
        // NC switch with pull-up: high (true) = active, low (false) = inactive
        assert!(polarity.is_active(true));
        assert!(!polarity.is_active(false));
    }

    #[test]
    fn test_switch_config() {
        let config = SwitchConfig::new(SwitchPolarity::NO);
        assert!(config.enabled);
        assert!(config.is_active(false));
        assert!(!config.is_active(true));
    }

    #[test]
    fn test_disabled_switch() {
        let config = SwitchConfig::disabled();
        assert!(!config.enabled);
        // Disabled switch should never report active
        assert!(!config.is_active(false));
        assert!(!config.is_active(true));
    }

    #[test]
    fn test_switches_config() {
        let config = SwitchesConfig::home_only(SwitchPolarity::NO);
        assert!(config.has_switches());
        assert!(config.has_home_switch());
        assert!(!config.has_limit_switches());

        let config =
            SwitchesConfig::with_limits(SwitchPolarity::NO, SwitchPolarity::NC, SwitchPolarity::NC);
        assert!(config.has_switches());
        assert!(config.has_home_switch());
        assert!(config.has_limit_switches());
    }
}
