//! Example: Homing with physical switches
//!
//! This example demonstrates how to use physical switches for homing
//! a stepper motor. It shows:
//!
//! - Configuring home and limit switches (NO/NC)
//! - Setting up homing configuration
//! - Executing a homing sequence
//!
//! # Hardware Setup
//!
//! Connect switches to GPIO pins with pull-up resistors:
//! - Home switch: Triggers when motor reaches home position
//! - Min limit: Triggers at minimum travel limit (optional)
//! - Max limit: Triggers at maximum travel limit (optional)
//!
//! # Switch Wiring
//!
//! For Normally Open (NO) switches with internal pull-up:
//! - Switch open (not triggered): Pin reads HIGH
//! - Switch closed (triggered): Pin reads LOW
//!
//! For Normally Closed (NC) switches with internal pull-up:
//! - Switch closed (not triggered): Pin reads LOW  
//! - Switch open (triggered): Pin reads HIGH

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};

use stepper_motion::{
    config::units::{Degrees, DegreesPerSec, DegreesPerSecSquared, Microsteps},
    config::{MechanicalConstraints, MotorConfig},
    HomingConfig, HomingDirection, HomingSwitches, SwitchConfig, SwitchPolarity, SwitchesConfig,
};

/// Mock output pin for demonstration
struct MockOutputPin {
    state: bool,
}

impl MockOutputPin {
    fn new() -> Self {
        Self { state: false }
    }
}

impl embedded_hal::digital::ErrorType for MockOutputPin {
    type Error = core::convert::Infallible;
}

impl OutputPin for MockOutputPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        Ok(())
    }
}

/// Mock input pin that simulates a switch
struct MockSwitch {
    triggered: bool,
    trigger_at_step: Option<i64>,
    current_step: i64,
}

impl MockSwitch {
    fn new() -> Self {
        Self {
            triggered: false,
            trigger_at_step: None,
            current_step: 0,
        }
    }

    /// Set the switch to trigger after a certain number of steps
    fn trigger_after_steps(mut self, steps: i64) -> Self {
        self.trigger_at_step = Some(steps);
        self
    }

    /// Simulate a step (for testing)
    fn step(&mut self) {
        self.current_step += 1;
        if let Some(trigger_step) = self.trigger_at_step {
            if self.current_step >= trigger_step {
                self.triggered = true;
            }
        }
    }
}

impl embedded_hal::digital::ErrorType for MockSwitch {
    type Error = core::convert::Infallible;
}

impl InputPin for MockSwitch {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        // NO switch with pull-up: triggered = LOW, not triggered = HIGH
        Ok(!self.triggered)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.triggered)
    }
}

/// Mock delay
struct MockDelay;

impl DelayNs for MockDelay {
    fn delay_ns(&mut self, _ns: u32) {
        // In real hardware, this would actually delay
    }
}

fn main() {
    println!("Stepper Motor Homing Example");
    println!("============================\n");

    // Example 1: Basic homing configuration
    println!("1. Switch Configuration Examples:");
    println!("   - NO (Normally Open): Switch closes circuit when triggered");
    println!("   - NC (Normally Closed): Switch opens circuit when triggered\n");

    // Create switch configurations
    let home_switch_config = SwitchConfig::new(SwitchPolarity::NO);
    let min_limit_config = SwitchConfig::new(SwitchPolarity::NC);
    let max_limit_config = SwitchConfig::new(SwitchPolarity::NC);

    println!("   Home switch: {:?}", home_switch_config);
    println!("   Min limit: {:?}", min_limit_config);
    println!("   Max limit: {:?}", max_limit_config);

    // Example 2: Switches configuration in motor config
    println!("\n2. Motor Configuration with Switches:");

    let switches_config = SwitchesConfig::with_limits(
        SwitchPolarity::NO, // Home switch is NO
        SwitchPolarity::NC, // Min limit is NC
        SwitchPolarity::NC, // Max limit is NC
    );

    println!("   Has home switch: {}", switches_config.has_home_switch());
    println!(
        "   Has limit switches: {}",
        switches_config.has_limit_switches()
    );

    // Example 3: Homing configuration
    println!("\n3. Homing Configuration:");

    let homing_config = HomingConfig::home_switch(HomingDirection::ToMin)
        .with_fast_velocity(DegreesPerSec(90.0))
        .with_slow_velocity(DegreesPerSec(10.0))
        .with_backoff(Degrees(5.0))
        .with_max_travel(Degrees(400.0))
        .with_home_position(Degrees(0.0));

    println!("   Strategy: {:?}", homing_config.strategy);
    println!("   Direction: {:?}", homing_config.direction);
    println!("   Fast velocity: {} deg/s", homing_config.fast_velocity.0);
    println!("   Slow velocity: {} deg/s", homing_config.slow_velocity.0);
    println!("   Backoff: {} deg", homing_config.backoff_distance.0);
    println!("   Max travel: {} deg", homing_config.max_travel.0);

    // Example 4: Create mock hardware for demonstration
    println!("\n4. Simulated Homing Sequence:");

    // In real code, these would be actual GPIO pins
    let mut step_pin = MockOutputPin::new();
    let mut dir_pin = MockOutputPin::new();
    let mut delay = MockDelay;

    // Create mock switches - home switch triggers after 1000 steps
    let mut home_switch = MockSwitch::new().trigger_after_steps(1000);
    let mut min_limit = MockSwitch::new();
    let mut max_limit = MockSwitch::new();

    // Create motor configuration
    let motor_config = MotorConfig {
        name: heapless::String::try_from("x_axis").unwrap(),
        steps_per_revolution: 200,
        microsteps: Microsteps::SIXTEENTH,
        gear_ratio: 1.0,
        max_velocity: DegreesPerSec(360.0),
        max_acceleration: DegreesPerSecSquared(720.0),
        invert_direction: false,
        limits: None,
        backlash_compensation: None,
        switches: Some(switches_config),
        homing: Some(homing_config.clone()),
    };

    let constraints = MechanicalConstraints::from_config(&motor_config);

    println!("   Motor: {}", motor_config.name);
    println!("   Steps per degree: {:.2}", constraints.steps_per_degree);

    // Create switch holder for homing - using explicit type with all three switch types
    let mut switches: HomingSwitches<'_, MockSwitch, MockSwitch, MockSwitch> = HomingSwitches::new(
        Some(&mut home_switch),
        Some(home_switch_config),
        None, // No min limit
        None,
        None, // No max limit
        None,
    );

    println!("\n   Note: In a real application, execute_homing_blocking would");
    println!("   move the motor until the switch is triggered.\n");

    // Show TOML configuration example
    println!("5. TOML Configuration Example:");
    println!(
        r#"
   [[motors]]
   name = "x_axis"
   steps_per_revolution = 200
   microsteps = 16
   gear_ratio = 1.0
   max_velocity_deg_per_sec = 360.0
   max_acceleration_deg_per_sec2 = 720.0
   invert_direction = false

   [motors.switches]
   [motors.switches.home]
   polarity = "NO"
   enabled = true

   [motors.switches.limit_min]
   polarity = "NC"
   enabled = true

   [motors.switches.limit_max]
   polarity = "NC"  
   enabled = true

   [motors.homing]
   strategy = "home_switch"
   direction = "to_min"
   fast_velocity_deg_per_sec = 90.0
   slow_velocity_deg_per_sec = 10.0
   backoff_degrees = 5.0
   max_travel_degrees = 400.0
   home_position_degrees = 0.0
"#
    );

    println!("Homing example complete!");
}
