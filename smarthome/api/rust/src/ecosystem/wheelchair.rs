//! # Smart Wheelchair Smart Home Integration
//!
//! Enables location-based automation triggered by wheelchair position.

use super::*;
use crate::error::{Result, SmartHomeError};
use crate::types::{DeviceId, ZoneId};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Wheelchair trigger types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum WheelchairTrigger {
    /// Entered a zone
    EnterZone,
    /// Exited a zone
    ExitZone,
    /// Approaching a device/door
    Proximity,
    /// Stopped in zone
    Dwell,
    /// Emergency (tilt, stuck, etc.)
    Emergency,
}

/// Wheelchair state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelchairState {
    pub position: Position2D,
    pub current_zone: Option<ZoneId>,
    pub velocity: f32,
    pub heading: f32,
    pub battery_level: u8,
    pub is_moving: bool,
    pub navigation_target: Option<ZoneId>,
}

impl Default for WheelchairState {
    fn default() -> Self {
        Self {
            position: Position2D { x: 0.0, y: 0.0 },
            current_zone: None,
            velocity: 0.0,
            heading: 0.0,
            battery_level: 100,
            is_moving: false,
            navigation_target: None,
        }
    }
}

/// Zone automation rule
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ZoneAutomation {
    pub zone_id: ZoneId,
    pub zone_name: String,
    pub enter_actions: Vec<AutomationAction>,
    pub exit_actions: Vec<AutomationAction>,
    pub dwell_actions: Vec<DwellAction>,
    pub enabled: bool,
}

/// Automation action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutomationAction {
    pub device_id: DeviceId,
    pub action: DeviceAction,
    pub delay_ms: u32,
    pub condition: Option<AutomationCondition>,
}

/// Dwell action (triggered after staying in zone)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DwellAction {
    pub device_id: DeviceId,
    pub action: DeviceAction,
    pub dwell_time_ms: u32,
}

/// Automation condition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AutomationCondition {
    TimeRange { start_hour: u8, end_hour: u8 },
    DayOfWeek { days: Vec<u8> },
    DeviceState { device_id: DeviceId, expected: String },
    Custom { condition: String },
}

/// Proximity trigger
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProximityTrigger {
    pub id: String,
    pub device_id: DeviceId,
    pub position: Position2D,
    pub trigger_distance_m: f32,
    pub action: DeviceAction,
    pub cooldown_ms: u32,
    pub last_triggered_ms: u64,
}

/// Smart wheelchair adapter for smart home
#[derive(Debug)]
pub struct WheelchairAdapter {
    config: WheelchairConfig,
    zone_automations: HashMap<ZoneId, ZoneAutomation>,
    proximity_triggers: Vec<ProximityTrigger>,
    current_state: WheelchairState,
    previous_zone: Option<ZoneId>,
    zone_enter_time_ms: u64,
}

/// Wheelchair adapter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelchairConfig {
    /// Enable zone automation
    pub zone_automation_enabled: bool,
    /// Enable proximity triggers
    pub proximity_enabled: bool,
    /// Enable path lighting
    pub path_lighting_enabled: bool,
    /// Exit delay (ms) before triggering exit actions
    pub exit_delay_ms: u32,
    /// Minimum dwell time for dwell actions
    pub min_dwell_ms: u32,
    /// Enable emergency detection
    pub emergency_detection_enabled: bool,
}

impl Default for WheelchairConfig {
    fn default() -> Self {
        Self {
            zone_automation_enabled: true,
            proximity_enabled: true,
            path_lighting_enabled: true,
            exit_delay_ms: 5000,
            min_dwell_ms: 3000,
            emergency_detection_enabled: true,
        }
    }
}

impl WheelchairAdapter {
    pub fn new() -> Self {
        Self {
            config: WheelchairConfig::default(),
            zone_automations: HashMap::new(),
            proximity_triggers: Vec::new(),
            current_state: WheelchairState::default(),
            previous_zone: None,
            zone_enter_time_ms: 0,
        }
    }

    pub fn with_config(mut self, config: WheelchairConfig) -> Self {
        self.config = config;
        self
    }

    /// Add zone automation
    pub fn add_zone_automation(&mut self, automation: ZoneAutomation) {
        self.zone_automations
            .insert(automation.zone_id, automation);
    }

    /// Add proximity trigger
    pub fn add_proximity_trigger(&mut self, trigger: ProximityTrigger) {
        self.proximity_triggers.push(trigger);
    }

    /// Update wheelchair state and get triggered actions
    pub fn update_state(
        &mut self,
        new_state: WheelchairState,
        current_time_ms: u64,
    ) -> Vec<TriggeredAction> {
        let mut actions = Vec::new();

        // Check zone changes
        if self.config.zone_automation_enabled {
            if let Some(zone_actions) =
                self.check_zone_change(&new_state, current_time_ms)
            {
                actions.extend(zone_actions);
            }

            // Check dwell actions
            if let Some(dwell_actions) = self.check_dwell_actions(current_time_ms) {
                actions.extend(dwell_actions);
            }
        }

        // Check proximity triggers
        if self.config.proximity_enabled {
            if let Some(proximity_actions) =
                self.check_proximity_triggers(&new_state, current_time_ms)
            {
                actions.extend(proximity_actions);
            }
        }

        // Update state
        self.current_state = new_state;

        actions
    }

    fn check_zone_change(
        &mut self,
        new_state: &WheelchairState,
        current_time_ms: u64,
    ) -> Option<Vec<TriggeredAction>> {
        let mut actions = Vec::new();

        // Zone changed?
        if new_state.current_zone != self.current_state.current_zone {
            // Exit previous zone
            if let Some(prev_zone) = &self.current_state.current_zone {
                if let Some(automation) = self.zone_automations.get(prev_zone) {
                    if automation.enabled {
                        for exit_action in &automation.exit_actions {
                            actions.push(TriggeredAction {
                                trigger: WheelchairTrigger::ExitZone,
                                zone_id: Some(*prev_zone),
                                device_id: exit_action.device_id,
                                action: exit_action.action.clone(),
                                delay_ms: exit_action.delay_ms,
                            });
                        }
                    }
                }
            }

            // Enter new zone
            if let Some(new_zone) = &new_state.current_zone {
                self.zone_enter_time_ms = current_time_ms;

                if let Some(automation) = self.zone_automations.get(new_zone) {
                    if automation.enabled {
                        for enter_action in &automation.enter_actions {
                            actions.push(TriggeredAction {
                                trigger: WheelchairTrigger::EnterZone,
                                zone_id: Some(*new_zone),
                                device_id: enter_action.device_id,
                                action: enter_action.action.clone(),
                                delay_ms: enter_action.delay_ms,
                            });
                        }
                    }
                }
            }

            self.previous_zone = self.current_state.current_zone;
        }

        if actions.is_empty() {
            None
        } else {
            Some(actions)
        }
    }

    fn check_dwell_actions(&self, current_time_ms: u64) -> Option<Vec<TriggeredAction>> {
        let mut actions = Vec::new();

        if let Some(zone_id) = &self.current_state.current_zone {
            if let Some(automation) = self.zone_automations.get(zone_id) {
                let dwell_time = current_time_ms.saturating_sub(self.zone_enter_time_ms);

                for dwell_action in &automation.dwell_actions {
                    if dwell_time >= dwell_action.dwell_time_ms as u64 {
                        actions.push(TriggeredAction {
                            trigger: WheelchairTrigger::Dwell,
                            zone_id: Some(*zone_id),
                            device_id: dwell_action.device_id,
                            action: dwell_action.action.clone(),
                            delay_ms: 0,
                        });
                    }
                }
            }
        }

        if actions.is_empty() {
            None
        } else {
            Some(actions)
        }
    }

    fn check_proximity_triggers(
        &mut self,
        new_state: &WheelchairState,
        current_time_ms: u64,
    ) -> Option<Vec<TriggeredAction>> {
        let mut actions = Vec::new();

        for trigger in &mut self.proximity_triggers {
            // Check cooldown
            if current_time_ms.saturating_sub(trigger.last_triggered_ms) < trigger.cooldown_ms as u64
            {
                continue;
            }

            // Calculate distance
            let dx = new_state.position.x - trigger.position.x;
            let dy = new_state.position.y - trigger.position.y;
            let distance = (dx * dx + dy * dy).sqrt();

            if distance <= trigger.trigger_distance_m {
                actions.push(TriggeredAction {
                    trigger: WheelchairTrigger::Proximity,
                    zone_id: None,
                    device_id: trigger.device_id,
                    action: trigger.action.clone(),
                    delay_ms: 0,
                });
                trigger.last_triggered_ms = current_time_ms;
            }
        }

        if actions.is_empty() {
            None
        } else {
            Some(actions)
        }
    }

    /// Convert triggered action to unified command
    pub fn action_to_command(&self, action: TriggeredAction) -> UnifiedCommand {
        UnifiedCommand::new(
            CommandSource::Wheelchair {
                zone: action.zone_id.unwrap_or_default(),
                trigger: action.trigger,
                position: Some(self.current_state.position),
            },
            CommandTarget::Device(action.device_id),
            action.action,
        )
        .with_priority(CommandPriority::Automation)
    }

    /// Get current state
    pub fn get_state(&self) -> &WheelchairState {
        &self.current_state
    }

    /// Create path lighting sequence
    pub fn create_path_lighting(
        &self,
        path_zones: &[ZoneId],
        light_devices: &HashMap<ZoneId, DeviceId>,
    ) -> Vec<TriggeredAction> {
        path_zones
            .iter()
            .filter_map(|zone_id| {
                light_devices.get(zone_id).map(|device_id| TriggeredAction {
                    trigger: WheelchairTrigger::EnterZone,
                    zone_id: Some(*zone_id),
                    device_id: *device_id,
                    action: DeviceAction::Power { on: true },
                    delay_ms: 0,
                })
            })
            .collect()
    }

    /// Setup common automation (living room example)
    pub fn setup_living_room_automation(
        &mut self,
        zone_id: ZoneId,
        light_id: DeviceId,
        ac_id: Option<DeviceId>,
    ) {
        let mut enter_actions = vec![AutomationAction {
            device_id: light_id,
            action: DeviceAction::Power { on: true },
            delay_ms: 0,
            condition: None,
        }];

        if let Some(ac) = ac_id {
            enter_actions.push(AutomationAction {
                device_id: ac,
                action: DeviceAction::Power { on: true },
                delay_ms: 1000,
                condition: None,
            });
        }

        let exit_actions = vec![AutomationAction {
            device_id: light_id,
            action: DeviceAction::Power { on: false },
            delay_ms: self.config.exit_delay_ms,
            condition: None,
        }];

        self.add_zone_automation(ZoneAutomation {
            zone_id,
            zone_name: "Living Room".to_string(),
            enter_actions,
            exit_actions,
            dwell_actions: vec![],
            enabled: true,
        });
    }
}

impl Default for WheelchairAdapter {
    fn default() -> Self {
        Self::new()
    }
}

/// Triggered automation action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TriggeredAction {
    pub trigger: WheelchairTrigger,
    pub zone_id: Option<ZoneId>,
    pub device_id: DeviceId,
    pub action: DeviceAction,
    pub delay_ms: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wheelchair_adapter_creation() {
        let adapter = WheelchairAdapter::new();
        assert!(adapter.config.zone_automation_enabled);
    }

    #[test]
    fn test_zone_enter_trigger() {
        let mut adapter = WheelchairAdapter::new();

        let zone_id = Uuid::new_v4();
        let light_id = Uuid::new_v4();

        adapter.add_zone_automation(ZoneAutomation {
            zone_id,
            zone_name: "Living Room".to_string(),
            enter_actions: vec![AutomationAction {
                device_id: light_id,
                action: DeviceAction::Power { on: true },
                delay_ms: 0,
                condition: None,
            }],
            exit_actions: vec![],
            dwell_actions: vec![],
            enabled: true,
        });

        // Update state to enter zone
        let new_state = WheelchairState {
            current_zone: Some(zone_id),
            ..Default::default()
        };

        let actions = adapter.update_state(new_state, 0);
        assert_eq!(actions.len(), 1);
        assert_eq!(actions[0].device_id, light_id);
        assert!(matches!(actions[0].trigger, WheelchairTrigger::EnterZone));
    }

    #[test]
    fn test_zone_exit_trigger() {
        let mut adapter = WheelchairAdapter::new();

        let zone_id = Uuid::new_v4();
        let light_id = Uuid::new_v4();

        adapter.add_zone_automation(ZoneAutomation {
            zone_id,
            zone_name: "Living Room".to_string(),
            enter_actions: vec![],
            exit_actions: vec![AutomationAction {
                device_id: light_id,
                action: DeviceAction::Power { on: false },
                delay_ms: 5000,
                condition: None,
            }],
            dwell_actions: vec![],
            enabled: true,
        });

        // First enter the zone
        adapter.current_state.current_zone = Some(zone_id);

        // Then exit
        let new_state = WheelchairState {
            current_zone: None,
            ..Default::default()
        };

        let actions = adapter.update_state(new_state, 1000);
        assert_eq!(actions.len(), 1);
        assert!(matches!(actions[0].trigger, WheelchairTrigger::ExitZone));
        assert_eq!(actions[0].delay_ms, 5000);
    }

    #[test]
    fn test_proximity_trigger() {
        let mut adapter = WheelchairAdapter::new();

        let door_id = Uuid::new_v4();

        adapter.add_proximity_trigger(ProximityTrigger {
            id: "front_door".to_string(),
            device_id: door_id,
            position: Position2D { x: 5.0, y: 0.0 },
            trigger_distance_m: 2.0,
            action: DeviceAction::Door { open: true },
            cooldown_ms: 10000,
            last_triggered_ms: 0,
        });

        // Move close to door (time must be > cooldown_ms since last_triggered)
        let new_state = WheelchairState {
            position: Position2D { x: 4.0, y: 0.0 },
            ..Default::default()
        };

        let actions = adapter.update_state(new_state, 15000); // Past cooldown period
        assert_eq!(actions.len(), 1);
        assert!(matches!(actions[0].trigger, WheelchairTrigger::Proximity));
    }
}
