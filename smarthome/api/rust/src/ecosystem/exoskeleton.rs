//! # Exoskeleton Smart Home Integration
//!
//! Enables mobility state-aware automation for exoskeleton users.

use super::*;
use crate::error::{Result, SmartHomeError};
use crate::types::DeviceId;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Mobility state of exoskeleton user
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MobilityState {
    /// User is seated (in wheelchair or chair)
    Seated,
    /// User is standing with exoskeleton support
    Standing,
    /// User is walking with exoskeleton
    Walking,
    /// User is using stairs
    Stairs,
    /// User is transitioning between states
    Transitioning,
    /// Exoskeleton is in rest/charging mode
    Rest,
    /// Emergency state (fall detected, etc.)
    Emergency,
}

impl Default for MobilityState {
    fn default() -> Self {
        Self::Seated
    }
}

/// Exoskeleton state data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExoskeletonState {
    pub mobility_state: MobilityState,
    pub battery_level: u8,
    pub joint_angles: JointAngles,
    pub balance_score: f32,
    pub steps_taken: u32,
    pub active_time_minutes: u32,
    pub alerts: Vec<ExoskeletonAlert>,
}

impl Default for ExoskeletonState {
    fn default() -> Self {
        Self {
            mobility_state: MobilityState::Seated,
            battery_level: 100,
            joint_angles: JointAngles::default(),
            balance_score: 1.0,
            steps_taken: 0,
            active_time_minutes: 0,
            alerts: Vec::new(),
        }
    }
}

/// Joint angles for exoskeleton
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct JointAngles {
    pub left_hip: f32,
    pub left_knee: f32,
    pub left_ankle: f32,
    pub right_hip: f32,
    pub right_knee: f32,
    pub right_ankle: f32,
}

/// Exoskeleton alert types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ExoskeletonAlert {
    LowBattery { level: u8 },
    BalanceWarning { score: f32 },
    FallDetected,
    ObstacleDetected { distance_m: f32 },
    OverheatWarning,
    MaintenanceRequired,
}

/// Mobility state automation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MobilityAutomation {
    pub from_state: MobilityState,
    pub to_state: MobilityState,
    pub actions: Vec<MobilityAction>,
    pub enabled: bool,
}

/// Mobility-triggered action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MobilityAction {
    pub device_id: DeviceId,
    pub action: DeviceAction,
    pub priority: CommandPriority,
    pub description: String,
}

/// Balance assist configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BalanceAssistConfig {
    /// Enable stabilization lighting
    pub stabilization_lighting: bool,
    /// Highlight handrail locations
    pub handrail_indicators: bool,
    /// Enable floor obstacle alerts
    pub floor_obstacle_alert: bool,
    /// Balance warning threshold (0.0-1.0)
    pub balance_warning_threshold: f32,
    /// Emergency grab points to highlight
    pub emergency_grab_points: Vec<String>,
}

impl Default for BalanceAssistConfig {
    fn default() -> Self {
        Self {
            stabilization_lighting: true,
            handrail_indicators: true,
            floor_obstacle_alert: true,
            balance_warning_threshold: 0.7,
            emergency_grab_points: Vec::new(),
        }
    }
}

/// Exoskeleton adapter for smart home
#[derive(Debug)]
pub struct ExoskeletonAdapter {
    config: ExoskeletonConfig,
    automations: Vec<MobilityAutomation>,
    balance_config: BalanceAssistConfig,
    current_state: ExoskeletonState,
    state_devices: HashMap<MobilityState, Vec<MobilityAction>>,
}

/// Exoskeleton adapter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExoskeletonConfig {
    /// Enable mobility state automation
    pub state_automation_enabled: bool,
    /// Enable balance assist features
    pub balance_assist_enabled: bool,
    /// Enable emergency automation
    pub emergency_automation_enabled: bool,
    /// Transition delay before triggering (ms)
    pub transition_delay_ms: u32,
    /// Emergency response timeout (ms)
    pub emergency_timeout_ms: u32,
}

impl Default for ExoskeletonConfig {
    fn default() -> Self {
        Self {
            state_automation_enabled: true,
            balance_assist_enabled: true,
            emergency_automation_enabled: true,
            transition_delay_ms: 500,
            emergency_timeout_ms: 5000,
        }
    }
}

impl ExoskeletonAdapter {
    pub fn new() -> Self {
        Self {
            config: ExoskeletonConfig::default(),
            automations: Vec::new(),
            balance_config: BalanceAssistConfig::default(),
            current_state: ExoskeletonState::default(),
            state_devices: HashMap::new(),
        }
    }

    pub fn with_config(mut self, config: ExoskeletonConfig) -> Self {
        self.config = config;
        self
    }

    pub fn with_balance_config(mut self, config: BalanceAssistConfig) -> Self {
        self.balance_config = config;
        self
    }

    /// Add mobility automation
    pub fn add_automation(&mut self, automation: MobilityAutomation) {
        self.automations.push(automation);
    }

    /// Set devices to control for a mobility state
    pub fn set_state_devices(&mut self, state: MobilityState, actions: Vec<MobilityAction>) {
        self.state_devices.insert(state, actions);
    }

    /// Update exoskeleton state and get triggered actions
    pub fn update_state(&mut self, new_state: ExoskeletonState) -> Vec<ExoskeletonTriggeredAction> {
        let mut actions = Vec::new();

        // Check state transition
        if self.config.state_automation_enabled {
            if new_state.mobility_state != self.current_state.mobility_state {
                if let Some(transition_actions) = self.handle_state_transition(
                    self.current_state.mobility_state,
                    new_state.mobility_state,
                ) {
                    actions.extend(transition_actions);
                }
            }
        }

        // Check balance warnings
        if self.config.balance_assist_enabled {
            if new_state.balance_score < self.balance_config.balance_warning_threshold {
                if let Some(balance_actions) = self.handle_balance_warning(new_state.balance_score) {
                    actions.extend(balance_actions);
                }
            }
        }

        // Check emergency state
        if self.config.emergency_automation_enabled {
            if new_state.mobility_state == MobilityState::Emergency {
                if let Some(emergency_actions) = self.handle_emergency(&new_state) {
                    actions.extend(emergency_actions);
                }
            }
        }

        // Check alerts
        for alert in &new_state.alerts {
            if let Some(alert_actions) = self.handle_alert(alert) {
                actions.extend(alert_actions);
            }
        }

        self.current_state = new_state;

        actions
    }

    fn handle_state_transition(
        &self,
        from: MobilityState,
        to: MobilityState,
    ) -> Option<Vec<ExoskeletonTriggeredAction>> {
        let mut actions = Vec::new();

        // Find matching automation
        for automation in &self.automations {
            if automation.enabled && automation.from_state == from && automation.to_state == to {
                for action in &automation.actions {
                    actions.push(ExoskeletonTriggeredAction {
                        trigger: ExoskeletonTrigger::StateChange { from, to },
                        device_id: action.device_id,
                        action: action.action.clone(),
                        priority: action.priority,
                        description: action.description.clone(),
                    });
                }
            }
        }

        // Also apply state-specific devices
        if let Some(state_actions) = self.state_devices.get(&to) {
            for action in state_actions {
                actions.push(ExoskeletonTriggeredAction {
                    trigger: ExoskeletonTrigger::StateEnter(to),
                    device_id: action.device_id,
                    action: action.action.clone(),
                    priority: action.priority,
                    description: action.description.clone(),
                });
            }
        }

        if actions.is_empty() {
            None
        } else {
            Some(actions)
        }
    }

    fn handle_balance_warning(&self, score: f32) -> Option<Vec<ExoskeletonTriggeredAction>> {
        if !self.balance_config.stabilization_lighting {
            return None;
        }

        // Would typically trigger brighter lighting and handrail indicators
        Some(vec![ExoskeletonTriggeredAction {
            trigger: ExoskeletonTrigger::BalanceWarning(score),
            device_id: Uuid::nil(), // Placeholder - would be actual light device
            action: DeviceAction::Brightness { level: 100 },
            priority: CommandPriority::Safety,
            description: "Increase lighting for balance support".to_string(),
        }])
    }

    fn handle_emergency(&self, _state: &ExoskeletonState) -> Option<Vec<ExoskeletonTriggeredAction>> {
        Some(vec![
            ExoskeletonTriggeredAction {
                trigger: ExoskeletonTrigger::Emergency,
                device_id: Uuid::nil(), // All lights
                action: DeviceAction::Power { on: true },
                priority: CommandPriority::Emergency,
                description: "Turn on all lights".to_string(),
            },
            ExoskeletonTriggeredAction {
                trigger: ExoskeletonTrigger::Emergency,
                device_id: Uuid::nil(), // All door locks
                action: DeviceAction::Lock { locked: false },
                priority: CommandPriority::Emergency,
                description: "Unlock all doors for emergency access".to_string(),
            },
        ])
    }

    fn handle_alert(&self, alert: &ExoskeletonAlert) -> Option<Vec<ExoskeletonTriggeredAction>> {
        match alert {
            ExoskeletonAlert::FallDetected => self.handle_emergency(&self.current_state),
            ExoskeletonAlert::ObstacleDetected { distance_m } if *distance_m < 1.0 => {
                Some(vec![ExoskeletonTriggeredAction {
                    trigger: ExoskeletonTrigger::Alert(format!("Obstacle at {}m", distance_m)),
                    device_id: Uuid::nil(),
                    action: DeviceAction::Custom {
                        name: "obstacle_warning".to_string(),
                        params: serde_json::json!({ "distance": distance_m }),
                    },
                    priority: CommandPriority::Safety,
                    description: "Obstacle warning".to_string(),
                }])
            }
            _ => None,
        }
    }

    /// Convert triggered action to unified command
    pub fn action_to_command(&self, action: ExoskeletonTriggeredAction) -> UnifiedCommand {
        UnifiedCommand::new(
            CommandSource::Exoskeleton {
                state: self.current_state.mobility_state,
                previous_state: None,
            },
            CommandTarget::Device(action.device_id),
            action.action,
        )
        .with_priority(action.priority)
    }

    /// Get current state
    pub fn get_state(&self) -> &ExoskeletonState {
        &self.current_state
    }

    /// Setup standing automation (example)
    pub fn setup_standing_automation(&mut self, light_id: DeviceId, counter_height_device: Option<DeviceId>) {
        let mut actions = vec![MobilityAction {
            device_id: light_id,
            action: DeviceAction::Brightness { level: 100 },
            priority: CommandPriority::Normal,
            description: "Brighten lights for standing".to_string(),
        }];

        if let Some(counter_id) = counter_height_device {
            actions.push(MobilityAction {
                device_id: counter_id,
                action: DeviceAction::Custom {
                    name: "adjust_height".to_string(),
                    params: serde_json::json!({ "mode": "standing" }),
                },
                priority: CommandPriority::Normal,
                description: "Adjust counter to standing height".to_string(),
            });
        }

        self.add_automation(MobilityAutomation {
            from_state: MobilityState::Seated,
            to_state: MobilityState::Standing,
            actions,
            enabled: true,
        });
    }

    /// Setup walking automation (example)
    pub fn setup_walking_automation(&mut self, path_light_ids: Vec<DeviceId>) {
        let actions = path_light_ids
            .into_iter()
            .map(|light_id| MobilityAction {
                device_id: light_id,
                action: DeviceAction::Power { on: true },
                priority: CommandPriority::Normal,
                description: "Enable path lighting".to_string(),
            })
            .collect();

        self.add_automation(MobilityAutomation {
            from_state: MobilityState::Standing,
            to_state: MobilityState::Walking,
            actions,
            enabled: true,
        });
    }
}

impl Default for ExoskeletonAdapter {
    fn default() -> Self {
        Self::new()
    }
}

/// Exoskeleton trigger types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ExoskeletonTrigger {
    StateChange {
        from: MobilityState,
        to: MobilityState,
    },
    StateEnter(MobilityState),
    BalanceWarning(f32),
    Emergency,
    Alert(String),
}

/// Triggered action from exoskeleton
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExoskeletonTriggeredAction {
    pub trigger: ExoskeletonTrigger,
    pub device_id: DeviceId,
    pub action: DeviceAction,
    pub priority: CommandPriority,
    pub description: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_exoskeleton_adapter_creation() {
        let adapter = ExoskeletonAdapter::new();
        assert!(adapter.config.state_automation_enabled);
    }

    #[test]
    fn test_state_transition() {
        let mut adapter = ExoskeletonAdapter::new();

        let light_id = Uuid::new_v4();

        adapter.add_automation(MobilityAutomation {
            from_state: MobilityState::Seated,
            to_state: MobilityState::Standing,
            actions: vec![MobilityAction {
                device_id: light_id,
                action: DeviceAction::Brightness { level: 100 },
                priority: CommandPriority::Normal,
                description: "Brighten".to_string(),
            }],
            enabled: true,
        });

        // Transition from seated to standing
        let new_state = ExoskeletonState {
            mobility_state: MobilityState::Standing,
            ..Default::default()
        };

        let actions = adapter.update_state(new_state);
        assert!(!actions.is_empty());
    }

    #[test]
    fn test_emergency_handling() {
        let mut adapter = ExoskeletonAdapter::new();

        let new_state = ExoskeletonState {
            mobility_state: MobilityState::Emergency,
            ..Default::default()
        };

        let actions = adapter.update_state(new_state);
        assert!(!actions.is_empty());

        // Should have emergency priority
        assert!(actions
            .iter()
            .all(|a| a.priority == CommandPriority::Emergency));
    }

    #[test]
    fn test_balance_warning() {
        let mut adapter = ExoskeletonAdapter::new();
        adapter.balance_config.balance_warning_threshold = 0.7;

        let new_state = ExoskeletonState {
            mobility_state: MobilityState::Walking,
            balance_score: 0.5, // Below threshold
            ..Default::default()
        };

        let actions = adapter.update_state(new_state);
        assert!(!actions.is_empty());
    }
}
