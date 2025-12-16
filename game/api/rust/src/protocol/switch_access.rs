//! Switch Access Controller
//! 弘益人間 - Gaming for Everyone
//!
//! Switch scanning and access for motor accessibility.

use super::event::{Button, InputEvent};
use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

/// Scan mode for switch access
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScanMode {
    /// Automatic scanning - items highlight automatically
    AutoScan,
    /// Step scanning - switch advances to next item
    StepScan,
    /// Row-column scanning - select row then column
    RowColumn,
    /// Group scanning - select group then item
    GroupScan,
}

/// Switch access configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwitchConfig {
    /// Scanning speed in milliseconds
    pub scan_speed_ms: u32,
    /// Scan mode
    pub scan_mode: ScanMode,
    /// Auto restart at end
    pub auto_restart: bool,
    /// Loops before exit (0 = infinite)
    pub loops_before_exit: u8,
    /// Audio feedback enabled
    pub audio_feedback: bool,
    /// Visual highlight enabled
    pub visual_highlight: bool,
    /// Switch 1 action (select)
    pub switch1_action: SwitchAction,
    /// Switch 2 action (next/back)
    pub switch2_action: Option<SwitchAction>,
    /// Dwell time for auto-select (0 = disabled)
    pub dwell_time_ms: u32,
    /// Acceptance delay to prevent accidental activations
    pub acceptance_delay_ms: u32,
}

impl Default for SwitchConfig {
    fn default() -> Self {
        Self {
            scan_speed_ms: 1000,
            scan_mode: ScanMode::AutoScan,
            auto_restart: true,
            loops_before_exit: 2,
            audio_feedback: true,
            visual_highlight: true,
            switch1_action: SwitchAction::Select,
            switch2_action: Some(SwitchAction::Next),
            dwell_time_ms: 0,
            acceptance_delay_ms: 100,
        }
    }
}

/// Switch action types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SwitchAction {
    /// Select current item
    Select,
    /// Move to next item
    Next,
    /// Move to previous item
    Previous,
    /// Go back/cancel
    Back,
    /// Start/pause scanning
    StartPause,
    /// Exit scanning mode
    Exit,
}

/// Scannable item
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScanItem {
    pub id: String,
    pub label: String,
    pub action: ScanItemAction,
    pub group: Option<String>,
    pub row: Option<u32>,
    pub column: Option<u32>,
}

/// Action to perform when item is selected
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ScanItemAction {
    /// Press a button
    Button(Button),
    /// Navigate to submenu
    Submenu(String),
    /// Execute custom action
    Custom(String),
}

/// Switch access controller
#[derive(Debug)]
pub struct SwitchAccessController {
    enabled: bool,
    config: SwitchConfig,
    items: Vec<ScanItem>,
    current_index: usize,
    current_group: Option<usize>,
    current_row: Option<usize>,
    is_scanning: bool,
    last_scan_time: Option<Instant>,
    loop_count: u8,
    last_switch_time: Option<Instant>,
}

impl SwitchAccessController {
    /// Create a new switch access controller
    pub fn new() -> Self {
        Self {
            enabled: false,
            config: SwitchConfig::default(),
            items: Vec::new(),
            current_index: 0,
            current_group: None,
            current_row: None,
            is_scanning: false,
            last_scan_time: None,
            loop_count: 0,
            last_switch_time: None,
        }
    }

    /// Enable switch access
    pub fn enable(&mut self) {
        self.enabled = true;
        self.is_scanning = false;
        self.current_index = 0;
    }

    /// Disable switch access
    pub fn disable(&mut self) {
        self.enabled = false;
        self.is_scanning = false;
    }

    /// Check if enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Set configuration
    pub fn set_config(&mut self, config: SwitchConfig) {
        self.config = config;
    }

    /// Get configuration
    pub fn config(&self) -> &SwitchConfig {
        &self.config
    }

    /// Set scannable items
    pub fn set_items(&mut self, items: Vec<ScanItem>) {
        self.items = items;
        self.current_index = 0;
    }

    /// Add a scannable item
    pub fn add_item(&mut self, item: ScanItem) {
        self.items.push(item);
    }

    /// Start scanning
    pub fn start_scanning(&mut self) {
        if !self.items.is_empty() {
            self.is_scanning = true;
            self.current_index = 0;
            self.loop_count = 0;
            self.last_scan_time = Some(Instant::now());
        }
    }

    /// Stop scanning
    pub fn stop_scanning(&mut self) {
        self.is_scanning = false;
    }

    /// Get current highlighted item
    pub fn current_item(&self) -> Option<&ScanItem> {
        self.items.get(self.current_index)
    }

    /// Move to next item
    pub fn next(&mut self) {
        if self.items.is_empty() {
            return;
        }

        self.current_index = (self.current_index + 1) % self.items.len();

        // Check for loop completion
        if self.current_index == 0 {
            self.loop_count += 1;
            if self.config.loops_before_exit > 0 && self.loop_count >= self.config.loops_before_exit {
                if !self.config.auto_restart {
                    self.stop_scanning();
                }
            }
        }

        self.last_scan_time = Some(Instant::now());
    }

    /// Move to previous item
    pub fn previous(&mut self) {
        if self.items.is_empty() {
            return;
        }

        self.current_index = if self.current_index == 0 {
            self.items.len() - 1
        } else {
            self.current_index - 1
        };

        self.last_scan_time = Some(Instant::now());
    }

    /// Select current item
    pub fn select(&self) -> Option<&ScanItem> {
        self.current_item()
    }

    /// Process input event
    pub fn process_event(&mut self, event: &InputEvent) -> Option<InputEvent> {
        if !self.enabled {
            return None;
        }

        // Check acceptance delay
        if let Some(last_time) = self.last_switch_time {
            if last_time.elapsed() < Duration::from_millis(self.config.acceptance_delay_ms as u64) {
                return None;
            }
        }

        match event {
            InputEvent::ButtonPressed { button, .. } => {
                // Check if this is a switch button
                if let Some(action) = self.get_action_for_button(button) {
                    self.last_switch_time = Some(Instant::now());
                    return self.handle_action(action);
                }
            }
            _ => {}
        }

        None
    }

    /// Get action for switch button
    fn get_action_for_button(&self, button: &Button) -> Option<SwitchAction> {
        match button {
            Button::Switch(0) | Button::A => Some(self.config.switch1_action),
            Button::Switch(1) | Button::B => self.config.switch2_action,
            _ => None,
        }
    }

    /// Handle switch action
    fn handle_action(&mut self, action: SwitchAction) -> Option<InputEvent> {
        match action {
            SwitchAction::Select => {
                if let Some(item) = self.current_item() {
                    match &item.action {
                        ScanItemAction::Button(button) => {
                            return Some(InputEvent::ButtonPressed {
                                device_id: uuid::Uuid::nil(),
                                button: *button,
                                timestamp: InputEvent::now(),
                            });
                        }
                        ScanItemAction::Submenu(_) | ScanItemAction::Custom(_) => {
                            // Would need to handle these cases
                        }
                    }
                }
            }
            SwitchAction::Next => {
                self.next();
            }
            SwitchAction::Previous => {
                self.previous();
            }
            SwitchAction::Back => {
                // Go back to parent group if in submenu
                self.current_group = None;
                self.current_row = None;
            }
            SwitchAction::StartPause => {
                if self.is_scanning {
                    self.stop_scanning();
                } else {
                    self.start_scanning();
                }
            }
            SwitchAction::Exit => {
                self.stop_scanning();
            }
        }
        None
    }

    /// Update auto-scan (call periodically)
    pub fn update(&mut self) -> bool {
        if !self.enabled || !self.is_scanning || self.items.is_empty() {
            return false;
        }

        if self.config.scan_mode != ScanMode::AutoScan {
            return false;
        }

        if let Some(last_time) = self.last_scan_time {
            if last_time.elapsed() >= Duration::from_millis(self.config.scan_speed_ms as u64) {
                self.next();
                return true;
            }
        }

        false
    }

    /// Get highlighted item index for visual feedback
    pub fn highlighted_index(&self) -> Option<usize> {
        if self.enabled && self.is_scanning {
            Some(self.current_index)
        } else {
            None
        }
    }
}

impl Default for SwitchAccessController {
    fn default() -> Self {
        Self::new()
    }
}

/// Create default game control items
pub fn create_default_game_items() -> Vec<ScanItem> {
    vec![
        ScanItem {
            id: "move_up".to_string(),
            label: "Move Up".to_string(),
            action: ScanItemAction::Button(Button::DPadUp),
            group: Some("movement".to_string()),
            row: Some(0),
            column: Some(1),
        },
        ScanItem {
            id: "move_down".to_string(),
            label: "Move Down".to_string(),
            action: ScanItemAction::Button(Button::DPadDown),
            group: Some("movement".to_string()),
            row: Some(2),
            column: Some(1),
        },
        ScanItem {
            id: "move_left".to_string(),
            label: "Move Left".to_string(),
            action: ScanItemAction::Button(Button::DPadLeft),
            group: Some("movement".to_string()),
            row: Some(1),
            column: Some(0),
        },
        ScanItem {
            id: "move_right".to_string(),
            label: "Move Right".to_string(),
            action: ScanItemAction::Button(Button::DPadRight),
            group: Some("movement".to_string()),
            row: Some(1),
            column: Some(2),
        },
        ScanItem {
            id: "action_a".to_string(),
            label: "Action A".to_string(),
            action: ScanItemAction::Button(Button::A),
            group: Some("actions".to_string()),
            row: Some(0),
            column: Some(0),
        },
        ScanItem {
            id: "action_b".to_string(),
            label: "Action B".to_string(),
            action: ScanItemAction::Button(Button::B),
            group: Some("actions".to_string()),
            row: Some(0),
            column: Some(1),
        },
        ScanItem {
            id: "action_x".to_string(),
            label: "Action X".to_string(),
            action: ScanItemAction::Button(Button::X),
            group: Some("actions".to_string()),
            row: Some(1),
            column: Some(0),
        },
        ScanItem {
            id: "action_y".to_string(),
            label: "Action Y".to_string(),
            action: ScanItemAction::Button(Button::Y),
            group: Some("actions".to_string()),
            row: Some(1),
            column: Some(1),
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_switch_controller() {
        let mut controller = SwitchAccessController::new();
        controller.set_items(create_default_game_items());
        controller.enable();

        assert!(controller.is_enabled());
        assert!(controller.current_item().is_some());
    }

    #[test]
    fn test_navigation() {
        let mut controller = SwitchAccessController::new();
        controller.set_items(create_default_game_items());
        controller.enable();

        let first_id = controller.current_item().unwrap().id.clone();

        controller.next();
        let second_id = controller.current_item().unwrap().id.clone();

        assert_ne!(first_id, second_id);

        controller.previous();
        assert_eq!(controller.current_item().unwrap().id, first_id);
    }

    #[test]
    fn test_auto_scan_wrapping() {
        let mut controller = SwitchAccessController::new();
        controller.set_items(vec![
            ScanItem {
                id: "1".to_string(),
                label: "One".to_string(),
                action: ScanItemAction::Button(Button::A),
                group: None,
                row: None,
                column: None,
            },
            ScanItem {
                id: "2".to_string(),
                label: "Two".to_string(),
                action: ScanItemAction::Button(Button::B),
                group: None,
                row: None,
                column: None,
            },
        ]);
        controller.enable();

        assert_eq!(controller.current_index, 0);
        controller.next();
        assert_eq!(controller.current_index, 1);
        controller.next();
        assert_eq!(controller.current_index, 0); // Wrapped
    }
}
